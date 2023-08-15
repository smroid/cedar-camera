use canonical_error::{CanonicalError, failed_precondition_error};

use std::ffi::CStr;
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, SystemTime};

use image::GrayImage;
use log::{error, info, warn};

use asi_camera2::asi_camera2_sdk;
use crate::abstract_camera::{AbstractCamera, BinFactor, CaptureParams, CapturedImage,
                             Celsius, Flip, Gain, Offset, RegionOfInterest};

// State shared between video capture thread and the ASICamera methods.
struct SharedState {
    // Unchanging camera info.
    info: asi_camera2_sdk::ASI_CAMERA_INFO,
    // Gain values in SDK units.
    default_gain: i32,
    min_gain: i32,
    max_gain: i32,

    // Current camera settings as set via ASICamera methods. Will be put into
    // effect when the current exposure finishes, influencing the following
    // exposure.
    camera_settings: CaptureParams,
    setting_changed: bool,

    // Most recent completed capture and its id value.
    most_recent_capture: Option<Arc<CapturedImage>>,
    frame_id: i32,

    // Set by stop(); the video capture thread exits when it sees this.
    stop_request: bool,

    // Camera settings in effect when the in-progress capture started.
    current_capture_settings: CaptureParams,

    // Our video capture thread. Executes video_capture_thread_worker().
    video_capture_thread: Option<thread::JoinHandle<()>>,
}

pub struct ASICamera {
    // The SDK wrapper object. After initialization, the video capture thread is
    // the only thing that touches asi_cam_sdk.
    asi_cam_sdk: Arc<Mutex<asi_camera2_sdk::ASICamera>>,

    // Our state, shared between ASICamera methods and the video capture thread.
    state: Arc<Mutex<SharedState>>,

    // Condition variable signalled whenever `state.most_recent_capture` is
    // populated; also signalled when the worker thread exits.
    capture_done: Arc<Condvar>,
}

impl ASICamera {
    /// Returns an ASICamera instance that implements the AbstractCamera API.
    pub fn new(mut asi_cam_sdk: asi_camera2_sdk::ASICamera)
               -> Result<Self, CanonicalError> {
        let info = match asi_cam_sdk.get_property() {
            Ok(prop) => prop,
            Err(e) => return Err(failed_precondition_error(&e.to_string()))
        };
        // Find the camera's min/max gain values.
        match asi_cam_sdk.open() {
            Ok(()) => (),
            Err(e) => return Err(failed_precondition_error(&e.to_string()))
        }
        match asi_cam_sdk.init() {
            Ok(()) => (),
            Err(e) => return Err(failed_precondition_error(&e.to_string()))
        }
        let mut got_gain = false;
        let mut default_gain = 0;
        let mut min_gain = 0;
        let mut max_gain = 0;
        let num_controls = asi_cam_sdk.get_num_controls().unwrap();
        for control_index in 0..num_controls {
            let control_caps = asi_cam_sdk.get_control_caps(control_index).unwrap();
            if control_caps.ControlType == asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_GAIN {
                default_gain = control_caps.DefaultValue as i32;
                min_gain = control_caps.MinValue as i32;
                max_gain = control_caps.MaxValue as i32;
                got_gain = true;
                break;
            }
        }
        if !got_gain {
            return Err(failed_precondition_error(
                "Could not find control caps for ASI_CONTROL_TYPE_ASI_GAIN"));
        }
        let mut cam = ASICamera{
            asi_cam_sdk: Arc::new(Mutex::new(asi_cam_sdk)),
            state: Arc::new(Mutex::new(SharedState{
                info, default_gain, min_gain, max_gain,
                camera_settings: CaptureParams::new(),
                setting_changed: false,
                most_recent_capture: None,
                frame_id: 0,
                stop_request: false,
                current_capture_settings: CaptureParams::new(),
                video_capture_thread: None,
            })),
            capture_done: Arc::new(Condvar::new()),
        };
        cam.set_gain(cam.optimal_gain())?;
        let mut roi = cam.get_region_of_interest();
        roi.capture_dimensions = cam.dimensions();
        cam.set_region_of_interest(roi)?;
        info!("Created ASICamera API object");
        Ok(cam)
    }  // new().

    // This function is called whenever a camera setting is changed. The most
    // recently captured image is invalidated, and the next call to capture_image()
    // will wait for the setting change to be reflected in the captured image
    // stream.
    fn changed_setting(locked_state: &mut MutexGuard<SharedState>) {
        locked_state.setting_changed = true;
        locked_state.most_recent_capture = None;
    }

    // The first call to capture_image() starts the video capture thread that
    // executes this function.
    fn video_capture_thread_worker(state: Arc<Mutex<SharedState>>,
                                   capture_done: Arc<Condvar>,
                                   asi_cam_sdk:
                                   Arc<Mutex<asi_camera2_sdk::ASICamera>>) {
        let mut sdk = asi_cam_sdk.lock().unwrap();
        let mut starting = true;
        // Whenever we change the camera settings, we will have discarded the
        // in-progress exposure, because the old settings were in effect when it
        // started. Not only that, but ASI cameras seem to have some kind of
        // internal pipeline, such that a few images need to be discarded after
        // a setting change.
        let pipeline_depth = 2;  // TODO: does this differ across ASI models?
        let mut discard_image_count = 0;
        loop {
            let capture_width;
            let capture_height;
            // Do we need to change any camera settings? This is also where
            // the initial camera settings are processed.
            {
                let mut locked_state = state.lock().unwrap();
                let new_settings = &locked_state.camera_settings;
                let old_settings = &locked_state.current_capture_settings;
                if locked_state.stop_request {
                    info!("Stopping video capture");
                    match sdk.stop_video_capture() {
                        Ok(()) => (),
                        Err(e) =>
                            warn!("Error stopping video capture: {}", &e.to_string())
                    }
                    locked_state.video_capture_thread = None;
                    capture_done.notify_all();
                    locked_state.stop_request = false;
                    return;
                }
                // Propagate changed settings, if any, into the camera.
                if starting || new_settings.flip != old_settings.flip {
                    match sdk.set_control_value(
                        asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_FLIP,
                        Self::sdk_flip(new_settings.flip) as i64,
                        /*auto=*/false) {
                        Ok(()) => (),
                        Err(e) => warn!("Error setting flip mode: {}", &e.to_string())
                    }
                }
                if starting ||
                    new_settings.exposure_duration != old_settings.exposure_duration
                {
                    match sdk.set_control_value(
                        asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_EXPOSURE,
                        new_settings.exposure_duration.as_micros() as i64,
                        /*auto=*/false) {
                        Ok(()) => (),
                        Err(e) => warn!("Error setting exposure: {}", &e.to_string())
                    }
                }
                let new_roi = &new_settings.roi;
                let old_roi = &old_settings.roi;
                if starting || new_roi.binning != old_roi.binning ||
                    new_roi.capture_dimensions != old_roi.capture_dimensions
                {
                    match sdk.set_roi_format(
                        new_roi.capture_dimensions.0, new_roi.capture_dimensions.1,
                        match new_roi.binning { BinFactor::X1 => 1, BinFactor::X2 => 2, },
                        asi_camera2_sdk::ASI_IMG_TYPE_ASI_IMG_RAW8) {
                        Ok(()) => (),
                        Err(e) => warn!("Error setting ROI: {}", &e.to_string())
                    }
                }
                (capture_width, capture_height) = new_roi.capture_dimensions;
                if starting || new_roi.capture_startpos != old_roi.capture_startpos {
                    match sdk.set_start_pos(
                        new_roi.capture_startpos.0, new_roi.capture_startpos.1) {
                        Ok(()) => (),
                        Err(e) => warn!("Error setting ROI startpos: {}", &e.to_string())
                    }
                }
                if starting || new_settings.gain != old_settings.gain {
                    match sdk.set_control_value(
                        asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_GAIN,
                        Self::sdk_gain(new_settings.gain.value(),
                                       locked_state.min_gain, locked_state.max_gain),
                        /*auto=*/false) {
                        Ok(()) => (),
                        Err(e) => warn!("Error setting gain: {}", &e.to_string())
                    }
                }
                if starting || new_settings.offset != old_settings.offset {
                    match sdk.set_control_value(
                        asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_OFFSET,
                        new_settings.offset.value() as i64,
                        /*auto=*/false) {
                        Ok(()) => (),
                        Err(e) => warn!("Error setting offset: {}", &e.to_string())
                    }
                }
                // We're done processing the new settings, they're now current.
                locked_state.current_capture_settings = locked_state.camera_settings;
                if starting {
                    match sdk.start_video_capture() {
                        Ok(()) => (),
                        Err(e) => {
                            error!("Error starting video capture: {}", &e.to_string());
                            locked_state.video_capture_thread = None;
                            return;  // Abandon thread execution!
                        }
                    }
                    info!("Starting video capture");
                    starting = false;
                }
            }  // state.lock().

            // Allocate uninitialized storage to receive the image data.
            let num_pixels = capture_width * capture_height;
            let mut image_data = Vec::<u8>::with_capacity(num_pixels as usize);
            unsafe { image_data.set_len(num_pixels as usize) }
            match sdk.get_video_data(image_data.as_mut_ptr(), num_pixels as i64,
                                     /*wait_ms=*/1000) {
                Ok(()) => (),
                Err(e) => {
                    warn!("Error getting video data: {}", &e.to_string());
                    continue
                }
            }
            if discard_image_count > 0 {
                discard_image_count -= 1;
            } else {
                let mut locked_state = state.lock().unwrap();
                if locked_state.setting_changed {
                    discard_image_count = pipeline_depth;
                    locked_state.setting_changed = false;
                } else {
                    let temp = match sdk.get_control_value(
                        asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_TEMPERATURE) {
                        Ok(x) => { Celsius((x.0 / 10) as i32) },
                        Err(e) => { warn!("Error getting temperature: {}",
                                          &e.to_string());
                                    Celsius(0) }
                    };
                    locked_state.most_recent_capture = Some(Arc::new(CapturedImage {
                        capture_params: locked_state.camera_settings,
                        image: GrayImage::from_raw(
                            capture_width as u32, capture_height as u32,
                            image_data).unwrap(),
                        readout_time: SystemTime::now(),
                        temperature: temp,
                    }));
                    locked_state.frame_id += 1;
                    capture_done.notify_all();
                }
            }
        }  // loop.
    }  // video_capture_thread_worker().

    // Translate our Flip to the camera SDK's flip enum value.
    fn sdk_flip(abstract_flip: Flip) -> u32 {
        match abstract_flip {
            Flip::None => asi_camera2_sdk::ASI_FLIP_STATUS_ASI_FLIP_NONE,
            Flip::Horizontal => asi_camera2_sdk::ASI_FLIP_STATUS_ASI_FLIP_HORIZ,
            Flip::Vertical => asi_camera2_sdk::ASI_FLIP_STATUS_ASI_FLIP_VERT,
            Flip::Both => asi_camera2_sdk::ASI_FLIP_STATUS_ASI_FLIP_BOTH,
        }
    }

    // Translate our 0..100 into the camera SDK's min/max range.
    fn sdk_gain(abstract_gain: i32, sdk_min_gain: i32, sdk_max_gain: i32)
                -> i64 {
        // The camera might not use 0..100 for range of gain values.
        if sdk_min_gain != 0 || sdk_max_gain != 100 {
            // Translate our 0..100 into the camera's min/max range.
            let frac = abstract_gain as f64 / 100.0;
            (sdk_min_gain as f64 + (sdk_max_gain - sdk_min_gain) as f64 * frac) as i64
        } else {
            abstract_gain as i64
        }
    }
}

/// We arrange to call stop() when ASICamera object goes out of scope.
impl Drop for ASICamera {
    fn drop(&mut self) {
        self.stop().unwrap_or_else(|err| {
            panic!("Error stopping camera: {}", err);
        });
    }
}

impl AbstractCamera for ASICamera {
    fn model(&self) -> Result<String, CanonicalError> {
        let state = self.state.lock().unwrap();
        let cstr = CStr::from_bytes_until_nul(&state.info.Name).unwrap();
        Ok(cstr.to_str().unwrap().to_owned())
    }

    fn dimensions(&self) -> (i32, i32) {
        let state = self.state.lock().unwrap();
        (state.info.MaxWidth as i32, state.info.MaxHeight as i32)
    }

    fn sensor_size(&self) -> (f32, f32) {
        let (width, height) = self.dimensions();
        let state = self.state.lock().unwrap();
        let pixel_size_microns = state.info.PixelSize as f64;
        ((width as f64 * pixel_size_microns / 1000.0) as f32,
         (height as f64 * pixel_size_microns / 1000.0) as f32)
    }

    fn optimal_gain(&self) -> Gain {
        let state = self.state.lock().unwrap();
        // We could do a match of the model() and research the optimal gain value
        // for each. Instead, we just grab ASI's default gain value according to
        // the SDK.
        // The camera might not use 0..100 for range of gain values.
        if state.min_gain == 0 && state.max_gain == 100 {
            return Gain::new(state.default_gain as i32);
        }
        // Normalize default_gain according to our 0..100 range.
        let frac = (state.default_gain - state.min_gain) as f64 /
            (state.max_gain - state.min_gain) as f64;
        Gain::new((100.0 * frac) as i32)
    }

    fn set_flip_mode(&mut self, flip_mode: Flip) -> Result<(), CanonicalError> {
        let mut state = self.state.lock().unwrap();
        state.camera_settings.flip = flip_mode;
        ASICamera::changed_setting(&mut state);
        Ok(())
    }
    fn get_flip_mode(&self) -> Flip {
        let state = self.state.lock().unwrap();
        state.camera_settings.flip
    }

    fn set_exposure_duration(&mut self, exp_duration: Duration)
                             -> Result<(), CanonicalError> {
        let mut state = self.state.lock().unwrap();
        state.camera_settings.exposure_duration = exp_duration;
        ASICamera::changed_setting(&mut state);
        Ok(())
    }
    fn get_exposure_duration(&self) -> Duration {
        let state = self.state.lock().unwrap();
        state.camera_settings.exposure_duration
    }

    fn set_region_of_interest(&mut self, mut roi: RegionOfInterest)
                              -> Result<RegionOfInterest, CanonicalError> {
        let mut state = self.state.lock().unwrap();

        // Validate/adjust capture dimensions.
        let (mut roi_width, mut roi_height) = roi.capture_dimensions;
        // ASI doc says width%8 must be 0, and height%2 must be 0.
        roi_width -= roi_width % 8;
        roi_height -= roi_height % 2;
        // Additionally, ASI doc says that for ASI120 model, width*height%1024
        // must be 0. We punt on this for now.
        roi.capture_dimensions = (roi_width, roi_height);
        state.camera_settings.roi = roi;
        ASICamera::changed_setting(&mut state);
        Ok(state.camera_settings.roi)
    }
    fn get_region_of_interest(&self) -> RegionOfInterest {
        let state = self.state.lock().unwrap();
        state.camera_settings.roi
    }

    fn set_gain(&mut self, gain: Gain) -> Result<(), CanonicalError> {
        let mut state = self.state.lock().unwrap();
        state.camera_settings.gain = gain;
        ASICamera::changed_setting(&mut state);
        Ok(())
    }
    fn get_gain(&self) -> Gain {
        let state = self.state.lock().unwrap();
        state.camera_settings.gain
    }

    fn set_offset(&mut self, offset: Offset) -> Result<(), CanonicalError> {
        let mut state = self.state.lock().unwrap();
        state.camera_settings.offset = offset;
        ASICamera::changed_setting(&mut state);
        Ok(())
    }
    fn get_offset(&self) -> Offset {
        let state = self.state.lock().unwrap();
        state.camera_settings.offset
    }

    fn capture_image(&mut self, prev_frame_id: Option<i32>)
                     -> Result<(Arc<CapturedImage>, i32), CanonicalError> {
        let mut state = self.state.lock().unwrap();
        // Start video capture thread if not yet started.
        if state.video_capture_thread.is_none() {
            let cloned_state = self.state.clone();
            let cloned_sdk = self.asi_cam_sdk.clone();
            let cloned_condvar = self.capture_done.clone();
            state.video_capture_thread = Some(thread::spawn(|| {
                ASICamera::video_capture_thread_worker(
                    cloned_state, cloned_condvar, cloned_sdk);
            }));
        }
        // Get the most recently posted image; wait if there is none yet or the
        // currently posted image's frame id is the same as `prev_frame_id`.
        while state.most_recent_capture.is_none() ||
            (prev_frame_id.is_some() && prev_frame_id.unwrap() == state.frame_id)
        {
            state = self.capture_done.wait(state).unwrap();
        }
        Ok((state.most_recent_capture.clone().unwrap(), state.frame_id))
    }

    fn stop(&mut self) -> Result<(), CanonicalError> {
        let mut state = self.state.lock().unwrap();
        if state.video_capture_thread.is_none() {
            return Ok(());
        }
        state.stop_request = true;
        while state.video_capture_thread.is_some() {
            state = self.capture_done.wait(state).unwrap();
        }
        Ok(())
    }
}
