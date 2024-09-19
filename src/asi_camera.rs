use canonical_error::{CanonicalError, failed_precondition_error};

use std::ffi::CStr;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant, SystemTime};

use async_trait::async_trait;
use image::GrayImage;
use log::{debug, info, warn};

use asi_camera2::asi_camera2_sdk;
use crate::abstract_camera::{AbstractCamera, CaptureParams, CapturedImage,
                             Celsius, EnumeratedCameraInfo, Gain, Offset};

pub struct ASICamera {
    // The SDK wrapper object. After initialization, the video capture thread is
    // the only thing that touches asi_cam_sdk.
    asi_cam_sdk: Arc<Mutex<asi_camera2_sdk::ASICamera>>,

    // Unchanging camera info.
    info: asi_camera2_sdk::ASI_CAMERA_INFO,
    // Gain values in SDK units.
    default_gain: i32,
    min_gain: i32,
    max_gain: i32,

    // Our state, shared between ASICamera methods and the video capture thread.
    state: Arc<Mutex<SharedState>>,

    // Our video capture thread. Executes worker().
    video_capture_thread: Option<tokio::task::JoinHandle<()>>,
}

// State shared between video capture thread and the ASICamera methods.
struct SharedState {
    // Current camera settings as set via ASICamera methods. Will be put into
    // effect when the current exposure finishes, influencing the following
    // exposure.
    camera_settings: CaptureParams,
    inverted: bool,
    setting_changed: bool,

    // Zero means go fast as camera frames are available.
    update_interval: Duration,

    // Estimated time at which `most_recent_capture` will next be updated.
    eta: Option<Instant>,

    // Most recent completed capture and its id value.
    most_recent_capture: Option<CapturedImage>,
    frame_id: i32,

    // Set by stop(); the video capture thread exits when it sees this.
    stop_request: bool,

    // Camera settings in effect when the in-progress capture started.
    current_capture_settings: CaptureParams,
    current_capture_inverted: bool,
}

impl ASICamera {
    pub fn enumerate_cameras() -> Vec<EnumeratedCameraInfo> {
        let mut answer = vec![];
        let num_cameras = asi_camera2_sdk::ASICamera::num_connected_asi_cameras();
        for cam_index in 0..num_cameras {
            let camera_info = asi_camera2_sdk::ASICamera::get_property(cam_index).unwrap();
            let cstr = CStr::from_bytes_until_nul(&camera_info.Name).unwrap();
            answer.push(EnumeratedCameraInfo{model: cstr.to_str().unwrap().to_owned(),
                                             width: camera_info.MaxWidth as u32,
                                             height: camera_info.MaxHeight as u32});
        }
        answer
    }

    fn open_and_init(asi_cam_sdk: &mut asi_camera2_sdk::ASICamera)
                     -> Result<(), CanonicalError> {
        if let Err(e) = asi_cam_sdk.open() {
            return Err(failed_precondition_error(&e.to_string()));
        }
        if let Err(e) = asi_cam_sdk.init() {
            return Err(failed_precondition_error(&e.to_string()));
        }
        Ok(())
    }

    /// Returns an ASICamera instance that implements the AbstractCamera API.
    /// `camera_index` is w.r.t. the enumerate_cameras() vector length.
    pub fn new(camera_index: i32) -> Result<Self, CanonicalError> {
        let info = match asi_camera2_sdk::ASICamera::get_property(camera_index) {
            Ok(prop) => prop,
            Err(e) => return Err(failed_precondition_error(&e.to_string()))
        };
        let mut asi_cam_sdk = asi_camera2_sdk::ASICamera::new(info.CameraID);

        // Find the camera's min/max gain values.
        loop {
            if let Err(open_err) = Self::open_and_init(&mut asi_cam_sdk) {
                warn!("Error opening asi_cam_sdk: {}", &open_err.to_string());
                if let Err(close_err) = asi_cam_sdk.close() {
                    warn!("Error closing asi_cam_sdk: {}", &close_err.to_string());
                }
                asi_camera2_sdk::reset_asi_cameras();
                std::thread::sleep(Duration::from_secs(1));
                continue;
            }
            break;
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
            info, default_gain, min_gain, max_gain,
            state: Arc::new(Mutex::new(SharedState{
                camera_settings: CaptureParams::new(),
                inverted: false,
                setting_changed: false,
                update_interval: Duration::ZERO,
                eta: None,
                most_recent_capture: None,
                frame_id: 0,
                stop_request: false,
                current_capture_settings: CaptureParams::new(),
                current_capture_inverted: false,
            })),
            video_capture_thread: None,
        };
        cam.set_gain(cam.optimal_gain())?;
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
    fn worker(min_gain: i32, max_gain: i32,
              width: usize, height: usize,
              state: Arc<Mutex<SharedState>>,
              asi_cam_sdk: Arc<Mutex<asi_camera2_sdk::ASICamera>>) {
        debug!("Starting ASICamera worker");
        let mut locked_sdk = asi_cam_sdk.lock().unwrap();
        let mut starting = true;
        let mut recover_camera = false;
        // Whenever we change the camera settings, we will have discarded the
        // in-progress exposure, because the old settings were in effect when it
        // started. Not only that, but ASI cameras seem to have some kind of
        // internal pipeline, such that a few images need to be discarded after
        // a setting change.
        let pipeline_depth = 2;  // TODO: does this differ across ASI models?
        let mut discard_image_count = 0;
        // Keep track of when we grabbed a frame.
        let mut last_frame_time: Option<Instant> = None;
        loop {
            let update_interval: Duration;
            let exp_duration;
            // Do we need to change any camera settings? This is also where
            // the initial camera settings are processed.
            {
                let mut locked_state = state.lock().unwrap();
                if recover_camera {
                    if let Err(close_err) = locked_sdk.close() {
                        warn!("Error closing asi_cam_sdk: {}", &close_err.to_string());
                    }
                    asi_camera2_sdk::reset_asi_cameras();
                    std::thread::sleep(Duration::from_secs(5));
                    if let Err(open_err) = Self::open_and_init(&mut locked_sdk) {
                        warn!("Error reopening asi_cam_sdk: {}", &open_err.to_string());
                        continue;
                    }
                    recover_camera = false;
                    starting = true;  // Re-init all camera params.
                }
                update_interval = locked_state.update_interval;
                let new_settings = &locked_state.camera_settings;
                let old_settings = &locked_state.current_capture_settings;
                let old_inverted = locked_state.current_capture_inverted;
                if locked_state.stop_request {
                    debug!("Stopping video capture");
                    if let Err(e) = locked_sdk.stop_video_capture() {
                        warn!("Error stopping video capture: {}", &e.to_string());
                    }
                    locked_state.stop_request = false;
                    return;  // Exit thread.
                }
                // Propagate changed settings, if any, into the camera.
                if starting ||
                    new_settings.exposure_duration != old_settings.exposure_duration
                {
                    if let Err(e) = locked_sdk.set_control_value(
                        asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_EXPOSURE,
                        new_settings.exposure_duration.as_micros() as i64,
                        /*auto=*/false)
                    {
                        warn!("Error setting exposure: {}", &e.to_string());
                    }
                }
                exp_duration = new_settings.exposure_duration;
                if starting || new_settings.gain != old_settings.gain {
                    if let Err(e) = locked_sdk.set_control_value(
                        asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_GAIN,
                        Self::sdk_gain(new_settings.gain.value(),
                                       min_gain, max_gain),
                        /*auto=*/false)
                    {
                        warn!("Error setting gain to {:?}: {}",
                              Self::sdk_gain(new_settings.gain.value(),
                                             min_gain, max_gain), &e.to_string());
                    }
                }
                if starting || new_settings.offset != old_settings.offset {
                    if let Err(e) = locked_sdk.set_control_value(
                        asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_OFFSET,
                        new_settings.offset.value() as i64,
                        /*auto=*/false)
                    {
                        warn!("Error setting offset to {:?}: {}",
                              new_settings.offset, &e.to_string());
                    }
                }
                if starting || locked_state.inverted != old_inverted {
                    let value = if locked_state.inverted {
                        asi_camera2_sdk::ASI_FLIP_STATUS_ASI_FLIP_BOTH
                    } else {
                        asi_camera2_sdk::ASI_FLIP_STATUS_ASI_FLIP_NONE
                    } as i64;
                    if let Err(e) = locked_sdk.set_control_value(
                        asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_FLIP,
                        value, /*auto=*/false)
                    {
                        warn!("Error setting inverted to {}: {}",
                              locked_state.inverted, &e.to_string());
                    }
                }
                // We're done processing the new settings, they're now current.
                locked_state.current_capture_settings = locked_state.camera_settings;
                locked_state.current_capture_inverted = locked_state.inverted;
                if starting {
                    if let Err(e) = locked_sdk.start_video_capture() {
                        warn!("Error starting video capture: {:?}; resetting and retrying", e);
                        recover_camera = true;
                        continue;
                    }
                    debug!("Starting video capture");
                    starting = false;
                }
            }  // state.lock().

            // Is it time to grab a frame?
            let now = Instant::now();
            if last_frame_time.is_some() {
                let next_update_time = last_frame_time.unwrap() + update_interval;
                if next_update_time > now {
                    let delay = next_update_time - now;
                    state.lock().unwrap().eta = Some(Instant::now() + delay);
                    std::thread::sleep(delay);
                    continue;
                }
                state.lock().unwrap().eta = None;
            }

            // Time to grab a frame.
            last_frame_time = Some(now);

            // Allocate uninitialized storage to receive the image data.
            let num_pixels:usize = width * height;
            let mut image_data = Vec::<u8>::with_capacity(num_pixels as usize);
            unsafe { image_data.set_len(num_pixels as usize) }
            if let Err(e) = locked_sdk.get_video_data(
                image_data.as_mut_ptr(), num_pixels as i64, /*wait_ms=*/2000) {
                warn!("Error getting video data: {:?}; resetting and retrying", e);
                recover_camera = true;
                continue;
            }
            if update_interval == Duration::ZERO {
                state.lock().unwrap().eta = Some(Instant::now() + exp_duration);
            }
            if discard_image_count > 0 {
                discard_image_count -= 1;
            } else {
                let mut locked_state = state.lock().unwrap();
                if locked_state.setting_changed {
                    discard_image_count = pipeline_depth;
                    locked_state.setting_changed = false;
                } else {
                    let temp = match locked_sdk.get_control_value(
                        asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_TEMPERATURE) {
                        Ok(x) => { Celsius((x.0 / 10) as i32) },
                        Err(e) => {
                            warn!("Error getting temperature: {}", &e.to_string());
                            Celsius(0)
                        }
                    };
                    let image = GrayImage::from_raw(width as u32, height as u32,
                                                    image_data).unwrap();
                    locked_state.most_recent_capture = Some(CapturedImage {
                        capture_params: locked_state.camera_settings,
                        image: Arc::new(image),
                        readout_time: SystemTime::now(),
                        temperature: temp,
                    });
                    locked_state.frame_id += 1;
                }
            }
        }  // loop.
    }  // worker().

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
        // https://stackoverflow.com/questions/71541765/rust-async-drop
        futures::executor::block_on(self.stop());
    }
}

#[async_trait]
impl AbstractCamera for ASICamera {
    fn model(&self) -> String {
        let cstr = CStr::from_bytes_until_nul(&self.info.Name).unwrap();
        cstr.to_str().unwrap().to_owned()
    }

    fn dimensions(&self) -> (i32, i32) {
        (self.info.MaxWidth as i32, self.info.MaxHeight as i32)
    }

    fn is_color(&self) -> bool {
        self.info.IsColorCam != 0
    }

    fn sensor_size(&self) -> (f32, f32) {
        let (width, height) = self.dimensions();
        let pixel_size_microns = self.info.PixelSize as f64;
        ((width as f64 * pixel_size_microns / 1000.0) as f32,
         (height as f64 * pixel_size_microns / 1000.0) as f32)
    }

    fn optimal_gain(&self) -> Gain {
        let optimal_gain;  // In SDK units.
        // Use the optimal gain value for each ASI camera model.
        match self.model().as_str() {
            "ZWO ASI120MM Mini" => {
                // Per graphs at
                // https://astronomy-imaging-camera.com/product/asi120mm-mini-mono
                // the read noise is low at this gain while the dynamic range
                // is ~9, which is adequate for our use of 8-bit mode.
                optimal_gain = 50;
            },
            _ => {
                // Likely a decent fallback.
                optimal_gain = self.default_gain;
            }
        }
        // Normalize optimal_gain according to our 0..100 range.
        let frac = (optimal_gain - self.min_gain) as f64 /
            (self.max_gain - self.min_gain) as f64;
        Gain::new((100.0 * frac) as i32)
    }

    fn set_exposure_duration(&mut self, exp_duration: Duration)
                             -> Result<(), CanonicalError> {
        let mut locked_state = self.state.lock().unwrap();
        if locked_state.camera_settings.exposure_duration != exp_duration {
            ASICamera::changed_setting(&mut locked_state);
        }
        locked_state.camera_settings.exposure_duration = exp_duration;
        Ok(())
    }
    fn get_exposure_duration(&self) -> Duration {
        let locked_state = self.state.lock().unwrap();
        locked_state.camera_settings.exposure_duration
    }

    fn set_gain(&mut self, gain: Gain) -> Result<(), CanonicalError> {
        let mut locked_state = self.state.lock().unwrap();
        if locked_state.camera_settings.gain != gain {
            ASICamera::changed_setting(&mut locked_state);
        }
        locked_state.camera_settings.gain = gain;
        Ok(())
    }
    fn get_gain(&self) -> Gain {
        let locked_state = self.state.lock().unwrap();
        locked_state.camera_settings.gain
    }

    fn set_offset(&mut self, offset: Offset) -> Result<(), CanonicalError> {
        let mut locked_state = self.state.lock().unwrap();
        if locked_state.camera_settings.offset != offset {
            ASICamera::changed_setting(&mut locked_state);
        }
        locked_state.camera_settings.offset = offset;
        Ok(())
    }
    fn get_offset(&self) -> Offset {
        let locked_state = self.state.lock().unwrap();
        locked_state.camera_settings.offset
    }

    fn set_inverted(&mut self, inverted: bool) -> Result<(), CanonicalError> {
        let mut locked_state = self.state.lock().unwrap();
        if locked_state.inverted != inverted {
            ASICamera::changed_setting(&mut locked_state);
        }
        locked_state.inverted = inverted;
        Ok(())
    }
    fn get_inverted(&self) -> bool {
        let locked_state = self.state.lock().unwrap();
        locked_state.inverted
    }

    fn set_update_interval(&mut self, update_interval: Duration)
                           -> Result<(), CanonicalError> {
        let mut locked_state = self.state.lock().unwrap();
        locked_state.update_interval = update_interval;
        Ok(())
    }

    async fn capture_image(&mut self, prev_frame_id: Option<i32>)
                           -> Result<(CapturedImage, i32), CanonicalError> {
        // Has the worker terminated for some reason?
        if self.video_capture_thread.is_some() &&
            self.video_capture_thread.as_ref().unwrap().is_finished()
        {
            self.video_capture_thread.take().unwrap().await.unwrap();
        }
        // Start video capture thread if terminated or not yet started.
        if self.video_capture_thread.is_none() {
            let min_gain = self.min_gain;
            let max_gain = self.max_gain;
            let width = self.info.MaxWidth as usize;
            let height = self.info.MaxHeight as usize;
            let cloned_state = self.state.clone();
            let cloned_sdk = self.asi_cam_sdk.clone();
            self.video_capture_thread = Some(tokio::task::spawn_blocking(move || {
                ASICamera::worker(min_gain, max_gain, width, height, cloned_state, cloned_sdk);
            }));
        }
        // Get the most recently posted image; wait if there is none yet or the
        // currently posted image's frame id is the same as `prev_frame_id`.
        loop {
            let mut sleep_duration = Duration::from_millis(1);
            {
                let locked_state = self.state.lock().unwrap();
                if locked_state.most_recent_capture.is_some() &&
                    (prev_frame_id.is_none() ||
                     prev_frame_id.unwrap() != locked_state.frame_id)
                {
                    // Don't consume it, other clients may want it.
                    return Ok((locked_state.most_recent_capture.clone().unwrap(),
                               locked_state.frame_id));
                }
                if locked_state.eta.is_some() {
                    let time_to_eta =
                        locked_state.eta.unwrap().saturating_duration_since(Instant::now());
                    if time_to_eta > sleep_duration {
                        sleep_duration = time_to_eta;
                    }
                }
            }
            tokio::time::sleep(sleep_duration).await;
        }
    }

    fn estimate_delay(&self, prev_frame_id: Option<i32>) -> Option<Duration> {
        let locked_state = self.state.lock().unwrap();
        if locked_state.most_recent_capture.is_some() &&
            (prev_frame_id.is_none() ||
             prev_frame_id.unwrap() != locked_state.frame_id)
        {
            Some(Duration::ZERO)
        } else if locked_state.eta.is_some() {
            Some(locked_state.eta.unwrap().saturating_duration_since(Instant::now()))
        } else {
            None
        }
    }

    async fn stop(&mut self) {
        if self.video_capture_thread.is_some() {
            self.state.lock().unwrap().stop_request = true;
            self.video_capture_thread.take().unwrap().await.unwrap();
        }
    }
}
