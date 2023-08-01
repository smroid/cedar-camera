use canonical_error::{CanonicalError, failed_precondition_error};

use std::ffi::CStr;
use std::time::{Duration, Instant, SystemTime};

use log::info;

use asi_camera2::asi_camera2_sdk;
use crate::abstract_camera::{AbstractCamera, BinFactor, CaptureParams, CapturedImage,
                             Celsius, Flip, Gain, Offset, RegionOfInterest};

pub struct ASICamera {
    // The SDK wrapper object.
    asi_cam_sdk: asi_camera2_sdk::ASICamera,

    // Unchanging camera info.
    info: asi_camera2_sdk::ASI_CAMERA_INFO,
    default_gain: i32,
    min_gain: i32,
    max_gain: i32,

    // Current camera settings.
    camera_settings: CaptureParams,

    // Keep track of whether video capture is started.
    vid_running: bool,
}

impl ASICamera {
    /// Returns an ASICamera instance that implements the AbstractCamera API.
    pub fn new(asi_cam_sdk: asi_camera2_sdk::ASICamera)
               -> Result<Self, CanonicalError> {
        let info = match asi_cam_sdk.get_property() {
            Ok(x) => x,
            Err(e) => return Err(failed_precondition_error(&e.to_string()))
        };
        let mut asi_cam = ASICamera{asi_cam_sdk,
                                    info,
                                    default_gain: 0, min_gain: 0, max_gain: 0,
                                    camera_settings: CaptureParams{
                                        flip: Flip::None,
                                        exposure_duration: Duration::from_millis(100),
                                        roi: RegionOfInterest{binning: BinFactor::X1,
                                                              capture_dimensions: (-1, -1),
                                                              capture_startpos: (-1, -1)},
                                        gain: Gain::new(0),
                                        offset: Offset::new(0),
                                    },
                                    vid_running: false};
        match asi_cam.asi_cam_sdk.open() {
            Ok(()) => (),
            Err(e) => return Err(failed_precondition_error(&e.to_string()))
        }
        match asi_cam.asi_cam_sdk.init() {
            Ok(()) => (),
            Err(e) => return Err(failed_precondition_error(&e.to_string()))
        }
        // Find the camera's min/max gain values.
        let mut got_gain = false;
        let num_controls = asi_cam.asi_cam_sdk.get_num_controls().unwrap();
        for control_index in 0..num_controls {
            let control_caps = asi_cam.asi_cam_sdk.get_control_caps(control_index).unwrap();
            if control_caps.ControlType == asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_GAIN {
                asi_cam.default_gain = control_caps.DefaultValue as i32;
                asi_cam.min_gain = control_caps.MinValue as i32;
                asi_cam.max_gain = control_caps.MaxValue as i32;
                got_gain = true;
                break;
            }
        }
        if !got_gain {
            return Err(failed_precondition_error(
                "Could not find control caps for ASI_CONTROL_TYPE_ASI_GAIN"));
        }
        // Push the defaults to the camera.
        asi_cam.set_flip_mode(asi_cam.camera_settings.flip)?;
        asi_cam.set_exposure_duration(asi_cam.camera_settings.exposure_duration)?;
        asi_cam.set_region_of_interest(
            RegionOfInterest{ binning: BinFactor::X1,
                              capture_startpos: (0, 0),
                              capture_dimensions: asi_cam.dimensions(),
            })?;
        asi_cam.set_gain(asi_cam.optimal_gain())?;
        asi_cam.set_offset(Offset::new(0))?;

        info!("Created ASICamera API object");
        Ok(asi_cam)
    }  // new().
}

/// We arrange to call stop() when ASICamera object goes out of scope.
impl Drop for ASICamera {
    fn drop(&mut self) {
        self.stop().unwrap_or_else(|err| {
            panic!("Error stopping camera id {}: {}",
                   self.asi_cam_sdk.camera_id(), err);
        });
    }
}

impl AbstractCamera for ASICamera {
    fn model(&self) -> Result<String, CanonicalError> {
        let cstr = CStr::from_bytes_until_nul(&self.info.Name).unwrap();
        Ok(cstr.to_str().unwrap().to_owned())
    }

    fn dimensions(&self) -> (i32, i32) {
        (self.info.MaxWidth as i32, self.info.MaxHeight as i32)
    }

    fn optimal_gain(&self) -> Gain {
        // We could do a match of the model() and research the optimal gain value
        // for each. Instead, we just grab ASI's default gain value according to
        // the SDK.
        // The camera might not use 0..100 for range of gain values.
        if self.min_gain == 0 && self.max_gain == 100 {
            return Gain::new(self.default_gain as i32);
        }
        // Normalize default_gain according to our 0..100 range.
        let frac = (self.default_gain - self.min_gain) as f64 /
            (self.max_gain - self.min_gain) as f64;
        Gain::new((100.0 * frac) as i32)
    }

    fn set_flip_mode(&mut self, flip_mode: Flip) -> Result<(), CanonicalError> {
        let sdk_flip = match flip_mode {
            Flip::None => asi_camera2_sdk::ASI_FLIP_STATUS_ASI_FLIP_NONE,
            Flip::Horizontal => asi_camera2_sdk::ASI_FLIP_STATUS_ASI_FLIP_HORIZ,
            Flip::Vertical => asi_camera2_sdk::ASI_FLIP_STATUS_ASI_FLIP_VERT,
            Flip::Both => asi_camera2_sdk::ASI_FLIP_STATUS_ASI_FLIP_BOTH,
        };
        match self.asi_cam_sdk.set_control_value(
            asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_FLIP, sdk_flip as i64,
            /*auto=*/false) {
            Ok(x) => { self.camera_settings.flip = flip_mode; return Ok(x) },
            Err(e) => Err(failed_precondition_error(&e.to_string()))
        }
    }
    fn get_flip_mode(&self) -> Flip { self.camera_settings.flip }

    fn set_exposure_duration(&mut self, exp_duration: Duration)
                             -> Result<(), CanonicalError> {
        let exp_micros = exp_duration.as_micros() as i64;
        match self.asi_cam_sdk.set_control_value(
            asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_EXPOSURE, exp_micros,
            /*auto=*/false) {
            Ok(()) => { self.camera_settings.exposure_duration =
                        exp_duration; Ok(()) },
            Err(e) => Err(failed_precondition_error(&e.to_string()))
        }
    }
    fn get_exposure_duration(&self) -> Duration {
        self.camera_settings.exposure_duration
    }

    fn set_region_of_interest(&mut self, roi: RegionOfInterest)
                              -> Result<RegionOfInterest, CanonicalError> {
        // ASI SDK has separate functions for setting binning & capture size vs
        // capture position. Detect what's changing and only make the needed
        // call(s).
        if self.camera_settings.roi.binning != roi.binning ||
           self.camera_settings.roi.capture_dimensions != roi.capture_dimensions {
            // Validate/adjust capture dimensions.
            let (mut roi_width, mut roi_height) = roi.capture_dimensions;
            // ASI doc says width%8 must be 0, and height%2 must be 0.
            roi_width -= roi_width % 8;
            roi_height -= roi_height % 2;
            // Additionally, ASI doc says that for ASI120 model, width*height%1024
            // must be 0. We punt on this for now.

            match self.asi_cam_sdk.set_roi_format(
                roi_width, roi_height,
                match roi.binning { BinFactor::X1 => 1, BinFactor::X2 => 2, },
                asi_camera2_sdk::ASI_IMG_TYPE_ASI_IMG_RAW8) {
                Ok(()) => { self.camera_settings.roi.binning = roi.binning;
                            self.camera_settings.roi.capture_dimensions =
                            (roi_width, roi_height); },
                Err(e) => return Err(failed_precondition_error(&e.to_string()))
            }
        }
        if self.camera_settings.roi.capture_startpos != roi.capture_startpos {
            match self.asi_cam_sdk.set_start_pos(
                roi.capture_startpos.0, roi.capture_startpos.1,) {
                Ok(()) => { self.camera_settings.roi.capture_startpos =
                            roi.capture_startpos; },
                Err(e) => return Err(failed_precondition_error(&e.to_string()))
            }
        }
        Ok(self.camera_settings.roi)
    }
    fn get_region_of_interest(&self) -> RegionOfInterest { self.camera_settings.roi }

    fn set_gain(&mut self, gain: Gain) -> Result<(), CanonicalError> {
        let mut camera_gain = gain.value();
        // The camera might not use 0..100 for range of gain values.
        if self.min_gain != 0 || self.max_gain != 100 {
            // Translate our 0..100 into the camera's min/max range.
            let frac = gain.value() as f64 / 100.0;
            camera_gain =
                self.min_gain + ((self.max_gain - self.min_gain) as f64 * frac) as i32;
        }
        match self.asi_cam_sdk.set_control_value(
            asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_GAIN, camera_gain as i64,
            /*auto=*/false) {
            Ok(()) => { self.camera_settings.gain = gain; Ok(()) },
            Err(e) => Err(failed_precondition_error(&e.to_string()))
        }
    }
    fn get_gain(&self) -> Gain { self.camera_settings.gain }

    fn set_offset(&mut self, offset: Offset) -> Result<(), CanonicalError> {
        match self.asi_cam_sdk.set_control_value(
            asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_OFFSET, offset.value() as i64,
            /*auto=*/false) {
            Ok(()) => { self.camera_settings.offset = offset; Ok(()) },
            Err(e) => Err(failed_precondition_error(&e.to_string()))
        }
    }
    fn get_offset(&self) -> Offset { self.camera_settings.offset }

    fn capture_image(&mut self) -> Result<CapturedImage, CanonicalError> {
        let capture_start = Instant::now();
        if !self.vid_running {
            info!("Starting video capture");
            match self.asi_cam_sdk.start_video_capture() {
                Ok(()) => { self.vid_running = true; },
                Err(e) => return Err(failed_precondition_error(&e.to_string()))
            }
        }
        let (w, h) = self.camera_settings.roi.capture_dimensions;
        let mut image_data = Vec::<u8>::new();
        image_data.resize((w * h) as usize, 0);
        match self.asi_cam_sdk.get_video_data(image_data.as_mut_ptr(), (w*h) as i64,
                                              /*wait_ms=*/-1) {
            Ok(()) => (),
            Err(e) => return Err(failed_precondition_error(&e.to_string()))
        }
        let temp = match self.asi_cam_sdk.get_control_value(
            asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_TEMPERATURE) {
            Ok(x) => { Celsius((x.0 / 10) as i32) },
            Err(e) => { return Err(failed_precondition_error(&e.to_string())); }
        };
        info!("capture_image took {:?}", capture_start.elapsed());
        Ok(CapturedImage {
            capture_params: self.camera_settings,
            image_data: image_data,
            readout_time: SystemTime::now(),
            temperature: temp,
        })
    }

    fn stop(&mut self) -> Result<(), CanonicalError> {
        if self.vid_running {
            info!("Stopping video capture");
            match self.asi_cam_sdk.stop_video_capture() {
                Ok(()) => { self.vid_running = false; },
                Err(e) => return Err(failed_precondition_error(&e.to_string()))
            }
        }
        Ok(())
    }
}
