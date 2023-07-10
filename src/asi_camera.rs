use canonical_error::{CanonicalError, failed_precondition_error,
                      invalid_argument_error};

use std::ffi::CStr;
use std::time::{Duration, SystemTime};

use asi_camera2::asi_camera2_sdk;
use crate::abstract_camera::{AbstractCamera, BinFactor, CapturedImage,
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
    flip: Flip,
    exposure_duration: Duration,
    roi: RegionOfInterest,
    gain: Gain,
    offset: Offset,

    // Keep track of whether video capture is started.
    vid_running: bool,
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
            Ok(x) => { self.flip = flip_mode; return Ok(x) },
            Err(e) => Err(failed_precondition_error(&e.to_string()))
        }
    }
    fn get_flip_mode(&self) -> Flip { self.flip }

    fn set_exposure_duration(&mut self, exp_duration: Duration)
                             -> Result<(), CanonicalError> {
        let exp_micros = exp_duration.as_micros() as i64;
        match self.asi_cam_sdk.set_control_value(
            asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_EXPOSURE, exp_micros,
            /*auto=*/false) {
            Ok(()) => { self.exposure_duration = exp_duration; Ok(()) },
            Err(e) => Err(failed_precondition_error(&e.to_string()))
        }
    }
    fn get_exposure_duration(&self) -> Duration { self.exposure_duration }

    fn set_region_of_interest(&mut self, roi: RegionOfInterest)
                              -> Result<RegionOfInterest, CanonicalError> {
        // ASI SDK has separate functions for setting binning & capture size vs
        // capture position. Detect what's changing and only make the needed
        // call(s).
        if self.roi.binning != roi.binning ||
           self.roi.capture_dimensions != roi.capture_dimensions {
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
                Ok(()) => { self.roi.binning = roi.binning;
                            self.roi.capture_dimensions = (roi_width, roi_height); },
                Err(e) => return Err(failed_precondition_error(&e.to_string()))
            }
        }
        if self.roi.capture_startpos != roi.capture_startpos {
            match self.asi_cam_sdk.set_start_pos(
                roi.capture_startpos.0, roi.capture_startpos.1,) {
                Ok(()) => { self.roi.capture_startpos = roi.capture_startpos; },
                Err(e) => return Err(failed_precondition_error(&e.to_string()))
            }
        }
        Ok(self.roi)
    }
    fn get_region_of_interest(&self) -> RegionOfInterest { self.roi }

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
            Ok(()) => { self.gain = gain; Ok(()) },
            Err(e) => Err(failed_precondition_error(&e.to_string()))
        }
    }
    fn get_gain(&self) -> Gain { self.gain }

    fn set_offset(&mut self, offset: Offset) -> Result<(), CanonicalError> {
        match self.asi_cam_sdk.set_control_value(
            asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_OFFSET, offset.value() as i64,
            /*auto=*/false) {
            Ok(()) => { self.offset = offset; Ok(()) },
            Err(e) => Err(failed_precondition_error(&e.to_string()))
        }
    }
    fn get_offset(&self) -> Offset { self.offset }

    fn capture_image(&mut self, mut image_data: Vec<u8>)
                     -> Result<CapturedImage, CanonicalError> {
        if !self.vid_running {
            match self.asi_cam_sdk.start_video_capture() {
                Ok(()) => { self.vid_running = true; },
                Err(e) => return Err(failed_precondition_error(&e.to_string()))
            }
        }
        let (w, h) = self.roi.capture_dimensions;
        if (image_data.len() as i32) < w * h {
            return Err(invalid_argument_error(
                format!("image_data length {} too small for ROI width*height {}*{}",
                        image_data.len(), w, h).as_str()));
        }
        match self.asi_cam_sdk.get_video_data(image_data.as_mut_ptr(), (w*h) as i64,
                                              /*wait_ms=*/-1) {
            Ok(()) => (),
            Err(e) => return Err(failed_precondition_error(&e.to_string()))
        }
        let temp = match self.asi_cam_sdk.get_control_value(
            asi_camera2_sdk::ASI_CONTROL_TYPE_ASI_TEMPERATURE) {
            Ok(x) => { Celsius(x.0 as i32) },
            Err(e) => { return Err(failed_precondition_error(&e.to_string())); }
        };
        Ok(CapturedImage {
            image_data: image_data,
            flip: self.flip,
            exposure_duration: self.exposure_duration,
            roi: self.roi,
            gain: self.gain,
            offset: self.offset,
            readout_time: SystemTime::now(),
            temperature: temp,
        })
    }

    fn stop(&mut self) -> Result<(), CanonicalError> {
        if self.vid_running {
            match self.asi_cam_sdk.stop_video_capture() {
                Ok(()) => { self.vid_running = false; },
                Err(e) => return Err(failed_precondition_error(&e.to_string()))
            }
        }
        Ok(())
    }
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

/// Returns an ASICamera instance that implements the AbstractCamera API.
#[allow(dead_code)]
pub fn create_asi_camera(asi_cam_sdk: asi_camera2_sdk::ASICamera)
                         -> Result<ASICamera, CanonicalError> {
    let info = match asi_cam_sdk.get_property() {
        Ok(x) => x,
        Err(e) => return Err(failed_precondition_error(&e.to_string()))
    };
    let mut asi_cam = ASICamera{asi_cam_sdk,
                                info,
                                default_gain: 0, min_gain: 0, max_gain: 0,
                                flip: Flip::None,
                                exposure_duration: Duration::from_millis(100),
                                roi: RegionOfInterest{binning: BinFactor::X1,
                                                      capture_dimensions: (-1, -1),
                                                      capture_startpos: (-1, -1)},
                                gain: Gain::new(0),
                                offset: Offset::new(0),
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
    asi_cam.set_flip_mode(asi_cam.flip)?;
    asi_cam.set_exposure_duration(asi_cam.exposure_duration)?;
    asi_cam.set_region_of_interest(
        RegionOfInterest{ binning: BinFactor::X1,
                          capture_startpos: (0, 0),
                          capture_dimensions: asi_cam.dimensions(),
        })?;
    asi_cam.set_gain(asi_cam.optimal_gain())?;
    asi_cam.set_offset(Offset::new(0))?;

    Ok(asi_cam)
}
