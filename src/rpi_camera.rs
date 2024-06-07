use canonical_error::{CanonicalError, failed_precondition_error};

use async_trait::async_trait;

use libcamera::{
    // camera::CameraConfigurationStatus,
    camera_manager::CameraManager,
    // framebuffer::AsFrameBuffer,
    // framebuffer_allocator::{FrameBuffer, FrameBufferAllocator},
    // framebuffer_map::MemoryMappedFrameBuffer,
    // pixel_format::PixelFormat,
    properties,
    // stream::StreamRole,
};

use crate::abstract_camera::{AbstractCamera, BinFactor, CaptureParams, CapturedImage,
                             Celsius, EnumeratedCameraInfo, Flip, Gain, Offset,
                             RegionOfInterest};

pub struct RpiCamera {
}

impl RpiCamera {
    pub fn enumerate_cameras() -> Vec<EnumeratedCameraInfo> {
        let mgr = CameraManager::new().unwrap();
        let cameras = mgr.cameras();
        let mut answer = vec![];
        for i in 0..cameras.len() {
            let cam = cameras.get(i).unwrap();
            let props = cam.properties();
            let model = props.get::<properties::Model>().unwrap();
            let pixel_array_size = props.get::<properties::PixelArraySize>().unwrap();
            answer.push(EnumeratedCameraInfo{
                model: model.to_string(),
                width: pixel_array_size.width,
                height: pixel_array_size.height})
        }
        answer
    }

    // Returns a RpiCamera instance that implements the AbstractCamera API.
    // `camera_index` is w.r.t. the enumerate_cameras() vector length.
    // pub fn new(camera_index: i32) -> Result<Self, CanonicalError> {
    // }
}

/// We arrange to call stop() when RpiCamera object goes out of scope.
// TODO: revise
impl Drop for RpiCamera {
    fn drop(&mut self) {
        // // https://stackoverflow.com/questions/71541765/rust-async-drop
        // futures::executor::block_on(self.stop());
    }
}

// #[async_trait]  // TODO: drop async_trait?
// impl AbstractCamera for RpiCamera {
    // fn model(&self) -> Result<String, CanonicalError> {
    //     let cstr = CStr::from_bytes_until_nul(&self.info.Name).unwrap();
    //     Ok(cstr.to_str().unwrap().to_owned())
    // }

    // fn dimensions(&self) -> (i32, i32) {
    //     (self.info.MaxWidth as i32, self.info.MaxHeight as i32)
    // }

    // fn sensor_size(&self) -> (f32, f32) {
    //     let (width, height) = self.dimensions();
    //     let pixel_size_microns = self.info.PixelSize as f64;
    //     ((width as f64 * pixel_size_microns / 1000.0) as f32,
    //      (height as f64 * pixel_size_microns / 1000.0) as f32)
    // }

    // fn optimal_gain(&self) -> Gain {
    //     let optimal_gain;  // In SDK units.
    //     // Use the optimal gain value for each ASI camera model.
    //     match self.model().unwrap().as_str() {
    //         "ZWO ASI120MM Mini" => {
    //             // Per graphs at
    //             // https://astronomy-imaging-camera.com/product/asi120mm-mini-mono
    //             // the read noise is low at this gain while the dynamic range
    //             // is ~9, which is adequate for our use of 8-bit mode.
    //             optimal_gain = 50;
    //         },
    //         _ => {
    //             // Likely a decent fallback.
    //             optimal_gain = self.default_gain;
    //         }
    //     }
    //     // Normalize optimal_gain according to our 0..100 range.
    //     let frac = (optimal_gain - self.min_gain) as f64 /
    //         (self.max_gain - self.min_gain) as f64;
    //     Gain::new((100.0 * frac) as i32)

    //     // We could do a match of the model() and research the optimal gain value
    //     // for each. Instead, we just grab ASI's default gain value according to
    //     // the SDK.
    //     // Normalize default_gain according to our 0..100 range.
    //     // let frac = (self.default_gain - self.min_gain) as f64 /
    //     //     (self.max_gain - self.min_gain) as f64;
    //     // Gain::new((100.0 * frac) as i32)
    // }

    // fn set_flip_mode(&mut self, flip_mode: Flip) -> Result<(), CanonicalError> {
    //     let mut locked_state = self.state.lock().unwrap();
    //     locked_state.camera_settings.flip = flip_mode;
    //     ASICamera::changed_setting(&mut locked_state);
    //     Ok(())
    // }
    // fn get_flip_mode(&self) -> Flip {
    //     let locked_state = self.state.lock().unwrap();
    //     locked_state.camera_settings.flip
    // }

    // fn set_exposure_duration(&mut self, exp_duration: Duration)
    //                          -> Result<(), CanonicalError> {
    //     let mut locked_state = self.state.lock().unwrap();
    //     locked_state.camera_settings.exposure_duration = exp_duration;
    //     ASICamera::changed_setting(&mut locked_state);
    //     Ok(())
    // }
    // fn get_exposure_duration(&self) -> Duration {
    //     let locked_state = self.state.lock().unwrap();
    //     locked_state.camera_settings.exposure_duration
    // }

    // fn set_region_of_interest(&mut self, mut roi: RegionOfInterest)
    //                           -> Result<RegionOfInterest, CanonicalError> {
    //     let mut locked_state = self.state.lock().unwrap();

    //     // Validate/adjust capture dimensions.
    //     let (mut roi_width, mut roi_height) = roi.capture_dimensions;
    //     // ASI doc says width%8 must be 0, and height%2 must be 0.
    //     roi_width -= roi_width % 8;
    //     roi_height -= roi_height % 2;
    //     // Additionally, ASI doc says that for ASI120 model, width*height%1024
    //     // must be 0. We punt on this for now.
    //     roi.capture_dimensions = (roi_width, roi_height);
    //     locked_state.camera_settings.roi = roi;
    //     ASICamera::changed_setting(&mut locked_state);
    //     Ok(locked_state.camera_settings.roi)
    // }
    // fn get_region_of_interest(&self) -> RegionOfInterest {
    //     let locked_state = self.state.lock().unwrap();
    //     locked_state.camera_settings.roi
    // }

    // fn set_gain(&mut self, gain: Gain) -> Result<(), CanonicalError> {
    //     let mut locked_state = self.state.lock().unwrap();
    //     locked_state.camera_settings.gain = gain;
    //     ASICamera::changed_setting(&mut locked_state);
    //     Ok(())
    // }
    // fn get_gain(&self) -> Gain {
    //     let locked_state = self.state.lock().unwrap();
    //     locked_state.camera_settings.gain
    // }

    // fn set_offset(&mut self, offset: Offset) -> Result<(), CanonicalError> {
    //     let mut locked_state = self.state.lock().unwrap();
    //     locked_state.camera_settings.offset = offset;
    //     ASICamera::changed_setting(&mut locked_state);
    //     Ok(())
    // }
    // fn get_offset(&self) -> Offset {
    //     let locked_state = self.state.lock().unwrap();
    //     locked_state.camera_settings.offset
    // }

    // fn set_update_interval(&mut self, update_interval: Duration)
    //                        -> Result<(), CanonicalError> {
    //     let mut locked_state = self.state.lock().unwrap();
    //     locked_state.update_interval = update_interval;
    //     Ok(())
    // }

    // async fn capture_image(&mut self, prev_frame_id: Option<i32>)
    //                        -> Result<(CapturedImage, i32), CanonicalError> {
    //     // Has the worker terminated for some reason?
    //     if self.video_capture_thread.is_some() &&
    //         self.video_capture_thread.as_ref().unwrap().is_finished()
    //     {
    //         self.video_capture_thread.take().unwrap().await.unwrap();
    //     }
    //     // Start video capture thread if terminated or not yet started.
    //     if self.video_capture_thread.is_none() {
    //         let min_gain = self.min_gain;
    //         let max_gain = self.max_gain;
    //         let cloned_state = self.state.clone();
    //         let cloned_sdk = self.asi_cam_sdk.clone();
    //         self.video_capture_thread = Some(tokio::task::spawn_blocking(move || {
    //             ASICamera::worker(
    //                 min_gain, max_gain, cloned_state, cloned_sdk);
    //         }));
    //     }
    //     // Get the most recently posted image; wait if there is none yet or the
    //     // currently posted image's frame id is the same as `prev_frame_id`.
    //     loop {
    //         let mut sleep_duration = Duration::from_millis(1);
    //         {
    //             let locked_state = self.state.lock().unwrap();
    //             if locked_state.most_recent_capture.is_some() &&
    //                 (prev_frame_id.is_none() ||
    //                  prev_frame_id.unwrap() != locked_state.frame_id)
    //             {
    //                 // Don't consume it, other clients may want it.
    //                 return Ok((locked_state.most_recent_capture.clone().unwrap(),
    //                            locked_state.frame_id));
    //             }
    //             if locked_state.eta.is_some() {
    //                 let time_to_eta =
    //                     locked_state.eta.unwrap().saturating_duration_since(Instant::now());
    //                 if time_to_eta > sleep_duration {
    //                     sleep_duration = time_to_eta;
    //                 }
    //             }
    //         }
    //         tokio::time::sleep(sleep_duration).await;
    //     }
    // }

    // fn estimate_delay(&self, prev_frame_id: Option<i32>) -> Option<Duration> {
    //     let locked_state = self.state.lock().unwrap();
    //     if locked_state.most_recent_capture.is_some() &&
    //         (prev_frame_id.is_none() ||
    //          prev_frame_id.unwrap() != locked_state.frame_id)
    //     {
    //         Some(Duration::ZERO)
    //     } else if locked_state.eta.is_some() {
    //         Some(locked_state.eta.unwrap().saturating_duration_since(Instant::now()))
    //     } else {
    //         None
    //     }
    // }

    // async fn stop(&mut self) {
    //     if self.video_capture_thread.is_some() {
    //         self.state.lock().unwrap().stop_request = true;
    //         self.video_capture_thread.take().unwrap().await.unwrap();
    //     }
    // }
// }
