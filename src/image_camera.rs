// Fake camera that yields a fixed image. For testing.

use std::time::{Duration, SystemTime};
use std::sync::Arc;

use async_trait::async_trait;
use canonical_error::{CanonicalError, unimplemented_error};
use image::GrayImage;

use crate::abstract_camera::{AbstractCamera, BinFactor, CaptureParams, CapturedImage,
                             Celsius, Flip, Gain, Offset, RegionOfInterest};

pub struct ImageCamera {
    image: Arc<GrayImage>,
    exposure_duration: Duration,

    offset: Offset,
    gain: Gain,

    // Most recent completed capture and its id value.
    most_recent_capture: Option<CapturedImage>,
    frame_id: i32,
}

impl ImageCamera {
    pub fn new(image: GrayImage) -> Result<Self, CanonicalError> {
        Ok(ImageCamera{image: Arc::new(image),
                       exposure_duration: Duration::from_millis(100),
                       offset: Offset::new(3),
                       gain: Gain::new(50),
                       most_recent_capture: None,
                       frame_id: 0,})
    }
}

#[async_trait]
impl AbstractCamera for ImageCamera {
    fn model(&self) -> Result<String, CanonicalError> {
        Ok("ImageCamera".to_string())
    }

    fn dimensions(&self) -> (i32, i32) {
        (self.image.dimensions().0 as i32, self.image.dimensions().1 as i32)
    }

    fn sensor_size(&self) -> (f32, f32) {
        (4.8, 3.6)
    }

    fn optimal_gain(&self) -> Gain {
        Gain::new(50)
    }

    fn set_flip_mode(&mut self, _flip_mode: Flip) -> Result<(), CanonicalError> {
        Err(unimplemented_error("set_flip_mode not implemented"))
    }
    fn get_flip_mode(&self) -> Flip {
        Flip::None
    }

    fn set_exposure_duration(&mut self, _exp_duration: Duration)
                             -> Result<(), CanonicalError> {
        Ok(())  // Quietly ignore.
    }
    fn get_exposure_duration(&self) -> Duration {
        self.exposure_duration
    }

    fn set_region_of_interest(&mut self, _roi: RegionOfInterest)
                              -> Result<RegionOfInterest, CanonicalError> {
        Err(unimplemented_error("set_region_of_interest not implemented"))
    }
    fn get_region_of_interest(&self) -> RegionOfInterest {
        RegionOfInterest{binning: BinFactor::X1,
                         capture_startpos: (0, 0),
                         capture_dimensions: self.dimensions()}
    }

    fn set_gain(&mut self, gain: Gain) -> Result<(), CanonicalError> {
        self.gain = gain;
        Ok(())
    }
    fn get_gain(&self) -> Gain {
        self.gain
    }

    fn set_offset(&mut self, offset: Offset) -> Result<(), CanonicalError> {
        self.offset = offset;
        Ok(())
    }
    fn get_offset(&self) -> Offset {
        self.offset
    }

    fn set_update_interval(&mut self, _update_interval: Duration)
                           -> Result<(), CanonicalError> {
        Ok(())
    }

    async fn capture_image(&mut self, prev_frame_id: Option<i32>)
                           -> Result<(CapturedImage, i32), CanonicalError> {
        if prev_frame_id.is_some() && prev_frame_id.unwrap() == self.frame_id {
            tokio::time::sleep(self.exposure_duration).await;
            self.frame_id += 1;
            self.most_recent_capture = None;
        }
        if self.most_recent_capture.is_none() {
            self.most_recent_capture = Some(CapturedImage {
                capture_params: CaptureParams {
                    flip: Flip::None,
                    exposure_duration: self.get_exposure_duration(),
                    roi: self.get_region_of_interest(),
                    gain: self.get_gain(),
                    offset: self.get_offset(),
                },
                image: self.image.clone(),
                readout_time: SystemTime::now(),
                temperature: Celsius(20),
            });
        }
        Ok((self.most_recent_capture.clone().unwrap(), self.frame_id))
    }

    fn estimate_delay(&self, prev_frame_id: Option<i32>) -> Option<Duration> {
        if prev_frame_id.is_some() && prev_frame_id.unwrap() == self.frame_id {
            Some(self.exposure_duration)
        } else {
            Some(Duration::ZERO)
        }
    }

    async fn stop(&mut self) {}
}
