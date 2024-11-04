// Fake camera that yields a fixed image. For testing.

use std::cmp;
use std::time::{Duration, Instant, SystemTime};
use std::ops::Deref;
use std::sync::Arc;

use async_trait::async_trait;
use canonical_error::{CanonicalError};
use image::GrayImage;
use image::imageops::rotate180;

use crate::abstract_camera::{AbstractCamera, CaptureParams, CapturedImage,
                             Celsius, Gain, Offset};

pub struct ImageCamera {
    image: Arc<GrayImage>,
    exposure_duration: Duration,

    offset: Offset,
    gain: Gain,
    inverted: bool,

    // Zero means go fast as camera frames are available.
    update_interval: Duration,

    // Most recent completed capture and its id value.
    most_recent_capture: Option<CapturedImage>,
    frame_id: i32,
    last_frame_time: Instant,
}

impl ImageCamera {
    pub fn new(image: GrayImage) -> Result<Self, CanonicalError> {
        Ok(ImageCamera{image: Arc::new(image),
                       exposure_duration: Duration::from_millis(100),
                       offset: Offset::new(3),
                       gain: Gain::new(50),
                       inverted: false,
                       update_interval: Duration::ZERO,
                       most_recent_capture: None,
                       frame_id: 0,
                       last_frame_time: Instant::now(),})
    }

    fn capture_image(&mut self) {
        let mut image = self.image.deref().clone();
        if self.inverted {
            image = rotate180(&image);
        }
        self.most_recent_capture = Some(CapturedImage {
            capture_params: CaptureParams {
                exposure_duration: self.get_exposure_duration(),
                gain: self.get_gain(),
                offset: self.get_offset(),
            },
            image: Arc::new(image),
            readout_time: SystemTime::now(),
            temperature: Celsius(20),
        });
        self.frame_id += 1;
        self.last_frame_time = Instant::now();
    }
}

#[async_trait]
impl AbstractCamera for ImageCamera {
    fn model(&self) -> String {
        "ImageCamera".to_string()
    }

    fn dimensions(&self) -> (i32, i32) {
        (self.image.dimensions().0 as i32, self.image.dimensions().1 as i32)
    }

    fn is_color(&self) -> bool {
        false
    }

    fn sensor_size(&self) -> (f32, f32) {
        (4.8, 3.6)
    }

    fn optimal_gain(&self) -> Gain {
        Gain::new(50)
    }

    fn set_exposure_duration(&mut self, _exp_duration: Duration)
                             -> Result<(), CanonicalError> {
        Ok(())  // Quietly ignore.
    }
    fn get_exposure_duration(&self) -> Duration {
        self.exposure_duration
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

    fn set_inverted(&mut self, inverted: bool) -> Result<(), CanonicalError> {
        self.inverted = inverted;
        Ok(())
    }
    fn get_inverted(&self) -> bool {
        self.inverted
    }

    fn set_update_interval(&mut self, update_interval: Duration)
                           -> Result<(), CanonicalError> {
        self.update_interval = update_interval;
        Ok(())
    }

    async fn capture_image(&mut self, prev_frame_id: Option<i32>)
                           -> Result<(CapturedImage, i32), CanonicalError> {
        let need_new_image =
            prev_frame_id.is_some() && prev_frame_id.unwrap() == self.frame_id;
        let interval = cmp::max(self.exposure_duration, self.update_interval);
        let next_frame_time = self.last_frame_time + interval;
        let sleep_interval = next_frame_time.saturating_duration_since(Instant::now());
        if sleep_interval == Duration::ZERO {
            self.capture_image();
        } else if need_new_image {
            tokio::time::sleep(sleep_interval).await;
            self.capture_image();
        }
        Ok((self.most_recent_capture.clone().unwrap(), self.frame_id))
    }

    async fn try_capture_image(
        &mut self, prev_frame_id: Option<i32>)
        -> Result<Option<(CapturedImage, i32)>, CanonicalError>
    {
        let need_new_image =
            prev_frame_id.is_some() && prev_frame_id.unwrap() == self.frame_id;
        let interval = cmp::max(self.exposure_duration, self.update_interval);
        let next_frame_time = self.last_frame_time + interval;
        let sleep_interval = next_frame_time.saturating_duration_since(Instant::now());
        if sleep_interval == Duration::ZERO {
            self.capture_image();
        } else if need_new_image {
            return Ok(None);
        }
        Ok(Some((self.most_recent_capture.clone().unwrap(), self.frame_id)))
    }

    fn estimate_delay(&self, prev_frame_id: Option<i32>) -> Option<Duration> {
        if prev_frame_id.is_some() && prev_frame_id.unwrap() == self.frame_id {
            let interval = cmp::max(self.exposure_duration, self.update_interval);
            let next_frame_time = self.last_frame_time + interval;
            let now = Instant::now();
            if now >= next_frame_time {
                return Some(Duration::ZERO);
            }
            Some(next_frame_time - now)
        } else {
            Some(Duration::ZERO)
        }
    }

    async fn stop(&mut self) {}
}
