use canonical_error::{CanonicalError, failed_precondition_error, unimplemented_error};

use async_trait::async_trait;
use image::GrayImage;
use log::{debug, info, warn};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant, SystemTime};

use libcamera::{
    camera::{CameraConfigurationStatus},
    camera_manager::CameraManager,
    control::ControlList,
    controls::{AeEnable, AnalogueGain, ExposureTime},
    framebuffer_allocator::{FrameBuffer, FrameBufferAllocator},
    framebuffer_map::MemoryMappedFrameBuffer,
    properties,
    request::{ReuseFlag},
    stream::StreamRole,
};

use crate::abstract_camera::{AbstractCamera, CaptureParams, CapturedImage,
                             EnumeratedCameraInfo, Gain, Offset};

pub struct RpiCamera {
    // Dimensions, in mm, of the sensor.
    sensor_width: f32,
    sensor_height: f32,

    is_color: bool,

    // Our state, shared between RpiCamera methods and the capture thread.
    state: Arc<Mutex<SharedState>>,

    // Our capture thread. Executes worker().
    capture_thread: Option<tokio::task::JoinHandle<()>>,
}

// State shared between capture thread and the RpiCamera methods.
struct SharedState {
    model: String,

    // Different models have different max analog gain. Min analog gain is always 1.
    max_gain: i32,

    width: usize,
    height: usize,

    // If neither is true, then it is 8 bits.
    is_10_bit: bool,
    is_12_bit: bool,

    is_packed: bool,

    // Whether image should be inverted, according to set_inverted() API.
    inverted: bool,

    // We use the convention that the Rpi camera should produce an upright image
    // when the module is rotated to have the connector at the bottom. Some modules
    // are inverted (rot180) with respect to this, so we correct during readout.
    correct_invert: bool,

    // Current camera settings as set via RpiCamera methods. Will be put into
    // effect when the current exposure finishes, influencing the following
    // exposures.
    camera_settings: CaptureParams,
    setting_changed: bool,

    // Zero means go fast as camera frames are available.
    update_interval: Duration,

    // Estimated time at which `most_recent_capture` will next be updated.
    eta: Option<Instant>,

    // Most recent completed capture and its id value.
    most_recent_capture: Option<CapturedImage>,
    frame_id: i32,

    // Set by stop(); the capture thread exits when it sees this.
    stop_request: bool,
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
    pub fn new(camera_index: i32) -> Result<Self, CanonicalError> {
        let mgr = CameraManager::new().unwrap();
        let cameras = mgr.cameras();
        let cam = cameras.get(camera_index as usize).unwrap();
        let props = cam.properties();
        let model = props.get::<properties::Model>().unwrap();
        let pixel_array_size = props.get::<properties::PixelArraySize>().unwrap();
        let width = pixel_array_size.width as usize;
        let height = pixel_array_size.height as usize;
        let pixel_size_nanometers = props.get::<properties::UnitCellSize>().unwrap();
        // This will generate default configuration for Raw.
        let mut cfgs = cam.generate_configuration(&[StreamRole::Raw]).unwrap();
        match cfgs.validate() {
            CameraConfigurationStatus::Valid => (),
            CameraConfigurationStatus::Adjusted => {
                debug!("Camera configuration was adjusted: {:#?}", cfgs);
            },
            CameraConfigurationStatus::Invalid => {
                return Err(failed_precondition_error(
                    format!("Camera configuration was rejected: {:#?}", cfgs).as_str()));
            },
        }
        let stream_config = cfgs.get(0).unwrap();
        let pixel_format = format!("{:?}", stream_config.get_pixel_format());
        let is_12_bit = pixel_format.ends_with("12_CSI2P") || pixel_format.ends_with("12");
        let is_10_bit = pixel_format.ends_with("10_CSI2P") || pixel_format.ends_with("10");
        let is_packed = pixel_format.ends_with("_CSI2P");
        let is_color = pixel_format.starts_with("S");

        // Annoyingly, different Rpi cameras have different max analog gain values.
        let max_gain = match model.as_str() {
            "imx477" => 22,
            "imx296" => 15,
            _ => 63,
        };
        debug!("max_gain {}", max_gain);

        // Unfortunately the libcamera API's Rotation property doesn't seem to correlate
        // with our connector-oriented definition. Figure it out based on the model.
        let correct_invert = match model.as_str() {
            "imx296" => true,
            _ => false,
        };
        debug!("correct_invert {}", correct_invert);

        let mut cam = RpiCamera{
            sensor_width: width as f32 * pixel_size_nanometers.width as f32 / 1000000.0,
            sensor_height: height as f32 * pixel_size_nanometers.height as f32 / 1000000.0,
            is_color,
            state: Arc::new(Mutex::new(SharedState{
                model: model.to_string(),
                max_gain,
                width, height,
                is_10_bit, is_12_bit, is_packed,
                inverted: false,
                correct_invert,
                camera_settings: CaptureParams::new(),
                setting_changed: false,
                update_interval: Duration::ZERO,
                eta: None,
                most_recent_capture: None,
                frame_id: 0,
                stop_request: false,
            })),
            capture_thread: None,
        };
        cam.set_gain(cam.optimal_gain())?;
        Ok(cam)
    }

    // This function is called whenever a camera setting is changed. The most
    // recently captured image is invalidated, and the next call to capture_image()
    // will wait for the setting change to be reflected in the captured image
    // stream.
    fn changed_setting(locked_state: &mut MutexGuard<SharedState>) {
        locked_state.setting_changed = true;
        locked_state.most_recent_capture = None;
    }

    fn setup_camera_request(controls: &mut ControlList, state: &MutexGuard<SharedState>) {
        let settings = &state.camera_settings;
        let exp_duration_micros = settings.exposure_duration.as_micros();
        let abstract_gain = settings.gain.value();
        controls.set(AeEnable(false)).unwrap();
        controls.set(ExposureTime(exp_duration_micros as i32)).unwrap();
        controls.set(AnalogueGain(Self::cam_gain(abstract_gain, state.max_gain))).unwrap();
    }

    // Convert 10 or 12 bit pixels to 8 bits, by keeping the upper 8 bits.
    //
    // About raw color images: Because CedarDetect works with monochrome data,
    // if the captured image is raw color (Bayer) we need to convert to
    // grayscale. We do so not by debayering (interpolating color into each
    // output pixel) but instead we IGNORE the bayer nature of the pixel grid
    // and just pass the pixel values through.
    // This has the following benefits:
    // * It is fast.
    // * It allows CedarDetect's hot pixel detection to work effectively.
    // * Star images are only weakly colored anyway.
    // * CedarDetect usually does 2x2 binning prior to running its star detection
    //   algorithm, so the R/G/B values are roughly converted to luma in the
    //   binning process.
    fn convert_to_8bit(stride: usize,
                       buf_data: &[u8],
                       image_data: &mut Vec<u8>,
                       width: usize,
                       height: usize,
                       inverted: bool,
                       correct_invert: bool,
                       is_10_bit: bool,
                       is_12_bit: bool,
                       is_packed: bool) {
        let inverted = inverted ^ correct_invert;
        if is_10_bit {
            if is_packed {
                // Convert from packed 10 bit to 8 bit.
                for row in 0..height {
                    let out_row = if inverted { height - row - 1 } else { row };

                    let buf_row_start = (row * stride) as usize;
                    let buf_row_end = buf_row_start + (width*5/4) as usize;
                    let pix_row_start = (out_row * width) as usize;
                    let pix_row_end = pix_row_start + width as usize;
                    if inverted {
                        for (buf_chunk, pix_chunk)
                            in buf_data[buf_row_start..buf_row_end].chunks_exact(5).zip(
                                image_data[pix_row_start..pix_row_end].rchunks_exact_mut(4))
                        {
                            // Keep upper 8 bits; discard 2 lsb.
                            let pix0 = buf_chunk[0];
                            let pix1 = buf_chunk[1];
                            let pix2 = buf_chunk[2];
                            let pix3 = buf_chunk[3];
                            // pix4 has the lsb values, which we discard.
                            pix_chunk[3] = pix0;
                            pix_chunk[2] = pix1;
                            pix_chunk[1] = pix2;
                            pix_chunk[0] = pix3;
                        }
                    } else {
                        for (buf_chunk, pix_chunk)
                            in buf_data[buf_row_start..buf_row_end].chunks_exact(5).zip(
                                image_data[pix_row_start..pix_row_end].chunks_exact_mut(4))
                        {
                            // Keep upper 8 bits; discard 2 lsb.
                            let pix0 = buf_chunk[0];
                            let pix1 = buf_chunk[1];
                            let pix2 = buf_chunk[2];
                            let pix3 = buf_chunk[3];
                            // pix4 has the lsb values, which we discard.
                            pix_chunk[0] = pix0;
                            pix_chunk[1] = pix1;
                            pix_chunk[2] = pix2;
                            pix_chunk[3] = pix3;
                        }
                    }
                }
            } else {
                panic!("Unpacked raw not yet supported");
            }
        } else if is_12_bit {
            if is_packed {
                // Convert from packed 12 bit to 8 bit.
                for row in 0..height {
                    let out_row = if inverted { height - row - 1 } else { row };

                    let buf_row_start = (row * stride) as usize;
                    let buf_row_end = buf_row_start + (width*3/2) as usize;
                    let pix_row_start = (out_row * width) as usize;
                    let pix_row_end = pix_row_start + width as usize;
                    if inverted {
                        for (buf_chunk, pix_pair)
                            in buf_data[buf_row_start..buf_row_end].chunks_exact(3).zip(
                                image_data[pix_row_start..pix_row_end].rchunks_exact_mut(2))
                        {
                            // Keep upper 8 bits; discard 4 lsb.
                            let pix0 = buf_chunk[0];
                            let pix1 = buf_chunk[1];
                            // pix2 has the lsb values, which we discard.
                            pix_pair[1] = pix0;
                            pix_pair[0] = pix1;
                        }
                    } else {
                        for (buf_chunk, pix_pair)
                            in buf_data[buf_row_start..buf_row_end].chunks_exact(3).zip(
                                image_data[pix_row_start..pix_row_end].chunks_exact_mut(2))
                        {
                            // Keep upper 8 bits; discard 4 lsb.
                            let pix0 = buf_chunk[0];
                            let pix1 = buf_chunk[1];
                            // pix2 has the lsb values, which we discard.
                            pix_pair[0] = pix0;
                            pix_pair[1] = pix1;
                        }
                    }
                }
            } else {
                panic!("Unpacked raw not yet supported");
            }
        } else {
            panic!("8 bit raw not yet supported");
        }
    }

    // The first call to capture_image() starts the capture thread that
    // executes this function.
    fn worker(state: Arc<Mutex<SharedState>>) {
        info!("Starting RpiCamera worker");

        // Whenever we change the camera settings, we will have discarded the
        // in-progress exposure, because the old settings were in effect when it
        // started. We need to discard a number of images after a setting change.
        let pipeline_depth = match state.lock().unwrap().model.as_str() {
            "imx477" => 6,
            "imx296" => 4,
            _ => 5,
        };

        let mut discard_image_count = 0;

        let mgr = CameraManager::new().unwrap();
        let cameras = mgr.cameras();
        let cam = cameras.get(0).expect("No cameras found");
        let mut active_cam = cam.acquire().expect("Unable to acquire camera");

        let mut cfgs = cam.generate_configuration(&[StreamRole::Raw]).unwrap();
        match cfgs.validate() {
            CameraConfigurationStatus::Invalid => {
                panic!("Camera configuration was rejected: {:#?}", cfgs);
            },
            _ => ()
        }
        active_cam.configure(&mut cfgs).expect("Unable to configure camera");
        let cfg = cfgs.get(0).unwrap();
        let stride = cfg.get_stride() as usize;

        // Completed capture requests are returned as a callback, which stuffs the
        // completed request into a channel.
        let (tx, rx) = std::sync::mpsc::channel();
        active_cam.on_request_completed(move |req| {
            tx.send(req).unwrap();
        });

        // Allocate frame buffers for the stream.
        let mut alloc = FrameBufferAllocator::new(&cam);
        let stream = cfg.stream().unwrap();
        let buffers = alloc.alloc(&stream).unwrap();
        // Convert FrameBuffer to MemoryMappedFrameBuffer, which allows reading &[u8].
        let buffers = buffers
            .into_iter()
            .map(|buf| MemoryMappedFrameBuffer::new(buf).unwrap())
            .collect::<Vec<_>>();

        let reqs;
        {
            let locked_state = state.lock().unwrap();
            // Create capture requests and attach buffers.
            reqs = buffers
                .into_iter()
                .map(|buf| {
                    let mut req = active_cam.create_request(None).unwrap();
                    req.add_buffer(&stream, buf).unwrap();
                    let controls = req.controls_mut();
                    Self::setup_camera_request(controls, &locked_state);
                    req
                })
                .collect::<Vec<_>>();
        }
        // Enqueue all requests to the camera.
        active_cam.start(None).unwrap();
        for req in reqs {
            active_cam.queue_request(req).unwrap();
        }
        info!("Starting capturing");

        // Keep track of when we grabbed a frame.
        let mut last_frame_time: Option<Instant> = None;
        loop {
            let update_interval: Duration;
            let exp_duration: Duration;
            {
                let mut locked_state = state.lock().unwrap();
                update_interval = locked_state.update_interval;
                if locked_state.stop_request {
                    info!("Stopping capture");
                    if let Err(e) = active_cam.stop() {
                        warn!("Error stopping capture: {}", &e.to_string());
                    }
                    locked_state.stop_request = false;
                    return;  // Exit thread.
                }
            }

            // Is it time to grab a frame?
            if let Some(lft) = last_frame_time {
                let next_update_time = lft + update_interval;
                let now = Instant::now();
                if next_update_time > now {
                    let delay = next_update_time - now;
                    state.lock().unwrap().eta = Some(now + delay);
                    std::thread::sleep(delay);
                    continue;
                }
                state.lock().unwrap().eta = None;
            }
            // Time to grab a frame.
            let mut req = match rx.recv_timeout(Duration::from_secs(5)) {
                Ok(req) => req,
                Err(e) => {
                    warn!("Camera request failed: {:?}", e);
                    continue;
                }
            };
            last_frame_time = Some(Instant::now());
            let mut do_queue = false;
            let width;
            let height;
            let inverted;
            let correct_invert;
            let is_10_bit;
            let is_12_bit;
            let is_packed;
            {
                let mut locked_state = state.lock().unwrap();
                width = locked_state.width;
                height = locked_state.height;
                inverted = locked_state.inverted;
                correct_invert = locked_state.correct_invert;
                is_10_bit = locked_state.is_10_bit;
                is_12_bit = locked_state.is_12_bit;
                is_packed = locked_state.is_packed;
                if locked_state.setting_changed {
                    discard_image_count = pipeline_depth;
                    locked_state.setting_changed = false;
                    req.reuse(ReuseFlag::REUSE_BUFFERS);
                    let controls = req.controls_mut();
                    Self::setup_camera_request(controls, &locked_state);
                    do_queue = true;
                } else if discard_image_count > 0 {
                    discard_image_count -= 1;
                    req.reuse(ReuseFlag::REUSE_BUFFERS);
                    let controls = req.controls_mut();
                    Self::setup_camera_request(controls, &locked_state);
                    do_queue = true;
                }
                exp_duration = locked_state.camera_settings.exposure_duration;
            }
            if do_queue {
                active_cam.queue_request(req).unwrap();
                continue;
            }

            // Grab the image.
            let framebuffer: &MemoryMappedFrameBuffer<FrameBuffer> =
                req.buffer(&stream).unwrap();
            // Raw format has only one data plane containing bayer or mono data.
            let planes = framebuffer.data();
            let buf_data = planes.get(0).unwrap();

            // Allocate uninitialized storage to receive the converted image data.
            let num_pixels = width * height;

            let mut image_data = Vec::<u8>::with_capacity(num_pixels as usize);
            unsafe { image_data.set_len(num_pixels as usize) }

            Self::convert_to_8bit(stride, &buf_data, &mut image_data,
                                  width, height, inverted, correct_invert,
                                  is_10_bit, is_12_bit, is_packed);
            let image = GrayImage::from_raw(
                width as u32, height as u32, image_data).unwrap();

            // Re-queue the request.
            req.reuse(ReuseFlag::REUSE_BUFFERS);
            let controls = req.controls_mut();
            let mut locked_state = state.lock().unwrap();
            Self::setup_camera_request(controls, &locked_state);
            active_cam.queue_request(req).unwrap();

            if update_interval == Duration::ZERO {
                locked_state.eta = Some(Instant::now() + exp_duration);
            }

            if !locked_state.setting_changed {
                locked_state.most_recent_capture = Some(CapturedImage {
                    capture_params: locked_state.camera_settings,
                    image: Arc::new(image),
                    readout_time: SystemTime::now(),
                });
                locked_state.frame_id += 1;
            }
        }  // loop.
    }  // worker().

    // Translate our 0..100 into the camera min/max range for analog gain.
    fn cam_gain(abstract_gain: i32, cam_max_gain: i32) -> f32 {
        let frac = abstract_gain as f64 / 100.0;
        (1.0 + (cam_max_gain as f64 - 1.0) * frac) as f32
    }

    async fn manage_worker_thread(&mut self) {
        // Has the worker terminated for some reason?
        if self.capture_thread.is_some() &&
            self.capture_thread.as_ref().unwrap().is_finished()
        {
            self.capture_thread.take().unwrap().await.unwrap();
        }
        // Start capture thread if terminated or not yet started.
        if self.capture_thread.is_none() {
            let cloned_state = self.state.clone();
            self.capture_thread = Some(tokio::task::spawn_blocking(move || {
                Self::worker(cloned_state);
            }));
        }
    }
}  // impl RpiCamera.

/// We arrange to call stop() when RpiCamera object goes out of scope.
impl Drop for RpiCamera {
    fn drop(&mut self) {
        // https://stackoverflow.com/questions/71541765/rust-async-drop
        futures::executor::block_on(self.stop());
    }
}

#[async_trait]
impl AbstractCamera for RpiCamera {
    fn model(&self) -> String {
        let locked_state = self.state.lock().unwrap();
        locked_state.model.clone()
    }

    fn dimensions(&self) -> (i32, i32) {
        let locked_state = self.state.lock().unwrap();
        (locked_state.width as i32, locked_state.height as i32)
    }

    fn is_color(&self) -> bool {
        self.is_color
    }

    fn sensor_size(&self) -> (f32, f32) {
        (self.sensor_width, self.sensor_height)
    }

    fn optimal_gain(&self) -> Gain {
        Gain::new(100)
    }

    fn set_exposure_duration(&mut self, exp_duration: Duration)
                             -> Result<(), CanonicalError> {
        let mut locked_state = self.state.lock().unwrap();
        if locked_state.camera_settings.exposure_duration != exp_duration {
            RpiCamera::changed_setting(&mut locked_state);
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
            RpiCamera::changed_setting(&mut locked_state);
        }
        locked_state.camera_settings.gain = gain;
        Ok(())
    }
    fn get_gain(&self) -> Gain {
        let locked_state = self.state.lock().unwrap();
        locked_state.camera_settings.gain
    }

    fn set_offset(&mut self, _offset: Offset) -> Result<(), CanonicalError> {
        Err(unimplemented_error(
            format!("Offset not supported for {}", self.model()).as_str()))
    }
    fn get_offset(&self) -> Offset {
        Offset::new(0)
    }

    fn set_inverted(&mut self, inverted: bool) -> Result<(), CanonicalError> {
        let mut locked_state = self.state.lock().unwrap();
        if locked_state.inverted != inverted {
            RpiCamera::changed_setting(&mut locked_state);
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
        self.manage_worker_thread().await;
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

    async fn try_capture_image(
        &mut self, prev_frame_id: Option<i32>)
        -> Result<Option<(CapturedImage, i32)>, CanonicalError>
    {
        self.manage_worker_thread().await;
        // Get the most recently posted image; return none if there is none yet
        // or the currently posted image's frame id is the same as
        // `prev_frame_id`.
        loop {
            let locked_state = self.state.lock().unwrap();
            if locked_state.most_recent_capture.is_some() &&
                (prev_frame_id.is_none() ||
                 prev_frame_id.unwrap() != locked_state.frame_id)
            {
                // Don't consume it, other clients may want it.
                return Ok(Some((locked_state.most_recent_capture.clone().unwrap(),
                                locked_state.frame_id)));
            }
            return Ok(None);
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
        if self.capture_thread.is_some() {
            self.state.lock().unwrap().stop_request = true;
            self.capture_thread.take().unwrap().await.unwrap();
        }
    }
}
