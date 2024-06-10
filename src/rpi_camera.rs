use canonical_error::{CanonicalError, failed_precondition_error, unimplemented_error};

use async_trait::async_trait;
use image::GrayImage;
use log::{debug, error, info, warn};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant, SystemTime};

use libcamera::{
    camera::{CameraConfigurationStatus},
    camera_manager::CameraManager,
    control::ControlList,
    controls::{AeEnable, AnalogueGain, ExposureTime, SensorTemperature},
    framebuffer_allocator::{FrameBuffer, FrameBufferAllocator},
    framebuffer_map::MemoryMappedFrameBuffer,
    properties,
    request::ReuseFlag,
    stream::StreamRole,
};

use crate::abstract_camera::{AbstractCamera, BinFactor, CaptureParams, CapturedImage,
                             Celsius, EnumeratedCameraInfo, Flip, Gain, Offset,
                             RegionOfInterest};

pub struct RpiCamera {
    model: String,

    // Dimensions, in mm, of the sensor.
    sensor_width: f32,
    sensor_height: f32,

    // Our state, shared between RpiCamera methods and the video capture thread.
    state: Arc<Mutex<SharedState>>,

    // Our video capture thread. Executes worker().
    video_capture_thread: Option<tokio::task::JoinHandle<()>>,
}

// State shared between video capture thread and the RpiCamera methods.
struct SharedState {
    // Different models have different max analog gain. Min analog gain is always 1.
    max_gain: i32,

    width: usize,
    height: usize,

    // If neither is true, then it is 8 bits.
    is_10_bit: bool,
    is_12_bit: bool,

    is_packed: bool,

    // is_color: bool,  // TODO: drop this?
    first_pixel_green: bool,
    // Amount by which we scale red/blue pixel sites to roughly equalize with green.
    // Scale factor is scale_rb_num/2.
    scale_rb_num: u16,

    // Current camera settings as set via RpiCamera methods. Will be put into
    // effect when the current exposure finishes, influencing the following
    // exposures.
    camera_settings: CaptureParams,
    setting_changed: bool,

    // Zero means go fast as camerea frames are available.
    update_interval: Duration,

    // Estimated time at which `most_recent_capture` will next be updated.
    eta: Option<Instant>,

    // Most recent completed capture and its id value.
    most_recent_capture: Option<CapturedImage>,
    frame_id: i32,

    // Set by stop(); the video capture thread exits when it sees this.
    stop_request: bool,

    // Camera settings in effect when the in-progress capture started.
    current_capture_settings: CaptureParams,  // TODO: drop this
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
                warn!("Camera configuration was adjusted: {:#?}", cfgs);
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
        // let is_color = pixel_format.starts_with("S");
        let first_pixel_green = pixel_format.starts_with("SG");

        // For HQ, Empirically it seems that blue and red pixels generally have
        // half intensity of green pixels for white stimulus.
        // Use 4/2 or 3/2 scaling to bring red/blue roughly to parity with green.
        let scale_rb_num = if model.as_str() == "imx477" { 4 } else { 3 };

        // TODO: revise for other camera types.
        let max_gain = if model.as_str() == "imx477" { 22 } else { 63 };

        Ok(RpiCamera{model: model.to_string(),
                     sensor_width: width as f32 * pixel_size_nanometers.width as f32 / 1000000.0,
                     sensor_height: height as f32 * pixel_size_nanometers.height as f32 / 1000000.0,
                     state: Arc::new(Mutex::new(SharedState{
                         max_gain,
                         width, height,
                         is_10_bit, is_12_bit,
                         is_packed,
                         // is_color,
                         first_pixel_green,
                         scale_rb_num,
                         camera_settings: CaptureParams::new(),
                         setting_changed: false,
                         update_interval: Duration::ZERO,
                         eta: None,
                         most_recent_capture: None,
                         frame_id: 0,
                         stop_request: false,
                         current_capture_settings: CaptureParams::new(),
                     })),
                     video_capture_thread: None,
        })
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

    fn convert_to_8bit(stride: usize,
                       buf_data: &[u8],
                       image_data: &mut Vec<u8>,
                       state: &MutexGuard<SharedState>) {
        let scale_rb_numerator = state.scale_rb_num;
        if state.is_10_bit {
            if state.is_packed {
                // Convert from packed 10 bit to 8 bit.
                for row in 0..state.height {
                    let green_phase =
                        if state.first_pixel_green { (row & 1) == 0 } else { (row & 1) != 0 };
                    let buf_row_start = (row * stride) as usize;
                    let buf_row_end = buf_row_start + (state.width*5/4) as usize;
                    let pix_row_start = (row * state.width) as usize;
                    let pix_row_end = pix_row_start + state.width as usize;
                    for (buf_chunk, pix_chunk)
                        in buf_data[buf_row_start..buf_row_end].chunks_exact(5).zip(
                            image_data[pix_row_start..pix_row_end].chunks_exact_mut(4))
                    {
                        // Discard 2 lsb.
                        let mut pix0 = buf_chunk[0];
                        let mut pix1 = buf_chunk[1];
                        let mut pix2 = buf_chunk[2];
                        let mut pix3 = buf_chunk[3];
                        if green_phase {
                            pix1 = if pix1 > 170 { 255 } else { ((scale_rb_numerator * pix1 as u16) / 2) as u8 };
                            pix3 = if pix3 > 170 { 255 } else { ((scale_rb_numerator * pix3 as u16) / 2) as u8 };
                        } else {
                            pix0 = if pix0 > 170 { 255 } else { ((scale_rb_numerator * pix0 as u16) / 2) as u8 };
                            pix2 = if pix2 > 170 { 255 } else { ((scale_rb_numerator * pix2 as u16) / 2) as u8 };
                        }
                        pix_chunk[0] = pix0;
                        pix_chunk[1] = pix1;
                        pix_chunk[2] = pix2;
                        pix_chunk[3] = pix3;
                    }
                }
            } else {
                // TODO: deal with unpacked.
            }
        } else if state.is_12_bit {
            if state.is_packed {
                // Convert from packed 12 bit to 8 bit.
                for row in 0..state.height {
                    let green_phase =
                        if state.first_pixel_green { (row & 1) == 0 } else { (row & 1) != 0 };
                    let buf_row_start = (row * stride) as usize;
                    let buf_row_end = buf_row_start + (state.width*3/2) as usize;
                    let pix_row_start = (row * state.width) as usize;
                    let pix_row_end = pix_row_start + state.width as usize;
                    for (buf_chunk, pix_pair)
                        in buf_data[buf_row_start..buf_row_end].chunks_exact(3).zip(
                            image_data[pix_row_start..pix_row_end].chunks_exact_mut(2))
                    {
                        // Discard 4 lsb.
                        let mut pix0 = buf_chunk[0];
                        let mut pix1 = buf_chunk[1];
                        if green_phase {
                            pix1 = if pix1 > 127 { 255 } else { ((scale_rb_numerator * pix1 as u16) / 2) as u8 };
                        } else {
                            pix0 = if pix0 > 127 { 255 } else { ((scale_rb_numerator * pix0 as u16) / 2) as u8 };
                        }
                        pix_pair[0] = pix0;
                        pix_pair[1] = pix1;
                    }
                }
            } else {
                // TODO: deal with unpacked.
            }
        } else {
            // TODO: 8 bits.
        }
    }

    // The first call to capture_image() starts the video capture thread that
    // executes this function.
    fn worker(state: Arc<Mutex<SharedState>>) {
        info!("Starting RpiCamera worker");

        // Whenever we change the camera settings, we will have discarded the
        // in-progress exposure, because the old settings were in effect when it
        // started. We need to discard a few images after a setting change.
        let pipeline_depth = 5;
        let mut discard_image_count = 0;

        let mgr = CameraManager::new().unwrap();
        let cameras = mgr.cameras();
        let cam = cameras.get(0).expect("No cameras found");
        let mut active_cam = cam.acquire().expect("Unable to acquire camera");

        let mut cfgs = cam.generate_configuration(&[StreamRole::Raw]).unwrap();
        match cfgs.validate() {
            CameraConfigurationStatus::Invalid => {
                error!("Camera configuration was rejected: {:#?}", cfgs);
                return;  // Abandon thread execution!
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

        {
            let mut locked_state = state.lock().unwrap();
            // Create capture requests and attach buffers.
            let reqs = buffers
                .into_iter()
                .map(|buf| {
                    let mut req = active_cam.create_request(None).unwrap();
                    req.add_buffer(&stream, buf).unwrap();
                    let controls = req.controls_mut();
                    Self::setup_camera_request(controls, &locked_state);
                    req
                })
                .collect::<Vec<_>>();

            // Enqueue all requests to the camera.
            active_cam.start(None).unwrap();
            for req in reqs {
                active_cam.queue_request(req).unwrap();
            }
            locked_state.current_capture_settings = locked_state.camera_settings;
            info!("Starting capturing");
        }  // locked_state

        // Keep track of when we grabbed a frame.
        let mut last_frame_time: Option<Instant> = None;
        loop {
            let update_interval: Duration;
            let exp_duration: Duration;
            {
                let mut locked_state = state.lock().unwrap();
                update_interval = locked_state.update_interval;
                if locked_state.stop_request {
                    info!("Stopping video capture");
                    if let Err(e) = active_cam.stop() {
                        warn!("Error stopping capture: {}", &e.to_string());
                    }
                    locked_state.stop_request = false;
                    return;  // Exit thread.
                }
            }

            // Is it time to grab a frame?
            let now = Instant::now();
            if let Some(lft) = last_frame_time {
                let next_update_time = lft + update_interval;
                if next_update_time > now {
                    let delay = next_update_time - now;
                    state.lock().unwrap().eta = Some(Instant::now() + delay);
                    std::thread::sleep(delay);
                    continue;
                }
                state.lock().unwrap().eta = None;
            }
            // Time to grab a frame.
            let mut req = rx.recv_timeout(Duration::from_secs(5)).expect("Camera request failed");
            last_frame_time = Some(Instant::now());
            {
                let mut locked_state = state.lock().unwrap();
                if locked_state.setting_changed {
                    // active_cam.stop().unwrap();
                    // active_cam.start(None).unwrap();
                    discard_image_count = pipeline_depth;
                    locked_state.setting_changed = false;
                    req.reuse(ReuseFlag::REUSE_BUFFERS);
                    let controls = req.controls_mut();
                    Self::setup_camera_request(controls, &locked_state);
                    active_cam.queue_request(req).unwrap();
                    continue;
                }
                if discard_image_count > 0 {
                    discard_image_count -= 1;
                    req.reuse(ReuseFlag::REUSE_BUFFERS);
                    // let controls = req.controls_mut();
                    // Self::setup_camera_request(controls, &locked_state);
                    active_cam.queue_request(req).unwrap();
                    continue;
                }
                exp_duration = locked_state.camera_settings.exposure_duration;

                // Grab the image.
                let framebuffer: &MemoryMappedFrameBuffer<FrameBuffer> = req.buffer(&stream).unwrap();
                // Raw format has only one data plane containing bayer or mono data.
                let planes = framebuffer.data();
                let buf_data = planes.get(0).unwrap();

                // Allocate uninitialized storage to receive the converted image data.
                let num_pixels = locked_state.width * locked_state.height;
                let mut image_data = Vec::<u8>::with_capacity(num_pixels as usize);
                unsafe { image_data.set_len(num_pixels as usize) }

                Self::convert_to_8bit(stride, &buf_data, &mut image_data, &locked_state);
                let image = GrayImage::from_raw(
                    locked_state.width as u32, locked_state.height as u32, image_data).unwrap();

                let metadata = req.metadata();
                let temperature = Celsius(metadata.get::<SensorTemperature>().unwrap().0 as i32);

                // Re-queue the request.
                req.reuse(ReuseFlag::REUSE_BUFFERS);
                active_cam.queue_request(req).unwrap();

                if update_interval == Duration::ZERO {
                    locked_state.eta = Some(Instant::now() + exp_duration);
                }

                locked_state.most_recent_capture = Some(CapturedImage {
                    capture_params: locked_state.camera_settings,
                    image: Arc::new(image),
                    readout_time: SystemTime::now(),
                    temperature,
                });
                locked_state.frame_id += 1;
            }
        }  // loop.
    }  // worker().

    // Translate our 0..100 into the camera min/max range for analog gain.
    fn cam_gain(abstract_gain: i32, cam_max_gain: i32) -> f32 {
        // Translate our 0..100 into the camera's min/max range.
        let frac = abstract_gain as f64 / 100.0;
        (1.0 + (cam_max_gain as f64 - 1.0) * frac) as f32
    }
}

/// We arrange to call stop() when RpiCamera object goes out of scope.
impl Drop for RpiCamera {
    fn drop(&mut self) {
        // https://stackoverflow.com/questions/71541765/rust-async-drop
        futures::executor::block_on(self.stop());
    }
}

#[async_trait]
impl AbstractCamera for RpiCamera {
    fn model(&self) -> Result<String, CanonicalError> {
        Ok(self.model.clone())
    }

    fn dimensions(&self) -> (i32, i32) {
        let locked_state = self.state.lock().unwrap();
        (locked_state.width as i32, locked_state.height as i32)
    }

    fn sensor_size(&self) -> (f32, f32) {
        (self.sensor_width, self.sensor_height)
    }

    fn optimal_gain(&self) -> Gain {
        // For Rpi cameras, we use the maximum analog gain as the optimum gain value.
        // We don't use digital gain.
        if self.model.as_str() == "imx477" {
            // See: https://www.strollswithmydog.com/pi-hq-cam-sensor-performance/
            Gain::new(20)  // Corresponds to analog gain 4.
        } else {
            // TODO: revise this, use a per-model optimal gain value.
            Gain::new(50)
        }
    }

    fn set_flip_mode(&mut self, _flip_mode: Flip) -> Result<(), CanonicalError> {
        Err(unimplemented_error(
            format!("Flip mode not supported for {}", self.model().unwrap()).as_str()))
    }
    fn get_flip_mode(&self) -> Flip {
        Flip::None
    }

    fn set_exposure_duration(&mut self, exp_duration: Duration)
                             -> Result<(), CanonicalError> {
        let mut locked_state = self.state.lock().unwrap();
        locked_state.camera_settings.exposure_duration = exp_duration;
        RpiCamera::changed_setting(&mut locked_state);
        Ok(())
    }
    fn get_exposure_duration(&self) -> Duration {
        let locked_state = self.state.lock().unwrap();
        locked_state.camera_settings.exposure_duration
    }

    fn set_region_of_interest(&mut self, _roi: RegionOfInterest)
                              -> Result<RegionOfInterest, CanonicalError> {
        Err(unimplemented_error(
            format!("Region of interest not supported for {}", self.model().unwrap()).as_str()))
    }
    fn get_region_of_interest(&self) -> RegionOfInterest {
        let locked_state = self.state.lock().unwrap();
        RegionOfInterest{binning: BinFactor::X1,
                         capture_startpos: (0, 0),
                         capture_dimensions: (locked_state.width as i32,
                                              locked_state.height as i32)}
    }

    fn set_gain(&mut self, gain: Gain) -> Result<(), CanonicalError> {
        let mut locked_state = self.state.lock().unwrap();
        locked_state.camera_settings.gain = gain;
        RpiCamera::changed_setting(&mut locked_state);
        Ok(())
    }
    fn get_gain(&self) -> Gain {
        let locked_state = self.state.lock().unwrap();
        locked_state.camera_settings.gain
    }

    fn set_offset(&mut self, _offset: Offset) -> Result<(), CanonicalError> {
        Err(unimplemented_error(
            format!("Offset not supported for {}", self.model().unwrap()).as_str()))
    }
    fn get_offset(&self) -> Offset {
        Offset::new(0)
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
            let cloned_state = self.state.clone();
            self.video_capture_thread = Some(tokio::task::spawn_blocking(move || {
                RpiCamera::worker(cloned_state);
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
