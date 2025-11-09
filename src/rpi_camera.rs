// Copyright (c) 2025 Steven Rosenthal smr@dt3.org
// See LICENSE file in root directory for license terms.

use canonical_error::{CanonicalError, failed_precondition_error, unimplemented_error};

use async_trait::async_trait;

use image::GrayImage;
use log::{debug, info, warn};
use std::fs;
use std::process::Command;
use std::sync::{Arc, OnceLock, Mutex, MutexGuard};
use std::time::{Duration, Instant, SystemTime};

use libcamera::{
    camera::{Camera, CameraConfiguration, CameraConfigurationStatus},
    camera_manager::CameraManager,
    control::ControlList,
    controls::{AnalogueGain, AeEnable, AwbEnable,
               ExposureTime, NoiseReductionMode},
    framebuffer_allocator::{FrameBuffer, FrameBufferAllocator},
    framebuffer_map::MemoryMappedFrameBuffer,
    geometry,
    logging::{log_set_target, LoggingTarget},
    pixel_format::PixelFormat,
    properties,
    request::{ReuseFlag},
    stream::StreamRole,
};

use crate::abstract_camera::{AbstractCamera, CaptureParams, CapturedImage,
                             EnumeratedCameraInfo, Gain, Offset};
use crate::pisp_compression;

pub type ConvertFn = fn(
    stride: usize,
    buf_data: &[u8],
    image_data: &mut [u8],
    width: usize,
    height: usize,
    is_10_bit: bool,
    is_12_bit: bool,
    is_packed: bool,
);
static CONVERT_FN: OnceLock<ConvertFn> = OnceLock::new();
pub fn set_converter(func: ConvertFn) {
    log::info!("Setting raw image converter function.");
    let _ = CONVERT_FN.set(func);  // Ignores error if already set.
}

pub struct RpiCamera {
    // Dimensions, in mm, of the sensor.
    sensor_width: f32,
    sensor_height: f32,

    is_color: bool,

    // Our state, shared between RpiCamera methods and the capture thread.
    state: Arc<Mutex<SharedState>>,

    // Our capture thread. Executes worker().
    capture_thread: Option<std::thread::JoinHandle<()>>,
}

// State shared between capture thread and the RpiCamera methods.
struct SharedState {
    model: String,
    model_detail: Option<String>,  // dtoverlay params, if any.

    // Different models have different min/max analog gain.
    min_gain: i32,
    max_gain: i32,

    width: usize,
    height: usize,

    // Raspberry Pi 5 and later use a PiSP compressed raw mode.
    pisp_compressed: bool,
    pisp_compression_mode: i32,

    // The next three fields are ignored if PiSP compression (previous field) is used.
    // If neither is true, then it is 8 bits.
    is_10_bit: bool,
    is_12_bit: bool,

    is_packed: bool,

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

    // HCG (High Conversion Gain) mode state for IMX290 sensors.
    hcg_enabled: bool,
    hcg_needs_reinit: bool,

    // Pipeline quiescing state for safe HCG changes.
    hcg_draining_pipeline: bool,
    in_flight_request_count: usize,
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
            let cfgs = Self::get_camera_configs(&cam).unwrap();
            let stream_config = cfgs.get(0).unwrap();
            answer.push(EnumeratedCameraInfo{
                model: model.to_string(),
                width: stream_config.get_size().width,
                height: stream_config.get_size().height})
        }
        answer
    }

    // Returns a RpiCamera instance that implements the AbstractCamera API.
    // `camera_index` is w.r.t. the enumerate_cameras() vector length.
    pub fn new(camera_index: i32) -> Result<Self, CanonicalError> {
        let mgr = CameraManager::new().unwrap();
        let cameras = mgr.cameras();
        let cam = cameras.get(camera_index as usize).unwrap();
        let cfgs = Self::get_camera_configs(&cam)?;
        let stream_config = cfgs.get(0).unwrap();
        let props = cam.properties();
        let model = props.get::<properties::Model>().unwrap();
        let width = stream_config.get_size().width as usize;
        let height = stream_config.get_size().height as usize;
        let pixel_size_nanometers = match props.get::<properties::UnitCellSize>() {
            Ok(psn) => psn,
            Err(e) => {
                // Some sensors are missing this static property.
                if model.as_str() == "ov9281" {
                    properties::UnitCellSize(geometry::Size{width: 3000,
                                                            height: 3000})
                } else {
                    return Err(failed_precondition_error(
                        format!("Missing properties::UnitCellSize: {:?}",
                                e).as_str()));
                }
            },
        };
        let pixel_format = format!("{:?}", stream_config.get_pixel_format());

        let model_detail = match Self::extract_dtoverlay_config(
            "/boot/firmware/config.txt", model.as_str()) {
            Ok(md) => md,
            Err(e) => {
                warn!("Error examining config.txt: {:?}", e);
                None
            },
        };

        // See https://git.libcamera.org/libcamera/libcamera.git/tree/src/libcamera/formats.cpp
        let pisp_compressed = pixel_format.contains("PISP_COMP");
        let pisp_compression_mode =
            if pisp_compressed {
                match pixel_format.chars().last().unwrap() {
                    '1' => 1,
                    '2' => 2,
                    '3' => 3,
                    _ => panic!("Unexpected PiSP compression mode: {}", pixel_format),
                }
            } else {
                0  // Don't care.
            };
        let is_12_bit = pixel_format.ends_with("12_CSI2P") || pixel_format.ends_with("12");
        let is_10_bit = pixel_format.ends_with("10_CSI2P") || pixel_format.ends_with("10");
        let is_packed = pixel_format.ends_with("_CSI2P");
        let is_color =
            if pisp_compressed {
                !pixel_format.contains("MONO") && !pixel_format.contains("GREY") &&
                !pixel_format.contains("Y")
            } else {
                // Bayer patterns (color): SRGGB, SGRBG, SGBRG, SBGGR.
                // Monochrome patterns: Y8, Y10, Y12, GREY, MONO.
                pixel_format.starts_with("S") && (
                    pixel_format.contains("RGGB") ||
                    pixel_format.contains("GRBG") ||
                    pixel_format.contains("GBRG") ||
                    pixel_format.contains("BGGR")
                )
            };

        // Annoyingly, different Rpi cameras have different analog gain values.
        let min_gain = match model.as_str() {
            "imx290" => 5,  // AKA imx462.
            _ => 1,
        };
        debug!("min_gain {}", min_gain);
        let max_gain = match model.as_str() {
            "imx477" => 22,
            "imx219" => 16,
            "imx290" => 31,  // AKA imx462.
            "imx296" => 15,
            "ov5647" => 63,
            _ => 63,
        };
        debug!("max_gain {}", max_gain);

        let mut cam = RpiCamera{
            sensor_width: width as f32 * pixel_size_nanometers.width as f32 / 1000000.0,
            sensor_height: height as f32 * pixel_size_nanometers.height as f32 / 1000000.0,
            is_color,
            state: Arc::new(Mutex::new(SharedState{
                model: model.to_string(),
                model_detail,
                min_gain, max_gain,
                width, height,
                pisp_compressed, pisp_compression_mode,
                is_10_bit, is_12_bit, is_packed,
                camera_settings: CaptureParams::new(),
                setting_changed: false,
                update_interval: Duration::ZERO,
                eta: None,
                most_recent_capture: None,
                frame_id: 0,
                stop_request: false,
                hcg_enabled: false,
                hcg_needs_reinit: false,
                hcg_draining_pipeline: false,
                in_flight_request_count: 0,
            })),
            capture_thread: None,
        };
        cam.set_gain(cam.optimal_gain())?;
        Ok(cam)
    }

    fn get_camera_configs(cam: &Camera)
                          -> Result<CameraConfiguration, CanonicalError>
    {
        // This will generate default configuration for Raw.
        let mut cfgs = cam.generate_configuration(&[StreamRole::Raw]).unwrap();
        let stream_config = cfgs.get(0).unwrap();
        let original_format = stream_config.get_pixel_format();

        // Collect all available formats and look for optimal format
        // (8-bit > 10-bit > 12-bit).
        let formats = stream_config.formats();
        let all_formats: Vec<PixelFormat> =
            formats.pixel_formats().into_iter().collect();
        let mut preferred_format: Option<PixelFormat> = None;
        let mut format_type = "";

        // First pass: look for 8-bit format (most efficient).
        for format in &all_formats {
            let pix_fmt = format!("{:?}", format);
            if pix_fmt.contains("8") &&
               (pix_fmt.contains("RGGB") || pix_fmt.contains("GREY") ||
                pix_fmt.contains("Y8") || pix_fmt.contains("8_CSI2P"))
            {
                preferred_format = Some(*format);
                format_type = "8-bit";
                break;
            }
        }
        // Second pass: look for 10-bit format if no 8-bit found.
        if preferred_format.is_none() {
            for format in &all_formats {
                let pix_fmt = format!("{:?}", format);
                if pix_fmt.contains("10") &&
                   (pix_fmt.contains("RGGB") || pix_fmt.contains("GREY") ||
                    pix_fmt.contains("Y10") || pix_fmt.contains("10_CSI2P"))
                {
                    preferred_format = Some(*format);
                    format_type = "10-bit";
                    break;
                }
            }
        }

        // Try to set preferred format if available.
        if let Some(target_format) = preferred_format {
            cfgs.get_mut(0).unwrap().set_pixel_format(target_format);
            // Check the result of attempting to set the pixel format.
            match cfgs.validate() {
                CameraConfigurationStatus::Valid => {
                    // Success - no logging needed in normal case.
                },
                CameraConfigurationStatus::Adjusted => {
                    debug!("Camera configuration was adjusted: {:#?}", cfgs);
                },
                CameraConfigurationStatus::Invalid => {
                    // Fallback to original format if preferred format not
                    // supported.
                    warn!("{} format rejected, listing available formats:",
                          format_type);
                    for format in &all_formats {
                        warn!("  {:?}", format);
                    }
                    cfgs.get_mut(0).unwrap().set_pixel_format(original_format);
                    match cfgs.validate() {
                        CameraConfigurationStatus::Valid | CameraConfigurationStatus::Adjusted => {
                            warn!("Using fallback format: {:?}", original_format);
                        },
                        CameraConfigurationStatus::Invalid => {
                            return Err(failed_precondition_error(
                                format!("Camera configuration was rejected: {:#?}",
                                        cfgs).as_str()));
                        },
                    }
                },
            }
        } else {
            // No preferred format found - list all available formats.
            warn!("No 8-bit or 10-bit format found. Available formats:");
            for format in &all_formats {
                warn!("  {:?}", format);
            }
            let default_format = cfgs.get(0).unwrap().get_pixel_format();
            warn!("Using default format: {:?}", default_format);
            match cfgs.validate() {
                CameraConfigurationStatus::Valid => (),
                CameraConfigurationStatus::Adjusted => {
                    debug!("Camera configuration was adjusted: {:#?}", cfgs);
                },
                CameraConfigurationStatus::Invalid => {
                    return Err(failed_precondition_error(
                        format!("Camera configuration was rejected: {:#?}",
                                cfgs).as_str()));
                },
            }
        }
        Ok(cfgs)
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
        controls.set(AwbEnable(false)).unwrap();
        controls.set(ExposureTime(exp_duration_micros as i32)).unwrap();
        controls.set(AnalogueGain(Self::cam_gain(
            abstract_gain, state.min_gain, state.max_gain))).unwrap();
        controls.set(NoiseReductionMode::Off).unwrap();
    }

    // Convert 10 or 12 bit pixels to 8 bits, by keeping the upper 8 bits.
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
                       image_data: &mut [u8],
                       width: usize,
                       height: usize,
                       is_10_bit: bool,
                       is_12_bit: bool,
                       is_packed: bool) {
        // Handle 8-bit format: no conversion needed, direct copy.
        if !is_10_bit && !is_12_bit {
            // For 8-bit, stride should equal width (1 byte per pixel).
            if stride == width {
                // Simple copy for contiguous data.
                image_data[..width * height].copy_from_slice(&buf_data[..width * height]);
            } else {
                // Copy row by row if there's stride padding.
                for row in 0..height {
                    let src_start = row * stride;
                    let dst_start = row * width;
                    image_data[dst_start..dst_start + width]
                        .copy_from_slice(&buf_data[src_start..src_start + width]);
                }
            }
            return;
        }

        // Use optimized function if we have one.
        if let Some(f) = CONVERT_FN.get() {
            f(stride, buf_data, image_data, width, height,
              is_10_bit, is_12_bit, is_packed);
            return;
        }
        if !is_packed {
            panic!("Unpacked raw format not yet supported");
        }
        if is_10_bit {
            // Convert from packed 10 bit to 8 bit.
            for row in 0..height {
                let buf_row_start = row * stride;
                let buf_row_end = buf_row_start + width*5/4;
                let pix_row_start = row * width;
                let pix_row_end = pix_row_start + width;
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
        } else {
            // Must be 12-bit since we handled 8-bit and 10-bit above
            assert!(is_12_bit, "Expected 12-bit format");
            // Convert from packed 12 bit to 8 bit.
            for row in 0..height {
                let buf_row_start = row * stride;
                let buf_row_end = buf_row_start + width*3/2;
                let pix_row_start = row * width;
                let pix_row_end = pix_row_start + width;
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
    }

    // The first call to capture_image() starts the capture thread that
    // executes this function.
    fn worker(state: Arc<Mutex<SharedState>>) {
        info!("Starting RpiCamera worker");

        // Whenever we change the camera settings, we will have discarded the
        // in-progress exposure, because the old settings were in effect when it
        // started. We need to mark dirty a number of images after a setting
        // change.
        let pipeline_depth = match state.lock().unwrap().model.as_str() {
            // These values are determined empirically by using the exposure_sweep
            // test program.
            "imx477" => 6,
            "imx219" => 4,
            "imx290" => 5,
            "imx296" => 4,
            "ov5647" => 4,
            "ov9281" => 3,
            _ => 5,
        };

        // How many captured images we will mark with params_accurate=false.
        let mut mark_image_count = 0;
        let mgr = CameraManager::new().unwrap();

        // Setting AeEnable(false) in setup_camera_request causes the libcamera C
        // implementation to log spurious warnings. I was unable to set the log
        // level to Error, so just turn off libcamera's logging for now.
        log_set_target(LoggingTarget::None).unwrap();
        let cameras = mgr.cameras();
        let cam = cameras.get(0).expect("No cameras found");
        let mut active_cam = cam.acquire().expect("Failed to acquire camera");

        let mut cfgs = Self::get_camera_configs(&cam).expect(
            "Failed to get camera configs in worker");

        // Increase buffer count by 1 to allow re-queueing before processing
        // completes.
        {
            let mut cfg = cfgs.get_mut(0).unwrap();
            let original_buffer_count = cfg.get_buffer_count();
            cfg.set_buffer_count(original_buffer_count + 1);
        }

        active_cam.configure(&mut cfgs).expect("Failed to configure camera");
        let cfg = cfgs.get(0).unwrap();
        let stride = cfg.get_stride() as usize;

        // Log the selected format.
        info!("Using camera format: {:?}", cfg.get_pixel_format());

        // Completed capture requests are returned as a callback, which stuffs
        // the completed request into a channel.
        let (tx, rx) = std::sync::mpsc::channel();
        active_cam.on_request_completed(move |req| {
            tx.send(req).unwrap();
        });

        // Allocate frame buffers for the stream.
        let mut alloc = FrameBufferAllocator::new(&cam);
        let stream = cfg.stream().unwrap();
        let buffers = alloc.alloc(&stream).unwrap();
        // Convert FrameBuffer to MemoryMappedFrameBuffer, which allows reading
        // &[u8].
        let buffers = buffers
            .into_iter()
            .map(|buf| MemoryMappedFrameBuffer::new(buf).unwrap())
            .collect::<Vec<_>>();

        let mut reqs;
        {
            let mut locked_state = state.lock().unwrap();
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
            // Initialize in-flight request count (will be reduced by 1 when we
            // hold back spare).
            locked_state.in_flight_request_count = reqs.len();
        }

        // Enqueue all but one request to the camera, holding back one for
        // immediate re-queueing.
        active_cam.start(None).unwrap();
        let spare_request = reqs.pop().expect(
            "Need at least one buffer for spare request");
        for req in reqs {
            active_cam.queue_request(req).unwrap();
        }
        // Adjust in-flight count since we held back one request.
        {
            let mut locked_state = state.lock().unwrap();
            locked_state.in_flight_request_count -= 1;
        }

        // Keep track of when we grabbed a frame.
        let mut last_frame_time: Option<Instant> = None;

        // For collecting requests during pipeline draining for HCG changes.
        let mut collected_requests = Vec::new();
        // Spare request for immediate re-queueing
        let mut spare_request = Some(spare_request);

        loop {
            let update_interval: Duration;
            {
                let mut locked_state = state.lock().unwrap();
                update_interval = locked_state.update_interval;
                if locked_state.stop_request {
                    info!("Stopping capture");
                    if let Err(e) = active_cam.stop() {
                        warn!("Error stopping capture: {:?}", e);
                    }
                    // Drain remaining completed requests to prevent resource
                    // leaks.
                    while let Ok(_req) = rx.try_recv() {
                        debug!("Drained completed request during shutdown");
                        // Request will be dropped here, releasing its buffer.
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
                    std::thread::sleep(delay);
                    continue;
                }
            }
            // Time to grab a frame.
            let mut req = match rx.recv_timeout(Duration::from_secs(5)) {
                Ok(req) => req,
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                    warn!("Camera request timeout - checking for stop request");
                    continue;  // Check stop_request in next loop iteration.
                },
                Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                    warn!("Camera request channel disconnected");
                    break;  // Exit loop and cleanup.
                },
            };
            let metadata = req.metadata();

            last_frame_time = Some(Instant::now());
            // Use the sensor's hardware timestamp from metadata for accurate capture time.
            // SensorTimestamp is in nanoseconds from the system's epoch.
            let readout_time = metadata.get::<libcamera::controls::SensorTimestamp>()
                .map(|libcamera::controls::SensorTimestamp(ns)| {
                    SystemTime::UNIX_EPOCH + Duration::from_nanos(ns as u64)
                })
                .unwrap_or_else(|_| {
                    // Fallback to current time if sensor timestamp unavailable
                    warn!("SensorTimestamp not available in metadata, using SystemTime::now()");
                    SystemTime::now()
                });

            let width;
            let height;
            let pisp_compressed;
            let pisp_compression_mode;
            let is_10_bit;
            let is_12_bit;
            let is_packed;
            let actual_exposure_duration;
            let actual_abstract_gain;
            {
                let mut locked_state = state.lock().unwrap();
                actual_exposure_duration = metadata.get::<ExposureTime>()
                    .map(|ExposureTime(exp_us)| Duration::from_micros(exp_us as u64))
                    .ok();
                actual_abstract_gain = metadata.get::<AnalogueGain>()
                    .map(|AnalogueGain(gain)| Self::abstract_gain(
                        gain, locked_state.min_gain, locked_state.max_gain))
                    .ok();
                width = locked_state.width;
                height = locked_state.height;
                pisp_compressed = locked_state.pisp_compressed;
                pisp_compression_mode = locked_state.pisp_compression_mode;
                is_10_bit = locked_state.is_10_bit;
                is_12_bit = locked_state.is_12_bit;
                is_packed = locked_state.is_packed;
                if locked_state.setting_changed {
                    mark_image_count = pipeline_depth;
                    locked_state.setting_changed = false;

                    // Handle HCG mode changes for IMX290.
                    if locked_state.hcg_needs_reinit &&
                       !locked_state.hcg_draining_pipeline
                    {
                        debug!("Starting pipeline drain for HCG mode change");
                        locked_state.hcg_draining_pipeline = true;
                        // We'll apply the actual HCG change when pipeline is
                        // empty.
                    }
                } else if mark_image_count > 0 {
                    mark_image_count -= 1;
                }
            }

            // Grab the image.
            let framebuffer: &MemoryMappedFrameBuffer<FrameBuffer> =
                req.buffer(&stream).unwrap();
            // Raw format has only one data plane containing bayer or mono data.
            let planes = framebuffer.data();
            let buf_data = planes.first().unwrap();

            // Early re-queueing: immediately queue the spare request before
            // expensive processing (unless we're draining the pipeline for HCG
            // changes).
            if let Some(mut spare) = spare_request.take() {
                let controls = spare.controls_mut();
                let locked_state = state.lock().unwrap();
                let draining = locked_state.hcg_draining_pipeline;
                Self::setup_camera_request(controls, &locked_state);
                drop(locked_state);

                if draining {
                    // During pipeline drain, collect the spare request instead
                    // of queueing.
                    collected_requests.push(spare);
                } else {
                    // Normal operation: queue the spare request immediately.
                    if let Err(e) = active_cam.queue_request(spare) {
                        warn!("Failed to queue spare request: {:?}", e);
                    } else {
                        // Update in-flight count for the queued spare request.
                        let mut locked_state = state.lock().unwrap();
                        locked_state.in_flight_request_count += 1;
                    }
                }
            }

            // Allocate uninitialized storage to receive the converted image data.
            let num_pixels = width * height;
            let mut image_data = Vec::<u8>::with_capacity(num_pixels);
            #[allow(clippy::uninit_vec)]
            unsafe { image_data.set_len(num_pixels) }

            if pisp_compressed {
                pisp_compression::uncompress(stride, buf_data, &mut image_data,
                                             width, height, pisp_compression_mode);
            } else {
                Self::convert_to_8bit(stride, buf_data, &mut image_data,
                                      width, height, is_10_bit, is_12_bit, is_packed);
            }
            let image = GrayImage::from_raw(
                width as u32, height as u32, image_data).unwrap();
            // Empirically I have seen that there's a transfer time from the
            // camera that is not captured in the measured processing_duration.
            // Apply a fudge factor for the transfer time.
            let processing_duration =
                Some(last_frame_time.unwrap().elapsed().mul_f64(1.5));

            // Handle request re-queueing or collection during pipeline
            // draining.
            req.reuse(ReuseFlag::REUSE_BUFFERS);
            let controls = req.controls_mut();
            let mut locked_state = state.lock().unwrap();

            // Decrement in-flight count for this completed request.
            locked_state.in_flight_request_count -= 1;

            if locked_state.hcg_draining_pipeline {
                // During pipeline drain, collect requests instead of
                // re-queueing.
                Self::setup_camera_request(controls, &locked_state);
                collected_requests.push(req);

                // Check if pipeline is fully drained.
                if locked_state.in_flight_request_count == 0 {
                    debug!("Pipeline drained, performing full camera restart for HCG mode change");
                    let hcg_enabled = locked_state.hcg_enabled;
                    drop(locked_state);

                    // Stop camera completely.
                    debug!("Stopping camera for HCG mode change");
                    if let Err(e) = active_cam.stop() {
                        warn!("Error stopping camera for HCG change: {:?}", e);
                    }
                    // Drain any remaining requests from the channel.
                    while rx.try_recv().is_ok() {
                        debug!("Drained request during camera stop");
                    }
                    // Re-configure the camera to reset internal state
                    debug!("Re-configuring camera after HCG mode change");
                    // Note: We can't create a fresh CameraManager due to
                    // lifetime constraints, but re-configuring should reset
                    // libcamera's internal state sufficiently.

                    // Apply HCG change after fresh configuration.
                    if let Err(e) = Self::set_hcg_mode(hcg_enabled) {
                        log::error!("Failed to set IMX290 HCG mode: {}", e);
                    } else {
                        log::info!("Successfully set IMX290 HCG mode to: {}",
                                   hcg_enabled);
                    }
                    // Wait for HCG change to settle.
                    std::thread::sleep(Duration::from_millis(50));

                    // Restart camera.
                    debug!("Restarting camera after HCG mode change");
                    if let Err(e) = active_cam.start(None) {
                        warn!("Error restarting camera after HCG change: {:?}", e);
                    }
                    // Apply HCG change again after camera restart in case
                    // active_cam.start() reset sensor registers.
                    if let Err(e) = Self::set_hcg_mode(hcg_enabled) {
                        log::error!("Failed to re-apply IMX290 HCG mode after start: {}", e);
                    } else {
                        log::debug!("Re-applied IMX290 HCG mode after start: {}",
                                    hcg_enabled);
                    }
                    // Brief delay for the second HCG change to settle before
                    // resuming capture.
                    std::thread::sleep(Duration::from_millis(50));

                    // Re-queue all collected requests.
                    let num_collected = collected_requests.len();
                    debug!("Re-queueing {} collected requests after restart",
                           num_collected);
                    for collected_req in collected_requests.drain(..) {
                        if let Err(e) = active_cam.queue_request(collected_req) {
                            warn!("Failed to re-queue collected request after restart: {:?}", e);
                            break;
                        }
                    }

                    // Reset state and update counters.
                    locked_state = state.lock().unwrap();
                    locked_state.hcg_needs_reinit = false;
                    locked_state.hcg_draining_pipeline = false;
                    locked_state.in_flight_request_count = num_collected;
                    // Extra delay for full restart.
                    mark_image_count = pipeline_depth + 4;
                }
            } else {
                // Normal operation: store current request as spare for next
                // early re-queue.
                Self::setup_camera_request(controls, &locked_state);
                spare_request = Some(req);
                // Don't increment in-flight count since we're holding this as
                // spare.
            }

            if !locked_state.setting_changed {
                locked_state.most_recent_capture = Some(CapturedImage {
                    capture_params: CaptureParams {
                        exposure_duration: actual_exposure_duration.unwrap_or(locked_state.camera_settings.exposure_duration),
                        gain: Gain::new(actual_abstract_gain.unwrap_or(locked_state.camera_settings.gain.value())),
                        offset: locked_state.camera_settings.offset,
                    },
                    params_accurate: mark_image_count == 0,
                    image: Arc::new(image),
                    readout_time,
                    processing_duration,
                });
                locked_state.frame_id += 1;

                // Estimate of time at which `most_recent_capture` will next be
                // updated.
                let now = Instant::now();
                if update_interval == Duration::ZERO {
                    locked_state.eta = Some(
                        now + actual_exposure_duration.unwrap_or(locked_state.camera_settings.exposure_duration));
                } else {
                    locked_state.eta = Some(now + update_interval);
                }
            }
        }  // loop.
        // Fallback cleanup if loop exits unexpectedly.
        warn!("Worker loop exited unexpectedly, performing cleanup");
        if let Err(e) = active_cam.stop() {
            warn!("Error in fallback camera stop: {:?}", e);
        }
        while rx.try_recv().is_ok() {}  // Drain channel.
    }  // worker().

    // Translate our 0..100 into the camera min/max range for analog gain.
    fn cam_gain(abstract_gain: i32, cam_min_gain: i32, cam_max_gain: i32) -> f32 {
        let frac = abstract_gain as f64 / 100.0;
        let cmg = cam_min_gain as f64;
        (cmg + (cam_max_gain as f64 - cmg) * frac) as f32
    }

    // Translate camera analog gain to our 0..100 abstract gain.
    fn abstract_gain(cam_gain: f32, cam_min_gain: i32, cam_max_gain: i32) -> i32 {
        let cmg = cam_min_gain as f64;
        let frac = (cam_gain as f64 - cmg) / (cam_max_gain as f64 - cmg);
        (frac * 100.0).ceil().clamp(0.0, 100.0) as i32
    }

    fn manage_worker_thread(&mut self) {
        // Has the worker terminated for some reason?
        if self.capture_thread.is_some() &&
            self.capture_thread.as_ref().unwrap().is_finished()
        {
            self.capture_thread.take().unwrap();
        }
        // Start capture thread if terminated or not yet started.
        if self.capture_thread.is_none() {
            let cloned_state = self.state.clone();
            // Allocate a thread for concurrent execution of image acquisition
            // and uncompressing with other activities.
            self.capture_thread = Some(std::thread::spawn(move || {
                Self::worker(cloned_state);
            }));
        }
    }

    // Function to enable high conversion gain, relevant for IMX462.
    fn set_hcg_mode(enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        if !Self::ensure_i2c_access() {
            // We've logged a warning.
            return Ok(());
        }
        let bus = Self::find_imx290_bus()?;
        let value = if enable { "0x11" } else { "0x01" };
        let output = Command::new("i2ctransfer")
            .args(["-f", "-y", &bus.to_string(), "w3@0x1a", "0x30", "0x09", value])
            .output()?;
        if !output.status.success() {
            let error = String::from_utf8_lossy(&output.stderr);
            return Err(format!("Failed to set HCG: {}", error).into());
        }
        log::debug!("HCG mode set to: {}", enable);
        Ok(())
    }

    fn ensure_i2c_access() -> bool {
        // Test if we can access I2C.
        let test_result = Command::new("i2cdetect")
            .args(["-y", "1"])
            .output();
        match test_result {
            Ok(output) if output.status.success() => true,
            Ok(output) => {
                let stderr = String::from_utf8_lossy(&output.stderr);
                if stderr.contains("Permission denied") {
                    log::warn!(
                        "I2C access denied. Please ensure user {} is in group i2c",
                        std::env::var("USER").unwrap_or_else(|_| "$USER".to_string())
                    );
                } else {
                    log::warn!("I2C error: {}", stderr);
                }
                false
            },
            Err(e) => {
                log::warn!("Failed to run i2cdetect: {:?}", e);
                false
            }
        }
    }

    fn find_imx290_bus() -> Result<u8, Box<dyn std::error::Error>> {
        // Common I2C buses to check (in order of likelihood).
        let candidate_buses = [11, 1, 0, 10, 12, 13, 14, 15];
        for &bus in &candidate_buses {
            if Self::test_imx290_on_bus(bus)? {
                log::debug!("Found IMX290 on I2C bus {}", bus);
                return Ok(bus);
            }
        }
        Err("IMX290 sensor not found on any I2C bus".into())
    }

    fn test_imx290_on_bus(bus: u8) -> Result<bool, Box<dyn std::error::Error>> {
        // Try to read a known register (0x3009) from IMX290.
        let output = Command::new("i2ctransfer")
            .args(["-f", "-y", &bus.to_string(), "w2@0x1a", "0x30", "0x09", "r1"])
            .output()?;
        // If command succeeds and returns data, sensor is present.
        if output.status.success() && !output.stdout.is_empty() {
            let response = String::from_utf8_lossy(&output.stdout);
            log::debug!("Bus {} response: {}", bus, response.trim());
            return Ok(true);
        }
        Ok(false)
    }


    fn extract_dtoverlay_config(file_path: &str, overlay_name: &str)
                                -> Result<Option<String>, std::io::Error> {
        let content = fs::read_to_string(file_path)?;
        for line in content.lines() {
            let trimmed = line.trim();
            if let Some(config) = Self::parse_dtoverlay_line(trimmed, overlay_name) {
                return Ok(Some(config));
            }
        }
        Ok(None)
    }

    fn parse_dtoverlay_line(line: &str, overlay_name: &str) -> Option<String> {
        let prefix = format!("dtoverlay={},", overlay_name);
        if line.starts_with(&prefix) {
            let config = &line[prefix.len()..];
            return Some(config.to_string());
        }
        None
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

    fn model_detail(&self) -> Option<String> {
        self.state.lock().unwrap().model_detail.clone()
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

        // Handle HCG mode switching for IMX290 based on gain threshold.
        if locked_state.model == "imx290" {
            let should_enable_hcg = gain.value() >= 50;
            if locked_state.hcg_enabled != should_enable_hcg {
                log::debug!("Switching IMX290 HCG mode: {} -> {}",
                            locked_state.hcg_enabled, should_enable_hcg);
                locked_state.hcg_enabled = should_enable_hcg;
                locked_state.hcg_needs_reinit = true;
                RpiCamera::changed_setting(&mut locked_state);
            }
        }

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

    fn set_update_interval(&mut self, update_interval: Duration)
                           -> Result<(), CanonicalError> {
        let mut locked_state = self.state.lock().unwrap();
        locked_state.update_interval = update_interval;
        Ok(())
    }

    async fn capture_image(&mut self, prev_frame_id: Option<i32>)
                           -> Result<(CapturedImage, i32), CanonicalError> {
        self.manage_worker_thread();
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
        self.manage_worker_thread();
        // Get the most recently posted image; return none if there is none yet
        // or the currently posted image's frame id is the same as
        // `prev_frame_id`.
        let locked_state = self.state.lock().unwrap();
        if locked_state.most_recent_capture.is_some() &&
            (prev_frame_id.is_none() ||
             prev_frame_id.unwrap() != locked_state.frame_id)
        {
            // Don't consume it, other clients may want it.
            return Ok(Some((locked_state.most_recent_capture.clone().unwrap(),
                            locked_state.frame_id)));
        }
        Ok(None)
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
            self.capture_thread.take().unwrap();
        }
    }
}
