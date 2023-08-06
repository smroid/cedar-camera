extern crate chrono;
use chrono::offset::Local;
use chrono::DateTime;

use std::env;
use std::time::Duration;

use log::info;
use env_logger;

use asi_camera2::asi_camera2_sdk;
use camera_service::abstract_camera::{AbstractCamera, Gain, Offset};
use camera_service::asi_camera;

fn main() {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("info")).init();
    let args: Vec<String> = env::args().collect();

    let num_cameras = asi_camera2_sdk::ASICamera::num_connected_asi_cameras();
    if num_cameras == 0 {
        panic!("No camera??");
    }
    if num_cameras > 1 {
        println!("num_cameras: {}; using first camera", num_cameras);
    }
    let mut asi_camera = asi_camera::ASICamera::new(
        asi_camera2_sdk::ASICamera::new(0)).unwrap();
    asi_camera.set_offset(Offset::new(2)).unwrap();
    asi_camera.set_gain(Gain::new(100)).unwrap();

    let exposure_time_millisec = 0.2;
    asi_camera.set_exposure_duration(Duration::from_micros(
        (exposure_time_millisec * 1000.0) as u64)).unwrap();
    let captured_image = asi_camera.capture_image().unwrap();

    // Move captured_image's image data into a GrayImage.
    let image = &captured_image.image;

    // Modify the filename to incorporate the exposure time. The .bmp extension
    // is automatically appended (it should not be provided on the command
    // line).
    let filename = format!("{}_{}ms.bmp", &args[1], exposure_time_millisec);
    image.save(filename).unwrap();

    let datetime: DateTime<Local> = captured_image.readout_time.into();
    info!("Image obtained at {} with temperature {}",
          datetime.format("%d/%m/%Y %T"), captured_image.temperature.0);
}
