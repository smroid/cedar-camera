extern crate chrono;

use std::time::Duration;

use clap::Parser;
use env_logger;
use log::info;

use asi_camera2::asi_camera2_sdk;
use camera_service::abstract_camera::{AbstractCamera, Gain, Offset};
use camera_service::asi_camera;

/// Utility program for capturing an series of images from the camera over a
/// range of gain values and exposure times.
#[derive(Parser, Debug)]
#[command(author, version, about, long_about=None)]
struct Args {
    /// Base name of the output file. We add the gain and exposure values to the
    /// filename and a .bmp extension.
    #[arg(short, long)]
    output: String,
}

fn main() {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("info")).init();
    let args = Args::parse();

    let num_cameras = asi_camera2_sdk::ASICamera::num_connected_asi_cameras();
    if num_cameras == 0 {
        panic!("No camera??");
    }
    if num_cameras > 1 {
        println!("num_cameras: {}; using first camera", num_cameras);
    }
    let mut asi_camera = asi_camera::ASICamera::new(
        asi_camera2_sdk::ASICamera::new(0)).unwrap();
    asi_camera.set_offset(Offset::new(3)).unwrap();

    let mut frame_id = -1;
    for gain in [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100] {
        asi_camera.set_gain(Gain::new(gain)).unwrap();
        for exp_ms in [10, 20, 50, 100] {
            info!("gain {}, exp time {}ms", gain, exp_ms);
            asi_camera.set_exposure_duration(Duration::from_micros(
                exp_ms as u64 * 1000)).unwrap();
            let (captured_image, new_frame_id) =
                asi_camera.capture_image(Some(frame_id)).unwrap();
            frame_id = new_frame_id;

            // Move captured_image's image data into a GrayImage.
            let image = &captured_image.image;

            // Modify the filename to incorporate the gain and exposure time. The .bmp
            // extension is automatically appended (it should not be provided on the
            // command line).
            let filename = format!("{}_g{}_e{}ms.bmp", args.output, gain, exp_ms);
            image.save(filename).unwrap();
        }
    }
}
