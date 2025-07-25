// Copyright (c) 2023 Steven Rosenthal smr@dt3.org
// See LICENSE file in root directory for license terms.

use std::time::Duration;
use std::ops::Deref;

use clap::Parser;
use env_logger;
use log::info;
use imageproc::rect::Rect;

use cedar_camera::abstract_camera::{Gain, Offset, bin_2x2};
use cedar_camera::select_camera::select_camera;
use cedar_detect::algorithm::{estimate_background_from_image_region};

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

#[tokio::main]
async fn main() {
    // If any thread panics, bail out.
    std::panic::set_hook(Box::new(|panic_info| {
        eprintln!("Thread panicked: {}", panic_info);
        std::process::exit(1);
    }));
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("info")).init();
    let args = Args::parse();
    let mut camera = select_camera(None, 0).unwrap();
    info!("camera: {}", camera.model());
    info!("detail: {:?}", camera.model_detail());
    // Ignore cameras that can't set offset.
    let _ = camera.set_offset(Offset::new(3));

    let (width, height) = camera.dimensions();
    // Central region.
    let roi = Rect::at(width / 2, height / 2).of_size(30, 30);

    let mut frame_id = -1;
    for gain in [0, 1, 2, 5, 10, 20, 50, 100] {
        camera.set_gain(Gain::new(gain)).unwrap();
        for exp_ms in [1, 2, 5, 10] {
            camera.set_exposure_duration(Duration::from_micros(
                exp_ms as u64 * 1000)).unwrap();
            let image;
            let mut captured_image;
            loop {
                (captured_image, frame_id) =
                    camera.capture_image(Some(frame_id)).await.unwrap();
                // Wait for camera's image to accurately reflect the camera's
                // settings.
                if captured_image.params_accurate {
                    image = &captured_image.image;
                    break;
                }
            }

            // let image = &captured_image.image;
            let (background, noise);
            if camera.is_color() {
                // Bayer grid will exhibit high noise. Do a 2x2 downsample which is
                // a poor-man's debayering.
                let mono_image = bin_2x2(image.deref().clone());
                let (binned_width, binned_height) = mono_image.dimensions();
                let binned_roi = Rect::at(
                    binned_width as i32 / 2, binned_height as i32 / 2).of_size(30, 30);
                (background, noise) = estimate_background_from_image_region(
                    &mono_image, &binned_roi);
            } else {
                (background, noise) = estimate_background_from_image_region(image, &roi);
            }
            info!("params {:?}; background/noise {:.2}/{:.2}",
                  captured_image.capture_params, background, noise);

            // Modify the filename to incorporate the gain and exposure time. The .bmp
            // extension is automatically appended (it should not be provided on the
            // command line).
            let filename = format!("{}_g{}_e{}ms.bmp", args.output, gain, exp_ms);
            image.save(filename).unwrap();
        }
    }
}
