use chrono::offset::Local;
use chrono::DateTime;

use std::time::Duration;

use clap::Parser;
use env_logger;
use log::info;

use cedar_camera::abstract_camera::{Gain, Offset};
use cedar_camera::select_camera::select_camera;

/// Example program for capturing an image from the camera and saving it to a
/// file.
#[derive(Parser, Debug)]
#[command(author, version, about, long_about=None)]
struct Args {
    /// Base name of the output file. We add the gain and exposure values to the
    /// filename and a .bmp extension.
    #[arg(short, long)]
    output: String,

    /// Exposure integration time, in milliseconds.
    #[arg(short, long, default_value_t = 5)]
    exposure_time: i32,

    /// Camera gain, [0..100].
    #[arg(short, long, default_value_t = 100)]
    gain: i32,

    /// Camera offset, [0..20].
    #[arg(long, default_value_t = 3)]
    offset: i32,
}

#[tokio::main]
async fn main() {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("info")).init();

    let args = Args::parse();
    let mut camera = select_camera(None, 0).unwrap();

    // Ignore cameras that can't set offset.
    let _ = camera.set_offset(Offset::new(args.offset));

    camera.set_gain(Gain::new(args.gain)).unwrap();

    let exposure_time_millisec = args.exposure_time;
    camera.set_exposure_duration(Duration::from_micros(
        exposure_time_millisec as u64 * 1000)).unwrap();
    println!("capture_image");
    let (captured_image, _frame_id) = camera.capture_image(None).await.unwrap();
    println!("finished capture_image");

    // Move captured_image's image data into a GrayImage.
    let image = &captured_image.image;

    // Modify the filename to incorporate the gain and exposure time. The .bmp
    // extension is automatically appended (it should not be provided on the
    // command line).
    let filename = format!("{}_g{}_e{}ms.bmp",
                           args.output, args.gain, args.exposure_time);
    image.save(filename).unwrap();

    let datetime: DateTime<Local> = captured_image.readout_time.into();
    info!("Image obtained at {} with temperature {}",
          datetime.format("%d/%m/%Y %T"), captured_image.temperature.0);
}
