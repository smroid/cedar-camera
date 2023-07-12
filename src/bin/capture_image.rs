extern crate chrono;
use chrono::offset::Local;
use chrono::DateTime;

use std::time::Duration;

use image::GrayImage;

use asi_camera2::asi_camera2_sdk;
use camera_service::abstract_camera::AbstractCamera;
use camera_service::asi_camera;

fn main() {
    let num_cameras = asi_camera2_sdk::num_connected_asi_cameras();
    if num_cameras == 0 {
        panic!("No camera??");
    }
    if num_cameras > 1 {
        println!("num_cameras: {}; using first camera", num_cameras);
    }
    let mut asi_camera = asi_camera::create_asi_camera(
        asi_camera2_sdk::create_asi_camera(0)).unwrap();
    let (width, height) = asi_camera.dimensions();

    // Allocate buffer to receive camera data.
    let pixels = vec![0u8; (width*height) as usize];

    let exposure_time_millisec = 5;
    asi_camera.set_exposure_duration(Duration::from_millis(
        exposure_time_millisec)).unwrap();
    let captured_image = asi_camera.capture_image(pixels).unwrap();

    // Move captured_image's image data into a GrayImage.
    let image = GrayImage::from_raw(width as u32, height as u32,
                                    captured_image.image_data).unwrap();
    image.save("image.jpg").unwrap();

    let datetime: DateTime<Local> = captured_image.readout_time.into();
    println!("Image obtained at {} with temperature {}",
             datetime.format("%d/%m/%Y %T"), captured_image.temperature.0);
}
