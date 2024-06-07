use cedar_camera::asi_camera::ASICamera;
use cedar_camera::rpi_camera::RpiCamera;

fn main() {
    // Enumerate ASI cameras.
    let asi_cameras = ASICamera::enumerate_cameras();
    println!("Found {} ASI cameras: ", asi_cameras.len());
    for (i, info) in asi_cameras.iter().enumerate() {
        println!("{}: {:?}", i, info);
    }

    // Enumerate Rpi cameras.
    let rpi_cameras = RpiCamera::enumerate_cameras();
    println!("Found {} Rpi cameras: ", rpi_cameras.len());
    for (i, info) in rpi_cameras.iter().enumerate() {
        println!("{}: {:?}", i, info);
    }
}
