// Copyright (c) 2023 Steven Rosenthal smr@dt3.org
// See LICENSE file in root directory for license terms.

use env_logger;

#[cfg(feature = "asi")]
use cedar_camera::asi_camera::ASICamera;
#[cfg(feature = "rpi")]
use cedar_camera::rpi_camera::RpiCamera;

fn main() {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("info")).init();

    // Enumerate ASI cameras.
    #[cfg(feature = "asi")]
    {
        let asi_cameras = ASICamera::enumerate_cameras();
        println!("Found {} ASI cameras: ", asi_cameras.len());
        for (i, info) in asi_cameras.iter().enumerate() {
            println!("{}: {:?}", i, info);
        }
    }
    #[cfg(not(feature = "asi"))]
    println!("ASI camera support not compiled in (enable the 'asi' feature).");

    // Enumerate Rpi cameras.
    #[cfg(feature = "rpi")]
    {
        let rpi_cameras = RpiCamera::enumerate_cameras();
        println!("Found {} Rpi cameras: ", rpi_cameras.len());
        for (i, info) in rpi_cameras.iter().enumerate() {
            println!("{}: {:?}", i, info);
        }
    }
    #[cfg(not(feature = "rpi"))]
    println!("Rpi camera support not compiled in (enable the 'rpi' feature).");
}
