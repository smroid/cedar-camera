use crate::abstract_camera::AbstractCamera;
use crate::asi_camera::ASICamera;
use crate::rpi_camera::RpiCamera;

use canonical_error::{CanonicalError, failed_precondition_error, not_found_error};

#[derive(Debug, PartialEq)]
pub enum CameraInterface {
    ASI,
    Rpi,
}

// Enumerates the supported camera interface types (currently ASI and Rpi)
// and returns a camera:
// * If there is only one camera interface type present, that interface's camera
//   is returned. In this case `camera_interface` (if given) is checked against
//   the located camera.
// * If there is more than one camera interface present, `camera_interface`
//   indicates which interface's camera is to be returned.
// * `camera_index` controls which camera (on the selected interface) is
//   returned.
pub fn select_camera(camera_interface: Option<CameraInterface>,
                     camera_index: i32) -> Result<Box<dyn AbstractCamera>, CanonicalError> {
    // Enumerate cameras on supported interfaces.
    let asi_cameras = ASICamera::enumerate_cameras();
    let rpi_cameras = RpiCamera::enumerate_cameras();
    if asi_cameras.len() == 0 && rpi_cameras.len() == 0 {
        return Err(not_found_error("No camera found"));
    }
    if asi_cameras.len() > 0 && rpi_cameras.len() == 0 {
        if let Some(ci) = camera_interface {
            if ci != CameraInterface::ASI {
                return Err(failed_precondition_error(
                    format!("Only ASI camera found but {:?} was requested", ci).as_str()));
            }
        }
        return Ok(Box::new(ASICamera::new(camera_index)?));
    }
    if rpi_cameras.len() > 0 && asi_cameras.len() == 0 {
        if let Some(ci) = camera_interface {
            if ci != CameraInterface::Rpi {
                return Err(failed_precondition_error(
                    format!("Only Rpi camera found but {:?} was requested", ci).as_str()));
            }
        }
        return Ok(Box::new(RpiCamera::new(camera_index)?));
    }
    // Both camera interfaces are present.
    match camera_interface {
        None => {
            Err(failed_precondition_error(
                "Both ASI and Rpi cameras found but no 'camera_interface' selector was passed"))
        },
        Some(CameraInterface::ASI) => {
            Ok(Box::new(ASICamera::new(camera_index)?))
        },
        Some(CameraInterface::Rpi) => {
            Ok(Box::new(RpiCamera::new(camera_index)?))
        },
    }
}
