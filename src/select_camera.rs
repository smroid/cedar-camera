// Copyright (c) 2023 Steven Rosenthal smr@dt3.org
// See LICENSE file in root directory for license terms.

use crate::abstract_camera::AbstractCamera;
#[cfg(feature = "asi")]
use crate::asi_camera::ASICamera;
#[cfg(feature = "rpi")]
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
//
// A camera interface is only "present" if its Cargo feature is enabled at build
// time (`asi` / `rpi`). With neither feature enabled no hardware camera can be
// located and `not_found_error` is returned; callers (e.g. cedar-server) fall
// back to an `ImageCamera` in that case.
#[cfg_attr(not(any(feature = "asi", feature = "rpi")), allow(unused_variables, unused_mut))]
pub async fn select_camera(
    mut camera_interface: Option<&CameraInterface>, camera_index: usize)
    -> Result<Box<dyn AbstractCamera + Send>, CanonicalError>
{
    // Enumerate cameras on supported interfaces. An interface whose feature is
    // not compiled in is treated as having no cameras.
    #[cfg(feature = "asi")]
    let has_asi = !ASICamera::enumerate_cameras().is_empty();
    #[cfg(not(feature = "asi"))]
    let has_asi = false;

    #[cfg(feature = "rpi")]
    let has_rpi = !RpiCamera::enumerate_cameras().is_empty();
    #[cfg(not(feature = "rpi"))]
    let has_rpi = false;

    if !has_asi && !has_rpi {
        return Err(not_found_error("No camera found"));
    }
    if has_asi && !has_rpi {
        if let Some(ci) = camera_interface {
            if *ci != CameraInterface::ASI {
                return Err(failed_precondition_error(
                    format!("Only ASI camera found but {:?} was requested", ci).as_str()));
            }
        }
        camera_interface = Some(&CameraInterface::ASI);
    }
    if has_rpi && !has_asi {
        if let Some(ci) = camera_interface {
            if *ci != CameraInterface::Rpi {
                return Err(failed_precondition_error(
                    format!("Only Rpi camera found but {:?} was requested", ci).as_str()));
            }
        }
        camera_interface = Some(&CameraInterface::Rpi);
    }
    match camera_interface {
        None => {
            Err(failed_precondition_error(
                "Both ASI and Rpi cameras found but no 'camera_interface' selector was passed"))
        },
        Some(CameraInterface::ASI) => {
            #[cfg(feature = "asi")]
            { Ok(Box::new(ASICamera::new(camera_index).await?)) }
            #[cfg(not(feature = "asi"))]
            { Err(failed_precondition_error(
                "ASI camera support is not compiled in (enable the 'asi' feature)")) }
        },
        Some(CameraInterface::Rpi) => {
            #[cfg(feature = "rpi")]
            { Ok(Box::new(RpiCamera::new(camera_index).await?)) }
            #[cfg(not(feature = "rpi"))]
            { Err(failed_precondition_error(
                "Rpi camera support is not compiled in (enable the 'rpi' feature)")) }
        },
    }
}
