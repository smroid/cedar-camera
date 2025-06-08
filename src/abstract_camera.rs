// Copyright (c) 2023 Steven Rosenthal smr@dt3.org
// See LICENSE file in root directory for license terms.

use std::sync::Arc;
use std::time::{Duration, SystemTime};

use async_trait::async_trait;
use image::GrayImage;
use fast_image_resize::images::Image;
use fast_image_resize::{FilterType, Resizer, ResizeOptions,
                        ResizeAlg::Convolution, ResizeAlg};
use canonical_error::CanonicalError;

/// Each kind of camera interface (e.g. ASI (USB port), Raspberry Pi (CSI
/// connector)) provides a function to enumerate the connected camera(s), if
/// any, for that interface type.
/// The enumeration function returns an instance of this struct for each
/// detected camera.
#[derive(Debug)]
pub struct EnumeratedCameraInfo {
    /// Identifies what kind of camera this is. e.g. "ASI120mm mini", "RPiCam2", etc.
    pub model: String,

    /// Number of pixels in the camera's sensor.
    pub width: u32,
    pub height: u32,
}

/// Abstract camera gain values range from 0 to 100, inclusive. Each camera type
/// scales this gain value as needed, mapping 0 to its actual lowest gain and 100
/// to its highest gain.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Gain(i32);

impl Gain {
    pub fn new(gain: i32) -> Gain {
        assert!(gain >= 0);
        assert!(gain <= 100);
        Gain(gain)
    }

    pub fn value(&self) -> i32 {
        self.0
    }
}

/// Abstract camera offset values range from 0 to 20, inclusive. Each camera type
/// scales the offset value as appropriate. 0 always means no offset; a non-zero
/// offset can be used to "lift" pixel values up from 0 ADU to avoid crushing
/// dark pixels to black.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Offset(i32);

impl Offset {
    pub fn new(offset: i32) -> Offset {
        assert!(offset >= 0);
        assert!(offset <= 20);
        Offset(offset)
    }

    pub fn value(&self) -> i32 {
        self.0
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct CaptureParams {
    pub exposure_duration: Duration,
    pub gain: Gain,
    pub offset: Offset,
}

impl CaptureParams {
    pub fn new() -> CaptureParams {
        CaptureParams{exposure_duration: Duration::from_millis(10),
                      gain: Gain::new(0),
                      offset: Offset::new(0),
        }
    }
}

#[derive(Clone, Debug)]
pub struct CapturedImage {
    /// The parameters that were in effect when the image capture
    /// occurred.
    pub capture_params: CaptureParams,

    /// After a change to gain, exposure, or offset, the next few CapturedImages
    /// will have the new `capture_params` but the `image` might still reflect
    /// the previous parameters. This field identifies when `image` is
    /// consistent with `capture_params`. Most consumers of `image` don't care,
    /// but the auto-exposure and calibration algorithms need to be mindful not
    /// to use `image` data to influence the exposure when `params_accurate` is
    /// false.
    pub params_accurate: bool,

    /// 8 bit pixel data stored in row major order. For color cameras, the raw
    /// values of the photosites are returned (linearly scaled to 8 bits); no
    /// demosaicing is done. The caller is responsible for dealing with the
    /// Bayer patterning of color sensor; a simple approach used by Cedar Detect
    /// is to apply a simple 2x2 binning which yields a workable luma value
    /// suitable for detecting and centroiding stars.
    pub image: Arc<GrayImage>,

    pub readout_time: SystemTime,
}

/// AbstractCamera models an 8-bit greyscale camera. This trait defines methods
/// for obtaining information about the camera, setting its operating parameters,
/// and capturing images.
/// Note that we do not provide a 'gamma' setting; all AbstractCamera implementations
/// should configure the camera for linear mapping.
#[async_trait]
pub trait AbstractCamera {
    // Unchanging attributes.

    /// Returns a string identifying what kind of camera this is. e.g.
    /// "ASI120mm mini", "imx477", etc.
    fn model(&self) -> String;

    /// Returns the (width, height) pixel count of this camera type's sensor.
    fn dimensions(&self) -> (i32, i32);

    /// Tells if this camera is color (true) or monochrome (false).
    fn is_color(&self) -> bool;

    /// Returns the (width, height) dimensions, in mm, of this camera type's
    /// sensor.
    fn sensor_size(&self) -> (f32, f32);

    /// Identifies the gain value that maximizes signal-to-noise performance
    /// for this camera type. See https://www.youtube.com/watch?v=SYQ1i4k62eI
    /// for an explanation of this idea.
    fn optimal_gain(&self) -> Gain;

    // Changeable parameters that influence subsequent image captures.

    /// Returns InvalidArgument if specified exposure duration cannot be
    /// implemented by this camera type. Default is 100ms.
    fn set_exposure_duration(&mut self, exp_duration: Duration)
                             -> Result<(), CanonicalError>;
    /// Returns the exposure duration to be used for the next exposure.
    fn get_exposure_duration(&self) -> Duration;

    /// Default is the optimal_gain() value.
    fn set_gain(&mut self, gain: Gain) -> Result<(), CanonicalError>;
    fn get_gain(&self) -> Gain;

    /// Default is 0.
    fn set_offset(&mut self, offset: Offset) -> Result<(), CanonicalError>;
    fn get_offset(&self) -> Offset;

    // Determines how often an image is captured for return in capture_image().
    // An interval of zero means run continuously-- images are captured as soon
    // as they become available in the camera.
    fn set_update_interval(&mut self, update_interval: Duration)
                           -> Result<(), CanonicalError>;

    // Action methods.

    /// Obtains a single image from this camera, as configured above. The
    /// returned image is "fresh" in that we either wait for a new exposure or
    /// return the most recently completed exposure.
    /// This function does not "consume" the image that it returns; multiple
    /// callers will receive the current image (or next image, if there is not
    /// yet a current image) if `prev_frame_id` is omitted. If `prev_frame_id`
    /// is supplied, the call blocks while the current image has the same id
    /// value.
    /// Returns: the captured image along with its frame_id value.
    async fn capture_image(&mut self, prev_frame_id: Option<i32>)
                           -> Result<(CapturedImage, i32), CanonicalError>;

    /// Non-blocking variant of `capture_image()`. Returns None if
    /// `capture_image()` would block.
    async fn try_capture_image(&mut self, prev_frame_id: Option<i32>)
                               -> Result<Option<(CapturedImage, i32)>, CanonicalError>;

    /// Returns an estimate of how long a call to capture_image() with the given
    /// id will block. None if there is no estimate.
    fn estimate_delay(&self, prev_frame_id: Option<i32>) -> Option<Duration>;

    /// Some implementations can shut down the camera to save power, e.g. by
    /// discontinuing video mode. A subsequent call to capture_image() will
    /// re-start the camera, at the expense of that capture_image() call taking
    /// longer than usual.
    /// Many implementations will treat stop() as a no-op.
    async fn stop(&mut self);
}

// Utility functions.
pub fn bin_2x2(image: GrayImage) -> GrayImage {
    resize_2x2(image, Convolution(FilterType::Box))
}

pub fn sample_2x2(image: GrayImage) -> GrayImage {
    resize_2x2(image, ResizeAlg::Nearest)
}

fn resize_2x2(image: GrayImage, alg: ResizeAlg) -> GrayImage {
    let (width, height) = image.dimensions();
    let resized_width = width / 2;
    let resized_height = height / 2;

    // Convert GrayImage to Image for fast_image_resize.
    let src_image = Image::from_vec_u8(width, height, image.into_raw(),
                                       fast_image_resize::PixelType::U8).unwrap();
    let mut dst_image = Image::new(resized_width, resized_height,
                                   src_image.pixel_type());
    // Resize the image into the dst_image buffer.
    let mut resizer = Resizer::new();
    resizer.resize(
        &src_image, &mut dst_image,
        &ResizeOptions::new().resize_alg(alg)).unwrap();

    GrayImage::from_raw(resized_width, resized_height,
                        dst_image.into_vec()).unwrap()
}
