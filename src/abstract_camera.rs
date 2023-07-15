#![allow(dead_code)]

use std::fmt;

use canonical_error::CanonicalError;

/// Abstract camera gain values range from 0 to 100, inclusive. Each camera type
/// scales this gain value as needed, mapping 0 to its actual lowest gain and 100
/// to its highest gain.
#[derive(Copy, Clone, Debug)]
pub struct Gain(i32);

impl fmt::Display for Gain {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", self)  // Just re-use Debug.
    }
}

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
#[derive(Copy, Clone, Debug)]
pub struct Offset(i32);

impl fmt::Display for Offset {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", self)  // Just re-use Debug.
    }
}

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

#[derive(Copy, Clone, Debug)]
pub struct Celsius(pub i32);

#[derive(Copy, Clone, Debug)]
pub enum Flip {
    None, Horizontal, Vertical, Both
}

/// Some cameras provide in-hardware binning. We require all cameras
/// to support unbinned and 2x binned operation; if a camera does
/// not natively provide binning capability, the AbstractCamera
/// implementation must perform software binning.
/// Note that binning is either by summing or averaging; it is up
/// to application logic to react and set exposure accordingly.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum BinFactor {
    X1,  // Unbinned.
    X2,  // Each output pixel is the combined value of 2x2 input pixels.
}

#[derive(Copy, Clone, Debug)]
pub struct RegionOfInterest {
    pub binning: BinFactor,

    // The capture position and dimensions are w.r.t. the dimensions
    // of the sensor after binning.

    /// (x, y) from top left.
    pub capture_startpos: (i32, i32),

    /// (width, height).
    pub capture_dimensions: (i32, i32),
}

pub struct CapturedImage {
    /// Pixel data stored in row major order. This is typically (but not
    /// always) the vector passed to AbstractCamera::capture_image().
    pub image_data: Vec<u8>,

    pub flip: Flip,
    pub exposure_duration: std::time::Duration,
    pub roi: RegionOfInterest,
    pub gain: Gain,
    pub offset: Offset,

    pub readout_time: std::time::SystemTime,
    pub temperature: Celsius,
}

/// AbstractCamera models an 8-bit greyscale camera. This trait defines methods
/// for obtaining information about the camera, setting its operating parameters,
/// and capturing images.
/// Note that we do not provide a 'gamma' setting; all AbstractCamera implementations
/// should configure the camera for linear mapping.
pub trait AbstractCamera {
    // Unchanging attributes.

    /// Returns a string identifying what kind of camera this is. e.g.
    /// "ASI120mm mini", "RPiCam2", etc.
    fn model(&self) -> Result<String, CanonicalError>;

    /// Returns the (width, height) of this camera type's sensor.
    fn dimensions(&self) -> (i32, i32);

    /// Identifies the gain value that maximizes signal-to-noise performance
    /// for this camera type. See https://www.youtube.com/watch?v=SYQ1i4k62eI
    /// for an explanation of this idea.
    fn optimal_gain(&self) -> Gain;

    // Changeable parameters that influence subsequent image captures.

    /// Default is None.
    fn set_flip_mode(&mut self, flip_mode: Flip) -> Result<(), CanonicalError>;
    fn get_flip_mode(&self) -> Flip;

    /// Returns InvalidArgument if specified exposure duration cannot be
    /// implemented by this camera type. Default is 100ms.
    fn set_exposure_duration(&mut self, exp_duration: std::time::Duration)
                             -> Result<(), CanonicalError>;
    /// Returns the exposure duration to be used for the next exposure.
    fn get_exposure_duration(&self) -> std::time::Duration;

    /// Default is unbinned, whole image. When setting region of interest,
    /// the implementation should adjust capture_startpos and/or capture_dimensions
    /// as needed to satisfy constraints of this camera type (e.g. capture width
    /// might need to be a multiple of 16). The adjusted region of interest is
    /// returned.
    fn set_region_of_interest(&mut self, roi: RegionOfInterest)
                              -> Result<RegionOfInterest, CanonicalError>;
    fn get_region_of_interest(&self) -> RegionOfInterest;

    /// Default is the optimal_gain() value.
    fn set_gain(&mut self, gain: Gain) -> Result<(), CanonicalError>;
    fn get_gain(&self) -> Gain;

    /// Default is 0.
    fn set_offset(&mut self, offset: Offset) -> Result<(), CanonicalError>;
    fn get_offset(&self) -> Offset;

    // Action methods.

    /// Obtains a single image from this camera, as configured above. The
    /// returned image is "fresh" in that we either initiate the exposure or
    /// we grab the next video frame exposure to complete.
    /// Once an image has been obtained, calling this function again obtains a
    /// new image.
    /// This function blocks until the image is available. The wait time is
    /// related to the exposure duration but can be shorter or longer depending
    /// on the implementation of this camera type:
    /// Shorter: If the implementation runs the camera in video mode, a call to
    ///     capture_image() that occurs just as a video frame interval ends can
    ///     return quickly as it does not need to wait for the video frame time
    ///     to finish. A randomly timed call to capture_image() would only need
    ///     to wait on average for half the exposure duration.
    /// Longer: The first call to capture_image(), or the next call to
    ///     capture_image() after changing certain settings, can incur significant
    ///     delay beyond the exposure duration.
    /// `image_data` must be sized to at least the ROI's capture dimensions
    ///     width*height. The `image_data` is typically moved (not copied) to the
    ///     returned CapturedImage, though some implementations might manage
    ///     memory differently.
    fn capture_image(&mut self, image_data: Vec<u8>)
                     -> Result<CapturedImage, CanonicalError>;

    /// Some implementations can shut down the camera to save power, e.g. by
    /// discontinuing video mode. A subsequent call to capture_image() will
    /// re-start the camera, at the expense of that capture_image() call taking
    /// longer than usual.
    /// Many implementations will treat stop() as a no-op.
    fn stop(&mut self) -> Result<(), CanonicalError>;
}
