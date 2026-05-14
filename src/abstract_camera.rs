// Copyright (c) 2023 Steven Rosenthal smr@dt3.org
// See LICENSE file in root directory for license terms.

use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use async_trait::async_trait;
use image::GrayImage;
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
    /// The parameters that were in effect when the image capture occurred.
    pub capture_params: CaptureParams,

    /// After a change to gain, exposure, or offset, the next few CapturedImages
    /// will have their `image` still reflecting the previous parameters. This
    /// field identifies when `image` is consistent with `capture_params`. Most
    /// consumers of `image` don't care, but the auto-exposure and calibration
    /// algorithms need to be mindful not to use `image` data to influence the
    /// exposure when `params_accurate` is false.
    pub params_accurate: bool,

    /// 8 bit pixel data stored in row major order. For color cameras with
    /// binning==1, the raw values of the photosites are returned (linearly
    /// scaled to 8 bits); no demosaicing is done. The caller is responsible for
    /// dealing with the Bayer patterning of the color sensor; a simple approach
    /// used by Cedar Detect is to apply a simple 2x2 binning which yields a
    /// workable 8-bit monochrome value suitable for detecting and centroiding
    /// stars.
    /// For color cameras with binning==2, capture_image() applies 2x2 binning
    /// resulting in `image` being reduced resolution 8-bit monochrome.
    pub image: Arc<GrayImage>,

    /// The binning factor applied to produce `image`. Matches the value of
    /// AbstractCamera::binning() on the camera that captured this image.
    pub binning: u32,

    /// Whether `image` is from a color (Bayer pattern) sensor. Matches the
    /// value of AbstractCamera::is_color() on the camera that captured this image.
    pub is_color: bool,

    pub readout_time: SystemTime,
    pub readout_instant: Instant,

    // For some camera interfaces (e.g. RPi), there is some post-readout
    // processing needed i.e. to convert pixel format. Typically, this
    // processing is overlapped with the next camera exposure, but not always.
    // We measure the total time needed to acquire and process the camera image
    // and report it here. Not all camera types populate this field.
    pub processing_duration: Option<Duration>,
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
    async fn model(&self) -> String;

    /// Additional information beyond model(). Usually absent; on Raspberry Pi
    /// cameras this might include information from the dtoverlay entry for
    /// the camera, e.g. "clock-frequency=74250000".
    async fn model_detail(&self) -> Option<String>;

    /// Returns the (width, height) pixel count of this camera type's sensor.
    async fn dimensions(&self) -> (u32, u32);

    /// Tells if this camera is color (true) or monochrome (false). See notes
    /// on CapturedImage.image field.
    fn is_color(&self) -> bool;

    /// Returns the (width, height) dimensions, in mm, of this camera type's
    /// sensor.
    async fn sensor_size(&self) -> (f32, f32);

    /// Identifies the gain value that maximizes signal-to-noise performance
    /// for this camera type. See https://www.youtube.com/watch?v=SYQ1i4k62eI
    /// for an explanation of this idea.
    async fn optimal_gain(&self) -> Gain;

    // Tells if CapturedImage.image is the full dimensions() of this camera
    // (binning==1) or whether 2x2 binning has been applied (binning==2), in
    // which case the CapturedImage.image is half size in both dimensions.
    fn binning(&self) -> u32;

    // Changeable parameters that influence subsequent image captures.

    /// Returns InvalidArgument if specified exposure duration cannot be
    /// implemented by this camera type. Default is 100ms.
    async fn set_exposure_duration(&mut self, exp_duration: Duration)
                                   -> Result<(), CanonicalError>;
    /// Returns the exposure duration to be used for the next exposure.
    async fn get_exposure_duration(&self) -> Duration;

    /// Default is the optimal_gain() value.
    async fn set_gain(&mut self, gain: Gain) -> Result<(), CanonicalError>;
    async fn get_gain(&self) -> Gain;

    /// Default is 0.
    async fn set_offset(&mut self, offset: Offset) -> Result<(), CanonicalError>;
    async fn get_offset(&self) -> Offset;

    // Determines how often an image is captured for return in capture_image().
    // An interval of zero means run continuously-- images are captured as soon
    // as they become available in the camera.
    async fn set_update_interval(&mut self, update_interval: Duration)
                                 -> Result<(), CanonicalError>;

    // If binning()==2, determines whether the 2x2 binning is done on-chip or
    // in ISP pipeline (when true) or the 2x2 binning is done in software post-
    // processing (when false).
    // If binning()==1 this parameter is ignored.
    // Hardware/ISP binning is faster but for color sensors might result in some
    // S/N loss (TBD).
    async fn set_hardware_binning(&mut self, hw_binning: bool)
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
    async fn estimate_delay(&self, prev_frame_id: Option<i32>) -> Option<Duration>;

    /// Some implementations can shut down the camera to save power, e.g. by
    /// discontinuing video mode. A subsequent call to capture_image() will
    /// re-start the camera, at the expense of that capture_image() call taking
    /// longer than usual.
    /// Many implementations will treat stop() as a no-op.
    async fn stop(&mut self);
}
