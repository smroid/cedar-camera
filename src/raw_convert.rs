// Copyright (c) 2026 Steven Rosenthal smr@dt3.org
// See LICENSE file in root directory for license terms.

// Optional fast conversion of raw camera pixels to 8 bits, shared by camera
// backends. An application can install a platform-optimized implementation
// with set_converter(); backends use their own portable code otherwise.

use std::sync::OnceLock;

/// How raw pixel values are laid out in a capture buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RawLayout {
    /// One byte per pixel.
    Raw8,
    /// MIPI CSI-2 RAW10 packing: 4 pixels in 5 bytes, bytes 0-3 holding the
    /// high 8 bits of each pixel and byte 4 their low 2 bits.
    CsiPacked10,
    /// MIPI CSI-2 RAW12 packing: 2 pixels in 3 bytes, bytes 0-1 holding the
    /// high 8 bits of each pixel and byte 2 their low 4 bits.
    CsiPacked12,
    /// A little-endian 16-bit word per pixel, 10-bit value in the low bits.
    Word16Low10,
    /// A little-endian 16-bit word per pixel, 12-bit value in the low bits.
    Word16Low12,
    /// A little-endian 16-bit word per pixel, value in the high bits.
    Word16High,
    /// 10-bit pixels as a continuous little-endian bit stream: 4 pixels in 5
    /// bytes, pixel 0 in bits 0-9 of the group, pixel 1 in bits 10-19, and so
    /// on.
    Compact10,
}

/// Converts raw pixels to 8 bits by keeping the most significant 8 bits,
/// writing the full-resolution image to `image_data` and its 2x2-binned
/// average to `binned_data`. `stride` is the byte length of a raw row.
pub type ConvertFn = fn(
    stride: usize,
    buf_data: &[u8],
    image_data: &mut [u8],
    binned_data: &mut [u8],
    width: usize,
    height: usize,
    layout: RawLayout,
);

static CONVERT_FN: OnceLock<ConvertFn> = OnceLock::new();

pub fn set_converter(func: ConvertFn) {
    log::info!("Setting raw image converter function.");
    let _ = CONVERT_FN.set(func); // Ignores error if already set.
}

/// The installed converter, if any.
pub fn converter() -> Option<ConvertFn> {
    CONVERT_FN.get().copied()
}
