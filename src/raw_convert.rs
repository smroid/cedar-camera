// Copyright (c) 2026 Steven Rosenthal smr@dt3.org
// See LICENSE file in root directory for license terms.

// Optional fast conversion of raw camera pixels to 8 bits, shared by camera
// backends. An application can install a platform-optimized implementation
// with set_converter(); backends use their own portable code otherwise.

use std::sync::OnceLock;

/// Converts raw pixels to 8 bits by keeping the most significant 8 bits,
/// writing the full-resolution image to `image_data` and its 2x2-binned
/// average to `binned_data`. `stride` is the byte length of a raw row.
///
/// Raw formats, by flags:
/// * none set: 8 bits per pixel.
/// * `is_10_bit` or `is_12_bit` with `is_packed`: MIPI CSI-2 packed.
/// * `is_10_bit` or `is_12_bit` without `is_packed`: a little-endian 16-bit
///   word per pixel, value in the low bits.
/// * `is_16_bit`: a little-endian 16-bit word per pixel, value in the high
///   bits.
pub type ConvertFn = fn(
    stride: usize,
    buf_data: &[u8],
    image_data: &mut [u8],
    binned_data: &mut [u8],
    width: usize,
    height: usize,
    is_10_bit: bool,
    is_12_bit: bool,
    is_16_bit: bool,
    is_packed: bool,
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
