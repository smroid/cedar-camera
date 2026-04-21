// Copyright (c) 2025 Steven Rosenthal smr@dt3.org
// See LICENSE file in root directory for license terms.

// Starting with Raspberry Pi 5, RAW images are in PiSP compressed raw format.
//
// See:
// https://datasheets.raspberrypi.com/camera/raspberry-pi-image-signal-processor-specification.pdf
// Section 4 (page 7)
// and
// https://www.kernel.org/doc/html/v6.12/userspace-api/media/v4l/pixfmt-srggb8-pisp-comp.html
// https://github.com/raspberrypi/rpicam-apps/blob/main/image/dng.cpp

const COMPRESS_OFFSET: u32 = 2048;
const COMPRESS_MODE: i32 = 1;

// Combine dequantize + postprocess + >>8 into a single operation returning u8.
// postprocess adds COMPRESS_OFFSET (2048) and clamps to 0xFFFF, then >>8 gives
// the top byte. Inlining eliminates intermediate u16 values.
#[inline(always)]
fn dequantize_to_u8(q: i32, qmode: i32) -> u8 {
    let dq: u32 = match qmode {
        0 => if q < 320 { (16 * q) as u32 } else { (32 * (q - 160)) as u32 },
        1 => (64 * q) as u32,
        2 => (128 * q) as u32,
        _ => if q < 94 { (256 * q) as u32 } else { u32::min(0xFFFF, (512 * (q - 47)) as u32) },
    };
    // postprocess: clamp(dq + COMPRESS_OFFSET, 0xFFFF) >> 8
    (u32::min(0xFFFF, dq + COMPRESS_OFFSET) >> 8) as u8
}

#[inline(always)]
fn sub_block_function(w: u32) -> [u8; 4] {
    let qmode = (w & 3) as i32;
    let q: [i32; 4];

    if qmode < 3 {
        let field0 = ((w >> 2) & 511) as i32;
        let field1 = ((w >> 11) & 127) as i32;
        let field2 = ((w >> 18) & 127) as i32;
        let field3 = ((w >> 25) & 127) as i32;
        let q1;
        let q2;
        if qmode == 2 && field0 >= 384 {
            q1 = field0;
            q2 = field1 + 384;
        } else {
            q1 = if field1 >= 64 { field0 } else { field0 + 64 - field1 };
            q2 = if field1 >= 64 { field0 + field1 - 64 } else { field0 };
        }
        let mut p1 = i32::max(0, q1 - 64);
        let mut p2 = i32::max(0, q2 - 64);
        if qmode == 2 {
            p1 = i32::min(384, p1);
            p2 = i32::min(384, p2);
        }
        q = [p1 + field2, q1, q2, p2 + field3];
    } else {
        let pack0 = ((w >> 2) & 32767) as i32;
        let pack1 = ((w >> 17) & 32767) as i32;
        q = [
            (pack0 & 15) + 16 * ((pack0 >> 8) / 11),
            (pack0 >> 4) % 176,
            (pack1 & 15) + 16 * ((pack1 >> 8) / 11),
            (pack1 >> 4) % 176,
        ];
    }

    [dequantize_to_u8(q[0], qmode),
     dequantize_to_u8(q[1], qmode),
     dequantize_to_u8(q[2], qmode),
     dequantize_to_u8(q[3], qmode)]
}

// Convert PiSP compressed raw to 16 bits, and then retain only the upper 8 bits.
pub fn uncompress(stride: usize,
		  buf_data: &[u8],
                  image_data: &mut [u8],
                  width: usize,
                  height: usize,
		  pisp_compression_mode: i32) {
    if pisp_compression_mode != COMPRESS_MODE {
	    panic!("Only PiSP mode 1 is expected");
    }
    for row in 0..height {
        let buf_row_start = row * stride;
        let buf_row_end = buf_row_start + width;
        let pix_row_start = row * width;
        let pix_row_end = pix_row_start + width;
        for (buf_chunk, pix_chunk)
            in buf_data[buf_row_start..buf_row_end].chunks_exact(8).zip(
                image_data[pix_row_start..pix_row_end].chunks_exact_mut(8))
        {
            // COMPRESS_MODE = 1, so mode & 1 is always true.
            let w0 = u32::from_le_bytes(buf_chunk[0..4].try_into().unwrap());
            let w1 = u32::from_le_bytes(buf_chunk[4..8].try_into().unwrap());
            let r0 = sub_block_function(w0);
            pix_chunk[0] = r0[0];
            pix_chunk[2] = r0[1];
            pix_chunk[4] = r0[2];
            pix_chunk[6] = r0[3];
            let r1 = sub_block_function(w1);
            pix_chunk[1] = r1[0];
            pix_chunk[3] = r1[1];
            pix_chunk[5] = r1[2];
            pix_chunk[7] = r1[3];
        }
    }
}
