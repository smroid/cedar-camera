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

fn postprocess(mut a: u16) -> u16 {
    if COMPRESS_MODE & 2 != 0 {
        if COMPRESS_MODE == 3 && a < 0x4000 {
            a >>= 2;
        } else if a < 0x1000 {
            a >>= 4;
        } else if a < 0x1800 {
            a = (a - 0x800) >> 3;
        } else if a < 0x3000 {
            a = (a - 0x1000) >> 2;
        } else if a < 0x6000 {
            a = (a - 0x2000) >> 1;
        } else if a < 0xC000 {
            a -= 0x4000;
        } else {
            a = 2 * (a - 0x8000);
        }
    }
    u32::min(0xFFFF, a as u32 + COMPRESS_OFFSET) as u16
}

fn dequantize(q: u16, qmode: i32) -> u16 {
    match qmode {
        0 => if q < 320 { 16 * q } else { 32 * (q - 160) },
        1 => 64 * q,
        2 => 128 * q,
        _ => if q < 94 { 256 * q } else { u16::min(0xFFFF, 512 * (q - 47)) },
    }
}

fn sub_block_function(w: u32) -> [u16; 4] {
    let mut q = [0; 4];
    let qmode = (w & 3) as i32;

    if qmode < 3 {
        let field0 = ((w >> 2) & 511) as i32;
        let field1 = ((w >> 11) & 127) as i32;
        let field2 = ((w >> 18) & 127) as i32;
        let field3 = ((w >> 25) & 127) as i32;
        if qmode == 2 && field0 >= 384 {
            q[1] = field0;
            q[2] = field1 + 384;
        } else {
            q[1] = if field1 >= 64 { field0 } else { field0 + 64 - field1 };
            q[2] = if field1 >= 64 { field0 + field1 - 64 } else { field0 };
        }
        let mut p1 = i32::max(0, q[1] - 64);
        if qmode == 2 {
            p1 = i32::min(384, p1);
        }
        let mut p2 = i32::max(0, q[2] - 64);
        if qmode == 2 {
            p2 = i32::min(384, p2);
        }
        q[0] = p1 + field2;
        q[3] = p2 + field3;
    } else {
        let pack0 = ((w >> 2) & 32767) as i32;
        let pack1 = ((w >> 17) & 32767) as i32;
        q[0] = (pack0 & 15) + 16 * ((pack0 >> 8) / 11);
        q[1] = (pack0 >> 4) % 176;
        q[2] = (pack1 & 15) + 16 * ((pack1 >> 8) / 11);
        q[3] = (pack1 >> 4) % 176;
    }

    [dequantize(q[0] as u16, qmode),
     dequantize(q[1] as u16, qmode),
     dequantize(q[2] as u16, qmode),
     dequantize(q[3] as u16, qmode),
    ]
}

// Convert PiSP compressed raw to 16 bits, and then retain only the upper 8 bits.
pub fn uncompress(stride: usize,
		  buf_data: &[u8],
                  image_data: &mut [u8],
                  width: usize,
                  height: usize,
		  pisp_compression_mode: i32) {
    if pisp_compression_mode != COMPRESS_MODE {
	// Although we do have the logic for modes 2 and 3.
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
	    if pisp_compression_mode & 1 != 0 {
		let mut w0 = 0u32;
                let mut w1 = 0u32;
                for b in 0..4 {
		    w0 |= (buf_chunk[b] as u32) << (b * 8);
                }
                for b in 0..4 {
		    w1 |= (buf_chunk[4 + b] as u32) << (b * 8);
                }
                let r0 = sub_block_function(w0);
		pix_chunk[0] = (postprocess(r0[0]) >> 8) as u8;
		pix_chunk[2] = (postprocess(r0[1]) >> 8) as u8;
		pix_chunk[4] = (postprocess(r0[2]) >> 8) as u8;
		pix_chunk[6] = (postprocess(r0[3]) >> 8) as u8;
                let r1 = sub_block_function(w1);
		pix_chunk[1] = (postprocess(r1[0]) >> 8) as u8;
		pix_chunk[3] = (postprocess(r1[1]) >> 8) as u8;
		pix_chunk[5] = (postprocess(r1[2]) >> 8) as u8;
		pix_chunk[7] = (postprocess(r1[3]) >> 8) as u8;
	    } else {
		for i in 0..8 {
		    pix_chunk[i] = (postprocess((buf_chunk[i] as u16) << 8) >> 8) as u8;
		}
	    }
        }
    }
}
