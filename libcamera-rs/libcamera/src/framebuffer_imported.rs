use std::{
    io,
    os::fd::{AsRawFd, OwnedFd},
    ptr::NonNull,
};

use libcamera_sys::*;

use crate::framebuffer::AsFrameBuffer;

/// FrameBuffer constructed from an externally-allocated dma-buf fd (e.g. from
/// /dev/dma_heap/linux,cma). Owns both the underlying libcamera::FrameBuffer
/// (heap-allocated, freed on drop) and the dma-buf fd.
pub struct ImportedFrameBuffer {
    ptr: NonNull<libcamera_framebuffer_t>,
    _fd: OwnedFd,
    length: usize,
}

unsafe impl Send for ImportedFrameBuffer {}

impl ImportedFrameBuffer {
    /// Construct a single-plane FrameBuffer wrapping `fd` at the given offset and length.
    /// The fd is duplicated by libcamera internally; this wrapper retains ownership of the
    /// original via `OwnedFd` so the dma-buf stays alive at least as long as the wrapper.
    pub fn new(fd: OwnedFd, offset: usize, length: usize, cookie: u64) -> io::Result<Self> {
        let raw_fd = fd.as_raw_fd();
        let ptr = unsafe {
            libcamera_framebuffer_create_single_plane(raw_fd, offset, length, cookie)
        };
        let ptr = NonNull::new(ptr).ok_or_else(|| {
            io::Error::new(io::ErrorKind::Other, "libcamera_framebuffer_create_single_plane failed")
        })?;

        // Initialize metadata status sentinel; see FrameBufferAllocator for rationale.
        unsafe {
            libcamera_framebuffer_metadata(ptr.as_ptr())
                .cast_mut()
                .cast::<u32>()
                .write(u32::MAX);
        }

        Ok(Self { ptr, _fd: fd, length })
    }

    pub fn raw_fd(&self) -> i32 {
        self._fd.as_raw_fd()
    }

    pub fn length(&self) -> usize {
        self.length
    }
}

impl core::fmt::Debug for ImportedFrameBuffer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ImportedFrameBuffer")
            .field("fd", &self._fd.as_raw_fd())
            .field("length", &self.length)
            .field("metadata", &self.metadata())
            .field("planes", &self.planes())
            .finish()
    }
}

impl AsFrameBuffer for ImportedFrameBuffer {
    unsafe fn ptr(&self) -> NonNull<libcamera_framebuffer_t> {
        self.ptr
    }
}

impl Drop for ImportedFrameBuffer {
    fn drop(&mut self) {
        unsafe { libcamera_framebuffer_destroy(self.ptr.as_ptr()) };
        // _fd dropped after.
    }
}