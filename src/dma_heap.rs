// Copyright (c) 2026 Steven Rosenthal smr@dt3.org
// See LICENSE file in root directory for license terms.
//
// Allocate cached, physically-contiguous DMA buffers from the kernel Contiguous
// Memory Allocator (CMA) dma-heap and use them as camera framebuffers via
// libcamera's import path.
//
// On Raspberry Pi (Bookworm) the kernel's CMA dma-heap is exposed at
// /dev/dma_heap/linux,cma (or the udev symlink /dev/dma_heap/vidbuf_cached).
// Buffers from this heap are cacheable in userspace, unlike the default
// videobuf2-dma-contig buffers libcamera allocates internally, which are
// uncached and slow to read from the CPU (~130 MB/s on Pi Zero 2W).
//
// CPU access to cached dma-buf memory must be bracketed with DMA_BUF_IOCTL_SYNC
// to keep CPU caches coherent with device DMA writes.

use std::{
    fs::{File, OpenOptions},
    io,
    os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd},
};

// From linux/dma-heap.h.
#[repr(C)]
struct DmaHeapAllocationData {
    len: u64,
    fd: u32,
    fd_flags: u32,
    heap_flags: u64,
}

// From linux/dma-buf.h.
#[repr(C)]
struct DmaBufSync {
    flags: u64,
}

const DMA_BUF_SYNC_READ: u64 = 1 << 0;
const DMA_BUF_SYNC_START: u64 = 0 << 2;
const DMA_BUF_SYNC_END: u64 = 1 << 2;

// _IOWR('H', 0, struct dma_heap_allocation_data)
// _IOW('b', 0, struct dma_buf_sync)
// Encoded manually to avoid pulling in a full ioctl crate.
const DMA_HEAP_IOCTL_ALLOC: libc::c_ulong = 0xc018_4800;
const DMA_BUF_IOCTL_SYNC: libc::c_ulong = 0x4008_6200;

/// Handle to an open dma-heap device.
pub struct DmaHeap {
    file: File,
}

impl DmaHeap {
    /// Open the kernel CMA dma-heap. Tries the udev symlink first, then the
    /// canonical device name. Returns an error if neither is present or the
    /// caller lacks permission (typically the `video` group).
    pub fn open_cma() -> io::Result<Self> {
        for path in &["/dev/dma_heap/vidbuf_cached", "/dev/dma_heap/linux,cma"] {
            match OpenOptions::new().read(true).write(true).open(path) {
                Ok(file) => {
                    log::info!("Opened dma-heap at {}", path);
                    return Ok(Self { file });
                }
                Err(e) if e.kind() == io::ErrorKind::NotFound => continue,
                Err(e) => return Err(e),
            }
        }
        Err(io::Error::new(
            io::ErrorKind::NotFound,
            "No dma-heap device found (looked for /dev/dma_heap/vidbuf_cached and /dev/dma_heap/linux,cma)",
        ))
    }

    /// Allocate a contiguous buffer of `len` bytes. Returns an owned dma-buf fd
    /// that can be mmap'd (cached) and imported into libcamera.
    pub fn alloc(&self, len: usize) -> io::Result<OwnedFd> {
        let mut req = DmaHeapAllocationData {
            len: len as u64,
            fd: 0,
            fd_flags: (libc::O_RDWR | libc::O_CLOEXEC) as u32,
            heap_flags: 0,
        };
        let ret = unsafe {
            libc::ioctl(self.file.as_raw_fd(), DMA_HEAP_IOCTL_ALLOC, &mut req)
        };
        if ret < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(unsafe { OwnedFd::from_raw_fd(req.fd as RawFd) })
    }
}

/// RAII guard that brackets CPU read access to a dma-buf with the required
/// cache-coherency ioctls. Calls `DMA_BUF_IOCTL_SYNC START|READ` on
/// construction and `DMA_BUF_IOCTL_SYNC END|READ` on drop.
pub struct DmaBufReadGuard(RawFd);

impl DmaBufReadGuard {
    pub fn new(fd: RawFd) -> io::Result<Self> {
        let s = DmaBufSync { flags: DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ };
        let ret = unsafe { libc::ioctl(fd, DMA_BUF_IOCTL_SYNC, &s) };
        if ret < 0 { Err(io::Error::last_os_error()) } else { Ok(Self(fd)) }
    }
}

impl Drop for DmaBufReadGuard {
    fn drop(&mut self) {
        let s = DmaBufSync { flags: DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ };
        let ret = unsafe { libc::ioctl(self.0, DMA_BUF_IOCTL_SYNC, &s) };
        if ret < 0 {
            log::warn!("DMA_BUF_IOCTL_SYNC END failed: {:?}",
                       std::io::Error::last_os_error());
        }
    }
}