// SPDX-License-Identifier: GPL-2.0
//! Carveout memory + device-VA helpers for the Edge TPU accel driver.
//!
//! The firmware carveout (`tpu_fw@93000000`, 16 MiB) is reached by the TPU
//! through the GSA-configured instruction remap: TPU-VA `0x10000000` maps to
//! phys `0x93000000` for the full 16 MiB (decoded from the firmware
//! `image_config` iommu-mapping[0], size-field `0xc` → `(1<<12)<<12` = 16 MiB).
//!
//! Within that window only the region between the end of the firmware body and
//! the firmware's own `secure_data_start` (phys `0x93200000`) is free for the
//! driver to use — the AP and the TPU both reach it without the SysMMU, exactly
//! like the KCI queues. That 1 MiB gap holds the KCI queues and the VII mailbox
//! queues:
//!
//! ```text
//!   0x93100000  KCI cmd queue      (kci.rs)
//!   0x93110000  KCI resp queue     (kci.rs)
//!   0x93120000  KCI fw_info + OPEN_DEVICE detail scratch (kci.rs)
//!   0x93130000  VII cmd queue      (vii.rs)
//!   0x93140000  VII resp queue     (vii.rs)
//!   0x93150000..0x93200000  free (was the carveout BO heap; now spare — a
//!                           second VII mailbox's queues can live here)
//! ```
//!
//! Buffer objects are NO LONGER carveout-backed: the ~704 KiB gap could not hold
//! a whole model forward's executables, forcing a VCID recycle every ~35 ops. BOs
//! are now backed by system memory (`BoAllocator`; pages need not be physically
//! contiguous), each mapped into the SysMMU as one contiguous range in the
//! `IOVA_BASE..IOVA_END` window, so the whole forward's executables can stay
//! resident (register-once-dispatch).

use kernel::{
    alloc::KVec,
    bindings,
    error::to_result,
    prelude::*,
};

/// Instruction-remap: TPU-VA `0x10000000` → phys `0x93000000` (firmware_base).
pub(crate) const CARVEOUT_PHYS_BASE: u64 = 0x9300_0000;
pub(crate) const TPU_REMAP_BASE: u64 = 0x1000_0000;

/// Translate a carveout physical address to the TPU-side virtual address the
/// firmware sees through the instruction remap.
pub(crate) const fn tpu_va(phys: u64) -> u64 {
    phys - CARVEOUT_PHYS_BASE + TPU_REMAP_BASE
}

/// TPU SysMMU device-address (IOVA) window for inference buffers — inside the
/// DT `dma-window` (`0x18000000..0xFFFFF000`). We hand these IOVAs to the
/// firmware; the SysMMU translates them back to each BO's system-memory backing.
///
/// The bump is monotonic across clients (never reused until it wraps at
/// [`IOVA_END`], see `BoAllocator::reset`), because the firmware deduplicates a
/// graph registration by (IOVA, executable content) and hands back the *previous*
/// session's token — which still references that session's now-freed backing page
/// — when the same executable is re-registered at the same IOVA. Distinct IOVAs
/// per session dodge the stale dedup. A wrap re-poisons only if the exact same
/// executable lands on a recycled IOVA, so the window is as large as the DT allows:
/// 3.1 GiB holds ~280 sessions of an ~11 MiB finch forward, or ~27 of the ~114 MiB
/// Qwen3-235B predictor set, before wrapping.
///
/// The top 512 MiB of the `dma-window` stay with the DMA API: attaching an imported
/// dma-buf still makes iommu-dma map it (top-down from the window end), although the
/// driver hands the firmware its own bump mapping instead (see `edgetpu_gsa.h`).
pub(crate) const IOVA_BASE: u64 = 0x1800_0000;
pub(crate) const IOVA_END: u64 = 0xE000_0000;

/// Map the carveout data region (KCI/VII queues + BO heap) once, so runtime
/// [`write`]/[`read`] memcpy through a persistent mapping instead of a
/// memremap()/memunmap() per access. Idempotent; call once during probe before
/// any queue access.
pub(crate) fn init() -> Result {
    // SAFETY: FFI to the companion module; idempotent, no arguments.
    to_result(unsafe { bindings::edgetpu_data_init() })
}

/// Register the (inactive) MIF/INT bus bandwidth votes. The firmware DMAs every
/// inference op through memory and the bus governor can't see that load, so
/// without a vote the buses idle at ~13% and every op runs ~5x slower. The votes
/// are raised/dropped per attached client by [`bus_qos_get`]/[`bus_qos_put`].
/// Best-effort (never fails probe); call once during probe.
pub(crate) fn bus_qos_init() {
    // SAFETY: FFI to the companion module; idempotent, best-effort, no arguments.
    unsafe { bindings::edgetpu_bus_qos_init() };
}

/// Raise MIF/INT to max for an attaching client (refcounted; only the first
/// client actually raises the buses).
pub(crate) fn bus_qos_get() {
    // SAFETY: FFI to the companion module; internally refcounted, no arguments.
    unsafe { bindings::edgetpu_bus_qos_get() };
}

/// Drop the MIF/INT vote for a detaching client (only the last client drops the
/// buses back to the governor's floor).
pub(crate) fn bus_qos_put() {
    // SAFETY: FFI to the companion module; internally refcounted, no arguments.
    unsafe { bindings::edgetpu_bus_qos_put() };
}

/// Raise the TPU DVFS clock to its active rate for an attaching client
/// (refcounted; only the first client actually raises it). Bring-up leaves the
/// clock at the DVFS floor to protect the boot UVLO budget, so without this
/// every inference runs at ~1/4.7 of NOM.
pub(crate) fn dvfs_vote_get() {
    // SAFETY: FFI to the companion module; internally refcounted, no arguments.
    unsafe { bindings::edgetpu_dvfs_vote_get() };
}

/// Drop this client's TPU DVFS vote (only the last client returns the clock to
/// the boot floor, so an idle TPU costs no extra power).
pub(crate) fn dvfs_vote_put() {
    // SAFETY: FFI to the companion module; internally refcounted, no arguments.
    unsafe { bindings::edgetpu_dvfs_vote_put() };
}

/// Write `bytes` into the carveout at `phys` via the C glue (persistent WC map).
/// Used for the KCI/VII mailbox queues (BOs use [`BoEntry`] system memory now).
pub(crate) fn write(phys: u64, bytes: &[u8]) -> Result {
    // SAFETY: `bytes` is a valid slice; the glue memremaps `phys` for `len`.
    to_result(unsafe { bindings::edgetpu_mem_write(phys, bytes.as_ptr().cast(), bytes.len()) })
}

/// Read `bytes.len()` from the carveout at `phys` via the C glue.
pub(crate) fn read(phys: u64, bytes: &mut [u8]) -> Result {
    // SAFETY: `bytes` is a valid mutable slice; the glue memremaps `phys`.
    to_result(unsafe { bindings::edgetpu_mem_read(phys, bytes.as_mut_ptr().cast(), bytes.len()) })
}

/// A buffer object, mapped into the SysMMU as one contiguous range at `iova`.
/// An allocated BO ([`BoAllocator::alloc`]) is system memory that need not be
/// physically contiguous; host access (BO_WRITE/BO_READ) memcpy's through `cpu_va`, a kernel
/// vmap of its pages, and `sysmem` is the C glue's handle for freeing it. An imported dma-buf
/// ([`BoAllocator::import`]) has `cpu_va == 0` and a non-zero `import`, the glue's handle for
/// releasing it; the host reaches it through the dma-buf's own mmap, never BO_WRITE/BO_READ.
/// `size` is the page-aligned length mapped at `iova`.
pub(crate) struct BoEntry {
    pub(crate) cpu_va: u64,
    pub(crate) iova: u64,
    pub(crate) size: u64,
    pub(crate) sysmem: usize,
    pub(crate) import: usize,
}

/// Copy `bytes` into the system-memory BO at kernel VA `cpu_va` + `offset`.
pub(crate) fn bo_write(cpu_va: u64, offset: u64, bytes: &[u8]) {
    // SAFETY: `cpu_va`+`offset`..+len is inside a live BO (bounds-checked by the
    // caller against `BoEntry::size`); `bytes` is a valid slice.
    unsafe {
        bindings::edgetpu_bo_write_bytes(cpu_va as *mut _, offset, bytes.as_ptr().cast(), bytes.len())
    };
}

/// Copy `bytes.len()` out of the system-memory BO at `cpu_va` + `offset`.
pub(crate) fn bo_read(cpu_va: u64, offset: u64, bytes: &mut [u8]) {
    // SAFETY: same in-bounds guarantee as [`bo_write`]; `bytes` is a valid slice.
    unsafe {
        bindings::edgetpu_bo_read_bytes(cpu_va as *const _, offset, bytes.as_mut_ptr().cast(), bytes.len())
    };
}

/// Trivial bump allocator over the SysMMU IOVA window, for both allocated BOs and imported
/// dma-bufs. No per-BO free (a whole client's BOs are reclaimed at once by [`reset`], on each
/// DRM open / RESET ioctl), which suits the map-once/submit-many pattern.
pub(crate) struct BoAllocator {
    next_iova: u64,
    table: KVec<BoEntry>,
}

impl BoAllocator {
    pub(crate) fn new() -> Self {
        Self {
            next_iova: IOVA_BASE,
            table: KVec::new(),
        }
    }

    /// Reserve `aligned` bytes of IOVA from the monotonic bump. It wraps only when this
    /// reservation would overrun the window; it is NOT reset per client (see [`reset`]), so
    /// distinct sessions get distinct IOVAs — the firmware otherwise serves a previous session's
    /// registration (and its freed backing) when an IOVA is reused across an fd close/reopen.
    ///
    /// The start is aligned to the largest SysMMU entry the buffer can use (1 MiB section for a
    /// buffer of at least 1 MiB, 64 KiB large page from 64 KiB), so physically contiguous chunks
    /// of the backing map with large entries: `iommu_map` only uses one where the IOVA and the
    /// physical address are both aligned to it. With a 4 KiB-aligned start every BO was mapped
    /// with 4 KiB entries, and streaming a multi-MB executable's weights per run walked the
    /// tables far more often than the DMA API's size-aligned mappings on AOSP do.
    fn reserve(&mut self, aligned: u64) -> Result<u64> {
        if aligned == 0 || aligned > IOVA_END - IOVA_BASE {
            return Err(EINVAL);
        }
        let align: u64 = if aligned >= 0x10_0000 { 0x10_0000 } else if aligned >= 0x1_0000 { 0x1_0000 } else { 0x1000 };
        let mut iova = (self.next_iova + align - 1) & !(align - 1);
        if iova + aligned > IOVA_END {
            iova = IOVA_BASE;
        }
        // Callers advance the bump past `aligned` bytes from here once the mapping succeeds.
        self.next_iova = iova;
        Ok(iova)
    }

    /// Allocate a page-aligned BO of `size` bytes backed by system memory, map it
    /// into the TPU SysMMU domain, and return `(handle, iova)`. `dev` is the
    /// edgetpu platform device (whose default domain the IOMMU core attached).
    pub(crate) fn alloc(&mut self, size: u64, dev: *mut bindings::device) -> Result<(u32, u64)> {
        let aligned = size.checked_add(0xfff).ok_or(EINVAL)? & !0xfff;
        let iova = self.reserve(aligned)?;
        let mut sysmem: *mut core::ffi::c_void = core::ptr::null_mut();
        // Allocate system pages + SysMMU-map them so the firmware can reach the buffer.
        // SAFETY: `dev` is the live edgetpu device; the glue allocates + maps `aligned` bytes at
        // the reserved `iova` and writes its handle to the valid local `sysmem`.
        let cpu_va = unsafe { bindings::edgetpu_bo_sysmem_alloc(dev, iova, aligned as usize, &mut sysmem) };
        if cpu_va.is_null() {
            return Err(ENOMEM);
        }
        self.next_iova += aligned;
        let handle = self.table.len() as u32;
        let entry = BoEntry { cpu_va: cpu_va as u64, iova, size: aligned, sysmem: sysmem as usize, import: 0 };
        if let Err(e) = self.table.push(entry, GFP_KERNEL) {
            // SAFETY: `sysmem` came from edgetpu_bo_sysmem_alloc above and was not stored anywhere.
            unsafe { bindings::edgetpu_bo_sysmem_free(dev, sysmem, iova, aligned as usize) };
            return Err(e.into());
        }
        Ok((handle, iova))
    }

    /// Import the dma-buf behind `fd` as a BO and return `(handle, iova, size)`. The glue attaches
    /// (pins) it; it is then mapped at an IOVA from the same monotonic bump as [`alloc`], with the
    /// same attributes, so an import gets a fresh IOVA every session just like a BO does.
    pub(crate) fn import(&mut self, fd: i32, dev: *mut bindings::device) -> Result<(u32, u64, u64)> {
        let mut handle: *mut core::ffi::c_void = core::ptr::null_mut();
        let mut size: u64 = 0;
        // SAFETY: `dev` is the live edgetpu device; the out-pointers are valid locals.
        to_result(unsafe { bindings::edgetpu_dmabuf_import(dev, fd, &mut handle, &mut size) })?;
        let release = |h: *mut core::ffi::c_void| {
            // SAFETY: `h` came from edgetpu_dmabuf_import and is not stored anywhere.
            unsafe { bindings::edgetpu_dmabuf_release(h) }
        };
        let iova = match self.reserve(size) {
            Ok(iova) => iova,
            Err(e) => {
                release(handle);
                return Err(e);
            }
        };
        // SAFETY: `handle` is a live import; `size` bytes at `iova` are reserved for it.
        if let Err(e) = to_result(unsafe { bindings::edgetpu_dmabuf_map(handle, iova) }) {
            release(handle);
            return Err(e);
        }
        self.next_iova += size;
        let bo = self.table.len() as u32;
        if let Err(e) = self.table.push(BoEntry { cpu_va: 0, iova, size, sysmem: 0, import: handle as usize }, GFP_KERNEL) {
            release(handle);
            return Err(e.into());
        }
        Ok((bo, iova, size))
    }

    /// Look up a BO by handle.
    pub(crate) fn get(&self, handle: u32) -> Option<&BoEntry> {
        self.table.get(handle as usize)
    }

    /// Reclaim the whole heap: unmap + free every BO and release every import. The IOVA bump is
    /// deliberately NOT reset — a fresh client continues from where the last left off (wrapping in
    /// [`reserve`] only at window-end), so an IOVA is not reused across an fd close/reopen. Reusing
    /// one there makes the TPU read stale content from the freed page (register-once keeps a
    /// forward's BOs resident, so the reused-IOVA window is a whole model of buffers), producing
    /// wrong output on the 2nd+ session against one driver load.
    pub(crate) fn reset(&mut self, dev: *mut bindings::device) {
        for e in self.table.iter() {
            if e.import != 0 {
                // SAFETY: `import` came from edgetpu_dmabuf_import and is released exactly once,
                // here, before the table is dropped.
                unsafe { bindings::edgetpu_dmabuf_release(e.import as *mut _) };
                continue;
            }
            // SAFETY: `dev` is live; sysmem/iova/size were returned by edgetpu_bo_sysmem_alloc.
            unsafe {
                bindings::edgetpu_bo_sysmem_free(dev, e.sysmem as *mut _, e.iova, e.size as usize)
            };
        }
        self.table = KVec::new();
    }
}
