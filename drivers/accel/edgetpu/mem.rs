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
//! are now backed by physically-contiguous system memory (`BoAllocator`), mapped
//! into the SysMMU at IOVAs in the 128 MiB `IOVA_BASE..IOVA_END` window, so the
//! whole forward's executables can stay resident (register-once-dispatch).

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
/// 128 MiB — enough to keep a whole model forward's executables resident.
pub(crate) const IOVA_BASE: u64 = 0x1800_0000;
pub(crate) const IOVA_END: u64 = 0x2000_0000;

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

/// A buffer object: physically-contiguous system memory mapped into the SysMMU
/// at `iova`. Host access (BO_WRITE/BO_READ) memcpy's through `cpu_va` (the
/// backing pages' kernel VA); the firmware reaches it via `iova`. `size` is the
/// page-aligned allocation the SysMMU mapping and free must both use.
pub(crate) struct BoEntry {
    pub(crate) cpu_va: u64,
    pub(crate) iova: u64,
    pub(crate) size: u64,
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

/// Trivial bump allocator over the SysMMU IOVA window, backing each BO with
/// physically-contiguous system memory. No per-BO free (a whole client's BOs are
/// reclaimed at once by [`reset`], on each DRM open / RESET ioctl), which suits
/// the map-once/submit-many pattern.
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

    /// Allocate a page-aligned BO of `size` bytes backed by system memory, map it
    /// into the TPU SysMMU domain, and return `(handle, iova)`. `dev` is the
    /// edgetpu platform device (whose default domain the IOMMU core attached).
    pub(crate) fn alloc(&mut self, size: u64, dev: *mut bindings::device) -> Result<(u32, u64)> {
        let aligned = size.checked_add(0xfff).ok_or(EINVAL)? & !0xfff;
        if aligned == 0 || self.next_iova + aligned > IOVA_END {
            return Err(ENOMEM);
        }
        let iova = self.next_iova;
        // Allocate system pages + SysMMU-map them so the firmware can reach the buffer.
        // SAFETY: `dev` is the live edgetpu device; the glue allocates + maps `aligned` bytes.
        let cpu_va = unsafe { bindings::edgetpu_bo_sysmem_alloc(dev, iova, aligned as usize) };
        if cpu_va.is_null() {
            return Err(ENOMEM);
        }
        self.next_iova += aligned;
        let handle = self.table.len() as u32;
        self.table.push(BoEntry { cpu_va: cpu_va as u64, iova, size: aligned }, GFP_KERNEL)?;
        Ok((handle, iova))
    }

    /// Look up a BO by handle.
    pub(crate) fn get(&self, handle: u32) -> Option<&BoEntry> {
        self.table.get(handle as usize)
    }

    /// Reclaim the whole heap: unmap + free every BO's system memory, reset the bump.
    pub(crate) fn reset(&mut self, dev: *mut bindings::device) {
        for e in self.table.iter() {
            // SAFETY: `dev` is live; cpu_va/iova/size were returned by edgetpu_bo_sysmem_alloc.
            unsafe {
                bindings::edgetpu_bo_sysmem_free(dev, e.cpu_va as *mut _, e.iova, e.size as usize)
            };
        }
        self.next_iova = IOVA_BASE;
        self.table = KVec::new();
    }
}
