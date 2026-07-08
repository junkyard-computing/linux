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
//! like the KCI queues. That 1 MiB gap holds the KCI queues, the VII mailbox
//! queues, and the M2 buffer-object heap:
//!
//! ```text
//!   0x93100000  KCI cmd queue      (kci.rs)
//!   0x93110000  KCI resp queue     (kci.rs)
//!   0x93120000  KCI fw_info + OPEN_DEVICE detail scratch (kci.rs)
//!   0x93130000  VII cmd queue      (vii.rs)
//!   0x93140000  VII resp queue     (vii.rs)
//!   0x93150000  BO heap  ── grows up to ──  0x93200000 (secure_data_start)
//! ```

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

/// Buffer-object heap: from `0x93150000` up to `secure_data_start`
/// (`0x93200000`) — ~720 KiB. BOs are carveout-backed (so BO_WRITE/BO_READ keep
/// using the memremap path) but are SysMMU-mapped into the non-secure device
/// window, so the firmware reaches them via valid IOVAs. A larger heap would
/// mean backing BOs with normal pages; the carveout is enough for small models.
pub(crate) const BO_HEAP_PHYS: u64 = 0x9315_0000;
pub(crate) const BO_HEAP_END: u64 = 0x9320_0000;

/// TPU SysMMU device-address (IOVA) window for inference buffers — inside the
/// DT `dma-window` (`0x18000000..0xFFFFF000`). We hand these IOVAs to the
/// firmware; the SysMMU translates them back to the carveout backing.
pub(crate) const IOVA_BASE: u64 = 0x1800_0000;
pub(crate) const IOVA_END: u64 = 0x2000_0000;

/// Write `bytes` into the carveout at `phys` via the C glue (memremap WC).
pub(crate) fn write(phys: u64, bytes: &[u8]) -> Result {
    // SAFETY: `bytes` is a valid slice; the glue memremaps `phys` for `len`.
    to_result(unsafe { bindings::edgetpu_mem_write(phys, bytes.as_ptr().cast(), bytes.len()) })
}

/// Read `bytes.len()` from the carveout at `phys` via the C glue.
pub(crate) fn read(phys: u64, bytes: &mut [u8]) -> Result {
    // SAFETY: `bytes` is a valid mutable slice; the glue memremaps `phys`.
    to_result(unsafe { bindings::edgetpu_mem_read(phys, bytes.as_mut_ptr().cast(), bytes.len()) })
}

/// A buffer object: a page-aligned carveout range, mapped into the SysMMU at
/// `iova`. Host access (BO_WRITE/BO_READ) uses `phys`; the firmware uses `iova`.
pub(crate) struct BoEntry {
    pub(crate) phys: u64,
    pub(crate) iova: u64,
    pub(crate) size: u64,
}

/// Trivial bump allocator over the carveout BO heap + the SysMMU IOVA window.
/// No per-BO free (a whole client's BOs are reclaimed at once by [`reset`],
/// called on each DRM open), which suits the map-once/submit-many pattern.
pub(crate) struct BoAllocator {
    next_phys: u64,
    next_iova: u64,
    table: KVec<BoEntry>,
}

impl BoAllocator {
    pub(crate) fn new() -> Self {
        Self {
            next_phys: BO_HEAP_PHYS,
            next_iova: IOVA_BASE,
            table: KVec::new(),
        }
    }

    /// Allocate a page-aligned BO of `size` bytes, map its carveout backing into
    /// the TPU SysMMU domain, and return `(handle, iova)`. `dev` is the edgetpu
    /// platform device (whose default domain the IOMMU core attached).
    pub(crate) fn alloc(&mut self, size: u64, dev: *mut bindings::device) -> Result<(u32, u64)> {
        let aligned = size.checked_add(0xfff).ok_or(EINVAL)? & !0xfff;
        if aligned == 0 || self.next_phys + aligned > BO_HEAP_END {
            return Err(ENOMEM);
        }
        if self.next_iova + aligned > IOVA_END {
            return Err(ENOMEM);
        }
        let phys = self.next_phys;
        let iova = self.next_iova;
        // Map carveout phys -> IOVA so the firmware can reach the buffer.
        // SAFETY: `dev` is the live edgetpu device; the glue looks up its domain.
        to_result(unsafe {
            bindings::edgetpu_iommu_map(dev, iova, phys, aligned as usize)
        })?;
        self.next_phys += aligned;
        self.next_iova += aligned;
        let handle = self.table.len() as u32;
        self.table.push(BoEntry { phys, iova, size: aligned }, GFP_KERNEL)?;
        Ok((handle, iova))
    }

    /// Look up a BO by handle.
    pub(crate) fn get(&self, handle: u32) -> Option<&BoEntry> {
        self.table.get(handle as usize)
    }

    /// Reclaim the whole heap: unmap every BO from the SysMMU, reset the bumps.
    pub(crate) fn reset(&mut self, dev: *mut bindings::device) {
        for e in self.table.iter() {
            // SAFETY: `dev` is the live edgetpu device; iova/size were mapped here.
            unsafe { bindings::edgetpu_iommu_unmap(dev, e.iova, e.size as usize) };
        }
        self.next_phys = BO_HEAP_PHYS;
        self.next_iova = IOVA_BASE;
        self.table = KVec::new();
    }
}
