// SPDX-License-Identifier: GPL-2.0
//! Persistent TPU CSR access for the runtime (post-probe) mailbox path.
//!
//! The Rust probe maps the main TPU CSR block as an `IoMem` that only lives for
//! the duration of `probe()`. The VII inference mailbox is driven from ioctls
//! long after that, so those accesses go through a module-lifetime `ioremap()`
//! kept in the `edgetpu_gsa` C companion. [`init`] is idempotent and must be
//! called once from the single-threaded probe path before any [`read`]/[`write`].

use kernel::{
    bindings,
    error::to_result,
    prelude::*,
};

/// Map the TPU CSR block (idempotent). Call once during probe.
pub(crate) fn init() -> Result {
    // SAFETY: FFI to the companion module; idempotent, no arguments.
    to_result(unsafe { bindings::edgetpu_csr_init() })
}

/// Read a 32-bit CSR at `off` bytes from the TPU CSR block base.
pub(crate) fn read(off: usize) -> u32 {
    // SAFETY: FFI; the C side bounds-checks `off` and returns 0 if unmapped.
    unsafe { bindings::edgetpu_csr_read32(off as u32) }
}

/// Write a 32-bit CSR at `off` bytes from the TPU CSR block base.
pub(crate) fn write(off: usize, val: u32) {
    // SAFETY: FFI; the C side bounds-checks `off` and no-ops if unmapped.
    unsafe { bindings::edgetpu_csr_write32(off as u32, val) }
}
