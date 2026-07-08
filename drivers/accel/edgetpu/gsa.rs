// SPDX-License-Identifier: GPL-2.0
//! Safe wrappers over the `edgetpu_gsa` companion module's C ABI
//! (see `edgetpu_gsa.c` / `edgetpu_gsa.h`).
//!
//! The secure gs201 TPU firmware is authenticated and released by GSA over
//! Trusty IPC; these calls are the mainline equivalent of the AOSP janeiro
//! `mobile_firmware_gsa_authenticate()` / `reset_cpu()` path.

use kernel::{
    bindings,
    device::Device,
    error::to_result,
    prelude::*,
};

/// `enum gsa_tpu_cmd` values (must match `include/linux/gsa/gsa_tpu.h`).
pub(crate) const GSA_TPU_GET_STATE: u32 = 0;
pub(crate) const GSA_TPU_START: u32 = 1;
#[allow(dead_code)] // used by the M1b suspend/teardown path
pub(crate) const GSA_TPU_SHUTDOWN: u32 = 4;

/// `enum gsa_tpu_state` values.
pub(crate) const GSA_TPU_STATE_INACTIVE: i32 = 0;
pub(crate) const GSA_TPU_STATE_RUNNING: i32 = 2;

/// RAII handle to the resolved GSA `struct device` (holds a reference for its
/// lifetime; dropped via `edgetpu_gsa_put`).
pub(crate) struct Gsa(*mut bindings::device);

impl Gsa {
    /// Resolve the `gsa-device` phandle of `etdev`.
    pub(crate) fn get(etdev: &Device) -> Result<Self> {
        // SAFETY: `etdev.as_raw()` is a valid `struct device *` for the call.
        let gsa = unsafe { bindings::edgetpu_gsa_get(etdev.as_raw()) };
        if gsa.is_null() {
            return Err(ENODEV);
        }
        Ok(Gsa(gsa))
    }

    /// Authenticate + load the firmware. `hdr` must be at least the signed 4K
    /// header; `body_phys` is the carveout base already populated with the body.
    pub(crate) fn load_fw(&self, hdr: &[u8], body_phys: u64) -> Result {
        // SAFETY: `self.0` is a valid GSA device; the glue copies exactly 4K
        // from `hdr` (callers pass a slice of at least that size).
        to_result(unsafe {
            bindings::edgetpu_gsa_load_fw(self.0, hdr.as_ptr().cast(), body_phys)
        })
    }

    /// Issue a TPU management command; returns the new TPU state (`>= 0`).
    pub(crate) fn send_cmd(&self, cmd: u32) -> Result<i32> {
        // SAFETY: `self.0` is a valid GSA device.
        let ret = unsafe { bindings::edgetpu_gsa_send_cmd(self.0, cmd) };
        if ret < 0 {
            Err(Error::from_errno(ret))
        } else {
            Ok(ret)
        }
    }

    /// Unlock/unload a previously loaded image.
    pub(crate) fn unload(&self) -> Result {
        // SAFETY: `self.0` is a valid GSA device.
        to_result(unsafe { bindings::edgetpu_gsa_unload(self.0) })
    }
}

impl Drop for Gsa {
    fn drop(&mut self) {
        // SAFETY: `self.0` came from `edgetpu_gsa_get` and has not been put yet.
        unsafe { bindings::edgetpu_gsa_put(self.0) };
    }
}

/// Copy the firmware body into the no-map carveout at `carveout_phys`
/// (size `carveout_sz`) via a temporary write-combining mapping.
pub(crate) fn copy_body(carveout_phys: u64, carveout_sz: usize, body: &[u8]) -> Result {
    // SAFETY: `body` is a valid slice; the glue bounds-checks `len <= carveout_sz`.
    to_result(unsafe {
        bindings::edgetpu_fw_copy_body(
            carveout_phys,
            carveout_sz,
            body.as_ptr().cast(),
            body.len(),
        )
    })
}
