// SPDX-License-Identifier: GPL-2.0
//! Google Edge TPU (gs201 / Tensor G2) compute-accelerator driver.
//!
//! A clean-room mainline DRM `accel` driver for the on-SoC Edge TPU found in the
//! Pixel Fold (felix). It reuses the hardware knowledge of Google's out-of-tree
//! `edgetpu`/`rio` stack (register map, KCI mailbox protocol, firmware handshake,
//! LPM/PSM power sequence) but none of its GCIP/Android structure. Userspace (the
//! `finch` runtime) builds the DarwiNN/VII command payloads.
//!
//! M0: platform bind + `/dev/accel/accelN` registration.
//! M1a: authenticate + start the secure firmware via the (now-ported) GSA/Trusty
//! path — power the ACPM rail, walk the PSMs, `request_firmware`, GSA-authenticate
//! the signed image and `GSA_TPU_START` the R52. The KCI handshake + UAPI land in
//! M1b/M2 once the TPU SysMMU is ported.

use crate::driver::EdgeTpuPlatformDriver;

mod bringup;
mod csr;
mod driver;
mod file;
mod gem;
mod gsa;
mod ioctl;
mod kci;
mod mem;
mod regs;
mod vii;

kernel::module_platform_driver! {
    type: EdgeTpuPlatformDriver,
    name: "edgetpu",
    authors: ["Junkyard Computing"],
    description: "Google Edge TPU (gs201) accelerator driver",
    license: "GPL",
}
