// SPDX-License-Identifier: GPL-2.0
//! gs201 Edge TPU control-register offsets.
//!
//! Offsets are relative to the reg resources of `edgetpu@1ce00000`:
//!   * index 0 ("tpu")  — the main TPU CSR block (0x1ce00000, 2 MiB window).
//!   * index 1 ("ssmt") — the SSMT stream-ID table (0x1ccf0000, 64 KiB).
//!
//! Values verified against the AOSP janeiro reference (`janeiro-pm.c`,
//! `mobile-firmware.c`, `config-tpu-cpu.h`). Only the subset needed for the
//! M1a firmware bring-up is defined here; the KCI mailbox CSRs land in M1b.

// --- Main TPU CSR block (reg index 0) ---------------------------------------

/// Power State Machine 0 config / start / status (janeiro-pm.c).
pub(crate) const PSM0_CFG: usize = 0x1c1880;
pub(crate) const PSM0_START: usize = 0x1c1884;
pub(crate) const PSM0_STATUS: usize = 0x1c1888;

/// Power State Machine 1 config / start / status.
pub(crate) const PSM1_CFG: usize = 0x1c2880;
pub(crate) const PSM1_START: usize = 0x1c2884;
pub(crate) const PSM1_STATUS: usize = 0x1c2888;

/// `PSMx_STATUS` bit set once the state machine reaches "done" (bit 7).
pub(crate) const PSM_STATUS_DONE: u32 = 0x80;

/// Write `1` to `PSMx_START` to kick the state machine.
pub(crate) const PSM_START_GO: u32 = 0x1;

// --- SSMT stream-ID table (reg index 1) -------------------------------------

/// Non-secure read / write stream-VID table bases (`mobile-firmware.c`),
/// indexed `base + 4 * stream`.
pub(crate) const SSMT_NS_READ_STREAM_VID: usize = 0x1000;
pub(crate) const SSMT_NS_WRITE_STREAM_VID: usize = 0x1200;

/// Number of TPU contexts / SSMT streams (`EDGETPU_NCONTEXTS`).
pub(crate) const SSMT_NUM_STREAMS: usize = 8;
