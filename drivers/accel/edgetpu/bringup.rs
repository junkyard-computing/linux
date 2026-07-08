// SPDX-License-Identifier: GPL-2.0
//! M1a firmware bring-up for the gs201 Edge TPU.
//!
//! Sequence (mirrors the AOSP janeiro `power_up` + `mobile_firmware_run`
//! path, minus the KCI handshake which needs the TPU SysMMU — that is M1b):
//!
//!   1. Block power/clock is already up (the caller `clk_set_rate`d the ACPM
//!      DVFS channel — the same rail-power primitive the mainline GPU uses).
//!   2. Walk the two Power State Machines to "done" (LPM up).
//!   3. `request_firmware` the signed image; copy its body into the carveout.
//!   4. GSA authenticates + loads the 4K signed header (Trusty-backed).
//!   5. Zero the SSMT stream-VID tables.
//!   6. `GSA_TPU_START` releases the R52 out of reset; success is the returned
//!      `GSA_TPU_STATE_RUNNING`.
//!
//! Liveness beyond STATE_RUNNING (a KCI `FW_INFO` round-trip) is deferred to
//! M1b since the mailbox queues require the (still-unported) TPU SysMMU.

use kernel::{
    device::Device,
    firmware::Firmware,
    io::{mem::IoMem, Io},
    prelude::*,
    sizes::{SZ_2M, SZ_64K},
    time::{delay::fsleep, Delta},
};

use crate::gsa::{
    self,
    Gsa,
    GSA_TPU_GET_STATE,
    GSA_TPU_START,
    GSA_TPU_STATE_INACTIVE,
    GSA_TPU_STATE_RUNNING,
};
use crate::kci::Kci;
use crate::regs;
use crate::vii::Vii;

/// Main TPU CSR block window (reg index 0).
pub(crate) type TpuRegs<'a> = IoMem<'a, SZ_2M>;
/// SSMT stream-ID table window (reg index 1).
pub(crate) type SsmtRegs<'a> = IoMem<'a, SZ_64K>;

/// Firmware image parameters (AOSP janeiro `config.h` / on-disk blob).
const FW_NAME: &CStr = c"google/edgetpu-janeiro.fw";
const FW_HEADER_SIZE: usize = 0x1000; // MOBILE_FW_HEADER_SIZE (SZ_4K)
const FW_MAGIC_OFFSET: usize = 0x400; // 'TPUF' at header+0x400
const FW_MAGIC: u32 = 0x4655_5054; // 'TPUF' little-endian
/// Firmware carveout (DT `tpu_fw@93000000`); the driver copies at most
/// `EDGETPU_FW_SIZE_MAX` = 1 MiB, matching the AOSP `fw_region_size`.
const FW_CARVEOUT_PHYS: u64 = 0x9300_0000;
const FW_CARVEOUT_SIZE: usize = 0x10_0000;

/// Poll a `PSMx_STATUS` register until the "done" bit sets (30 ms budget,
/// matching `EDGETPU_LPM_CHANGE_TIMEOUT`).
fn poll_psm_done(reg: &TpuRegs<'_>, status_off: usize) -> Result {
    for _ in 0..1500 {
        if reg.try_read32(status_off)? & regs::PSM_STATUS_DONE != 0 {
            return Ok(());
        }
        fsleep(Delta::from_micros(20));
    }
    Err(ETIMEDOUT)
}

/// Bring the two Power State Machines up (janeiro `lpm_up`).
fn lpm_up(reg: &TpuRegs<'_>) -> Result {
    reg.try_write32(regs::PSM_START_GO, regs::PSM0_START)?;
    poll_psm_done(reg, regs::PSM0_STATUS)?;

    reg.try_write32(regs::PSM_START_GO, regs::PSM1_START)?;
    poll_psm_done(reg, regs::PSM1_STATUS)?;

    // Clear the clock-gating config on both PSMs.
    reg.try_write32(0, regs::PSM0_CFG)?;
    reg.try_write32(0, regs::PSM1_CFG)?;
    Ok(())
}

/// Zero the non-secure SSMT read/write stream-VID tables (8 streams).
fn ssmt_setup(ssmt: &SsmtRegs<'_>) -> Result {
    for i in 0..regs::SSMT_NUM_STREAMS {
        ssmt.try_write32(0, regs::SSMT_NS_READ_STREAM_VID + 4 * i)?;
        ssmt.try_write32(0, regs::SSMT_NS_WRITE_STREAM_VID + 4 * i)?;
    }
    Ok(())
}

/// Authenticate, load and start the secure TPU firmware, then activate the VII
/// inference mailbox. Returns `Ok(())` only once GSA reports the TPU as
/// `RUNNING`, the KCI `FW_INFO` handshake succeeds, and the VII mailbox is
/// bound (`OPEN_DEVICE`). The `kci`/`vii` state is retained by the caller for
/// the runtime submit path.
pub(crate) fn firmware_bringup(
    dev: &Device,
    reg: &TpuRegs<'_>,
    ssmt: &SsmtRegs<'_>,
    kci: &mut Kci,
    vii: &mut Vii,
) -> Result {
    // 1. Power state machines up (rail/clock already enabled by the caller).
    lpm_up(reg)?;
    dev_info!(dev, "edgetpu: PSM/LPM up\n");

    // 2. Fetch the signed firmware image.
    let fw = Firmware::request(FW_NAME, dev)?;
    let data = fw.data();
    if data.len() < FW_HEADER_SIZE {
        dev_err!(dev, "edgetpu: firmware too small ({} bytes)\n", data.len());
        return Err(EINVAL);
    }

    let magic = u32::from_le_bytes([
        data[FW_MAGIC_OFFSET],
        data[FW_MAGIC_OFFSET + 1],
        data[FW_MAGIC_OFFSET + 2],
        data[FW_MAGIC_OFFSET + 3],
    ]);
    if magic != FW_MAGIC {
        // Warn only — matches AOSP, which does not treat this as fatal.
        dev_warn!(dev, "edgetpu: unexpected fw magic {:#010x}\n", magic);
    }

    // 3. Copy the body (everything past the 4K header) into the carveout.
    let body = &data[FW_HEADER_SIZE..];
    gsa::copy_body(FW_CARVEOUT_PHYS, FW_CARVEOUT_SIZE, body)?;
    dev_info!(dev, "edgetpu: fw body ({} bytes) staged in carveout\n", body.len());

    // 4. Resolve GSA and authenticate/load the image.
    let gsa = Gsa::get(dev)?;

    // If a previous image is still loaded, unload it first (janeiro does this).
    match gsa.send_cmd(GSA_TPU_GET_STATE) {
        Ok(state) if state > GSA_TPU_STATE_INACTIVE => {
            dev_info!(dev, "edgetpu: unloading stale fw (state {})\n", state);
            let _ = gsa.unload();
        }
        Ok(_) => {}
        Err(e) => {
            dev_err!(dev, "edgetpu: GSA GET_STATE failed: {:?}\n", e);
            return Err(e);
        }
    }

    gsa.load_fw(&data[..FW_HEADER_SIZE], FW_CARVEOUT_PHYS)?;
    dev_info!(dev, "edgetpu: GSA authenticated + loaded fw\n");

    // 5. Program SSMT stream IDs before releasing the CPU.
    ssmt_setup(ssmt)?;

    // 6. Program the KCI mailbox + open the TPU data-path S2MPU BEFORE start —
    //    the firmware latches the queue-base CSRs when it boots.
    kci.setup(reg)?;

    // 7. Release the R52 out of reset via GSA.
    let state = gsa.send_cmd(GSA_TPU_START)?;
    if state != GSA_TPU_STATE_RUNNING {
        dev_err!(dev, "edgetpu: GSA_TPU_START -> state {} (not RUNNING)\n", state);
        return Err(EIO);
    }

    dev_info!(dev, "edgetpu: *** TPU firmware RUNNING (GSA state {}) ***\n", state);

    // 8. KCI boot handshake (FW_INFO): prove the firmware is interactively
    //    processing mailbox commands, not just started. Give the R52 a moment
    //    to finish its own boot before the first command.
    fsleep(Delta::from_millis(50));
    let flavor = kci.fw_info(dev, reg)?;
    dev_info!(dev, "edgetpu: *** KCI FW_INFO ok — fw_flavor={} ***\n", flavor);

    // 9. M2: bring up + bind the VII inference mailbox (mailbox 1) via a KCI
    //    OPEN_DEVICE. Success means the firmware accepted a per-context
    //    inference queue — the substrate the SUBMIT ioctl drives.
    vii.activate(dev, reg, kci)?;

    Ok(())
}
