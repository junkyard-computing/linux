// SPDX-License-Identifier: GPL-2.0
//! Minimal KCI (Kernel Control Interface) mailbox — the boot handshake.
//!
//! KCI is mailbox index 0. Its cmd/resp circular queues live in the firmware
//! carveout's "remapped data region" (TPU-VA `0x10100000+`, phys `0x93100000+`),
//! which the TPU reaches through the GSA-configured instruction remap — no
//! SysMMU/IOMMU mapping is needed. We place the queues + a FW_INFO scratch
//! buffer at fixed offsets in that region, program the mailbox CSRs with the
//! TPU-side addresses, push one `FIRMWARE_INFO` command and read the response.
//! A valid response proves the firmware is interactively processing commands.
//!
//! Layout / protocol verified against the AOSP janeiro `edgetpu-kci.c` +
//! `edgetpu-mailbox.c` (pre-gcip generation).

use kernel::{
    bindings,
    device::Device,
    error::to_result,
    io::Io,
    prelude::*,
    time::{delay::fsleep, Delta},
};

use crate::bringup::TpuRegs;

// --- Mailbox CSR bases (offsets from the main TPU CSR block, reg index 0) ----
const KCI_CTX_BASE: usize = 0xa0000; // context CSRs (mailbox 0)
const KCI_CMD_BASE: usize = 0xa1000; // cmd-queue CSRs
const KCI_RESP_BASE: usize = 0xa1800; // resp-queue CSRs

// Context CSR offsets (from KCI_CTX_BASE).
const CTX_ENABLE: usize = 0x00;
const CTX_CMD_Q_DOORBELL_ENABLE: usize = 0x08;
const CTX_CMD_Q_DOORBELL_CLEAR: usize = 0x10;
const CTX_CMD_Q_ADDR_LO: usize = 0x14;
const CTX_CMD_Q_ADDR_HI: usize = 0x18;
const CTX_CMD_Q_SIZE: usize = 0x1c;
const CTX_RESP_Q_DOORBELL_ENABLE: usize = 0x20;
const CTX_RESP_Q_ADDR_LO: usize = 0x28;
const CTX_RESP_Q_ADDR_HI: usize = 0x2c;
const CTX_RESP_Q_SIZE: usize = 0x30;

// Cmd-queue CSR offsets (from KCI_CMD_BASE).
const CMD_DOORBELL_SET: usize = 0x00;
const CMD_HEAD: usize = 0x08;
const CMD_TAIL: usize = 0x0c;

// Resp-queue CSR offsets (from KCI_RESP_BASE).
const RESP_DOORBELL_CLEAR: usize = 0x04;
const RESP_HEAD: usize = 0x0c;
const RESP_TAIL: usize = 0x10;

// --- Queue geometry ---------------------------------------------------------
const QUEUE_SIZE: u32 = 1023; // elements; wrap bit = 0x400
const CMD_ELEM: usize = 32;
const RESP_ELEM: usize = 16;

// --- Carveout placement -----------------------------------------------------
// Instruction remap: TPU-VA 0x10000000 -> phys 0x93000000 (firmware_base).
const CARVEOUT_PHYS_BASE: u64 = 0x9300_0000;
const TPU_REMAP_BASE: u64 = 0x1000_0000;

// Queues + FW_INFO buffer at AOSP's exact remapped-data base (0x10100000 =
// phys 0x93100000, right after the ~1 MiB firmware body) — this is inside the
// firmware's mapped region; higher addresses (0x10200000) sat at its edge.
const CMD_Q_PHYS: u64 = 0x9310_0000; // TPU-VA 0x10100000
const RESP_Q_PHYS: u64 = 0x9311_0000; // TPU-VA 0x10110000
const FW_INFO_PHYS: u64 = 0x9312_0000; // TPU-VA 0x10120000
const FW_INFO_SIZE: usize = 56;

// --- KCI protocol constants -------------------------------------------------
const KCI_CODE_FIRMWARE_INFO: u16 = 11;
const KCI_ERROR_OK: u16 = 0;
const KCI_ERROR_UNIMPLEMENTED: u16 = 12;

/// The TPU's own S2MPU (s2mpu_tpu@1cc60000, gs201 == S2MPU_V1). GSA sets up the
/// instruction path so the firmware runs, but the firmware's *data* reads of the
/// AP-placed KCI queues go through this S2MPU — left enforcing, it reads the
/// queues as zeros. Open it allow-all before the handshake.
const TPU_S2MPU_BASE: u64 = 0x1cc6_0000;

const fn tpu_va(phys: u64) -> u64 {
    phys - CARVEOUT_PHYS_BASE + TPU_REMAP_BASE
}

/// Program the KCI mailbox context + queue CSRs and enable it.
fn init_mailbox(reg: &TpuRegs<'_>) -> Result {
    let cmd_va = tpu_va(CMD_Q_PHYS);
    let resp_va = tpu_va(RESP_Q_PHYS);

    // Cmd queue.
    reg.try_write32((cmd_va & 0xffff_ffff) as u32, KCI_CTX_BASE + CTX_CMD_Q_ADDR_LO)?;
    reg.try_write32((cmd_va >> 32) as u32, KCI_CTX_BASE + CTX_CMD_Q_ADDR_HI)?;
    reg.try_write32(QUEUE_SIZE, KCI_CTX_BASE + CTX_CMD_Q_SIZE)?;
    reg.try_write32(0, KCI_CMD_BASE + CMD_TAIL)?;
    reg.try_write32(0, KCI_CMD_BASE + CMD_HEAD)?;

    // Resp queue.
    reg.try_write32((resp_va & 0xffff_ffff) as u32, KCI_CTX_BASE + CTX_RESP_Q_ADDR_LO)?;
    reg.try_write32((resp_va >> 32) as u32, KCI_CTX_BASE + CTX_RESP_Q_ADDR_HI)?;
    reg.try_write32(QUEUE_SIZE, KCI_CTX_BASE + CTX_RESP_Q_SIZE)?;
    reg.try_write32(0, KCI_RESP_BASE + RESP_HEAD)?;
    reg.try_write32(0, KCI_RESP_BASE + RESP_TAIL)?;

    // Clear stale doorbells, enable doorbells, enable the mailbox context.
    reg.try_write32(1, KCI_RESP_BASE + RESP_DOORBELL_CLEAR)?;
    reg.try_write32(1, KCI_CTX_BASE + CTX_CMD_Q_DOORBELL_CLEAR)?;
    reg.try_write32(1, KCI_CTX_BASE + CTX_CMD_Q_DOORBELL_ENABLE)?;
    reg.try_write32(1, KCI_CTX_BASE + CTX_RESP_Q_DOORBELL_ENABLE)?;
    reg.try_write32(1, KCI_CTX_BASE + CTX_ENABLE)?;
    Ok(())
}

/// Write `bytes` into the carveout at `phys` via the C glue.
fn mem_write(phys: u64, bytes: &[u8]) -> Result {
    // SAFETY: `bytes` is a valid slice; the glue memremaps `phys` for `len`.
    to_result(unsafe { bindings::edgetpu_mem_write(phys, bytes.as_ptr().cast(), bytes.len()) })
}

/// Read `bytes.len()` from the carveout at `phys` via the C glue.
fn mem_read(phys: u64, bytes: &mut [u8]) -> Result {
    // SAFETY: `bytes` is a valid mutable slice; the glue memremaps `phys`.
    to_result(unsafe {
        bindings::edgetpu_mem_read(phys, bytes.as_mut_ptr().cast(), bytes.len())
    })
}

/// Program the KCI mailbox + open the TPU data-path S2MPU. Must run BEFORE
/// `GSA_TPU_START`: the firmware reads the queue-base CSRs when it boots, so
/// they have to be in place first (matches AOSP's "mailbox reset, then firmware
/// run" ordering).
pub(crate) fn setup(reg: &TpuRegs<'_>) -> Result {
    // Enable TPU IO-coherency so the firmware's reads of the KCI queues snoop
    // the CPU's writes (otherwise it reads stale zeros and errors).
    // Open the TPU's data-path S2MPU so the firmware can reach the queues.
    // SAFETY: raw MMIO pokes of unclaimed TPU sysreg / S2MPU blocks.
    unsafe {
        bindings::edgetpu_enable_coherency();
        bindings::edgetpu_s2mpu_allow_all(TPU_S2MPU_BASE);
    }
    init_mailbox(reg)
}

/// Send the FIRMWARE_INFO KCI command and wait for the response. Returns the
/// firmware flavor on success. Call after the firmware is RUNNING.
pub(crate) fn fw_info(dev: &Device, reg: &TpuRegs<'_>) -> Result<u32> {
    // Zero the FW_INFO scratch buffer so we can tell the firmware wrote it.
    let zeros = [0u8; FW_INFO_SIZE];
    mem_write(FW_INFO_PHYS, &zeros)?;

    // Build the 32-byte command element:
    //   seq u64@0, code u16@8, reserved[3] u16@10, dma.address u64@16,
    //   dma.size u32@24, dma.flags u32@28.
    // The firmware tracks the sequence number and expects it to start at 0
    // (AOSP `kci->cur_seq = 0` for the first command); seq=1 is rejected.
    let seq: u64 = 0;
    let mut cmd = [0u8; CMD_ELEM];
    cmd[0..8].copy_from_slice(&seq.to_le_bytes());
    cmd[8..10].copy_from_slice(&KCI_CODE_FIRMWARE_INFO.to_le_bytes());
    // Point the firmware at the FW_INFO scratch buffer so it writes back the
    // fw_info struct (build time, flavor, changelist).
    cmd[16..24].copy_from_slice(&tpu_va(FW_INFO_PHYS).to_le_bytes());
    cmd[24..28].copy_from_slice(&(FW_INFO_SIZE as u32).to_le_bytes());

    // Push into cmd-queue slot 0, then advance the tail and ring the doorbell.
    mem_write(CMD_Q_PHYS, &cmd)?;
    reg.try_write32(1, KCI_CMD_BASE + CMD_TAIL)?;
    reg.try_write32(1, KCI_CMD_BASE + CMD_DOORBELL_SET)?;

    // Poll the resp-queue tail for the reply (1 s budget, matching KCI_TIMEOUT).
    let mut got = false;
    for _ in 0..1000 {
        if reg.try_read32(KCI_RESP_BASE + RESP_TAIL)? != 0 {
            got = true;
            break;
        }
        fsleep(Delta::from_millis(1));
    }
    if !got {
        // Dump the mailbox state to see whether the firmware even consumed the
        // command (cmd head advances) and whether any error latched.
        let cmd_head = reg.try_read32(KCI_CMD_BASE + CMD_HEAD)?;
        let cmd_tail = reg.try_read32(KCI_CMD_BASE + CMD_TAIL)?;
        let resp_head = reg.try_read32(KCI_RESP_BASE + RESP_HEAD)?;
        let resp_tail = reg.try_read32(KCI_RESP_BASE + RESP_TAIL)?;
        let ctx_en = reg.try_read32(KCI_CTX_BASE + CTX_ENABLE)?;
        let cmd_err = reg.try_read32(KCI_CMD_BASE + 0x14)?;
        let resp_err = reg.try_read32(KCI_RESP_BASE + 0x18)?;
        dev_err!(
            dev,
            "edgetpu: KCI timeout: ctx_en={} cmd[head={} tail={} err={:#x}] resp[head={} tail={} err={:#x}]\n",
            ctx_en, cmd_head, cmd_tail, cmd_err, resp_head, resp_tail, resp_err
        );
        return Err(ETIMEDOUT);
    }

    // Read the 16-byte response element from resp-queue slot 0.
    let mut resp = [0u8; RESP_ELEM];
    mem_read(RESP_Q_PHYS, &mut resp)?;
    let rseq = u64::from_le_bytes(resp[0..8].try_into().map_err(|_| EINVAL)?);
    let rcode = u16::from_le_bytes(resp[8..10].try_into().map_err(|_| EINVAL)?);

    // Ack: advance resp head, clear the resp doorbell.
    reg.try_write32(1, KCI_RESP_BASE + RESP_HEAD)?;
    reg.try_write32(1, KCI_RESP_BASE + RESP_DOORBELL_CLEAR)?;

    dev_info!(dev, "edgetpu: KCI response: seq={} code={}\n", rseq, rcode);
    if rseq != seq {
        dev_err!(dev, "edgetpu: KCI resp seq {} != {}\n", rseq, seq);
        return Err(EIO);
    }
    if rcode != KCI_ERROR_OK && rcode != KCI_ERROR_UNIMPLEMENTED {
        dev_err!(dev, "edgetpu: KCI FW_INFO returned code {}\n", rcode);
        return Err(EIO);
    }

    // Pull the firmware flavor (payload offset 0x08) the firmware wrote.
    let mut info = [0u8; FW_INFO_SIZE];
    mem_read(FW_INFO_PHYS, &mut info)?;
    let flavor = u32::from_le_bytes(info[8..12].try_into().map_err(|_| EINVAL)?);
    Ok(flavor)
}
