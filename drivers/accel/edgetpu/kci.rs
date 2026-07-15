// SPDX-License-Identifier: GPL-2.0
//! KCI (Kernel Control Interface) mailbox — the boot handshake and the VII
//! mailbox activation/deactivation commands.
//!
//! KCI is mailbox index 0. Its cmd/resp circular queues live in the firmware
//! carveout's remapped-data region (TPU-VA `0x10100000+`, phys `0x93100000+`),
//! reached through the GSA-configured instruction remap — no SysMMU. We keep a
//! [`Kci`] object across the boot sequence so the sequence number and ring
//! positions carry from the `FIRMWARE_INFO` liveness check into the
//! `OPEN_DEVICE` command that binds the VII inference mailbox.
//!
//! All CSR access goes through the persistent [`crate::csr`] mapping (the C
//! companion's module-lifetime ioremap), so KCI works both during probe and
//! afterwards — the runtime path re-issues `CLOSE_DEVICE`/`OPEN_DEVICE` to reset
//! the VII context per client. Layout / protocol verified against the AOSP
//! janeiro `edgetpu-kci.c` + `edgetpu-mailbox.c` (pre-gcip generation).

use kernel::{
    device::Device,
    prelude::*,
    time::{delay::fsleep, Delta},
};

use crate::csr;
use crate::mem::{self, tpu_va};

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
const CMD_ERROR_STATUS: usize = 0x14;

// Resp-queue CSR offsets (from KCI_RESP_BASE).
const RESP_DOORBELL_CLEAR: usize = 0x04;
const RESP_HEAD: usize = 0x0c;
const RESP_TAIL: usize = 0x10;
const RESP_ERROR_STATUS: usize = 0x18;

// --- Queue geometry ---------------------------------------------------------
const QUEUE_SIZE: u32 = 1023; // elements; wrap bit = 0x400
const CMD_ELEM: usize = 32;
const RESP_ELEM: usize = 16;

// Circular-queue head/tail pointers are NOT free-running counters: they carry a real index in
// bits [0,10) plus a wrap bit (bit 10) that toggles each traversal, matching the firmware and the
// AOSP `circular_queue_inc` (edgetpu-mailbox.h). The slot to read/write is the real index; the
// value written to the tail/head CSR is the full `real | wrap` word. A plain `+= 1` / `% size`
// diverges from this at the first wrap (QUEUE_SIZE=1023 vs the 0x400 wrap bit), corrupting the ring
// once a command stream exceeds the queue depth (a whole model forward = hundreds of activations).
const WRAP_BIT: u32 = 1 << 10; // 0x400
const INDEX_MASK: u32 = WRAP_BIT - 1; // 0x3ff

/// The real ring slot for a wrap-encoded head/tail pointer.
fn real_index(idx: u32) -> usize {
    (idx & INDEX_MASK) as usize
}

/// Advance a wrap-encoded circular-queue pointer by one, per the AOSP `circular_queue_inc`.
fn circ_inc(idx: u32, queue_size: u32) -> u32 {
    if (idx & INDEX_MASK) + 1 >= queue_size {
        (idx + 1 - queue_size) ^ WRAP_BIT
    } else {
        idx + 1
    }
}

// --- Carveout placement (see mem.rs for the full map) -----------------------
const CMD_Q_PHYS: u64 = 0x9310_0000; // TPU-VA 0x10100000
const RESP_Q_PHYS: u64 = 0x9311_0000; // TPU-VA 0x10110000
const FW_INFO_PHYS: u64 = 0x9312_0000; // TPU-VA 0x10120000
const FW_INFO_SIZE: usize = 56;
/// Scratch for the 8-byte `OPEN_DEVICE` DMA payload (kept clear of fw_info).
const DETAIL_PHYS: u64 = 0x9312_1000; // TPU-VA 0x10121000

// --- KCI protocol constants -------------------------------------------------
const KCI_CODE_FIRMWARE_INFO: u16 = 11;
const KCI_CODE_OPEN_DEVICE: u16 = 9;
const KCI_CODE_CLOSE_DEVICE: u16 = 10;
const KCI_ERROR_OK: u16 = 0;
const KCI_ERROR_UNIMPLEMENTED: u16 = 12;

/// The TPU's own S2MPU (s2mpu_tpu@1cc60000, gs201 == S2MPU_V1). Left enforcing,
/// the firmware reads the AP-placed KCI queues as zeros; open it allow-all.
const TPU_S2MPU_BASE: u64 = 0x1cc6_0000;

/// KCI mailbox state, threaded across the boot sequence and the runtime.
pub(crate) struct Kci {
    seq: u64,
    cmd_tail: u32,
    resp_head: u32,
}

impl Kci {
    pub(crate) fn new() -> Self {
        Self {
            seq: 0,
            cmd_tail: 0,
            resp_head: 0,
        }
    }

    /// Enable TPU IO-coherency + open the data-path S2MPU, then program the KCI
    /// mailbox context/queue CSRs. Must run BEFORE `GSA_TPU_START` — the
    /// firmware latches the KCI queue-base CSRs when it boots.
    pub(crate) fn setup(&mut self) -> Result {
        // SAFETY: raw MMIO pokes of unclaimed TPU sysreg / S2MPU blocks.
        unsafe {
            kernel::bindings::edgetpu_enable_coherency();
            kernel::bindings::edgetpu_s2mpu_allow_all(TPU_S2MPU_BASE);
        }
        self.init_mailbox();
        Ok(())
    }

    fn init_mailbox(&mut self) {
        let cmd_va = tpu_va(CMD_Q_PHYS);
        let resp_va = tpu_va(RESP_Q_PHYS);

        // Cmd queue.
        csr::write(KCI_CTX_BASE + CTX_CMD_Q_ADDR_LO, (cmd_va & 0xffff_ffff) as u32);
        csr::write(KCI_CTX_BASE + CTX_CMD_Q_ADDR_HI, (cmd_va >> 32) as u32);
        csr::write(KCI_CTX_BASE + CTX_CMD_Q_SIZE, QUEUE_SIZE);
        csr::write(KCI_CMD_BASE + CMD_TAIL, 0);
        csr::write(KCI_CMD_BASE + CMD_HEAD, 0);

        // Resp queue.
        csr::write(KCI_CTX_BASE + CTX_RESP_Q_ADDR_LO, (resp_va & 0xffff_ffff) as u32);
        csr::write(KCI_CTX_BASE + CTX_RESP_Q_ADDR_HI, (resp_va >> 32) as u32);
        csr::write(KCI_CTX_BASE + CTX_RESP_Q_SIZE, QUEUE_SIZE);
        csr::write(KCI_RESP_BASE + RESP_HEAD, 0);
        csr::write(KCI_RESP_BASE + RESP_TAIL, 0);

        // Clear stale doorbells, enable doorbells, enable the mailbox context.
        csr::write(KCI_RESP_BASE + RESP_DOORBELL_CLEAR, 1);
        csr::write(KCI_CTX_BASE + CTX_CMD_Q_DOORBELL_CLEAR, 1);
        csr::write(KCI_CTX_BASE + CTX_CMD_Q_DOORBELL_ENABLE, 1);
        csr::write(KCI_CTX_BASE + CTX_RESP_Q_DOORBELL_ENABLE, 1);
        csr::write(KCI_CTX_BASE + CTX_ENABLE, 1);

        self.seq = 0;
        self.cmd_tail = 0;
        self.resp_head = 0;
    }

    /// Push one command element and wait for its response. Returns
    /// `(response_code, retval)`. The `dma_*` fields describe an optional DMA
    /// buffer (address is a TPU-VA); pass `(0, 0, 0)` for none.
    fn transact(
        &mut self,
        dev: &Device,
        code: u16,
        dma_addr: u64,
        dma_size: u32,
        dma_flags: u32,
    ) -> Result<(u16, u32)> {
        // Build the 32-byte command element:
        //   seq u64@0, code u16@8, dma.address u64@16, dma.size u32@24,
        //   dma.flags u32@28.
        let slot = real_index(self.cmd_tail);
        let mut cmd = [0u8; CMD_ELEM];
        cmd[0..8].copy_from_slice(&self.seq.to_le_bytes());
        cmd[8..10].copy_from_slice(&code.to_le_bytes());
        cmd[16..24].copy_from_slice(&dma_addr.to_le_bytes());
        cmd[24..28].copy_from_slice(&dma_size.to_le_bytes());
        cmd[28..32].copy_from_slice(&dma_flags.to_le_bytes());

        mem::write(CMD_Q_PHYS + (slot * CMD_ELEM) as u64, &cmd)?;
        self.cmd_tail = circ_inc(self.cmd_tail, QUEUE_SIZE);
        csr::write(KCI_CMD_BASE + CMD_TAIL, self.cmd_tail);
        csr::write(KCI_CMD_BASE + CMD_DOORBELL_SET, 1);

        // Poll the resp-queue tail for the reply (1 s budget, KCI_TIMEOUT).
        let mut got = false;
        // Firmware responds in microseconds; spin-read first, then fall back to a short sleep.
        // The old 1ms sleep per mailbox round-trip dominated per-op latency (a full model forward
        // is hundreds of ops × several KCI/VII round-trips), so a tight read loop is a big win.
        'poll: for _ in 0..2000 {
            for _ in 0..2000 {
                if csr::read(KCI_RESP_BASE + RESP_TAIL) != self.resp_head {
                    got = true;
                    break 'poll;
                }
            }
            fsleep(Delta::from_micros(50));
        }
        if !got {
            dev_err!(
                dev,
                "edgetpu: KCI timeout (code {}): cmd[head={} tail={} err={:#x}] resp[tail={} err={:#x}]\n",
                code,
                csr::read(KCI_CMD_BASE + CMD_HEAD),
                self.cmd_tail,
                csr::read(KCI_CMD_BASE + CMD_ERROR_STATUS),
                self.resp_head,
                csr::read(KCI_RESP_BASE + RESP_ERROR_STATUS)
            );
            return Err(ETIMEDOUT);
        }

        // Read the 16-byte response element (seq u64@0, code u16@8, retval u32@12).
        let rslot = real_index(self.resp_head);
        let mut resp = [0u8; RESP_ELEM];
        mem::read(RESP_Q_PHYS + (rslot * RESP_ELEM) as u64, &mut resp)?;
        let rseq = u64::from_le_bytes(resp[0..8].try_into().map_err(|_| EINVAL)?);
        let rcode = u16::from_le_bytes(resp[8..10].try_into().map_err(|_| EINVAL)?);
        let retval = u32::from_le_bytes(resp[12..16].try_into().map_err(|_| EINVAL)?);

        // Ack: advance resp head (wrap-encoded), clear the resp doorbell.
        self.resp_head = circ_inc(self.resp_head, QUEUE_SIZE);
        csr::write(KCI_RESP_BASE + RESP_HEAD, self.resp_head);
        csr::write(KCI_RESP_BASE + RESP_DOORBELL_CLEAR, 1);

        // The firmware echoes the command's seq verbatim; it's a plain monotonic counter (the earlier
        // "resp seq 0 != N" at the wrap was a stale read from the wrong ring slot, now fixed above).
        if rseq != self.seq {
            dev_err!(dev, "edgetpu: KCI resp seq {} != {}\n", rseq, self.seq);
            return Err(EIO);
        }
        self.seq += 1;
        Ok((rcode, retval))
    }

    /// Send `FIRMWARE_INFO` and return the firmware flavor — proves the firmware
    /// is interactively processing mailbox commands.
    pub(crate) fn fw_info(&mut self, dev: &Device) -> Result<u32> {
        let zeros = [0u8; FW_INFO_SIZE];
        mem::write(FW_INFO_PHYS, &zeros)?;

        let (rcode, _) = self.transact(
            dev,
            KCI_CODE_FIRMWARE_INFO,
            tpu_va(FW_INFO_PHYS),
            FW_INFO_SIZE as u32,
            0,
        )?;
        if rcode != KCI_ERROR_OK && rcode != KCI_ERROR_UNIMPLEMENTED {
            dev_err!(dev, "edgetpu: KCI FW_INFO returned code {}\n", rcode);
            return Err(EIO);
        }

        let mut info = [0u8; FW_INFO_SIZE];
        mem::read(FW_INFO_PHYS, &mut info)?;
        Ok(u32::from_le_bytes(info[8..12].try_into().map_err(|_| EINVAL)?))
    }

    /// Bind a VII/external mailbox to a VCID (janeiro `edgetpu_kci_open_device`).
    /// `mailbox_id` is the mailbox index; the command carries `BIT(mailbox_id)`
    /// inline and an 8-byte detail `{client_priv, vcid, flags=(map<<1)|first}`.
    pub(crate) fn open_device(
        &mut self,
        dev: &Device,
        mailbox_id: u32,
        vcid: u16,
        first_open: bool,
    ) -> Result {
        let map = 1u32 << mailbox_id;
        let flags = (map << 1) | (first_open as u32);

        let mut detail = [0u8; 8];
        // client_priv u16@0 (0), vcid u16@2, flags u32@4.
        detail[2..4].copy_from_slice(&vcid.to_le_bytes());
        detail[4..8].copy_from_slice(&flags.to_le_bytes());
        mem::write(DETAIL_PHYS, &detail)?;

        let (rcode, _) =
            self.transact(dev, KCI_CODE_OPEN_DEVICE, tpu_va(DETAIL_PHYS), 8, map)?;
        if rcode != KCI_ERROR_OK {
            dev_err!(
                dev,
                "edgetpu: OPEN_DEVICE mailbox {} -> code {}\n",
                mailbox_id,
                rcode
            );
            return Err(EIO);
        }
        Ok(())
    }

    /// Unbind a previously opened mailbox (janeiro `edgetpu_kci_close_device`) —
    /// frees the firmware's per-context state (registered scratch/executables).
    /// Best-effort: only the mailbox bitmap is required.
    pub(crate) fn close_device(&mut self, dev: &Device, mailbox_id: u32) -> Result {
        let map = 1u32 << mailbox_id;
        let (_rcode, _) = self.transact(dev, KCI_CODE_CLOSE_DEVICE, 0, 0, map)?;
        Ok(())
    }
}
