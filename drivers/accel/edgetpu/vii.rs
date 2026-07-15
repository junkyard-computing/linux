// SPDX-License-Identifier: GPL-2.0
//! VII (Virtual Instruction Interface) mailbox — the per-context inference
//! mailbox that carries the runtime's DarwiNN command elements.
//!
//! The felix TPU (janeiro) exposes 7 VII mailboxes at indices 1..7; we use
//! index 1. Its CSR block sits at `0xa0000 + 1*0x2000 = 0xa2000` with the same
//! field layout as KCI (mailbox 0). The cmd/resp queues live in the firmware
//! carveout reached through the instruction remap, exactly like KCI — no
//! SysMMU. The 48-byte command / 24-byte response element formats are the
//! firmware↔runtime ABI; the kernel is oblivious to their contents and only
//! moves elements + rings the doorbell.
//!
//! Two register paths: [`setup`] runs during probe on the probe-time `IoMem`
//! (to program the queue-base CSRs before activation); [`submit`] runs from
//! ioctls long after probe and therefore uses the module-lifetime CSR mapping
//! in [`crate::csr`].

use kernel::{
    device::Device,
    prelude::*,
    time::{delay::fsleep, Delta},
};

use crate::csr;
use crate::kci::Kci;
use crate::mem::{self, tpu_va};

// --- VII mailbox #1 CSR bases (offsets from the main TPU CSR block) ----------
const VII_CTX_BASE: usize = 0xa2000; // context CSRs (mailbox 1)
const VII_CMD_BASE: usize = 0xa3000; // cmd-queue CSRs
const VII_RESP_BASE: usize = 0xa3800; // resp-queue CSRs

// Context CSR offsets (identical to KCI's layout).
const CTX_ENABLE: usize = 0x00;
const CTX_PRIORITY: usize = 0x04;
const CTX_CMD_Q_DOORBELL_ENABLE: usize = 0x08;
const CTX_CMD_Q_TAIL_DOORBELL_ENABLE: usize = 0x0c;
const CTX_CMD_Q_DOORBELL_CLEAR: usize = 0x10;
const CTX_CMD_Q_ADDR_LO: usize = 0x14;
const CTX_CMD_Q_ADDR_HI: usize = 0x18;
const CTX_CMD_Q_SIZE: usize = 0x1c;
const CTX_RESP_Q_DOORBELL_ENABLE: usize = 0x20;
const CTX_RESP_Q_ADDR_LO: usize = 0x28;
const CTX_RESP_Q_ADDR_HI: usize = 0x2c;
const CTX_RESP_Q_SIZE: usize = 0x30;

// Cmd-queue CSR offsets.
const CMD_DOORBELL_SET: usize = 0x00;
const CMD_HEAD: usize = 0x08;
const CMD_TAIL: usize = 0x0c;

// Resp-queue CSR offsets.
const RESP_DOORBELL_CLEAR: usize = 0x04;
const RESP_HEAD: usize = 0x0c;
const RESP_TAIL: usize = 0x10;

// --- Queue geometry ---------------------------------------------------------
const QUEUE_SIZE: u32 = 1023; // elements; wrap bit = 0x400
pub(crate) const CMD_ELEM: usize = 48; // VII command element
pub(crate) const RESP_ELEM: usize = 24; // VII response element

// Circular-queue head/tail carry a real index (bits [0,10)) plus a wrap bit (bit 10), advanced by
// the AOSP `circular_queue_inc`. The slot is the real index; the CSR gets the full `real | wrap`
// word. A free-running `+= 1` / `% size` diverges at the first wrap (QUEUE_SIZE=1023 vs the 0x400
// wrap bit) and wedges the ring once a submit stream exceeds the queue depth. See kci.rs.
const WRAP_BIT: u32 = 1 << 10; // 0x400
const INDEX_MASK: u32 = WRAP_BIT - 1; // 0x3ff

fn real_index(idx: u32) -> usize {
    (idx & INDEX_MASK) as usize
}

fn circ_inc(idx: u32, queue_size: u32) -> u32 {
    if (idx & INDEX_MASK) + 1 >= queue_size {
        (idx + 1 - queue_size) ^ WRAP_BIT
    } else {
        idx + 1
    }
}

// --- Carveout placement (see mem.rs) ----------------------------------------
const CMD_Q_PHYS: u64 = 0x9313_0000; // TPU-VA 0x10130000
const RESP_Q_PHYS: u64 = 0x9314_0000; // TPU-VA 0x10140000

// --- Activation parameters --------------------------------------------------
const VII_MAILBOX_ID: u32 = 1;
// The firmware keys registered scratch/executables by VCID and does NOT drop them on CLOSE_DEVICE
// (only `first_open` clears a VCID's context). The AOSP driver therefore allocates a FRESH VCID per
// device group and activates it with first_open=true (edgetpu-device-group.c:
// `edgetpu_mailbox_activate(..., group->vcid, !group->activated)`), freeing it on group teardown.
// We mirror that by rotating the VCID across sessions so each reset_client gets a fresh context
// instead of resuming (and accumulating in) a single fixed VCID. gs201 has EDGETPU_NUM_VCIDS=16.
const NUM_VCIDS: u16 = 16;

/// VII mailbox state (ring positions + activation status + rotating VCID).
pub(crate) struct Vii {
    cmd_tail: u32,
    resp_head: u32,
    activated: bool,
    vcid: u16,
}

impl Vii {
    pub(crate) fn new() -> Self {
        Self {
            cmd_tail: 0,
            resp_head: 0,
            activated: false,
            vcid: NUM_VCIDS - 1, // first `next_vcid()` rolls to 0
        }
    }

    /// Advance to the next VCID (round-robin over the pool). Each session binds a fresh VCID so
    /// the firmware gives it a clean context rather than accumulating on a reused one.
    fn next_vcid(&mut self) -> u16 {
        self.vcid = (self.vcid + 1) % NUM_VCIDS;
        self.vcid
    }

    pub(crate) fn is_activated(&self) -> bool {
        self.activated
    }

    /// Program the VII mailbox context/queue CSRs (via the persistent CSR map).
    /// The firmware reads these when the mailbox is activated by `OPEN_DEVICE`.
    fn setup(&mut self) {
        let cmd_va = tpu_va(CMD_Q_PHYS);
        let resp_va = tpu_va(RESP_Q_PHYS);

        csr::write(VII_CTX_BASE + CTX_PRIORITY, 0);
        // Explicit doorbell (like KCI) rather than tail-write auto-doorbell.
        csr::write(VII_CTX_BASE + CTX_CMD_Q_TAIL_DOORBELL_ENABLE, 0);

        // Cmd queue.
        csr::write(VII_CTX_BASE + CTX_CMD_Q_ADDR_LO, (cmd_va & 0xffff_ffff) as u32);
        csr::write(VII_CTX_BASE + CTX_CMD_Q_ADDR_HI, (cmd_va >> 32) as u32);
        csr::write(VII_CTX_BASE + CTX_CMD_Q_SIZE, QUEUE_SIZE);
        csr::write(VII_CMD_BASE + CMD_TAIL, 0);
        csr::write(VII_CMD_BASE + CMD_HEAD, 0);

        // Resp queue.
        csr::write(VII_CTX_BASE + CTX_RESP_Q_ADDR_LO, (resp_va & 0xffff_ffff) as u32);
        csr::write(VII_CTX_BASE + CTX_RESP_Q_ADDR_HI, (resp_va >> 32) as u32);
        csr::write(VII_CTX_BASE + CTX_RESP_Q_SIZE, QUEUE_SIZE);
        csr::write(VII_RESP_BASE + RESP_HEAD, 0);
        csr::write(VII_RESP_BASE + RESP_TAIL, 0);

        // Clear stale doorbells, enable doorbells, enable the context.
        csr::write(VII_RESP_BASE + RESP_DOORBELL_CLEAR, 1);
        csr::write(VII_CTX_BASE + CTX_CMD_Q_DOORBELL_CLEAR, 1);
        csr::write(VII_CTX_BASE + CTX_CMD_Q_DOORBELL_ENABLE, 1);
        csr::write(VII_CTX_BASE + CTX_RESP_Q_DOORBELL_ENABLE, 1);
        csr::write(VII_CTX_BASE + CTX_ENABLE, 1);

        self.cmd_tail = 0;
        self.resp_head = 0;
    }

    /// Program the VII mailbox and bind it to a fresh VCID via a KCI `OPEN_DEVICE` (first_open).
    /// Called at probe.
    pub(crate) fn activate(&mut self, dev: &Device, kci: &mut Kci) -> Result {
        self.setup();
        let vcid = self.next_vcid();
        kci.open_device(dev, VII_MAILBOX_ID, vcid, true)?;
        self.activated = true;
        Ok(())
    }

    /// Reset the VII inference context for a new client. The firmware retains a VCID's registered
    /// scratch/executables across CLOSE_DEVICE, so resuming a single fixed VCID (first_open=false)
    /// accumulates them until inferences are rejected after ~one model forward. Instead we CLOSE
    /// the mailbox and re-`OPEN_DEVICE` on the NEXT VCID with first_open=true — a clean context per
    /// session, exactly as the AOSP driver does per device group. Best-effort close.
    pub(crate) fn reactivate(&mut self, dev: &Device, kci: &mut Kci) -> Result {
        if self.activated {
            let _ = kci.close_device(dev, VII_MAILBOX_ID);
            self.activated = false;
        }
        self.setup();
        let vcid = self.next_vcid();
        kci.open_device(dev, VII_MAILBOX_ID, vcid, true)?;
        self.activated = true;
        Ok(())
    }

    /// Submit one 48-byte VII command element and wait for its 24-byte response.
    /// Runs from ioctls, so all CSR access goes through the persistent mapping.
    pub(crate) fn submit(&mut self, command: &[u8; CMD_ELEM], timeout_ms: u32) -> Result<[u8; RESP_ELEM]> {
        if !self.activated {
            return Err(ENODEV);
        }

        let slot = real_index(self.cmd_tail);
        mem::write(CMD_Q_PHYS + (slot * CMD_ELEM) as u64, command)?;
        self.cmd_tail = circ_inc(self.cmd_tail, QUEUE_SIZE);
        csr::write(VII_CMD_BASE + CMD_TAIL, self.cmd_tail);
        csr::write(VII_CMD_BASE + CMD_DOORBELL_SET, 1);

        let budget = if timeout_ms == 0 { 1000 } else { timeout_ms as usize };
        let mut got = false;
        // Spin-read before sleeping: inference ops complete in microseconds, so the old 1ms sleep
        // per submit was the dominant per-op latency for a full model forward. ~20 short-sleep
        // iters replace each former 1ms iter, preserving the overall timeout budget.
        'poll: for _ in 0..(budget * 20) {
            for _ in 0..2000 {
                if csr::read(VII_RESP_BASE + RESP_TAIL) != self.resp_head {
                    got = true;
                    break 'poll;
                }
            }
            fsleep(Delta::from_micros(50));
        }
        if !got {
            pr_err!(
                "edgetpu: VII submit timeout: cmd[head={} tail={}] resp[tail={}]\n",
                csr::read(VII_CMD_BASE + CMD_HEAD),
                self.cmd_tail,
                csr::read(VII_RESP_BASE + RESP_TAIL)
            );
            return Err(ETIMEDOUT);
        }

        let rslot = real_index(self.resp_head);
        let mut resp = [0u8; RESP_ELEM];
        mem::read(RESP_Q_PHYS + (rslot * RESP_ELEM) as u64, &mut resp)?;
        self.resp_head = circ_inc(self.resp_head, QUEUE_SIZE);
        csr::write(VII_RESP_BASE + RESP_HEAD, self.resp_head);
        csr::write(VII_RESP_BASE + RESP_DOORBELL_CLEAR, 1);

        Ok(resp)
    }
}
