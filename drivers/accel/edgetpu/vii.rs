// SPDX-License-Identifier: GPL-2.0
//! VII (Virtual Instruction Interface) mailbox — the per-context inference
//! mailbox that carries the runtime's DarwiNN command elements.
//!
//! The felix TPU (janeiro) exposes 7 VII mailboxes at indices 1..7. Each
//! mailbox binds to its own VCID (virtual context), and the firmware caps the
//! number of graphs registered in ONE VCID at ~158 — a whole model forward is
//! more executables than that. To keep a whole forward's executables resident
//! (register-once-dispatch) we drive TWO VII mailboxes (indices 1 and 2) as two
//! concurrent VCID contexts, so registrations spread across ~2×150 slots. Each
//! mailbox's CSR block sits at `0xa0000 + index*0x2000` with the same field
//! layout as KCI (mailbox 0); its cmd/resp queues live in the firmware carveout
//! reached through the instruction remap, exactly like KCI — no SysMMU. The
//! 48-byte command / 24-byte response element formats are the firmware↔runtime
//! ABI; the kernel is oblivious to their contents and only moves elements + rings
//! the doorbell.
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

// --- VII mailbox CSR geometry (offsets from the main TPU CSR block) ----------
// Mailbox N's context CSRs are at 0xa0000 + N*0x2000 (janeiro config-mailbox.h
// `edgetpu_mailbox_get_context_csr_base`); the cmd/resp queue CSRs are at
// +0x1000 / +0x1800 within that block.
const MBOX_CSR_BASE: usize = 0xa0000;
const MBOX_CSR_STRIDE: usize = 0x2000;
const CMD_CSR_OFF: usize = 0x1000;
const RESP_CSR_OFF: usize = 0x1800;

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
// Two VII mailboxes, each with its own cmd/resp queues. Mailbox 1's queues sit
// where they always did; mailbox 2's live in the region freed by moving buffer
// objects to system memory (0x93150000+ was the carveout BO heap).
pub(crate) const NUM_VII: usize = 2;
const VII_CMD_Q_PHYS: [u64; NUM_VII] = [0x9313_0000, 0x9315_0000]; // TPU-VA 0x1013/0x1015 0000
const VII_RESP_Q_PHYS: [u64; NUM_VII] = [0x9314_0000, 0x9316_0000]; // TPU-VA 0x1014/0x1016 0000

// --- Activation parameters --------------------------------------------------
// The firmware keys registered scratch/executables by VCID and does NOT drop them on CLOSE_DEVICE;
// only `first_open` clears a VCID's context. Each mailbox therefore keeps a FIXED VCID (slot 0 ->
// VCID 0, slot 1 -> VCID 1) and re-activates it with first_open=true every session, which clears the
// prior session's registrations. Rotating to a fresh VCID per session instead would LEAK the old
// VCID's registrations (never re-opened → never cleared), exhausting the firmware's global
// registration pool after a couple of register-once sessions. gs201 has EDGETPU_NUM_VCIDS=16, far
// more than the two fixed VCIDs we use.

/// VII mailbox state (mailbox index + derived CSR bases + its own queues + ring positions +
/// activation status + the VCID it is currently bound to).
pub(crate) struct Vii {
    mailbox_id: u32,
    ctx_base: usize,
    cmd_base: usize,
    resp_base: usize,
    cmd_q_phys: u64,
    resp_q_phys: u64,
    cmd_tail: u32,
    resp_head: u32,
    activated: bool,
    vcid: u16,
}

impl Vii {
    /// Build the `slot`-th VII mailbox (0 -> mailbox index 1, 1 -> index 2, …). Derives the CSR
    /// bases from the mailbox index and takes its dedicated cmd/resp queues from the tables above.
    pub(crate) fn new(slot: usize) -> Self {
        let mailbox_id = (slot + 1) as u32; // VII mailboxes are 1..7 (KCI is mailbox 0)
        let ctx_base = MBOX_CSR_BASE + (mailbox_id as usize) * MBOX_CSR_STRIDE;
        Self {
            mailbox_id,
            ctx_base,
            cmd_base: ctx_base + CMD_CSR_OFF,
            resp_base: ctx_base + RESP_CSR_OFF,
            cmd_q_phys: VII_CMD_Q_PHYS[slot],
            resp_q_phys: VII_RESP_Q_PHYS[slot],
            cmd_tail: 0,
            resp_head: 0,
            activated: false,
            vcid: 0,
        }
    }

    /// Program the VII mailbox context/queue CSRs (via the persistent CSR map).
    /// The firmware reads these when the mailbox is activated by `OPEN_DEVICE`.
    fn setup(&mut self) {
        let cmd_va = tpu_va(self.cmd_q_phys);
        let resp_va = tpu_va(self.resp_q_phys);

        csr::write(self.ctx_base + CTX_PRIORITY, 0);
        // Explicit doorbell (like KCI) rather than tail-write auto-doorbell.
        csr::write(self.ctx_base + CTX_CMD_Q_TAIL_DOORBELL_ENABLE, 0);

        // Cmd queue.
        csr::write(self.ctx_base + CTX_CMD_Q_ADDR_LO, (cmd_va & 0xffff_ffff) as u32);
        csr::write(self.ctx_base + CTX_CMD_Q_ADDR_HI, (cmd_va >> 32) as u32);
        csr::write(self.ctx_base + CTX_CMD_Q_SIZE, QUEUE_SIZE);
        csr::write(self.cmd_base + CMD_TAIL, 0);
        csr::write(self.cmd_base + CMD_HEAD, 0);

        // Resp queue.
        csr::write(self.ctx_base + CTX_RESP_Q_ADDR_LO, (resp_va & 0xffff_ffff) as u32);
        csr::write(self.ctx_base + CTX_RESP_Q_ADDR_HI, (resp_va >> 32) as u32);
        csr::write(self.ctx_base + CTX_RESP_Q_SIZE, QUEUE_SIZE);
        csr::write(self.resp_base + RESP_HEAD, 0);
        csr::write(self.resp_base + RESP_TAIL, 0);

        // Clear stale doorbells, enable doorbells, enable the context.
        csr::write(self.resp_base + RESP_DOORBELL_CLEAR, 1);
        csr::write(self.ctx_base + CTX_CMD_Q_DOORBELL_CLEAR, 1);
        csr::write(self.ctx_base + CTX_CMD_Q_DOORBELL_ENABLE, 1);
        csr::write(self.ctx_base + CTX_RESP_Q_DOORBELL_ENABLE, 1);
        csr::write(self.ctx_base + CTX_ENABLE, 1);

        self.cmd_tail = 0;
        self.resp_head = 0;
    }

    /// Program the VII mailbox and bind it to `vcid` via a KCI `OPEN_DEVICE` (first_open). Called at
    /// probe; the caller supplies a VCID distinct from every other mailbox's.
    pub(crate) fn activate(&mut self, dev: &Device, kci: &mut Kci, vcid: u16) -> Result {
        self.setup();
        kci.open_device(dev, self.mailbox_id, vcid, true)?;
        self.vcid = vcid;
        self.activated = true;
        Ok(())
    }

    /// Reset the VII inference context for a new client. The firmware retains a VCID's registered
    /// scratch/executables across CLOSE_DEVICE, so resuming a single fixed VCID (first_open=false)
    /// accumulates them until inferences are rejected after ~one model forward. Instead we CLOSE
    /// the mailbox and re-`OPEN_DEVICE` on `vcid` (a fresh one, supplied by the caller) with
    /// first_open=true — a clean context per session, as the AOSP driver does per device group.
    /// Best-effort close.
    pub(crate) fn reactivate(&mut self, dev: &Device, kci: &mut Kci, vcid: u16) -> Result {
        if self.activated {
            let _ = kci.close_device(dev, self.mailbox_id);
            self.activated = false;
        }
        self.setup();
        kci.open_device(dev, self.mailbox_id, vcid, true)?;
        self.vcid = vcid;
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
        mem::write(self.cmd_q_phys + (slot * CMD_ELEM) as u64, command)?;
        self.cmd_tail = circ_inc(self.cmd_tail, QUEUE_SIZE);
        csr::write(self.cmd_base + CMD_TAIL, self.cmd_tail);
        csr::write(self.cmd_base + CMD_DOORBELL_SET, 1);

        let budget = if timeout_ms == 0 { 1000 } else { timeout_ms as usize };
        let mut got = false;
        // Hard-spin the response CSR before falling back to a short sleep: an in-kernel fsleep can
        // overshoot a sub-millisecond firmware response by its own granularity, so spin long enough
        // to catch the common case in-loop; the sleep tail only covers the (error) timeout budget.
        'poll: for _ in 0..(budget * 20) {
            for _ in 0..60000 {
                if csr::read(self.resp_base + RESP_TAIL) != self.resp_head {
                    got = true;
                    break 'poll;
                }
            }
            fsleep(Delta::from_micros(50));
        }
        if !got {
            pr_err!(
                "edgetpu: VII mailbox {} submit timeout: cmd[head={} tail={}] resp[tail={}]\n",
                self.mailbox_id,
                csr::read(self.cmd_base + CMD_HEAD),
                self.cmd_tail,
                csr::read(self.resp_base + RESP_TAIL)
            );
            return Err(ETIMEDOUT);
        }

        let rslot = real_index(self.resp_head);
        let mut resp = [0u8; RESP_ELEM];
        mem::read(self.resp_q_phys + (rslot * RESP_ELEM) as u64, &mut resp)?;
        self.resp_head = circ_inc(self.resp_head, QUEUE_SIZE);
        csr::write(self.resp_base + RESP_HEAD, self.resp_head);
        csr::write(self.resp_base + RESP_DOORBELL_CLEAR, 1);

        Ok(resp)
    }
}
