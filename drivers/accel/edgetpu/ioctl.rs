// SPDX-License-Identifier: GPL-2.0
//! DRM ioctl handlers for the Edge TPU accel driver (M2 UAPI).
//!
//! The surface is deliberately thin: allocate carveout-backed buffer objects
//! (returning a TPU device-VA), copy bytes in/out of them, and submit a single
//! VII command element to the inference mailbox. Userspace (finch) builds the
//! DarwiNN command payloads; the kernel only moves bytes and rings doorbells.

use kernel::{
    alloc::KVec,
    bindings,
    device::Device,
    prelude::*,
    uaccess::{UserPtr, UserSlice},
    uapi,
};

use crate::driver::EdgeTpuDevice;
use crate::file::{EdgeTpuFile, EdgeTpuFileData};
use crate::kci::Kci;
use crate::mem::{self, BoAllocator};
use crate::vii::{Vii, CMD_ELEM, NUM_VII, RESP_ELEM};

/// Device-global mailbox + buffer state, guarded by a mutex in the DRM device
/// data. A single client (finch) drives the `NUM_VII` VII contexts (mailboxes 1
/// and 2), each bound to a FIXED VCID (slot 0 -> VCID 0, slot 1 -> VCID 1) so
/// >158 graph registrations fit across them (register-once-dispatch).
///
/// "A single client" is an invariant, not a hope: because this state is device-global, admitting a
/// second client would mean [`reset_client`](Self::reset_client) wiping a live one's registrations
/// and heap. `file.rs` enforces it via `client_attached` (`EBUSY` on a second open).
pub(crate) struct MailboxState {
    pub(crate) kci: Kci,
    pub(crate) vii: [Vii; NUM_VII],
    pub(crate) bo: BoAllocator,
    /// Whether a client currently holds the device: set by `file.rs` on open (which refuses a
    /// second attach with `EBUSY`) and cleared when the fd closes.
    pub(crate) client_attached: bool,
}

impl MailboxState {
    /// Prepare a fresh session for a new client: reset EVERY VII context onto its FIXED VCID with
    /// first_open=true (which clears that VCID's prior registrations) and reclaim the shared BO heap.
    ///
    /// Each mailbox reuses the SAME VCID every session rather than rotating through the pool. Rotation
    /// (a fresh VCID per open) LEAKS the previous session's registrations: the firmware only clears a
    /// VCID's context on first_open of THAT VCID, so an abandoned VCID keeps its ~263 registrations
    /// live, and after a couple of sessions the firmware's global registration pool is exhausted —
    /// exec-register then silently fails and the next dispatch returns NOT_FOUND. Reusing a fixed VCID
    /// means first_open=true clears last session's registrations, so nothing accumulates.
    /// `dev`/`dev_raw` are the edgetpu platform device (for logging / IOMMU unmap).
    pub(crate) fn reset_client(&mut self, dev: &Device, dev_raw: *mut bindings::device) {
        for i in 0..NUM_VII {
            let _ = self.vii[i].reactivate(dev, &mut self.kci, i as u16);
        }
        self.bo.reset(dev_raw);
    }

    /// Full firmware power-cycle: GSA-restart the R52 in place and re-drive the
    /// KCI + VII handshake ([`crate::bringup::firmware_restart`]). This is the
    /// step-1 restart — the rail/clock stay up; it proves the firmware boots
    /// cleanly a second time, the prerequisite for gating the TPU rail at idle.
    pub(crate) fn power_cycle(&mut self, dev: &Device) -> Result {
        crate::bringup::firmware_restart(dev, &mut self.kci, &mut self.vii)
    }
}

/// Largest single BO we allow — a sanity bound on one allocation, well under the
/// page allocator's `MAX_ORDER` block (a contiguous system-memory BO). The whole
/// forward's worth of BOs is bounded instead by the 128 MiB IOVA window.
const BO_MAX: u64 = 4 * 1024 * 1024;

impl EdgeTpuFileData {
    /// Allocate a carveout BO and return its handle + TPU device-VA.
    pub(crate) fn create_bo(
        ddev: &EdgeTpuDevice,
        args: &mut uapi::drm_edgetpu_create_bo,
        _file: &EdgeTpuFile,
    ) -> Result<u32> {
        if args.size == 0 || args.size > BO_MAX {
            return Err(EINVAL);
        }
        // The edgetpu platform device owns the SysMMU domain (via `iommus=`).
        let dev = ddev.pdev.as_ref().as_raw();
        let mut st = ddev.mbox.lock();
        let (handle, iova) = st.bo.alloc(args.size, dev)?;
        args.handle = handle;
        args.tpu_va = iova;
        args.pad = 0;
        Ok(0)
    }

    /// Copy userspace bytes into a BO.
    pub(crate) fn bo_write(
        ddev: &EdgeTpuDevice,
        args: &mut uapi::drm_edgetpu_bo_write,
        _file: &EdgeTpuFile,
    ) -> Result<u32> {
        if args.size == 0 || args.size > BO_MAX {
            return Err(EINVAL);
        }
        // Stage the user data first (may fault/sleep) before taking the lock.
        let len = args.size as usize;
        let mut buf = KVec::with_capacity(len, GFP_KERNEL)?;
        buf.resize(len, 0, GFP_KERNEL)?;
        UserSlice::new(UserPtr::from_addr(args.user_ptr as usize), len)
            .reader()
            .read_slice(&mut buf)?;

        let cpu_va = {
            let st = ddev.mbox.lock();
            let bo = st.bo.get(args.handle).ok_or(EINVAL)?;
            let end = args.offset.checked_add(args.size).ok_or(EINVAL)?;
            if end > bo.size {
                return Err(EINVAL);
            }
            bo.cpu_va
        };
        // Copy after dropping the lock: the memcpy can be large and the backing
        // memory is stable for this client (only its own RESET/close frees it).
        mem::bo_write(cpu_va, args.offset, &buf);
        Ok(0)
    }

    /// Copy BO bytes back out to userspace.
    pub(crate) fn bo_read(
        ddev: &EdgeTpuDevice,
        args: &mut uapi::drm_edgetpu_bo_read,
        _file: &EdgeTpuFile,
    ) -> Result<u32> {
        if args.size == 0 || args.size > BO_MAX {
            return Err(EINVAL);
        }
        let len = args.size as usize;
        let cpu_va = {
            let st = ddev.mbox.lock();
            let bo = st.bo.get(args.handle).ok_or(EINVAL)?;
            let end = args.offset.checked_add(args.size).ok_or(EINVAL)?;
            if end > bo.size {
                return Err(EINVAL);
            }
            bo.cpu_va
        };

        let mut buf = KVec::with_capacity(len, GFP_KERNEL)?;
        buf.resize(len, 0, GFP_KERNEL)?;
        mem::bo_read(cpu_va, args.offset, &mut buf);
        UserSlice::new(UserPtr::from_addr(args.user_ptr as usize), len)
            .writer()
            .write_slice(&buf)?;
        Ok(0)
    }

    /// Submit one VII command element to the mailbox selected by `args.context`
    /// (0 or 1) and return its response. Routing by context lets a client keep
    /// two concurrent VCID contexts and dispatch each registered graph to the
    /// mailbox that holds its registration (register-once-dispatch).
    pub(crate) fn submit(
        ddev: &EdgeTpuDevice,
        args: &mut uapi::drm_edgetpu_submit,
        _file: &EdgeTpuFile,
    ) -> Result<u32> {
        let ctx = args.context as usize;
        if ctx >= NUM_VII {
            return Err(EINVAL);
        }
        let cmd: [u8; CMD_ELEM] = args.command;
        let mut st = ddev.mbox.lock();
        let resp: [u8; RESP_ELEM] = st.vii[ctx].submit(&cmd, args.timeout_ms)?;
        args.response = resp;
        args.context = 0;
        Ok(0)
    }

    /// Reset the client's VII context (fresh VCID) and reclaim the BO heap — the
    /// same work done on a fresh DRM open, exposed as an ioctl so a long-lived
    /// client can recycle mid-run without re-opening the fd. See
    /// [`MailboxState::reset_client`].
    pub(crate) fn reset(
        ddev: &EdgeTpuDevice,
        args: &mut uapi::drm_edgetpu_reset,
        _file: &EdgeTpuFile,
    ) -> Result<u32> {
        let pdev = ddev.pdev.as_ref();
        let raw = pdev.as_raw();
        // `flags` bit 0 (experimental, repurposes the reserved field): do a full
        // firmware power-cycle instead of the soft VII-context reset — used to
        // validate the runtime restart path from userspace during TPU idle-PM
        // bring-up. bit 0 == 0 keeps the original soft-reset behaviour.
        if args.flags & 1 != 0 {
            ddev.mbox.lock().power_cycle(pdev)?;
        } else {
            ddev.mbox.lock().reset_client(pdev, raw);
        }
        Ok(0)
    }
}
