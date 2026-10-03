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
    drm,
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
    /// Consecutive submit timeouts per mailbox in which the firmware had not even dequeued the
    /// command (see [`Vii::stalled`]).
    pub(crate) stalls: [u32; NUM_VII],
    /// The driver restarted the firmware under this client (see [`recover`](Self::recover)), so
    /// its registrations are gone. Submits fail with `EPIPE` until it RESETs or reopens.
    pub(crate) lost: bool,
}

/// A mailbox that stops consuming commands for this many consecutive submit timeouts is taken as
/// wedged (one timeout alone may be a legitimately slow op that the firmware has not reached yet).
const WEDGE_STALLS: u32 = 2;

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
        self.stalls = [0; NUM_VII];
        self.lost = false;
    }

    /// After a submit on mailbox `ctx` timed out: decide whether the firmware is wedged, and if
    /// so restart it in place rather than leave every later command to time out (and the device
    /// dead until a client thinks to issue RESET flags=1). Wedged means the firmware said so (a
    /// CRASH / JOB_LOCKUP reverse KCI) or it stopped dequeuing this mailbox for [`WEDGE_STALLS`]
    /// timeouts running. The restart wipes every registration, so the client is marked lost.
    fn recover(&mut self, dev: &Device, ctx: usize) {
        let fatal = self.kci.take_fatal(dev);
        if self.vii[ctx].stalled() {
            self.stalls[ctx] += 1;
        } else {
            self.stalls[ctx] = 0;
        }
        if fatal.is_none() && self.stalls[ctx] < WEDGE_STALLS {
            return;
        }
        dev_err!(
            dev,
            "edgetpu: TPU firmware wedged (mailbox {}: {} stalled timeouts, reverse KCI {:?}); restarting it\n",
            ctx + 1,
            self.stalls[ctx],
            fatal
        );
        if let Err(e) = self.power_cycle(dev) {
            dev_err!(dev, "edgetpu: firmware restart failed ({:?}); RESET flags=1 to retry\n", e);
        }
        self.stalls = [0; NUM_VII];
        self.lost = true;
    }

    /// Full firmware power-cycle: GSA-restart the R52 in place and re-drive the
    /// KCI + VII handshake ([`crate::bringup::firmware_restart`]). This is the
    /// step-1 restart — the rail/clock stay up; it proves the firmware boots
    /// cleanly a second time, the prerequisite for gating the TPU rail at idle.
    pub(crate) fn power_cycle(&mut self, dev: &Device) -> Result {
        crate::bringup::firmware_restart(dev, &mut self.kci, &mut self.vii)
    }
}

/// Largest single BO we allow — a sanity bound on one allocation. A BO's pages need not be
/// physically contiguous (it was capped at the page allocator's 4 MiB largest block while they
/// had to be, which an 8-layer decode executable outgrew); the whole forward's worth of BOs is
/// bounded by the IOVA window instead.
const BO_MAX: u64 = 256 * 1024 * 1024;

/// BO_WRITE/BO_READ stage through a kernel buffer of at most this many bytes per step, so a
/// large copy needs no large contiguous kmalloc.
const COPY_CHUNK: usize = 256 * 1024;

impl EdgeTpuFileData {
    /// Allocate a carveout BO and return its handle + TPU device-VA.
    pub(crate) fn create_bo(
        ddev: &EdgeTpuDevice<drm::Registered>,
        _reg: &(),
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

    /// Import a dma-buf as a BO and return its handle, TPU-VA and size.
    pub(crate) fn import_bo(
        ddev: &EdgeTpuDevice<drm::Registered>,
        _reg: &(),
        args: &mut uapi::drm_edgetpu_import_bo,
        _file: &EdgeTpuFile,
    ) -> Result<u32> {
        let dev = ddev.pdev.as_ref().as_raw();
        let mut st = ddev.mbox.lock();
        let (handle, iova, size) = st.bo.import(args.fd, dev)?;
        args.handle = handle;
        args.tpu_va = iova;
        args.size = size;
        Ok(0)
    }

    /// Copy userspace bytes into a BO.
    pub(crate) fn bo_write(
        ddev: &EdgeTpuDevice<drm::Registered>,
        _reg: &(),
        args: &mut uapi::drm_edgetpu_bo_write,
        _file: &EdgeTpuFile,
    ) -> Result<u32> {
        let cpu_va = Self::host_bo(ddev, args.handle, args.offset, args.size)?;
        // Copy after dropping the lock: the copy can be large and the backing memory is stable
        // for this client (only its own RESET/close frees it). Stage chunk by chunk, so no large
        // kernel buffer is needed and a user fault leaves nothing half-staged in the lock.
        let len = args.size as usize;
        let mut buf = KVec::with_capacity(len.min(COPY_CHUNK), GFP_KERNEL)?;
        let mut reader = UserSlice::new(UserPtr::from_addr(args.user_ptr as usize), len).reader();
        let mut done = 0usize;
        while done < len {
            let n = (len - done).min(COPY_CHUNK);
            buf.clear();
            buf.resize(n, 0, GFP_KERNEL)?;
            reader.read_slice(&mut buf)?;
            mem::bo_write(cpu_va, args.offset + done as u64, &buf);
            done += n;
        }
        Ok(0)
    }

    /// Validate a host copy of `size` bytes at `offset` into BO `handle` and return the BO's
    /// kernel VA. Imported dma-bufs have none (the host uses the dma-buf's own mmap).
    fn host_bo(ddev: &EdgeTpuDevice<drm::Registered>, handle: u32, offset: u64, size: u64) -> Result<u64> {
        if size == 0 || size > BO_MAX {
            return Err(EINVAL);
        }
        let st = ddev.mbox.lock();
        let bo = st.bo.get(handle).ok_or(EINVAL)?;
        let end = offset.checked_add(size).ok_or(EINVAL)?;
        if end > bo.size || bo.cpu_va == 0 {
            return Err(EINVAL);
        }
        Ok(bo.cpu_va)
    }

    /// Copy BO bytes back out to userspace.
    pub(crate) fn bo_read(
        ddev: &EdgeTpuDevice<drm::Registered>,
        _reg: &(),
        args: &mut uapi::drm_edgetpu_bo_read,
        _file: &EdgeTpuFile,
    ) -> Result<u32> {
        let cpu_va = Self::host_bo(ddev, args.handle, args.offset, args.size)?;
        let len = args.size as usize;
        let mut buf = KVec::with_capacity(len.min(COPY_CHUNK), GFP_KERNEL)?;
        let mut writer = UserSlice::new(UserPtr::from_addr(args.user_ptr as usize), len).writer();
        let mut done = 0usize;
        while done < len {
            let n = (len - done).min(COPY_CHUNK);
            buf.clear();
            buf.resize(n, 0, GFP_KERNEL)?;
            mem::bo_read(cpu_va, args.offset + done as u64, &mut buf);
            writer.write_slice(&buf)?;
            done += n;
        }
        Ok(0)
    }

    /// Submit one VII command element to the mailbox selected by `args.context`
    /// (0 or 1) and return its response. Routing by context lets a client keep
    /// two concurrent VCID contexts and dispatch each registered graph to the
    /// mailbox that holds its registration (register-once-dispatch).
    pub(crate) fn submit(
        ddev: &EdgeTpuDevice<drm::Registered>,
        _reg: &(),
        args: &mut uapi::drm_edgetpu_submit,
        _file: &EdgeTpuFile,
    ) -> Result<u32> {
        let ctx = args.context as usize;
        if ctx >= NUM_VII {
            return Err(EINVAL);
        }
        let cmd: [u8; CMD_ELEM] = args.command;
        let mut st = ddev.mbox.lock();
        if st.lost {
            return Err(EPIPE);
        }
        let resp: [u8; RESP_ELEM] = match st.vii[ctx].submit(&cmd, args.timeout_ms) {
            Ok(r) => r,
            Err(e) => {
                if e == ETIMEDOUT {
                    st.recover(ddev.pdev.as_ref(), ctx);
                }
                return Err(e);
            }
        };
        st.stalls[ctx] = 0;
        args.response = resp;
        args.context = 0;
        Ok(0)
    }

    /// Reset the client's VII context (fresh VCID) and reclaim the BO heap — the
    /// same work done on a fresh DRM open, exposed as an ioctl so a long-lived
    /// client can recycle mid-run without re-opening the fd. See
    /// [`MailboxState::reset_client`].
    pub(crate) fn reset(
        ddev: &EdgeTpuDevice<drm::Registered>,
        _reg: &(),
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
            let mut st = ddev.mbox.lock();
            st.power_cycle(pdev)?;
            // The client asked for this restart, so it knows its registrations are gone.
            st.stalls = [0; NUM_VII];
            st.lost = false;
        } else {
            ddev.mbox.lock().reset_client(pdev, raw);
        }
        Ok(0)
    }
}
