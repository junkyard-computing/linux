// SPDX-License-Identifier: GPL-2.0
//! DRM ioctl handlers for the Edge TPU accel driver (M2 UAPI).
//!
//! The surface is deliberately thin: allocate carveout-backed buffer objects
//! (returning a TPU device-VA), copy bytes in/out of them, and submit a single
//! VII command element to the inference mailbox. Userspace (finch) builds the
//! DarwiNN command payloads; the kernel only moves bytes and rings doorbells.

use kernel::{
    alloc::KVec,
    prelude::*,
    uaccess::{UserPtr, UserSlice},
    uapi,
};

use crate::driver::EdgeTpuDevice;
use crate::file::{EdgeTpuFile, EdgeTpuFileData};
use crate::kci::Kci;
use crate::mem::{self, BoAllocator, BO_HEAP_END, BO_HEAP_PHYS};
use crate::vii::{Vii, CMD_ELEM, RESP_ELEM};

/// Device-global mailbox + buffer state, guarded by a mutex in the DRM device
/// data. A single client (finch) drives one VII context at a time.
pub(crate) struct MailboxState {
    pub(crate) kci: Kci,
    pub(crate) vii: Vii,
    pub(crate) bo: BoAllocator,
}

/// Largest BO the heap could hold — used to reject absurd allocation requests
/// before touching the allocator.
const BO_MAX: u64 = BO_HEAP_END - BO_HEAP_PHYS;

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

        let st = ddev.mbox.lock();
        let bo = st.bo.get(args.handle).ok_or(EINVAL)?;
        let end = args.offset.checked_add(args.size).ok_or(EINVAL)?;
        if end > bo.size {
            return Err(EINVAL);
        }
        mem::write(bo.phys + args.offset, &buf)?;
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
        let phys = {
            let st = ddev.mbox.lock();
            let bo = st.bo.get(args.handle).ok_or(EINVAL)?;
            let end = args.offset.checked_add(args.size).ok_or(EINVAL)?;
            if end > bo.size {
                return Err(EINVAL);
            }
            bo.phys + args.offset
        };

        let mut buf = KVec::with_capacity(len, GFP_KERNEL)?;
        buf.resize(len, 0, GFP_KERNEL)?;
        mem::read(phys, &mut buf)?;
        UserSlice::new(UserPtr::from_addr(args.user_ptr as usize), len)
            .writer()
            .write_slice(&buf)?;
        Ok(0)
    }

    /// Submit one VII command element and return its response.
    pub(crate) fn submit(
        ddev: &EdgeTpuDevice,
        args: &mut uapi::drm_edgetpu_submit,
        _file: &EdgeTpuFile,
    ) -> Result<u32> {
        let cmd: [u8; CMD_ELEM] = args.command;
        let mut st = ddev.mbox.lock();
        let resp: [u8; RESP_ELEM] = st.vii.submit(&cmd, args.timeout_ms)?;
        args.response = resp;
        args.pad = 0;
        Ok(0)
    }
}
