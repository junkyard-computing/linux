// SPDX-License-Identifier: GPL-2.0

use kernel::{
    drm,
    prelude::*,
    sync::aref::ARef, //
};

use crate::driver::{EdgeTpuDevice, EdgeTpuDriver};

/// Per-open client state.
///
/// The mailbox + BO heap are device-global (a single VII context), so there is no per-client
/// resource to hold — but that same fact means the driver supports exactly ONE client at a time,
/// and this type is what makes that real: [`open`](EdgeTpuFileData::open) refuses a second attach
/// with `EBUSY`. Without it, a second open would reactivate the VII contexts (clearing the
/// firmware's graph registrations) and reclaim the shared heap out from under a LIVE client; its
/// firmware graphs then point at IOVAs whose pages have been freed and handed to someone else, and
/// the next dispatch faults the SysMMU and takes the kernel down with it.
///
/// (AOSP's downstream driver instead isolates every client — its own device group, VCID and
/// mailbox, torn down on release. We keep the single-context design, which is all finch needs, and
/// enforce it rather than assume it.)
///
/// The device reference exists so [`Drop`] can tear the session down on CLOSE: the VII contexts are
/// reactivated and the BO heap reclaimed the moment the fd goes away — including on an abnormal
/// exit — instead of lingering until some later client happens to open. Holding this data also
/// holds one MIF/INT bus bandwidth vote, so the memory buses run at max only while a client is
/// attached.
#[pin_data(PinnedDrop)]
pub(crate) struct EdgeTpuFileData {
    dev: ARef<EdgeTpuDevice>,
}

/// Convenience type alias for our DRM `File` type.
pub(crate) type EdgeTpuFile = drm::file::File<EdgeTpuFileData>;

impl drm::file::DriverFile for EdgeTpuFileData {
    type Driver = EdgeTpuDriver;

    fn open(dev: &drm::Device<Self::Driver>) -> Result<Pin<KBox<Self>>> {
        let pdev = dev.pdev.as_ref();
        let raw = pdev.as_raw();
        {
            let mut st = dev.mbox.lock();
            // One client at a time: the mailbox and BO heap are device-global, so a second client
            // cannot be admitted without clobbering the first.
            if st.client_attached {
                dev_warn!(pdev, "edgetpu: busy — a client is already attached\n");
                return Err(EBUSY);
            }
            // Fresh session for the new client: reset the VII inference contexts (so the firmware's
            // per-context registrations don't accumulate across clients) and reclaim/unmap the BO
            // heap. Safe here only because no other client holds them — the invariant `EBUSY` above
            // maintains, and `Drop` releases.
            st.reset_client(pdev, raw);
            st.client_attached = true;
        }
        match KBox::try_pin_init(try_pin_init!(Self { dev: ARef::from(dev) }), GFP_KERNEL) {
            Ok(data) => {
                // Raise the MIF/INT buses to max for the duration this client is attached (the bus
                // governor can't see the firmware's DMA traffic; without this every op runs ~5x
                // slower). Dropped when the fd closes (PinnedDrop). Taken only after the data is
                // created, so its Drop always pairs this get.
                crate::mem::bus_qos_get();
                Ok(data)
            }
            Err(e) => {
                // No `Drop` runs for a client we never finished attaching, so release the claim by
                // hand — otherwise a failed open would wedge the device closed forever.
                dev.mbox.lock().client_attached = false;
                Err(e)
            }
        }
    }
}

#[pinned_drop]
impl PinnedDrop for EdgeTpuFileData {
    fn drop(self: Pin<&mut Self>) {
        // Client detached (fd closed — including a crash, SIGKILL or SIGHUP'd orphan): tear the
        // session down NOW rather than leaving it for the next client's open. Reactivating the VII
        // contexts clears the firmware's graph registrations and reclaiming the heap unmaps their
        // IOVAs, so a dead client can never leave the firmware pointing into memory the allocator
        // subsequently hands to someone else.
        let pdev = self.dev.pdev.as_ref();
        let raw = pdev.as_raw();
        {
            let mut st = self.dev.mbox.lock();
            st.reset_client(pdev, raw);
            st.client_attached = false;
        }
        // Drop this client's bus vote. When the last client leaves, MIF/INT fall back to the
        // devfreq governor's floor.
        crate::mem::bus_qos_put();
    }
}
