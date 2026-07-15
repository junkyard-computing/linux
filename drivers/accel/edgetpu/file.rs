// SPDX-License-Identifier: GPL-2.0

use kernel::{
    drm,
    prelude::*, //
};

use crate::driver::EdgeTpuDriver;

/// Per-open client state. The mailbox + BO heap are device-global (a single
/// VII context), so the per-fd data is empty; opening a new fd reclaims the BO
/// heap so a fresh client starts with an empty carveout. Holding this data also
/// holds one MIF/INT bus bandwidth vote (raised in [`open`], dropped in `Drop`),
/// so the memory buses run at max only while a client is attached.
#[pin_data(PinnedDrop)]
pub(crate) struct EdgeTpuFileData {}

/// Convenience type alias for our DRM `File` type.
pub(crate) type EdgeTpuFile = drm::file::File<EdgeTpuFileData>;

impl drm::file::DriverFile for EdgeTpuFileData {
    type Driver = EdgeTpuDriver;

    fn open(dev: &drm::Device<Self::Driver>) -> Result<Pin<KBox<Self>>> {
        // Fresh session for the new client: reset the VII inference context (so
        // the firmware's per-context registrations don't accumulate across
        // inferences) and reclaim/unmap the carveout BO heap.
        let pdev = dev.pdev.as_ref();
        let raw = pdev.as_raw();
        dev.mbox.lock().reset_client(pdev, raw);
        let data = KBox::try_pin_init(try_pin_init!(Self {}), GFP_KERNEL)?;
        // Raise the MIF/INT buses to max for the duration this client is attached
        // (the bus governor can't see the firmware's DMA traffic; without this
        // every op runs ~5x slower). Dropped when the fd closes (PinnedDrop). Taken
        // only after the data is created, so its Drop always pairs this get.
        crate::mem::bus_qos_get();
        Ok(data)
    }
}

#[pinned_drop]
impl PinnedDrop for EdgeTpuFileData {
    fn drop(self: Pin<&mut Self>) {
        // Client detached (fd closed): drop its bus vote. When the last client
        // leaves, MIF/INT fall back to the devfreq governor's floor.
        crate::mem::bus_qos_put();
    }
}
