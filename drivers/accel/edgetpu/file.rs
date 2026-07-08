// SPDX-License-Identifier: GPL-2.0

use kernel::{
    drm,
    prelude::*, //
};

use crate::driver::EdgeTpuDriver;

/// Per-open client state. The mailbox + BO heap are device-global (a single
/// VII context), so the per-fd data is empty; opening a new fd reclaims the BO
/// heap so a fresh client starts with an empty carveout.
#[pin_data]
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
        KBox::try_pin_init(try_pin_init!(Self {}), GFP_KERNEL)
    }
}
