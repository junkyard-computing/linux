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
        // Reclaim the carveout BO heap for the new client (map-once/submit-many
        // means we never need to free individual BOs mid-session). This also
        // unmaps the prior client's buffers from the SysMMU.
        let raw = dev.pdev.as_ref().as_raw();
        dev.mbox.lock().bo.reset(raw);
        KBox::try_pin_init(try_pin_init!(Self {}), GFP_KERNEL)
    }
}
