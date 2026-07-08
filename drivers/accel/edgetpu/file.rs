// SPDX-License-Identifier: GPL-2.0

use kernel::{
    drm,
    prelude::*, //
};

use crate::driver::EdgeTpuDriver;

/// Per-open client state. Empty for M0; M2 adds the per-fd IOMMU domain, the
/// `drm_mm` VA allocator, and the scheduler entity.
#[pin_data]
pub(crate) struct EdgeTpuFileData {}

impl drm::file::DriverFile for EdgeTpuFileData {
    type Driver = EdgeTpuDriver;

    fn open(_dev: &drm::Device<Self::Driver>) -> Result<Pin<KBox<Self>>> {
        KBox::try_pin_init(try_pin_init!(Self {}), GFP_KERNEL)
    }
}
