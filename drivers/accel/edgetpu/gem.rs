// SPDX-License-Identifier: GPL-2.0
//! GEM buffer objects for the Edge TPU driver (shmem-backed).
//!
//! M0 defines the minimal object type required by the `drm::Driver` trait. The
//! IOMMU-mapped CREATE_BO path (per-fd device address space) lands in M2.

use kernel::{
    drm::{
        gem,
        DeviceContext, //
    },
    prelude::*, //
};

use crate::driver::{
    EdgeTpuDevice,
    EdgeTpuDriver, //
};

/// Driver-private data attached to each GEM object.
#[pin_data]
pub(crate) struct BoData {}

/// Arguments passed when creating a [`BoData`].
pub(crate) struct BoCreateArgs {}

impl gem::DriverObject for BoData {
    type Driver = EdgeTpuDriver;
    type Args = BoCreateArgs;

    fn new<Ctx: DeviceContext>(
        _dev: &EdgeTpuDevice<Ctx>,
        _size: usize,
        _args: BoCreateArgs,
    ) -> impl PinInit<Self, Error> {
        try_pin_init!(Self {})
    }
}
