// SPDX-License-Identifier: GPL-2.0

use kernel::{
    clk::{Clk, Hertz},
    device::Core,
    dma::{
        Device as DmaDevice,
        DmaMask, //
    },
    drm,
    of,
    platform,
    prelude::*,
    sizes::{SZ_2M, SZ_64K},
    sync::aref::ARef,
};

use crate::file::EdgeTpuFileData;
use crate::gem::BoData;

// The Edge TPU "TOP" CSR block is a single MMIO window; the highest register we
// touch (LPM/PSM at ~0x1d0068) fits comfortably in 2 MiB. M1 will retain the
// mapping (as `IoMem<'_, SZ_2M>`) via devres for the KCI/power path; M0 only
// validates the resource, so no persistent alias is needed yet.

pub(crate) struct EdgeTpuDriver;

/// Convenience alias for the DRM device type for this driver.
pub(crate) type EdgeTpuDevice<Ctx = drm::Registered> = drm::Device<EdgeTpuDriver, Ctx>;

pub(crate) struct EdgeTpuPlatformDriver;

#[pin_data(PinnedDrop)]
pub(crate) struct EdgeTpuPlatformData {
    _device: ARef<EdgeTpuDevice>,
    // The ACPM DVFS clock ("tpu", channel 7) is the TPU block's rail-power
    // primitive (same model as the mainline GPU). Held for the device's
    // lifetime so the rail stays powered while the firmware runs.
    _clk: Clk,
}

#[pin_data]
pub(crate) struct EdgeTpuData {
    pub(crate) pdev: ARef<platform::Device>,
}

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    <EdgeTpuPlatformDriver as platform::Driver>::IdInfo,
    [(of::DeviceId::new(c"google,edgetpu-gs201"), ())]
);

impl platform::Driver for EdgeTpuPlatformDriver {
    type IdInfo = ();
    type Data<'bound> = EdgeTpuPlatformData;
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        _info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        // Map the two reg windows: index 0 = main TPU CSR block (2 MiB),
        // index 1 = SSMT stream-ID table (64 KiB). Both are used by the M1a
        // power/SSMT sequence and dropped at the end of probe (M1b retains the
        // CSR window via devres for the KCI mailbox).
        let reg = pdev
            .io_request_by_index(0)
            .ok_or(ENODEV)?
            .iomap_sized::<SZ_2M>()?;
        let ssmt = pdev
            .io_request_by_index(1)
            .ok_or(ENODEV)?
            .iomap_sized::<SZ_64K>()?;

        // The Edge TPU is a 36-bit DMA master (rio: dma_set_mask(36)).
        // SAFETY: the device is still probing; no concurrent DMA use.
        unsafe { pdev.dma_set_mask_and_coherent(DmaMask::try_new(36)?)? };

        // Power the TPU rail via the ACPM DVFS channel (clk "tpu"). Requesting a
        // non-zero rate is what actually powers the block on this port — there is
        // no genpd yet (same as the mainline GPU). Kept alive in the driver data.
        let clk = Clk::get(pdev.as_ref(), Some(c"tpu"))?;
        clk.set_rate(Hertz::from_mhz(627))?;
        clk.prepare_enable()?;

        let platform: ARef<platform::Device> = pdev.into();

        let data = try_pin_init!(EdgeTpuData {
            pdev: platform.clone(),
        });

        let tdev = drm::UnregisteredDevice::<EdgeTpuDriver>::new(pdev.as_ref(), data)?;
        let tdev = drm::driver::Registration::new_foreign_owned(tdev, pdev.as_ref(), 0)?;

        dev_info!(pdev, "Edge TPU (gs201): accel device registered.\n");

        // M1a firmware bring-up: authenticate + start the secure firmware via
        // GSA. Logged but non-fatal so the accel device stays bound for
        // inspection even if the handshake fails.
        match crate::bringup::firmware_bringup(pdev.as_ref(), &reg, &ssmt) {
            Ok(()) => dev_info!(pdev, "edgetpu: M1a bring-up complete\n"),
            Err(e) => dev_err!(pdev, "edgetpu: M1a bring-up failed: {:?}\n", e),
        }

        Ok(EdgeTpuPlatformData {
            _device: tdev.into(),
            _clk: clk,
        })
    }
}

#[pinned_drop]
impl PinnedDrop for EdgeTpuPlatformData {
    fn drop(self: Pin<&mut Self>) {}
}

const INFO: drm::DriverInfo = drm::DriverInfo {
    major: 0,
    minor: 1,
    patchlevel: 0,
    name: c"edgetpu",
    desc: c"Google Edge TPU (gs201) accelerator",
};

#[vtable]
impl drm::Driver for EdgeTpuDriver {
    type Data = EdgeTpuData;
    type File = EdgeTpuFileData;
    type Object<R: drm::DeviceContext> = drm::gem::shmem::Object<BoData, R>;

    const INFO: drm::DriverInfo = INFO;
    const FEAT_ACCEL: bool = true;

    // M0 exposes no ioctls; the CREATE_BO/SUBMIT/PREP/FINI ABI lands in M2.
    const IOCTLS: &'static [drm::ioctl::DrmIoctlDescriptor] = &[];
}
