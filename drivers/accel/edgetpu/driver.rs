// SPDX-License-Identifier: GPL-2.0

use kernel::{
    bindings,
    clk::{Clk, Hertz},
    device::Core,
    dma::{
        Device as DmaDevice,
        DmaMask, //
    },
    drm,
    drm::ioctl,
    new_mutex,
    of,
    platform,
    prelude::*,
    sizes::{SZ_2M, SZ_64K},
    sync::{
        aref::ARef,
        Mutex, //
    },
};

use crate::file::EdgeTpuFileData;
use crate::gem::BoData;
use crate::ioctl::MailboxState;
use crate::kci::Kci;
use crate::mem::BoAllocator;
use crate::vii::Vii;

// The Edge TPU "TOP" CSR block is a single MMIO window; the mailbox/power CSRs
// we touch fit in 2 MiB. M2 drives the runtime VII mailbox from ioctls via a
// module-lifetime CSR mapping in the C companion (see `csr` + `edgetpu_gsa.c`),
// so this probe-time `IoMem` only needs to survive the boot bring-up.

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

    /// Mailbox + buffer-object state, shared by the ioctl handlers.
    #[pin]
    pub(crate) mbox: Mutex<MailboxState>,
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
        // Gate the whole probe on GSA/Trusty IPC being reachable, BEFORE acquiring
        // any resources. edgetpu's platform device probes ~10-80 ms before the
        // trusty_ipc virtio transport comes online, so on the very first cold boot
        // the GSA channel can't open (tipc_create_channel -> -ENOENT) and firmware
        // bring-up would abort permanently while the accel node still registers
        // (making the dead TPU look alive). Returning EPROBE_DEFER asks the driver
        // core to retry once trusty_ipc is up.
        //
        // This must be the very first thing so a deferral unwinds cleanly: no reg
        // regions are requested yet, and — critically — the "tpu" clock is not yet
        // prepare_enable'd. `Clk`'s Drop only clk_put's; it does NOT
        // clk_disable_unprepare, so deferring after clk.prepare_enable() would leak
        // one clock-enable ref per retry. A bare GET_STATE query needs neither the
        // reg mapping nor the TPU rail — only the GSA Trusty channel.
        match crate::gsa::Gsa::get(pdev.as_ref())
            .and_then(|g| g.send_cmd(crate::gsa::GSA_TPU_GET_STATE))
        {
            Ok(_) => {}
            Err(e) if e == ENOENT => {
                dev_info!(pdev, "edgetpu: GSA/Trusty IPC not ready yet, deferring probe\n");
                Err::<(), _>(EPROBE_DEFER)?;
            }
            // Any other error: proceed and let the real bring-up below report it.
            Err(_) => {}
        }

        // Map the two reg windows: index 0 = main TPU CSR block (2 MiB),
        // index 1 = SSMT stream-ID table (64 KiB). Used by the boot bring-up
        // and dropped at the end of probe.
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
        //
        // Bring the TPU up at UUD (226 MHz) — the janeiro DVFS *floor*, not the
        // ceiling (config-pwr-state.h: UUD 226 / SUD 627 / UD 845 / NOM 1066 MHz).
        // Pinning NOM (1066) here the instant the firmware starts is a sharp
        // current step that trips the IF-PMIC (max77759) UVLO at ~12.2 s of boot,
        // *while GPU + display are also ramping* — a reboot loop
        // (0xcfcd UVLO). Coming up at the floor removes that boot-time current
        // step; the rate is raised on demand for inference via the runtime knob
        // (`/sys/kernel/debug/edgetpu/tpu_clk_hz`, edgetpu_dvfs_debugfs_init).
        // The firmware runs correctly at any DVFS state, just slower per mailbox
        // op at a lower clock (SUD/627 was ~1.7× slower than NOM; the whole
        // per-submit gap vs the AOSP chardev was this clock, not driver overhead).
        let clk = Clk::get(pdev.as_ref(), Some(c"tpu"))?;
        clk.set_rate(Hertz::from_mhz(226))?; // UUD — the DVFS floor (see above)
        clk.prepare_enable()?;
        // Expose a debugfs knob to set the TPU DVFS rate at runtime (Hz). Because
        // the *boot* clock above is the safe floor, ramping to a higher state via
        // this knob is self-recovering: if a rate trips UVLO the device just
        // resets and boots again at the floor. Best-effort (diagnostic only).
        // SAFETY: FFI to the companion module; `as_raw()` is a valid device ptr.
        unsafe {
            bindings::edgetpu_dvfs_debugfs_init(pdev.as_ref().as_raw());
        }

        // Map the persistent CSR window + carveout data region used by the runtime VII mailbox path
        // (so per-submit queue access memcpy's through a held mapping, not a memremap per call).
        crate::csr::init()?;
        crate::mem::init()?;
        // Pin MIF/INT to top frequency — the firmware DMAs every inference op through memory and the
        // bus governor can't see that load, so the buses otherwise idle at ~13% and each op runs ~5x
        // slower (measured: 154 us vs 731 us per submit). Best-effort; never blocks bring-up.
        crate::mem::bus_qos_init();

        // Bring up the firmware (M1a/M1b) and activate the VII mailbox (M2).
        // Non-fatal so the accel device stays bound for inspection on failure.
        let mut kci = Kci::new();
        let mut vii = [Vii::new(0), Vii::new(1)];
        match crate::bringup::firmware_bringup(pdev.as_ref(), &reg, &ssmt, &mut kci, &mut vii) {
            Ok(()) => dev_info!(pdev, "edgetpu: M2 bring-up complete\n"),
            Err(e) => dev_err!(pdev, "edgetpu: bring-up failed: {:?}\n", e),
        }

        let platform: ARef<platform::Device> = pdev.into();

        let data = try_pin_init!(EdgeTpuData {
            pdev: platform.clone(),
            mbox <- new_mutex!(MailboxState {
                kci,
                // Each mailbox keeps the FIXED VCID it was activated on at bring-up (slot i -> VCID
                // i); every reset reuses it with first_open=true so registrations never accumulate.
                vii,
                bo: BoAllocator::new(),
                // Nobody holds the device until the node is opened; `file.rs` admits exactly one.
                client_attached: false,
            }),
        });

        let tdev = drm::UnregisteredDevice::<EdgeTpuDriver>::new(pdev.as_ref(), data)?;
        let tdev = drm::driver::Registration::new_foreign_owned(tdev, pdev.as_ref(), 0)?;

        dev_info!(pdev, "Edge TPU (gs201): accel device registered.\n");

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
    minor: 2,
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

    kernel::declare_drm_ioctls! {
        (EDGETPU_CREATE_BO, drm_edgetpu_create_bo, ioctl::RENDER_ALLOW, EdgeTpuFileData::create_bo),
        (EDGETPU_BO_WRITE, drm_edgetpu_bo_write, ioctl::RENDER_ALLOW, EdgeTpuFileData::bo_write),
        (EDGETPU_BO_READ, drm_edgetpu_bo_read, ioctl::RENDER_ALLOW, EdgeTpuFileData::bo_read),
        (EDGETPU_SUBMIT, drm_edgetpu_submit, ioctl::RENDER_ALLOW, EdgeTpuFileData::submit),
        (EDGETPU_RESET, drm_edgetpu_reset, ioctl::RENDER_ALLOW, EdgeTpuFileData::reset),
    }
}
