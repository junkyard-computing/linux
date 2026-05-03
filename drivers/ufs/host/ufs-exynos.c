// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFS Host Controller driver for Exynos specific extensions
 *
 * Copyright (C) 2014-2015 Samsung Electronics Co., Ltd.
 * Author: Seungwon Jeon  <essuuj@gmail.com>
 * Author: Alim Akhtar <alim.akhtar@samsung.com>
 *
 */

#include <linux/unaligned.h>
#include <crypto/aes.h>
#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <scsi/scsi_device.h>

#include <ufs/ufshcd.h>
#include "ufshcd-pltfrm.h"
#include <ufs/ufshci.h>
#include <ufs/unipro.h>

#include "ufs-exynos.h"

#define DATA_UNIT_SIZE		4096

/*
 * Exynos's Vendor specific registers for UFSHCI
 */
#define HCI_TXPRDT_ENTRY_SIZE	0x00
#define PRDT_PREFETCH_EN	BIT(31)
#define HCI_RXPRDT_ENTRY_SIZE	0x04
#define HCI_1US_TO_CNT_VAL	0x0C
#define CNT_VAL_1US_MASK	0x3FF
#define HCI_UTRL_NEXUS_TYPE	0x40
#define HCI_UTMRL_NEXUS_TYPE	0x44
#define HCI_SW_RST		0x50
#define UFS_LINK_SW_RST		BIT(0)
#define UFS_UNIPRO_SW_RST	BIT(1)
#define UFS_SW_RST_MASK		(UFS_UNIPRO_SW_RST | UFS_LINK_SW_RST)
#define HCI_DATA_REORDER	0x60
#define HCI_UNIPRO_APB_CLK_CTRL	0x68
#define UNIPRO_APB_CLK(v, x)	(((v) & ~0xF) | ((x) & 0xF))
#define HCI_AXIDMA_RWDATA_BURST_LEN	0x6C
#define WLU_EN			BIT(31)
#define WLU_BURST_LEN(x)	((x) << 27 | ((x) & 0xF))
#define HCI_GPIO_OUT		0x70
#define HCI_ERR_EN_PA_LAYER	0x78
#define HCI_ERR_EN_DL_LAYER	0x7C
#define HCI_ERR_EN_N_LAYER	0x80
#define HCI_ERR_EN_T_LAYER	0x84
#define HCI_ERR_EN_DME_LAYER	0x88
#define HCI_V2P1_CTRL		0x8C
#define IA_TICK_SEL		BIT(16)
#define HCI_CLKSTOP_CTRL	0xB0
#define REFCLKOUT_STOP		BIT(4)
#define MPHY_APBCLK_STOP	BIT(3)
#define REFCLK_STOP		BIT(2)
#define UNIPRO_MCLK_STOP	BIT(1)
#define UNIPRO_PCLK_STOP	BIT(0)
#define CLK_STOP_MASK		(REFCLKOUT_STOP | REFCLK_STOP |\
				 UNIPRO_MCLK_STOP | MPHY_APBCLK_STOP|\
				 UNIPRO_PCLK_STOP)
/* HCI_MISC is also known as HCI_FORCE_HCS */
#define HCI_MISC		0xB4
#define REFCLK_CTRL_EN		BIT(7)
#define UNIPRO_PCLK_CTRL_EN	BIT(6)
#define UNIPRO_MCLK_CTRL_EN	BIT(5)
#define HCI_CORECLK_CTRL_EN	BIT(4)
#define CLK_CTRL_EN_MASK	(REFCLK_CTRL_EN |\
				 UNIPRO_PCLK_CTRL_EN |\
				 UNIPRO_MCLK_CTRL_EN)

#define HCI_IOP_ACG_DISABLE	0x100
#define HCI_IOP_ACG_DISABLE_EN	BIT(0)

/* Device fatal error */
#define DFES_ERR_EN		BIT(31)
#define DFES_DEF_L2_ERRS	(UIC_DATA_LINK_LAYER_ERROR_RX_BUF_OF |\
				 UIC_DATA_LINK_LAYER_ERROR_PA_INIT)
#define DFES_DEF_L3_ERRS	(UIC_NETWORK_UNSUPPORTED_HEADER_TYPE |\
				 UIC_NETWORK_BAD_DEVICEID_ENC |\
				 UIC_NETWORK_LHDR_TRAP_PACKET_DROPPING)
#define DFES_DEF_L4_ERRS	(UIC_TRANSPORT_UNSUPPORTED_HEADER_TYPE |\
				 UIC_TRANSPORT_UNKNOWN_CPORTID |\
				 UIC_TRANSPORT_NO_CONNECTION_RX |\
				 UIC_TRANSPORT_BAD_TC)

/* UFS Shareability */
#define UFS_EXYNOSAUTO_WR_SHARABLE	BIT(2)
#define UFS_EXYNOSAUTO_RD_SHARABLE	BIT(1)
#define UFS_EXYNOSAUTO_SHARABLE		(UFS_EXYNOSAUTO_WR_SHARABLE | \
					 UFS_EXYNOSAUTO_RD_SHARABLE)
#define UFS_EXYNOSAUTOV920_WR_SHARABLE	BIT(3)
#define UFS_EXYNOSAUTOV920_RD_SHARABLE	BIT(2)
#define UFS_EXYNOSAUTOV920_SHARABLE	(UFS_EXYNOSAUTOV920_WR_SHARABLE |\
					 UFS_EXYNOSAUTOV920_RD_SHARABLE)
#define UFS_GS101_WR_SHARABLE		BIT(1)
#define UFS_GS101_RD_SHARABLE		BIT(0)
#define UFS_GS101_SHARABLE		(UFS_GS101_WR_SHARABLE | \
					 UFS_GS101_RD_SHARABLE)
#define UFS_SHAREABILITY_OFFSET		0x710

/* Multi-host registers */
#define MHCTRL			0xC4
#define MHCTRL_EN_VH_MASK	(0xE)
#define MHCTRL_EN_VH(vh)	(vh << 1)
#define PH2VH_MBOX		0xD8

#define MH_MSG_MASK		(0xFF)

#define MH_MSG(id, msg)		((id << 8) | (msg & 0xFF))
#define MH_MSG_PH_READY		0x1
#define MH_MSG_VH_READY		0x2

#define ALLOW_INQUIRY		BIT(25)
#define ALLOW_MODE_SELECT	BIT(24)
#define ALLOW_MODE_SENSE	BIT(23)
#define ALLOW_PRE_FETCH		GENMASK(22, 21)
#define ALLOW_READ_CMD_ALL	GENMASK(20, 18)	/* read_6/10/16 */
#define ALLOW_READ_BUFFER	BIT(17)
#define ALLOW_READ_CAPACITY	GENMASK(16, 15)
#define ALLOW_REPORT_LUNS	BIT(14)
#define ALLOW_REQUEST_SENSE	BIT(13)
#define ALLOW_SYNCHRONIZE_CACHE	GENMASK(8, 7)
#define ALLOW_TEST_UNIT_READY	BIT(6)
#define ALLOW_UNMAP		BIT(5)
#define ALLOW_VERIFY		BIT(4)
#define ALLOW_WRITE_CMD_ALL	GENMASK(3, 1)	/* write_6/10/16 */

#define ALLOW_TRANS_VH_DEFAULT	(ALLOW_INQUIRY | ALLOW_MODE_SELECT | \
				 ALLOW_MODE_SENSE | ALLOW_PRE_FETCH | \
				 ALLOW_READ_CMD_ALL | ALLOW_READ_BUFFER | \
				 ALLOW_READ_CAPACITY | ALLOW_REPORT_LUNS | \
				 ALLOW_REQUEST_SENSE | ALLOW_SYNCHRONIZE_CACHE | \
				 ALLOW_TEST_UNIT_READY | ALLOW_UNMAP | \
				 ALLOW_VERIFY | ALLOW_WRITE_CMD_ALL)

#define HCI_MH_ALLOWABLE_TRAN_OF_VH		0x30C
#define HCI_MH_IID_IN_TASK_TAG			0X308

#define PH_READY_TIMEOUT_MS			(5 * MSEC_PER_SEC)

enum {
	UNIPRO_L1_5 = 0,/* PHY Adapter */
	UNIPRO_L2,	/* Data Link */
	UNIPRO_L3,	/* Network */
	UNIPRO_L4,	/* Transport */
	UNIPRO_DME,	/* DME */
};

/*
 * UNIPRO registers
 */
#define UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER0	0x7888
#define UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER1	0x788c
#define UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER2	0x7890
#define UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER0	0x78B8
#define UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER1	0x78BC
#define UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER2	0x78C0

/*
 * (h11) Direct UNIPRO sfr offsets used by AOSP gs201 ufs-cal-if's
 * `ufs_cal_pre_pmc` / `calib_of_hs_rate_b` table, addressed via
 * `unipro_writel` (i.e. raw shadow writes, not DME_SET commands).
 * Names mirror the MIB attribute they shadow.
 */
#define UNIP_DL_ERROR_IRQ_MASK_REG		0x4844	/* shadow of DL error mask */
#define UNIP_DL_PA_ERROR_IND_RECEIVED_BIT	BIT(15)
#define UNIP_PA_TXHSADAPTTYPE			0x3350	/* MIB 0x15D4 */
#define UNIP_DL_FC0PROTTIMEOUTVAL		0x4104	/* MIB 0x2041 */
#define UNIP_DL_TC0REPLAYTIMEOUTVAL		0x4108	/* MIB 0x2042 */
#define UNIP_DL_AFC0REQTIMEOUTVAL		0x410C	/* MIB 0x2043 */
#define UNIP_PA_PWRMODEUSERDATA0_REG		0x32C0	/* MIB 0x15B0 */
#define UNIP_PA_PWRMODEUSERDATA1_REG		0x32C4	/* MIB 0x15B1 */
#define UNIP_PA_PWRMODEUSERDATA2_REG		0x32C8	/* MIB 0x15B2 */

/*
 * (h13) AOSP `__set_pcs` mechanism for per-lane PHY_PCS_RX/TX writes.
 * AOSP brackets the actual register write with a lane-selector gate
 * via `UNIP_COMP_AXI_AUX_FIELD = __WSTRB | __SEL_IDX(lane)`, then
 * resets the gate after the write. Mainline does
 * `ufshcd_dme_set(UIC_ARG_MIB_SEL(addr, lane), val)` instead, which
 * goes through the controller's DME state machine. Plausible the DME
 * path silently no-ops for some PCS attributes on gs201 silicon.
 *
 *   AOSP:  unipro_writel(__WSTRB | __SEL_IDX(lane), UNIP_COMP_AXI_AUX_FIELD)
 *          unipro_writel(val, sfr)
 *          unipro_writel(__WSTRB,                   UNIP_COMP_AXI_AUX_FIELD)
 *
 * Lane encoding (from cal-if):  TX_LANE_0 = 0, TX_LANE_1 = 1,
 *                                RX_LANE_0 = 4, RX_LANE_1 = 5.
 * SFR offset for MIB n in the PCS region: 0x2000 + (n << 2).
 */
#define UNIP_COMP_AXI_AUX_FIELD		0x040
#define EXYNOS_PCS_AUX_WSTRB		(0xF << 24)
#define EXYNOS_PCS_TX_LANE_0		0
#define EXYNOS_PCS_RX_LANE_0		4
#define EXYNOS_PCS_SFR(mib)		(0x2000 + ((mib) << 2))

/*
 * gs201 bring-up experiment switches. These need to be defined HERE
 * (above the functions that use them) — the C preprocessor scans
 * top-to-bottom and `#if` against an undefined macro silently
 * evaluates to 0. Caught one of these the hard way: GS201_AOSP_PCS_WRITES
 * was originally placed in the comment block before
 * `gs101_ufs_pre_pwr_change`, AFTER `gs101_ufs_pre_link` where it's
 * used, so the AOSP __set_pcs path was being silently skipped.
 *
 *  GS201_MAINLINE_FORCE_PWM_GEAR : if non-zero, force pwr_change to
 *      SLOWAUTO_MODE × that gear. Workaround for HS-Rate-B not
 *      working on mainline gs201 (CDR lock fails post-pwr-change).
 *      Set to 0 to test HS-Rate-B for real.
 *  GS201_AOSP_PRE_PMC : port AOSP `ufs_cal_pre_pmc` semantics into
 *      `gs101_ufs_pre_pwr_change` — mask PA_ERROR_IND_RECEIVED, write
 *      UserData/L2 timers via raw unipro_writel in AOSP order.
 *  GS201_AOSP_PRE_PMC_ADAPT : within the AOSP pre_pmc port, also write
 *      PA_TxHsAdaptType=1 for HS modes. Confirmed (h11a/h11b) to break
 *      pwr_change with upmcrs:0x5 — leave at 0 until we identify the
 *      missing precondition.
 *  GS201_AOSP_PCS_WRITES : (h13) Convert per-lane PHY_PCS_RX/TX writes
 *      in `gs101_ufs_pre_link` from `ufshcd_dme_set(UIC_ARG_MIB_SEL,...)`
 *      to AOSP's `__set_pcs` mechanism (raw unipro_writel bracketed by
 *      UNIP_COMP_AXI_AUX_FIELD lane-selector gate).
 */
/*
 * (A1-A4) Tested HS at both rates and multiple gears with the FULL fork
 * stack. Result: dl_err 0x80000002 (TCx_REPLAY_TIMER_EXPIRED) on the
 * first frame after pwr_change, regardless of rate or gear. CDR-lock
 * outcome does NOT correlate with link-layer success. PWM force back
 * on so the device at least enumerates and boots through pre-udev.
 */
#define GS201_MAINLINE_FORCE_PWM_GEAR	0	/* 0 = HS verification (terminator fix); 4 = PWM-G4 fallback */
#define GS201_AOSP_PRE_PMC		1
/*
 * (A2b) Confirmed via UART log `2026-05-02_152349.log` that ADAPT at
 * Rate-A breaks pwr_change identically to Rate-B (`upmcrs:0x5`, link
 * broken). ADAPT is broken on this PHY at any rate — left at 0.
 */
#define GS201_AOSP_PRE_PMC_ADAPT	0
#define GS201_AOSP_PCS_WRITES		1
/*
 * (h9) Mask SYSTEM_BUS_FATAL_ERROR (IS bit 17) in REG_INTERRUPT_ENABLE
 * after link startup. While stuck in PWM (HS-Rate-B unresolved), the
 * (h6) SBFES wedge fires deterministically ~38s into boot when udev's
 * coldplug burst hits the controller with 4 parallel scsi_id INQUIRYs
 * + 2x 64KB READ_10s. With SBFES masked the IRQ never escalates the
 * controller to eh_fatal — best-case the command path drains and we
 * boot to login; worst-case the controller's bus state really is
 * broken and we see SCSI commands silently time out (kernel 30s
 * timeout) instead of the immediate eh_fatal cascade. Either way is
 * informative. Fork-only workaround — AOSP doesn't do this because
 * AOSP runs at HS speed and never trips SBFES.
 */
#define GS201_MASK_SBFES_IRQ		1

/*
 * (h15b) Force per-LU SCSI queue depth = 1 on gs201. After (h14)
 * (rd.udev.children-max=1) confirmed serializing udev workers does NOT
 * stop the wedge — even ONE udev-worker issuing two back-to-back 64KB
 * READ_10s at PWM gear hangs the controller (saved_err=0x0 with the
 * (h9) mask, command times out at 30s, full host reset). The previous
 * (h7) attempt to clamp via host->can_queue / cmd_per_lun in drv_init
 * failed because ufshcd overwrites those after vops->init returns.
 * The vops `config_scsi_dev` callback runs from ufshcd_sdev_configure
 * AFTER ufshcd_lu_init has set the per-LU depth from bLUQueueDepth, so
 * a scsi_change_queue_depth(sdev, 1) here actually sticks per device.
 * If h15b lets boot survive past 38s, the trigger is back-to-back
 * commands queued at the controller, not the 64KB transfer length.
 */
#define GS201_FORCE_QDEPTH_1		1

/*
 * (h15a) Clamp per-LU max_hw_sectors to GS201_MAX_HW_SECTORS_KB at PWM
 * gear on gs201. h15b confirmed queue depth = 1 doesn't help: with strict
 * one-cmd-at-a-time serialization, the SECOND back-to-back 64KB READ_10
 * to a high-LBA region (~end of disk, blkid backup-GPT probe) still hangs
 * with zero error indication ("No record of pa_err/dl_err/...", saved_err=0).
 * This clamp splits that 64KB udev probe into smaller chunks. If the wedge
 * disappears, the trigger is the 64KB transfer length itself at PWM. If
 * it persists, the trigger is reading near end-of-device at PWM, regardless
 * of length. Set to 0 to disable; set to 64 (=32KB) or 32 (=16KB) to test.
 */
#define GS201_MAX_HW_SECTORS_KB		32

/*
 * (h16) Port AOSP `ufs_cal_post_pmc` semantics for forced-PWM. Mainline
 * gates `phy_calibrate(CFG_POST_PWR_HS)` behind `ufshcd_is_hs_mode()`
 * (see exynos_ufs_pre_pwr_mode + exynos_ufs_post_pwr_mode), so when we
 * force PWM via GS201_MAINLINE_FORCE_PWM_GEAR the PHY state machine
 * never advances past CFG_PRE_PWR_HS. As a result,
 * `tensor_gs101_pre_pwr_hs_config` AND `tensor_gs101_post_pwr_hs_config`
 * never run, even though their PWR_MODE_PWM_ANY entries are exactly
 * AOSP's `post_calib_of_pwm` (3 PMA writes: 0x20=0x60 COMN,
 * 0x222=0x08 TRSV (= byte 0x888), 0x246=0x01 TRSV (= byte 0x918)).
 * Those writes might be the missing piece for the (h6/h14/h15)
 * back-to-back-READ_10 wedge at PWM gear.
 *
 * When set, gs101_ufs_post_pwr_change calls phy_calibrate() twice for
 * PWM mode to drive the state machine through PRE_PWR_HS → POST_PWR_HS,
 * applying both tables. wait_for_cdr will run on the 2nd call and
 * return -ETIMEDOUT (no CDR lock at PWM); samsung_ufs_phy_calibrate
 * propagates the error but ufs-exynos's exynos_ufs_post_pwr_mode does
 * not check the return value of phy_calibrate, so this is benign.
 */
#define GS201_AOSP_POST_PMC		1

/*
 * (h18) Force PRDT_PREFETCH_EN on HCI_TXPRDT_ENTRY_SIZE for gs201.
 * AOSP's exynos_ufs_config_host writes
 *   `hci_writel(PRDT_PREFECT_EN | PRDT_SET_SIZE(12), HCI_TXPRDT_ENTRY_SIZE)`
 * unconditionally for the gs201 path. Mainline's exynos_ufs_post_link
 * only sets PRDT_PREFETCH_EN when `hba->caps & UFSHCD_CAP_CRYPTO`. We
 * dropped UFSHCD_CAP_CRYPTO on gs201 (via the EXYNOS_UFS_OPT_UFSPR_SECURE
 * gating change in gs201_ufs_drvs), so mainline ships gs201 with TX-PRDT
 * prefetch DISABLED — the controller fetches PRDT entries on demand,
 * one-at-a-time. Strong candidate for the (h6/h14/h15) PWM back-to-back
 * READ_10 wedge: without prefetch, the second back-to-back command
 * arrives while the AXI master is still in some transient on-demand
 * fetch state from the first command. h16 ruled out the missing
 * post-PMC PMA writes; this is the next mainline-vs-AOSP HCI-level
 * difference our gs201 path actually hits.
 */
#define GS201_PRDT_PREFETCH		1

/*
 * (A2) Force HS-Rate-A instead of the negotiated HS-Rate-B at pwr_change.
 * AOSP supports both rates; mainline negotiates and ends up at Rate-B.
 * Rate-A is slower (~1.4 Gbps/lane vs ~2.9 for B) but the per-rate cal
 * tables differ (post_calib_of_hs_rate_a vs post_calib_of_hs_rate_b
 * are nearly identical, but maybe Rate-A side-steps whatever's broken
 * in the Rate-B CDR-lock path).
 *
 * Set to 0 to use the negotiated rate (default = Rate-B for our device).
 * Set to PA_HS_MODE_A (= 1) to override `pwr->hs_rate` in the
 * pre_pwr_change hook. Only effective when GS201_MAINLINE_FORCE_PWM_GEAR
 * is also 0 (otherwise we force PWM and never reach HS).
 */
#define GS201_FORCE_HS_RATE_A		0	/* A4: test Rate-B at G1 */

/*
 * (A2c) Clamp HS gear after rate selection. A2 confirmed Rate-A G4
 * locks CDR cleanly but the device never ACKs the first SCSI frame
 * (`dl_err 0x80000002 = TCx_REPLAY_TIMER_EXPIRED`). Lower symbol rates
 * may side-step whatever timing/integrity issue prevents the device
 * from ACKing at G4. Gear values: 1, 2, 3, or 4. Set to 0 to keep
 * the negotiated gear. Only effective for HS modes (PWM gear is
 * controlled by GS201_MAINLINE_FORCE_PWM_GEAR).
 */
#define GS201_FORCE_HS_GEAR		0	/* not the lever */

/*
 * (A2d) Settle delay (in ms) after a successful HS pwr_change before
 * returning from post_pwr_change. A2/A2c confirmed that at HS-Rate-A
 * any gear the device never ACKs the first SCSI frame
 * (`dl_err 0x80000002 = TCx_REPLAY_TIMER_EXPIRED`). The dl_err fires
 * within 30 ms of pwr_change — too tight to be a normal command-flow
 * issue. Hypothesis: the device-side pwr_change is not fully settled
 * when the host starts sending commands. AOSP may have an implicit
 * delay (clk_gating, runtime PM, dev_quirks) we don't replicate.
 *
 * Set non-zero to msleep that many ms after pwr_change before
 * proceeding. 100 ms is a guess; bisect down/up if it helps.
 * Set to 0 to disable.
 */
#define GS201_HS_PWR_SETTLE_MS		0	/* A2d shifted timing only */

/*
 * (C1) Probe HSI2 CMU divider/mux/gate state at strategic points around
 * pwr_change. Hypothesis from the AOSP-vs-mainline audit:
 *
 *   - mainline gs201.dtsi wires ufs_aclk and ufs_unipro as fixed-clocks
 *     because no cmu-hsi2 node is instantiated for gs201 (mainline's
 *     clk-gs101.c HAS a google,gs201-cmu-hsi2 probe handler at line
 *     ~4845 but no DT node ever hits it);
 *   - so ufshcd_setup_clocks/ufshcd_set_clk_freq don't reprogram any
 *     HSI2 divider when the kernel transitions gears;
 *   - and the BUS/MMC_CARD divider offsets are swapped between gs101
 *     and gs201 (gs101: 0x1898=BUS, 0x189c=MMC; gs201: 0x1898=MMC,
 *     0x189c=NOC/BUS) so even if it did try, the BUS write would hit
 *     the wrong divider.
 *
 * If true, this would explain: PWM works (lowest gear has forgiving
 * timing), HS-A and HS-B both wedge with dl_err 0x80000002 on the
 * first frame after PMC (controller's perceived unipro_clk doesn't
 * match the PHY's SYMBOL_CLK divisor for the new gear).
 *
 * To test the hypothesis without porting CCF infrastructure: ioremap
 * CMU_TOP and CMU_HSI2 directly, dump the relevant dividers/muxes/
 * gates at:
 *   - end of gs201_ufs_post_link        (baseline at link-up)
 *   - start of gs101_ufs_pre_pwr_change (before PMC writes)
 *   - end of gs101_ufs_post_pwr_change  (after PMC and PHY calibrate)
 *
 * If hardware values are identical at all three points, the kernel
 * really is doing nothing — and we can move to manual divider writes
 * in pre_pwr_change to test whether changing the rate fixes HS.
 *
 * Read-only probe; does not modify any register. Safe to leave on.
 *
 * NOTE: requires pKVM (kvm-arm.mode=protected) so EL1 CMU access is
 * unlocked by BL31. See memory/project_pkvm_cmu_unlock.md.
 *
 * RESULT (C1+C2 runs, 2026-05-02): bytes-identical at all three call sites
 * for both PWM-G4 and HS-Rate-B G4. HSI2 CMU is untouched across PMC. UFS_EMBD
 * divider DIV[0x18a4]=0x02 → /3 → 532.992MHz/3 = 177.664 MHz exactly matches
 * mainline's `ufs_unipro` fixed-clock stub claim. NOC divider DIV[0x189c]=0x01
 * → /2 → SHARED0_DIV4 (532.992 MHz) /2 = 266.5 MHz exactly matches `ufs_aclk`
 * stub claim of 267 MHz. So both rates are real, the clock framework not
 * touching HSI2 at PMC matches AOSP's behavior, and "wrong-rate-at-HS" is
 * fully ruled out as a cause of the dl_err 0x80000002 wedge. Switch left at
 * 1 so future bring-up work can re-enable trivially; the prints are short.
 */
#define GS201_PROBE_CMU_DIVIDERS	0

/*
 * (S1) Diagnostic: dump PA-layer device-reported / negotiated / active
 * attributes via DME_GET at strategic points around pwr_change. Ports
 * AOSP's `exynos_ufs_get_caps_after_link` + `exynos_ufs_update_active_lanes`
 * (private/google-modules/soc/gs/drivers/ufs/ufs-exynos.c:188,217) but uses
 * mainline's standard `ufshcd_dme_get(UIC_ARG_MIB(PA_*))` instead of AOSP's
 * raw `unipro_readl(handle, UNIP_PA_*)` shortcut. Same target attributes,
 * different mechanism — DME_GET goes through the controller's DME state
 * machine instead of reading a vendor-specific shadow MMIO.
 *
 * Attributes dumped:
 *   PA_MAXRXHSGEAR        — device-reported max HS gear (host learned at link)
 *   PA_CONNECTEDTXDATALANES, PA_CONNECTEDRXDATALANES — negotiated lane count
 *   PA_ACTIVETXDATALANES,    PA_ACTIVERXDATALANES    — active lanes post-PMC
 *   PA_PWRMODE                                      — active power mode
 *
 * Hypothesis: if `PA_ACTIVE*` differs from what mainline thinks (e.g. host
 * thinks 2 lanes but device actually entered 1 lane), that's a smoking gun
 * for "device entered a different mode than host expects" — which exactly
 * matches the dl_err 0x80000002 / device-never-ACKs-first-frame symptom.
 *
 * Read-only, no register writes. Safe to leave on. Default 1 — turn off
 * once the diagnostic question is answered.
 *
 * RESULT (S1 run, 2026-05-02): PA-layer state on host MATCHES what was
 * requested at HS PMC. post_link: MaxRxHSGear=4, Connected[2,2], Active[1,1],
 * PwrMode=0x55 (SLOWAUTO,SLOWAUTO). post_pwr_HS (HS-Rate-B G4 attempt,
 * after PMC reports success, before first frame fails): MaxRxHSGear=4,
 * Connected[2,2], Active[2,2], PwrMode=0x11 (FAST,FAST). Device fully
 * acknowledged the mode change. First frame STILL fails with dl_err
 * 0x80000002. Bug is below the PA layer (M-PHY signaling or controller-
 * internal handshake), not in mode-negotiation. Default flipped to 0.
 */
#define GS201_PROBE_PA_STATE	0

#define GS201_CMU_TOP_BASE		0x1e080000
#define GS201_CMU_HSI2_BASE		0x14400000

/*
 * UFS Protector registers
 */
#define UFSPRSECURITY	0x010
#define NSSMU		BIT(14)
#define UFSPSBEGIN0	0x200
#define UFSPSEND0	0x204
#define UFSPSLUN0	0x208
#define UFSPSCTRL0	0x20C

#define CNTR_DIV_VAL 40

static void exynos_ufs_auto_ctrl_hcc(struct exynos_ufs *ufs, bool en);
static void exynos_ufs_ctrl_clkstop(struct exynos_ufs *ufs, bool en);

static inline void exynos_ufs_enable_auto_ctrl_hcc(struct exynos_ufs *ufs)
{
	exynos_ufs_auto_ctrl_hcc(ufs, true);
}

static inline void exynos_ufs_disable_auto_ctrl_hcc(struct exynos_ufs *ufs)
{
	exynos_ufs_auto_ctrl_hcc(ufs, false);
}

static inline void exynos_ufs_disable_auto_ctrl_hcc_save(
					struct exynos_ufs *ufs, u32 *val)
{
	*val = hci_readl(ufs, HCI_MISC);
	exynos_ufs_auto_ctrl_hcc(ufs, false);
}

static inline void exynos_ufs_auto_ctrl_hcc_restore(
					struct exynos_ufs *ufs, u32 *val)
{
	hci_writel(ufs, *val, HCI_MISC);
}

static inline void exynos_ufs_gate_clks(struct exynos_ufs *ufs)
{
	exynos_ufs_ctrl_clkstop(ufs, true);
}

static inline void exynos_ufs_ungate_clks(struct exynos_ufs *ufs)
{
	exynos_ufs_ctrl_clkstop(ufs, false);
}

static int exynos_ufs_shareability(struct exynos_ufs *ufs)
{
	/* IO Coherency setting */
	if (ufs->sysreg) {
		return regmap_update_bits(ufs->sysreg,
					  ufs->iocc_offset,
					  ufs->iocc_mask, ufs->iocc_val);
	}

	return 0;
}

static int gs101_ufs_drv_init(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	u32 reg;

	/* Enable WriteBooster */
	hba->caps |= UFSHCD_CAP_WB_EN;

	/* Enable clock gating and hibern8 */
	hba->caps |= UFSHCD_CAP_CLK_GATING | UFSHCD_CAP_HIBERN8_WITH_CLK_GATING;

	/* set ACG to be controlled by UFS_ACG_DISABLE */
	reg = hci_readl(ufs, HCI_IOP_ACG_DISABLE);
	hci_writel(ufs, reg & (~HCI_IOP_ACG_DISABLE_EN), HCI_IOP_ACG_DISABLE);

	return exynos_ufs_shareability(ufs);
}

/*
 * gs201 CMU_HSI2 clock-state probe. Confirms (or rules out) the hypothesis
 * that the bootloader's "[UFS] shutdown complete" leaves UFS AXI/UNIPRO clocks
 * gated, while the PHY clock stays running. Mainline gs201.dtsi exposes UFS
 * clocks as `fixed-clock` stubs (no real gate hooks), so clk_prepare_enable
 * during probe is a no-op. If bit21 (CG_VAL) of the per-IP gate registers is
 * 0 here, the controller is running without an AXI clock — explains link
 * startup PASS + NOP-OUT response slot all-zero (controller "completed" but
 * DMA never wrote into memory).
 *
 * Offsets sourced from AOSP cal-if cmucal-sfr.c (gs201).
 */
static void gs201_dump_cmu_hsi2_ufs_gates(struct device *dev)
{
	void __iomem *base = ioremap(0x14400000, 0x4000);
	if (!base) {
		dev_err(dev, "CMU-HSI2 dump: ioremap failed\n");
		return;
	}

#define DUMP(name, off) \
	do { \
		u32 v = readl(base + (off)); \
		dev_dbg(dev, "CMU-HSI2 %-44s @0x%04x = 0x%08x  CG_VAL=%u  MANUAL=%u\n", \
			 name, (off), v, !!(v & BIT(21)), !!(v & BIT(20))); \
	} while (0)

	DUMP("PLL_CON0_MUX_CLKCMU_HSI2_UFS_EMBD_USER", 0x0630);
	DUMP("PLL_CON1_MUX_CLKCMU_HSI2_UFS_EMBD_USER", 0x0634);
	DUMP("CLK_CON_GAT_QE_UFS_EMBD_HSI2_ACLK     ", 0x20a4);
	DUMP("CLK_CON_GAT_QE_UFS_EMBD_HSI2_PCLK     ", 0x20a8);
	DUMP("CLK_CON_GAT_UFS_EMBD_I_ACLK           ", 0x20e8);
	DUMP("CLK_CON_GAT_UFS_EMBD_I_CLK_UNIPRO     ", 0x20ec);
	DUMP("CLK_CON_GAT_UFS_EMBD_I_FMP_CLK        ", 0x20f0);
#undef DUMP

	iounmap(base);
}

/*
 * Dump UFSP (UFS Protector / SMU security-fence) registers. mainline writes
 * UFSPSBEGIN0/END0/LUN0/CTRL0 in `exynos_ufs_config_smu` to put the fence
 * in pass-through mode for non-secure DMA. On gs201 these direct MMIO writes
 * may be silently absorbed by the bus (no SError observed) without actually
 * programming the fence — leaving it in a default "block all non-secure"
 * state. That would match our symptom: controller completes UPIU transactions
 * with no error, but device response never reaches memory because the fence
 * drops all DMA. Read the UFSP regs and check if our writes took effect.
 */
static void gs201_dump_ufsp(struct device *dev, const char *label)
{
	/*
	 * UFSPSBEGIN0/END0/LUN0/CTRL0 live at offset 0x200+, so a 0x100-byte
	 * ioremap (matching the DT region size) misses them. Map the full page
	 * to capture all the relevant registers.
	 */
	void __iomem *base = ioremap(0x14600000, 0x1000);
	if (!base) {
		dev_err(dev, "UFSP dump (%s): ioremap failed\n", label);
		return;
	}
	dev_dbg(dev, "UFSP %s: SECURITY=0x%08x SBEGIN0=0x%08x SEND0=0x%08x SLUN0=0x%08x SCTRL0=0x%08x\n",
		 label,
		 readl(base + 0x010),	/* UFSPRSECURITY */
		 readl(base + 0x200),	/* UFSPSBEGIN0   */
		 readl(base + 0x204),	/* UFSPSEND0     */
		 readl(base + 0x208),	/* UFSPSLUN0     */
		 readl(base + 0x20c));	/* UFSPSCTRL0    */
	iounmap(base);
}

#if GS201_PROBE_CMU_DIVIDERS
/*
 * (C1) Read-only probe of HSI2 clock-controller state from CMU_TOP and
 * CMU_HSI2. Dumps four CMU_TOP HSI2 dividers (0x1898/0x189c/0x18a0/0x18a4)
 * and the corresponding CMU_HSI2 USER muxes / IPCLKPORT gates. The four
 * TOP dividers cover the range [BUS, MMC_CARD, PCIE, UFS_EMBD] under both
 * the gs101 (mainline) and gs201 (AOSP cal-if) interpretations:
 *
 *   gs101:  0x1898=BUS    0x189c=MMC_CARD  0x18a0=PCIE  0x18a4=UFS_EMBD
 *   gs201:  0x1898=MMC    0x189c=NOC/BUS   0x18a0=PCIE  0x18a4=UFS_EMBD
 *
 * UFS_EMBD divider is at the same offset on both, so the value at 0x18a4
 * is unambiguous. The other three offsets are dumped raw; interpret per
 * the table above. PCIE divider at 0x18a0 is unaffected by the swap.
 *
 * If the values are byte-identical between the three call sites
 * (post_link, pre_pwr_change, post_pwr_change) — and they should be,
 * with mainline's fixed-clock stubs — the kernel really is doing nothing
 * to HSI2 dividers across PMC. That confirms the C1 hypothesis and
 * justifies a follow-up experiment that manually programs the divider.
 */
static void gs201_dump_cmu_hsi2(struct device *dev, const char *tag)
{
	void __iomem *top, *hsi2;

	top = ioremap(GS201_CMU_TOP_BASE, 0x10000);
	if (!top) {
		dev_warn(dev, "CMU probe (%s): CMU_TOP ioremap failed\n", tag);
		return;
	}
	hsi2 = ioremap(GS201_CMU_HSI2_BASE, 0x10000);
	if (!hsi2) {
		dev_warn(dev, "CMU probe (%s): CMU_HSI2 ioremap failed\n", tag);
		iounmap(top);
		return;
	}

	/*
	 * CMU_TOP HSI2 dividers (raw — see the swap note above).
	 * Layout per Samsung CMU divider register:
	 *   bits  3:0 = DIVRATIO   (actual divider = DIVRATIO + 1)
	 *   bit  16   = BUSY (transient; 0 == idle)
	 *   bit  28   = ENABLE_AUTOMATIC_CLKGATING
	 *   bit  30   = OVERRIDE_BY_HCH
	 */
	dev_dbg(dev,
		 "CMU probe (%s) TOP: DIV[0x1898]=0x%08x DIV[0x189c]=0x%08x DIV[0x18a0:PCIE]=0x%08x DIV[0x18a4:UFS_EMBD]=0x%08x\n",
		 tag,
		 readl(top + 0x1898),
		 readl(top + 0x189c),
		 readl(top + 0x18a0),
		 readl(top + 0x18a4));

	/*
	 * CMU_TOP HSI2 muxes (which input PLL feeds each divider). gs201
	 * cal-if mux offsets are 0x10a0/0x10a4/0x10a8/0x109c (NOC, PCIE,
	 * UFS_EMBD, MMC_CARD). gs101 mainline thinks 0x10a0/4/8/c are
	 * BUS/MMC/PCIE/UFS_EMBD. Same swap caveat as above.
	 */
	dev_dbg(dev,
		 "CMU probe (%s) TOP: MUX[0x10a0]=0x%08x MUX[0x10a4]=0x%08x MUX[0x10a8]=0x%08x MUX[0x10ac]=0x%08x\n",
		 tag,
		 readl(top + 0x10a0),
		 readl(top + 0x10a4),
		 readl(top + 0x10a8),
		 readl(top + 0x10ac));

	/*
	 * CMU_HSI2 USER muxes and IPCLKPORT gates for UFS:
	 *   0x0600 PLL_CON0_MUX_CLKCMU_HSI2_NOC_USER       (BUS user mux)
	 *   0x0630 PLL_CON0_MUX_CLKCMU_HSI2_UFS_EMBD_USER  (UFS_EMBD user mux)
	 *   0x20e8 GAT_GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_ACLK
	 *   0x20ec GAT_GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_CLK_UNIPRO
	 *   0x20f0 GAT_GOUT_BLK_HSI2_UID_UFS_EMBD_IPCLKPORT_I_FMP_CLK
	 *   0x1800 DIV_CLK_HSI2_NOCP   (NOC/peripheral clock divider)
	 *   0x1804 DIV_CLK_HSI2_NOC_LH (NOC long-hop divider)
	 */
	dev_dbg(dev,
		 "CMU probe (%s) HSI2: NOC_USER=0x%08x UFS_EMBD_USER=0x%08x ACLK_GATE=0x%08x UNIPRO_GATE=0x%08x FMP_GATE=0x%08x NOCP_DIV=0x%08x NOC_LH_DIV=0x%08x\n",
		 tag,
		 readl(hsi2 + 0x0600),
		 readl(hsi2 + 0x0630),
		 readl(hsi2 + 0x20e8),
		 readl(hsi2 + 0x20ec),
		 readl(hsi2 + 0x20f0),
		 readl(hsi2 + 0x1800),
		 readl(hsi2 + 0x1804));

	iounmap(hsi2);
	iounmap(top);
}
#else
static inline void gs201_dump_cmu_hsi2(struct device *dev, const char *tag) {}
#endif

#if GS201_PROBE_PA_STATE
/*
 * (S1) Read-only DME_GET dump of PA-layer attributes. See block comment near
 * GS201_PROBE_PA_STATE define for the hypothesis being tested.
 *
 * DME_GET is allowed before link startup (returns garbage) and after, but is
 * really only meaningful after link startup completes. Don't call from
 * pre_link or earlier.
 */
static void gs201_dump_pa_state(struct ufs_hba *hba, const char *tag)
{
	u32 max_hs_gear = 0, ctxlanes = 0, crxlanes = 0;
	u32 atxlanes = 0, arxlanes = 0, pwrmode = 0;
	int e1, e2, e3, e4, e5, e6;

	e1 = ufshcd_dme_get(hba, UIC_ARG_MIB(PA_MAXRXHSGEAR), &max_hs_gear);
	e2 = ufshcd_dme_get(hba, UIC_ARG_MIB(PA_CONNECTEDTXDATALANES), &ctxlanes);
	e3 = ufshcd_dme_get(hba, UIC_ARG_MIB(PA_CONNECTEDRXDATALANES), &crxlanes);
	e4 = ufshcd_dme_get(hba, UIC_ARG_MIB(PA_ACTIVETXDATALANES), &atxlanes);
	e5 = ufshcd_dme_get(hba, UIC_ARG_MIB(PA_ACTIVERXDATALANES), &arxlanes);
	e6 = ufshcd_dme_get(hba, UIC_ARG_MIB(PA_PWRMODE), &pwrmode);

	dev_dbg(hba->dev,
		 "S1 PA state (%s): MaxRxHSGear=%u (e=%d) Connected[Tx=%u Rx=%u] (e=%d,%d) Active[Tx=%u Rx=%u] (e=%d,%d) PwrMode=0x%02x (e=%d)\n",
		 tag,
		 max_hs_gear, e1,
		 ctxlanes, crxlanes, e2, e3,
		 atxlanes, arxlanes, e4, e5,
		 pwrmode, e6);
}
#else
static inline void gs201_dump_pa_state(struct ufs_hba *hba, const char *tag) {}
#endif

/*
 * gs201 SMU/UFSP init via SMC. mainline's exynos_ufs_config_smu writes UFSP
 * registers directly; the UFSP region returns a bus-fault sentinel
 * (0xffe26492) for every read from EL1, so the writes are silently absorbed
 * too. The fence stays in whatever state BL31 left it in, which (based on
 * the symptom — controller fires UTRCS without doing any DMA) is "block all
 * non-secure DMA".
 *
 * AOSP calls BOTH SMC_CMD_FMP_SECURITY and SMC_CMD_SMU(SMU_INIT) to have
 * BL31 program the fence. The previous attempt called only SMC_CMD_SMU;
 * try the full pair this time. We use CFG_DESCTYPE_3 like AOSP does — the
 * existing comment warns BL31 doesn't program DESCTYPE on gs201 anyway,
 * so it shouldn't matter for our 16-byte PRDT setup.
 *
 * SMC IDs from AOSP soc/samsung/exynos-smc.h:
 *   SMC_CMD_FMP_SECURITY = 0xC2001810
 *   SMC_CMD_SMU          = 0xC2001850
 *   SMU_INIT             = 0
 *   SMU_EMBEDDED         = 0
 *   CFG_DESCTYPE_3       = 3
 */
#define SMC_CMD_FMP_SECURITY	0xC2001810UL
#define SMC_CMD_SMU		0xC2001850UL
#define SMC_CMD_FMP_SMU_RESUME	0xC2001860UL
#define SMC_CMD_FMP_SMU_DUMP	0xC2001870UL
#define SMU_INIT		0UL
#define SMU_EMBEDDED		0UL
#define CFG_DESCTYPE_3		3UL

static int gs201_ufs_smu_init(struct device *dev)
{
	struct arm_smccc_res res;

	/*
	 * Set FMPSECURITY0.DESCTYPE=0 (16-byte standard PRDT entries) to match
	 * the mainline ufshcd default sg_entry_size. AOSP uses DESCTYPE=3 with
	 * 128-byte fmp_sg_entry for inline crypto; we don't enable FMP, so
	 * DESCTYPE must be 0. An earlier probe loop here ended at DESCTYPE=3,
	 * leaving the controller expecting 128-byte stride while we wrote
	 * 16-byte entries — single-entry PRDTs (INQUIRY) succeeded but the
	 * first multi-entry transfer (READ_10 32 KB = 8 PRDs) hung at
	 * tag 6 with OCS=0xf because entries 1..7 were read from beyond the
	 * 128-byte software PRDT region (zero/garbage DBA).
	 */
	arm_smccc_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, 0,
		      0, 0, 0, 0, &res);
	dev_dbg(dev, "SMC_CMD_FMP_SECURITY(0, SMU_EMBEDDED, DESCTYPE=0) -> a0=0x%lx a1=0x%lx\n",
		 res.a0, res.a1);

	arm_smccc_smc(SMC_CMD_SMU, SMU_INIT, SMU_EMBEDDED, 0, 0, 0, 0, 0, &res);
	dev_dbg(dev, "SMC_CMD_SMU(SMU_INIT, SMU_EMBEDDED) -> a0=0x%lx a1=0x%lx\n",
		 res.a0, res.a1);

	/* RESUME — name suggests "re-enable after suspend", might be what
	 * mainline needs at probe (we're effectively post-suspend from BL).
	 */
	arm_smccc_smc(SMC_CMD_FMP_SMU_RESUME, 0, SMU_EMBEDDED, 0, 0, 0, 0, 0, &res);
	dev_dbg(dev, "SMC_CMD_FMP_SMU_RESUME(0, SMU_EMBEDDED) -> a0=0x%lx a1=0x%lx\n",
		 res.a0, res.a1);

	arm_smccc_smc(SMC_CMD_FMP_SMU_DUMP, 0, SMU_EMBEDDED, 0, 0, 0, 0, 0, &res);
	dev_dbg(dev, "SMC_CMD_FMP_SMU_DUMP(0, SMU_EMBEDDED) -> a0=0x%lx a1=0x%lx a2=0x%lx a3=0x%lx\n",
		 res.a0, res.a1, res.a2, res.a3);

	return 0;
}

/*
 * Read sysreg_hsi2 + 0x710 directly. mainline's exynos_ufs_shareability uses
 * regmap_update_bits to set the IO-coherency bits there for HSI2 DMA. If
 * that write is silently absorbed (similar to UFSP), our IOCC setup is a
 * no-op and the controller's DMA may not be marked as coherent across the
 * memory hierarchy.
 *
 * sysreg_hsi2 base from gs201.dtsi: 0x14420000.
 */
static void gs201_dump_sysreg_hsi2_iocc(struct device *dev, const char *label)
{
	void __iomem *base = ioremap(0x14420000, 0x1000);
	if (!base) {
		dev_err(dev, "sysreg-HSI2 dump (%s): ioremap failed\n", label);
		return;
	}
	dev_dbg(dev,
		 "sysreg-HSI2 %s: @0x710(IOCC) = 0x%08x  @0x400(KDN_CTRL_MON) = 0x%08x\n",
		 label, readl(base + 0x710), readl(base + 0x400));
	iounmap(base);
}

/*
 * GSA mailbox shim for KDN_SET_OP_MODE(MKE=1, DT=0) — the only delta we
 * observed between AOSP-kernel (KDN_CTRL_MON=0x5, UFS works) and mainline
 * (KDN_CTRL_MON=0x4, dl_err wedge). Without porting the full GSA driver,
 * we issue the same mailbox command inline; polling instead of using the
 * GIC SPI 363 IRQ. Wire protocol mirrors AOSP's exec_mbox_cmd_sync_locked
 * (gsa_mbox.c:255). Mailbox base 0x17c90000 (gs201-gsa.dtsi:9). One-shot
 * at probe before UFS link startup. Refactor out of this driver into
 * a proper drivers/soc/samsung/exynos-gsa-mbox.c if it works.
 */
#define GSA_MBOX_BASE_PHYS		0x17c90000
#define GSA_MBOX_SIZE			0x1000
#define GSA_MBOX_INTCR0			0x0024
#define GSA_MBOX_INTMSR0		0x0030
#define GSA_MBOX_INTGR1			0x0040
#define GSA_MBOX_SR(n)			(0x0080 + (n) * 4)
#define GSA_MB_CMD_KDN_SET_OP_MODE	75
#define GSA_MB_KDN_SW_KDF_MODE		2
#define GSA_MB_KDN_UFS_DESCR_PRDT	0
#define GSA_MB_CMD_RSP_BIT		(1U << 31)

static int gs201_gsa_kdn_set_op_mode(struct device *dev)
{
	void __iomem *base;
	u32 sr0, sr1;
	int i;
	int ret = -ETIMEDOUT;

	base = ioremap(GSA_MBOX_BASE_PHYS, GSA_MBOX_SIZE);
	if (!base) {
		dev_err(dev, "kdn-shim: ioremap of GSA mailbox failed\n");
		return -ENOMEM;
	}

	/* Clear any stale response IRQ. */
	writel(0x1, base + GSA_MBOX_INTCR0);

	/* KDN_SET_OP_MODE: cmd=75, argc=2, args[0]=mode=2, args[1]=descr=0. */
	writel(GSA_MB_CMD_KDN_SET_OP_MODE, base + GSA_MBOX_SR(0));
	writel(2,                          base + GSA_MBOX_SR(1));
	writel(GSA_MB_KDN_SW_KDF_MODE,     base + GSA_MBOX_SR(2));
	writel(GSA_MB_KDN_UFS_DESCR_PRDT,  base + GSA_MBOX_SR(3));

	/* Doorbell — raises the request IRQ to GSA. */
	writel(0x1, base + GSA_MBOX_INTGR1);

	/* Poll for response (~10ms ceiling). */
	for (i = 0; i < 1000; i++) {
		if (readl(base + GSA_MBOX_INTMSR0) & 0x1) {
			ret = 0;
			break;
		}
		udelay(10);
	}

	if (ret) {
		dev_err(dev, "kdn-shim: timeout waiting for GSA response (10ms)\n");
		goto out;
	}

	sr0 = readl(base + GSA_MBOX_SR(0));
	sr1 = readl(base + GSA_MBOX_SR(1));

	dev_dbg(dev,
		 "kdn-shim: GSA response SR0=0x%08x SR1=0x%08x (expect 0x%08x, err=0)\n",
		 sr0, sr1,
		 GSA_MB_CMD_KDN_SET_OP_MODE | GSA_MB_CMD_RSP_BIT);

	if (sr0 != (GSA_MB_CMD_KDN_SET_OP_MODE | GSA_MB_CMD_RSP_BIT))
		ret = -EIO;
	else if (sr1 != 0)
		ret = -EIO;

	/* Clear so the IRQ can fire on the next call. */
	writel(0x1, base + GSA_MBOX_INTCR0);

out:
	iounmap(base);
	return ret;
}

static int gs201_ufs_drv_init(struct exynos_ufs *ufs)
{
	struct device *dev = ufs->hba->dev;
	struct ufs_hba *hba = ufs->hba;
	int ret;

	gs201_dump_cmu_hsi2_ufs_gates(dev);
	gs201_dump_ufsp(dev, "before-smc");
	gs201_dump_sysreg_hsi2_iocc(dev, "before-iocc-write");
	gs201_ufs_smu_init(dev);
	gs201_dump_ufsp(dev, "after-smc ");
	/*
	 * Falsification probe result (2026-05-03): with iocc_val=0 the wedge
	 * fires ~6s earlier (abort at 2.05s vs baseline 8.07s), no
	 * dl_err 0x80000002, but UPIU RSP still stamped 0xab. Conclusion:
	 * sysreg+0x710 is necessary-not-sufficient. KDN_CTRL_MON @ +0x400
	 * read as 0x4 at probe (vs AOSP's MKE=1 programmed state).
	 */
	ret = gs101_ufs_drv_init(ufs);   /* this calls exynos_ufs_shareability */
	gs201_dump_sysreg_hsi2_iocc(dev, "after-iocc-write ");

	/*
	 * Direct write to KDN_CTRL_MON @ +0x400 NOP'd (verified
	 * 2026-05-03 — GSA-owned register). Live AOSP read showed MKE=1
	 * (KDN_CTRL_MON=0x5) vs our mainline KDN_CTRL_MON=0x4. So we issue
	 * the actual mailbox command to the GSA processor at 0x17c90000
	 * via gs201_gsa_kdn_set_op_mode and re-dump KDN_CTRL_MON.
	 */
	{
		int kdn_ret = gs201_gsa_kdn_set_op_mode(dev);
		dev_dbg(dev,
			 "kdn-shim: gs201_gsa_kdn_set_op_mode -> %d\n", kdn_ret);
		gs201_dump_sysreg_hsi2_iocc(dev, "after-kdn-mbox-cmd");
	}

	/*
	 * (g4) Manual DME_HIBER_ENTER/EXIT cycles (driven by clock-gating-on-idle
	 * and runtime PM autosuspend) trigger HOST_BUS_FATAL_ERROR (IS BIT(17),
	 * saved_err=0x20000) ~36s into operation under the (g2) PWM workaround.
	 * The (g3) UFSHCD_QUIRK_BROKEN_AUTO_HIBERN8 quirk only suppresses the
	 * controller's auto-hibern8 timer, not these driver-initiated cycles.
	 * Strip the gating caps inherited from gs101_ufs_drv_init and pin both
	 * PM levels to ACTIVE/ACTIVE-LINK so the link never enters hibern8.
	 * Costs power but avoids the bus-fatal trip; remove once HS-Rate-B
	 * works and the H8-exit path is properly tested.
	 */
	hba->caps &= ~(UFSHCD_CAP_CLK_GATING |
		       UFSHCD_CAP_HIBERN8_WITH_CLK_GATING);
	hba->rpm_lvl = UFS_PM_LVL_0;
	hba->spm_lvl = UFS_PM_LVL_0;
	dev_dbg(dev,
		 "gs201 UFS: stripped clk-gating + hibern8-with-clk-gating; PM lvl pinned to LVL_0\n");

	/*
	 * (h7) Tried several ways to clamp SCSI queue depth to 1 to dodge the
	 * concurrent-command-triggered SBFES at ~38s in PWM mode. NONE took
	 * effect because the vops API doesn't expose a hook at the right
	 * point in the init flow:
	 *   - host->can_queue / cmd_per_lun in drv_init: overwritten at
	 *     ufshcd.c:11061 after vops->init returns.
	 *   - host->can_queue / cmd_per_lun in post_link: too late;
	 *     scsi_add_host (line 11158) already snapshotted the tag-set
	 *     size into the SCSI midlayer.
	 *   - hba->nutrs = 2 in drv_init: overwritten at ufshcd.c:2487 by
	 *     ufshcd_hba_capabilities reading the cap reg (which itself
	 *     runs AFTER vops->init).
	 *   - cmd_per_lun resets again in ufs_get_device_desc (line 8782)
	 *     to (nutrs, bqueuedepth) - reserved, well after probe.
	 * The only remaining clean fix is upstream: either add a vops hook
	 * between caps-read and scsi_add_host, or have ufshcd respect a
	 * per-variant max-can-queue.
	 */
	return ret;
}

static int exynosauto_ufs_drv_init(struct exynos_ufs *ufs)
{
	return exynos_ufs_shareability(ufs);
}

static int exynosauto_ufs_post_hce_enable(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;

	/* Enable Virtual Host #1 */
	ufshcd_rmwl(hba, MHCTRL_EN_VH_MASK, MHCTRL_EN_VH(1), MHCTRL);
	/* Default VH Transfer permissions */
	hci_writel(ufs, ALLOW_TRANS_VH_DEFAULT, HCI_MH_ALLOWABLE_TRAN_OF_VH);
	/* IID information is replaced in TASKTAG[7:5] instead of IID in UCD */
	hci_writel(ufs, 0x1, HCI_MH_IID_IN_TASK_TAG);

	return 0;
}

static int exynosauto_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	int i;
	u32 tx_line_reset_period, rx_line_reset_period;

	rx_line_reset_period = (RX_LINE_RESET_TIME * ufs->mclk_rate) / NSEC_PER_MSEC;
	tx_line_reset_period = (TX_LINE_RESET_TIME * ufs->mclk_rate) / NSEC_PER_MSEC;

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);
	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD_EN, i), 0x0);

		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE2, i),
			       (rx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE1, i),
			       (rx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE0, i),
			       (rx_line_reset_period) & 0xFF);

		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x2f, i), 0x79);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x84, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x25, i), 0xf6);
	}

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		/* Not to affect VND_TX_LINERESET_PVALUE to VND_TX_CLK_PRD */
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD_EN, i),
			       0x02);

		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE2, i),
			       (tx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE1, i),
			       (tx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE0, i),
			       (tx_line_reset_period) & 0xFF);

		/* TX PWM Gear Capability / PWM_G1_ONLY */
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x04, i), 0x1);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE), 0x0);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0xa011), 0x8000);

	return 0;
}

static int exynosauto_ufs_pre_pwr_change(struct exynos_ufs *ufs,
					 struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;

	/* PACP_PWR_req and delivered to the remote DME */
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA0), 12000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA1), 32000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA2), 16000);

	return 0;
}

static int exynosauto_ufs_post_pwr_change(struct exynos_ufs *ufs,
					  const struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;
	u32 enabled_vh;

	enabled_vh = ufshcd_readl(hba, MHCTRL) & MHCTRL_EN_VH_MASK;

	/* Send physical host ready message to virtual hosts */
	ufshcd_writel(hba, MH_MSG(enabled_vh, MH_MSG_PH_READY), PH2VH_MBOX);

	return 0;
}

static int exynos7_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	u32 val = attr->pa_dbg_opt_suite1_val;
	struct ufs_hba *hba = ufs->hba;
	int i;

	exynos_ufs_enable_ov_tm(hba);
	for_each_ufs_tx_lane(ufs, i)
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x297, i), 0x17);
	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x362, i), 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x363, i), 0x00);
	}
	exynos_ufs_disable_ov_tm(hba);

	for_each_ufs_tx_lane(ufs, i)
		ufshcd_dme_set(hba,
			UIC_ARG_MIB_SEL(TX_HIBERN8_CONTROL, i), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_TXPHY_CFGUPDT), 0x1);
	udelay(1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite1_off),
					val | (1 << 12));
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_SKIP_RESET_PHY), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_SKIP_LINE_RESET), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_LINE_RESET_REQ), 0x1);
	udelay(1600);
	ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite1_off), val);

	return 0;
}

static int exynos7_ufs_post_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	int i;

	exynos_ufs_enable_ov_tm(hba);
	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x28b, i), 0x83);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x29a, i), 0x07);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x277, i),
			TX_LINERESET_N(exynos_ufs_calc_time_cntr(ufs, 200000)));
	}
	exynos_ufs_disable_ov_tm(hba);

	exynos_ufs_enable_dbg_mode(hba);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_SAVECONFIGTIME), 0xbb8);
	exynos_ufs_disable_dbg_mode(hba);

	return 0;
}

static int exynos7_ufs_pre_pwr_change(struct exynos_ufs *ufs,
						struct ufs_pa_layer_attr *pwr)
{
	unipro_writel(ufs, 0x22, UNIPRO_DBG_FORCE_DME_CTRL_STATE);

	return 0;
}

static int exynos7_ufs_post_pwr_change(struct exynos_ufs *ufs,
				       const struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;
	int lanes = max_t(u32, pwr->lane_rx, pwr->lane_tx);

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_RXPHY_CFGUPDT), 0x1);

	if (lanes == 1) {
		exynos_ufs_enable_dbg_mode(hba);
		ufshcd_dme_set(hba, UIC_ARG_MIB(PA_CONNECTEDTXDATALANES), 0x1);
		exynos_ufs_disable_dbg_mode(hba);
	}

	return 0;
}

static int exynosautov920_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	int i;
	u32 tx_line_reset_period, rx_line_reset_period;

	rx_line_reset_period = (RX_LINE_RESET_TIME * ufs->mclk_rate)
				/ NSEC_PER_MSEC;
	tx_line_reset_period = (TX_LINE_RESET_TIME * ufs->mclk_rate)
				/ NSEC_PER_MSEC;

	unipro_writel(ufs, 0x5f, 0x44);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x202), 0x02);

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD_EN, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE2, i),
			       (rx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE1, i),
			       (rx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE0, i),
			       (rx_line_reset_period) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x2f, i), 0x69);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x84, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x25, i), 0xf6);
	}

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD_EN, i),
			       0x02);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE2, i),
			       (tx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE1, i),
			       (tx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE0, i),
			       (tx_line_reset_period) & 0xFF);

		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x04, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x7f, i), 0x0);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0xa011), 0x8000);

	return 0;
}

static int exynosautov920_ufs_post_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x9529), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x15a4), 0x3e8);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x9529), 0x0);

	return 0;
}

static int exynosautov920_ufs_pre_pwr_change(struct exynos_ufs *ufs,
					     struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x15d4), 0x1);

	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_FC0PROTTIMEOUTVAL), 8064);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_TC0REPLAYTIMEOUTVAL), 28224);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_AFC0REQTIMEOUTVAL), 20160);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA0), 12000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA1), 32000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA2), 16000);

	unipro_writel(ufs, 8064, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER0);
	unipro_writel(ufs, 28224, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER1);
	unipro_writel(ufs, 20160, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER2);
	unipro_writel(ufs, 12000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER0);
	unipro_writel(ufs, 32000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER1);
	unipro_writel(ufs, 16000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER2);

	return 0;
}

/*
 * exynos_ufs_auto_ctrl_hcc - HCI core clock control by h/w
 * Control should be disabled in the below cases
 * - Before host controller S/W reset
 * - Access to UFS protector's register
 */
static void exynos_ufs_auto_ctrl_hcc(struct exynos_ufs *ufs, bool en)
{
	u32 misc = hci_readl(ufs, HCI_MISC);

	if (en)
		hci_writel(ufs, misc | HCI_CORECLK_CTRL_EN, HCI_MISC);
	else
		hci_writel(ufs, misc & ~HCI_CORECLK_CTRL_EN, HCI_MISC);
}

static void exynos_ufs_ctrl_clkstop(struct exynos_ufs *ufs, bool en)
{
	u32 ctrl = hci_readl(ufs, HCI_CLKSTOP_CTRL);
	u32 misc = hci_readl(ufs, HCI_MISC);

	if (en) {
		hci_writel(ufs, misc | CLK_CTRL_EN_MASK, HCI_MISC);
		hci_writel(ufs, ctrl | CLK_STOP_MASK, HCI_CLKSTOP_CTRL);
	} else {
		hci_writel(ufs, ctrl & ~CLK_STOP_MASK, HCI_CLKSTOP_CTRL);
		hci_writel(ufs, misc & ~CLK_CTRL_EN_MASK, HCI_MISC);
	}
}

static int exynos_ufs_get_clk_info(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct list_head *head = &hba->clk_list_head;
	struct ufs_clk_info *clki;
	unsigned long pclk_rate;
	u32 f_min, f_max;
	u8 div = 0;
	int ret = 0;

	if (list_empty(head))
		goto out;

	list_for_each_entry(clki, head, list) {
		if (!IS_ERR(clki->clk)) {
			if (!strcmp(clki->name, "core_clk"))
				ufs->clk_hci_core = clki->clk;
			else if (!strcmp(clki->name, "sclk_unipro_main"))
				ufs->clk_unipro_main = clki->clk;
		}
	}

	if (!ufs->clk_hci_core || !ufs->clk_unipro_main) {
		dev_err(hba->dev, "failed to get clk info\n");
		ret = -EINVAL;
		goto out;
	}

	ufs->mclk_rate = clk_get_rate(ufs->clk_unipro_main);
	pclk_rate = clk_get_rate(ufs->clk_hci_core);
	f_min = ufs->pclk_avail_min;
	f_max = ufs->pclk_avail_max;

	if (ufs->opts & EXYNOS_UFS_OPT_HAS_APB_CLK_CTRL) {
		do {
			pclk_rate /= (div + 1);

			if (pclk_rate <= f_max)
				break;
			div++;
		} while (pclk_rate >= f_min);
	}

	if (unlikely(pclk_rate < f_min || pclk_rate > f_max)) {
		dev_err(hba->dev, "not available pclk range %lu\n", pclk_rate);
		ret = -EINVAL;
		goto out;
	}

	ufs->pclk_rate = pclk_rate;
	ufs->pclk_div = div;

out:
	return ret;
}

static void exynos_ufs_set_unipro_pclk_div(struct exynos_ufs *ufs)
{
	if (ufs->opts & EXYNOS_UFS_OPT_HAS_APB_CLK_CTRL) {
		u32 val;

		val = hci_readl(ufs, HCI_UNIPRO_APB_CLK_CTRL);
		hci_writel(ufs, UNIPRO_APB_CLK(val, ufs->pclk_div),
			   HCI_UNIPRO_APB_CLK_CTRL);
	}
}

static void exynos_ufs_set_pwm_clk_div(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;

	ufshcd_dme_set(hba,
		UIC_ARG_MIB(CMN_PWM_CLK_CTRL), attr->cmn_pwm_clk_ctrl);
}

static void exynos_ufs_calc_pwm_clk_div(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	const unsigned int div = 30, mult = 20;
	const unsigned long pwm_min = 3 * 1000 * 1000;
	const unsigned long pwm_max = 9 * 1000 * 1000;
	const int divs[] = {32, 16, 8, 4};
	unsigned long clk = 0, _clk, clk_period;
	int i = 0, clk_idx = -1;

	clk_period = UNIPRO_PCLK_PERIOD(ufs);
	for (i = 0; i < ARRAY_SIZE(divs); i++) {
		_clk = NSEC_PER_SEC * mult / (clk_period * divs[i] * div);
		if (_clk >= pwm_min && _clk <= pwm_max) {
			if (_clk > clk) {
				clk_idx = i;
				clk = _clk;
			}
		}
	}

	if (clk_idx == -1) {
		ufshcd_dme_get(hba, UIC_ARG_MIB(CMN_PWM_CLK_CTRL), &clk_idx);
		dev_err(hba->dev,
			"failed to decide pwm clock divider, will not change\n");
	}

	attr->cmn_pwm_clk_ctrl = clk_idx & PWM_CLK_CTRL_MASK;
}

long exynos_ufs_calc_time_cntr(struct exynos_ufs *ufs, long period)
{
	const int precise = 10;
	long pclk_rate = ufs->pclk_rate;
	long clk_period, fraction;

	clk_period = UNIPRO_PCLK_PERIOD(ufs);
	fraction = ((NSEC_PER_SEC % pclk_rate) * precise) / pclk_rate;

	return (period * precise) / ((clk_period * precise) + fraction);
}

static void exynos_ufs_specify_phy_time_attr(struct exynos_ufs *ufs)
{
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	struct ufs_phy_time_cfg *t_cfg = &ufs->t_cfg;

	if (ufs->opts & EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR)
		return;

	t_cfg->tx_linereset_p =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_dif_p_nsec);
	t_cfg->tx_linereset_n =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_dif_n_nsec);
	t_cfg->tx_high_z_cnt =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_high_z_cnt_nsec);
	t_cfg->tx_base_n_val =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_base_unit_nsec);
	t_cfg->tx_gran_n_val =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_gran_unit_nsec);
	t_cfg->tx_sleep_cnt =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_sleep_cnt);

	t_cfg->rx_linereset =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_dif_p_nsec);
	t_cfg->rx_hibern8_wait =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_hibern8_wait_nsec);
	t_cfg->rx_base_n_val =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_base_unit_nsec);
	t_cfg->rx_gran_n_val =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_gran_unit_nsec);
	t_cfg->rx_sleep_cnt =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_sleep_cnt);
	t_cfg->rx_stall_cnt =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_stall_cnt);
}

static void exynos_ufs_config_phy_time_attr(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct ufs_phy_time_cfg *t_cfg = &ufs->t_cfg;
	int i;

	exynos_ufs_set_pwm_clk_div(ufs);

	exynos_ufs_enable_ov_tm(hba);

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_FILLER_ENABLE, i),
				ufs->drv_data->uic_attr->rx_filler_enable);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_LINERESET_VAL, i),
				RX_LINERESET(t_cfg->rx_linereset));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_BASE_NVAL_07_00, i),
				RX_BASE_NVAL_L(t_cfg->rx_base_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_BASE_NVAL_15_08, i),
				RX_BASE_NVAL_H(t_cfg->rx_base_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_GRAN_NVAL_07_00, i),
				RX_GRAN_NVAL_L(t_cfg->rx_gran_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_GRAN_NVAL_10_08, i),
				RX_GRAN_NVAL_H(t_cfg->rx_gran_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_OV_SLEEP_CNT_TIMER, i),
				RX_OV_SLEEP_CNT(t_cfg->rx_sleep_cnt));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_OV_STALL_CNT_TIMER, i),
				RX_OV_STALL_CNT(t_cfg->rx_stall_cnt));
	}

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_LINERESET_P_VAL, i),
				TX_LINERESET_P(t_cfg->tx_linereset_p));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_HIGH_Z_CNT_07_00, i),
				TX_HIGH_Z_CNT_L(t_cfg->tx_high_z_cnt));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_HIGH_Z_CNT_11_08, i),
				TX_HIGH_Z_CNT_H(t_cfg->tx_high_z_cnt));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_BASE_NVAL_07_00, i),
				TX_BASE_NVAL_L(t_cfg->tx_base_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_BASE_NVAL_15_08, i),
				TX_BASE_NVAL_H(t_cfg->tx_base_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_GRAN_NVAL_07_00, i),
				TX_GRAN_NVAL_L(t_cfg->tx_gran_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_GRAN_NVAL_10_08, i),
				TX_GRAN_NVAL_H(t_cfg->tx_gran_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_OV_SLEEP_CNT_TIMER, i),
				TX_OV_H8_ENTER_EN |
				TX_OV_SLEEP_CNT(t_cfg->tx_sleep_cnt));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_MIN_ACTIVATETIME, i),
				ufs->drv_data->uic_attr->tx_min_activatetime);
	}

	exynos_ufs_disable_ov_tm(hba);
}

static void exynos_ufs_config_phy_cap_attr(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	int i;

	exynos_ufs_enable_ov_tm(hba);

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G1_SYNC_LENGTH_CAP, i),
				attr->rx_hs_g1_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G2_SYNC_LENGTH_CAP, i),
				attr->rx_hs_g2_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G3_SYNC_LENGTH_CAP, i),
				attr->rx_hs_g3_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G1_PREP_LENGTH_CAP, i),
				attr->rx_hs_g1_prep_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G2_PREP_LENGTH_CAP, i),
				attr->rx_hs_g2_prep_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G3_PREP_LENGTH_CAP, i),
				attr->rx_hs_g3_prep_sync_len_cap);
	}

	if (attr->rx_adv_fine_gran_sup_en == 0) {
		for_each_ufs_rx_lane(ufs, i) {
			ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_ADV_GRANULARITY_CAP, i), 0);

			if (attr->rx_min_actv_time_cap)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(
					RX_MIN_ACTIVATETIME_CAPABILITY, i),
					attr->rx_min_actv_time_cap);

			if (attr->rx_hibern8_time_cap)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(RX_HIBERN8TIME_CAP, i),
						attr->rx_hibern8_time_cap);
		}
	} else if (attr->rx_adv_fine_gran_sup_en == 1) {
		for_each_ufs_rx_lane(ufs, i) {
			if (attr->rx_adv_fine_gran_step)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(RX_ADV_GRANULARITY_CAP,
						i), RX_ADV_FINE_GRAN_STEP(
						attr->rx_adv_fine_gran_step));

			if (attr->rx_adv_min_actv_time_cap)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(
						RX_ADV_MIN_ACTIVATETIME_CAP, i),
						attr->rx_adv_min_actv_time_cap);

			if (attr->rx_adv_hibern8_time_cap)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(RX_ADV_HIBERN8TIME_CAP,
						i),
						attr->rx_adv_hibern8_time_cap);
		}
	}

	exynos_ufs_disable_ov_tm(hba);
}

static void exynos_ufs_establish_connt(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	enum {
		DEV_ID		= 0x00,
		PEER_DEV_ID	= 0x01,
		PEER_CPORT_ID	= 0x00,
		TRAFFIC_CLASS	= 0x00,
	};

	/* allow cport attributes to be set */
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_IDLE);

	/* local unipro attributes */
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID), DEV_ID);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID_VALID), true);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERDEVICEID), PEER_DEV_ID);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERCPORTID), PEER_CPORT_ID);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CPORTFLAGS), CPORT_DEF_FLAGS);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_TRAFFICCLASS), TRAFFIC_CLASS);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_CONNECTED);
}

static void exynos_ufs_config_smu(struct exynos_ufs *ufs)
{
	u32 reg, val;

	if (ufs->opts & EXYNOS_UFS_OPT_UFSPR_SECURE)
		return;

	exynos_ufs_disable_auto_ctrl_hcc_save(ufs, &val);

	/* make encryption disabled by default */
	reg = ufsp_readl(ufs, UFSPRSECURITY);
	ufsp_writel(ufs, reg | NSSMU, UFSPRSECURITY);
	ufsp_writel(ufs, 0x0, UFSPSBEGIN0);
	ufsp_writel(ufs, 0xffffffff, UFSPSEND0);
	ufsp_writel(ufs, 0xff, UFSPSLUN0);
	ufsp_writel(ufs, 0xf1, UFSPSCTRL0);

	exynos_ufs_auto_ctrl_hcc_restore(ufs, &val);
}

static void exynos_ufs_config_sync_pattern_mask(struct exynos_ufs *ufs,
					struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;
	u8 g = max_t(u32, pwr->gear_rx, pwr->gear_tx);
	u32 mask, sync_len;
	enum {
		SYNC_LEN_G1 = 80 * 1000, /* 80us */
		SYNC_LEN_G2 = 40 * 1000, /* 40us */
		SYNC_LEN_G3 = 20 * 1000, /* 20us */
	};
	int i;

	if (g == 1)
		sync_len = SYNC_LEN_G1;
	else if (g == 2)
		sync_len = SYNC_LEN_G2;
	else if (g == 3)
		sync_len = SYNC_LEN_G3;
	else
		return;

	mask = exynos_ufs_calc_time_cntr(ufs, sync_len);
	mask = (mask >> 8) & 0xff;

	exynos_ufs_enable_ov_tm(hba);

	for_each_ufs_rx_lane(ufs, i)
		ufshcd_dme_set(hba,
			UIC_ARG_MIB_SEL(RX_SYNC_MASK_LENGTH, i), mask);

	exynos_ufs_disable_ov_tm(hba);
}

#define UFS_HW_VER_MAJOR_MASK   GENMASK(15, 8)

static u32 exynos_ufs_get_hs_gear(struct ufs_hba *hba)
{
	u8 major;

	major = FIELD_GET(UFS_HW_VER_MAJOR_MASK, hba->ufs_version);

	if (major >= 3)
		return UFS_HS_G4;

	/* Default is HS-G3 */
	return UFS_HS_G3;
}

static int exynos_ufs_pre_pwr_mode(struct ufs_hba *hba,
				struct ufs_pa_layer_attr *dev_req_params)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	struct phy *generic_phy = ufs->phy;
	int ret;

	if (!dev_req_params) {
		pr_err("%s: incoming dev_req_params is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (ufs->drv_data->pre_pwr_change)
		ufs->drv_data->pre_pwr_change(ufs, dev_req_params);

	if (ufshcd_is_hs_mode(dev_req_params)) {
		exynos_ufs_config_sync_pattern_mask(ufs, dev_req_params);

		switch (dev_req_params->hs_rate) {
		case PA_HS_MODE_A:
		case PA_HS_MODE_B:
			phy_calibrate(generic_phy);
			break;
		}
	}

	/* setting for three timeout values for traffic class #0 */
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_FC0PROTTIMEOUTVAL), 8064);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_TC0REPLAYTIMEOUTVAL), 28224);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_AFC0REQTIMEOUTVAL), 20160);

	return 0;
out:
	return ret;
}

#define PWR_MODE_STR_LEN	64
static int exynos_ufs_post_pwr_mode(struct ufs_hba *hba,
				const struct ufs_pa_layer_attr *pwr_req)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	struct phy *generic_phy = ufs->phy;
	int gear = max_t(u32, pwr_req->gear_rx, pwr_req->gear_tx);
	int lanes = max_t(u32, pwr_req->lane_rx, pwr_req->lane_tx);
	char pwr_str[PWR_MODE_STR_LEN] = "";

	/* let default be PWM Gear 1, Lane 1 */
	if (!gear)
		gear = 1;

	if (!lanes)
		lanes = 1;

	if (ufs->drv_data->post_pwr_change)
		ufs->drv_data->post_pwr_change(ufs, pwr_req);

	if ((ufshcd_is_hs_mode(pwr_req))) {
		switch (pwr_req->hs_rate) {
		case PA_HS_MODE_A:
		case PA_HS_MODE_B:
			phy_calibrate(generic_phy);
			break;
		}

		snprintf(pwr_str, PWR_MODE_STR_LEN, "%s series_%s G_%d L_%d",
			"FAST",	pwr_req->hs_rate == PA_HS_MODE_A ? "A" : "B",
			gear, lanes);
	} else {
		snprintf(pwr_str, PWR_MODE_STR_LEN, "%s G_%d L_%d",
			"SLOW", gear, lanes);
	}

	dev_dbg(hba->dev, "Power mode changed to : %s\n", pwr_str);

	return 0;
}

static void exynos_ufs_specify_nexus_t_xfer_req(struct ufs_hba *hba,
						int tag, bool is_scsi_cmd)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	u32 type;

	type =  hci_readl(ufs, HCI_UTRL_NEXUS_TYPE);

	if (is_scsi_cmd)
		hci_writel(ufs, type | (1 << tag), HCI_UTRL_NEXUS_TYPE);
	else
		hci_writel(ufs, type & ~(1 << tag), HCI_UTRL_NEXUS_TYPE);
}

static void exynos_ufs_specify_nexus_t_tm_req(struct ufs_hba *hba,
						int tag, u8 func)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	u32 type;

	type =  hci_readl(ufs, HCI_UTMRL_NEXUS_TYPE);

	switch (func) {
	case UFS_ABORT_TASK:
	case UFS_QUERY_TASK:
		hci_writel(ufs, type | (1 << tag), HCI_UTMRL_NEXUS_TYPE);
		break;
	case UFS_ABORT_TASK_SET:
	case UFS_CLEAR_TASK_SET:
	case UFS_LOGICAL_RESET:
	case UFS_QUERY_TASK_SET:
		hci_writel(ufs, type & ~(1 << tag), HCI_UTMRL_NEXUS_TYPE);
		break;
	}
}

static int exynos_ufs_phy_init(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct phy *generic_phy = ufs->phy;
	int ret = 0;

	if (ufs->avail_ln_rx == 0 || ufs->avail_ln_tx == 0) {
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILRXDATALANES),
			&ufs->avail_ln_rx);
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILTXDATALANES),
			&ufs->avail_ln_tx);
		WARN(ufs->avail_ln_rx != ufs->avail_ln_tx,
			"available data lane is not equal(rx:%d, tx:%d)\n",
			ufs->avail_ln_rx, ufs->avail_ln_tx);
	}

	phy_set_bus_width(generic_phy, ufs->avail_ln_rx);

	if (generic_phy->power_count) {
		phy_power_off(generic_phy);
		phy_exit(generic_phy);
	}

	ret = phy_init(generic_phy);
	if (ret) {
		dev_err(hba->dev, "%s: phy init failed, ret = %d\n",
			__func__, ret);
		return ret;
	}

	ret = phy_power_on(generic_phy);
	if (ret)
		goto out_exit_phy;

	return 0;

out_exit_phy:
	phy_exit(generic_phy);

	return ret;
}

static void exynos_ufs_config_unipro(struct exynos_ufs *ufs)
{
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	struct ufs_hba *hba = ufs->hba;

	if (attr->pa_dbg_clk_period_off)
		ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_clk_period_off),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXTRAILINGCLOCKS),
			ufs->drv_data->uic_attr->tx_trailingclks);

	if (attr->pa_dbg_opt_suite1_off)
		ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite1_off),
			       attr->pa_dbg_opt_suite1_val);

	if (attr->pa_dbg_opt_suite2_off)
		ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite2_off),
			       attr->pa_dbg_opt_suite2_val);
}

static void exynos_ufs_config_intr(struct exynos_ufs *ufs, u32 errs, u8 index)
{
	switch (index) {
	case UNIPRO_L1_5:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_PA_LAYER);
		break;
	case UNIPRO_L2:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_DL_LAYER);
		break;
	case UNIPRO_L3:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_N_LAYER);
		break;
	case UNIPRO_L4:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_T_LAYER);
		break;
	case UNIPRO_DME:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_DME_LAYER);
		break;
	}
}

static int exynos_ufs_setup_clocks(struct ufs_hba *hba, bool on,
				   enum ufs_notify_change_status status)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	if (!ufs)
		return 0;

	if (on && status == PRE_CHANGE) {
		if (ufs->opts & EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL)
			exynos_ufs_disable_auto_ctrl_hcc(ufs);
		exynos_ufs_ungate_clks(ufs);
	} else if (!on && status == POST_CHANGE) {
		exynos_ufs_gate_clks(ufs);
		if (ufs->opts & EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL)
			exynos_ufs_enable_auto_ctrl_hcc(ufs);
	}

	return 0;
}

static int exynos_ufs_pre_link(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	/* hci */
	exynos_ufs_config_intr(ufs, DFES_DEF_L2_ERRS, UNIPRO_L2);
	exynos_ufs_config_intr(ufs, DFES_DEF_L3_ERRS, UNIPRO_L3);
	exynos_ufs_config_intr(ufs, DFES_DEF_L4_ERRS, UNIPRO_L4);
	exynos_ufs_set_unipro_pclk_div(ufs);

	exynos_ufs_setup_clocks(hba, true, PRE_CHANGE);

	/* unipro */
	exynos_ufs_config_unipro(ufs);

	if (ufs->drv_data->pre_link)
		ufs->drv_data->pre_link(ufs);

	/* m-phy */
	exynos_ufs_phy_init(ufs);
	if (!(ufs->opts & EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR)) {
		exynos_ufs_config_phy_time_attr(ufs);
		exynos_ufs_config_phy_cap_attr(ufs);
	}

	return 0;
}

static void exynos_ufs_fit_aggr_timeout(struct exynos_ufs *ufs)
{
	u32 val;

	/* Select function clock (mclk) for timer tick */
	if (ufs->opts & EXYNOS_UFS_OPT_TIMER_TICK_SELECT) {
		val = hci_readl(ufs, HCI_V2P1_CTRL);
		val |= IA_TICK_SEL;
		hci_writel(ufs, val, HCI_V2P1_CTRL);
	}

	val = exynos_ufs_calc_time_cntr(ufs, IATOVAL_NSEC / CNTR_DIV_VAL);
	hci_writel(ufs, val & CNT_VAL_1US_MASK, HCI_1US_TO_CNT_VAL);
}

static int exynos_ufs_post_link(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	struct phy *generic_phy = ufs->phy;
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	u32 val = ilog2(DATA_UNIT_SIZE);

	exynos_ufs_establish_connt(ufs);
	exynos_ufs_fit_aggr_timeout(ufs);

	hci_writel(ufs, 0xa, HCI_DATA_REORDER);

	if (hba->caps & UFSHCD_CAP_CRYPTO)
		val |= PRDT_PREFETCH_EN;
	hci_writel(ufs, val, HCI_TXPRDT_ENTRY_SIZE);

	hci_writel(ufs, ilog2(DATA_UNIT_SIZE), HCI_RXPRDT_ENTRY_SIZE);
	hci_writel(ufs, BIT(hba->nutrs) - 1, HCI_UTRL_NEXUS_TYPE);
	hci_writel(ufs, BIT(hba->nutmrs) - 1, HCI_UTMRL_NEXUS_TYPE);
	hci_writel(ufs, 0xf, HCI_AXIDMA_RWDATA_BURST_LEN);

	if (ufs->opts & EXYNOS_UFS_OPT_SKIP_CONNECTION_ESTAB)
		ufshcd_dme_set(hba,
			UIC_ARG_MIB(T_DBG_SKIP_INIT_HIBERN8_EXIT), true);

	if (attr->pa_granularity) {
		exynos_ufs_enable_dbg_mode(hba);
		ufshcd_dme_set(hba, UIC_ARG_MIB(PA_GRANULARITY),
				attr->pa_granularity);
		exynos_ufs_disable_dbg_mode(hba);

		if (attr->pa_tactivate)
			ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TACTIVATE),
					attr->pa_tactivate);
		if (attr->pa_hibern8time &&
		    !(ufs->opts & EXYNOS_UFS_OPT_USE_SW_HIBERN8_TIMER))
			ufshcd_dme_set(hba, UIC_ARG_MIB(PA_HIBERN8TIME),
					attr->pa_hibern8time);
	}

	if (ufs->opts & EXYNOS_UFS_OPT_USE_SW_HIBERN8_TIMER) {
		if (!attr->pa_granularity)
			ufshcd_dme_get(hba, UIC_ARG_MIB(PA_GRANULARITY),
					&attr->pa_granularity);
		if (!attr->pa_hibern8time)
			ufshcd_dme_get(hba, UIC_ARG_MIB(PA_HIBERN8TIME),
					&attr->pa_hibern8time);
		/*
		 * not wait for HIBERN8 time to exit hibernation
		 */
		ufshcd_dme_set(hba, UIC_ARG_MIB(PA_HIBERN8TIME), 0);

		if (attr->pa_granularity < 1 || attr->pa_granularity > 6) {
			/* Valid range for granularity: 1 ~ 6 */
			dev_warn(hba->dev,
				"%s: pa_granularity %d is invalid, assuming backwards compatibility\n",
				__func__,
				attr->pa_granularity);
			attr->pa_granularity = 6;
		}
	}

	phy_calibrate(generic_phy);

	if (ufs->drv_data->post_link)
		ufs->drv_data->post_link(ufs);

	return 0;
}

static int exynos_ufs_parse_dt(struct device *dev, struct exynos_ufs *ufs)
{
	struct device_node *np = dev->of_node;
	struct exynos_ufs_uic_attr *attr;
	int ret = 0;

	ufs->drv_data = device_get_match_data(dev);

	if (ufs->drv_data && ufs->drv_data->uic_attr) {
		attr = ufs->drv_data->uic_attr;
	} else {
		dev_err(dev, "failed to get uic attributes\n");
		ret = -EINVAL;
		goto out;
	}

	ufs->sysreg = syscon_regmap_lookup_by_phandle(np, "samsung,sysreg");
	if (IS_ERR(ufs->sysreg))
		ufs->sysreg = NULL;
	else {
		if (of_property_read_u32_index(np, "samsung,sysreg", 1,
					       &ufs->iocc_offset)) {
			dev_warn(dev, "can't get an offset from sysreg. Set to default value\n");
			ufs->iocc_offset = UFS_SHAREABILITY_OFFSET;
		}
	}

	ufs->iocc_mask = ufs->drv_data->iocc_mask;
	/*
	 * no 'dma-coherent' property means the descriptors are
	 * non-cacheable so iocc shareability should be disabled.
	 */
	if (of_dma_is_coherent(dev->of_node))
		ufs->iocc_val = ufs->iocc_mask;
	else
		ufs->iocc_val = 0;

	ufs->pclk_avail_min = PCLK_AVAIL_MIN;
	ufs->pclk_avail_max = PCLK_AVAIL_MAX;

	attr->rx_adv_fine_gran_sup_en = RX_ADV_FINE_GRAN_SUP_EN;
	attr->rx_adv_fine_gran_step = RX_ADV_FINE_GRAN_STEP_VAL;
	attr->rx_adv_min_actv_time_cap = RX_ADV_MIN_ACTV_TIME_CAP;
	attr->pa_granularity = PA_GRANULARITY_VAL;
	attr->pa_tactivate = PA_TACTIVATE_VAL;
	attr->pa_hibern8time = PA_HIBERN8TIME_VAL;

out:
	return ret;
}

static inline void exynos_ufs_priv_init(struct ufs_hba *hba,
					struct exynos_ufs *ufs)
{
	ufs->hba = hba;
	ufs->opts = ufs->drv_data->opts;
	ufs->rx_sel_idx = PA_MAXDATALANES;
	if (ufs->opts & EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX)
		ufs->rx_sel_idx = 0;
	hba->priv = (void *)ufs;
	hba->quirks = ufs->drv_data->quirks;
}

#ifdef CONFIG_SCSI_UFS_CRYPTO

/*
 * Support for Flash Memory Protector (FMP), which is the inline encryption
 * hardware on Exynos and Exynos-based SoCs.  The interface to this hardware is
 * not compatible with the standard UFS crypto.  It requires that encryption be
 * configured in the PRDT using a nonstandard extension.
 */

enum fmp_crypto_algo_mode {
	FMP_BYPASS_MODE = 0,
	FMP_ALGO_MODE_AES_CBC = 1,
	FMP_ALGO_MODE_AES_XTS = 2,
};
enum fmp_crypto_key_length {
	FMP_KEYLEN_256BIT = 1,
};

/**
 * struct fmp_sg_entry - nonstandard format of PRDT entries when FMP is enabled
 *
 * @base: The standard PRDT entry, but with nonstandard bitfields in the high
 *	bits of the 'size' field, i.e. the last 32-bit word.  When these
 *	nonstandard bitfields are zero, the data segment won't be encrypted or
 *	decrypted.  Otherwise they specify the algorithm and key length with
 *	which the data segment will be encrypted or decrypted.
 * @file_iv: The initialization vector (IV) with all bytes reversed
 * @file_enckey: The first half of the AES-XTS key with all bytes reserved
 * @file_twkey: The second half of the AES-XTS key with all bytes reserved
 * @disk_iv: Unused
 * @reserved: Unused
 */
struct fmp_sg_entry {
	struct ufshcd_sg_entry base;
	__be64 file_iv[2];
	__be64 file_enckey[4];
	__be64 file_twkey[4];
	__be64 disk_iv[2];
	__be64 reserved[2];
};

#define SMC_CMD_FMP_SECURITY	\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
			   ARM_SMCCC_OWNER_SIP, 0x1810)
#define SMC_CMD_SMU		\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
			   ARM_SMCCC_OWNER_SIP, 0x1850)
#define SMC_CMD_FMP_SMU_RESUME	\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
			   ARM_SMCCC_OWNER_SIP, 0x1860)
#define SMU_EMBEDDED			0
#define SMU_INIT			0
#define CFG_DESCTYPE_3			3

static void exynos_ufs_fmp_init(struct ufs_hba *hba, struct exynos_ufs *ufs)
{
	struct blk_crypto_profile *profile = &hba->crypto_profile;
	struct arm_smccc_res res;
	int err;

	/*
	 * Check for the standard crypto support bit, since it's available even
	 * though the rest of the interface to FMP is nonstandard.
	 *
	 * This check should have the effect of preventing the driver from
	 * trying to use FMP on old Exynos SoCs that don't have FMP.
	 */
	if (!(ufshcd_readl(hba, REG_CONTROLLER_CAPABILITIES) &
	      MASK_CRYPTO_SUPPORT))
		return;

	/*
	 * The below sequence of SMC calls to enable FMP can be found in the
	 * downstream driver source for gs101 and other Exynos-based SoCs.  It
	 * is the only way to enable FMP that works on SoCs such as gs101 that
	 * don't make the FMP registers accessible to Linux.  It probably works
	 * on other Exynos-based SoCs too, and might even still be the only way
	 * that works.  But this hasn't been properly tested, and this code is
	 * mutually exclusive with exynos_ufs_config_smu().  So for now only
	 * enable FMP support on SoCs with EXYNOS_UFS_OPT_UFSPR_SECURE.
	 */
	if (!(ufs->opts & EXYNOS_UFS_OPT_UFSPR_SECURE))
		return;

	/*
	 * This call (which sets DESCTYPE to 0x3 in the FMPSECURITY0 register)
	 * is needed to make the hardware use the larger PRDT entry size.
	 */
	BUILD_BUG_ON(sizeof(struct fmp_sg_entry) != 128);
	arm_smccc_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, CFG_DESCTYPE_3,
		      0, 0, 0, 0, &res);
	if (res.a0) {
		dev_warn(hba->dev,
			 "SMC_CMD_FMP_SECURITY failed on init: %ld.  Disabling FMP support.\n",
			 res.a0);
		return;
	}
	ufshcd_set_sg_entry_size(hba, sizeof(struct fmp_sg_entry));

	/*
	 * This is needed to initialize FMP.  Without it, errors occur when
	 * inline encryption is used.
	 */
	arm_smccc_smc(SMC_CMD_SMU, SMU_INIT, SMU_EMBEDDED, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(hba->dev,
			"SMC_CMD_SMU(SMU_INIT) failed: %ld.  Disabling FMP support.\n",
			res.a0);
		return;
	}

	/* Advertise crypto capabilities to the block layer. */
	err = devm_blk_crypto_profile_init(hba->dev, profile, 0);
	if (err) {
		/* Only ENOMEM should be possible here. */
		dev_err(hba->dev, "Failed to initialize crypto profile: %d\n",
			err);
		return;
	}
	profile->max_dun_bytes_supported = AES_BLOCK_SIZE;
	profile->key_types_supported = BLK_CRYPTO_KEY_TYPE_RAW;
	profile->dev = hba->dev;
	profile->modes_supported[BLK_ENCRYPTION_MODE_AES_256_XTS] =
		DATA_UNIT_SIZE;

	/* Advertise crypto support to ufshcd-core. */
	hba->caps |= UFSHCD_CAP_CRYPTO;

	/* Advertise crypto quirks to ufshcd-core. */
	hba->quirks |= UFSHCD_QUIRK_CUSTOM_CRYPTO_PROFILE |
		       UFSHCD_QUIRK_BROKEN_CRYPTO_ENABLE |
		       UFSHCD_QUIRK_KEYS_IN_PRDT;

}

static void exynos_ufs_fmp_resume(struct ufs_hba *hba)
{
	struct arm_smccc_res res;

	if (!(hba->caps & UFSHCD_CAP_CRYPTO))
		return;

	arm_smccc_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, CFG_DESCTYPE_3,
		      0, 0, 0, 0, &res);
	if (res.a0)
		dev_err(hba->dev,
			"SMC_CMD_FMP_SECURITY failed on resume: %ld\n", res.a0);

	arm_smccc_smc(SMC_CMD_FMP_SMU_RESUME, 0, SMU_EMBEDDED, 0, 0, 0, 0, 0,
		      &res);
	if (res.a0)
		dev_err(hba->dev,
			"SMC_CMD_FMP_SMU_RESUME failed: %ld\n", res.a0);
}

static inline __be64 fmp_key_word(const u8 *key, int j)
{
	return cpu_to_be64(get_unaligned_le64(
			key + AES_KEYSIZE_256 - (j + 1) * sizeof(u64)));
}

/* Fill the PRDT for a request according to the given encryption context. */
static int exynos_ufs_fmp_fill_prdt(struct ufs_hba *hba,
				    const struct bio_crypt_ctx *crypt_ctx,
				    void *prdt, unsigned int num_segments)
{
	struct fmp_sg_entry *fmp_prdt = prdt;
	const u8 *enckey = crypt_ctx->bc_key->bytes;
	const u8 *twkey = enckey + AES_KEYSIZE_256;
	u64 dun_lo = crypt_ctx->bc_dun[0];
	u64 dun_hi = crypt_ctx->bc_dun[1];
	unsigned int i;

	/* If FMP wasn't enabled, we shouldn't get any encrypted requests. */
	if (WARN_ON_ONCE(!(hba->caps & UFSHCD_CAP_CRYPTO)))
		return -EIO;

	/* Configure FMP on each segment of the request. */
	for (i = 0; i < num_segments; i++) {
		struct fmp_sg_entry *prd = &fmp_prdt[i];
		int j;

		/* Each segment must be exactly one data unit. */
		if (prd->base.size != cpu_to_le32(DATA_UNIT_SIZE - 1)) {
			dev_err(hba->dev,
				"data segment is misaligned for FMP\n");
			return -EIO;
		}

		/* Set the algorithm and key length. */
		prd->base.size |= cpu_to_le32((FMP_ALGO_MODE_AES_XTS << 28) |
					      (FMP_KEYLEN_256BIT << 26));

		/* Set the IV. */
		prd->file_iv[0] = cpu_to_be64(dun_hi);
		prd->file_iv[1] = cpu_to_be64(dun_lo);

		/* Set the key. */
		for (j = 0; j < AES_KEYSIZE_256 / sizeof(u64); j++) {
			prd->file_enckey[j] = fmp_key_word(enckey, j);
			prd->file_twkey[j] = fmp_key_word(twkey, j);
		}

		/* Increment the data unit number. */
		dun_lo++;
		if (dun_lo == 0)
			dun_hi++;
	}
	return 0;
}

#else /* CONFIG_SCSI_UFS_CRYPTO */

static void exynos_ufs_fmp_init(struct ufs_hba *hba, struct exynos_ufs *ufs)
{
}

static void exynos_ufs_fmp_resume(struct ufs_hba *hba)
{
}

#define exynos_ufs_fmp_fill_prdt NULL

#endif /* !CONFIG_SCSI_UFS_CRYPTO */

static int exynos_ufs_init(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos_ufs *ufs;
	int ret;

	ufs = devm_kzalloc(dev, sizeof(*ufs), GFP_KERNEL);
	if (!ufs)
		return -ENOMEM;

	/* exynos-specific hci */
	ufs->reg_hci = devm_platform_ioremap_resource_byname(pdev, "vs_hci");
	if (IS_ERR(ufs->reg_hci)) {
		dev_err(dev, "cannot ioremap for hci vendor register\n");
		return PTR_ERR(ufs->reg_hci);
	}

	/* unipro */
	ufs->reg_unipro = devm_platform_ioremap_resource_byname(pdev, "unipro");
	if (IS_ERR(ufs->reg_unipro)) {
		dev_err(dev, "cannot ioremap for unipro register\n");
		return PTR_ERR(ufs->reg_unipro);
	}

	/* ufs protector */
	ufs->reg_ufsp = devm_platform_ioremap_resource_byname(pdev, "ufsp");
	if (IS_ERR(ufs->reg_ufsp)) {
		dev_err(dev, "cannot ioremap for ufs protector register\n");
		return PTR_ERR(ufs->reg_ufsp);
	}

	ret = exynos_ufs_parse_dt(dev, ufs);
	if (ret) {
		dev_err(dev, "failed to get dt info.\n");
		goto out;
	}

	ufs->phy = devm_phy_get(dev, "ufs-phy");
	if (IS_ERR(ufs->phy)) {
		ret = PTR_ERR(ufs->phy);
		dev_err(dev, "failed to get ufs-phy\n");
		goto out;
	}

	exynos_ufs_priv_init(hba, ufs);

	exynos_ufs_fmp_init(hba, ufs);

	if (ufs->drv_data->drv_init) {
		ret = ufs->drv_data->drv_init(ufs);
		if (ret) {
			dev_err(dev, "failed to init drv-data\n");
			goto out;
		}
	}

	ret = exynos_ufs_get_clk_info(ufs);
	if (ret)
		goto out;
	exynos_ufs_specify_phy_time_attr(ufs);

	exynos_ufs_config_smu(ufs);

	hba->host->dma_alignment = DATA_UNIT_SIZE - 1;
	return 0;

out:
	hba->priv = NULL;
	return ret;
}

static void exynos_ufs_exit(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	phy_power_off(ufs->phy);
	phy_exit(ufs->phy);
}

static int exynos_ufs_host_reset(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	unsigned long timeout = jiffies + msecs_to_jiffies(1);
	u32 val;
	int ret = 0;

	exynos_ufs_disable_auto_ctrl_hcc_save(ufs, &val);

	hci_writel(ufs, UFS_SW_RST_MASK, HCI_SW_RST);

	do {
		if (!(hci_readl(ufs, HCI_SW_RST) & UFS_SW_RST_MASK))
			goto out;
	} while (time_before(jiffies, timeout));

	dev_err(hba->dev, "timeout host sw-reset\n");
	ret = -ETIMEDOUT;

out:
	exynos_ufs_auto_ctrl_hcc_restore(ufs, &val);
	return ret;
}

static void exynos_ufs_dev_hw_reset(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	hci_writel(ufs, 0 << 0, HCI_GPIO_OUT);
	udelay(5);
	hci_writel(ufs, 1 << 0, HCI_GPIO_OUT);
}

static void exynos_ufs_pre_hibern8(struct ufs_hba *hba, enum uic_cmd_dme cmd)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	static const union phy_notify phystate = {
		.ufs_state = PHY_UFS_HIBERN8_EXIT
	};

	if (cmd == UIC_CMD_DME_HIBER_EXIT) {
		if (ufs->opts & EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL)
			exynos_ufs_disable_auto_ctrl_hcc(ufs);
		exynos_ufs_ungate_clks(ufs);

		phy_notify_state(ufs->phy, phystate);

		if (ufs->opts & EXYNOS_UFS_OPT_USE_SW_HIBERN8_TIMER) {
			static const unsigned int granularity_tbl[] = {
				1, 4, 8, 16, 32, 100
			};
			int h8_time = attr->pa_hibern8time *
				granularity_tbl[attr->pa_granularity - 1];
			unsigned long us;
			s64 delta;

			do {
				delta = h8_time - ktime_us_delta(ktime_get(),
							ufs->entry_hibern8_t);
				if (delta <= 0)
					break;

				us = min_t(s64, delta, USEC_PER_MSEC);
				if (us >= 10)
					usleep_range(us, us + 10);
			} while (1);
		}
	}
}

static void exynos_ufs_post_hibern8(struct ufs_hba *hba, enum uic_cmd_dme cmd)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	static const union phy_notify phystate = {
		.ufs_state = PHY_UFS_HIBERN8_ENTER
	};

	if (cmd == UIC_CMD_DME_HIBER_ENTER) {
		ufs->entry_hibern8_t = ktime_get();
		exynos_ufs_gate_clks(ufs);
		if (ufs->opts & EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL)
			exynos_ufs_enable_auto_ctrl_hcc(ufs);

		phy_notify_state(ufs->phy, phystate);
	}
}

static int exynos_ufs_hce_enable_notify(struct ufs_hba *hba,
					enum ufs_notify_change_status status)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	int ret = 0;

	switch (status) {
	case PRE_CHANGE:
		/*
		 * The maximum segment size must be set after scsi_host_alloc()
		 * has been called and before LUN scanning starts
		 * (ufshcd_async_scan()). Note: this callback may also be called
		 * from other functions than ufshcd_init().
		 */
		hba->host->max_segment_size = DATA_UNIT_SIZE;

		if (ufs->drv_data->pre_hce_enable) {
			ret = ufs->drv_data->pre_hce_enable(ufs);
			if (ret)
				return ret;
		}

		ret = exynos_ufs_host_reset(hba);
		if (ret)
			return ret;
		exynos_ufs_dev_hw_reset(hba);
		break;
	case POST_CHANGE:
		exynos_ufs_calc_pwm_clk_div(ufs);
		if (!(ufs->opts & EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL))
			exynos_ufs_enable_auto_ctrl_hcc(ufs);

		if (ufs->drv_data->post_hce_enable)
			ret = ufs->drv_data->post_hce_enable(ufs);

		break;
	}

	return ret;
}

static int exynos_ufs_link_startup_notify(struct ufs_hba *hba,
					  enum ufs_notify_change_status status)
{
	int ret = 0;

	switch (status) {
	case PRE_CHANGE:
		ret = exynos_ufs_pre_link(hba);
		break;
	case POST_CHANGE:
		ret = exynos_ufs_post_link(hba);
		break;
	}

	return ret;
}

static int exynos_ufs_negotiate_pwr_mode(struct ufs_hba *hba,
					 const struct ufs_pa_layer_attr *dev_max_params,
					 struct ufs_pa_layer_attr *dev_req_params)
{
	struct ufs_host_params host_params;

	ufshcd_init_host_params(&host_params);

	/* This driver only support symmetric gear setting e.g. hs_tx_gear == hs_rx_gear */
	host_params.hs_tx_gear = exynos_ufs_get_hs_gear(hba);
	host_params.hs_rx_gear = exynos_ufs_get_hs_gear(hba);

	return ufshcd_negotiate_pwr_params(&host_params, dev_max_params, dev_req_params);
}

static int exynos_ufs_pwr_change_notify(struct ufs_hba *hba,
				enum ufs_notify_change_status status,
				struct ufs_pa_layer_attr *dev_req_params)
{
	int ret = 0;

	switch (status) {
	case PRE_CHANGE:
		ret = exynos_ufs_pre_pwr_mode(hba, dev_req_params);
		break;
	case POST_CHANGE:
		ret = exynos_ufs_post_pwr_mode(hba, dev_req_params);
		break;
	}

	return ret;
}

static void exynos_ufs_hibern8_notify(struct ufs_hba *hba,
				     enum uic_cmd_dme cmd,
				     enum ufs_notify_change_status notify)
{
	switch ((u8)notify) {
	case PRE_CHANGE:
		exynos_ufs_pre_hibern8(hba, cmd);
		break;
	case POST_CHANGE:
		exynos_ufs_post_hibern8(hba, cmd);
		break;
	}
}

static int gs101_ufs_suspend(struct exynos_ufs *ufs)
{
	hci_writel(ufs, 0 << 0, HCI_GPIO_OUT);
	return 0;
}

static int exynos_ufs_suspend(struct ufs_hba *hba, enum ufs_pm_op pm_op,
	enum ufs_notify_change_status status)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	if (status == PRE_CHANGE)
		return 0;

	if (ufs->drv_data->suspend)
		ufs->drv_data->suspend(ufs);

	if (!ufshcd_is_link_active(hba))
		phy_power_off(ufs->phy);

	return 0;
}

static int exynos_ufs_resume(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	if (!ufshcd_is_link_active(hba))
		phy_power_on(ufs->phy);

	exynos_ufs_config_smu(ufs);
	exynos_ufs_fmp_resume(hba);
	return 0;
}

static int exynosauto_ufs_vh_link_startup_notify(struct ufs_hba *hba,
						 enum ufs_notify_change_status status)
{
	if (status == POST_CHANGE) {
		ufshcd_set_link_active(hba);
		ufshcd_set_ufs_dev_active(hba);
	}

	return 0;
}

static int exynosauto_ufs_vh_wait_ph_ready(struct ufs_hba *hba)
{
	u32 mbox;
	ktime_t start, stop;

	start = ktime_get();
	stop = ktime_add(start, ms_to_ktime(PH_READY_TIMEOUT_MS));

	do {
		mbox = ufshcd_readl(hba, PH2VH_MBOX);
		/* TODO: Mailbox message protocols between the PH and VHs are
		 * not implemented yet. This will be supported later
		 */
		if ((mbox & MH_MSG_MASK) == MH_MSG_PH_READY)
			return 0;

		usleep_range(40, 50);
	} while (ktime_before(ktime_get(), stop));

	return -ETIME;
}

static int exynosauto_ufs_vh_init(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos_ufs *ufs;
	int ret;

	ufs = devm_kzalloc(dev, sizeof(*ufs), GFP_KERNEL);
	if (!ufs)
		return -ENOMEM;

	/* exynos-specific hci */
	ufs->reg_hci = devm_platform_ioremap_resource_byname(pdev, "vs_hci");
	if (IS_ERR(ufs->reg_hci)) {
		dev_err(dev, "cannot ioremap for hci vendor register\n");
		return PTR_ERR(ufs->reg_hci);
	}

	ret = exynosauto_ufs_vh_wait_ph_ready(hba);
	if (ret)
		return ret;

	ufs->drv_data = device_get_match_data(dev);
	if (!ufs->drv_data)
		return -ENODEV;

	exynos_ufs_priv_init(hba, ufs);

	return 0;
}

static int fsd_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	struct ufs_hba *hba = ufs->hba;
	int i;

	ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_clk_period_off),
		       DIV_ROUND_UP(NSEC_PER_SEC,  ufs->mclk_rate));
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x201), 0x12);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0xAA, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x8F, i), 0x3F);
	}

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x12, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x5C, i), 0x38);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x0F, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x65, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x69, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x21, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x22, i), 0x0);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_AUTOMODE_THLD), 0x4E20);

	ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite1_off),
		       0x2e820183);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE), 0x0);

	exynos_ufs_establish_connt(ufs);

	return 0;
}

static int fsd_ufs_post_link(struct exynos_ufs *ufs)
{
	int i;
	struct ufs_hba *hba = ufs->hba;
	u32 hw_cap_min_tactivate;
	u32 peer_rx_min_actv_time_cap;
	u32 max_rx_hibern8_time_cap;

	ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(0x8F, 4),
			&hw_cap_min_tactivate); /* HW Capability of MIN_TACTIVATE */
	ufshcd_dme_get(hba, UIC_ARG_MIB(PA_TACTIVATE),
			&peer_rx_min_actv_time_cap);    /* PA_TActivate */
	ufshcd_dme_get(hba, UIC_ARG_MIB(PA_HIBERN8TIME),
			&max_rx_hibern8_time_cap);      /* PA_Hibern8Time */

	if (peer_rx_min_actv_time_cap >= hw_cap_min_tactivate)
		ufshcd_dme_peer_set(hba, UIC_ARG_MIB(PA_TACTIVATE),
					peer_rx_min_actv_time_cap + 1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_HIBERN8TIME), max_rx_hibern8_time_cap + 1);

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_MODE), 0x01);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_SAVECONFIGTIME), 0xFA);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_MODE), 0x00);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x35, i), 0x05);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x73, i), 0x01);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x41, i), 0x02);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x42, i), 0xAC);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);

	return 0;
}

static int fsd_ufs_pre_pwr_change(struct exynos_ufs *ufs,
					struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXTERMINATION), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_RXTERMINATION), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA0), 12000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA1), 32000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA2), 16000);

	unipro_writel(ufs, 12000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER0);
	unipro_writel(ufs, 32000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER1);
	unipro_writel(ufs, 16000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER2);

	return 0;
}

static int fsd_ufs_suspend(struct exynos_ufs *ufs)
{
	exynos_ufs_gate_clks(ufs);
	hci_writel(ufs, 0, HCI_GPIO_OUT);
	return 0;
}

static inline u32 get_mclk_period_unipro_18(struct exynos_ufs *ufs)
{
	return (16 * 1000 * 1000000UL / ufs->mclk_rate);
}

static int gs101_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	int i;
	u32 tx_line_reset_period, rx_line_reset_period;

	rx_line_reset_period = (RX_LINE_RESET_TIME * ufs->mclk_rate)
				/ NSEC_PER_MSEC;
	tx_line_reset_period = (TX_LINE_RESET_TIME * ufs->mclk_rate)
				/ NSEC_PER_MSEC;

	unipro_writel(ufs, get_mclk_period_unipro_18(ufs), COMP_CLK_PERIOD);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);

	/*
	 * MIB 0x202 = 0x02 — AOSP cal-if init_cfg_evt0 line 83 (38.4 MHz
	 * branch). gs201 ships USE_UFS_REFCLK == USE_38_4_MHZ. Mainline
	 * was missing this entirely. Cal-if writes 0x12 for 26 MHz and
	 * 0x02 for 19.2/38.4 MHz; gs201 = 0x02.
	 */
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x202), 0x02);

#if GS201_AOSP_PCS_WRITES
	/*
	 * (h13) AOSP `__set_pcs` mechanism: bracket each per-lane raw
	 * unipro_writel with a UNIP_COMP_AXI_AUX_FIELD lane-selector
	 * gate. Lane encoding: TX 0..1, RX 4..5. SFR offset is
	 * EXYNOS_PCS_SFR(mib) = 0x2000 + (mib << 2). Computed values
	 * (mclk_period_rnd_off, line_reset_period) match what mainline
	 * writes via the VND_* aliases — only the mechanism differs.
	 */
	dev_dbg(hba->dev, "h13 pre_link: using AOSP __set_pcs mechanism for per-lane PCS writes\n");

	for_each_ufs_rx_lane(ufs, i) {
		u8 lane = EXYNOS_PCS_RX_LANE_0 + i;
		u32 mclk_per = DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate);

		unipro_writel(ufs, EXYNOS_PCS_AUX_WSTRB | (lane & 0xFFFF),
			      UNIP_COMP_AXI_AUX_FIELD);
		unipro_writel(ufs, mclk_per, EXYNOS_PCS_SFR(VND_RX_CLK_PRD));
		unipro_writel(ufs, 0x0, EXYNOS_PCS_SFR(VND_RX_CLK_PRD_EN));
		unipro_writel(ufs, (rx_line_reset_period >> 16) & 0xFF,
			      EXYNOS_PCS_SFR(VND_RX_LINERESET_VALUE2));
		unipro_writel(ufs, (rx_line_reset_period >> 8) & 0xFF,
			      EXYNOS_PCS_SFR(VND_RX_LINERESET_VALUE1));
		unipro_writel(ufs, rx_line_reset_period & 0xFF,
			      EXYNOS_PCS_SFR(VND_RX_LINERESET_VALUE0));
		unipro_writel(ufs, 0x69, EXYNOS_PCS_SFR(0x2f));
		unipro_writel(ufs, 0x1,  EXYNOS_PCS_SFR(0x84));
		unipro_writel(ufs, 0xf6, EXYNOS_PCS_SFR(0x25));
		unipro_writel(ufs, EXYNOS_PCS_AUX_WSTRB, UNIP_COMP_AXI_AUX_FIELD);
	}

	for_each_ufs_tx_lane(ufs, i) {
		u8 lane = EXYNOS_PCS_TX_LANE_0 + i;
		u32 mclk_per = DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate);

		unipro_writel(ufs, EXYNOS_PCS_AUX_WSTRB | (lane & 0xFFFF),
			      UNIP_COMP_AXI_AUX_FIELD);
		unipro_writel(ufs, mclk_per, EXYNOS_PCS_SFR(VND_TX_CLK_PRD));
		unipro_writel(ufs, 0x02, EXYNOS_PCS_SFR(VND_TX_CLK_PRD_EN));
		unipro_writel(ufs, (tx_line_reset_period >> 16) & 0xFF,
			      EXYNOS_PCS_SFR(VND_TX_LINERESET_PVALUE2));
		unipro_writel(ufs, (tx_line_reset_period >> 8) & 0xFF,
			      EXYNOS_PCS_SFR(VND_TX_LINERESET_PVALUE1));
		unipro_writel(ufs, tx_line_reset_period & 0xFF,
			      EXYNOS_PCS_SFR(VND_TX_LINERESET_PVALUE0));
		unipro_writel(ufs, 1, EXYNOS_PCS_SFR(0x04));
		unipro_writel(ufs, 0, EXYNOS_PCS_SFR(0x7F));
		unipro_writel(ufs, EXYNOS_PCS_AUX_WSTRB, UNIP_COMP_AXI_AUX_FIELD);
	}
#else
	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD_EN, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE2, i),
			       (rx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE1, i),
			       (rx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE0, i),
			       (rx_line_reset_period) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x2f, i), 0x69);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x84, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x25, i), 0xf6);
	}

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD_EN, i),
			       0x02);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE2, i),
			       (tx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE1, i),
			       (tx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE0, i),
			       (tx_line_reset_period) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x04, i), 1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x7F, i), 0);
	}
#endif

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID_VALID), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERDEVICEID), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_CONNECTED);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0xA006), 0x8000);

	return 0;
}

static int gs101_ufs_post_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;

	/*
	 * Enable Write Line Unique. This field has to be 0x3
	 * to support Write Line Unique transaction on gs101.
	 */
	hci_writel(ufs, WLU_EN | WLU_BURST_LEN(3), HCI_AXIDMA_RWDATA_BURST_LEN);

	exynos_ufs_enable_dbg_mode(hba);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_SAVECONFIGTIME), 0x3e8);
	exynos_ufs_disable_dbg_mode(hba);

	return 0;
}

/*
 * gs201 post-link: gs101 baseline plus the two UniPro T_AdaptLength
 * writes from AOSP post_init_cfg_evt0/evt1. Pre-link experiment removed
 * after it broke link startup; the AOSP cal-if writes can't be ported
 * piecemeal — see project_ufs_bringup_state.md.
 */
static int gs201_ufs_post_link(struct exynos_ufs *ufs)
{
	int ret = gs101_ufs_post_link(ufs);

	if (ret)
		return ret;

	/*
	 * AOSP cal-if post_init_cfg_evt0 lines 326-327 mark these
	 * (MIB 0x15D2/0x15D3 = bytes 0x3348/0x334C) as
	 * UNIPRO_ADAPT_LENGTH, which is a conditional RMW (cal-if.c:800):
	 *   if (val & 0x80) and (val & 0x7F) < 2: write 0x82
	 *   else if ((val + 1) & 0x3): write val | 0x3
	 * For the typical reset value of 0x0, branch 2 fires and
	 * writes 0x3 (not 0x0). Mainline previously wrote raw 0x0 here
	 * which is a different value. Encode the RMW directly.
	 */
	{
		u32 v;
		v = unipro_readl(ufs, 0x3348);
		if ((v & 0x80) && (v & 0x7F) < 2)
			unipro_writel(ufs, 0x82, 0x3348);
		else if ((v + 1) & 0x3)
			unipro_writel(ufs, v | 0x3, 0x3348);
		v = unipro_readl(ufs, 0x334C);
		if ((v & 0x80) && (v & 0x7F) < 2)
			unipro_writel(ufs, 0x82, 0x334C);
		else if ((v + 1) & 0x3)
			unipro_writel(ufs, v | 0x3, 0x334C);
	}

#if GS201_PRDT_PREFETCH
	{
		u32 val = PRDT_PREFETCH_EN | (ilog2(DATA_UNIT_SIZE) & 0x1F);

		hci_writel(ufs, val, HCI_TXPRDT_ENTRY_SIZE);
		dev_dbg(ufs->hba->dev,
			 "h18 post_link: HCI_TXPRDT_ENTRY_SIZE = 0x%08x (PRDT_PREFETCH_EN | size=12, AOSP parity)\n",
			 val);
	}
#endif

	gs201_dump_ufsp(ufs->hba->dev, "after-cfg");
	gs201_dump_cmu_hsi2(ufs->hba->dev, "post_link");
	gs201_dump_pa_state(ufs->hba, "post_link ");

	return 0;
}

/* (g2) gs201 mainline can't yet survive any HS-Rate-B mode — even after
 * (g) capped at G3 the link reached "FAST series_B G_3 L_2" but immediately
 * hit dl_err 0x80000002 on the first SCSI command, exactly like G4. The
 * CDR lock check (gs101_phy_wait_for_cdr_lock / TRSV_REG339 bit 3) also
 * fails identically at G3 and G4. Conclusion: the PHY isn't producing
 * usable HS data at any gear, not just G4. Force PWM mode so SCSI works
 * and rootfs is reachable while HS is debugged. PWM_G4 × 2L ≈ 5–10 MB/s
 * — slow but functional. Remove once HS-Rate-B is fixed.
 *
 * (h10) Tried disabling this with the phy-gs101-ufs.c PMA transcription-bug
 * fixes (0x25D->0x27D, 0x29E->0x2BE) in place. CDR lock still failed —
 * R338 calibration state DID change (0x9d -> 0x9f) and R33B reached 0
 * earlier, so the typo fix moved something in the PHY, but TRSV_REG339
 * bit 3 still never set. Same dl_err 0x80000002 / scsi_eh_scmd_add WARN
 * as h5. Re-enabled PWM clamp so the device keeps booting; typo fixes
 * remain in tree as a separate (correct-but-not-sufficient) patch.
 *
 * (h11) AOSP `ufs_cal_pre_pmc` port test: when GS201_AOSP_PRE_PMC=1
 * the function below replaces the original DME_SET-based UserData
 * setup with raw unipro_writel matching AOSP's calib_of_hs_rate_b
 * table mechanism+order, masks PA_ERROR_IND_RECEIVED, and writes
 * PA_TxHsAdaptType=1 for HS modes. Disable PWM force at the same
 * time so HS is actually attempted.
 */
/*
 * (h11/h11b) Bisect of the AOSP `ufs_cal_pre_pmc` port:
 *  - h11a (PRE_PMC=1, ADAPT=1): mask + unipro_writel UserData/L2 in AOSP order
 *    + PA_TxHsAdaptType=1. pwr_change fails with upmcrs:0x5, all UIC error
 *    counters zero. Same fatal as m2/m4/m5.
 *  - h11b (PRE_PMC=1, ADAPT=0): same as h11a but skip the adapt write.
 *    pwr_change reaches FAST series_B G_4 L_2 cleanly. CDR lock still fails
 *    identically (TRSV_REG339=0, dl_err 0x80000002). Confirms two things:
 *      1) PA_TxHsAdaptType=1 is the SOLE trigger of upmcrs:0x5 — independent
 *         of write mechanism, mask, or AOSP order.
 *      2) CDR lock failure is independent of pre_pmc semantics — happens
 *         even with AOSP-faithful pre_pmc. Bug is in PHY/PCS state, not PA.
 *
 * Default reverted: PWM force on, AOSP pre_pmc on (no harm at PWM, slightly
 * better matches AOSP), adapt off (would break pwr_change). Re-enable adapt
 * only if you've found whatever AOSP-only precondition makes it safe.
 */
/*
 * (h12) Tested HS-Rate-B with the buggy (h4) PCS clobber-writes REMOVED.
 * Result: same CDR-lock failure. TRSV_REG339=0, dl_err 0x80000002,
 * scsi_eh_scmd_add WARN. So the (h4) writes were destructive bugs of
 * my own making, but they were NOT the cause of HS-Rate-B failure —
 * removing them gets the PHY init clean but CDR lock still doesn't
 * advance.
 *
 * (h13) Test the AOSP `__set_pcs` mechanism for per-lane PHY_PCS_RX/TX
 * writes. Mainline uses ufshcd_dme_set(UIC_ARG_MIB_SEL(addr, lane), val)
 * for these, going through the controller's DME state machine. AOSP
 * uses raw unipro_writel bracketed by UNIP_COMP_AXI_AUX_FIELD lane
 * selector. Plausible the DME path silently no-ops for some PCS
 * attributes on gs201. With GS201_AOSP_PCS_WRITES=1, gs101_ufs_pre_link
 * uses the AOSP mechanism for the per-lane writes; everything else
 * (PHY_PCS_COMN, UNIPRO_STD_MIB) stays on dme_set for now.
 *
 * NOTE: the GS201_* switch defines themselves live near the top of
 * this file (above gs101_ufs_pre_link), not here — the preprocessor
 * scans top-to-bottom and undefined-macro `#if` silently evaluates
 * to 0, so any switch used above its definition gets silently
 * skipped. Learned the hard way during h13.
 */

static int gs101_ufs_pre_pwr_change(struct exynos_ufs *ufs,
					 struct ufs_pa_layer_attr *pwr)
{
	struct ufs_hba *hba = ufs->hba;
#if GS201_AOSP_PRE_PMC
	bool is_hs;
	u32 dl_err_mask;
#endif

	gs201_dump_cmu_hsi2(hba->dev, "pre_pwr ");

#if GS201_MAINLINE_FORCE_PWM_GEAR
	if (pwr->pwr_rx == FAST_MODE || pwr->pwr_rx == FASTAUTO_MODE ||
	    pwr->pwr_tx == FAST_MODE || pwr->pwr_tx == FASTAUTO_MODE) {
		dev_dbg(hba->dev,
			 "gs101_pre_pwr: forcing PWM gear=%u rx/tx (was rx=%u/%u tx=%u/%u hs_rate=%u) — HS broken on mainline\n",
			 GS201_MAINLINE_FORCE_PWM_GEAR,
			 pwr->pwr_rx, pwr->gear_rx,
			 pwr->pwr_tx, pwr->gear_tx, pwr->hs_rate);
		pwr->pwr_rx = SLOWAUTO_MODE;
		pwr->pwr_tx = SLOWAUTO_MODE;
		pwr->gear_rx = GS201_MAINLINE_FORCE_PWM_GEAR;
		pwr->gear_tx = GS201_MAINLINE_FORCE_PWM_GEAR;
		pwr->hs_rate = 0;
	}
#endif

#if GS201_FORCE_HS_RATE_A
	if ((pwr->pwr_rx == FAST_MODE || pwr->pwr_rx == FASTAUTO_MODE ||
	     pwr->pwr_tx == FAST_MODE || pwr->pwr_tx == FASTAUTO_MODE) &&
	    pwr->hs_rate != PA_HS_MODE_A) {
		dev_dbg(hba->dev,
			 "A2 pre_pwr: forcing hs_rate=A (was %u) — testing whether Rate-A side-steps Rate-B CDR-lock failure\n",
			 pwr->hs_rate);
		pwr->hs_rate = PA_HS_MODE_A;
	}
#endif

#if GS201_FORCE_HS_GEAR
	if ((pwr->pwr_rx == FAST_MODE || pwr->pwr_rx == FASTAUTO_MODE ||
	     pwr->pwr_tx == FAST_MODE || pwr->pwr_tx == FASTAUTO_MODE) &&
	    (pwr->gear_rx > GS201_FORCE_HS_GEAR ||
	     pwr->gear_tx > GS201_FORCE_HS_GEAR)) {
		dev_dbg(hba->dev,
			 "A2c pre_pwr: clamping HS gear to %u (was rx=%u tx=%u) — testing if lower symbol rate fixes dl_err 0x80000002\n",
			 GS201_FORCE_HS_GEAR, pwr->gear_rx, pwr->gear_tx);
		pwr->gear_rx = GS201_FORCE_HS_GEAR;
		pwr->gear_tx = GS201_FORCE_HS_GEAR;
	}
#endif

#if GS201_AOSP_PRE_PMC
	/*
	 * (h11) Port AOSP gs201 ufs_cal_pre_pmc semantics into mainline.
	 *
	 * AOSP's pre_pmc does, in this order:
	 *   1. read-modify-write UNIP_DL_ERROR_IRQ_MASK |= PA_ERROR_IND_RECEIVED
	 *   2. iterate `calib_of_hs_rate_b` (HS) or `calib_of_pwm` (PWM/SLOW),
	 *      where every entry is `unipro_writel(val, sfr)` — the UNIPRO_STD_MIB
	 *      type goes through `unipro_writel`, NOT `ufshcd_dme_set`, and there
	 *      is no DME state-machine command issued.
	 *   3. The PHY_PMA_TRSV writes (0xDA4=0x11, 0x918=0x03) come last in
	 *      the cal table; mainline does these via phy_calibrate(CFG_PRE_PWR_HS)
	 *      after this vop returns, which is functionally equivalent.
	 *
	 * The earlier (m2/m4/m5) attempts only added a single piece (PA_TxHsAdaptType
	 * via various write mechanisms ± mask) on top of the original DME_SET
	 * UserData writes. This port replaces ALL the local-PA writes with raw
	 * unipro_writel and reorders to match AOSP exactly. Hypothesis: DME_SET
	 * for UserData triggers DME state-machine side-effects that interact
	 * badly with a subsequent adapt-enabled pwr_change. AOSP avoids them by
	 * never issuing DME_SET commands for these attributes.
	 */
	is_hs = (pwr->pwr_rx == FAST_MODE || pwr->pwr_rx == FASTAUTO_MODE ||
		 pwr->pwr_tx == FAST_MODE || pwr->pwr_tx == FASTAUTO_MODE);

	dl_err_mask = unipro_readl(ufs, UNIP_DL_ERROR_IRQ_MASK_REG) |
		      UNIP_DL_PA_ERROR_IND_RECEIVED_BIT;
	unipro_writel(ufs, dl_err_mask, UNIP_DL_ERROR_IRQ_MASK_REG);
	dev_dbg(hba->dev,
		 "h11 pre_pmc: masked PA_ERROR_IND_RECEIVED -> 0x%08x (is_hs=%d)\n",
		 dl_err_mask, is_hs);

#if GS201_AOSP_PRE_PMC_ADAPT
	if (is_hs) {
		/* AOSP calib_of_hs_rate_b, in original table order */
		unipro_writel(ufs, 0x1, UNIP_PA_TXHSADAPTTYPE);
		dev_dbg(hba->dev,
			 "h11 pre_pmc: PA_TxHsAdaptType=1 (sfr 0x3350) written\n");
	}
#endif

	unipro_writel(ufs, 8064, UNIP_DL_FC0PROTTIMEOUTVAL);
	unipro_writel(ufs, 28224, UNIP_DL_TC0REPLAYTIMEOUTVAL);
	unipro_writel(ufs, 20160, UNIP_DL_AFC0REQTIMEOUTVAL);
	unipro_writel(ufs, 12000, UNIP_PA_PWRMODEUSERDATA0_REG);
	unipro_writel(ufs, 32000, UNIP_PA_PWRMODEUSERDATA1_REG);
	unipro_writel(ufs, 16000, UNIP_PA_PWRMODEUSERDATA2_REG);
	unipro_writel(ufs, 8064, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER0);
	unipro_writel(ufs, 28224, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER1);
	unipro_writel(ufs, 20160, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER2);
	unipro_writel(ufs, 12000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER0);
	unipro_writel(ufs, 32000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER1);
	unipro_writel(ufs, 16000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER2);

	dev_dbg(hba->dev,
		 "h11 pre_pmc: AOSP-style writes applied (is_hs=%d)\n", is_hs);
#else
	/*
	 * Original mainline pre_pwr_change. (m2/m4/m5) tried adding adapt here
	 * piecemeal and broke pwr_change with upmcrs:0x5 every time. The h11
	 * AOSP port above replaces this whole block when enabled.
	 */
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA0), 12000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA1), 32000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA2), 16000);
	unipro_writel(ufs, 8064, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER0);
	unipro_writel(ufs, 28224, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER1);
	unipro_writel(ufs, 20160, UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER2);
	unipro_writel(ufs, 12000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER0);
	unipro_writel(ufs, 32000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER1);
	unipro_writel(ufs, 16000, UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER2);
#endif

#if GS201_MASK_SBFES_IRQ
	{
		u32 ie_before = ufshcd_readl(hba, REG_INTERRUPT_ENABLE);
		u32 ie_after  = ie_before & ~SYSTEM_BUS_FATAL_ERROR;

		if (ie_before != ie_after) {
			ufshcd_writel(hba, ie_after, REG_INTERRUPT_ENABLE);
			dev_dbg(hba->dev,
				 "h9 pre_pwr: masked SBFES in IE: 0x%08x -> 0x%08x (PWM workaround)\n",
				 ie_before, ie_after);
		} else {
			dev_dbg(hba->dev,
				 "h9 pre_pwr: IE = 0x%08x (SBFES already clear, mask was no-op)\n",
				 ie_before);
		}
	}
#endif

	return 0;
}

#if GS201_AOSP_POST_PMC
/*
 * gs101/gs201 post-PMC hook. For PWM mode, drives the Samsung PHY state
 * machine through CFG_PRE_PWR_HS → CFG_POST_PWR_HS → CFG_PRE_INIT to apply
 * the tensor_gs101_pre_pwr_hs_config and tensor_gs101_post_pwr_hs_config
 * tables (which mainline otherwise skips for PWM, see exynos_ufs_pre_pwr_mode
 * + exynos_ufs_post_pwr_mode HS-only gating). For HS modes, those calibrate
 * calls already happen in the surrounding exynos_ufs_*_pwr_mode paths, so
 * this hook does nothing.
 */
static int gs101_ufs_post_pwr_change(struct exynos_ufs *ufs,
				     const struct ufs_pa_layer_attr *pwr_req)
{
	struct ufs_hba *hba = ufs->hba;
	struct phy *generic_phy = ufs->phy;

	if (ufshcd_is_hs_mode(pwr_req)) {
#if GS201_HS_PWR_SETTLE_MS
		dev_dbg(hba->dev,
			 "A2d post_pwr: HS pwr_change OK, settling for %u ms before allowing first SCSI\n",
			 (unsigned int)GS201_HS_PWR_SETTLE_MS);
		msleep(GS201_HS_PWR_SETTLE_MS);
#endif
		gs201_dump_cmu_hsi2(hba->dev, "post_pwr_HS ");
		gs201_dump_pa_state(hba, "post_pwr_HS ");
		return 0;
	}

	dev_dbg(hba->dev,
		 "h16 post_pwr: forcing PHY calibrate x2 for non-HS gear (advance state machine through PRE/POST_PWR_HS to apply AOSP-equivalent post_calib_of_pwm writes)\n");

	/* Advance: CFG_PRE_PWR_HS → CFG_POST_PWR_HS, runs PRE_PWR_HS table. */
	phy_calibrate(generic_phy);
	/*
	 * Advance: CFG_POST_PWR_HS → CFG_PRE_INIT, runs POST_PWR_HS table
	 * including the PWM-gated PMA writes. Also calls drvdata->wait_for_cdr
	 * which times out for PWM (no CDR lock); samsung_ufs_phy_calibrate
	 * returns -ETIMEDOUT but exynos_ufs_post_pwr_mode discards the rc.
	 */
	phy_calibrate(generic_phy);

	gs201_dump_cmu_hsi2(hba->dev, "post_pwr_PWM");
	gs201_dump_pa_state(hba, "post_pwr_PWM");

	return 0;
}
#endif

static int exynos_ufs_set_dma_mask(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);
	u8 bits;

	if (ufs->drv_data && ufs->drv_data->dma_mask_bits) {
		bits = ufs->drv_data->dma_mask_bits;
		dev_dbg(hba->dev,
			 "exynos UFS: forcing %u-bit DMA mask (variant override; controller cap MASK_64_ADDRESSING_SUPPORT may lie)\n",
			 bits);
		return dma_set_mask_and_coherent(hba->dev, DMA_BIT_MASK(bits));
	}
	if (hba->capabilities & MASK_64_ADDRESSING_SUPPORT) {
		if (!dma_set_mask_and_coherent(hba->dev, DMA_BIT_MASK(64)))
			return 0;
	}
	return dma_set_mask_and_coherent(hba->dev, DMA_BIT_MASK(32));
}

static const struct exynos_ufs_drv_data gs201_ufs_drvs;

static void exynos_ufs_config_scsi_dev(struct scsi_device *sdev,
				       struct queue_limits *lim)
{
	struct ufs_hba *hba = shost_priv(sdev->host);
	struct exynos_ufs *ufs = ufshcd_get_variant(hba);

	if (ufs->drv_data != &gs201_ufs_drvs)
		return;

#if GS201_FORCE_QDEPTH_1
	scsi_change_queue_depth(sdev, 1);
	dev_dbg(hba->dev,
		 "h15b: clamped sdev lun=%llu queue_depth=1 (gs201 PWM workaround)\n",
		 sdev->lun);
#endif

#if GS201_MAX_HW_SECTORS_KB
	{
		/*
		 * Mutate the `lim` snapshot the SCSI core passes in — that's
		 * the source of truth, applied to q->limits via
		 * queue_limits_commit_update() right after this hook returns
		 * (drivers/scsi/scsi_scan.c). Direct writes to q->limits here
		 * are silently overwritten by that commit, which is what the
		 * h15a v3 attempt hit. The vops signature was extended to
		 * include `lim` specifically to make this clamp possible.
		 */
		unsigned int max_sectors =
			(GS201_MAX_HW_SECTORS_KB * 1024) >> SECTOR_SHIFT;

		if (lim->max_hw_sectors > max_sectors)
			lim->max_hw_sectors = max_sectors;
		if (lim->max_sectors    > max_sectors)
			lim->max_sectors    = max_sectors;
		dev_dbg(hba->dev,
			 "h15a: clamped sdev lun=%llu max_hw_sectors=%u (=%uKB) (gs201 PWM workaround)\n",
			 sdev->lun, max_sectors,
			 (unsigned int)GS201_MAX_HW_SECTORS_KB);
	}
#endif
}

static const struct ufs_hba_variant_ops ufs_hba_exynos_ops = {
	.name				= "exynos_ufs",
	.init				= exynos_ufs_init,
	.exit				= exynos_ufs_exit,
	.hce_enable_notify		= exynos_ufs_hce_enable_notify,
	.link_startup_notify		= exynos_ufs_link_startup_notify,
	.negotiate_pwr_mode		= exynos_ufs_negotiate_pwr_mode,
	.pwr_change_notify		= exynos_ufs_pwr_change_notify,
	.setup_clocks			= exynos_ufs_setup_clocks,
	.setup_xfer_req			= exynos_ufs_specify_nexus_t_xfer_req,
	.setup_task_mgmt		= exynos_ufs_specify_nexus_t_tm_req,
	.hibern8_notify			= exynos_ufs_hibern8_notify,
	.suspend			= exynos_ufs_suspend,
	.resume				= exynos_ufs_resume,
	.fill_crypto_prdt		= exynos_ufs_fmp_fill_prdt,
	.set_dma_mask			= exynos_ufs_set_dma_mask,
	.config_scsi_dev		= exynos_ufs_config_scsi_dev,
};

static struct ufs_hba_variant_ops ufs_hba_exynosauto_vh_ops = {
	.name				= "exynosauto_ufs_vh",
	.init				= exynosauto_ufs_vh_init,
	.link_startup_notify		= exynosauto_ufs_vh_link_startup_notify,
};

static int exynos_ufs_probe(struct platform_device *pdev)
{
	int err;
	struct device *dev = &pdev->dev;
	const struct ufs_hba_variant_ops *vops = &ufs_hba_exynos_ops;
	const struct exynos_ufs_drv_data *drv_data =
		device_get_match_data(dev);

	if (drv_data && drv_data->vops)
		vops = drv_data->vops;

	err = ufshcd_pltfrm_init(pdev, vops);
	if (err)
		dev_err(dev, "ufshcd_pltfrm_init() failed %d\n", err);

	return err;
}

static void exynos_ufs_remove(struct platform_device *pdev)
{
	ufshcd_pltfrm_remove(pdev);
}

static struct exynos_ufs_uic_attr exynos7_uic_attr = {
	.tx_trailingclks		= 0x10,
	.tx_dif_p_nsec			= 3000000,	/* unit: ns */
	.tx_dif_n_nsec			= 1000000,	/* unit: ns */
	.tx_high_z_cnt_nsec		= 20000,	/* unit: ns */
	.tx_base_unit_nsec		= 100000,	/* unit: ns */
	.tx_gran_unit_nsec		= 4000,		/* unit: ns */
	.tx_sleep_cnt			= 1000,		/* unit: ns */
	.tx_min_activatetime		= 0xa,
	.rx_filler_enable		= 0x2,
	.rx_dif_p_nsec			= 1000000,	/* unit: ns */
	.rx_hibern8_wait_nsec		= 4000000,	/* unit: ns */
	.rx_base_unit_nsec		= 100000,	/* unit: ns */
	.rx_gran_unit_nsec		= 4000,		/* unit: ns */
	.rx_sleep_cnt			= 1280,		/* unit: ns */
	.rx_stall_cnt			= 320,		/* unit: ns */
	.rx_hs_g1_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g2_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g3_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g1_prep_sync_len_cap	= PREP_LEN(0xf),
	.rx_hs_g2_prep_sync_len_cap	= PREP_LEN(0xf),
	.rx_hs_g3_prep_sync_len_cap	= PREP_LEN(0xf),
	.pa_dbg_clk_period_off		= PA_DBG_CLK_PERIOD,
	.pa_dbg_opt_suite1_val		= 0x30103,
	.pa_dbg_opt_suite1_off		= PA_DBG_OPTION_SUITE,
};

static const struct exynos_ufs_drv_data exynosauto_ufs_drvs = {
	.uic_attr		= &exynos7_uic_attr,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.opts			= EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL |
				  EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR |
				  EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX,
	.iocc_mask		= UFS_EXYNOSAUTO_SHARABLE,
	.drv_init		= exynosauto_ufs_drv_init,
	.post_hce_enable	= exynosauto_ufs_post_hce_enable,
	.pre_link		= exynosauto_ufs_pre_link,
	.pre_pwr_change		= exynosauto_ufs_pre_pwr_change,
	.post_pwr_change	= exynosauto_ufs_post_pwr_change,
};

static const struct exynos_ufs_drv_data exynosauto_ufs_vh_drvs = {
	.vops			= &ufs_hba_exynosauto_vh_ops,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCI_QUIRK_BROKEN_HCE |
				  UFSHCD_QUIRK_BROKEN_UIC_CMD |
				  UFSHCD_QUIRK_SKIP_PH_CONFIGURATION |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.opts			= EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX,
};

static const struct exynos_ufs_drv_data exynos_ufs_drvs = {
	.uic_attr		= &exynos7_uic_attr,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
				  UFSHCI_QUIRK_BROKEN_HCE |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCI_QUIRK_SKIP_MANUAL_WB_FLUSH_CTRL |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.opts			= EXYNOS_UFS_OPT_HAS_APB_CLK_CTRL |
				  EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL |
				  EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX |
				  EXYNOS_UFS_OPT_SKIP_CONNECTION_ESTAB |
				  EXYNOS_UFS_OPT_USE_SW_HIBERN8_TIMER,
	.pre_link		= exynos7_ufs_pre_link,
	.post_link		= exynos7_ufs_post_link,
	.pre_pwr_change		= exynos7_ufs_pre_pwr_change,
	.post_pwr_change	= exynos7_ufs_post_pwr_change,
};

static struct exynos_ufs_uic_attr gs101_uic_attr = {
	.tx_trailingclks		= 0xff,
	.pa_dbg_opt_suite1_val		= 0x90913C1C,
	.pa_dbg_opt_suite1_off		= PA_GS101_DBG_OPTION_SUITE1,
	.pa_dbg_opt_suite2_val		= 0xE01C115F,
	.pa_dbg_opt_suite2_off		= PA_GS101_DBG_OPTION_SUITE2,
};

static struct exynos_ufs_uic_attr fsd_uic_attr = {
	.tx_trailingclks		= 0x10,
	.tx_dif_p_nsec			= 3000000,	/* unit: ns */
	.tx_dif_n_nsec			= 1000000,	/* unit: ns */
	.tx_high_z_cnt_nsec		= 20000,	/* unit: ns */
	.tx_base_unit_nsec		= 100000,	/* unit: ns */
	.tx_gran_unit_nsec		= 4000,		/* unit: ns */
	.tx_sleep_cnt			= 1000,		/* unit: ns */
	.tx_min_activatetime		= 0xa,
	.rx_filler_enable		= 0x2,
	.rx_dif_p_nsec			= 1000000,	/* unit: ns */
	.rx_hibern8_wait_nsec		= 4000000,	/* unit: ns */
	.rx_base_unit_nsec		= 100000,	/* unit: ns */
	.rx_gran_unit_nsec		= 4000,		/* unit: ns */
	.rx_sleep_cnt			= 1280,		/* unit: ns */
	.rx_stall_cnt			= 320,		/* unit: ns */
	.rx_hs_g1_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g2_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g3_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g1_prep_sync_len_cap	= PREP_LEN(0xf),
	.rx_hs_g2_prep_sync_len_cap	= PREP_LEN(0xf),
	.rx_hs_g3_prep_sync_len_cap	= PREP_LEN(0xf),
	.pa_dbg_clk_period_off		= PA_DBG_CLK_PERIOD,
	.pa_dbg_opt_suite1_val		= 0x2E820183,
	.pa_dbg_opt_suite1_off		= PA_DBG_OPTION_SUITE,
};

static const struct exynos_ufs_drv_data fsd_ufs_drvs = {
	.uic_attr               = &fsd_uic_attr,
	.quirks                 = UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR,
	.opts                   = EXYNOS_UFS_OPT_HAS_APB_CLK_CTRL |
				  EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL |
				  EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR |
				  EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX,
	.pre_link               = fsd_ufs_pre_link,
	.post_link              = fsd_ufs_post_link,
	.pre_pwr_change         = fsd_ufs_pre_pwr_change,
	.suspend                = fsd_ufs_suspend,
};

static const struct exynos_ufs_drv_data gs101_ufs_drvs = {
	.uic_attr		= &gs101_uic_attr,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCI_QUIRK_SKIP_MANUAL_WB_FLUSH_CTRL |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.opts			= EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR |
				  EXYNOS_UFS_OPT_UFSPR_SECURE |
				  EXYNOS_UFS_OPT_TIMER_TICK_SELECT,
	.iocc_mask		= UFS_GS101_SHARABLE,
	.drv_init		= gs101_ufs_drv_init,
	.pre_link		= gs101_ufs_pre_link,
	.post_link		= gs101_ufs_post_link,
	.pre_pwr_change		= gs101_ufs_pre_pwr_change,
#if GS201_AOSP_POST_PMC
	.post_pwr_change	= gs101_ufs_post_pwr_change,
#endif
	.suspend		= gs101_ufs_suspend,
};

static const struct exynos_ufs_drv_data exynosautov920_ufs_drvs = {
	.uic_attr               = &exynos7_uic_attr,
	.quirks                 = UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.opts                   = EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL |
				  EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR |
				  EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX |
				  EXYNOS_UFS_OPT_TIMER_TICK_SELECT,
	.iocc_mask		= UFS_EXYNOSAUTOV920_SHARABLE,
	.drv_init               = exynosauto_ufs_drv_init,
	.post_hce_enable        = exynosauto_ufs_post_hce_enable,
	.pre_link               = exynosautov920_ufs_pre_link,
	.post_link              = exynosautov920_ufs_post_link,
	.pre_pwr_change         = exynosautov920_ufs_pre_pwr_change,
};

/*
 * gs201 (Tensor G2) variant. Same hooks as gs101 EXCEPT:
 *
 * - EXYNOS_UFS_OPT_UFSPR_SECURE is dropped. With it set, fmp_init issues
 *   SMC_CMD_FMP_SECURITY + SMC_CMD_SMU(SMU_INIT) to BL31 to set
 *   FMPSECURITY0.DESCTYPE=3 (128-byte PRDT entries) and then
 *   ufshcd_set_sg_entry_size(128). On gs201 those SMCs return 0 (no error
 *   logged) but BL31 doesn't actually program FMPSECURITY0 — hardware keeps
 *   reading 16-byte PRDTs while the kernel writes 128-byte entries. Result:
 *   garbage UTRDs, garbage UPIU responses, NOP OUT failed -22 with the
 *   response slot still containing the original NOP_OUT (0x00).
 *
 *   Without UFSPR_SECURE, fmp_init early-returns (no FMP/inline-crypto), and
 *   exynos_ufs_config_smu falls through to direct MMIO writes at the UFSP
 *   base (0x14600000). If gs201 firewalls those, we'll see SError on probe;
 *   then we'd need a gs201-specific config_smu that routes through SMC.
 */
static const struct exynos_ufs_drv_data gs201_ufs_drvs = {
	.uic_attr		= &gs101_uic_attr,
	/*
	 * gs101 sets UFSHCD_QUIRK_PRDT_BYTE_GRAN — gs201 does NOT need it.
	 * gs201 controller follows UFS spec (UTRD response_upiu_offset and
	 * response_upiu_length are in DWORDS). With the quirk set, mainline
	 * writes those fields in BYTES; controller interprets as DWORDS and
	 * writes the response 4× further into the UCD than mainline reads
	 * back from. Result: link startup OK, NOP_OUT "completes" with no
	 * UIC errors, but RSP UPIU appears empty (it's actually at
	 * UCD+0x800 instead of UCD+0x200, confirmed via dump).
	 */
	.quirks			= UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCI_QUIRK_SKIP_MANUAL_WB_FLUSH_CTRL |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING |
				  /* (g3) Auto-hibern8 exit triggers HOST_BUS_FATAL_ERROR
				   * (IS BIT(17), saved_err=0x20000) ~36s into operation
				   * with the (g2) PWM workaround. Disable until either
				   * HS-Rate-B works (and we re-test H8 there) or the H8
				   * exit path on gs201 mainline is fully wired up. */
				  UFSHCD_QUIRK_BROKEN_AUTO_HIBERN8,
	.opts			= EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR |
				  EXYNOS_UFS_OPT_TIMER_TICK_SELECT,
	.iocc_mask		= UFS_GS101_SHARABLE,
	/*
	 * (h2/h3) gs201 controller advertises MASK_64_ADDRESSING_SUPPORT in
	 * caps reg (cap=0x1303ff1f, bit 24 set), but high-memory DMA writes
	 * trip SYSTEM_BUS_FATAL_ERROR (IS BIT(17), saved_err=0x20000) the
	 * moment the kernel hands out a >4GB buffer. Confirmed with the (h2)
	 * OCS_INVALID dump showing PRD addr 0x899fbd000 immediately before
	 * the bus fault. AOSP's ufs-exynos.c hard-codes
	 * `static u64 exynos_ufs_dma_mask = DMA_BIT_MASK(32)` for the entire
	 * exynos UFS family, suggesting this is a long-standing controller
	 * issue across the Tensor SoCs.
	 */
	.dma_mask_bits		= 32,
	.drv_init		= gs201_ufs_drv_init,
	.pre_link		= gs101_ufs_pre_link,
	.post_link		= gs201_ufs_post_link,
	.pre_pwr_change		= gs101_ufs_pre_pwr_change,
#if GS201_AOSP_POST_PMC
	.post_pwr_change	= gs101_ufs_post_pwr_change,
#endif
	.suspend		= gs101_ufs_suspend,
};

static const struct of_device_id exynos_ufs_of_match[] = {
	{ .compatible = "google,gs101-ufs",
	  .data	      = &gs101_ufs_drvs },
	{ .compatible = "google,gs201-ufs",
	  .data	      = &gs201_ufs_drvs },
	{ .compatible = "samsung,exynos7-ufs",
	  .data	      = &exynos_ufs_drvs },
	{ .compatible = "samsung,exynosautov9-ufs",
	  .data	      = &exynosauto_ufs_drvs },
	{ .compatible = "samsung,exynosautov9-ufs-vh",
	  .data	      = &exynosauto_ufs_vh_drvs },
	{ .compatible = "samsung,exynosautov920-ufs",
	  .data       = &exynosautov920_ufs_drvs },
	{ .compatible = "tesla,fsd-ufs",
	  .data       = &fsd_ufs_drvs },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_ufs_of_match);

static const struct dev_pm_ops exynos_ufs_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ufshcd_system_suspend, ufshcd_system_resume)
	SET_RUNTIME_PM_OPS(ufshcd_runtime_suspend, ufshcd_runtime_resume, NULL)
	.prepare	 = ufshcd_suspend_prepare,
	.complete	 = ufshcd_resume_complete,
};

static struct platform_driver exynos_ufs_pltform = {
	.probe	= exynos_ufs_probe,
	.remove = exynos_ufs_remove,
	.driver	= {
		.name	= "exynos-ufshc",
		.pm	= &exynos_ufs_pm_ops,
		.of_match_table = exynos_ufs_of_match,
	},
};
module_platform_driver(exynos_ufs_pltform);

MODULE_AUTHOR("Alim Akhtar <alim.akhtar@samsung.com>");
MODULE_AUTHOR("Seungwon Jeon  <essuuj@gmail.com>");
MODULE_DESCRIPTION("Exynos UFS HCI Driver");
MODULE_LICENSE("GPL v2");
