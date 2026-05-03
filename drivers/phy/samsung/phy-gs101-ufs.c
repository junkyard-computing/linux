// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFS PHY driver data for Google Tensor gs101 SoC
 *
 * Copyright (C) 2024 Linaro Ltd
 * Author: Peter Griffin <peter.griffin@linaro.org>
 */

#include "phy-samsung-ufs.h"

#define TENSOR_GS101_PHY_CTRL		0x3ec8
#define TENSOR_GS101_PHY_CTRL_MASK	0x1
#define TENSOR_GS101_PHY_CTRL_EN	BIT(0)
#define PHY_GS101_LANE_OFFSET		0x200
#define TRSV_REG338			0x338
#define LN0_MON_RX_CAL_DONE		BIT(3)
#define TRSV_REG339			0x339
#define LN0_MON_RX_CDR_FLD_CK_MODE_DONE BIT(3)
#define TRSV_REG222			0x222
#define LN0_OVRD_RX_CDR_EN		BIT(4)
#define LN0_RX_CDR_EN			BIT(3)

#define PHY_PMA_TRSV_ADDR(reg, lane)	(PHY_APB_ADDR((reg) + \
					((lane) * PHY_GS101_LANE_OFFSET)))

#define PHY_TRSV_REG_CFG_GS101(o, v, d) \
	PHY_TRSV_REG_CFG_OFFSET(o, v, d, PHY_GS101_LANE_OFFSET)

/* Calibration for phy initialization */
static const struct samsung_ufs_phy_cfg tensor_gs101_pre_init_cfg[] = {
	PHY_COMN_REG_CFG(0x43, 0x10,  PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x3C, 0x14,  PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x46, 0x48,  PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x200, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x201, 0x06, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x202, 0x06, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x203, 0x0a, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x204, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x205, 0x11, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x207, 0x0c, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2E1, 0xc0, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x22D, 0xb8, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x234, 0x60, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x238, 0x13, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x239, 0x48, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x23A, 0x01, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x23B, 0x25, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x23C, 0x2a, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x23D, 0x01, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x23E, 0x13, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x23F, 0x13, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x240, 0x4a, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x243, 0x40, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x244, 0x02, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x25D, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x25E, 0x3f, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x25F, 0xff, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x273, 0x33, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x274, 0x50, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x284, 0x02, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x285, 0x02, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2A2, 0x04, PWR_MODE_ANY),
	/*
	 * (h10) Was 0x25D=0x01: byte-offset to register-index transcription
	 * error in original mainline port. AOSP `init_cfg_evt0` writes
	 * 0x9F4=0x01 (= reg 0x27D), not 0x974=0x01 (= reg 0x25D). Mainline's
	 * 0x25D=0x01 also clobbered the earlier 0x25D=0x00 write at the top
	 * of this table, so the final state of that register was wrong too.
	 */
	PHY_TRSV_REG_CFG_GS101(0x27D, 0x01, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2FA, 0x01, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x286, 0x03, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x287, 0x03, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x288, 0x03, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x289, 0x03, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2B3, 0x04, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2B6, 0x0b, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2B7, 0x0b, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2B8, 0x0b, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2B9, 0x0b, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2BA, 0x0b, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2BB, 0x06, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2BC, 0x06, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2BD, 0x06, PWR_MODE_ANY),
	/*
	 * (h10) Was 0x29E=0x06: same transcription bug class as the 0x27D
	 * fix above. AOSP `init_cfg_evt0` writes 0xAF8=0x06 (= reg 0x2BE),
	 * not 0xA78=0x06 (= reg 0x29E). Mainline was writing to a register
	 * AOSP never touches and skipping the one AOSP does.
	 */
	PHY_TRSV_REG_CFG_GS101(0x2BE, 0x06, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2E4, 0x1a, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2ED, 0x25, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x269, 0x1a, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x2F4, 0x2f, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x34B, 0x01, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x34C, 0x23, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x34D, 0x23, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x34E, 0x45, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x34F, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x350, 0x31, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x351, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x352, 0x02, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x353, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x354, 0x01, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x43, 0x18, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x43, 0x00, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg tensor_gs101_pre_pwr_hs_config[] = {
	PHY_TRSV_REG_CFG_GS101(0x369, 0x11, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x246, 0x03, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

/* Calibration for HS mode series A/B */
static const struct samsung_ufs_phy_cfg tensor_gs101_post_pwr_hs_config[] = {
	PHY_COMN_REG_CFG(0x8, 0x60, PWR_MODE_PWM_ANY),
	PHY_TRSV_REG_CFG_GS101(0x222, 0x08, PWR_MODE_PWM_ANY),
	PHY_TRSV_REG_CFG_GS101(0x246, 0x01, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

/*
 * NOTE on AOSP `PHY_EMB_CDR_WAIT @ 0xCE4`: this is a POLL spec, not a write.
 * The AOSP cal-if entry `{0xCE4, 0x08, PHY_EMB_CDR_WAIT}` means "poll PMA reg
 * 0xCE4 for mask 0x08", which is exactly what gs101_phy_wait_for_cdr_lock
 * already does (`TRSV_REG339 << 2 == 0xCE4`, mask BIT(3) == 0x08). The earlier
 * "write 0x08 to 0x339" attempt was a no-op semantically. Instrumentation
 * added 2026-04-26 confirms TRSV_REG339 stays literal 0x00 across 8ms while
 * neighbouring R33B does change (3->0) — CDR engine is alive but never
 * advances to "lock done", suggesting a missing pre-pwr setup write rather
 * than a wrong polling target.
 */

static const struct samsung_ufs_phy_cfg tensor_gs101_post_h8_enter[] = {
	PHY_TRSV_REG_CFG_GS101(0x262, 0x08, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x265, 0x0A, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x1, 0x8,  PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x0, 0x86,  PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x8, 0x60,  PWR_MODE_HS_ANY),
	PHY_TRSV_REG_CFG_GS101(0x222, 0x08, PWR_MODE_HS_ANY),
	PHY_TRSV_REG_CFG_GS101(0x246, 0x01, PWR_MODE_HS_ANY),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg tensor_gs101_pre_h8_exit[] = {
	PHY_COMN_REG_CFG(0x0, 0xC6,  PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x1, 0x0C,  PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x262, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_GS101(0x265, 0x00, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x8, 0xE0,  PWR_MODE_HS_ANY),
	PHY_TRSV_REG_CFG_GS101(0x246, 0x03, PWR_MODE_HS_ANY),
	PHY_TRSV_REG_CFG_GS101(0x222, 0x18, PWR_MODE_HS_ANY),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg *tensor_gs101_ufs_phy_cfgs[CFG_TAG_MAX] = {
	[CFG_PRE_INIT]		= tensor_gs101_pre_init_cfg,
	[CFG_PRE_PWR_HS]	= tensor_gs101_pre_pwr_hs_config,
	[CFG_POST_PWR_HS]	= tensor_gs101_post_pwr_hs_config,
};


static const struct samsung_ufs_phy_cfg *tensor_gs101_hibern8_cfgs[] = {
	[CFG_POST_HIBERN8_ENTER]	= tensor_gs101_post_h8_enter,
	[CFG_PRE_HIBERN8_EXIT]		= tensor_gs101_pre_h8_exit,
};

static const char * const tensor_gs101_ufs_phy_clks[] = {
	"ref_clk",
};

static int gs101_phy_wait_for_calibration(struct phy *phy, u8 lane)
{
	struct samsung_ufs_phy *ufs_phy = get_samsung_ufs_phy(phy);
	const unsigned int timeout_us = 40000;
	const unsigned int sleep_us = 40;
	u32 val;
	u32 off;
	int err;

	off = PHY_PMA_TRSV_ADDR(TRSV_REG338, lane);

	err = readl_poll_timeout(ufs_phy->reg_pma + off,
				 val, (val & LN0_MON_RX_CAL_DONE),
				 sleep_us, timeout_us);

	if (err) {
		dev_err(ufs_phy->dev,
			"failed to get phy cal done %d\n", err);
	}

	return err;
}

#define DELAY_IN_US	40
#define RETRY_CNT	100
#define CDR_INSTR_DISTINCT_MAX 8
static int gs101_phy_wait_for_cdr_lock(struct phy *phy, u8 lane)
{
	struct samsung_ufs_phy *ufs_phy = get_samsung_ufs_phy(phy);
	u32 val;
	int i, k;
	u32 val_first = 0xDEADBEEF, val_last = 0xDEADBEEF;
	u32 ever_set = 0, ever_clear = 0;
	u32 distinct[CDR_INSTR_DISTINCT_MAX] = { 0 };
	int distinct_iter[CDR_INSTR_DISTINCT_MAX];
	int distinct_n = 0;
	u32 cal_done_338, cdr_ovrd_222_in;
	u32 reg336_in, reg337_in, reg33a_in, reg33b_in;
	u32 cdr_ovrd_222_out;
	u32 reg336_out, reg337_out, reg33a_out, reg33b_out;

	for (k = 0; k < CDR_INSTR_DISTINCT_MAX; k++)
		distinct_iter[k] = -1;

	/* Snapshot related TRSV regs at entry to bound the search */
	cal_done_338 = readl(ufs_phy->reg_pma +
			     PHY_PMA_TRSV_ADDR(TRSV_REG338, lane));
	cdr_ovrd_222_in = readl(ufs_phy->reg_pma +
				PHY_PMA_TRSV_ADDR(TRSV_REG222, lane));
	reg336_in = readl(ufs_phy->reg_pma + PHY_PMA_TRSV_ADDR(0x336, lane));
	reg337_in = readl(ufs_phy->reg_pma + PHY_PMA_TRSV_ADDR(0x337, lane));
	reg33a_in = readl(ufs_phy->reg_pma + PHY_PMA_TRSV_ADDR(0x33A, lane));
	reg33b_in = readl(ufs_phy->reg_pma + PHY_PMA_TRSV_ADDR(0x33B, lane));

	dev_info(ufs_phy->dev,
		 "cdr-instr lane=%u entry: R338(CAL_DONE)=0x%08x R222(OVRD)=0x%08x R336=0x%08x R337=0x%08x R33A=0x%08x R33B=0x%08x\n",
		 lane, cal_done_338, cdr_ovrd_222_in,
		 reg336_in, reg337_in, reg33a_in, reg33b_in);

	for (i = 0; i < RETRY_CNT; i++) {
		udelay(DELAY_IN_US);
		val = readl(ufs_phy->reg_pma +
			    PHY_PMA_TRSV_ADDR(TRSV_REG339, lane));

		if (i == 0)
			val_first = val;
		val_last = val;

		ever_set |= val;
		ever_clear |= ~val;

		{
			int seen = 0;
			for (k = 0; k < distinct_n; k++) {
				if (distinct[k] == val) {
					seen = 1;
					break;
				}
			}
			if (!seen && distinct_n < CDR_INSTR_DISTINCT_MAX) {
				distinct[distinct_n] = val;
				distinct_iter[distinct_n] = i;
				distinct_n++;
			}
		}

		if (val & LN0_MON_RX_CDR_FLD_CK_MODE_DONE) {
			dev_info(ufs_phy->dev,
				 "cdr lock OK lane=%u after %d iters first=0x%08x last=0x%08x\n",
				 lane, i + 1, val_first, val);
			return 0;
		}

		udelay(DELAY_IN_US);
		/* Override and enable clock data recovery */
		writel(LN0_OVRD_RX_CDR_EN, ufs_phy->reg_pma +
		       PHY_PMA_TRSV_ADDR(TRSV_REG222, lane));
		writel(LN0_OVRD_RX_CDR_EN | LN0_RX_CDR_EN,
		       ufs_phy->reg_pma + PHY_PMA_TRSV_ADDR(TRSV_REG222, lane));
	}

	cdr_ovrd_222_out = readl(ufs_phy->reg_pma +
				 PHY_PMA_TRSV_ADDR(TRSV_REG222, lane));
	reg336_out = readl(ufs_phy->reg_pma + PHY_PMA_TRSV_ADDR(0x336, lane));
	reg337_out = readl(ufs_phy->reg_pma + PHY_PMA_TRSV_ADDR(0x337, lane));
	reg33a_out = readl(ufs_phy->reg_pma + PHY_PMA_TRSV_ADDR(0x33A, lane));
	reg33b_out = readl(ufs_phy->reg_pma + PHY_PMA_TRSV_ADDR(0x33B, lane));

	dev_err(ufs_phy->dev,
		"failed to get cdr lock (lane=%u, TRSV_REG339 first=0x%08x last=0x%08x; bit3=LN0_MON_RX_CDR_FLD_CK_MODE_DONE)\n",
		lane, val_first, val_last);
	dev_err(ufs_phy->dev,
		"cdr-instr lane=%u summary: ever_set=0x%08x ever_clear=0x%08x distinct_n=%d\n",
		lane, ever_set, ever_clear, distinct_n);
	for (k = 0; k < distinct_n; k++)
		dev_err(ufs_phy->dev,
			"cdr-instr lane=%u: distinct[%d]=0x%08x first_seen_iter=%d\n",
			lane, k, distinct[k], distinct_iter[k]);
	dev_err(ufs_phy->dev,
		"cdr-instr lane=%u exit: R222(OVRD)=0x%08x R336=0x%08x R337=0x%08x R33A=0x%08x R33B=0x%08x\n",
		lane, cdr_ovrd_222_out,
		reg336_out, reg337_out, reg33a_out, reg33b_out);
	return -ETIMEDOUT;
}

const struct samsung_ufs_phy_drvdata tensor_gs101_ufs_phy = {
	.cfgs = tensor_gs101_ufs_phy_cfgs,
	.cfgs_hibern8 = tensor_gs101_hibern8_cfgs,
	.isol = {
		.offset = TENSOR_GS101_PHY_CTRL,
		.mask = TENSOR_GS101_PHY_CTRL_MASK,
		.en = TENSOR_GS101_PHY_CTRL_EN,
	},
	.clk_list = tensor_gs101_ufs_phy_clks,
	.num_clks = ARRAY_SIZE(tensor_gs101_ufs_phy_clks),
	.wait_for_cal = gs101_phy_wait_for_calibration,
	.wait_for_cdr = gs101_phy_wait_for_cdr_lock,
};

/*
 * GS201 (Tensor G2) UFS PHY. Same register layout as GS101. The PMU isolation
 * register at TENSOR_GS101_PHY_CTRL (PMU_ALIVE+0x3ec8) is direct-MMIO blocked
 * on gs201 (writes SError) but is on the BL31 SMC allowlist for
 * TENSOR_SMC_PMU_SEC_REG (verified by smc-probe). With pmu_alive's compat set
 * to just "google,gs201-pmu" (no "syscon" fallback), exynos-pmu installs an
 * SMC-routed regmap that handles the write transparently.
 */
const struct samsung_ufs_phy_drvdata tensor_gs201_ufs_phy = {
	.cfgs = tensor_gs101_ufs_phy_cfgs,
	.cfgs_hibern8 = tensor_gs101_hibern8_cfgs,
	.isol = {
		.offset = TENSOR_GS101_PHY_CTRL,
		.mask = TENSOR_GS101_PHY_CTRL_MASK,
		.en = TENSOR_GS101_PHY_CTRL_EN,
	},
	.clk_list = tensor_gs101_ufs_phy_clks,
	.num_clks = ARRAY_SIZE(tensor_gs101_ufs_phy_clks),
	.wait_for_cal = gs101_phy_wait_for_calibration,
	.wait_for_cdr = gs101_phy_wait_for_cdr_lock,
};
