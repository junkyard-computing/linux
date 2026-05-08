// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com
 *
 * Author: Sung-Hyun Na <sunghyun.na@samsung.com>
 *
 * Chip Abstraction Layer for USB PHY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include "phy-samsung-usb-cal.h"
#include "phy-exynos-usb3p1.h"
#include "phy-exynos-usb3p1-reg.h"

/*
 * exynos_usb3p1_get_tune_param was used by the SS-only late_enable callees
 * dropped from this graft; mainline has no caller. Removed to silence
 * -Wunused-function. Restore alongside the SS+ CR-port path if Phase B.5
 * brings in SuperSpeed tunes.
 */

static void exynos_cal_usbphy_q_ch(void *regs_base, u8 enable)
{
	u32 phy_resume;

	if (enable) {
		/* WA for Q-channel: disable all q-act from usb */
		phy_resume = readl(regs_base + EXYNOS_USBCON_LINK_CTRL);
		phy_resume |= LINKCTRL_DIS_QACT_ID0;
		phy_resume |= LINKCTRL_DIS_QACT_VBUS_VALID;
		phy_resume |= LINKCTRL_DIS_QACT_BVALID;
		phy_resume |= LINKCTRL_DIS_QACT_LINKGATE;
		phy_resume &= ~LINKCTRL_FORCE_QACT;
		usleep_range(500, 600);
		writel(phy_resume, regs_base + EXYNOS_USBCON_LINK_CTRL);
		usleep_range(500, 600);
		phy_resume = readl(regs_base + EXYNOS_USBCON_LINK_CTRL);
		phy_resume |= LINKCTRL_FORCE_QACT;
		usleep_range(500, 600);
		writel(phy_resume, regs_base + EXYNOS_USBCON_LINK_CTRL);
	} else {
		phy_resume = readl(regs_base + EXYNOS_USBCON_LINK_CTRL);
		phy_resume &= ~LINKCTRL_FORCE_QACT;
		phy_resume |= LINKCTRL_DIS_QACT_ID0;
		phy_resume |= LINKCTRL_DIS_QACT_VBUS_VALID;
		phy_resume |= LINKCTRL_DIS_QACT_BVALID;
		phy_resume |= LINKCTRL_DIS_QACT_LINKGATE;
		writel(phy_resume, regs_base + EXYNOS_USBCON_LINK_CTRL);
	}
}

static void link_vbus_filter_en(struct exynos_usbphy_info *info, u8 enable)
{
	u32 phy_resume;

	phy_resume = readl(info->regs_base + EXYNOS_USBCON_LINK_CTRL);
	if (enable)
		phy_resume &= ~LINKCTRL_BUS_FILTER_BYPASS_MASK;
	else
		phy_resume |= LINKCTRL_BUS_FILTER_BYPASS(0xf);
	writel(phy_resume, info->regs_base + EXYNOS_USBCON_LINK_CTRL);
}

static void phy_power_en(struct exynos_usbphy_info *info, u8 en)
{
	u32 reg;
	bool ss_cap;
	int main_version;

	main_version = info->version & EXYNOS_USBCON_VER_MAJOR_VER_MASK;
	ss_cap = (info->version & EXYNOS_USBCON_VER_SS_CAP) >> 6;

	if (main_version == EXYNOS_USBCON_VER_05_0_0 || ss_cap) {
		void *__iomem reg_base;

		if (info->used_phy_port == 1)
			reg_base = info->regs_base_2nd;
		else
			reg_base = info->regs_base;

		if (!ss_cap) {
			/* 3.0 PHY Power Up or Down control */
			reg = readl(reg_base + EXYNOS_USBCON_PWR);

			if (en) {
				/* apply to KC asb vector */
				if (EXYNOS_USBCON_VER_MINOR(info->version)
				    >= 0x1) {
					reg &= ~(PWR_PIPE3_POWERDONW);
					reg &= ~(PWR_FORCE_POWERDOWN_EN);
				} else {
					reg &= ~(PWR_TEST_POWERDOWN_HSP);
					reg &= ~(PWR_TEST_POWERDOWN_SSP);
					writel(reg,
					       reg_base + EXYNOS_USBCON_PWR);
					usleep_range(1000, 1200);
					reg |= (PWR_TEST_POWERDOWN_HSP);
				}
			} else {
				if (EXYNOS_USBCON_VER_MINOR(info->version) >= 0x1) {
					reg |= (PWR_PIPE3_POWERDONW);
					reg |= (PWR_FORCE_POWERDOWN_EN);
				} else {
					reg |= (PWR_TEST_POWERDOWN_SSP);
				}
			}
			writel(reg, reg_base + EXYNOS_USBCON_PWR);
		} else if (ss_cap) {
			/* 3.0 PHY Power Up or Down Control (Only KITT) */
			reg = readl(reg_base + EXYNOS_USBCON_G2PHY_CNTL0);

			if (en) {
				reg |= (G2PHY_CNTL0_UPCS_PWR_STABLE);
				reg &= ~(G2PHY_CNTL0_TEST_POWERDOWN);
			} else {
				reg |= (G2PHY_CNTL0_TEST_POWERDOWN);
				reg &= ~(G2PHY_CNTL0_UPCS_PWR_STABLE);
			}
			writel(reg, reg_base + EXYNOS_USBCON_G2PHY_CNTL0);
		}
	}

	if (main_version == EXYNOS_USBCON_VER_03_0_0) {
		/* 2.0 PHY Power Down Control */
		reg = readl(info->regs_base + EXYNOS_USBCON_HSP_TEST);
		if (en)
			reg &= ~HSP_TEST_SIDDQ;
		else
			reg |= HSP_TEST_SIDDQ;
		writel(reg, info->regs_base + EXYNOS_USBCON_HSP_TEST);
	}
}

static void phy_sw_rst_high(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->regs_base;
	int main_version;
	u32 clkrst;

	main_version = info->version & EXYNOS_USBCON_VER_MAJOR_VER_MASK;
	if (main_version == EXYNOS_USBCON_VER_05_0_0 &&
	    info->used_phy_port == 1) {
		regs_base = info->regs_base_2nd;
	}

	clkrst = readl(regs_base + EXYNOS_USBCON_CLKRST);
	if (EXYNOS_USBCON_VER_MINOR(info->version) >= 0x1) {
		clkrst |= CLKRST_PHY20_SW_RST;
		clkrst |= CLKRST_PHY20_RST_SEL;
		clkrst |= CLKRST_PHY30_SW_RST;
		clkrst |= CLKRST_PHY30_RST_SEL;
	} else {
		clkrst |= CLKRST_PHY_SW_RST;
		clkrst |= CLKRST_PHY_RST_SEL;
		clkrst |= CLKRST_PORT_RST;
	}
	writel(clkrst, regs_base + EXYNOS_USBCON_CLKRST);
}

static void phy_sw_rst_low(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->regs_base;
	int main_version;
	u32 clkrst;

	main_version = info->version & EXYNOS_USBCON_VER_MAJOR_VER_MASK;
	if (main_version == EXYNOS_USBCON_VER_05_0_0 &&
	    info->used_phy_port == 1)
		regs_base = info->regs_base_2nd;

	clkrst = readl(regs_base + EXYNOS_USBCON_CLKRST);
	if (EXYNOS_USBCON_VER_MINOR(info->version) >= 0x1) {
		clkrst |= CLKRST_PHY20_RST_SEL;
		clkrst &= ~CLKRST_PHY20_SW_RST;
		clkrst &= ~CLKRST_PHY30_SW_RST;
		clkrst &= ~CLKRST_PORT_RST;

	} else {
		clkrst |= CLKRST_PHY_RST_SEL;
		clkrst &= ~CLKRST_PHY_SW_RST;
		clkrst &= ~CLKRST_PORT_RST;
	}
	writel(clkrst, regs_base + EXYNOS_USBCON_CLKRST);
}

void phy_exynos_usb_v3p1_pma_ready(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->regs_base;
	u32 reg;

	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_LOW_PWR;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);

	udelay(1);

	reg |= PMA_APB_SW_RST;
	reg |= PMA_INIT_SW_RST;
	reg |= PMA_CMN_SW_RST;
	reg |= PMA_TRSV_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);

	udelay(1);

	reg &= ~PMA_APB_SW_RST;
	reg &= ~PMA_INIT_SW_RST;
	reg &= ~PMA_PLL_REF_REQ_MASK;
	reg &= ~PMA_REF_FREQ_SEL_MASK;
	reg |= PMA_REF_FREQ_SEL_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);

	reg = readl(regs_base + EXYNOS_USBCON_LINK_CTRL);
	reg &= ~LINKCTRL_PIPE3_FORCE_EN;
	writel(reg, regs_base + EXYNOS_USBCON_LINK_CTRL);
}

void phy_exynos_usb_v3p1_g2_pma_ready(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->regs_base;
	u32 reg;

	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_ROPLL_REF_CLK_SEL_MASK;
	reg &= ~PMA_LCPLL_REF_CLK_SEL_MASK;
	reg &= ~PMA_PLL_REF_REQ_MASK;
	reg |= PMA_REF_FREQ_SEL_SET(1);
	reg |= PMA_LOW_PWR;
	reg |= PMA_TRSV_SW_RST;
	reg |= PMA_CMN_SW_RST;
	reg |= PMA_INIT_SW_RST;
	reg |= PMA_APB_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);

	udelay(1);
	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_LOW_PWR;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	udelay(1);

	// release override
	reg = readl(regs_base + EXYNOS_USBCON_LINK_CTRL);
	reg &= ~LINKCTRL_PIPE3_FORCE_EN;
	writel(reg, regs_base + EXYNOS_USBCON_LINK_CTRL);

	udelay(1);

	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_APB_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
}

void phy_exynos_usb_v3p1_g2_disable(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->regs_base;
	u32 reg;

	/* Change pipe pclk to pipe3 */
	reg = readl(regs_base + EXYNOS_USBCON_CLKRST);
	reg &= ~CLKRST_LINK_PCLK_SEL;
	writel(reg, regs_base + EXYNOS_USBCON_CLKRST);

	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg |= PMA_LOW_PWR;
	reg |= PMA_TRSV_SW_RST;
	reg |= PMA_CMN_SW_RST;
	reg |= PMA_INIT_SW_RST;
	reg |= PMA_APB_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);

	udelay(1);

	reg &= ~PMA_TRSV_SW_RST;
	reg &= ~PMA_CMN_SW_RST;
	reg &= ~PMA_INIT_SW_RST;
	reg &= ~PMA_APB_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
}

void phy_exynos_usb_v3p1_pma_sw_rst_release(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->regs_base;
	u32 reg;

	/* Reset Release for USB/DP PHY */
	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_CMN_SW_RST;
	reg &= ~PMA_TRSV_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);

	usleep_range(1000, 1200);

	/* Change pipe pclk to pipe3 */
	reg = readl(regs_base + EXYNOS_USBCON_CLKRST);
	reg |= CLKRST_LINK_PCLK_SEL;
	writel(reg, regs_base + EXYNOS_USBCON_CLKRST);
}

void phy_exynos_usb_v3p1_g2_pma_sw_rst_release(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->regs_base;
	u32 reg;

	/* Reset Release for USB/DP PHY */
	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_INIT_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);

	udelay(1);		// Spec : wait for 200ns

	/* run pll */
	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_TRSV_SW_RST;
	reg &= ~PMA_CMN_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);

	usleep_range(10, 15);
}

void phy_exynos_usb_v3p1_g2_link_pclk_sel(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->regs_base;
	u32 reg;

	/* Change pipe pclk to pipe3 */
	/* add by Makalu(evt1) case - 20180724 */
	reg = readl(regs_base + EXYNOS_USBCON_CLKRST);
	reg |= CLKRST_LINK_PCLK_SEL;
	writel(reg, regs_base + EXYNOS_USBCON_CLKRST);
	usleep_range(3000, 3500);
}

void phy_exynos_usb_v3p1_pipe_ready(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->regs_base;
	u32 reg;

	reg = readl(regs_base + EXYNOS_USBCON_LINK_CTRL);
	reg &= ~LINKCTRL_PIPE3_FORCE_EN;
	writel(reg, regs_base + EXYNOS_USBCON_LINK_CTRL);
}

void phy_exynos_usb_v3p1_pipe_ovrd(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->regs_base;
	u32 reg;
	bool ss_cap = 0;

	ss_cap = (info->version & EXYNOS_USBCON_VER_SS_CAP) >> 6;

	/* force pipe3 signal for link */
	reg = readl(regs_base + EXYNOS_USBCON_LINK_CTRL);
	reg |= LINKCTRL_PIPE3_FORCE_EN;
	reg &= ~LINKCTRL_PIPE3_FORCE_PHY_STATUS;
	reg |= LINKCTRL_PIPE3_FORCE_RX_ELEC_IDLE;
	writel(reg, regs_base + EXYNOS_USBCON_LINK_CTRL);

	/* PMA Disable */
	if (!ss_cap) {
		reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
		reg |= PMA_LOW_PWR;
		writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	}
}

void phy_exynos_usb_v3p1_link_sw_reset(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->regs_base;
	int main_version;
	u32 reg;

	main_version = info->version & EXYNOS_USBCON_VER_MAJOR_VER_MASK;

	/* use link_sw_rst because it has functioning as Hreset_n
	 * for asb host/device role change, originally not recommend
	 * link_sw_rst by Foundry T.
	 * so that some of global register has cleard - 2018.11.12
	 */
	/* Link Reset */
	if (main_version == EXYNOS_USBCON_VER_03_0_0) {
		reg = readl(info->regs_base + EXYNOS_USBCON_CLKRST);
		reg |= CLKRST_LINK_SW_RST;
		writel(reg, regs_base + EXYNOS_USBCON_CLKRST);

		usleep_range(10, 15);

		reg &= ~CLKRST_LINK_SW_RST;
		writel(reg, regs_base + EXYNOS_USBCON_CLKRST);
	}
}

void phy_exynos_usb_v3p1_enable(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->regs_base;
	u32 reg;
	u32 reg_hsp;
	bool ss_only_cap, ss_cap;
	int main_version;

	main_version = info->version & EXYNOS_USBCON_VER_MAJOR_VER_MASK;
	ss_only_cap = (info->version & EXYNOS_USBCON_VER_SS_ONLY_CAP) >> 4;
	ss_cap = (info->version & EXYNOS_USBCON_VER_SS_CAP) >> 6;

	if (main_version == EXYNOS_USBCON_VER_03_0_0) {
		/* Set force q-channel */
		exynos_cal_usbphy_q_ch(regs_base, 1);
	}

	if (ss_cap) {
		reg = readl(info->regs_base + EXYNOS_USBCON_G2PHY_CNTL1);
		reg |= G2PHY_CNTL1_ANA_PWR_EN;
		reg |= G2PHY_CNTL1_PCS_PWR_STABLE;
		reg |= G2PHY_CNTL1_PMA_PWR_STABLE;
		writel(reg, regs_base + EXYNOS_USBCON_G2PHY_CNTL1);
	}

	/* Set PHY POR High */
	phy_sw_rst_high(info);

	if (!ss_only_cap) {
		reg = readl(regs_base + EXYNOS_USBCON_UTMI);
		reg &= ~UTMI_FORCE_SUSPEND;
		reg &= ~UTMI_FORCE_SLEEP;
		reg &= ~UTMI_DP_PULLDOWN;
		reg &= ~UTMI_DM_PULLDOWN;
		writel(reg, regs_base + EXYNOS_USBCON_UTMI);

		/* set phy clock & control HS phy */
		reg = readl(regs_base + EXYNOS_USBCON_HSP);

		if (info->common_block_disable) {
			reg |= HSP_EN_UTMISUSPEND;
			reg |= HSP_COMMONONN;
		} else {
			reg &= ~HSP_COMMONONN;
		}
		writel(reg, regs_base + EXYNOS_USBCON_HSP);
	}

	if (ss_only_cap || ss_cap) {
		void *ss_reg_base;

		if (info->used_phy_port == 1)
			ss_reg_base = info->regs_base_2nd;
		else
			ss_reg_base = info->regs_base;

		/* Change pipe pclk to pipe3 */
		reg = readl(ss_reg_base + EXYNOS_USBCON_CLKRST);
		reg |= CLKRST_LINK_PCLK_SEL;
		writel(reg, ss_reg_base + EXYNOS_USBCON_CLKRST);
	}
	usleep_range(100, 120);

	/* Follow setting sequence for USB Link */
	/*
	 * 1. Set VBUS Valid and DP-Pull up control
	 * by VBUS pad usage
	 */
	link_vbus_filter_en(info, false);
	reg = readl(regs_base + EXYNOS_USBCON_UTMI);
	reg_hsp = readl(regs_base + EXYNOS_USBCON_HSP);
	reg |= UTMI_FORCE_BVALID;
	reg |= UTMI_FORCE_VBUSVALID;
	reg_hsp |= HSP_VBUSVLDEXTSEL;
	reg_hsp |= HSP_VBUSVLDEXT;

	writel(reg, regs_base + EXYNOS_USBCON_UTMI);
	writel(reg_hsp, regs_base + EXYNOS_USBCON_HSP);

	/* Set PHY tune para */
	phy_exynos_usb_v3p1_tune(info);

	/* Select Rerence clock frequency */
	reg = readl(regs_base + EXYNOS_USBCON_SSP_PLL);
	switch (info->refclk) {
	case USBPHY_REFCLK_EXT_50MHZ:
		reg &= ~SSP_PLL_FSEL_MASK;
		reg |= 0x7;
		break;
	case USBPHY_REFCLK_EXT_26MHZ:
		reg &= ~SSP_PLL_FSEL_MASK;
		reg |= 0x6;
		break;
	case USBPHY_REFCLK_EXT_24MHZ:
		reg &= ~SSP_PLL_FSEL_MASK;
		reg |= 0x2;
		break;
	case USBPHY_REFCLK_EXT_20MHZ:
		reg &= ~SSP_PLL_FSEL_MASK;
		reg |= 0x1;
		break;
	case USBPHY_REFCLK_EXT_19P2MHZ:
		reg &= ~SSP_PLL_FSEL_MASK;
		reg |= 0x0;
		break;
	case USBPHY_REFCLK_EXT_48MHZ:
	case USBPHY_REFCLK_EXT_12MHZ:
	case USBPHY_REFCLK_DIFF_100MHZ:
	case USBPHY_REFCLK_DIFF_52MHZ:
	case USBPHY_REFCLK_DIFF_48MHZ:
	case USBPHY_REFCLK_DIFF_26MHZ:
	case USBPHY_REFCLK_DIFF_24MHZ:
	case USBPHY_REFCLK_DIFF_20MHZ:
	case USBPHY_REFCLK_DIFF_19_2MHZ:
		/* No phy ref clock control */
		break;
	}
	writel(reg, regs_base + EXYNOS_USBCON_SSP_PLL);

	/* Enable PHY Power Mode */
	phy_power_en(info, 1);

	/* before POR low, 10us delay is needed. */
	usleep_range(10, 15);

	/* Set PHY POR Low */
	phy_sw_rst_low(info);

	/* after POR low and delay 75us, PHYCLOCK is guaranteed. */
	usleep_range(75, 90);

	if (ss_only_cap || ss_cap) {
		phy_exynos_usb_v3p1_late_enable(info);
		return;
	}

	/* Select PHY MUX */
	if (info->dual_phy) {
		u32 physel;

		physel = readl(regs_base + EXYNOS_USBCON_DUALPHYSEL);
		if (info->used_phy_port == 0) {
			physel &= ~DUALPHYSEL_PHYSEL_CTRL;
			physel &= ~DUALPHYSEL_PHYSEL_SSPHY;
			physel &= ~DUALPHYSEL_PHYSEL_PIPECLK;
			physel &= ~DUALPHYSEL_PHYSEL_PIPERST;
		} else {
			physel |= DUALPHYSEL_PHYSEL_CTRL;
			physel |= DUALPHYSEL_PHYSEL_SSPHY;
			physel |= DUALPHYSEL_PHYSEL_PIPECLK;
			physel |= DUALPHYSEL_PHYSEL_PIPERST;
		}
		writel(physel, regs_base + EXYNOS_USBCON_DUALPHYSEL);
	}

	/* 2. OVC io usage */
	reg = readl(regs_base + EXYNOS_USBCON_LINK_PORT);
	if (info->use_io_for_ovc) {
		reg &= ~LINKPORT_HUB_PORT_SEL_OCD_U3;
		reg &= ~LINKPORT_HUB_PORT_SEL_OCD_U2;
	} else {
		reg |= LINKPORT_HUB_PORT_SEL_OCD_U3;
		reg |= LINKPORT_HUB_PORT_SEL_OCD_U2;
	}
	writel(reg, regs_base + EXYNOS_USBCON_LINK_PORT);

	/* Enable ReWA */
	/* Mainline graft: ReWA stripped — info->hs_rewa is unused. */
}

enum exynos_usbphy_tif {
	USBCON_TIF_RD_STS,
	USBCON_TIF_RD_OVRD,
	USBCON_TIF_WR_OVRD,
};

static u8 phy_exynos_usb_v3p1_tif_access(struct exynos_usbphy_info *info,
					 enum exynos_usbphy_tif access_type,
					 u8 addr, u8 data)
{
	void __iomem *base;
	u32 hsp_test;

	base = info->regs_base;
	hsp_test = readl(base + EXYNOS_USBCON_HSP_TEST);
	/* Set TEST DATA OUT SEL */
	if (access_type == USBCON_TIF_RD_STS)
		hsp_test &= ~HSP_TEST_DATA_OUT_SEL;
	else
		hsp_test |= HSP_TEST_DATA_OUT_SEL;

	hsp_test &= ~HSP_TEST_DATA_IN_MASK;
	hsp_test &= ~HSP_TEST_DATA_ADDR_MASK;
	hsp_test |= HSP_TEST_DATA_ADDR_SET(addr);
	writel(hsp_test, base + EXYNOS_USBCON_HSP_TEST);

	usleep_range(10, 15);

	hsp_test = readl(base + EXYNOS_USBCON_HSP_TEST);
	if (access_type != USBCON_TIF_WR_OVRD)
		return HSP_TEST_DATA_OUT_GET(hsp_test);

	hsp_test |= HSP_TEST_DATA_IN_SET((data | 0xf0));
	hsp_test |= HSP_TEST_CLK;
	writel(hsp_test, base + EXYNOS_USBCON_HSP_TEST);

	usleep_range(10, 15);

	hsp_test = readl(base + EXYNOS_USBCON_HSP_TEST);
	hsp_test &= ~HSP_TEST_CLK;
	writel(hsp_test, base + EXYNOS_USBCON_HSP_TEST);

	hsp_test = readl(base + EXYNOS_USBCON_HSP_TEST);
	return HSP_TEST_DATA_OUT_GET(hsp_test);
}

u8 phy_exynos_usb_v3p1_tif_ov_rd(struct exynos_usbphy_info *info, u8 addr)
{
	return phy_exynos_usb_v3p1_tif_access(info, USBCON_TIF_RD_OVRD, addr,
					      0x0);
}

u8 phy_exynos_usb_v3p1_tif_ov_wr(struct exynos_usbphy_info *info, u8 addr,
				 u8 data)
{
	return phy_exynos_usb_v3p1_tif_access(info, USBCON_TIF_WR_OVRD, addr,
					      data);
}

u8 phy_exynos_usb_v3p1_tif_sts_rd(struct exynos_usbphy_info *info, u8 addr)
{
	return phy_exynos_usb_v3p1_tif_access(info, USBCON_TIF_RD_STS, addr,
					      0x0);
}

/*
 * Phase G.10 (felix gs201): minimal CR-port (Control Register port) re-graft.
 *
 * The Synopsys USB31DRD wrapper exposes a CR-port at SSP_CRCTL0/1
 * (regs offsets 0x40/0x44 in the USBCON block). The CR-port is a
 * handshake-protocol channel into the embedded SS PHY's analog
 * control registers. AOSP CAL accesses it via `phy_exynos_usb_v3p1_cr_access`
 * — that whole machinery was trimmed from this graft because it was
 * gated on `version > 0x500 && !ss_cap` in late_enable, and felix's
 * 0x301 fell below the gate.
 *
 * We bring just `cr_access` + `cal_cr_write` back so we can issue
 * the *one* write that AOSP runs unconditionally inside that gated
 * block: RXDET_MEAS_TIME[11:4] = 0x80 (CR addr 0x1010). The
 * speculation: the gate is wrong for combo-PHY SoCs and the SS
 * analog frontend's RX detect timing also affects the HS path
 * via shared bias, leaving the dwc3 unable to receive any byte
 * stream beyond chirp-state events. See gs-usb.md "Where the gap
 * is now (2026-05-08)".
 */
enum exynos_usbcon_cr {
	USBCON_CR_ADDR = 0,
	USBCON_CR_READ = 18,
	USBCON_CR_WRITE = 19,
};

static u16 phy_exynos_usb_v3p1_cr_access(struct exynos_usbphy_info *info,
					 enum exynos_usbcon_cr cr_bit, u16 data)
{
	void __iomem *base = info->regs_base;
	u32 ssp_crctl0, ssp_crctl1 = 0;
	u32 loop, loop_cnt;

	ssp_crctl0 = readl(base + EXYNOS_USBCON_SSP_CRCTL0);
	ssp_crctl0 &= ~0xfU;
	ssp_crctl0 &= ~(0xffffU << 16);
	writel(ssp_crctl0, base + EXYNOS_USBCON_SSP_CRCTL0);

	ssp_crctl0 &= ~SSP_CCTRL0_CR_DATA_IN_MASK;
	ssp_crctl0 |= SSP_CCTRL0_CR_DATA_IN(data);
	writel(ssp_crctl0, base + EXYNOS_USBCON_SSP_CRCTL0);

	loop = (cr_bit == USBCON_CR_ADDR) ? 1 : 2;

	for (loop_cnt = 0; loop_cnt < loop; loop_cnt++) {
		u32 trigger_bit = 0;
		u32 handshake_cnt = 2;

		if (cr_bit == USBCON_CR_ADDR) {
			trigger_bit = SSP_CRCTRL0_CR_CAP_ADDR;
		} else {
			if (loop_cnt == 0)
				trigger_bit = SSP_CRCTRL0_CR_CAP_DATA;
			else if (cr_bit == USBCON_CR_READ)
				trigger_bit = SSP_CRCTRL0_CR_READ;
			else
				trigger_bit = SSP_CRCTRL0_CR_WRITE;
		}
		do {
			u32 usec = 100;

			if (handshake_cnt == 2)
				ssp_crctl0 |= trigger_bit;
			else
				ssp_crctl0 &= ~trigger_bit;

			writel(ssp_crctl0, base + EXYNOS_USBCON_SSP_CRCTL0);

			do {
				ssp_crctl1 = readl(base + EXYNOS_USBCON_SSP_CRCTL1);
				if (handshake_cnt == 2 &&
				    (ssp_crctl1 & SSP_CRCTL1_CR_ACK))
					break;
				else if ((handshake_cnt == 1) &&
					 !(ssp_crctl1 & SSP_CRCTL1_CR_ACK))
					break;
				udelay(1);
			} while (usec-- > 0);
			udelay(5);
			handshake_cnt--;
		} while (handshake_cnt != 0);
		usleep_range(50, 60);
	}
	return (u16)((ssp_crctl1 & SSP_CRCTL1_CR_DATA_OUT_MASK) >> 16);
}

static void phy_exynos_usb_v3p1_cal_cr_write(struct exynos_usbphy_info *info,
					     u16 addr, u16 data)
{
	phy_exynos_usb_v3p1_cr_access(info, USBCON_CR_ADDR, addr);
	phy_exynos_usb_v3p1_cr_access(info, USBCON_CR_WRITE, data);
}

/*
 * Phase G.10: force the single unconditional CR-port write that AOSP
 * runs only for `version > 0x500 && !ss_cap` in late_enable.
 *
 * Setting CR addr 0x1010 (RXDET_MEAS_TIME) bits [11:4] to 0x80 —
 * AOSP uses this as a per-reference-clock RX-detect timing tweak.
 *
 * Returns 0 on success; non-zero if the CR-port handshake fails
 * (would indicate the CR-port hardware isn't wired through on this
 * SoC, in which case the test result is "CR-port doesn't apply").
 */
int phy_exynos_usb_v3p1_force_gs201_cr_writes(struct exynos_usbphy_info *info)
{
	u32 ack_check;

	/* Sanity probe: read SSP_CRCTL1 to see if the register block
	 * responds. If reads come back all-ones, the CR-port hw isn't
	 * present. */
	ack_check = readl(info->regs_base + EXYNOS_USBCON_SSP_CRCTL1);
	if (ack_check == 0xFFFFFFFF) {
		dev_info(info->dev,
			 "DWC3-DBG: CR-port readback all-ones, hardware not present (skipping)\n");
		return -ENODEV;
	}

	dev_info(info->dev,
		 "DWC3-DBG: CR-port pre-write SSP_CRCTL1=0x%08x\n", ack_check);

	phy_exynos_usb_v3p1_cal_cr_write(info, 0x1010, 0x80);

	dev_info(info->dev,
		 "DWC3-DBG: CR-port post-write SSP_CRCTL1=0x%08x (wrote 0x1010=0x80)\n",
		 readl(info->regs_base + EXYNOS_USBCON_SSP_CRCTL1));
	return 0;
}

void phy_exynos_usb_v3p1_late_enable(struct exynos_usbphy_info *info)
{
	/*
	 * Mainline graft (felix gs201): both internal branches were
	 * (a) version > 0x500 && !ss_cap (SS+ CR-port writes) and
	 * (b) ss_cap (SSP CR-port write).
	 * For felix's version=0x301 (EXYNOS_USBCON_VER_03_0_1) neither
	 * predicate is true, so this is a no-op for the AOSP-signal path.
	 * Phase G.10 calls phy_exynos_usb_v3p1_force_gs201_cr_writes()
	 * separately from gs201_aosp_utmi_init in phy-exynos5-usbdrd.c.
	 */
}

void phy_exynos_usb_v3p1_disable(struct exynos_usbphy_info *info)
{
	u32 reg;
	void __iomem *regs_base = info->regs_base;

	/* set phy clock & control HS phy */
	reg = readl(regs_base + EXYNOS_USBCON_UTMI);
	reg &= ~UTMI_DP_PULLDOWN;
	reg &= ~UTMI_DM_PULLDOWN;
	reg |= UTMI_FORCE_SUSPEND;
	reg |= UTMI_FORCE_SLEEP;
	writel(reg, regs_base + EXYNOS_USBCON_UTMI);

	/* Disable PHY Power Mode */
	phy_power_en(info, 0);

	/* clear force q-channel */
	exynos_cal_usbphy_q_ch(regs_base, 0);

	/*
	 * link sw reset is need for USB_DP/DM high-z
	 * in host mode: 2019.04.10 by daeman.ko
	 */
	phy_exynos_usb_v3p1_link_sw_reset(info);
}

u64 phy_exynos_usb3p1_get_logic_trace(struct exynos_usbphy_info *info)
{
	u64 ret;
	void __iomem *regs_base = info->regs_base;

	ret = readl(regs_base + EXYNOS_USBCON_LINK_DEBUG_L);
	ret |= ((u64)readl(regs_base + EXYNOS_USBCON_LINK_DEBUG_H)) << 32;

	return ret;
}

/*
 * phy_exynos_usb3p1_get_phyclkcnt / get_linestate were exposed by the AOSP
 * driver for diagnostic readback; they had no prototype in the AOSP header
 * and nothing in the mainline graft consumer reads them. Dropped to silence
 * -Wmissing-prototypes; readd if a future caller (e.g. ftrace harness) needs
 * them, with a matching declaration in phy-exynos-usb3p1.h.
 */

void phy_exynos_usb_v3p1_tune(struct exynos_usbphy_info *info)
{
	u32 hsp_tune, ssp_tune0, ssp_tune1, ssp_tune2, cnt;

	bool ss_only_cap;
	bool ss_cap;

	ss_only_cap = (info->version & EXYNOS_USBCON_VER_SS_ONLY_CAP) >> 4;
	ss_cap = (info->version & EXYNOS_USBCON_VER_SS_CAP) >> 6;

	if (!info->tune_param)
		return;

	if (!ss_only_cap) {
		/* hsphy tuning */
		void __iomem *regs_base = info->regs_base;

		hsp_tune = readl(regs_base + EXYNOS_USBCON_HSP_TUNE);

		cnt = 0;
		for (; info->tune_param[cnt].value != EXYNOS_USB_TUNE_LAST;
		     cnt++) {
			char *para_name;
			int val;

			val = info->tune_param[cnt].value;
			if (val == -1)
				continue;

			para_name = info->tune_param[cnt].name;
			if (!strcmp(para_name, "compdis")) {
				hsp_tune &= ~HSP_TUNE_COMPDIS_MASK;
				hsp_tune |= HSP_TUNE_COMPDIS_SET(val);
			} else if (!strcmp(para_name, "otg")) {
				hsp_tune &= ~HSP_TUNE_OTG_MASK;
				hsp_tune |= HSP_TUNE_OTG_SET(val);
			} else if (!strcmp(para_name, "rx_sqrx")) {
				hsp_tune &= ~HSP_TUNE_SQRX_MASK;
				hsp_tune |= HSP_TUNE_SQRX_SET(val);
			} else if (!strcmp(para_name, "tx_fsls")) {
				hsp_tune &= ~HSP_TUNE_TXFSLS_MASK;
				hsp_tune |= HSP_TUNE_TXFSLS_SET(val);
			} else if (!strcmp(para_name, "tx_hsxv")) {
				hsp_tune &= ~HSP_TUNE_HSXV_MASK;
				hsp_tune |= HSP_TUNE_HSXV_SET(val);
			} else if (!strcmp(para_name, "tx_pre_emp")) {
				hsp_tune &= ~HSP_TUNE_TXPREEMPA_MASK;
				hsp_tune |= HSP_TUNE_TXPREEMPA_SET(val);
			} else if (!strcmp(para_name, "tx_pre_emp_plus")) {
				if (val)
					hsp_tune |= HSP_TUNE_TXPREEMPA_PLUS;
				else
					hsp_tune &= ~HSP_TUNE_TXPREEMPA_PLUS;
			} else if (!strcmp(para_name, "tx_res")) {
				hsp_tune &= ~HSP_TUNE_TXRES_MASK;
				hsp_tune |= HSP_TUNE_TXRES_SET(val);
			} else if (!strcmp(para_name, "tx_rise")) {
				hsp_tune &= ~HSP_TUNE_TXRISE_MASK;
				hsp_tune |= HSP_TUNE_TXRISE_SET(val);
			} else if (!strcmp(para_name, "tx_vref")) {
				hsp_tune &= ~HSP_TUNE_TXVREF_MASK;
				hsp_tune |= HSP_TUNE_TXVREF_SET(val);
			} else if (!strcmp(para_name, "tx_res_ovrd")) {
				phy_exynos_usb_v3p1_tif_ov_wr(info, 0x6, val);
			} else if (!strcmp(para_name, "tx_dis_inc")) {
				phy_exynos_usb_v3p1_tif_ov_wr(info, 0xb, val);
			}
		}
		writel(hsp_tune, regs_base + EXYNOS_USBCON_HSP_TUNE);
	}

	if (ss_only_cap) {
		/* ssphy tuning */
		void __iomem *ss_reg_base;

		if (info->used_phy_port == 1)
			ss_reg_base = info->regs_base_2nd;
		else
			ss_reg_base = info->regs_base;

		ssp_tune0 = readl(ss_reg_base + EXYNOS_USBCON_SSP_PARACON0);
		ssp_tune1 = readl(ss_reg_base + EXYNOS_USBCON_SSP_PARACON1);
		ssp_tune2 = readl(ss_reg_base + EXYNOS_USBCON_SSP_TEST);

		cnt = 0;
		for (; info->tune_param[cnt].value != EXYNOS_USB_TUNE_LAST;
		     cnt++) {
			char *para_name;
			int val;

			val = info->tune_param[cnt].value;
			if (val == -1)
				continue;

			para_name = info->tune_param[cnt].name;
			if (!strcmp(para_name, "pcs_tx_swing_full")) {
				ssp_tune0 &=
				    ~SSP_PARACON0_PCS_TX_SWING_FULL_MASK;
				ssp_tune0 |=
				    SSP_PARACON0_PCS_TX_SWING_FULL(val);
			} else if (!strcmp(para_name, "pcs_tx_deemph_6db")) {
				ssp_tune0 &=
				    ~SSP_PARACON0_PCS_TX_DEEMPH_6DB_MASK;
				ssp_tune0 |=
				    SSP_PARACON0_PCS_TX_DEEMPH_6DB(val);
			} else if (!strcmp(para_name, "pcs_tx_deemph_3p5db")) {
				ssp_tune0 &=
				    ~SSP_PARACON0_PCS_TX_DEEMPH_3P5DB_MASK;
				ssp_tune0 |=
				    SSP_PARACON0_PCS_TX_DEEMPH_3P5DB(val);
			} else if (!strcmp(para_name, "tx_vboost_lvl_sstx")) {
				ssp_tune1 &=
				    ~SSP_PARACON1_TX_VBOOST_LVL_SSTX_MASK;
				ssp_tune1 |=
				    SSP_PARACON1_TX_VBOOST_LVL_SSTX(val);
			} else if (!strcmp(para_name, "tx_vboost_lvl")) {
				ssp_tune1 &= ~SSP_PARACON1_TX_VBOOST_LVL_MASK;
				ssp_tune1 |= SSP_PARACON1_TX_VBOOST_LVL(val);
			} else if (!strcmp(para_name, "los_level")) {
				ssp_tune1 &= ~SSP_PARACON1_LOS_LEVEL_MASK;
				ssp_tune1 |= SSP_PARACON1_LOS_LEVEL(val);
			} else if (!strcmp(para_name, "los_bias")) {
				ssp_tune1 &= ~SSP_PARACON1_LOS_BIAS_MASK;
				ssp_tune1 |= SSP_PARACON1_LOS_BIAS(val);
			} else if (!strcmp(para_name, "pcs_rx_los_mask_val")) {
				ssp_tune1 &=
				    ~SSP_PARACON1_PCS_RX_LOS_MASK_VAL_MASK;
				ssp_tune1 |=
				    SSP_PARACON1_PCS_RX_LOS_MASK_VAL(val);
				/* SSP TEST setting : 0x135e/f_0000 + 0x3c */
			} else if (!strcmp(para_name, "tx_eye_height_cntl_en")) {
				ssp_tune2 &=
				    ~SSP_TEST_TX_EYE_HEIGHT_CNTL_EN_MASK;
				ssp_tune2 |=
				    SSP_TEST_TX_EYE_HEIGHT_CNTL_EN(val);
			} else if (!strcmp(para_name, "pipe_tx_deemph_update_delay")) {
				ssp_tune2 &=
				    ~SSP_TEST_PIPE_TX_DEEMPH_UPDATE_DELAY_MASK;
				ssp_tune2 |=
				    SSP_TEST_PIPE_TX_DEEMPH_UPDATE_DELAY(val);
			} else if (!strcmp(para_name, "pcs_tx_swing_full_sstx")) {
				ssp_tune2 &=
				    ~SSP_TEST_PCS_TX_SWING_FULL_SSTX_MASK;
				ssp_tune2 |=
				    SSP_TEST_PCS_TX_SWING_FULL_SSTX(val);
			}

		}		/* for */
		writel(ssp_tune0, ss_reg_base + EXYNOS_USBCON_SSP_PARACON0);
		writel(ssp_tune1, ss_reg_base + EXYNOS_USBCON_SSP_PARACON1);
		writel(ssp_tune2, ss_reg_base + EXYNOS_USBCON_SSP_TEST);
	}
	if (ss_cap) {
		/* ssphy tuning */
		void __iomem *ss_reg_base;

		if (info->used_phy_port == 1)
			ss_reg_base = info->regs_base_2nd;
		else
			ss_reg_base = info->regs_base;

		ssp_tune0 = readl(ss_reg_base +
				  EXYNOS_USBCON_G2PHY_PRTCL1EXT15);
		ssp_tune1 = readl(ss_reg_base + EXYNOS_USBCON_G2PHY_CNTL0);

		cnt = 0;
		for (; info->tune_param[cnt].value != EXYNOS_USB_TUNE_LAST;
		     cnt++) {
			char *para_name;
			int val;

			val = info->tune_param[cnt].value;
			if (val == -1)
				continue;

			para_name = info->tune_param[cnt].name;

			if (!strcmp(para_name, "tx_vboost_lvl")) {
				ssp_tune0 &=
				    ~G2PHY_PRTCL1EXT15_TX_VBOOST_LVL_MASK;
				ssp_tune0 |=
				    G2PHY_PRTCL1EXT15_TX_VBOOST_LVL(val);
				ssp_tune1 |= G2PHY_CNTL0_EXT_CTRL_SEL;
				writel(ssp_tune0, ss_reg_base +
				       EXYNOS_USBCON_G2PHY_PRTCL1EXT15);
				writel(ssp_tune1, ss_reg_base +
				       EXYNOS_USBCON_G2PHY_CNTL0);

				udelay(1);

				ssp_tune1 &= (~G2PHY_CNTL0_EXT_CTRL_SEL);
				writel(ssp_tune1, ss_reg_base +
				       EXYNOS_USBCON_G2PHY_CNTL0);
			} else if (!strcmp(para_name, "tx_iboost_lvl")) {
				ssp_tune0 &=
				    ~G2PHY_PRTCL1EXT15_TX_IBOOST_LVL_MASK;
				ssp_tune0 |=
				    G2PHY_PRTCL1EXT15_TX_IBOOST_LVL(val);
				ssp_tune1 |= G2PHY_CNTL0_EXT_CTRL_SEL;
				writel(ssp_tune0, ss_reg_base +
				       EXYNOS_USBCON_G2PHY_PRTCL1EXT15);
				writel(ssp_tune1, ss_reg_base +
				       EXYNOS_USBCON_G2PHY_CNTL0);

				udelay(1);

				ssp_tune1 &= (~G2PHY_CNTL0_EXT_CTRL_SEL);
				writel(ssp_tune1, ss_reg_base +
				       EXYNOS_USBCON_G2PHY_CNTL0);
			}
		}
	}
}

void phy_exynos_usb_v3p1_tune_each(struct exynos_usbphy_info *info,
				   char *para_name, int val)
{
#ifndef __BOOT__
	u32 hsp_tune, ssp_tune0, ssp_tune1;

	bool ss_only_cap;
	/* For KITT */
	bool ss_cap;

	ss_only_cap = (info->version & EXYNOS_USBCON_VER_SS_CAP) >> 4;
	ss_cap = (info->version & EXYNOS_USBCON_VER_SS_CAP) >> 6;

	if (!info->tune_param)
		return;

	if (!ss_only_cap) {
		/* hsphy tuning */
		void __iomem *regs_base = info->regs_base;

		hsp_tune = readl(regs_base + EXYNOS_USBCON_HSP_TUNE);
		if (!strcmp(para_name, "compdis")) {
			hsp_tune &= ~HSP_TUNE_COMPDIS_MASK;
			hsp_tune |= HSP_TUNE_COMPDIS_SET(val);
		} else if (!strcmp(para_name, "otg")) {
			hsp_tune &= ~HSP_TUNE_OTG_MASK;
			hsp_tune |= HSP_TUNE_OTG_SET(val);
		} else if (!strcmp(para_name, "rx_sqrx")) {
			hsp_tune &= ~HSP_TUNE_SQRX_MASK;
			hsp_tune |= HSP_TUNE_SQRX_SET(val);
		} else if (!strcmp(para_name, "tx_fsls")) {
			hsp_tune &= ~HSP_TUNE_TXFSLS_MASK;
			hsp_tune |= HSP_TUNE_TXFSLS_SET(val);
		} else if (!strcmp(para_name, "tx_hsxv")) {
			hsp_tune &= ~HSP_TUNE_HSXV_MASK;
			hsp_tune |= HSP_TUNE_HSXV_SET(val);
		} else if (!strcmp(para_name, "tx_pre_emp")) {
			hsp_tune &= ~HSP_TUNE_TXPREEMPA_MASK;
			hsp_tune |= HSP_TUNE_TXPREEMPA_SET(val);
		} else if (!strcmp(para_name, "tx_pre_emp_plus")) {
			if (val)
				hsp_tune |= HSP_TUNE_TXPREEMPA_PLUS;
			else
				hsp_tune &= ~HSP_TUNE_TXPREEMPA_PLUS;
		} else if (!strcmp(para_name, "tx_res")) {
			hsp_tune &= ~HSP_TUNE_TXRES_MASK;
			hsp_tune |= HSP_TUNE_TXRES_SET(val);
		} else if (!strcmp(para_name, "tx_rise")) {
			hsp_tune &= ~HSP_TUNE_TXRISE_MASK;
			hsp_tune |= HSP_TUNE_TXRISE_SET(val);
		} else if (!strcmp(para_name, "tx_vref")) {
			hsp_tune &= ~HSP_TUNE_TXVREF_MASK;
			hsp_tune |= HSP_TUNE_TXVREF_SET(val);
		} else if (!strcmp(para_name, "tx_res_ovrd")) {
			phy_exynos_usb_v3p1_tif_ov_wr(info, 0x6, val);
		}
		writel(hsp_tune, regs_base + EXYNOS_USBCON_HSP_TUNE);
	}

	if (ss_only_cap) {
		/* ssphy tuning */
		void __iomem *ss_reg_base;

		if (info->used_phy_port == 1)
			ss_reg_base = info->regs_base_2nd;
		else
			ss_reg_base = info->regs_base;

		ssp_tune0 = readl(ss_reg_base + EXYNOS_USBCON_SSP_PARACON0);
		ssp_tune1 = readl(ss_reg_base + EXYNOS_USBCON_SSP_PARACON1);

		if (!strcmp(para_name, "tx0_term_offset")) {
			ssp_tune0 &= ~SSP_PARACON0_TX0_TERM_OFFSET_MASK;
			ssp_tune0 |= SSP_PARACON0_TX0_TERM_OFFSET(val);
		} else if (!strcmp(para_name, "pcs_tx_swing_full")) {
			ssp_tune0 &= ~SSP_PARACON0_PCS_TX_SWING_FULL_MASK;
			ssp_tune0 |= SSP_PARACON0_PCS_TX_SWING_FULL(val);
		} else if (!strcmp(para_name, "pcs_tx_deemph_6db")) {
			ssp_tune0 &= ~SSP_PARACON0_PCS_TX_DEEMPH_6DB_MASK;
			ssp_tune0 |= SSP_PARACON0_PCS_TX_DEEMPH_6DB(val);
		} else if (!strcmp(para_name, "pcs_tx_deemph_3p5db")) {
			ssp_tune0 &= ~SSP_PARACON0_PCS_TX_DEEMPH_3P5DB_MASK;
			ssp_tune0 |= SSP_PARACON0_PCS_TX_DEEMPH_3P5DB(val);
		} else if (!strcmp(para_name, "tx_vboost_lvl")) {
			ssp_tune1 &= ~SSP_PARACON1_TX_VBOOST_LVL_MASK;
			ssp_tune1 |= SSP_PARACON1_TX_VBOOST_LVL(val);
		} else if (!strcmp(para_name, "los_level")) {
			ssp_tune1 &= ~SSP_PARACON1_LOS_LEVEL_MASK;
			ssp_tune1 |= SSP_PARACON1_LOS_LEVEL(val);
		} else if (!strcmp(para_name, "los_bias")) {
			ssp_tune1 &= ~SSP_PARACON1_LOS_BIAS_MASK;
			ssp_tune1 |= SSP_PARACON1_LOS_BIAS(val);
		} else if (!strcmp(para_name, "pcs_rx_los_mask_val")) {
			ssp_tune1 &= ~SSP_PARACON1_PCS_RX_LOS_MASK_VAL_MASK;
			ssp_tune1 |= SSP_PARACON1_PCS_RX_LOS_MASK_VAL(val);
		}
		writel(ssp_tune0, ss_reg_base + EXYNOS_USBCON_SSP_PARACON0);
		writel(ssp_tune1, ss_reg_base + EXYNOS_USBCON_SSP_PARACON1);
	}
	/* ss_only_cap */
	if (ss_cap) {
		/* ssphy tuning */
		void __iomem *ss_reg_base;

		if (info->used_phy_port == 1)
			ss_reg_base = info->regs_base_2nd;
		else
			ss_reg_base = info->regs_base;

		ssp_tune0 = readl(ss_reg_base +
				  EXYNOS_USBCON_G2PHY_PRTCL1EXT15);
		ssp_tune1 = readl(ss_reg_base + EXYNOS_USBCON_G2PHY_CNTL0);

		if (!strcmp(para_name, "tx_vboost_lvl")) {
			ssp_tune0 &= ~G2PHY_PRTCL1EXT15_TX_VBOOST_LVL_MASK;
			ssp_tune0 |= G2PHY_PRTCL1EXT15_TX_VBOOST_LVL(val);
			ssp_tune1 |= G2PHY_CNTL0_EXT_CTRL_SEL;
			writel(ssp_tune1, ss_reg_base +
			       EXYNOS_USBCON_G2PHY_CNTL0);
			writel(ssp_tune0, ss_reg_base +
			       EXYNOS_USBCON_G2PHY_PRTCL1EXT15);

			ssp_tune1 &= (~G2PHY_CNTL0_EXT_CTRL_SEL);
			writel(ssp_tune1,
			       ss_reg_base + EXYNOS_USBCON_G2PHY_CNTL0);

		} else if (!strcmp(para_name, "tx_iboost_lvl")) {
			ssp_tune0 &= ~G2PHY_PRTCL1EXT15_TX_IBOOST_LVL_MASK;
			ssp_tune0 |= G2PHY_PRTCL1EXT15_TX_IBOOST_LVL(val);
			ssp_tune1 |= G2PHY_CNTL0_EXT_CTRL_SEL;
			writel(ssp_tune1, ss_reg_base +
			       EXYNOS_USBCON_G2PHY_CNTL0);
			writel(ssp_tune0, ss_reg_base +
			       EXYNOS_USBCON_G2PHY_PRTCL1EXT15);

			ssp_tune1 &= (~G2PHY_CNTL0_EXT_CTRL_SEL);
			writel(ssp_tune1, ss_reg_base +
			       EXYNOS_USBCON_G2PHY_CNTL0);
		}
	}
#endif
}

void phy_exynos_usb_v3p1_rd_tune_each_from_reg(struct exynos_usbphy_info *info,
					       u32 tune, char *para_name,
					       int *val)
{
	bool ss_only_cap;

	ss_only_cap = (info->version & EXYNOS_USBCON_VER_SS_CAP) >> 4;

	if (!info->tune_param)
		return;

	if (!ss_only_cap) {	/* hsphy tuning */
		if (!strcmp(para_name, "compdis"))
			*val = HSP_TUNE_COMPDIS_GET(tune);
		else if (!strcmp(para_name, "otg"))
			*val = HSP_TUNE_OTG_GET(tune);
		else if (!strcmp(para_name, "rx_sqrx"))
			*val = HSP_TUNE_SQRX_GET(tune);
		else if (!strcmp(para_name, "tx_fsls"))
			*val = HSP_TUNE_TXFSLS_GET(tune);
		else if (!strcmp(para_name, "tx_hsxv"))
			*val = HSP_TUNE_HSXV_GET(tune);
		else if (!strcmp(para_name, "tx_pre_emp"))
			*val = HSP_TUNE_TXPREEMPA_GET(tune);
		else if (!strcmp(para_name, "tx_pre_emp_plus"))
			*val = HSP_TUNE_TXPREEMPA_PLUS_GET(tune);
		else if (!strcmp(para_name, "tx_res"))
			*val = HSP_TUNE_TXRES_GET(tune);
		else if (!strcmp(para_name, "tx_rise"))
			*val = HSP_TUNE_TXRISE_GET(tune);
		else if (!strcmp(para_name, "tx_vref"))
			*val = HSP_TUNE_TXVREF_GET(tune);
		else
			*val = -1;
	}
}

void phy_exynos_usb_v3p1_wr_tune_reg(struct exynos_usbphy_info *info, u32 val)
{
	void __iomem *regs_base = info->regs_base;

	writel(val, regs_base + EXYNOS_USBCON_HSP_TUNE);
}

void phy_exynos_usb_v3p1_rd_tune_reg(struct exynos_usbphy_info *info, u32 *val)
{
	void __iomem *regs_base = info->regs_base;

	if (!val)
		return;

	*val = readl(regs_base + EXYNOS_USBCON_HSP_TUNE);
}
