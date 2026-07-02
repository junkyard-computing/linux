// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung USB-DP Gen2 v4 combo PHY (PMA) bring-up for Google Tensor gs201.
 *
 * PMA register configuration + LCPLL/CDR lock sequence ported verbatim from the
 * AOSP google-modules gs driver (phy-exynos-usbdp-gen2-v4.c). Only the 19.2 MHz
 * refclk path is used on gs201; DT-tune params are not present, so tune /
 * tune_late / offset_cal_code are intentionally not ported.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>

#include "phy-samsung-usb-cal.h"
#include "phy-exynos-usb3p1-reg.h"
#include "phy-exynos-usbdp-gen2-v4-reg.h"
#include "phy-exynos-usbdp-gen2-v4-reg-pcs.h"
#include "phy-exynos-usbdp-gen2-v4.h"

static void
phy_exynos_usbdp_g2_v4_ctrl_pma_ready(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->ctrl_base;
	u32 reg;

	/* link pipe_clock selection to pclk of PMA */
	reg = readl(regs_base + EXYNOS_USBCON_CLKRST);
	reg |= CLKRST_LINK_PCLK_SEL;
	writel(reg, regs_base + EXYNOS_USBCON_CLKRST);

	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg |= PMA_REF_FREQ_SEL_SET(1);
	/* SFR reset */
	reg |= PMA_LOW_PWR;
	reg |= PMA_APB_SW_RST;
	/* reference clock 26MHz path from XTAL */
	reg &= ~PMA_ROPLL_REF_CLK_SEL_MASK;
	reg &= ~PMA_LCPLL_REF_CLK_SEL_MASK;
	/* PMA_POWER_OFF */
	reg |= PMA_TRSV_SW_RST;
	reg |= PMA_CMN_SW_RST;
	reg |= PMA_INIT_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);

	udelay(1);

	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_LOW_PWR;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);

	/* APB enable */
	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_APB_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
}

static void
phy_exynos_usbdp_g2_v4_ctrl_pma_rst_release(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->ctrl_base;
	u32 reg;

	/* Reset release from port */
	reg = readl(regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
	reg &= ~PMA_TRSV_SW_RST;
	reg &= ~PMA_CMN_SW_RST;
	reg &= ~PMA_INIT_SW_RST;
	writel(reg, regs_base + EXYNOS_USBCON_COMBO_PMA_CTRL);
}

static void
phy_exynos_usbdp_g2_v4_aux_force_off(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0008);
	reg &= USBDP_CMN_REG0008_OVRD_AUX_EN_CLR;
	reg |= USBDP_CMN_REG0008_OVRD_AUX_EN_SET(1);
	reg &= USBDP_CMN_REG0008_AUX_EN_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0008);
}

static inline void phy_exynos_usbdp_g2_v4_pma_default_sfr_update_19_2Mhz(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->pma_base;

	/* 190627: CDR data mode exit GEN1 ON / GEN2 OFF */
	writel(0xff, regs_base + 0x0c8c);
	writel(0xff, regs_base + 0x1c8c);
	writel(0x7d, regs_base + 0x0c9c);
	writel(0x7d, regs_base + 0x1c9c);

	/* 190708: Setting for improvement of EDS distribution */
	writel(0x06, regs_base + 0x0e7c);
	writel(0x00, regs_base + 0x09e0);
	writel(0x35, regs_base + 0x09e4);
	writel(0x06, regs_base + 0x1e7c);
	writel(0x00, regs_base + 0x19e0);
	writel(0x35, regs_base + 0x19e4);

	/* 190707: Improve LVCC */
	writel(0x30, regs_base + 0x08f0);
	writel(0x30, regs_base + 0x18f0);

	/* 190820: LFPS RX VIH shmoo hole */
	writel(0x0c, regs_base + 0x0a08);
	writel(0x0c, regs_base + 0x1a08);

	/* 190820: Remove unrelated option for v4 phy */
	writel(0x05, regs_base + 0x0a0c);
	writel(0x05, regs_base + 0x1a0c);

	/* 210521: Improve USB Gen2 LVCC  */
	writel(0x1c, regs_base + 0x00f8);
	writel(0x54, regs_base + 0x00fc);

	/* 191128: Change Vth of RCV_DET because of TD 7.40 Polling Retry Test */
	writel(0x05, regs_base + 0x104c);
	writel(0x05, regs_base + 0x204c);

	/* Gen1 */
	writel(0x00, regs_base + 0x0ca8);
	writel(0x00, regs_base + 0x1ca8);
	writel(0x04, regs_base + 0x0cac);
	writel(0x04, regs_base + 0x1cac);

	/* Gen2 */
	writel(0x00, regs_base + 0x0cb8);
	writel(0x00, regs_base + 0x1cb8);
	writel(0x04, regs_base + 0x0cbc);
	writel(0x04, regs_base + 0x1cbc);

	/* CDR Lock Delay for JTOL Link Training */
	writel(0x03, regs_base + 0x0320);
	writel(0x23, regs_base + 0x0324);

	/* common reset for 4lane tx */
	writel(0x63, regs_base + 0x0858);
	writel(0x63, regs_base + 0x1858);
	writel(0x63, regs_base + 0x1058);
	writel(0x63, regs_base + 0x2058);

	/*
	* Change rx rterm to 90 ohm
	*/
	writel(0xA0, regs_base + 0x0BB4);
	writel(0xA0, regs_base + 0x1BB4);

	/*
	* added setting
	*/
	writel(0x0C, regs_base + 0x0A08);
	writel(0x0C, regs_base + 0x1A08);

	writel(0x2B, regs_base + 0x0DEC);
	writel(0x2B, regs_base + 0x1DEC);

	/* change rx_hf_cs_ctrl[7:6] */
	writel(0x7F, regs_base + 0x0934);
	writel(0x7F, regs_base + 0x1934);
	/* change rx_vga_rl_ctrl[5:3] */
	writel(0x32, regs_base + 0x0948);
	writel(0x32, regs_base + 0x1948);
	/* change vga_bin_ssp[5:1] */
	writel(0x00, regs_base + 0x0DF4);
	writel(0x00, regs_base + 0x1DF4);

	writel(0x1D, regs_base + 0x091C);
	writel(0x1D, regs_base + 0x191C);

	writel(0x0C, regs_base + 0x0928);
	writel(0x0C, regs_base + 0x1928);

	writel(0x3C, regs_base + 0x0E0C);
	writel(0x3C, regs_base + 0x1E0C);

	writel(0x04, regs_base + 0x0EBC);
	writel(0x04, regs_base + 0x1EBC);

	writel(0x10, regs_base + 0x0908);
	writel(0x10, regs_base + 0x1908);

}

static inline void phy_exynos_usbdp_g2_v4_pma_default_sfr_update_19_2Mhz_Gen2(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0101);
	reg &= USBDP_CMN_REG0101_TIME_CDR_WATCHDOG_WAIT_RESTART__7_0_CLR;
	reg |= USBDP_CMN_REG0101_TIME_CDR_WATCHDOG_WAIT_RESTART__7_0_SET(0xBB);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0101);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG012C);
	reg &= USBDP_CMN_REG012C_RX_LFPS_DET_FILT_TH3_SHORT_RISE_SP_CLR;
	reg |= USBDP_CMN_REG012C_RX_LFPS_DET_FILT_TH3_SHORT_RISE_SP_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG012C);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG012D);
	reg &= USBDP_CMN_REG012D_RX_LFPS_DET_FILT_TH3_SHORT_FALL_SP_CLR;
	reg |= USBDP_CMN_REG012D_RX_LFPS_DET_FILT_TH3_SHORT_FALL_SP_SET(0x3);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG012D);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0134);
	reg &= USBDP_CMN_REG0134_RX_LFPS_DET_FILT_TH3_SHORT_RISE_SSP_CLR;
	reg |= USBDP_CMN_REG0134_RX_LFPS_DET_FILT_TH3_SHORT_RISE_SSP_SET(0x3);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0134);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0135);
	reg &= USBDP_CMN_REG0135_RX_LFPS_DET_FILT_TH3_SHORT_FALL_SSP_CLR;
	reg |= USBDP_CMN_REG0135_RX_LFPS_DET_FILT_TH3_SHORT_FALL_SSP_SET(0x3);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0135);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG013C);
	reg &= USBDP_CMN_REG013C_RX_LFPS_DET_FILT_TH3_LONG_RISE_SP_CLR;
	reg |= USBDP_CMN_REG013C_RX_LFPS_DET_FILT_TH3_LONG_RISE_SP_SET(0x0F);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG013C);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG013D);
	reg &= USBDP_CMN_REG013D_RX_LFPS_DET_FILT_TH3_LONG_FALL_SP_CLR;
	reg |= USBDP_CMN_REG013D_RX_LFPS_DET_FILT_TH3_LONG_FALL_SP_SET(0x0F);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG013D);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0144);
	reg &= USBDP_CMN_REG0144_RX_LFPS_DET_FILT_TH3_LONG_RISE_SSP_CLR;
	reg |= USBDP_CMN_REG0144_RX_LFPS_DET_FILT_TH3_LONG_RISE_SSP_SET(0x0F);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0144);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0145);
	reg &= USBDP_CMN_REG0145_RX_LFPS_DET_FILT_TH3_LONG_FALL_SSP_CLR;
	reg |= USBDP_CMN_REG0145_RX_LFPS_DET_FILT_TH3_LONG_FALL_SSP_SET(0x0F);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0145);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG016A);
	reg &= USBDP_CMN_REG016A_TX_LFPS_TP_GEN_T_PWM_CLR;
	reg |= USBDP_CMN_REG016A_TX_LFPS_TP_GEN_T_PWM_SET(0x2B);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG016A);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG016B);
	reg &= USBDP_CMN_REG016B_TX_LFPS_TP_GEN_T_LFPS0_CLR;
	reg |= USBDP_CMN_REG016B_TX_LFPS_TP_GEN_T_LFPS0_SET(0x0F);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG016B);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG016C);
	reg &= USBDP_CMN_REG016C_TX_LFPS_TP_GEN_T_LFPS1_CLR;
	reg |= USBDP_CMN_REG016C_TX_LFPS_TP_GEN_T_LFPS1_SET(0x1E);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG016C);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG016F);
	reg &= USBDP_CMN_REG016F_BIAS_ICAL_AUTO_COMP_EN_DELAY_CLR;
	reg |= USBDP_CMN_REG016F_BIAS_ICAL_AUTO_COMP_EN_DELAY_SET(0x13);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG016F);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0170);
	reg &= USBDP_CMN_REG0170_BIAS_ICAL_AUTO_CODE_DELAY_CLR;
	reg |= USBDP_CMN_REG0170_BIAS_ICAL_AUTO_CODE_DELAY_SET(0x13);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0170);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG031A);
	reg &= USBDP_TRSV_REG031A_LN0_TG_RXD_COMP_DELAY_TIME__15_8_CLR;
	reg |= USBDP_TRSV_REG031A_LN0_TG_RXD_COMP_DELAY_TIME__15_8_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG031A);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG031B);
	reg &= USBDP_TRSV_REG031B_LN0_TG_RXD_COMP_DELAY_TIME__7_0_CLR;
	reg |= USBDP_TRSV_REG031B_LN0_TG_RXD_COMP_DELAY_TIME__7_0_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG031B);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0324);
	reg &= USBDP_TRSV_REG0324_LN0_RX_CDR_DATA_MODE_EXIT_EXTEND_SP_CLR;
	reg |= USBDP_TRSV_REG0324_LN0_RX_CDR_DATA_MODE_EXIT_EXTEND_SP_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0324);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG032A);
	reg &= USBDP_TRSV_REG032A_LN0_RX_VALID_RSTN_DELAY_RISE_SP__15_8_CLR;
	reg |= USBDP_TRSV_REG032A_LN0_RX_VALID_RSTN_DELAY_RISE_SP__15_8_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG032A);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG032B);
	reg &= USBDP_TRSV_REG032B_LN0_RX_VALID_RSTN_DELAY_RISE_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG032B_LN0_RX_VALID_RSTN_DELAY_RISE_SP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG032B);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG032C);
	reg &= USBDP_TRSV_REG032C_LN0_RX_VALID_RSTN_DELAY_FALL_SP__15_8_CLR;
	reg |= USBDP_TRSV_REG032C_LN0_RX_VALID_RSTN_DELAY_FALL_SP__15_8_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG032C);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG032D);
	reg &= USBDP_TRSV_REG032D_LN0_RX_VALID_RSTN_DELAY_FALL_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG032D_LN0_RX_VALID_RSTN_DELAY_FALL_SP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG032D);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG032E);
	reg &= USBDP_TRSV_REG032E_LN0_RX_VALID_RSTN_DELAY_RISE_SSP__15_8_CLR;
	reg |= USBDP_TRSV_REG032E_LN0_RX_VALID_RSTN_DELAY_RISE_SSP__15_8_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG032E);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG032F);
	reg &= USBDP_TRSV_REG032F_LN0_RX_VALID_RSTN_DELAY_RISE_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG032F_LN0_RX_VALID_RSTN_DELAY_RISE_SSP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG032F);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0330);
	reg &= USBDP_TRSV_REG0330_LN0_RX_VALID_RSTN_DELAY_FALL_SSP__15_8_CLR;
	reg |= USBDP_TRSV_REG0330_LN0_RX_VALID_RSTN_DELAY_FALL_SSP__15_8_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0330);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0331);
	reg &= USBDP_TRSV_REG0331_LN0_RX_VALID_RSTN_DELAY_FALL_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG0331_LN0_RX_VALID_RSTN_DELAY_FALL_SSP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0331);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG071A);
	reg &= USBDP_TRSV_REG071A_LN2_TG_RXD_COMP_DELAY_TIME__15_8_CLR;
	reg |= USBDP_TRSV_REG071A_LN2_TG_RXD_COMP_DELAY_TIME__15_8_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG071A);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG071B);
	reg &= USBDP_TRSV_REG071B_LN2_TG_RXD_COMP_DELAY_TIME__7_0_CLR;
	reg |= USBDP_TRSV_REG071B_LN2_TG_RXD_COMP_DELAY_TIME__7_0_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG071B);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0724);
	reg &= USBDP_TRSV_REG0724_LN2_RX_CDR_DATA_MODE_EXIT_EXTEND_SP_CLR;
	reg |= USBDP_TRSV_REG0724_LN2_RX_CDR_DATA_MODE_EXIT_EXTEND_SP_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0724);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG072A);
	reg &= USBDP_TRSV_REG072A_LN2_RX_VALID_RSTN_DELAY_RISE_SP__15_8_CLR;
	reg |= USBDP_TRSV_REG072A_LN2_RX_VALID_RSTN_DELAY_RISE_SP__15_8_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG072A);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG072B);
	reg &= USBDP_TRSV_REG072B_LN2_RX_VALID_RSTN_DELAY_RISE_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG072B_LN2_RX_VALID_RSTN_DELAY_RISE_SP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG072B);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG072C);
	reg &= USBDP_TRSV_REG072C_LN2_RX_VALID_RSTN_DELAY_FALL_SP__15_8_CLR;
	reg |= USBDP_TRSV_REG072C_LN2_RX_VALID_RSTN_DELAY_FALL_SP__15_8_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG072C);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG072D);
	reg &= USBDP_TRSV_REG072D_LN2_RX_VALID_RSTN_DELAY_FALL_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG072D_LN2_RX_VALID_RSTN_DELAY_FALL_SP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG072D);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG072E);
	reg &= USBDP_TRSV_REG072E_LN2_RX_VALID_RSTN_DELAY_RISE_SSP__15_8_CLR;
	reg |= USBDP_TRSV_REG072E_LN2_RX_VALID_RSTN_DELAY_RISE_SSP__15_8_SET(0x03);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG072E);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG072F);
	reg &= USBDP_TRSV_REG072F_LN2_RX_VALID_RSTN_DELAY_RISE_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG072F_LN2_RX_VALID_RSTN_DELAY_RISE_SSP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG072F);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0730);
	reg &= USBDP_TRSV_REG0730_LN2_RX_VALID_RSTN_DELAY_FALL_SSP__15_8_CLR;
	reg |= USBDP_TRSV_REG0730_LN2_RX_VALID_RSTN_DELAY_FALL_SSP__15_8_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0730);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0731);
	reg &= USBDP_TRSV_REG0731_LN2_RX_VALID_RSTN_DELAY_FALL_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG0731_LN2_RX_VALID_RSTN_DELAY_FALL_SSP__7_0_SET(0x00);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0731);
}

static inline void phy_exynos_usbdp_g2_v4_pma_default_sfr_update_19_2Mhz_Gen1(struct exynos_usbphy_info *info)
{

	void __iomem *regs_base = info->pma_base;
	u32 reg;

	/* --- Common Block --- */

	/* ana_lcpll_pms_mdiv, mdiv_afc*/
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0027);
	reg &= USBDP_CMN_REG0027_ANA_LCPLL_PMS_MDIV_CLR;
	reg |= USBDP_CMN_REG0027_ANA_LCPLL_PMS_MDIV_SET(0x82);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0027);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0028);
	reg &= USBDP_CMN_REG0028_ANA_LCPLL_PMS_MDIV_AFC_CLR;

	reg |= USBDP_CMN_REG0028_ANA_LCPLL_PMS_MDIV_AFC_SET(0x82);
	// Fixed: 20210514
	// reg |= USBDP_CMN_REG0028_ANA_LCPLL_PMS_MDIV_AFC_SET(0xD4);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0028);

	/*lcpll_pms_sdiv_sp, ssp*/
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG002A);
	reg &= USBDP_CMN_REG002A_LCPLL_PMS_SDIV_SP_CLR;
	reg |= USBDP_CMN_REG002A_LCPLL_PMS_SDIV_SP_SET(0x3);
	reg &= USBDP_CMN_REG002A_LCPLL_PMS_SDIV_SSP_CLR;
	reg |= USBDP_CMN_REG002A_LCPLL_PMS_SDIV_SSP_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG002A);

	/* IQ Div for LC VCO and RO bypass */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG002D);
	reg &= USBDP_CMN_REG002D_ANA_LCPLL_IQDIV_BYPASS_CLR;
	reg |= USBDP_CMN_REG002D_ANA_LCPLL_IQDIV_BYPASS_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG002D);

	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0074);
	reg &= USBDP_CMN_REG0074_ANA_ROPLL_IQDIV_BYPASS_CLR;
	reg |= USBDP_CMN_REG0074_ANA_ROPLL_IQDIV_BYPASS_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0074);

	/* Numerator of SDM with i_lcpll_sdm_numerator_sign (-255~255) */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0034);
	reg &= USBDP_CMN_REG0034_ANA_LCPLL_SDM_NUMERATOR_CLR;
	reg |= USBDP_CMN_REG0034_ANA_LCPLL_SDM_NUMERATOR_SET(0x10);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0034);

	/* Denominator of SDM (Max. 255) */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0032);
	reg &= USBDP_CMN_REG0032_ANA_LCPLL_SDM_DENOMINATOR_CLR;
	reg |= USBDP_CMN_REG0032_ANA_LCPLL_SDM_DENOMINATOR_SET(0x27);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0032);

	/* LC PLL input clk phase, SDC divide-ratio  */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0035);
	reg &= USBDP_CMN_REG0035_ANA_LCPLL_SDM_PH_NUM_SEL_CLR;
	reg |= USBDP_CMN_REG0035_ANA_LCPLL_SDM_PH_NUM_SEL_SET(0x0);
	reg &= USBDP_CMN_REG0035_ANA_LCPLL_SDC_N_CLR;
	reg |= USBDP_CMN_REG0035_ANA_LCPLL_SDC_N_SET(0x5);
	reg &= USBDP_CMN_REG0035_ANA_LCPLL_SDC_N2_CLR;
	reg |= USBDP_CMN_REG0035_ANA_LCPLL_SDC_N2_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0035);

	/* RO PLL PI input clock phase number 0: 8-phase, 1: 4-phase */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0086);
	reg &= USBDP_CMN_REG0086_ANA_ROPLL_SDM_PH_NUM_SEL_CLR;
	reg |= USBDP_CMN_REG0086_ANA_ROPLL_SDM_PH_NUM_SEL_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0086);

	/* RO PLL SDC divide-ratio selection */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0089);
	reg &= USBDP_CMN_REG0089_ANA_ROPLL_SDC_N2_CLR;
	reg |= USBDP_CMN_REG0089_ANA_ROPLL_SDC_N2_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0089);

	/* RO PLL numerator of SDC (Max 65) */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0036);
	reg &= USBDP_CMN_REG0036_ANA_LCPLL_SDC_NUMERATOR_CLR;
	reg |= USBDP_CMN_REG0036_ANA_LCPLL_SDC_NUMERATOR_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0036);

	/* LC PLL denominator of SDC (Max 65) */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0037);
	reg &= USBDP_CMN_REG0037_ANA_LCPLL_SDC_DENOMINATOR_CLR;
	reg |= USBDP_CMN_REG0037_ANA_LCPLL_SDC_DENOMINATOR_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0037);

	/* LC PLL SSC modulation deviation control */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG0039);
	reg &= USBDP_CMN_REG0039_ANA_LCPLL_SSC_FM_DEVIATION_CLR;
	reg |= USBDP_CMN_REG0039_ANA_LCPLL_SSC_FM_DEVIATION_SET(0x32);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG0039);

	/* LC PLL SSC modulation frequency control */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG003A);
	reg &= USBDP_CMN_REG003A_ANA_LCPLL_SSC_FM_FREQ_CLR;
	reg |= USBDP_CMN_REG003A_ANA_LCPLL_SSC_FM_FREQ_SET(0x5);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG003A);

	/* --- TRSV Block --- */
	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02B6);
	reg &= USBDP_TRSV_REG02B6_LN0_RX_CDR_PMS_M_SP__8_CLR;
	reg |= USBDP_TRSV_REG02B6_LN0_RX_CDR_PMS_M_SP__8_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02B6);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02B7);
	reg &= USBDP_TRSV_REG02B7_LN0_RX_CDR_PMS_M_SP__7_0_CLR;
	// modified
	reg |= USBDP_TRSV_REG02B7_LN0_RX_CDR_PMS_M_SP__7_0_SET(0x82);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02B7);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02B8);
	reg &= USBDP_TRSV_REG02B8_LN0_RX_CDR_PMS_M_SSP__8_CLR;
	reg |= USBDP_TRSV_REG02B8_LN0_RX_CDR_PMS_M_SSP__8_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02B8);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG02B9);
	reg &= USBDP_TRSV_REG02B9_LN0_RX_CDR_PMS_M_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG02B9_LN0_RX_CDR_PMS_M_SSP__7_0_SET(0x4);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG02B9);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0386);
	reg &= USBDP_TRSV_REG0386_LN0_RX_CDR_AFC_PMS_M_SP__8_CLR;
	reg |= USBDP_TRSV_REG0386_LN0_RX_CDR_AFC_PMS_M_SP__8_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0386);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0387);
	reg &= USBDP_TRSV_REG0387_LN0_RX_CDR_AFC_PMS_M_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG0387_LN0_RX_CDR_AFC_PMS_M_SP__7_0_SET(0x82);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0387);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0388);
	reg &= USBDP_TRSV_REG0388_LN0_RX_CDR_AFC_PMS_M_SSP__8_CLR;
	reg |= USBDP_TRSV_REG0388_LN0_RX_CDR_AFC_PMS_M_SSP__8_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0388);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0389);
	reg &= USBDP_TRSV_REG0389_LN0_RX_CDR_AFC_PMS_M_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG0389_LN0_RX_CDR_AFC_PMS_M_SSP__7_0_SET(0x4);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0389);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06B6);
	reg &= USBDP_TRSV_REG06B6_LN2_RX_CDR_PMS_M_SP__8_CLR;
	reg |= USBDP_TRSV_REG06B6_LN2_RX_CDR_PMS_M_SP__8_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06B6);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06B7);
	reg &= USBDP_TRSV_REG06B7_LN2_RX_CDR_PMS_M_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG06B7_LN2_RX_CDR_PMS_M_SP__7_0_SET(0x82);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06B7);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06B8);
	reg &= USBDP_TRSV_REG06B8_LN2_RX_CDR_PMS_M_SSP__8_CLR;
	reg |= USBDP_TRSV_REG06B8_LN2_RX_CDR_PMS_M_SSP__8_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06B8);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG06B9);
	reg &= USBDP_TRSV_REG06B9_LN2_RX_CDR_PMS_M_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG06B9_LN2_RX_CDR_PMS_M_SSP__7_0_SET(0x4);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG06B9);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0786);
	reg &= USBDP_TRSV_REG0786_LN2_RX_CDR_AFC_PMS_M_SP__8_CLR;
	reg |= USBDP_TRSV_REG0786_LN2_RX_CDR_AFC_PMS_M_SP__8_SET(0x0);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0786);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0787);
	reg &= USBDP_TRSV_REG0787_LN2_RX_CDR_AFC_PMS_M_SP__7_0_CLR;
	reg |= USBDP_TRSV_REG0787_LN2_RX_CDR_AFC_PMS_M_SP__7_0_SET(0x82);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0787);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0788);
	reg &= USBDP_TRSV_REG0788_LN2_RX_CDR_AFC_PMS_M_SSP__8_CLR;
	reg |= USBDP_TRSV_REG0788_LN2_RX_CDR_AFC_PMS_M_SSP__8_SET(0x1);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0788);

	reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0789);
	reg &= USBDP_TRSV_REG0789_LN2_RX_CDR_AFC_PMS_M_SSP__7_0_CLR;
	reg |= USBDP_TRSV_REG0789_LN2_RX_CDR_AFC_PMS_M_SSP__7_0_SET(0x4);
	writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0789);
}

static void phy_exynos_usbdp_g2_v4_set_pcs(struct exynos_usbphy_info *info)
{
	void __iomem *regs_base = info->pcs_base;
	u32 reg;

	/* abnormal comman pattern mask */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_RX_BACK_END_MODE_VEC);
	reg &= USBDP_PCS_RX_BACK_END_MODE_VEC_DISABLE_DATA_MASK_CLR;
	writel(reg, regs_base + EXYNOS_USBDP_PCS_RX_BACK_END_MODE_VEC);

	/* De-serializer enabled when U2 */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_PM_OUT_VEC_2);
	reg &= USBDP_PCS_PM_OUT_VEC_2_B4_DYNAMIC_CLR;
	reg |= USBDP_PCS_PM_OUT_VEC_2_B4_SEL_OUT_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_PCS_PM_OUT_VEC_2);

	/* TX Keeper Disable, Squelch off when U3 */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_PM_OUT_VEC_3);
	reg &= USBDP_PCS_PM_OUT_VEC_3_B7_DYNAMIC_CLR;
	reg |= USBDP_PCS_PM_OUT_VEC_3_B7_SEL_OUT_SET(1);
	reg &= USBDP_PCS_PM_OUT_VEC_3_B2_SEL_OUT_CLR;
	/* Squelch on when U3 */
	reg |= USBDP_PCS_PM_OUT_VEC_3_B2_SEL_OUT_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_PCS_PM_OUT_VEC_3);

	/* 20180822 PCS SFR setting : Noh M.W */
	writel(0x05700000, regs_base + EXYNOS_USBDP_PCS_PM_NS_VEC_PS1_N1);
	writel(0x01700202, regs_base + EXYNOS_USBDP_PCS_PM_NS_VEC_PS2_N0);
	writel(0x01700707, regs_base + EXYNOS_USBDP_PCS_PM_NS_VEC_PS3_N0);
	writel(0x0070, regs_base + EXYNOS_USBDP_PCS_PM_TIMEOUT_0);

	/* Block Aligner Type B */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_RX_RX_CONTROL);
	reg &= USBDP_PCS_RX_RX_CONTROL_EN_BLOCK_ALIGNER_TYPE_B_CLR;
	reg |= USBDP_PCS_RX_RX_CONTROL_EN_BLOCK_ALIGNER_TYPE_B_SET(1);
	writel(reg, regs_base + EXYNOS_USBDP_PCS_RX_RX_CONTROL);

	/* Block align at TS1/TS2 for Gen2 stability (Gen2 only) */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_RX_RX_CONTROL_DEBUG);
	reg &= USBDP_PCS_RX_RX_CONTROL_DEBUG_EN_TS_CHECK_CLR;
	reg |= USBDP_PCS_RX_RX_CONTROL_DEBUG_EN_TS_CHECK_SET(1);
	/*
	 * increse pcs ts1 adding packet-cnt 1 --> 4
	 * lnx_rx_valid_rstn_delay_rise_sp/ssp :
	 * 19.6us(0x200) -> 15.3us(0x4)
	 */
	reg &= USBDP_PCS_RX_RX_CONTROL_DEBUG_NUM_COM_FOUND_CLR;
	reg |= USBDP_PCS_RX_RX_CONTROL_DEBUG_NUM_COM_FOUND_SET(4);
	writel(reg, regs_base + EXYNOS_USBDP_PCS_RX_RX_CONTROL_DEBUG);

	/* Gen1 Tx DRIVER pre-shoot, de-emphasis, level ctrl
	 * [15:12] de-emphasis
	 * [9:6] level - 0xb (max)
	 * [3:0] pre-shoot - Gen1 does not support pre-shoot
	 */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_LEQ_HS_TX_COEF_MAP_0);
	reg &= USBDP_PCS_LEQ_HS_TX_COEF_MAP_0_LEVEL_CLR;
	reg |= USBDP_PCS_LEQ_HS_TX_COEF_MAP_0_LEVEL_SET((8 << 12 |
			0xB << 6 | 0x0));
	writel(reg, regs_base + EXYNOS_USBDP_PCS_LEQ_HS_TX_COEF_MAP_0);

	/* Gen2 Tx DRIVER level ctrl */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_LEQ_LOCAL_COEF);
	reg &= USBDP_PCS_LEQ_LOCAL_COEF_PMA_CENTER_COEF_CLR;
	reg |= USBDP_PCS_LEQ_LOCAL_COEF_PMA_CENTER_COEF_SET(0xB);
	writel(reg, regs_base + EXYNOS_USBDP_PCS_LEQ_LOCAL_COEF);

	/* Gen2 U1 exit LFPS duration : 900ns ~ 1.2us */
	writel(0x1000, regs_base + EXYNOS_USBDP_PCS_PM_TIMEOUT_3);

	/* set skp_remove_th 0x2 -> 0x7 for avoiding retry problem. */
	reg = readl(regs_base + EXYNOS_USBDP_PCS_RX_EBUF_PARAM);
	reg &= USBDP_PCS_RX_EBUF_PARAM_SKP_REMOVE_TH_EMPTY_MODE_CLR;
	reg |= USBDP_PCS_RX_EBUF_PARAM_SKP_REMOVE_TH_EMPTY_MODE_SET(0x7);
	writel(reg, regs_base + EXYNOS_USBDP_PCS_RX_EBUF_PARAM);
}

static inline void
phy_exynos_usbdp_g2_v4_pma_lane_mux_sel(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;

	/* Lane configuration */
	reg = readl(regs_base + EXYNOS_USBDP_CMN_REG00B8);
	reg &= USBDP_CMN_REG00B8_LANE_MUX_SEL_DP_CLR;

	if (info->used_phy_port == 0)
		reg |= USBDP_CMN_REG00B8_LANE_MUX_SEL_DP_SET(0xc);
	else
		reg |= USBDP_CMN_REG00B8_LANE_MUX_SEL_DP_SET(0x3);
	writel(reg, regs_base + EXYNOS_USBDP_CMN_REG00B8);

	/*
	 * ln1_tx_rxd_en = 0
	 * ln3_tx_rxd_en = 0
	 */
	if (info->used_phy_port == 0) {
		/* ln3_tx_rxd_en = 0 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0413);
		reg &= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_COMP_EN_CLR;
		reg &= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_EN_CLR;
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0413);

		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0813);
		reg &= USBDP_TRSV_REG0813_OVRD_LN3_TX_RXD_COMP_EN_CLR;
		reg |= USBDP_TRSV_REG0813_OVRD_LN3_TX_RXD_COMP_EN_SET(1);
		reg &= USBDP_TRSV_REG0813_OVRD_LN3_TX_RXD_EN_CLR;
		reg |= USBDP_TRSV_REG0813_OVRD_LN3_TX_RXD_EN_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0813);
	} else {
		/* ln1_tx_rxd_en = 0 */
		reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG0413);
		reg &= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_COMP_EN_CLR;
		reg |= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_COMP_EN_SET(1);
		reg &= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_EN_CLR;
		reg |= USBDP_TRSV_REG0413_OVRD_LN1_TX_RXD_EN_SET(1);
		writel(reg, regs_base + EXYNOS_USBDP_TRSV_REG0413);
	}
}

static int
phy_exynos_usbdp_g2_v4_pma_check_pll_lock(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;
	u32 cnt;

	for (cnt = 1000; cnt != 0; cnt--) {
		reg = readl(regs_base + EXYNOS_USBDP_CMN_REG01C0);
		if ((reg & (USBDP_CMN_REG01C0_ANA_LCPLL_LOCK_DONE_MSK
			| USBDP_CMN_REG01C0_ANA_LCPLL_AFC_DONE_MSK))) {
			break;
		}
		udelay(1);
	}

	if (!cnt)
		return -1;

	return 0;
}

static int
phy_exynos_usbdp_g2_v4_pma_check_cdr_lock(struct exynos_usbphy_info
		*info)
{
	void __iomem *regs_base = info->pma_base;
	u32 reg;
	u32 cnt;

	if (info->used_phy_port == 0) {
		for (cnt = 1000; cnt != 0; cnt--) {
			reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG03C3);
			if ((reg &
			(USBDP_TRSV_REG03C3_LN0_MON_RX_CDR_LOCK_DONE_MSK |
			USBDP_TRSV_REG03C3_LN0_MON_RX_CDR_FLD_PLL_MODE_DONE_MSK
			| USBDP_TRSV_REG03C3_LN0_MON_RX_CDR_CAL_DONE_MSK
			| USBDP_TRSV_REG03C3_LN0_MON_RX_CDR_AFC_DONE_MSK))) {
				break;
			}
			udelay(1);
		}
	} else {
		for (cnt = 1000; cnt != 0; cnt--) {
			reg = readl(regs_base + EXYNOS_USBDP_TRSV_REG07C3);
			if ((reg &
			(USBDP_TRSV_REG07C3_LN2_MON_RX_CDR_LOCK_DONE_MSK |
			USBDP_TRSV_REG07C3_LN2_MON_RX_CDR_FLD_PLL_MODE_DONE_MSK
			| USBDP_TRSV_REG07C3_LN2_MON_RX_CDR_CAL_DONE_MSK
			| USBDP_TRSV_REG07C3_LN2_MON_RX_CDR_AFC_DONE_MSK))) {
				break;
			}
			udelay(1);
		}
	}

	if (!cnt)
		return -2;

	return 0;
}

int phy_exynos_usbdp_g2_v4_enable(struct exynos_usbphy_info *info)
{
	int ret = 0;
	int pll_ret = 0;
	int cdr_ret = 0;

	phy_exynos_usbdp_g2_v4_ctrl_pma_ready(info);
	phy_exynos_usbdp_g2_v4_aux_force_off(info);

	/* Our refclk is always 19.2 MHz on gs201. */
	/* pma default sfr updated for RefClk 19.2Mhz */
	phy_exynos_usbdp_g2_v4_pma_default_sfr_update_19_2Mhz(info);
	/* pma configuration for RefClk 19.2Mhz-Gen1 */
	phy_exynos_usbdp_g2_v4_pma_default_sfr_update_19_2Mhz_Gen1(info);
	/* pma configuration for RefClk 19.2Mhz-Gen2 */
	phy_exynos_usbdp_g2_v4_pma_default_sfr_update_19_2Mhz_Gen2(info);

	phy_exynos_usbdp_g2_v4_set_pcs(info);
	phy_exynos_usbdp_g2_v4_pma_lane_mux_sel(info);
	phy_exynos_usbdp_g2_v4_ctrl_pma_rst_release(info);

	pll_ret = phy_exynos_usbdp_g2_v4_pma_check_pll_lock(info);
	ret = pll_ret;
	if (!ret) {
		cdr_ret = phy_exynos_usbdp_g2_v4_pma_check_cdr_lock(info);
		ret = cdr_ret;
	}

	mdelay(10);

	pr_info("%s: pll_lock ret=%d cdr_lock ret=%d\n", __func__,
		pll_ret, cdr_ret);

	return ret;
}
