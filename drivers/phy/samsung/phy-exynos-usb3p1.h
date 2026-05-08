/* SPDX-License-Identifier: GPL-2.0 */
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

#ifndef DRIVER_USB_USBPHY_CAL_PHY_EXYNOS_USB3P1_H_
#define DRIVER_USB_USBPHY_CAL_PHY_EXYNOS_USB3P1_H_

/*
 * Mainline graft (felix gs201): pruned to the surface used by the
 * gs201_aosp_utmi_init wrapper in phy-exynos5-usbdrd.c. ReWA, BC 1.2,
 * DP altmode, host-mode helpers, and the SS+ CR-port path are dropped.
 */
void phy_exynos_usb_v3p1_enable(struct exynos_usbphy_info *info);
void phy_exynos_usb_v3p1_disable(struct exynos_usbphy_info *info);
u64 phy_exynos_usb3p1_get_logic_trace(struct exynos_usbphy_info *info);
void phy_exynos_usb_v3p1_link_sw_reset(struct exynos_usbphy_info *info);
/* USB/DP PHY control */
void phy_exynos_usb_v3p1_pma_ready(struct exynos_usbphy_info *info);
void phy_exynos_usb_v3p1_g2_pma_ready(struct exynos_usbphy_info *info);
void phy_exynos_usb_v3p1_g2_disable(struct exynos_usbphy_info *info);
void phy_exynos_usb_v3p1_pma_sw_rst_release(struct exynos_usbphy_info *info);
void phy_exynos_usb_v3p1_g2_link_pclk_sel(struct exynos_usbphy_info *info);
void phy_exynos_usb_v3p1_g2_pma_sw_rst_release(struct exynos_usbphy_info *info);
void phy_exynos_usb_v3p1_pipe_ovrd(struct exynos_usbphy_info *info);
void phy_exynos_usb_v3p1_pipe_ready(struct exynos_usbphy_info *info);
/* Tune */
void phy_exynos_usb_v3p1_tune(struct exynos_usbphy_info *info);
void phy_exynos_usb_v3p1_tune_each(struct exynos_usbphy_info *info,
				   char *para_name, int val);
void phy_exynos_usb_v3p1_rd_tune_each_from_reg(struct exynos_usbphy_info
			*info, u32 tune, char *para_name, int *val);
void phy_exynos_usb_v3p1_wr_tune_reg(struct exynos_usbphy_info *info, u32 val);
void phy_exynos_usb_v3p1_rd_tune_reg(struct exynos_usbphy_info *info, u32 *val);
/* 2.0 PHY Test I/F Access (used by phy_exynos_usb_v3p1_tune for tx_res_ovrd / tx_dis_inc) */
u8 phy_exynos_usb_v3p1_tif_ov_rd(struct exynos_usbphy_info *info, u8 addr);
u8 phy_exynos_usb_v3p1_tif_ov_wr(struct exynos_usbphy_info *info, u8 addr, u8 data);
u8 phy_exynos_usb_v3p1_tif_sts_rd(struct exynos_usbphy_info *info, u8 addr);
void phy_exynos_usb_v3p1_late_enable(struct exynos_usbphy_info *info);

/* Phase G.10 (felix gs201): force the unconditional CR-port write AOSP
 * gates on version > 0x500. See phy-exynos-usb3p1.c for rationale. */
int phy_exynos_usb_v3p1_force_gs201_cr_writes(struct exynos_usbphy_info *info);

#endif /* DRIVER_USB_USBPHY_CAL_PHY_EXYNOS_USB3P1_H_ */
