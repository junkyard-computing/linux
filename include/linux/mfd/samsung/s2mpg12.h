/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 Google Inc
 * Copyright 2026 Junkyard Computing
 *
 * Register map for the S2MPG12 MAIN PMIC (Google Tensor gs201 / Pixel Fold).
 * Transcribed from the AOSP s2mpg12-register.h into the mainline s2mpg1x
 * per-bank-offset scheme: the ACPM regmap selects the bank (COMMON/PMIC/METER)
 * and the enum value is the register offset within that bank.
 *
 * SCOPE / SAFETY: only the METER bank is actually consumed, by the ODPM power
 * meter. The main PMIC owns the CPU/GPU/MIF/TPU supply rails, so this chip
 * deliberately registers NO regulator cell -- letting the regulator core see
 * (and potentially disable, or reap as "unused") S1M_VDD_MIF or S4M_VDD_CPUCL0
 * would kill the SoC. The PMIC bank is declared only so the regmap bound is
 * correct; nothing writes it.
 *
 * The METER bank layout is byte-identical to the S2MPG13 sub-PMIC (verified
 * against AOSP s2mpg12-register.h vs s2mpg13-register.h: CTRL1 = 0x08,
 * MUXSEL0 = 0x11, LPF_DATA_CH0_1 = 0xae), which is why the two chips share one
 * power-meter driver. What differs is the per-rail resolution table.
 */

#ifndef __LINUX_MFD_S2MPG12_H
#define __LINUX_MFD_S2MPG12_H

/*
 * Meter trim bank (0x0e). Bit 7 of COMMON is the meter software reset; the
 * power meter does not sample until it is toggled (AOSP s2mpg1x_meter_sw_reset).
 * The main PMIC puts it at COMMON (0x29), where the sub-PMIC uses COMMON2
 * (0x34) -- different offsets, same bit.
 */
#define S2MPG12_MT_TRIM_COMMON		0x29
#define S2MPG12_MT_TRIM_METER_SW_RST	BIT(7)

/* Common registers (bank 0x000) */
enum s2mpg12_common_reg {
	S2MPG12_COMMON_VGPIO0,
	S2MPG12_COMMON_VGPIO1,
	S2MPG12_COMMON_VGPIO2,
	S2MPG12_COMMON_VGPIO3,
	S2MPG12_COMMON_I3C_DAA,
	S2MPG12_COMMON_IBI0,
	S2MPG12_COMMON_IBI1,
	S2MPG12_COMMON_IBI2,
	S2MPG12_COMMON_IBI3,
	S2MPG12_COMMON_CHIPID = 0x0b,
	S2MPG12_COMMON_I3C_CFG1 = 0x0c,
	S2MPG12_COMMON_I3C_CFG2 = 0x0d,
	S2MPG12_COMMON_I3C_STA = 0x0e,
	S2MPG12_COMMON_IBIM1 = 0x0f,
	S2MPG12_COMMON_IBIM2 = 0x10,
};

/*
 * PMIC registers (bank 0x100). Declared for the regmap bound only -- see the
 * SCOPE note above. The main-PMIC bucks are the SoC core rails.
 */
enum s2mpg12_pmic_reg {
	S2MPG12_PMIC_INT1,
	S2MPG12_PMIC_SW_RESET = 0xec,	/* last PMIC register */
};

/* Meter registers (bank 0xa00) */
enum s2mpg12_meter_reg {
	S2MPG12_METER_INT1,
	S2MPG12_METER_INT2,
	S2MPG12_METER_INT3,
	S2MPG12_METER_INT4,
	S2MPG12_METER_INT1M,
	S2MPG12_METER_INT2M,
	S2MPG12_METER_INT3M,
	S2MPG12_METER_INT4M,
	S2MPG12_METER_CTRL1,		/* 0x08 - meter enable */
	S2MPG12_METER_CTRL2,
	S2MPG12_METER_CTRL3,
	S2MPG12_METER_CTRL4,
	S2MPG12_METER_CTRL5,
	S2MPG12_METER_CTRL6,		/* 0x0d - LPF power/current mode */
	S2MPG12_METER_CTRL7,
	/* Per-channel rail selectors, MUXSEL0..11 = 0x11..0x1c. */
	S2MPG12_METER_MUXSEL0 = 0x11,
	S2MPG12_METER_MUXSEL1,
	S2MPG12_METER_MUXSEL2,
	S2MPG12_METER_MUXSEL3,
	S2MPG12_METER_MUXSEL4,
	S2MPG12_METER_MUXSEL5,
	S2MPG12_METER_MUXSEL6,
	S2MPG12_METER_MUXSEL7,
	S2MPG12_METER_MUXSEL8,
	S2MPG12_METER_MUXSEL9,
	S2MPG12_METER_MUXSEL10,
	S2MPG12_METER_MUXSEL11,
	/* 12 x 3-byte 21-bit LPF samples, CH0..CH11 = 0xae..0xd1. */
	S2MPG12_METER_LPF_DATA_CH0_1 = 0xae,
	S2MPG12_METER_VBAT_DATA1 = 0xd2,
	S2MPG12_METER_VBAT_DATA2 = 0xd3,
	S2MPG12_METER_EXT_SIGNED_DATA1 = 0xe4,
	S2MPG12_METER_EXT_SIGNED_DATA2 = 0xe5,
};

#endif /* __LINUX_MFD_S2MPG12_H */
