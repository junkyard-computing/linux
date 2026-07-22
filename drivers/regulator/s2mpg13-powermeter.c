// SPDX-License-Identifier: GPL-2.0
/*
 * ODPM (on-device power meter) reader for the Google gs201 PMICs: s2mpg13
 * (sub) and s2mpg12 (main).
 *
 * The METER block continuously low-pass-filters the instantaneous power of up
 * to 12 rails. Each channel's rail is selected by a MUXSEL register and its
 * 21-bit LPF sample is scaled by a per-rail resolution to milliwatts. This
 * driver reads that out and dumps it via debugfs
 * (<debugfs>/s2mpg1{2,3}-powermeter/power).
 *
 * The MUXSEL->resolution tables and the fixed-point (Q30) scaling come from
 * AOSP. The METER register block is byte-identical between the two chips, so
 * they share everything except the rail decode (see s2mpg1x_pm_variant).
 *
 * The sub PMIC carries peripheral rails (display/MIPI/UFS PLL, camera, GPU,
 * DDR); the main PMIC carries the SoC core rails -- VDD_MIF, VDD_CPUCL0/1/2,
 * VDD_INT, VDD_TPU, VDD_SLC -- which is where idle-power questions about the
 * CPU/memory subsystem actually get answered.
 *
 * Copyright 2021 Google LLC
 * Copyright 2026 Junkyard Computing
 */

#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/math64.h>
#include <linux/mfd/samsung/s2mpg12.h>
#include <linux/mfd/samsung/s2mpg13.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>

#define S2MPG13_METER_CHANNELS		12
#define S2MPG13_LPF_BUF			3	/* 3 bytes / 21-bit LPF sample */
/*
 * Accumulator path: 6 bytes of accumulated samples per channel plus a shared
 * 3-byte sample count. This is what the meter actually keeps up to date -- the
 * LPF_DATA registers this driver used to read are a snapshot that never
 * refreshed, so every channel returned a byte-identical value forever (caught
 * when S1S_VDD_CAM was switched off at the PMIC and its "power" did not move).
 * AOSP reads the same accumulators for its ODPM energy interface.
 */
#define S2MPG13_ACC_BUF			6
#define S2MPG13_ACC_COUNT_BUF		3

/* METER_CTRL1: meter enable plus the internal sampling rate. */
#define S2MPG13_METER_EN		BIT(0)
#define S2MPG13_INT_SAMP_RATE_MASK	(0x7 << 2)
#define S2MPG13_INT_SAMP_RATE_125HZ	(0x4 << 2)

/*
 * METER_CTRL2: write to latch the accumulators into the readable registers;
 * self-clearing.
 *
 * The latch does not complete until the meter finishes an internal sample, so
 * the wait is a function of the sampling rate set above, not a fixed constant
 * (AOSP s2mpg1x_meter_get_acquisition_time_us, b/209886118). At 125 Hz that is
 * 8 ms; poll in sixteenths of it as AOSP does, so a fast latch returns fast.
 */
#define S2MPG13_METER_ASYNC_RD		BIT(7)
#define S2MPG13_ACQUISITION_US		8000
#define S2MPG13_LATCH_POLLS		16
#define S2MPG13_LATCH_POLL_US		(S2MPG13_ACQUISITION_US / S2MPG13_LATCH_POLLS)
#define S2MPG13_RESET_SETTLE_US		200
/*
 * Power/current mode is per channel, one bit each, and is selected separately
 * for the two data paths: the accumulators (CTRL4 + CTRL5[3:0]) and the LPF
 * snapshot (CTRL6 + CTRL7[3:0]). 0 = power, 1 = current. We read accumulators,
 * so CTRL4/CTRL5 are the ones that matter; CTRL6/CTRL7 are set to match so the
 * LPF registers stay meaningful for anything that reads them.
 */
#define S2MPG13_MODE_HI_MASK		0x0f

/*
 * Per-rail power resolution in milliwatts/LSB, in Q30 fixed point (value * 2^30
 * rounded). Precomputed from the AOSP float constants to avoid FP in-kernel.
 */
#define CMS_BUCK_POWER			6555200u   /* 0.006105006 mW/LSB */
#define CMT_BUCK_POWER			19665601u  /* 0.018315018 mW/LSB */
#define VM_POWER			13110401u  /* 0.012210012 mW/LSB */
#define BB_POWER			11799361u  /* 0.010989011 mW/LSB */
#define DVS_NLDO_POWER_150mA		491640u    /* 0.000457875 mW/LSB */
#define PLDO_POWER_150mA		983280u    /* 0.000915751 mW/LSB */
#define NLDO_POWER_300mA		983280u    /* 0.000915751 mW/LSB */
#define PLDO_POWER_300mA		1966560u   /* 0.001831502 mW/LSB */
#define DVS_NLDO_POWER_800mA		1311040u   /* 0.001221001 mW/LSB */
#define NLDO_POWER_800mA		2622080u   /* 0.002442002 mW/LSB */
#define NLDO_POWER_1200mA		1522835u   /* 0.001418251 mW/LSB */

/* MUXSEL rail selectors (the subset s2mpg13 uses). */
enum s2mpg13_muxsel {
	MUXSEL_NONE	= 0x00,
	MUXSEL_BUCK1	= 0x01,
	MUXSEL_BUCK2	= 0x02,
	MUXSEL_BUCK3	= 0x03,
	MUXSEL_BUCK4	= 0x04,
	MUXSEL_BUCK5	= 0x05,
	MUXSEL_BUCK6	= 0x06,
	MUXSEL_BUCK7	= 0x07,
	MUXSEL_BUCK8	= 0x08,
	MUXSEL_BUCK9	= 0x09,
	MUXSEL_BUCK10	= 0x0a,
	MUXSEL_BUCKD	= 0x0b,
	MUXSEL_BUCKA	= 0x0c,
	MUXSEL_BUCKC	= 0x0d,
	MUXSEL_BUCKBOOST = 0x10,
	MUXSEL_VBAT	= 0x1f,
	MUXSEL_LDO1	= 0x21,
	MUXSEL_LDO2	= 0x22,
	MUXSEL_LDO3	= 0x23,
	MUXSEL_LDO4	= 0x24,
	MUXSEL_LDO5	= 0x25,
	MUXSEL_LDO6	= 0x26,
	MUXSEL_LDO7	= 0x27,
	MUXSEL_LDO8	= 0x28,
	MUXSEL_LDO9	= 0x29,
	MUXSEL_LDO10	= 0x2a,
	MUXSEL_LDO11	= 0x2b,
	MUXSEL_LDO12	= 0x2c,
	MUXSEL_LDO13	= 0x2d,
	MUXSEL_LDO14	= 0x2e,
	MUXSEL_LDO15	= 0x2f,
	MUXSEL_LDO16	= 0x30,
	MUXSEL_LDO17	= 0x31,
	MUXSEL_LDO18	= 0x32,
	MUXSEL_LDO19	= 0x33,
	MUXSEL_LDO20	= 0x34,
	MUXSEL_LDO21	= 0x35,
	MUXSEL_LDO22	= 0x36,
	MUXSEL_LDO23	= 0x37,
	MUXSEL_LDO24	= 0x38,
	MUXSEL_LDO25	= 0x39,
	MUXSEL_LDO26	= 0x3a,
	MUXSEL_LDO27	= 0x3b,
	MUXSEL_LDO28	= 0x3c,
	/* Voltage-sense channels (measure voltage, not power). */
	MUXSEL_VSEN1	= 0x5c,
	MUXSEL_VSEN2	= 0x5d,
	MUXSEL_VSEN3	= 0x5e,
};

/* Power resolution (mW/LSB, Q30) for a given rail selector. */
static u32 s2mpg13_muxsel_power_resolution(u8 m)
{
	switch (m) {
	case MUXSEL_BUCK3:
	case MUXSEL_BUCK4:
	case MUXSEL_BUCK5:
	case MUXSEL_BUCK6:
	case MUXSEL_BUCK8:
	case MUXSEL_BUCK9:
	case MUXSEL_BUCK10:
	case MUXSEL_BUCKC:
		return CMS_BUCK_POWER;
	case MUXSEL_BUCK1:
	case MUXSEL_BUCK2:
		return CMT_BUCK_POWER;
	case MUXSEL_BUCK7:
	case MUXSEL_BUCKD:
	case MUXSEL_BUCKA:
		return VM_POWER;
	case MUXSEL_BUCKBOOST:
		return BB_POWER;
	case MUXSEL_LDO9:
		return DVS_NLDO_POWER_150mA;
	case MUXSEL_LDO4:
	case MUXSEL_LDO5:
	case MUXSEL_LDO6:
	case MUXSEL_LDO7:
	case MUXSEL_LDO10:
	case MUXSEL_LDO11:
	case MUXSEL_LDO13:
	case MUXSEL_LDO15:
	case MUXSEL_LDO16:
	case MUXSEL_LDO17:
	case MUXSEL_LDO18:
	case MUXSEL_LDO19:
	case MUXSEL_LDO20:
	case MUXSEL_LDO22:
	case MUXSEL_LDO28:
		return PLDO_POWER_150mA;
	case MUXSEL_LDO26:
		return NLDO_POWER_300mA;
	case MUXSEL_LDO12:
	case MUXSEL_LDO14:
	case MUXSEL_LDO27:
		return PLDO_POWER_300mA;
	case MUXSEL_LDO1:
	case MUXSEL_LDO23:
	case MUXSEL_LDO24:
	case MUXSEL_LDO25:
		return DVS_NLDO_POWER_800mA;
	case MUXSEL_LDO2:
	case MUXSEL_LDO3:
	case MUXSEL_LDO21:
		return NLDO_POWER_800mA;
	case MUXSEL_LDO8:
		return NLDO_POWER_1200mA;
	default:
		return 0;
	}
}

static const char *s2mpg13_muxsel_name(u8 m)
{
	static const char * const bucks[] = {
		"none", "buck1", "buck2", "buck3", "buck4", "buck5", "buck6",
		"buck7", "buck8", "buck9", "buck10", "buckd", "bucka", "buckc",
	};

	if (m < ARRAY_SIZE(bucks))
		return bucks[m];
	if (m == MUXSEL_BUCKBOOST)
		return "buckboost";
	if (m == MUXSEL_VBAT)
		return "vbat";
	if (m >= MUXSEL_VSEN1 && m <= MUXSEL_VSEN3) {
		static const char * const vsen[] = { "vsen1", "vsen2", "vsen3" };

		return vsen[m - MUXSEL_VSEN1];
	}
	if (m >= MUXSEL_LDO1 && m <= MUXSEL_LDO28) {
		static char ldo[8];

		scnprintf(ldo, sizeof(ldo), "ldo%u", m - MUXSEL_LDO1 + 1);
		return ldo;
	}
	return "?";
}

/*
 * s2mpg12 (gs201 MAIN PMIC) power resolution, mW/LSB in the same Q30 encoding.
 *
 * Indexed by (muxsel - 1), transcribed verbatim from the table AOSP's shipped
 * s2mpg12-powermeter.ko uses in s2mpg12_muxsel_to_power_resolution() (the .c is
 * not vendored, so this was extracted from .rodata+0x190 of the prebuilt
 * module). Only BUCK1M..BUCK10M (0x01-0x0a) and LDO1M..LDO28M (0x21-0x3c) are
 * valid on this chip; everything else is 0 and reports as unmetered.
 */
static const u32 s2mpg12_power_res[0x3c] = {
	/* 0x01..0x0a: BUCK1M..BUCK10M */
	[0x01 - 1] = 6555200,	/* S1M_VDD_MIF */
	[0x02 - 1] = 19665600,	/* S2M_VDD_CPUCL2 */
	[0x03 - 1] = 13110400,	/* S3M_VDD_CPUCL1 */
	[0x04 - 1] = 6555200,	/* S4M_VDD_CPUCL0 */
	[0x05 - 1] = 19665600,	/* S5M_VDD_INT */
	[0x06 - 1] = 6555200,
	[0x07 - 1] = 6555200,
	[0x08 - 1] = 6555200,
	[0x09 - 1] = 6555200,
	[0x0a - 1] = 19665600,	/* S10M_VDD_TPU */
	/* 0x21..0x3c: LDO1M..LDO28M */
	[0x21 - 1] = 983280,   [0x22 - 1] = 5244160, [0x23 - 1] = 983280,
	[0x24 - 1] = 983280,   [0x25 - 1] = 491639,  [0x26 - 1] = 1474919,
	[0x27 - 1] = 1311039,  [0x28 - 1] = 983280,  [0x29 - 1] = 983280,
	[0x2a - 1] = 983280,   [0x2b - 1] = 1311039, [0x2c - 1] = 1311039,
	[0x2d - 1] = 1311039,  [0x2e - 1] = 983280,  [0x2f - 1] = 1311039,
	[0x30 - 1] = 2622079,  [0x31 - 1] = 1311039, [0x32 - 1] = 983280,
	[0x33 - 1] = 1311039,  [0x34 - 1] = 983280,  [0x35 - 1] = 1966560,
	[0x36 - 1] = 1311039,  [0x37 - 1] = 983280,  [0x38 - 1] = 3933120,
	[0x39 - 1] = 983280,   [0x3a - 1] = 1966560, [0x3b - 1] = 983280,
	[0x3c - 1] = 1474919,
};

static u32 s2mpg12_muxsel_power_resolution(u8 m)
{
	if (m < 1 || m > ARRAY_SIZE(s2mpg12_power_res))
		return 0;
	return s2mpg12_power_res[m - 1];
}

static const char *s2mpg12_muxsel_name(u8 m)
{
	static char buf[8];

	if (m == MUXSEL_NONE)
		return "none";
	if (m >= MUXSEL_BUCK1 && m <= MUXSEL_BUCK10) {
		scnprintf(buf, sizeof(buf), "buck%um", m - MUXSEL_BUCK1 + 1);
		return buf;
	}
	if (m >= MUXSEL_LDO1 && m <= MUXSEL_LDO28) {
		scnprintf(buf, sizeof(buf), "ldo%um", m - MUXSEL_LDO1 + 1);
		return buf;
	}
	if (m >= MUXSEL_VSEN1 && m <= MUXSEL_VSEN3) {
		static const char * const vsen[] = { "vsen1", "vsen2", "vsen3" };

		return vsen[m - MUXSEL_VSEN1];
	}
	return "?";
}

/*
 * Per-chip differences. The METER register block itself is byte-identical
 * between s2mpg12 and s2mpg13 (CTRL1 0x08, MUXSEL0 0x11, LPF_DATA_CH0 0xae),
 * so only the rail decode varies.
 */
struct s2mpg1x_pm_variant {
	const char *name;
	u32 (*power_resolution)(u8 muxsel);
	const char *(*muxsel_name)(u8 muxsel);
	/*
	 * Register in the MT_TRIM bank whose bit 7 is the meter software reset.
	 * The two PMICs put it at different offsets (s2mpg13 COMMON2 0x34,
	 * s2mpg12 COMMON 0x29), so the reset cannot hardcode one.
	 */
	unsigned int mt_trim_rst_reg;
};

static const struct s2mpg1x_pm_variant s2mpg13_pm_variant = {
	.name = "s2mpg13-powermeter",
	.power_resolution = s2mpg13_muxsel_power_resolution,
	.muxsel_name = s2mpg13_muxsel_name,
	.mt_trim_rst_reg = S2MPG13_MT_TRIM_COMMON2,
};

static const struct s2mpg1x_pm_variant s2mpg12_pm_variant = {
	.name = "s2mpg12-powermeter",
	.power_resolution = s2mpg12_muxsel_power_resolution,
	.muxsel_name = s2mpg12_muxsel_name,
	.mt_trim_rst_reg = S2MPG12_MT_TRIM_COMMON,
};

struct s2mpg13_powermeter {
	struct device *dev;
	struct regmap *meter;
	struct regmap *mt_trim;
	struct dentry *debugfs;
	const struct s2mpg1x_pm_variant *variant;
	u8 muxsel[S2MPG13_METER_CHANNELS];
};

/*
 * Latch a fresh sample set before reading.
 *
 * The LPF_DATA registers are NOT live: they are a snapshot that the meter only
 * refreshes from its internal accumulators when ASYNC_RD is written. Without
 * this the debugfs file happily returns a stale set forever -- every channel
 * byte-identical read after read, which is exactly how this was found: after
 * the regulator core switched S1S_VDD_CAM off at the PMIC (B1S_CTRL f8 -> 38),
 * the "measured" power for that rail did not budge from 62637 uW.
 *
 * Writing the bit transfers the accumulator data to the readable registers and
 * then self-clears, so poll for it to clear before reading. AOSP does the same
 * in s2mpg1x_meter_set_async_blocking(): the wait is bounded by one internal
 * sample period, since the data and the sample count are not updated at the
 * same instant and reading between them would mismatch.
 */
/*
 * Bring the meter out of the state firmware leaves it in.
 *
 * The software reset lives in the MT_TRIM bank rather than the meter bank:
 * drop bit 7 of the variant's MT_TRIM reset register and raise it again (AOSP
 * s2mpg1x_meter_sw_reset). Toggling METER_EN alone is not enough -- doing only
 * that leaves the accumulators frozen with acc_count pinned at 0xfffff.
 */
static int s2mpg1x_meter_reset(struct s2mpg13_powermeter *pm)
{
	unsigned int reg = pm->variant->mt_trim_rst_reg;
	int ret;

	if (!pm->mt_trim)
		return 0;

	ret = regmap_update_bits(pm->mt_trim, reg,
				 S2MPG13_MT_TRIM_METER_SW_RST, 0);
	if (ret)
		return ret;

	ret = regmap_update_bits(pm->mt_trim, reg,
				 S2MPG13_MT_TRIM_METER_SW_RST,
				 S2MPG13_MT_TRIM_METER_SW_RST);
	if (ret)
		return ret;

	usleep_range(S2MPG13_RESET_SETTLE_US, S2MPG13_RESET_SETTLE_US + 100);
	return 0;
}

static int s2mpg1x_meter_latch(struct s2mpg13_powermeter *pm)
{
	unsigned int val;
	int i, ret;

	ret = regmap_update_bits(pm->meter, S2MPG13_METER_CTRL2,
				 S2MPG13_METER_ASYNC_RD,
				 S2MPG13_METER_ASYNC_RD);
	if (ret)
		return ret;

	/* Typically complete well inside the first poll. */
	for (i = 0; i < S2MPG13_LATCH_POLLS; i++) {
		ret = regmap_read(pm->meter, S2MPG13_METER_CTRL2, &val);
		if (ret)
			return ret;
		if (!(val & S2MPG13_METER_ASYNC_RD))
			return 0;
		usleep_range(S2MPG13_LATCH_POLL_US,
			     S2MPG13_LATCH_POLL_US + 100);
	}

	return -ETIMEDOUT;
}

static int s2mpg13_power_show(struct seq_file *s, void *unused)
{
	struct s2mpg13_powermeter *pm = s->private;
	u8 cnt_buf[S2MPG13_ACC_COUNT_BUF];
	u32 acc_count;
	int i, ret;

	/*
	 * Do NOT reset here: the reset clears the meter's configuration, and
	 * accumulators are meant to run continuously. Each read is a latched
	 * snapshot of the running totals -- difference two reads to get power
	 * over that interval, which is also how the AOSP ODPM interface is used.
	 */
	ret = s2mpg1x_meter_latch(pm);
	if (ret) {
		dev_warn_once(pm->dev,
			      "meter latch failed (%d); readings would be stale\n",
			      ret);
		return ret;
	}

	/*
	 * Sample count shared by every channel. Read it once, after the latch,
	 * so it matches the accumulator snapshot.
	 */
	ret = regmap_bulk_read(pm->meter, S2MPG13_METER_ACC_COUNT_1, cnt_buf,
			       S2MPG13_ACC_COUNT_BUF);
	if (ret)
		return ret;
	acc_count = cnt_buf[0] | (cnt_buf[1] << 8) | (cnt_buf[2] << 16);

	seq_printf(s, "acc_count=%u\n", acc_count);

	for (i = 0; i < S2MPG13_METER_CHANNELS; i++) {
		u8 buf[S2MPG13_ACC_BUF];
		u64 acc = 0;
		u32 res;
		u64 uw = 0;
		int b;

		ret = regmap_bulk_read(pm->meter,
				       S2MPG13_METER_ACC_DATA_CH0_1 +
					       S2MPG13_ACC_BUF * i,
				       buf, S2MPG13_ACC_BUF);
		if (ret)
			return ret;

		for (b = 0; b < S2MPG13_ACC_BUF; b++)
			acc |= (u64)buf[b] << (8 * b);

		res = pm->variant->power_resolution(pm->muxsel[i]);
		/*
		 * Mean power over the accumulation window:
		 *   mW(Q30) = acc / acc_count * resolution
		 * Scale by 1000 before dividing to keep sub-uW precision. The
		 * per-sample mean is ~21 bits, so (mean * 1000) * res stays well
		 * inside u64; acc alone would not.
		 */
		if (res && acc_count) {
			u64 mean_m = div64_u64(acc * 1000, acc_count);

			uw = (mean_m * res) >> 30;
		}

		seq_printf(s, "CH%-2d %-9s acc=0x%012llx  %llu uW\n", i,
			   pm->variant->muxsel_name(pm->muxsel[i]), acc, uw);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(s2mpg13_power);

static int s2mpg13_powermeter_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpg13_powermeter *pm;
	unsigned int val;
	int i, ret, n;

	pm = devm_kzalloc(dev, sizeof(*pm), GFP_KERNEL);
	if (!pm)
		return -ENOMEM;

	pm->dev = dev;
	pm->variant = of_device_get_match_data(dev);
	if (!pm->variant)
		return dev_err_probe(dev, -ENODEV, "no variant match data\n");
	pm->meter = dev_get_regmap(dev->parent, "meter");
	if (!pm->meter)
		return dev_err_probe(dev, -ENODEV, "no 'meter' regmap on parent\n");
	/*
	 * Optional so a variant without it still probes, but without the MT_TRIM
	 * bank the meter cannot be reset and every reading is a frozen snapshot,
	 * so say so loudly rather than reporting fiction.
	 */
	pm->mt_trim = dev_get_regmap(dev->parent, "mt_trim");
	if (!pm->mt_trim)
		dev_warn(dev,
			 "no 'mt_trim' regmap: meter cannot be reset, readings will be stale\n");

	/*
	 * Reset the meter BEFORE configuring it. Without this the block never
	 * samples at all: its LPF and accumulator registers stay frozen and
	 * even survive a reboot unchanged. The reset also clears the meter's
	 * configuration, which is why it must come first -- everything below
	 * reprograms it, and resetting afterwards would undo that.
	 */
	ret = s2mpg1x_meter_reset(pm);
	if (ret)
		return dev_err_probe(dev, ret, "meter reset failed\n");

	/*
	 * Set the internal sampling rate explicitly. Firmware does not leave a
	 * usable one behind, and the rate sets how long a latch takes, so this
	 * has to agree with S2MPG13_ACQUISITION_US above. 125 Hz is what AOSP
	 * picks for this PMIC.
	 */
	ret = regmap_update_bits(pm->meter, S2MPG13_METER_CTRL1,
				 S2MPG13_INT_SAMP_RATE_MASK,
				 S2MPG13_INT_SAMP_RATE_125HZ);
	if (ret)
		return ret;

	/* Power mode (0) for all 12 channels, on both the ACC and LPF paths. */
	ret = regmap_write(pm->meter, S2MPG13_METER_CTRL4, 0x00);
	if (ret)
		return ret;
	ret = regmap_update_bits(pm->meter, S2MPG13_METER_CTRL5,
				 S2MPG13_MODE_HI_MASK, 0x00);
	if (ret)
		return ret;
	ret = regmap_write(pm->meter, S2MPG13_METER_CTRL6, 0x00);
	if (ret)
		return ret;
	ret = regmap_update_bits(pm->meter, S2MPG13_METER_CTRL7,
				 S2MPG13_MODE_HI_MASK, 0x00);
	if (ret)
		return ret;

	/* Make sure the meter is running. */
	ret = regmap_update_bits(pm->meter, S2MPG13_METER_CTRL1,
				 S2MPG13_METER_EN, S2MPG13_METER_EN);
	if (ret)
		return ret;

	/*
	 * Program the per-channel rail selection. Firmware leaves the MUXSEL
	 * registers cleared on this port, so unless DT assigns rails every
	 * channel reads "none". "google,channel-muxsel" is a list of up to 12
	 * MUXSEL codes (see enum s2mpg13_muxsel), one per channel; absent means
	 * leave whatever firmware set.
	 */
	n = device_property_count_u8(dev, "google,channel-muxsel");
	if (n > 0) {
		u8 cfg[S2MPG13_METER_CHANNELS];

		n = min(n, S2MPG13_METER_CHANNELS);
		ret = device_property_read_u8_array(dev, "google,channel-muxsel",
						    cfg, n);
		if (ret)
			return dev_err_probe(dev, ret,
					     "bad google,channel-muxsel\n");
		for (i = 0; i < n; i++) {
			ret = regmap_write(pm->meter,
					   S2MPG13_METER_MUXSEL0 + i, cfg[i]);
			if (ret)
				return ret;
		}
	}

	/* Read back the (now-configured) rail selection for reporting. */
	for (i = 0; i < S2MPG13_METER_CHANNELS; i++) {
		ret = regmap_read(pm->meter, S2MPG13_METER_MUXSEL0 + i, &val);
		if (ret)
			return ret;
		pm->muxsel[i] = val;
	}

	pm->debugfs = debugfs_create_dir(pm->variant->name, NULL);
	debugfs_create_file("power", 0444, pm->debugfs, pm,
			    &s2mpg13_power_fops);

	platform_set_drvdata(pdev, pm);
	dev_info(dev, "%s ODPM: %d channels\n", pm->variant->name,
		 S2MPG13_METER_CHANNELS);
	return 0;
}

static void s2mpg13_powermeter_remove(struct platform_device *pdev)
{
	struct s2mpg13_powermeter *pm = platform_get_drvdata(pdev);

	debugfs_remove_recursive(pm->debugfs);
}

static const struct of_device_id s2mpg13_powermeter_of_match[] = {
	{ .compatible = "google,s2mpg13-powermeter",
	  .data = &s2mpg13_pm_variant },
	{ .compatible = "google,s2mpg12-powermeter",
	  .data = &s2mpg12_pm_variant },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpg13_powermeter_of_match);

static struct platform_driver s2mpg13_powermeter_driver = {
	.probe = s2mpg13_powermeter_probe,
	.remove = s2mpg13_powermeter_remove,
	.driver = {
		.name = "s2mpg13-powermeter",
		.of_match_table = s2mpg13_powermeter_of_match,
	},
};
module_platform_driver(s2mpg13_powermeter_driver);

MODULE_DESCRIPTION("Google s2mpg12/s2mpg13 gs201 ODPM power meter");
MODULE_LICENSE("GPL");
