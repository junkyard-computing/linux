// SPDX-License-Identifier: GPL-2.0
/*
 * ODPM (on-device power meter) reader for the Google gs201 s2mpg13 sub-PMIC.
 *
 * The s2mpg13 METER block continuously low-pass-filters the instantaneous
 * power of up to 12 rails. Each channel's rail is selected by a MUXSEL register
 * and its 21-bit LPF sample is scaled by a per-rail resolution to milliwatts.
 * This driver reads that out and dumps it via debugfs
 * (<debugfs>/s2mpg13-powermeter/power).
 *
 * The MUXSEL->resolution table and the fixed-point (Q30) scaling are
 * transcribed from the AOSP s2mpg13-powermeter driver.
 *
 * NOTE: this only sees rails on the s2mpg13 *sub* PMIC; the main SoC rails
 * (CPU/GPU/MIF) are on the s2mpg12 main PMIC, which is not ported.
 *
 * Copyright 2021 Google LLC
 * Copyright 2026 Junkyard Computing
 */

#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/mfd/samsung/s2mpg13.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>

#define S2MPG13_METER_CHANNELS		12
#define S2MPG13_LPF_BUF			3	/* 3 bytes / 21-bit LPF sample */

/* METER_CTRL1. */
#define S2MPG13_METER_EN		BIT(0)
/* LPF mode (CTRL6 + CTRL7[3:0]): 0 = power, 1 = current, per channel. */
#define S2MPG13_LPF_MODE_HI_MASK	0x0f

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

struct s2mpg13_powermeter {
	struct device *dev;
	struct regmap *meter;
	struct dentry *debugfs;
	u8 muxsel[S2MPG13_METER_CHANNELS];
};

static int s2mpg13_power_show(struct seq_file *s, void *unused)
{
	struct s2mpg13_powermeter *pm = s->private;
	int i, ret;

	for (i = 0; i < S2MPG13_METER_CHANNELS; i++) {
		u8 buf[S2MPG13_LPF_BUF];
		u32 raw, res;
		u64 uw;

		ret = regmap_bulk_read(pm->meter,
				       S2MPG13_METER_LPF_DATA_CH0_1 +
					       S2MPG13_LPF_BUF * i,
				       buf, S2MPG13_LPF_BUF);
		if (ret)
			return ret;

		/* 21-bit LPF sample. */
		raw = buf[0] | (buf[1] << 8) | ((buf[2] & 0x1f) << 16);
		res = s2mpg13_muxsel_power_resolution(pm->muxsel[i]);
		/* mW(Q30) = raw * res; ->uW: *1000; ->int: >>30. */
		uw = res ? (((u64)raw * res * 1000) >> 30) : 0;

		seq_printf(s, "CH%-2d %-9s raw=0x%06x  %llu uW\n", i,
			   s2mpg13_muxsel_name(pm->muxsel[i]), raw, uw);
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
	pm->meter = dev_get_regmap(dev->parent, "meter");
	if (!pm->meter)
		return dev_err_probe(dev, -ENODEV, "no 'meter' regmap on parent\n");

	/* Select LPF power mode for all 12 channels (0 = power). */
	ret = regmap_write(pm->meter, S2MPG13_METER_CTRL6, 0x00);
	if (ret)
		return ret;
	ret = regmap_update_bits(pm->meter, S2MPG13_METER_CTRL7,
				 S2MPG13_LPF_MODE_HI_MASK, 0x00);
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

	pm->debugfs = debugfs_create_dir("s2mpg13-powermeter", NULL);
	debugfs_create_file("power", 0444, pm->debugfs, pm,
			    &s2mpg13_power_fops);

	platform_set_drvdata(pdev, pm);
	dev_info(dev, "s2mpg13 ODPM: %d channels\n", S2MPG13_METER_CHANNELS);
	return 0;
}

static void s2mpg13_powermeter_remove(struct platform_device *pdev)
{
	struct s2mpg13_powermeter *pm = platform_get_drvdata(pdev);

	debugfs_remove_recursive(pm->debugfs);
}

static const struct of_device_id s2mpg13_powermeter_of_match[] = {
	{ .compatible = "google,s2mpg13-powermeter" },
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

MODULE_DESCRIPTION("Google s2mpg13 sub-PMIC ODPM power meter");
MODULE_LICENSE("GPL");
