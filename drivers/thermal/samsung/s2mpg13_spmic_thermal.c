// SPDX-License-Identifier: GPL-2.0
/*
 * NTC thermistor thermal-sensor driver for the Google gs201 (Tensor G2)
 * s2mpg13 sub-PMIC.
 *
 * The s2mpg13 METER block samples up to 8 off-die NTC thermistors placed
 * around the board (skin, USB, display, GNSS, ...) and low-pass-filters each
 * into a 12-bit code in the METER register bank (read over ACPM via the "meter"
 * regmap the sec-acpm MFD exposes). This driver converts those codes to
 * temperatures and presents them to the Linux thermal core so DT thermal-zones
 * (notably skin_therm, the board-surface backstop) can act on them.
 *
 * The code->millidegC map and its interpolation are transcribed from the AOSP
 * s2mpg13_spmic_thermal driver.
 *
 * Copyright 2021 Google LLC
 * Copyright 2026 Junkyard Computing
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/mfd/samsung/s2mpg13.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

/* s2mpg13 exposes 8 NTC channels; 2 bytes of LPF data per channel. */
#define S2MPG13_NTC_CHANNELS	8
#define S2MPG13_NTC_BUF		2

/* METER_CTRL1 fields. */
#define S2MPG13_METER_EN		BIT(0)
#define S2MPG13_NTC_SAMP_RATE_MASK	GENMASK(7, 5)
#define S2MPG13_NTC_0P15625HZ		1

struct s2mpg13_adc_map_pt {
	int volt;	/* 12-bit ADC code */
	int temp;	/* milli-degrees C */
};

/*
 * Lookup table, descending in ADC code / ascending in temperature. Transcribed
 * verbatim from AOSP; s2mpg13_map_volt_temp() interpolates between points.
 */
static const struct s2mpg13_adc_map_pt s2mpg13_adc_map[] = {
	{ 0xF8D, -26428 }, { 0xF6A, -21922 }, { 0xF29, -15958 },
	{ 0xEE4, -11060 }, { 0xE9D, -6890 },  { 0xE3F, -2264 },
	{ 0xDBF, 2961 },   { 0xD33, 7818 },   { 0xC97, 12525 },
	{ 0xBF5, 16945 },  { 0xB3A, 21623 },  { 0xA42, 27431 },
	{ 0x7F1, 40631 },  { 0x734, 44960 },  { 0x66B, 49757 },
	{ 0x5A3, 54854 },  { 0x4EE, 59898 },  { 0x446, 65076 },
	{ 0x43A, 65779 },  { 0x430, 65856 },  { 0x3C3, 69654 },
	{ 0x3BD, 69873 },  { 0x33B, 74910 },  { 0x2BB, 80691 },
	{ 0x259, 85844 },  { 0x206, 90915 },  { 0x1CE, 94873 },
	{ 0x191, 99720 },  { 0x160, 104216 }, { 0x12E, 109531 },
	{ 0xF9, 116445 },  { 0xD7, 121600 },  { 0x9F, 131839 },
};

static int s2mpg13_map_volt_temp(int input)
{
	int low = 0;
	int high = ARRAY_SIZE(s2mpg13_adc_map) - 1;
	int mid;

	if (s2mpg13_adc_map[low].volt <= input)
		return s2mpg13_adc_map[low].temp;
	if (s2mpg13_adc_map[high].volt >= input)
		return s2mpg13_adc_map[high].temp;

	/* Binary search; the result lands between index low and low - 1. */
	while (low <= high) {
		mid = (low + high) / 2;
		if (s2mpg13_adc_map[mid].volt < input)
			high = mid - 1;
		else if (s2mpg13_adc_map[mid].volt > input)
			low = mid + 1;
		else
			return s2mpg13_adc_map[mid].temp;
	}

	return s2mpg13_adc_map[low].temp +
	       mult_frac(s2mpg13_adc_map[low - 1].temp - s2mpg13_adc_map[low].temp,
			 input - s2mpg13_adc_map[low].volt,
			 s2mpg13_adc_map[low - 1].volt - s2mpg13_adc_map[low].volt);
}

struct s2mpg13_spmic_thermal;

struct s2mpg13_spmic_sensor {
	struct s2mpg13_spmic_thermal *chip;
	unsigned int adc_chan;
	struct thermal_zone_device *tzd;
};

struct s2mpg13_spmic_thermal {
	struct device *dev;
	struct regmap *meter;
	struct s2mpg13_spmic_sensor sensors[S2MPG13_NTC_CHANNELS];
};

static int s2mpg13_spmic_read_raw(struct s2mpg13_spmic_sensor *s, int *raw)
{
	unsigned int reg = S2MPG13_METER_LPF_DATA_NTC0_1 +
			   S2MPG13_NTC_BUF * s->adc_chan;
	u8 buf[S2MPG13_NTC_BUF];
	int ret;

	ret = regmap_bulk_read(s->chip->meter, reg, buf, S2MPG13_NTC_BUF);
	if (ret)
		return ret;

	/* 12-bit code: low byte + low nibble of the high byte. */
	*raw = buf[0] + ((buf[1] & 0xf) << 8);
	return 0;
}

static int s2mpg13_spmic_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct s2mpg13_spmic_sensor *s = thermal_zone_device_priv(tz);
	int ret, raw;

	ret = s2mpg13_spmic_read_raw(s, &raw);
	if (ret)
		return ret;

	/* A zero code means the channel is not sampling (not enabled). */
	if (!raw)
		return -ENODATA;

	*temp = s2mpg13_map_volt_temp(raw);
	return 0;
}

static const struct thermal_zone_device_ops s2mpg13_spmic_ops = {
	.get_temp = s2mpg13_spmic_get_temp,
};

/*
 * Bring the METER block up and start the NTC channels: set the NTC sample rate,
 * unmask the channels in CTRL3, and make sure the meter is running
 * (CTRL1.METER_EN). The LPF data takes a few sample periods to settle from zero.
 *
 * The s2mpg13-powermeter driver also enables the meter (for the power channels)
 * and probes after this one, where it does a full MT_TRIM software reset. That
 * reset does not touch CTRL3, so the NTC channels enabled here survive it; the
 * two drivers own disjoint fields of CTRL1 (NTC rate [7:5] here, INT rate [4:2]
 * there) and only share METER_EN, which is idempotent.
 */
static int s2mpg13_spmic_enable_ntc(struct s2mpg13_spmic_thermal *chip,
				    u8 adc_chan_en)
{
	int ret;

	ret = regmap_update_bits(chip->meter, S2MPG13_METER_CTRL1,
				 S2MPG13_NTC_SAMP_RATE_MASK,
				 FIELD_PREP(S2MPG13_NTC_SAMP_RATE_MASK,
					    S2MPG13_NTC_0P15625HZ));
	if (ret)
		return ret;

	ret = regmap_write(chip->meter, S2MPG13_METER_CTRL3, adc_chan_en);
	if (ret)
		return ret;

	ret = regmap_update_bits(chip->meter, S2MPG13_METER_CTRL1,
				 S2MPG13_METER_EN, S2MPG13_METER_EN);
	if (ret)
		return ret;

	/* Give the low-pass filter a moment to start producing data. */
	msleep(100);
	return 0;
}

static int s2mpg13_spmic_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpg13_spmic_thermal *chip;
	unsigned int registered = 0;
	int i, ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;

	/* The METER regmap is attached to the sec-acpm parent by the MFD core. */
	chip->meter = dev_get_regmap(dev->parent, "meter");
	if (!chip->meter)
		return dev_err_probe(dev, -ENODEV,
				     "no 'meter' regmap on parent\n");

	/* Enable all 8 NTC channels; unreferenced ones just go unread. */
	ret = s2mpg13_spmic_enable_ntc(chip, GENMASK(S2MPG13_NTC_CHANNELS - 1, 0));
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable NTC channels\n");

	for (i = 0; i < S2MPG13_NTC_CHANNELS; i++) {
		struct s2mpg13_spmic_sensor *s = &chip->sensors[i];
		struct thermal_zone_device *tzd;
		int raw;

		s->chip = chip;
		s->adc_chan = i;

		tzd = devm_thermal_of_zone_register(dev, i, s, &s2mpg13_spmic_ops);
		if (IS_ERR(tzd)) {
			/* No DT thermal-zone references this channel. */
			if (PTR_ERR(tzd) == -ENODEV)
				continue;
			return dev_err_probe(dev, PTR_ERR(tzd),
					     "failed to register NTC channel %d\n",
					     i);
		}
		s->tzd = tzd;
		registered++;

		if (!s2mpg13_spmic_read_raw(s, &raw))
			dev_info(dev, "NTC%d raw=0x%03x%s\n", i, raw,
				 raw ? "" : " (channel idle / not enabled)");
	}

	if (!registered)
		return dev_err_probe(dev, -ENODEV,
				     "no NTC channel referenced by a thermal-zone\n");

	dev_info(dev, "s2mpg13 spmic-thermal: %u NTC sensors\n", registered);
	return 0;
}

static const struct of_device_id s2mpg13_spmic_thermal_of_match[] = {
	{ .compatible = "google,s2mpg13-spmic-thermal" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpg13_spmic_thermal_of_match);

static struct platform_driver s2mpg13_spmic_thermal_driver = {
	.probe = s2mpg13_spmic_thermal_probe,
	.driver = {
		.name = "s2mpg13-spmic-thermal",
		.of_match_table = s2mpg13_spmic_thermal_of_match,
	},
};
module_platform_driver(s2mpg13_spmic_thermal_driver);

MODULE_DESCRIPTION("Google s2mpg13 sub-PMIC NTC thermal sensor driver");
MODULE_LICENSE("GPL");
