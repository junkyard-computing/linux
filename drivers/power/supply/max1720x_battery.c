// SPDX-License-Identifier: GPL-2.0+
/*
 * Fuel gauge driver for Maxim 17201/17205
 *
 * based on max1721x_battery.c
 *
 * Copyright (C) 2024 Liebherr-Electronics and Drives GmbH
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include <linux/unaligned.h>

/* SBS compliant registers */
#define MAX172XX_TEMP1			0x34
#define MAX172XX_INT_TEMP		0x35
#define MAX172XX_TEMP2			0x3B

/* Nonvolatile registers */
#define MAX1720X_NXTABLE0		0x80
#define MAX1720X_NRSENSE		0xCF	/* RSense in 10^-5 Ohm */
#define MAX1720X_NDEVICE_NAME4		0xDF

/* ModelGauge m5 */
#define MAX172XX_STATUS			0x00	/* Status */
#define MAX172XX_STATUS_BAT_ABSENT	BIT(3)	/* Battery absent */
#define MAX172XX_STATUS_IMX		BIT(6)	/* Maximum Current Alert Threshold Exceeded */
#define MAX172XX_STATUS_VMN		BIT(8)	/* Minimum Voltage Alert Threshold Exceeded */
#define MAX172XX_STATUS_TMN		BIT(9)	/* Minimum Temperature Alert Threshold Exceeded */
#define MAX172XX_STATUS_VMX		BIT(12)	/* Maximum Voltage Alert Threshold Exceeded */
#define MAX172XX_STATUS_TMX		BIT(13)	/* Maximum Temperature Alert Threshold Exceeded */
#define MAX172XX_REPCAP			0x05	/* Average capacity */
#define MAX172XX_REPSOC			0x06	/* Percentage of charge */
#define MAX172XX_TEMP			0x08	/* Temperature */
#define MAX172XX_VCELL			0x09	/* Per-cell voltage */
#define MAX172XX_CURRENT		0x0A	/* Actual current */
#define MAX172XX_AVG_CURRENT		0x0B	/* Average current */
#define MAX172XX_FULL_CAP		0x10	/* Calculated full capacity */
#define MAX172XX_TTE			0x11	/* Time to empty */
#define MAX172XX_AVG_TA			0x16	/* Average temperature */
#define MAX172XX_CYCLES			0x17
#define MAX172XX_DESIGN_CAP		0x18	/* Design capacity */
#define MAX172XX_AVG_VCELL		0x19
#define MAX172XX_TTF			0x20	/* Time to full */
#define MAX172XX_DEV_NAME		0x21	/* Device name */
#define MAX172XX_DEV_NAME_TYPE_MASK	GENMASK(3, 0)
#define MAX172XX_DEV_NAME_TYPE_MAX17201	BIT(0)
#define MAX172XX_DEV_NAME_TYPE_MAX17205	(BIT(0) | BIT(2))
#define MAX172XX_QR_TABLE10		0x22
#define MAX172XX_CONFIG			0x1D	/* Config */
#define MAX172XX_CONFIG_TEX		BIT(8)	/* Temp measured externally (host-written) */
#define MAX172XX_TGAIN			0x2C	/* Thermistor gain */
#define MAX172XX_TOFF			0x2D	/* Thermistor offset */
#define MAX172XX_BATT			0xDA	/* Battery voltage */
#define MAX172XX_ATAVCAP		0xDF
#define MAX172XX_TCURVE			0xB9	/* Thermistor curve */

/*
 * ModelGauge m5 custom-model load registers, used only to reprogram the
 * characterized model on a gauge with no nvmem companion (the felix base pack).
 * These live outside the normal driver's restricted register map, so the reload
 * uses a private permissive regmap (max1720x_m5load_regmap_cfg).
 */
#define MAX172XX_STATUS_POR		BIT(1)	/* Power-on reset */
#define MAX172XX_M5_ATRATE		0x04
#define MAX172XX_M5_REPCAP		0x05
#define MAX172XX_M5_QRTABLE00		0x12
#define MAX172XX_M5_FULLSOCTHR		0x13
#define MAX172XX_M5_ICHGTERM		0x1E
#define MAX172XX_M5_FULLCAPNOM		0x23
#define MAX172XX_M5_LEARNCFG		0x28
#define MAX172XX_M5_FILTERCFG		0x29
#define MAX172XX_M5_RELAXCFG		0x2A
#define MAX172XX_M5_MISCCFG		0x2B
#define MAX172XX_M5_QRTABLE20		0x32
#define MAX172XX_M5_FULLCAPREP		0x35
#define MAX172XX_M5_RCOMP0		0x38
#define MAX172XX_M5_TEMPCO		0x39
#define MAX172XX_M5_VEMPTY		0x3A
#define MAX172XX_M5_TASKPERIOD		0x3C
#define MAX172XX_M5_FSTAT		0x3D
#define MAX172XX_M5_FSTAT_DNR		BIT(0)	/* Data not ready */
#define MAX172XX_M5_QRTABLE30		0x42
#define MAX172XX_M5_DQACC		0x45
#define MAX172XX_M5_DPACC		0x46
#define MAX172XX_M5_VFSOC0		0x48
#define MAX172XX_M5_CONVGCFG		0x49
#define MAX172XX_M5_UNLOCK_EXTRA		0x60	/* extra-config unlock/command */
#define MAX172XX_M5_UNLOCK_EXTRA_CODE	0x0080
#define MAX172XX_M5_LOCK_EXTRA_CODE	0x0000
#define MAX172XX_M5_UNLOCK_MODEL0	0x62	/* model-access unlock word 0 */
#define MAX172XX_M5_UNLOCK_MODEL1	0x63	/* model-access unlock word 1 */
#define MAX172XX_M5_FG_MODEL_START	0x80
#define MAX172XX_M5_FG_MODEL_COUNT	48
#define MAX172XX_M5_CV_MIXCAP		0xB6
#define MAX172XX_M5_CV_HALFTIME		0xB7
#define MAX172XX_M5_CONFIG2		0xBB
#define MAX172XX_M5_CONFIG2_LDMDL	BIT(5)	/* load model in progress */
#define MAX172XX_M5_VFSOC		0xFF

static const char *const max1720x_manufacturer = "Maxim Integrated";
static const char *const max17201_model = "MAX17201";
static const char *const max17205_model = "MAX17205";
static const char *const max1720x_model = "MAX1720X";

struct max1720x_device_info {
	struct regmap *regmap;
	struct regmap *regmap_nv;
	struct i2c_client *ancillary;
	int rsense;
	struct power_supply_desc desc;
};

/*
 * Model Gauge M5 Algorithm output register
 * Volatile data (must not be cached)
 */
static const struct regmap_range max1720x_volatile_allow[] = {
	regmap_reg_range(MAX172XX_STATUS, MAX172XX_CYCLES),
	regmap_reg_range(MAX172XX_AVG_VCELL, MAX172XX_TTF),
	regmap_reg_range(MAX172XX_QR_TABLE10, MAX172XX_ATAVCAP),
};

static const struct regmap_range max1720x_readable_allow[] = {
	regmap_reg_range(MAX172XX_STATUS, MAX172XX_ATAVCAP),
};

static const struct regmap_range max1720x_readable_deny[] = {
	/* unused registers */
	regmap_reg_range(0x24, 0x26),
	regmap_reg_range(0x30, 0x31),
	regmap_reg_range(0x33, 0x34),
	regmap_reg_range(0x37, 0x37),
	regmap_reg_range(0x3B, 0x3C),
	regmap_reg_range(0x40, 0x41),
	regmap_reg_range(0x43, 0x44),
	regmap_reg_range(0x47, 0x49),
	regmap_reg_range(0x4B, 0x4C),
	regmap_reg_range(0x4E, 0xAF),
	regmap_reg_range(0xB1, 0xB3),
	regmap_reg_range(0xB5, 0xB7),
	regmap_reg_range(0xBF, 0xD0),
	regmap_reg_range(0xDB, 0xDB),
	regmap_reg_range(0xE0, 0xFF),
};

static const struct regmap_access_table max1720x_readable_regs = {
	.yes_ranges	= max1720x_readable_allow,
	.n_yes_ranges	= ARRAY_SIZE(max1720x_readable_allow),
	.no_ranges	= max1720x_readable_deny,
	.n_no_ranges	= ARRAY_SIZE(max1720x_readable_deny),
};

static const struct regmap_access_table max1720x_volatile_regs = {
	.yes_ranges	= max1720x_volatile_allow,
	.n_yes_ranges	= ARRAY_SIZE(max1720x_volatile_allow),
	.no_ranges	= max1720x_readable_deny,
	.n_no_ranges	= ARRAY_SIZE(max1720x_readable_deny),
};

static const struct regmap_config max1720x_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = MAX172XX_ATAVCAP,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.rd_table = &max1720x_readable_regs,
	.volatile_table = &max1720x_volatile_regs,
	.cache_type = REGCACHE_MAPLE,
};

static const struct regmap_range max1720x_nvmem_allow[] = {
	regmap_reg_range(MAX172XX_TEMP1, MAX172XX_INT_TEMP),
	regmap_reg_range(MAX172XX_TEMP2, MAX172XX_TEMP2),
	regmap_reg_range(MAX1720X_NXTABLE0, MAX1720X_NDEVICE_NAME4),
};

static const struct regmap_range max1720x_nvmem_deny[] = {
	regmap_reg_range(0x00, 0x33),
	regmap_reg_range(0x36, 0x3A),
	regmap_reg_range(0x3C, 0x7F),
	regmap_reg_range(0xE0, 0xFF),
};

static const struct regmap_access_table max1720x_nvmem_regs = {
	.yes_ranges	= max1720x_nvmem_allow,
	.n_yes_ranges	= ARRAY_SIZE(max1720x_nvmem_allow),
	.no_ranges	= max1720x_nvmem_deny,
	.n_no_ranges	= ARRAY_SIZE(max1720x_nvmem_deny),
};

static const struct regmap_config max1720x_nvmem_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = MAX1720X_NDEVICE_NAME4,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.rd_table = &max1720x_nvmem_regs,
};

static const struct nvmem_cell_info max1720x_nvmem_cells[] = {
	{ .name = "nXTable0",  .offset = 0,  .bytes = 2, },
	{ .name = "nXTable1",  .offset = 2,  .bytes = 2, },
	{ .name = "nXTable2",  .offset = 4,  .bytes = 2, },
	{ .name = "nXTable3",  .offset = 6,  .bytes = 2, },
	{ .name = "nXTable4",  .offset = 8,  .bytes = 2, },
	{ .name = "nXTable5",  .offset = 10, .bytes = 2, },
	{ .name = "nXTable6",  .offset = 12, .bytes = 2, },
	{ .name = "nXTable7",  .offset = 14, .bytes = 2, },
	{ .name = "nXTable8",  .offset = 16, .bytes = 2, },
	{ .name = "nXTable9",  .offset = 18, .bytes = 2, },
	{ .name = "nXTable10", .offset = 20, .bytes = 2, },
	{ .name = "nXTable11", .offset = 22, .bytes = 2, },
	{ .name = "nUser18C",  .offset = 24, .bytes = 2, },
	{ .name = "nUser18D",  .offset = 26, .bytes = 2, },
	{ .name = "nODSCTh",   .offset = 28, .bytes = 2, },
	{ .name = "nODSCCfg",  .offset = 30, .bytes = 2, },

	{ .name = "nOCVTable0",  .offset = 32, .bytes = 2, },
	{ .name = "nOCVTable1",  .offset = 34, .bytes = 2, },
	{ .name = "nOCVTable2",  .offset = 36, .bytes = 2, },
	{ .name = "nOCVTable3",  .offset = 38, .bytes = 2, },
	{ .name = "nOCVTable4",  .offset = 40, .bytes = 2, },
	{ .name = "nOCVTable5",  .offset = 42, .bytes = 2, },
	{ .name = "nOCVTable6",  .offset = 44, .bytes = 2, },
	{ .name = "nOCVTable7",  .offset = 46, .bytes = 2, },
	{ .name = "nOCVTable8",  .offset = 48, .bytes = 2, },
	{ .name = "nOCVTable9",  .offset = 50, .bytes = 2, },
	{ .name = "nOCVTable10", .offset = 52, .bytes = 2, },
	{ .name = "nOCVTable11", .offset = 54, .bytes = 2, },
	{ .name = "nIChgTerm",   .offset = 56, .bytes = 2, },
	{ .name = "nFilterCfg",  .offset = 58, .bytes = 2, },
	{ .name = "nVEmpty",     .offset = 60, .bytes = 2, },
	{ .name = "nLearnCfg",   .offset = 62, .bytes = 2, },

	{ .name = "nQRTable00",  .offset = 64, .bytes = 2, },
	{ .name = "nQRTable10",  .offset = 66, .bytes = 2, },
	{ .name = "nQRTable20",  .offset = 68, .bytes = 2, },
	{ .name = "nQRTable30",  .offset = 70, .bytes = 2, },
	{ .name = "nCycles",     .offset = 72, .bytes = 2, },
	{ .name = "nFullCapNom", .offset = 74, .bytes = 2, },
	{ .name = "nRComp0",     .offset = 76, .bytes = 2, },
	{ .name = "nTempCo",     .offset = 78, .bytes = 2, },
	{ .name = "nIAvgEmpty",  .offset = 80, .bytes = 2, },
	{ .name = "nFullCapRep", .offset = 82, .bytes = 2, },
	{ .name = "nVoltTemp",   .offset = 84, .bytes = 2, },
	{ .name = "nMaxMinCurr", .offset = 86, .bytes = 2, },
	{ .name = "nMaxMinVolt", .offset = 88, .bytes = 2, },
	{ .name = "nMaxMinTemp", .offset = 90, .bytes = 2, },
	{ .name = "nSOC",        .offset = 92, .bytes = 2, },
	{ .name = "nTimerH",     .offset = 94, .bytes = 2, },

	{ .name = "nConfig",    .offset = 96,  .bytes = 2, },
	{ .name = "nRippleCfg", .offset = 98,  .bytes = 2, },
	{ .name = "nMiscCfg",   .offset = 100, .bytes = 2, },
	{ .name = "nDesignCap", .offset = 102, .bytes = 2, },
	{ .name = "nHibCfg",    .offset = 104, .bytes = 2, },
	{ .name = "nPackCfg",   .offset = 106, .bytes = 2, },
	{ .name = "nRelaxCfg",  .offset = 108, .bytes = 2, },
	{ .name = "nConvgCfg",  .offset = 110, .bytes = 2, },
	{ .name = "nNVCfg0",    .offset = 112, .bytes = 2, },
	{ .name = "nNVCfg1",    .offset = 114, .bytes = 2, },
	{ .name = "nNVCfg2",    .offset = 116, .bytes = 2, },
	{ .name = "nSBSCfg",    .offset = 118, .bytes = 2, },
	{ .name = "nROMID0",    .offset = 120, .bytes = 2, },
	{ .name = "nROMID1",    .offset = 122, .bytes = 2, },
	{ .name = "nROMID2",    .offset = 124, .bytes = 2, },
	{ .name = "nROMID3",    .offset = 126, .bytes = 2, },

	{ .name = "nVAlrtTh",      .offset = 128, .bytes = 2, },
	{ .name = "nTAlrtTh",      .offset = 130, .bytes = 2, },
	{ .name = "nSAlrtTh",      .offset = 132, .bytes = 2, },
	{ .name = "nIAlrtTh",      .offset = 134, .bytes = 2, },
	{ .name = "nUser1C4",      .offset = 136, .bytes = 2, },
	{ .name = "nUser1C5",      .offset = 138, .bytes = 2, },
	{ .name = "nFullSOCThr",   .offset = 140, .bytes = 2, },
	{ .name = "nTTFCfg",       .offset = 142, .bytes = 2, },
	{ .name = "nCGain",        .offset = 144, .bytes = 2, },
	{ .name = "nTCurve",       .offset = 146, .bytes = 2, },
	{ .name = "nTGain",        .offset = 148, .bytes = 2, },
	{ .name = "nTOff",         .offset = 150, .bytes = 2, },
	{ .name = "nManfctrName0", .offset = 152, .bytes = 2, },
	{ .name = "nManfctrName1", .offset = 154, .bytes = 2, },
	{ .name = "nManfctrName2", .offset = 156, .bytes = 2, },
	{ .name = "nRSense",       .offset = 158, .bytes = 2, },

	{ .name = "nUser1D0",       .offset = 160, .bytes = 2, },
	{ .name = "nUser1D1",       .offset = 162, .bytes = 2, },
	{ .name = "nAgeFcCfg",      .offset = 164, .bytes = 2, },
	{ .name = "nDesignVoltage", .offset = 166, .bytes = 2, },
	{ .name = "nUser1D4",       .offset = 168, .bytes = 2, },
	{ .name = "nRFastVShdn",    .offset = 170, .bytes = 2, },
	{ .name = "nManfctrDate",   .offset = 172, .bytes = 2, },
	{ .name = "nFirstUsed",     .offset = 174, .bytes = 2, },
	{ .name = "nSerialNumber0", .offset = 176, .bytes = 2, },
	{ .name = "nSerialNumber1", .offset = 178, .bytes = 2, },
	{ .name = "nSerialNumber2", .offset = 180, .bytes = 2, },
	{ .name = "nDeviceName0",   .offset = 182, .bytes = 2, },
	{ .name = "nDeviceName1",   .offset = 184, .bytes = 2, },
	{ .name = "nDeviceName2",   .offset = 186, .bytes = 2, },
	{ .name = "nDeviceName3",   .offset = 188, .bytes = 2, },
	{ .name = "nDeviceName4",   .offset = 190, .bytes = 2, },
};

static const enum power_supply_property max1720x_battery_props[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

/* Convert regs value to power_supply units */

static int max172xx_time_to_ps(unsigned int reg)
{
	return reg * 5625 / 1000;	/* in sec. */
}

static int max172xx_percent_to_ps(unsigned int reg)
{
	return reg / 256;	/* in percent from 0 to 100 */
}

static int max172xx_voltage_to_ps(unsigned int reg)
{
	return reg * 1250;	/* Batt reg, LSB 1.25 mV, in uV */
}

static int max172xx_vcell_to_ps(unsigned int reg)
{
	return reg * 625 / 8;	/* VCell reg, LSB 78.125 uV, in uV */
}

static int max172xx_capacity_to_ps(unsigned int reg,
				   struct max1720x_device_info *info)
{
	return reg * (500000 / info->rsense);	/* in uAh */
}

/*
 * Current and temperature is signed values, so unsigned regs
 * value must be converted to signed type
 */

static int max172xx_temperature_to_ps(unsigned int reg)
{
	int val = (int16_t)reg;

	return val * 10 / 256; /* in tenths of deg. C */
}

/*
 * Calculating current registers resolution:
 *
 * RSense stored in 10^-5 Ohm, so measurement voltage must be
 * in 10^-11 Volts for get current in uA.
 * 16 bit current reg fullscale +/-51.2mV is 102400 uV.
 * So: 102400 / 65535 * 10^5 = 156252
 */
static int max172xx_current_to_voltage(unsigned int reg)
{
	int val = (int16_t)reg;

	return val * 156252;
}

/*
 * Physically-sane Li-ion health limits. We derive health from the measured
 * temperature and per-cell voltage rather than from the gauge's Status alert
 * latches (VMN/VMX/TMN/TMX): those latch against the user-programmable
 * VAlrtTh/TAlrtTh thresholds, and the felix packs ship with those
 * misconfigured (the base gauge's TAlrtTh selects a +3 degC max-temp alert,
 * so TMX -> "Overheat" trips at any normal temperature, and a stale min-
 * voltage latch spuriously reads "Dead"). The thermistor calibration itself
 * is correct, so the measured Temp/VCell registers are trustworthy.
 */
#define MAX172XX_HEALTH_HOT_DDEGC	600	/* 60.0 degC, tenths */
#define MAX172XX_HEALTH_COLD_DDEGC	0	/*  0.0 degC, tenths */
#define MAX172XX_HEALTH_OV_UV		4500000	/* 4.50 V per cell */
#define MAX172XX_HEALTH_DEAD_UV		2500000	/* 2.50 V per cell */

static int max172xx_battery_health(struct max1720x_device_info *info,
				   unsigned int *health)
{
	unsigned int reg_val;
	int temp, vcell, ret;

	ret = regmap_read(info->regmap, MAX172XX_STATUS, &reg_val);
	if (ret < 0)
		return ret;
	if (FIELD_GET(MAX172XX_STATUS_BAT_ABSENT, reg_val)) {
		*health = POWER_SUPPLY_HEALTH_NO_BATTERY;
		return 0;
	}

	ret = regmap_read(info->regmap, MAX172XX_TEMP, &reg_val);
	if (ret < 0)
		return ret;
	temp = max172xx_temperature_to_ps(reg_val);

	ret = regmap_read(info->regmap, MAX172XX_VCELL, &reg_val);
	if (ret < 0)
		return ret;
	vcell = max172xx_vcell_to_ps(reg_val);

	if (temp >= MAX172XX_HEALTH_HOT_DDEGC)
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (temp <= MAX172XX_HEALTH_COLD_DDEGC)
		*health = POWER_SUPPLY_HEALTH_COLD;
	else if (vcell >= MAX172XX_HEALTH_OV_UV)
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if (vcell > 0 && vcell <= MAX172XX_HEALTH_DEAD_UV)
		*health = POWER_SUPPLY_HEALTH_DEAD;
	else
		*health = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int max1720x_battery_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct max1720x_device_info *info = power_supply_get_drvdata(psy);
	unsigned int reg_val;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
		ret = max172xx_battery_health(info, &reg_val);
		val->intval = reg_val;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		/*
		 * POWER_SUPPLY_PROP_PRESENT will always readable via
		 * sysfs interface. Value return 0 if battery not
		 * present or unaccesable via I2c.
		 */
		ret = regmap_read(info->regmap, MAX172XX_STATUS, &reg_val);
		if (ret < 0) {
			val->intval = 0;
			return 0;
		}

		val->intval = !FIELD_GET(MAX172XX_STATUS_BAT_ABSENT, reg_val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = regmap_read(info->regmap, MAX172XX_REPSOC, &reg_val);
		val->intval = max172xx_percent_to_ps(reg_val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = regmap_read(info->regmap, MAX172XX_BATT, &reg_val);
		if (ret < 0)
			break;
		if (reg_val) {
			val->intval = max172xx_voltage_to_ps(reg_val);
		} else {
			/*
			 * Single-cell gauges (e.g. the felix base pack) leave
			 * the multi-cell Batt reg (0xDA) at 0; fall back to the
			 * per-cell VCell reg (0x09).
			 */
			ret = regmap_read(info->regmap, MAX172XX_VCELL, &reg_val);
			val->intval = max172xx_vcell_to_ps(reg_val);
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = regmap_read(info->regmap, MAX172XX_DESIGN_CAP, &reg_val);
		val->intval = max172xx_capacity_to_ps(reg_val, info);
		break;
	case POWER_SUPPLY_PROP_CHARGE_AVG:
		ret = regmap_read(info->regmap, MAX172XX_REPCAP, &reg_val);
		val->intval = max172xx_capacity_to_ps(reg_val, info);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = regmap_read(info->regmap, MAX172XX_TTE, &reg_val);
		val->intval = max172xx_time_to_ps(reg_val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		ret = regmap_read(info->regmap, MAX172XX_TTF, &reg_val);
		val->intval = max172xx_time_to_ps(reg_val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = regmap_read(info->regmap, MAX172XX_TEMP, &reg_val);
		val->intval = max172xx_temperature_to_ps(reg_val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = regmap_read(info->regmap, MAX172XX_CURRENT, &reg_val);
		val->intval = max172xx_current_to_voltage(reg_val) / info->rsense;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = regmap_read(info->regmap, MAX172XX_AVG_CURRENT, &reg_val);
		val->intval = max172xx_current_to_voltage(reg_val) / info->rsense;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = regmap_read(info->regmap, MAX172XX_FULL_CAP, &reg_val);
		val->intval = max172xx_capacity_to_ps(reg_val, info);
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		ret = regmap_read(info->regmap, MAX172XX_DEV_NAME, &reg_val);
		if (ret)
			return ret;
		reg_val = FIELD_GET(MAX172XX_DEV_NAME_TYPE_MASK, reg_val);
		if (reg_val == MAX172XX_DEV_NAME_TYPE_MAX17201)
			val->strval = max17201_model;
		else if (reg_val == MAX172XX_DEV_NAME_TYPE_MAX17205)
			val->strval = max17205_model;
		else
			val->strval = max1720x_model;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = max1720x_manufacturer;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int max1720x_read_temp(struct device *dev, u8 reg, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct max1720x_device_info *info = power_supply_get_drvdata(psy);
	unsigned int val;
	int ret;

	ret = regmap_read(info->regmap_nv, reg, &val);
	if (ret < 0)
		return ret;

	/*
	 * Temperature in degrees Celsius starting at absolute zero, -273C or
	 * 0K with an LSb of 0.1C
	 */
	return sysfs_emit(buf, "%d\n", val - 2730);
}

static ssize_t temp_ain1_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	return max1720x_read_temp(dev, MAX172XX_TEMP1, buf);
}

static ssize_t temp_ain2_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	return max1720x_read_temp(dev, MAX172XX_TEMP2, buf);
}

static ssize_t temp_int_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return max1720x_read_temp(dev, MAX172XX_INT_TEMP, buf);
}

static DEVICE_ATTR_RO(temp_ain1);
static DEVICE_ATTR_RO(temp_ain2);
static DEVICE_ATTR_RO(temp_int);

static struct attribute *max1720x_attrs[] = {
	&dev_attr_temp_ain1.attr,
	&dev_attr_temp_ain2.attr,
	&dev_attr_temp_int.attr,
	NULL
};
ATTRIBUTE_GROUPS(max1720x);

static
int max1720x_nvmem_reg_read(void *priv, unsigned int off, void *val, size_t len)
{
	struct max1720x_device_info *info = priv;
	unsigned int reg = MAX1720X_NXTABLE0 + (off / 2);

	return regmap_bulk_read(info->regmap_nv, reg, val, len / 2);
}

static void max1720x_unregister_ancillary(void *data)
{
	struct max1720x_device_info *info = data;

	i2c_unregister_device(info->ancillary);
}

static int max1720x_probe_nvmem(struct i2c_client *client,
				struct max1720x_device_info *info)
{
	struct device *dev = &client->dev;
	struct nvmem_config nvmem_config = {
		.dev = dev,
		.name = "max1720x_nvmem",
		.cells = max1720x_nvmem_cells,
		.ncells = ARRAY_SIZE(max1720x_nvmem_cells),
		.read_only = true,
		.root_only = true,
		.reg_read = max1720x_nvmem_reg_read,
		.size = ARRAY_SIZE(max1720x_nvmem_cells) * 2,
		.word_size = 2,
		.stride = 2,
		.priv = info,
	};
	struct nvmem_device *nvmem;
	unsigned int val;
	int ret;

	info->ancillary = i2c_new_ancillary_device(client, "nvmem", 0xb);
	if (IS_ERR(info->ancillary)) {
		dev_err(dev, "Failed to initialize ancillary i2c device\n");
		return PTR_ERR(info->ancillary);
	}

	ret = devm_add_action_or_reset(dev, max1720x_unregister_ancillary, info);
	if (ret) {
		dev_err(dev, "Failed to add unregister callback\n");
		return ret;
	}

	info->regmap_nv = devm_regmap_init_i2c(info->ancillary,
					       &max1720x_nvmem_regmap_cfg);
	if (IS_ERR(info->regmap_nv)) {
		dev_err(dev, "regmap initialization of nvmem failed\n");
		return PTR_ERR(info->regmap_nv);
	}

	ret = regmap_read(info->regmap_nv, MAX1720X_NRSENSE, &val);
	if (ret < 0) {
		dev_err(dev, "Failed to read sense resistor value\n");
		return ret;
	}

	info->rsense = val;
	if (!info->rsense) {
		dev_warn(dev, "RSense not calibrated, set 10 mOhms!\n");
		info->rsense = 1000; /* in regs in 10^-5 */
	}

	nvmem = devm_nvmem_register(dev, &nvmem_config);
	if (IS_ERR(nvmem)) {
		dev_err(dev, "Could not register nvmem!");
		return PTR_ERR(nvmem);
	}

	return 0;
}

static const struct power_supply_desc max1720x_bat_desc = {
	.name = "max1720x",
	.no_thermal = true,
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = max1720x_battery_props,
	.num_properties = ARRAY_SIZE(max1720x_battery_props),
	.get_property = max1720x_battery_get_property,
};

/*
 * (Re)program the external-thermistor calibration from DT.
 *
 * A gauge with a 0x0b nvmem companion restores its thermistor cal from nvram on
 * every power-up, so it needs nothing here. The felix base pack's gauge has no
 * such companion: its cal lives only in volatile shadow RAM, so any power loss
 * reverts it to defaults — Config.Tex gets set (the chip then stops measuring
 * the thermistor and freezes Temp at a host-written value) and TGain/TOff/TCurve
 * drift — which silently corrupts the only thermal sensor on this platform.
 *
 * When the DT supplies the golden cal, rewrite it and clear Config.Tex at probe
 * so the reading is correct regardless of power history. Absent the properties
 * (e.g. the nvram-backed secondary pack) this is a no-op.
 */
static void max1720x_program_thermistor_cal(struct max1720x_device_info *info,
					    struct device *dev)
{
	u16 tgain, toff, tcurve;

	if (device_property_read_u16(dev, "maxim,thermistor-tgain", &tgain))
		return;
	if (device_property_read_u16(dev, "maxim,thermistor-toff", &toff) ||
	    device_property_read_u16(dev, "maxim,thermistor-tcurve", &tcurve)) {
		dev_warn(dev, "incomplete thermistor cal in DT; skipping\n");
		return;
	}

	if (regmap_write(info->regmap, MAX172XX_TGAIN, tgain) ||
	    regmap_write(info->regmap, MAX172XX_TOFF, toff) ||
	    regmap_write(info->regmap, MAX172XX_TCURVE, tcurve) ||
	    regmap_clear_bits(info->regmap, MAX172XX_CONFIG, MAX172XX_CONFIG_TEX))
		dev_warn(dev, "failed to program thermistor cal\n");
}

/*
 * Positional layout of the "maxim,fg-params" DT array. This is the same order
 * used by the AOSP felix battery-data, so its values can be copied verbatim.
 */
enum {
	M5P_IAVGEMPTY, M5P_RELAXCFG, M5P_LEARNCFG, M5P_CONFIG, M5P_CONFIG2,
	M5P_FULLSOCTHR, M5P_FULLCAPREP, M5P_DESIGNCAP, M5P_DPACC, M5P_DQACC,
	M5P_FULLCAPNOM, M5P_VEMPTY, M5P_QRTABLE00, M5P_QRTABLE10, M5P_QRTABLE20,
	M5P_QRTABLE30, M5P_RCOMP0, M5P_TEMPCO, M5P_ICHGTERM, M5P_TGAIN, M5P_TOFF,
	M5P_TCURVE, M5P_MISCCFG, M5P_ATRATE, M5P_CONVGCFG, M5P_FILTERCFG,
	M5P_TASKPERIOD, M5P_COUNT
};

/*
 * The model/parameter registers live outside the driver's normal restricted
 * map (0x60-0xFF, 0x80-0xAF), so the reload drives them through a private
 * permissive regmap built on the same i2c client.
 */
static const struct regmap_config max1720x_m5load_regmap_cfg = {
	.name = "m5load",
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = MAX172XX_M5_VFSOC,
	.val_format_endian = REGMAP_ENDIAN_LITTLE, /* the gauge is little-endian */
};

/*
 * Write and verify the 48-word characterized model into 0x80..0xAF.
 *
 * The unlock, the model block, and the lock are each issued as a single raw
 * (multi-word) transaction — the gauge only opens model access for an atomic
 * two-word write of the unlock code, so per-register writes leave it locked
 * (the model then reads back as all-0xffff).
 */
static int max1720x_m5_write_model(struct device *dev, struct regmap *rm,
				   const u16 *model)
{
	u16 rb[MAX172XX_M5_FG_MODEL_COUNT];
	const u16 unlock[2] = { 0x0059, 0x00C4 };
	const u16 lock[2] = { 0x0000, 0x0000 };
	int i, ret;

	/* unlock model access (single 2-word write to 0x62/0x63) */
	ret = regmap_raw_write(rm, MAX172XX_M5_UNLOCK_MODEL0, unlock,
			       sizeof(unlock));
	if (ret)
		return ret;

	ret = regmap_raw_write(rm, MAX172XX_M5_FG_MODEL_START, model,
			       MAX172XX_M5_FG_MODEL_COUNT * sizeof(u16));
	if (ret)
		goto relock;

	ret = regmap_raw_read(rm, MAX172XX_M5_FG_MODEL_START, rb, sizeof(rb));
	if (ret)
		goto relock;

	for (i = 0; i < MAX172XX_M5_FG_MODEL_COUNT; i++) {
		if (rb[i] != model[i]) {
			dev_err(dev, "m5 model verify failed at %d (%#x != %#x)\n",
				i, rb[i], model[i]);
			ret = -EIO;
			goto relock;
		}
	}
relock:
	/* lock model access regardless of outcome */
	regmap_raw_write(rm, MAX172XX_M5_UNLOCK_MODEL0, lock, sizeof(lock));
	return ret;
}

/* Write the custom parameters (Maxim m5 "custom full INI", step 7). */
static int max1720x_m5_write_params(struct regmap *rm, const u16 *p)
{
	unsigned int vfsoc;
	int ret;

	ret = regmap_write(rm, MAX172XX_M5_REPCAP, 0);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_RELAXCFG, p[M5P_RELAXCFG]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_UNLOCK_EXTRA,
				   MAX172XX_M5_UNLOCK_EXTRA_CODE);
	if (ret)
		return ret;

	ret = regmap_read(rm, MAX172XX_M5_VFSOC, &vfsoc);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_VFSOC0, vfsoc);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_LEARNCFG, p[M5P_LEARNCFG]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_CONFIG, p[M5P_CONFIG]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_CONFIG2, p[M5P_CONFIG2]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_FULLSOCTHR, p[M5P_FULLSOCTHR]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_FULLCAPREP, p[M5P_FULLCAPREP]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_DESIGN_CAP, p[M5P_DESIGNCAP]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_DPACC, p[M5P_DPACC]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_DQACC, p[M5P_DQACC]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_FULLCAPNOM, p[M5P_FULLCAPNOM]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_VEMPTY, p[M5P_VEMPTY]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_QRTABLE00, p[M5P_QRTABLE00]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_QR_TABLE10, p[M5P_QRTABLE10]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_QRTABLE20, p[M5P_QRTABLE20]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_QRTABLE30, p[M5P_QRTABLE30]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_RCOMP0, p[M5P_RCOMP0]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_TEMPCO, p[M5P_TEMPCO]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_TASKPERIOD, p[M5P_TASKPERIOD]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_ICHGTERM, p[M5P_ICHGTERM]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_TGAIN, p[M5P_TGAIN]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_TOFF, p[M5P_TOFF]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_MISCCFG, p[M5P_MISCCFG]);
	if (ret)
		return ret;

	/* second batch needs the extra-config unlock re-applied */
	ret = regmap_write(rm, MAX172XX_M5_UNLOCK_EXTRA,
			   MAX172XX_M5_UNLOCK_EXTRA_CODE);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_ATRATE, p[M5P_ATRATE]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_CV_MIXCAP,
				   (p[M5P_FULLCAPNOM] * 75) / 100);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_CV_HALFTIME, 0x600);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_CONVGCFG, p[M5P_CONVGCFG]);

	/* lock the extra config back up */
	regmap_write(rm, MAX172XX_M5_UNLOCK_EXTRA, MAX172XX_M5_LOCK_EXTRA_CODE);
	if (ret)
		return ret;

	/* tcurve and filtercfg are not part of the model proper */
	ret = regmap_write(rm, MAX172XX_TCURVE, p[M5P_TCURVE]);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_FILTERCFG, p[M5P_FILTERCFG]);
	return ret;
}

/*
 * Reload the ModelGauge m5 characterized model after a power-on reset.
 *
 * A gauge with an nvmem companion restores its own model on power-up. The felix
 * base pack's gauge has none, so on every power loss it comes up on POR defaults
 * and its learned full-capacity mis-converges (observed collapsing to ~half of
 * design), making SoC read garbage. When the DT supplies the golden model
 * (maxim,fg-model + maxim,fg-params, the AOSP characterized battery-data),
 * rewrite it whenever the POR bit is set and then clear POR. Absent the
 * properties (e.g. the nvram-backed secondary pack) this is a no-op.
 */
static void max1720x_load_m5_model(struct max1720x_device_info *info,
				   struct device *dev)
{
	u16 model[MAX172XX_M5_FG_MODEL_COUNT], params[M5P_COUNT];
	unsigned int status, dcap, cfg2, repcap;
	u32 rsense_uohm;
	struct regmap *rm;
	int ret, retries;

	if (device_property_read_u16_array(dev, "maxim,fg-model", model,
					   MAX172XX_M5_FG_MODEL_COUNT))
		return;
	if (device_property_read_u16_array(dev, "maxim,fg-params", params,
					   M5P_COUNT)) {
		dev_warn(dev, "maxim,fg-model without a valid maxim,fg-params; skipping model reload\n");
		return;
	}

	/*
	 * Reload if the gauge came up on POR, or if its DesignCap doesn't match the
	 * golden model (never loaded / mis-converged). A warm reboot with the model
	 * already in place keeps it and only re-applies rsense below.
	 */
	if (regmap_read(info->regmap, MAX172XX_STATUS, &status) ||
	    regmap_read(info->regmap, MAX172XX_DESIGN_CAP, &dcap))
		return;
	if (!(status & MAX172XX_STATUS_POR) && dcap == params[M5P_DESIGNCAP])
		goto apply_rsense;

	rm = devm_regmap_init_i2c(to_i2c_client(dev),
				  &max1720x_m5load_regmap_cfg);
	if (IS_ERR(rm)) {
		dev_warn(dev, "m5 load regmap init failed (%ld)\n", PTR_ERR(rm));
		return;
	}

	/* wait for the gauge's data-not-ready flag to clear */
	for (retries = 20; retries > 0; retries--) {
		if (!regmap_read(rm, MAX172XX_M5_FSTAT, &status) &&
		    !(status & MAX172XX_M5_FSTAT_DNR))
			break;
		msleep(50);
	}

	if (!regmap_read(rm, MAX172XX_M5_CONFIG2, &cfg2) &&
	    (cfg2 & MAX172XX_M5_CONFIG2_LDMDL)) {
		dev_err(dev, "m5 model load already in progress (%#x)\n", cfg2);
		return;
	}

	if (max1720x_m5_write_model(dev, rm, model)) {
		dev_err(dev, "m5 model write failed\n");
		return;
	}
	if (max1720x_m5_write_params(rm, params)) {
		dev_err(dev, "m5 params write failed\n");
		return;
	}

	/* trigger the gauge to recompute its state from the new model */
	ret = regmap_read(rm, MAX172XX_M5_CONFIG2, &cfg2);
	if (!ret)
		ret = regmap_write(rm, MAX172XX_M5_CONFIG2,
				   cfg2 | MAX172XX_M5_CONFIG2_LDMDL);
	if (ret) {
		dev_err(dev, "m5 load trigger failed (%d)\n", ret);
		return;
	}
	for (retries = 20; retries > 0; retries--) {
		msleep(50);
		if (regmap_read(rm, MAX172XX_M5_CONFIG2, &cfg2) ||
		    (cfg2 & MAX172XX_M5_CONFIG2_LDMDL))
			continue;
		if (!regmap_read(rm, MAX172XX_M5_REPCAP, &repcap) && repcap)
			break;
	}
	if (retries == 0) {
		dev_warn(dev, "m5 model load did not settle\n");
		return;
	}

	/* clear POR so a warm reboot won't reload the model */
	regmap_update_bits(rm, MAX172XX_STATUS, MAX172XX_STATUS_POR, 0);

	/*
	 * The model was written through this private regmap, so the driver's main
	 * (cached) regmap still holds pre-reload values for any non-volatile
	 * register it caches — notably DesignCap (0x18). Drop the cache so
	 * subsequent reads (e.g. CHARGE_FULL_DESIGN) reflect the new model.
	 */
	regcache_drop_region(info->regmap, MAX172XX_DESIGN_CAP,
			     MAX172XX_DESIGN_CAP);
	dev_info(dev, "m5 characterized model reloaded\n");

apply_rsense:
	/*
	 * The model's capacity registers only read out correctly with the pack's
	 * real sense resistor. Apply it here rather than in probe so a failed
	 * reload (which returns early above, leaving the stale 4x-too-large
	 * DesignCap) can't be paired with the new rsense.
	 */
	if (device_property_read_u32(dev, "shunt-resistor-micro-ohms",
				     &rsense_uohm) == 0 && rsense_uohm)
		info->rsense = rsense_uohm / 10; /* to 10^-5 Ohm */
}

static int max1720x_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct device *dev = &client->dev;
	struct max1720x_device_info *info;
	struct power_supply *bat;
	int ret;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	psy_cfg.drv_data = info;
	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.attr_grp = max1720x_groups;
	i2c_set_clientdata(client, info);
	info->regmap = devm_regmap_init_i2c(client, &max1720x_regmap_cfg);
	if (IS_ERR(info->regmap))
		return dev_err_probe(dev, PTR_ERR(info->regmap),
				     "regmap initialization failed\n");

	ret = max1720x_probe_nvmem(client, info);
	if (ret) {
		/*
		 * Some gauges (e.g. the felix base pack) have no 0x0b nvmem
		 * companion; keep the gauge usable for live telemetry with a
		 * default sense-resistor value instead of failing the probe.
		 */
		dev_warn(dev, "nvmem unavailable (%d); using default rsense\n",
			 ret);
		info->rsense = 1000; /* 10 mOhm, in 10^-5 Ohm */
	}

	/* Restore volatile thermistor cal for gauges without an nvram companion. */
	max1720x_program_thermistor_cal(info, dev);

	/*
	 * Reload the characterized m5 model for a gauge without nvmem (the felix
	 * base pack). This also applies the pack's real sense resistor (from
	 * shunt-resistor-micro-ohms), which must match the loaded DesignCap.
	 */
	max1720x_load_m5_model(info, dev);

	/*
	 * Copy the template desc so a per-instance name can be applied — felix
	 * is dual-battery (two gauges on separate i2c buses), and two power
	 * supplies can't share the "max1720x" sysfs name. An optional DT "label"
	 * overrides it (e.g. maxfg_base / maxfg_secondary).
	 */
	info->desc = max1720x_bat_desc;
	device_property_read_string(dev, "label", &info->desc.name);

	bat = devm_power_supply_register(dev, &info->desc, &psy_cfg);
	if (IS_ERR(bat))
		return dev_err_probe(dev, PTR_ERR(bat),
				     "Failed to register power supply\n");

	return 0;
}

static const struct of_device_id max1720x_of_match[] = {
	{ .compatible = "maxim,max17201" },
	{}
};
MODULE_DEVICE_TABLE(of, max1720x_of_match);

static struct i2c_driver max1720x_i2c_driver = {
	.driver = {
		.name = "max1720x",
		.of_match_table = max1720x_of_match,
	},
	.probe = max1720x_probe,
};
module_i2c_driver(max1720x_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dimitri Fedrau <dima.fedrau@gmail.com>");
MODULE_DESCRIPTION("Maxim MAX17201/MAX17205 Fuel Gauge IC driver");
