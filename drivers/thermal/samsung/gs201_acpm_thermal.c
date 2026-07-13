// SPDX-License-Identifier: GPL-2.0
/*
 * Thermal sensor driver for Google gs201 (Tensor G2) TMU via ACPM.
 *
 * On gs201 the Thermal Management Unit is not directly memory-mapped to the
 * AP; temperatures are read by IPC to the ACPM power-management coprocessor
 * (channel IPC_AP_TMU). The ACPM protocol layer (drivers/firmware/samsung/
 * exynos-acpm-tmu.c) already implements the message exchange and exposes
 * handle->ops->tmu.read_temp(); this driver bridges those per-sensor reads
 * into the Linux thermal framework so DT thermal-zones + cooling maps can
 * throttle CPU cpufreq / GPU devfreq under load.
 *
 * Copyright 2026 Junkyard Computing
 */

#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

/* gs201 exposes 8 TMU sensors (BIG, MID, LITTLE, G3D, ISP, TPU, AUR, ...). */
#define GS201_NR_TMU		8
/* ACPM IPC channel for the TMU service (AOSP: IPC_AP_TMU). */
#define GS201_TMU_ACPM_CHAN	9

struct gs201_tmu;

struct gs201_tmu_sensor {
	struct gs201_tmu *tmu;
	unsigned int id;
	struct thermal_zone_device *tzd;
};

struct gs201_tmu {
	struct device *dev;
	struct acpm_handle *handle;
	unsigned int chan_id;
	struct gs201_tmu_sensor sensors[GS201_NR_TMU];
};

static int gs201_tmu_read(struct gs201_tmu *tmu, unsigned int id, int *temp_c)
{
	return tmu->handle->ops->tmu.read_temp(tmu->handle, tmu->chan_id, id,
					       temp_c);
}

static int gs201_tmu_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct gs201_tmu_sensor *s = thermal_zone_device_priv(tz);
	int ret, t;

	ret = gs201_tmu_read(s->tmu, s->id, &t);
	if (ret)
		return ret;

	/* ACPM returns whole degrees C (s8); thermal core wants milli-degrees. */
	*temp = t * 1000;
	return 0;
}

static const struct thermal_zone_device_ops gs201_tmu_ops = {
	.get_temp = gs201_tmu_get_temp,
};

static int gs201_tmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gs201_tmu *tmu;
	unsigned int registered = 0;
	int i;

	tmu = devm_kzalloc(dev, sizeof(*tmu), GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;

	tmu->dev = dev;

	tmu->handle = devm_acpm_get_by_phandle(dev);
	if (IS_ERR(tmu->handle))
		return dev_err_probe(dev, PTR_ERR(tmu->handle),
				     "failed to get ACPM handle\n");

	tmu->chan_id = GS201_TMU_ACPM_CHAN;
	of_property_read_u32(dev->of_node, "samsung,tmu-channel", &tmu->chan_id);

	for (i = 0; i < GS201_NR_TMU; i++) {
		struct gs201_tmu_sensor *s = &tmu->sensors[i];
		struct thermal_zone_device *tzd;
		int temp_c;

		s->tmu = tmu;
		s->id = i;

		tzd = devm_thermal_of_zone_register(dev, i, s, &gs201_tmu_ops);
		if (IS_ERR(tzd)) {
			/* No DT thermal-zone references this sensor id. */
			if (PTR_ERR(tzd) == -ENODEV)
				continue;
			return dev_err_probe(dev, PTR_ERR(tzd),
					     "failed to register sensor %d\n", i);
		}
		s->tzd = tzd;
		registered++;

		if (!gs201_tmu_read(tmu, i, &temp_c))
			dev_info(dev, "TMU sensor %d online: %d C\n", i, temp_c);
		else
			dev_warn(dev, "TMU sensor %d registered but read failed\n",
				 i);
	}

	if (!registered)
		return dev_err_probe(dev, -ENODEV,
				     "no TMU sensors referenced by any thermal-zone\n");

	platform_set_drvdata(pdev, tmu);
	dev_info(dev, "gs201 ACPM thermal: %u sensors on ACPM channel %u\n",
		 registered, tmu->chan_id);
	return 0;
}

static const struct of_device_id gs201_tmu_of_match[] = {
	{ .compatible = "google,gs201-acpm-thermal" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs201_tmu_of_match);

static struct platform_driver gs201_tmu_driver = {
	.probe = gs201_tmu_probe,
	.driver = {
		.name = "gs201-acpm-thermal",
		.of_match_table = gs201_tmu_of_match,
	},
};
module_platform_driver(gs201_tmu_driver);

MODULE_DESCRIPTION("Google gs201 ACPM thermal sensor driver");
MODULE_LICENSE("GPL");
