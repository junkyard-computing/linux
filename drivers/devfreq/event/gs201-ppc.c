// SPDX-License-Identifier: GPL-2.0-only
/*
 * gs201 (Google Tensor G2) PPC devfreq-event driver.
 *
 * The PPC (Performance Profiling Counter) blocks sit on the memory-master
 * ports (CCI/DREX) and count bus cycles (CCNT) and read+write transactions
 * (PMCNT1). Exposed as devfreq-event devices, they let exynos-bus + the
 * simple_ondemand governor scale MIF with real memory load instead of leaving
 * it pinned at the boot frequency.
 *
 * Register semantics ported from the AOSP gs-ppc.c; wrapped in the mainline
 * devfreq_event_ops interface (one edev per counter, exynos-bus maxes them).
 */

#include <linux/bits.h>
#include <linux/devfreq-event.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>

#define PPC_PMNC		0x0004	/* Performance Monitor Control */
#define PPC_CNTENS		0x0008	/* Count Enable Set */
#define PPC_PMCNT0		0x0034	/* Event counter 0 (read transactions) */
#define PPC_PMCNT1		0x0038	/* Event counter 1 (write transactions) */
#define PPC_CCNT		0x0048	/* Cycle counter */

#define PPC_PMNC_REGVAL		BIT(24)	/* required on every PMNC write */
#define PPC_PMNC_RESETALL	0x6	/* reset CCNT + PMCNTx */
#define PPC_PMNC_GLBCNTEN	BIT(0)	/* global counter enable */
#define PPC_CNTENS_CHVAL	(BIT(31) | 0x3)	/* enable CCNT + PMCNT0/1 */

struct gs201_ppc {
	void __iomem *base;
	struct devfreq_event_dev *edev;
	struct devfreq_event_desc desc;
};

/* Reset the counters and (re)start a fresh measurement window. */
static int gs201_ppc_start(struct devfreq_event_dev *edev)
{
	struct gs201_ppc *ppc = devfreq_event_get_drvdata(edev);

	writel(PPC_PMNC_REGVAL | PPC_PMNC_RESETALL, ppc->base + PPC_PMNC);
	writel(PPC_CNTENS_CHVAL, ppc->base + PPC_CNTENS);
	writel(PPC_PMNC_REGVAL | PPC_PMNC_GLBCNTEN, ppc->base + PPC_PMNC);

	return 0;
}

static int gs201_ppc_disable(struct devfreq_event_dev *edev)
{
	struct gs201_ppc *ppc = devfreq_event_get_drvdata(edev);

	writel(PPC_PMNC_RESETALL, ppc->base + PPC_PMNC);

	return 0;
}

static int gs201_ppc_get_event(struct devfreq_event_dev *edev,
			       struct devfreq_event_data *edata)
{
	struct gs201_ppc *ppc = devfreq_event_get_drvdata(edev);

	/* Freeze the counters so the event and cycle counts are read from one
	 * window. PMCNT0/PMCNT1 count read/write transactions respectively;
	 * their sum is the total memory bandwidth the governor scales on.
	 */
	writel(PPC_PMNC_REGVAL, ppc->base + PPC_PMNC);

	edata->load_count = readl(ppc->base + PPC_PMCNT0) +
			    readl(ppc->base + PPC_PMCNT1);
	edata->total_count = readl(ppc->base + PPC_CCNT);

	return 0;
}

static const struct devfreq_event_ops gs201_ppc_ops = {
	.enable		= gs201_ppc_start,
	.disable	= gs201_ppc_disable,
	.reset		= gs201_ppc_start,
	.set_event	= gs201_ppc_start,
	.get_event	= gs201_ppc_get_event,
};

static const struct of_device_id gs201_ppc_id_match[] = {
	{ .compatible = "google,gs201-ppc" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs201_ppc_id_match);

static int gs201_ppc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gs201_ppc *ppc;

	ppc = devm_kzalloc(dev, sizeof(*ppc), GFP_KERNEL);
	if (!ppc)
		return -ENOMEM;

	ppc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ppc->base))
		return PTR_ERR(ppc->base);

	ppc->desc.ops = &gs201_ppc_ops;
	ppc->desc.driver_data = ppc;
	ppc->desc.name = dev_name(dev);
	device_property_read_string(dev, "event-name", &ppc->desc.name);

	ppc->edev = devm_devfreq_event_add_edev(dev, &ppc->desc);
	if (IS_ERR(ppc->edev))
		return dev_err_probe(dev, PTR_ERR(ppc->edev),
				     "failed to add devfreq-event device\n");

	platform_set_drvdata(pdev, ppc);

	return 0;
}

static struct platform_driver gs201_ppc_driver = {
	.probe	= gs201_ppc_probe,
	.driver	= {
		.name = "gs201-ppc",
		.of_match_table = gs201_ppc_id_match,
	},
};
module_platform_driver(gs201_ppc_driver);

MODULE_DESCRIPTION("gs201 PPC devfreq-event driver");
MODULE_LICENSE("GPL");
