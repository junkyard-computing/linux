// SPDX-License-Identifier: GPL-2.0
/*
 * gs201 (Tensor G2) generic power domain support.
 *
 * The generic Exynos driver (exynos-pm-domains.c) pokes the PMU
 * CONFIGURATION register with a plain writel() through of_iomap(). That does
 * not work on gs201: the PMU is EL3-only, and a direct MMIO write either
 * aborts or is silently dropped. Writes have to be mediated by BL31 through
 * TENSOR_SMC_PMU_SEC_REG, which is exactly what the regmap installed by
 * exynos-pmu.c for the "google,gs201-pmu" node already does (see
 * regmap_smccfg / tensor_sec_reg_{read,write} there). So this driver is the
 * generic one re-expressed on top of that regmap: same CONFIGURATION/STATUS
 * layout (STATUS = CONFIGURATION + 4, bit0 = power enable), different
 * accessor.
 *
 * Why this matters: without a genpd provider, nothing on gs201 ever powers a
 * domain down. felix's bootloader leaves every SoC domain on and mainline
 * simply left them that way, which measured ~5C hotter at idle than stock
 * (per-zone: tpu +6, g3d +5, mid/little +5..6). AOSP powers 18 of 20 domains
 * off at idle via its own exynos-pd; the CPUs were never the problem (stock
 * spends 99.95% of idle in plain WFI, same as us) -- it was static leakage
 * from blocks left energised.
 *
 * Registering the domains is sufficient to recover that: genpd's own
 * late_initcall_sync(genpd_power_off_unused) drops every domain with no
 * in-use device. Blocks that DO get a driver later (TPU, DPU, GPU) just add
 * a power-domains phandle and are powered on demand, which is why this
 * supersedes the boot-time force-gate hack in drivers/soc/samsung/
 * gs201-pd-gate.c -- that could only ever gate blocks nobody wanted, and had
 * to be edited every time a driver landed.
 */

#include <linux/err.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/soc/samsung/exynos-pmu.h>

/* Offset from the domain's CONFIGURATION register to its STATUS register. */
#define GS201_PD_STATUS_OFFSET	0x4
/* CONFIGURATION/STATUS bit0: 1 = powered up. */
#define GS201_PD_PWR_EN		BIT(0)

/*
 * The SMC round-trip per access is not free, and a domain can take a while to
 * settle, so poll generously. AOSP's exynos-pd uses a 1s timeout for the same
 * transition.
 */
#define GS201_PD_POLL_US	100
#define GS201_PD_TIMEOUT_US	1000000

struct gs201_pm_domain {
	struct generic_pm_domain pd;
	struct regmap *pmureg;
	u32 offset;
};

static inline struct gs201_pm_domain *to_gs201_pd(struct generic_pm_domain *pd)
{
	return container_of(pd, struct gs201_pm_domain, pd);
}

static int gs201_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct gs201_pm_domain *pd = to_gs201_pd(domain);
	u32 want = power_on ? GS201_PD_PWR_EN : 0;
	u32 status;
	int ret;

	ret = regmap_update_bits(pd->pmureg, pd->offset, GS201_PD_PWR_EN, want);
	if (ret) {
		dev_err(&domain->dev, "failed to %s: %d\n",
			power_on ? "power on" : "power off", ret);
		return ret;
	}

	ret = regmap_read_poll_timeout(pd->pmureg,
				       pd->offset + GS201_PD_STATUS_OFFSET,
				       status,
				       (status & GS201_PD_PWR_EN) == want,
				       GS201_PD_POLL_US, GS201_PD_TIMEOUT_US);
	if (ret)
		dev_err(&domain->dev, "timed out waiting for %s (status 0x%08x)\n",
			power_on ? "power on" : "power off", status);

	return ret;
}

static int gs201_pd_power_on(struct generic_pm_domain *domain)
{
	return gs201_pd_power(domain, true);
}

static int gs201_pd_power_off(struct generic_pm_domain *domain)
{
	return gs201_pd_power(domain, false);
}

static int gs201_pd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct gs201_pm_domain *pd;
	const char *label;
	u32 status;
	bool is_off;
	int ret;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	/*
	 * Returns -EPROBE_DEFER until exynos-pmu has probed and installed the
	 * SMC-routed regmap for the PMU node.
	 */
	pd->pmureg = exynos_get_pmu_regmap_by_phandle(np, "samsung,pmu-syscon");
	if (IS_ERR(pd->pmureg))
		return dev_err_probe(dev, PTR_ERR(pd->pmureg),
				     "failed to get PMU regmap\n");

	ret = of_property_read_u32(np, "samsung,pmu-offset", &pd->offset);
	if (ret)
		return dev_err_probe(dev, ret,
				     "missing samsung,pmu-offset\n");

	if (of_property_read_string(np, "label", &label))
		label = np->name;

	pd->pd.name = devm_kstrdup_const(dev, label, GFP_KERNEL);
	if (!pd->pd.name)
		return -ENOMEM;

	pd->pd.power_on = gs201_pd_power_on;
	pd->pd.power_off = gs201_pd_power_off;

	/* Adopt whatever state the bootloader left the domain in. */
	ret = regmap_read(pd->pmureg, pd->offset + GS201_PD_STATUS_OFFSET,
			  &status);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read STATUS\n");

	is_off = !(status & GS201_PD_PWR_EN);
	dev_dbg(dev, "%s: PMU+0x%04x, bootloader left it %s\n",
		pd->pd.name, pd->offset, is_off ? "off" : "on");

	ret = pm_genpd_init(&pd->pd, NULL, is_off);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init genpd\n");

	ret = of_genpd_add_provider_simple(np, &pd->pd);
	if (ret) {
		pm_genpd_remove(&pd->pd);
		return dev_err_probe(dev, ret, "failed to add genpd provider\n");
	}

	platform_set_drvdata(pdev, pd);

	return 0;
}

static void gs201_pd_remove(struct platform_device *pdev)
{
	struct gs201_pm_domain *pd = platform_get_drvdata(pdev);

	of_genpd_del_provider(pdev->dev.of_node);
	pm_genpd_remove(&pd->pd);
}

static const struct of_device_id gs201_pd_of_match[] = {
	{ .compatible = "google,gs201-pd" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs201_pd_of_match);

static struct platform_driver gs201_pd_driver = {
	.probe	= gs201_pd_probe,
	.remove	= gs201_pd_remove,
	.driver	= {
		.name = "gs201-pm-domains",
		.of_match_table = gs201_pd_of_match,
		/*
		 * Domains must not disappear underneath their consumers, and
		 * there is no ordering story for tearing the provider down.
		 */
		.suppress_bind_attrs = true,
	},
};

static int __init gs201_pd_init(void)
{
	return platform_driver_register(&gs201_pd_driver);
}
core_initcall(gs201_pd_init);

MODULE_DESCRIPTION("gs201 (Tensor G2) power domain driver");
MODULE_LICENSE("GPL");
