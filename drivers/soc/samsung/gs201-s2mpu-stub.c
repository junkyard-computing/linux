// SPDX-License-Identifier: GPL-2.0
/*
 * gs201 S2MPU (Stage-2 MPU) stub driver.
 *
 * Felix gs201 has per-master S2MPU instances that gate AXI transactions
 * from each subsystem master into system memory; the AOSP DT carries
 * `s2mpus = <&s2mpu_hsi0>;` on the USB PHY node so the AOSP s2mpu driver
 * (lives in private/ on the AOSP tree, not in this checkout) can claim
 * the device, ioremap the regs, and program per-client allow tables
 * during HSI0 power transitions.
 *
 * Mainline does not have an S2MPU driver. This stub exists ONLY to claim
 * the `google,s2mpu` OF compatible so:
 *   - The DT node binds (no "deferred probe pending: orphan" spam).
 *   - Probe-order dependencies (e.g. anything that uses the s2mpus
 *     phandle on another node) resolve cleanly.
 *   - We can confirm via UART that s2mpu_hsi0 is being seen at boot.
 *
 * It does NOT touch any S2MPU registers. The bootloader-programmed
 * permissions persist (no kernel writes), and the hypothesis being tested
 * is whether ATF/firmware power-domain transitions key off having a
 * registered driver claim the device. If the felix USB SETUP-delivery
 * symptom changes after this stub is in place, that's evidence S2MPU
 * gating is in the loop and a real driver port is justified.
 *
 * If the symptom does NOT change, S2MPU is exonerated and this stub
 * stays as scaffolding for any future work that does need to program
 * the device.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

struct gs201_s2mpu {
	struct device *dev;
	void __iomem *regs;
};

static int gs201_s2mpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gs201_s2mpu *s;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = dev;
	s->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(s->regs))
		return PTR_ERR(s->regs);

	platform_set_drvdata(pdev, s);

	dev_info(dev, "S2MPU stub claimed (regs=%p, no register writes)\n",
		 s->regs);
	return 0;
}

static const struct of_device_id gs201_s2mpu_match[] = {
	{ .compatible = "google,s2mpu" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs201_s2mpu_match);

static struct platform_driver gs201_s2mpu_driver = {
	.driver = {
		.name = "gs201-s2mpu-stub",
		.of_match_table = gs201_s2mpu_match,
	},
	.probe = gs201_s2mpu_probe,
};
module_platform_driver(gs201_s2mpu_driver);

MODULE_DESCRIPTION("gs201 S2MPU stub (probe-only, no register writes)");
MODULE_LICENSE("GPL");
