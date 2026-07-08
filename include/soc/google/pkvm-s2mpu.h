/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Minimal mainline stub for the Google pKVM S2MPU API.
 *
 * The real header lives in the Pixel/Exynos vendor tree and is not present in
 * mainline Linux. samsung-iommu only calls pkvm_s2mpu_of_link(), and only when
 * IS_ENABLED(CONFIG_PKVM_S2MPU) (which is not defined in mainline, so the call
 * is compiled out). Provide a declaration so the code still type-checks.
 */
#ifndef __SOC_GOOGLE_PKVM_S2MPU_STUB_H
#define __SOC_GOOGLE_PKVM_S2MPU_STUB_H

struct device;

static inline int pkvm_s2mpu_of_link(struct device *dev)
{
	return 0;
}

#endif /* __SOC_GOOGLE_PKVM_S2MPU_STUB_H */
