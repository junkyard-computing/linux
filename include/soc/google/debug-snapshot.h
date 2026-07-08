/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Minimal mainline stub for the AOSP/Google debug-snapshot (dss) API.
 *
 * The real header lives in the Pixel/Exynos vendor tree and is not present in
 * mainline Linux. The samsung-iommu driver only needs a handful of symbols to
 * build; provide no-op equivalents here so it links standalone. If the vendor
 * debug-snapshot subsystem is ever ported, replace this stub.
 */
#ifndef __SOC_GOOGLE_DEBUG_SNAPSHOT_STUB_H
#define __SOC_GOOGLE_DEBUG_SNAPSHOT_STUB_H

#include <linux/types.h>

/*
 * dpm "policy" action identifiers. Only GO_PANIC_ID is referenced by
 * samsung-iommu (as the default panic-action). Value is arbitrary for the
 * stub; keep it distinct/non-zero so it never accidentally matches a real
 * DT-provided action id of 0.
 */
#ifndef GO_PANIC_ID
#define GO_PANIC_ID		0xffffffffU
#endif

static inline void dbg_snapshot_do_dpm_policy(unsigned int policy,
					      const char *str)
{
	/* no-op stub */
}

#endif /* __SOC_GOOGLE_DEBUG_SNAPSHOT_STUB_H */
