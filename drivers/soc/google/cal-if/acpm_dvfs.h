/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Stub for AOSP's drivers/soc/google/cal-if/acpm_dvfs.h.
 *
 * The full cal-if (Chip Abstraction Layer interface) is a substantial AOSP
 * subsystem managing clocks, DVFS, and power domains via the ACPM
 * coprocessor protocol. exynos_pm_qos.c uses one function from it
 * (exynos_acpm_async_dvfs_enabled) to gate an async DVFS code path.
 *
 * Stub it to "disabled" so PM QoS uses its synchronous path; the full
 * cal-if can come in later if we need ACPM-driven DVFS.
 */
#ifndef _CAL_IF_ACPM_DVFS_STUB_H
#define _CAL_IF_ACPM_DVFS_STUB_H

static inline bool exynos_acpm_async_dvfs_enabled(void)
{
	return false;
}

#endif /* _CAL_IF_ACPM_DVFS_STUB_H */
