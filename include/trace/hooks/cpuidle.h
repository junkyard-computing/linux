/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Mainline stub of AOSP's android_vh_cpu_idle_{enter,exit} vendor hooks.
 *
 * Same pattern as include/trace/hooks/ufshcd.h: AOSP fires these from its
 * cpuidle core to let vendor modules (drivers/soc/google/exynos-cpupm.c)
 * observe enter/exit. Mainline doesn't carry the ANDROID_VENDOR_HOOKS
 * framework and our cpuidle core doesn't fire these, so register/unregister
 * become trivial no-ops.
 */
#ifndef _TRACE_HOOK_CPUIDLE_STUB_H
#define _TRACE_HOOK_CPUIDLE_STUB_H

#define _CPUIDLE_VH_STUB(name)						\
	static inline int register_trace_##name(void *probe, void *data) \
	{ return 0; }							\
	static inline int unregister_trace_##name(void *probe, void *data) \
	{ return 0; }

_CPUIDLE_VH_STUB(android_vh_cpu_idle_enter)
_CPUIDLE_VH_STUB(android_vh_cpu_idle_exit)

#undef _CPUIDLE_VH_STUB

#endif /* _TRACE_HOOK_CPUIDLE_STUB_H */
