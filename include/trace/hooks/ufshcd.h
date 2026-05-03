/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Mainline stub of AOSP's android_vh_ufs_* / android_rvh_ufs_* vendor hooks.
 *
 * On AOSP, these are tracepoints declared via DECLARE_HOOK in include/trace/hooks/
 * and fired from drivers/ufs/core/ufshcd.c. The vendor UFS driver
 * (drivers/ufs/host/exynos-gs/ufs-pixel.c) registers callbacks against them via
 * register_trace_<hook>(); on AOSP this lets the vendor module observe and
 * extend core UFS behavior without modifying the upstream code.
 *
 * Mainline doesn't carry the ANDROID_VENDOR_HOOKS framework, and our copy of
 * mainline's UFS core doesn't fire any of these hooks. Stubbing them out as
 * inline no-ops lets the vendor driver compile and run; the registered
 * callbacks will simply never be invoked, which matches what the kernel would
 * do anyway since nothing in mainline calls trace_android_vh_ufs_*().
 *
 * If we later need to actually observe these events from the vendor driver,
 * the right path is to port the ANDROID_VENDOR_HOOKS framework
 * (kernel/trace/trace_hooks.c + DECLARE_HOOK / DECLARE_RESTRICTED_HOOK macros)
 * and add the trace_*() call sites into mainline's drivers/ufs/core/ufshcd.c.
 */
#ifndef _TRACE_HOOK_UFSHCD_STUB_H
#define _TRACE_HOOK_UFSHCD_STUB_H

#define _UFS_VH_STUB(name)						\
	static inline int register_trace_##name(void *probe, void *data) \
	{ return 0; }							\
	static inline int unregister_trace_##name(void *probe, void *data) \
	{ return 0; }

_UFS_VH_STUB(android_vh_ufs_fill_prdt)
_UFS_VH_STUB(android_rvh_ufs_reprogram_all_keys)
_UFS_VH_STUB(android_rvh_ufs_complete_init)
_UFS_VH_STUB(android_vh_ufs_prepare_command)
_UFS_VH_STUB(android_vh_ufs_update_sysfs)
_UFS_VH_STUB(android_vh_ufs_send_command)
_UFS_VH_STUB(android_vh_ufs_compl_command)
_UFS_VH_STUB(android_vh_ufs_send_uic_command)
_UFS_VH_STUB(android_vh_ufs_send_tm_command)
_UFS_VH_STUB(android_vh_ufs_check_int_errors)
_UFS_VH_STUB(android_vh_ufs_update_sdev)
_UFS_VH_STUB(android_vh_ufs_clock_scaling)
_UFS_VH_STUB(android_vh_ufs_use_mcq_hooks)
_UFS_VH_STUB(android_vh_ufs_mcq_abort)

#undef _UFS_VH_STUB

#endif /* _TRACE_HOOK_UFSHCD_STUB_H */
