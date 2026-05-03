/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Mainline stub for AOSP's <misc/sbbm.h> (Pixel Black-Box Monitor).
 *
 * On AOSP this is a Pixel-specific instrumentation framework; the vendor UFS
 * driver uses it to publish per-state signals (active/idle, power-on, io
 * outstanding) into a ring buffer that gets bundled with bug reports. We don't
 * have an equivalent on mainline, and these calls are diagnostic-only — the
 * driver's correctness doesn't depend on them firing — so stub them out.
 */
#ifndef _MISC_SBBM_STUB_H
#define _MISC_SBBM_STUB_H

enum {
	SBB_SIG_UFS_ACTIVE_IDLE,
	SBB_SIG_UFS_POWER_ON,
	SBB_SIG_UFS_IO_OUTSTANDING,
};

#define SBBM_SIGNAL_UPDATE(sig, val) do { (void)(sig); (void)(val); } while (0)

#endif /* _MISC_SBBM_STUB_H */
