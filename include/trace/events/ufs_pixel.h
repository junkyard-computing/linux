/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Mainline stub of AOSP's <trace/events/ufs_pixel.h>.
 *
 * AOSP's vendor UFS driver fires a single per-stat tracepoint
 * (trace_ufs_stats) used for io-latency histograms surfaced via tracefs.
 * The tracepoint definition lives in AOSP's modified
 * drivers/scsi/ufs/ufs_pixel.h and we don't have it. Stub trace_ufs_stats
 * to a no-op so the driver compiles; if we later need the histogram, the
 * right path is to port the TRACE_EVENT definition + the corresponding
 * tracefs hookup.
 */
#ifndef _TRACE_UFS_PIXEL_STUB_H
#define _TRACE_UFS_PIXEL_STUB_H

struct exynos_ufs;

static inline void trace_ufs_stats(struct exynos_ufs *ufs, u64 avg_time)
{
	(void)ufs;
	(void)avg_time;
}

#endif /* _TRACE_UFS_PIXEL_STUB_H */
