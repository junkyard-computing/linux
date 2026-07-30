/* SPDX-License-Identifier: GPL-2.0 or MIT */
/* Copyright 2026 ARM Limited. All rights reserved. */
/*
 * Compatibility shim for the backported performance-counter series.
 *
 * Upstream consolidated the per-block register headers into a single
 * panthor_regs.h. This tree predates that refactor and still uses the split
 * headers (panthor_gpu_regs.h, panthor_mmu_regs.h, ...). Re-export them here so
 * the perfcnt code, which includes "panthor_regs.h", resolves against our
 * existing definitions instead of pulling in a duplicate consolidated header.
 */
#ifndef __PANTHOR_REGS_H__
#define __PANTHOR_REGS_H__

#include "panthor_fw_regs.h"
#include "panthor_gpu_regs.h"
#include "panthor_mmu_regs.h"

/*
 * Register-field accessors introduced by the upstream register-header
 * consolidation that the perfcnt series depends on but which our split headers
 * predate. Add them here rather than editing the split headers.
 */
#ifndef GPU_MEM_FEATURES_L2_SLICES
#define GPU_MEM_FEATURES_L2_SLICES(x)		((((x) & GENMASK(11, 8)) >> 8) + 1)
#endif

#endif /* __PANTHOR_REGS_H__ */
