/* SPDX-License-Identifier: GPL-2.0 */
/* mainline 7.1 compatibility shim for the AOSP samsung display graft.
 * Force-included (-include) before all driver TUs. */
#ifndef __SAMSUNG_FELIX_COMPAT_H
#define __SAMSUNG_FELIX_COMPAT_H

/* commit 5164f7e7ff8e "drm: Rename struct drm_atomic_state to
 * drm_atomic_commit" — the AOSP 6.1 driver predates the rename. */
#define drm_atomic_state		drm_atomic_commit
#define drm_atomic_state_alloc		drm_atomic_commit_alloc
#define drm_atomic_state_clear		drm_atomic_commit_clear
#define drm_atomic_state_get		drm_atomic_commit_get
#define drm_atomic_state_put		drm_atomic_commit_put

#endif /* __SAMSUNG_FELIX_COMPAT_H */

/* drm_debug_printer(prefix) -> drm_dbg_printer(dev,category,prefix) */
#include <drm/drm_print.h>
#define drm_debug_printer(prefix) drm_dbg_printer(NULL, DRM_UT_DRIVER, (prefix))

/* mainline moved several APIs behind explicit includes the AOSP tree got
 * transitively; force them here. */
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/string.h>
#include <linux/pinctrl/consumer.h>
#include <linux/dma-fence.h>
/* strlcpy removed in mainline (6.8); strscpy is the replacement. */
#ifndef strlcpy
#define strlcpy strscpy
#endif
/* exynos idle-IP tracking (soc/google) — no-op for bring-up */
static inline int exynos_get_idle_ip_index(const char *name, int a) { return -1; }
static inline void exynos_update_ip_idle_status(int idx, int status) {}
