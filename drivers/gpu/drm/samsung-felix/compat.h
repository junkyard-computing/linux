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

/* --- wave 1: more mainline includes + removed-API shims --- */
#include <linux/vmalloc.h>
#include <linux/hex.h>
#include <linux/fb.h>
#include <linux/err.h>
#ifndef FB_BLANK_UNBLANK
#define FB_BLANK_UNBLANK    0
#define FB_BLANK_POWERDOWN  4
#endif
/* iommu device-fault-handler API removed in mainline (macros swallow args) */
#define iommu_register_device_fault_handler(dev, handler, data) (0)
#define iommu_unregister_device_fault_handler(dev) (0)
/* dma-heap consumer funcs absent from mainline's linux/dma-heap.h */
struct dma_heap;
struct dma_buf;
static inline struct dma_heap *dma_heap_find(const char *name) { return NULL; }
static inline void dma_heap_put(struct dma_heap *h) {}
static inline struct dma_buf *dma_heap_buffer_alloc(struct dma_heap *h, size_t len,
			unsigned int fd_flags, unsigned int heap_flags) { return ERR_PTR(-ENOSYS); }
/* exynos idle-IP: arg-count-agnostic no-ops (override earlier inline decls) */
#undef exynos_get_idle_ip_index
#undef exynos_update_ip_idle_status
#define exynos_get_idle_ip_index(...) (-1)
#define exynos_update_ip_idle_status(...) do {} while (0)
#include <soc/google/exynos_pm_qos.h>
/* drm_panel_init removed for refcounted devm_drm_panel_alloc; the panel is
 * embedded in exynos_panel (devm-managed) so replicate the old init. */
#include <drm/drm_panel.h>
static inline void drm_panel_init(struct drm_panel *panel, struct device *dev,
			const struct drm_panel_funcs *funcs, int connector_type)
{
	INIT_LIST_HEAD(&panel->list);
	panel->dev = dev;
	panel->funcs = funcs;
	panel->connector_type = connector_type;
}
