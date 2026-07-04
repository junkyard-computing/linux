/* stub: exynos PM QoS — opaque handle + no-op ops for felix display bring-up */
#ifndef __STUB_EXYNOS_PM_QOS_H
#define __STUB_EXYNOS_PM_QOS_H
#include <linux/types.h>
struct exynos_pm_qos_request { int pm_qos_class; s32 val; };
enum { PM_QOS_BUS_THROUGHPUT = 1, PM_QOS_DEVICE_THROUGHPUT, PM_QOS_DISPLAY_THROUGHPUT };
static inline void exynos_pm_qos_add_request(struct exynos_pm_qos_request *r, int c, s32 v) {}
static inline void exynos_pm_qos_update_request(struct exynos_pm_qos_request *r, s32 v) {}
static inline void exynos_pm_qos_remove_request(struct exynos_pm_qos_request *r) {}
static inline int exynos_pm_qos_request_active(struct exynos_pm_qos_request *r) { return 0; }
#endif
