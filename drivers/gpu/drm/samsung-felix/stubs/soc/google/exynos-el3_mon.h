/* stub: EL3 monitor / secure PD interface — no-op for compile milestone */
#ifndef __STUB_EXYNOS_EL3_MON_H
#define __STUB_EXYNOS_EL3_MON_H
#include <linux/types.h>
#define EXYNOS_ERROR_TZASC_WRONG_REGION   (-1)
#define EXYNOS_ERROR_ALREADY_INITIALIZED  (1)
#define EXYNOS_ERROR_NOT_VALID_ADDRESS    (0x1000)
#define EXYNOS_GET_IN_PD_DOWN             (0)
#define EXYNOS_WAKEUP_PD_DOWN             (1)
#define RUNTIME_PM_TZPC_GROUP             (2)
static inline int exynos_pd_tz_save(unsigned int addr) { return 0; }
static inline int exynos_pd_tz_restore(unsigned int addr) { return 0; }
static inline int set_priv_reg(phys_addr_t reg, u32 val) { return 0; }
static inline int get_priv_reg(phys_addr_t reg) { return 0; }
#endif
