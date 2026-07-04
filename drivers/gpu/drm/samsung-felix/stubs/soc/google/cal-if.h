/* stub: Exynos CAL power/clock interface — NO-OP for compile milestone.
 * Real ACPM/EL3-SMC power-on (SMC_CMD_PREPARE_PD_ONOFF) + clk rates land in
 * the bring-up phase; see SCREEN_BRINGUP_PLAN.md M1.5. */
#ifndef __STUB_SOC_GOOGLE_CALIF_H
#define __STUB_SOC_GOOGLE_CALIF_H
#include <linux/types.h>
static inline int cal_pd_control(unsigned int id, int on) { return 0; }
static inline int cal_pd_status(unsigned int id) { return 1; }
static inline int cal_pd_set_smc_id(unsigned int id, int need_smc) { return 0; }
static inline unsigned long cal_dfs_get_rate(unsigned int id) { return 0; }
static inline int cal_dfs_set_rate(unsigned int id, unsigned long rate) { return 0; }
static inline int cal_clk_setrate(unsigned int id, unsigned long rate) { return 0; }
static inline unsigned long cal_clk_getrate(unsigned int id) { return 0; }
static inline int cal_clk_enable(unsigned int id) { return 0; }
static inline int cal_clk_disable(unsigned int id) { return 0; }
static inline unsigned long cal_dfs_get_max_freq(unsigned int id) { return 0; }
#define ACPM_DVFS_DISP 0
#endif
