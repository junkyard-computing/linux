/* stub: BTS bus-traffic-shaper no-ops for felix display bring-up */
#ifndef __STUB_SOC_GOOGLE_BTS_H
#define __STUB_SOC_GOOGLE_BTS_H
#include <linux/types.h>
struct bts_bw { unsigned int peak, read, write, rt; };
static inline unsigned int bts_get_bwindex(const char *name) { return 0; }
static inline unsigned int bts_get_scenindex(const char *name) { return 0; }
static inline unsigned int bts_get_urgent_lat_bts_index(const char *name) { return 0; }
static inline int bts_update_bw(unsigned int i, struct bts_bw bw) { return 0; }
static inline void bts_add_scenario(unsigned int i) {}
static inline void bts_del_scenario(unsigned int i) {}
static inline void bts_set_urgent_lat_read(unsigned int i, unsigned int v) {}
#endif
