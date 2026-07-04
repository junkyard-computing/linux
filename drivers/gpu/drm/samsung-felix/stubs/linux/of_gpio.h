/* stub: linux/of_gpio.h removed in mainline. of_get_named_gpio* return -ENOSYS
 * for the compile milestone; panel reset must be rewired to devm_gpiod_get()
 * in the bring-up phase (see SCREEN_BRINGUP_PLAN.md). */
#ifndef __STUB_OF_GPIO_H
#define __STUB_OF_GPIO_H
#include <linux/errno.h>
#include <linux/of.h>
enum of_gpio_flags { OF_GPIO_ACTIVE_LOW = 0x1, };
static inline int of_get_named_gpio(const struct device_node *np,
				    const char *propname, int index)
{ return -ENOSYS; }
static inline int of_get_named_gpio_flags(const struct device_node *np,
			const char *propname, int index, enum of_gpio_flags *f)
{ if (f) *f = 0; return -ENOSYS; }
#endif
