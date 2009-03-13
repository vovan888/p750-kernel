#ifndef __JBT6K74_H__
#define __JBT6K74_H__

#include <linux/spi/spi.h>

struct jbt6k74_platform_data {
	void (*reset)(int devindex, int level);
	void (*resuming)(int devindex); /* called when LCM is resumed */
	void (*probe_completed)(struct device *dev);
	int (*get_power_status)(int devindex);
};

enum jbt_state {
	JBT_STATE_DEEP_STANDBY,
	JBT_STATE_SLEEP,
	JBT_STATE_NORMAL,
	JBT_STATE_QVGA_NORMAL,
};

struct jbt_info {
	enum jbt_state state, normal_state;
	struct spi_device *spi_dev;
	struct mutex lock;		/* protects tx_buf and reg_cache */
	struct notifier_block fb_notif;
	u16 tx_buf[8];
	u16 reg_cache[0xEE];
	int have_resumed;
};

extern int jbt6k74_enter_state(struct jbt_info *jbt, enum jbt_state new_state);

#endif
