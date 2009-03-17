/*
 * w1-gpio2 interface to platform code
 * Copyright (C) 2009 Vladimir Ananiev <vovan888@gmail.com>
 *
 * Copyright (C) 2007 Ville Syrjala <syrjala@sci.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */
#ifndef _LINUX_W1_GPIO2_H
#define _LINUX_W1_GPIO2_H

/**
 * struct w1_gpio2_platform_data - Platform-dependent data for w1-gpio2
 * @pin_read: read bus GPIO pin
 * @pin_write: write to bus GPIO pin
 */
struct w1_gpio2_platform_data {
	unsigned int pin_read;
	unsigned int pin_write;
};

#endif /* _LINUX_W1_GPIO2_H */
