/* Control wireless devices power for p750 platform 
 * Copyright (C) Vladimir Ananiev <vovan888@gmail.com>
 * based on board-trout-rfkill.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Nick Pelly <npelly@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/gpio.h>

#include <mach/board-p750.h>

void rfkill_switch_all(enum rfkill_type type, enum rfkill_state state);

static struct rfkill *bt_rfk;
static struct rfkill *wifi_rfk;
static struct rfkill *gsm_rfk;
static struct rfkill *gps_rfk;

static int bluetooth_set_power(void *data, enum rfkill_state state)
{
	switch (state) {
	case RFKILL_STATE_UNBLOCKED:
		gpio_set_value(EGPIO_P750_BT_PWREN, 0);
		gpio_set_value(EGPIO_P750_BT_RESET_N, 0);
		udelay(10);
		gpio_set_value(EGPIO_P750_BT_PWREN, 1);
		udelay(3);
		gpio_set_value(EGPIO_P750_BT_RESET_N, 1);
		break;
	case RFKILL_STATE_SOFT_BLOCKED:
		gpio_set_value(EGPIO_P750_BT_PWREN, 0);
		gpio_set_value(EGPIO_P750_BT_RESET_N, 0);
		break;
	default:
		printk(KERN_ERR "bad bluetooth rfkill state %d\n", state);
	}
	return 0;
}

static int wifi_set_power(void *data, enum rfkill_state state)
{
	switch (state) {
	case RFKILL_STATE_UNBLOCKED:
		gpio_set_value(GPIO15_P750_WIFI_UNK1, 1);
		gpio_set_value(GPIO33_P750_WIFI_UNK2, 1);
		gpio_set_value(GPIO80_P750_WIFI_UNK3, 0);
		gpio_set_value(EGPIO_P750_WIFI_PWREN, 1);
		udelay(50);
		gpio_set_value(GPIO33_P750_WIFI_UNK2, 0);
		break;
	case RFKILL_STATE_SOFT_BLOCKED:
		gpio_set_value(GPIO15_P750_WIFI_UNK1, 1);
		gpio_set_value(GPIO33_P750_WIFI_UNK2, 1);
		gpio_set_value(GPIO80_P750_WIFI_UNK3, 1);
		gpio_set_value(EGPIO_P750_WIFI_PWREN, 0);
		gpio_set_value(GPIO33_P750_WIFI_UNK2, 0);
		break;
	default:
		printk(KERN_ERR "bad wifi rfkill state %d\n", state);
	}
	return 0;
}

static int gps_set_power(void *data, enum rfkill_state state)
{
	switch (state) {
	case RFKILL_STATE_UNBLOCKED:
		gpio_set_value(EGPIO_P750_GPS_PWREN, 0);
		gpio_set_value(EGPIO_P750_GPS_RESET_N, 0);
		udelay(10);
		gpio_set_value(EGPIO_P750_GPS_PWREN, 1);
		udelay(100);
		gpio_set_value(EGPIO_P750_GPS_RESET_N, 1);
		break;
	case RFKILL_STATE_SOFT_BLOCKED:
		gpio_set_value(EGPIO_P750_GPS_PWREN, 0);
		gpio_set_value(EGPIO_P750_GPS_RESET_N, 0);
		break;
	default:
		printk(KERN_ERR "bad gps rfkill state %d\n", state);
	}
	return 0;
}

static int gsm_set_power(void *data, enum rfkill_state state)
{
	/*TODO*/
	switch (state) {
	case RFKILL_STATE_UNBLOCKED:
		gpio_set_value(EGPIO_P750_BCR_MODEM_PWRON, 1);
		break;
	case RFKILL_STATE_SOFT_BLOCKED:
		gpio_set_value(EGPIO_P750_BCR_MODEM_PWRON, 0);
		break;
	default:
		printk(KERN_ERR "bad wifi rfkill state %d\n", state);
	}
	return 0;
}

static int p750_allocate_rfkill(struct device *dev, struct rfkill *rfkdev, char *name, int type, void *toggle_func)
{
	int rc = 0;

	rfkdev = rfkill_allocate(dev, type);
	if (!rfkdev)
		return -ENOMEM;

	rfkdev->name = name;
	rfkdev->state = RFKILL_STATE_SOFT_BLOCKED;
	/* userspace cannot take exclusive control */
	rfkdev->user_claim_unsupported = 1;
	rfkdev->user_claim = 0;
	rfkdev->data = NULL;  // user data
	rfkdev->toggle_radio = toggle_func;

	rc = rfkill_register(rfkdev);

	if (rc) {
		rfkill_free(rfkdev);
		printk(KERN_ERR "error registering %s \n", name);
	}

	return rc;
}

static int __init p750_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;

	/* default to bluetooth off */
	rfkill_switch_all(RFKILL_TYPE_BLUETOOTH, RFKILL_STATE_SOFT_BLOCKED);
	bluetooth_set_power(NULL, RFKILL_STATE_SOFT_BLOCKED);
	wifi_set_power(NULL, RFKILL_STATE_SOFT_BLOCKED);
	gps_set_power(NULL, RFKILL_STATE_SOFT_BLOCKED);
	gsm_set_power(NULL, RFKILL_STATE_SOFT_BLOCKED);

	rc = p750_allocate_rfkill(&pdev->dev, bt_rfk, "bt-rfkill", RFKILL_TYPE_BLUETOOTH, bluetooth_set_power);
	rc |= p750_allocate_rfkill(&pdev->dev, wifi_rfk, "wifi-rfkill", RFKILL_TYPE_WLAN, wifi_set_power);
	rc |= p750_allocate_rfkill(&pdev->dev, gps_rfk, "gps-rfkill", RFKILL_TYPE_UWB, gps_set_power);
	rc |= p750_allocate_rfkill(&pdev->dev, gsm_rfk, "gsm-rfkill", RFKILL_TYPE_WWAN, gsm_set_power);

	return rc;
}

static struct platform_driver p750_rfkill_driver = {
	.probe = p750_rfkill_probe,
	.driver = {
		.name = "p750_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init p750_rfkill_init(void)
{
	return platform_driver_register(&p750_rfkill_driver);
}

module_init(p750_rfkill_init);
MODULE_DESCRIPTION("p750 rfkill");
MODULE_AUTHOR("Vladimir Ananiev <vovan888@gmail.com>");
MODULE_LICENSE("GPL");
