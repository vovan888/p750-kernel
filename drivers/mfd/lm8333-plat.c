/*
 * Platform driver for lm8333 chip (keypad/gpio/pwm)
 * Copyright (C) 2009 Vladimir Ananiev <vovan888@gmail.com>
 *
 * Some parts based on menelaus.c:
 * Copyright (C) 2004 Texas Instruments
 * Copyright (C) 2004-2005 David Brownell
 * Copyright (C) 2005, 2006 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * 
 * GPIO interrupts and PWM interface are not supported
 */

#include <linux/module.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c/lm8333.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#define DRIVER_NAME "lm8333"

static int lm8333_plat_probe(struct platform_device *pdev)
{
	struct lm8333_platform_data *pdata = pdev->dev.platform_data;
	struct lm8333_plat_data *lm8333_plat;
	struct i2c_board_info i2c_info;
	struct i2c_adapter *adapter;
	int		err = -EINVAL;

	lm8333_plat = kzalloc(sizeof *lm8333_plat, GFP_KERNEL);
	if (!lm8333_plat)
		return -ENOMEM;

	if (pdata == NULL) {
		pr_err(DRIVER_NAME ": no platform data defined\n");
		goto fail1;
	}

	strlcpy(i2c_info.type, "lm8333", sizeof(i2c_info.type));
	i2c_info.addr = LM8333_I2C_ADDRESS;
	i2c_info.platform_data = pdata->lm8333_i2c_pdata;
	i2c_info.irq = pdata->irq;
	i2c_info.flags = 0;
	i2c_info.archdata = NULL;
	
	/* Register i2c device */
	adapter = i2c_get_adapter(pdata->i2c_adapter_id);
	if (adapter == NULL) {
		pr_err(DRIVER_NAME ": failed to get i2c_adapter for id=%x\n",pdata->i2c_adapter_id);
		goto fail1;
	}

	lm8333_plat->client = i2c_new_device(adapter, &(i2c_info));
	if (lm8333_plat->client == NULL) {
		pr_err(DRIVER_NAME ": failed to register i2c device\n");
		goto fail1;
	}

	platform_set_drvdata(pdev, lm8333_plat);

	return 0;
fail1:
	kfree(lm8333_plat);
	return err;
}

static int lm8333_plat_remove(struct platform_device *pdev)
{
	struct lm8333_plat_data *lm8333_plat = platform_get_drvdata(pdev);

	i2c_unregister_device(lm8333_plat->client);
	kfree(lm8333_plat);
	return 0;
}

static struct platform_driver lm8333_platform_driver = {
	.driver = {
		.name		= "lm8333",
		.owner		= THIS_MODULE,
	},
	.probe		= lm8333_plat_probe,
	.remove		= lm8333_plat_remove,
};

static int __init lm8333_core_init(void)
{
	return platform_driver_register(&lm8333_platform_driver);
}

static void __exit lm8333_core_exit(void)
{
	platform_driver_unregister(&lm8333_platform_driver);
}

MODULE_DESCRIPTION("Platform driver for LM8333 chip.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vladimir Ananiev <vovan888@gmail.com>");
MODULE_ALIAS("platform:lm8333");

module_init(lm8333_core_init);
module_exit(lm8333_core_exit);
