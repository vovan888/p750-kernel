/*
 * w1-gpio2 - GPIO w1 bus master driver with separate GPIOs
 * to read and write to the bus
 * Copyright (C) 2009 Vladimir Ananiev <vovan888@gmail.com>
 *
 * based on w1-gpio - GPIO w1 bus master driver
 * Copyright (C) 2007 Ville Syrjala <syrjala@sci.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/w1-gpio2.h>

#include "../w1.h"
#include "../w1_int.h"

#include <asm/gpio.h>

static void w1_gpio2_write_bit(void *data, u8 bit)
{
	struct w1_gpio2_platform_data *pdata = data;

	gpio_set_value(pdata->pin_write, bit);
}

static u8 w1_gpio2_read_bit(void *data)
{
	int val;
	struct w1_gpio2_platform_data *pdata = data;

	if (gpio_get_value(pdata->pin_read))
		val = 1;
	else
		val = 0;

	return val;
}

static int __init w1_gpio2_probe(struct platform_device *pdev)
{
	struct w1_bus_master *master;
	struct w1_gpio2_platform_data *pdata = pdev->dev.platform_data;
	int err;

	if (!pdata)
		return -ENXIO;

	master = kzalloc(sizeof(struct w1_bus_master), GFP_KERNEL);
	if (!master)
		return -ENOMEM;

	err = gpio_request(pdata->pin_write, "w1-write");
	err |= gpio_request(pdata->pin_read, "w1-read");
	if (err)
		goto free_master;

	master->data = pdata;
	master->read_bit = w1_gpio2_read_bit;
	master->write_bit = w1_gpio2_write_bit;

	gpio_direction_input(pdata->pin_read);
	gpio_direction_output(pdata->pin_write, 1);

	err = w1_add_master_device(master);
	if (err)
		goto free_gpio;

	platform_set_drvdata(pdev, master);

	return 0;

 free_gpio:
	gpio_free(pdata->pin_read);
	gpio_free(pdata->pin_write);
 free_master:
	kfree(master);

	return err;
}

static int __exit w1_gpio2_remove(struct platform_device *pdev)
{
	struct w1_bus_master *master = platform_get_drvdata(pdev);
	struct w1_gpio2_platform_data *pdata = pdev->dev.platform_data;

	w1_remove_master_device(master);
	gpio_free(pdata->pin_read);
	gpio_free(pdata->pin_write);
	kfree(master);

	return 0;
}

static struct platform_driver w1_gpio2_driver = {
	.driver = {
		.name	= "w1-gpio2",
		.owner	= THIS_MODULE,
	},
	.remove	= __exit_p(w1_gpio2_remove),
};

static int __init w1_gpio2_init(void)
{
	return platform_driver_probe(&w1_gpio2_driver, w1_gpio2_probe);
}

static void __exit w1_gpio2_exit(void)
{
	platform_driver_unregister(&w1_gpio2_driver);
}

module_init(w1_gpio2_init);
module_exit(w1_gpio2_exit);

MODULE_DESCRIPTION("GPIO2 w1 bus master driver");
MODULE_AUTHOR("Vladimir Ananiev <vovan888@gmail.com>");
MODULE_LICENSE("GPL");
