/*
 * Core driver for lm8333 chip (keypad/gpio/pwm)
 * Handles GPIO and i2c device creation
 * Copyright (C) 2009 Vladimir Ananiev <vovan888@gmail.com>
 *
 * some bits based on:
 *  pca953x.c - 4/8/16 bit I/O ports
 *
 *  Copyright (C) 2005 Ben Gardner <bgardner@wabtec.com>
 *  Copyright (C) 2007 Marvell International Ltd.
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
*/

/* Does not support GPIO interrupts  and PWM */

#include <linux/i2c.h>
#include <linux/i2c/lm8333.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <asm/gpio.h>

#define DRIVER_NAME "lm8333-gpio"

struct lm8333_gpio_data {
	struct i2c_client *client;
	/* GPIO interface */
	struct gpio_chip 	gpio_chip;
	uint8_t reg_output;
	uint8_t reg_direction;
};

/* GPIO interface */
/*-----------------------------------------------------------------------*/
static int lm8333_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct lm8333_gpio_data *chip;
	u8 reg_val;
	int ret;

	chip = container_of(gc, struct lm8333_gpio_data, gpio_chip);

	reg_val = chip->reg_direction | (1u << off);
	ret = lm8333_write_reg(LM8333_GEN_IO_DIR, reg_val);
	if (ret)
		return ret;

	chip->reg_direction = reg_val;
	return 0;
}

static int lm8333_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct lm8333_gpio_data *chip;
	u8 reg_val;
	int ret;

	chip = container_of(gc, struct lm8333_gpio_data, gpio_chip);

	/* set output level */
	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	ret = lm8333_write_reg(LM8333_GEN_IO_OUT, reg_val);
	if (ret)
		return ret;

	chip->reg_output = reg_val;

	/* then direction */
	reg_val = chip->reg_direction & ~(1u << off);
	ret = lm8333_write_reg(LM8333_GEN_IO_DIR, reg_val);
	if (ret)
		return ret;

	chip->reg_direction = reg_val;
	return 0;
}
static int lm8333_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct lm8333_gpio_data *chip;
	int ret;

	chip = container_of(gc, struct lm8333_gpio_data, gpio_chip);

	ret = lm8333_read_reg(LM8333_GEN_IO_IN);
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return (ret & (1u << off)) ? 1 : 0;
}

static void lm8333_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct lm8333_gpio_data *chip;
	u8 reg_val;
	int ret;

	chip = container_of(gc, struct lm8333_gpio_data, gpio_chip);

	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	ret = lm8333_write_reg(LM8333_GEN_IO_OUT, reg_val);
	if (ret)
		return;

	chip->reg_output = reg_val;
}

/*-----------------------------------------------------------------------*/
static int __init lm8333_gpio_probe(struct platform_device *pdev)
{
	struct lm8333_gpio_platform_data *pdata = pdev->dev.platform_data;
	struct lm8333_gpio_data *lm8333_gpio;
	struct lm8333_plat_data *lm8333_plat;
	int		err = -EINVAL;

	lm8333_gpio = kzalloc(sizeof *lm8333_gpio, GFP_KERNEL);
	if (!lm8333_gpio)
		return -ENOMEM;

	if (pdata == NULL) {
		pr_err(DRIVER_NAME ": no platform data defined\n");
		goto fail1;
	}

	if (pdev->dev.parent) {
		lm8333_plat = dev_get_drvdata(pdev->dev.parent);
		if (lm8333_plat) {
			lm8333_gpio->client = lm8333_plat->client;
		} else {
			pr_err(DRIVER_NAME ": no parent drvdata found!\n");
			goto fail1;
		}
	} else {
		pr_err(DRIVER_NAME ": no parent found!\n");
		goto fail1;
	}

	if (!lm8333_gpio->client) {
		pr_err(DRIVER_NAME ": i2c client is NULL!\n");
		goto fail1;
	}

	pr_err(DRIVER_NAME ": i2c client = %p\n", lm8333_gpio->client);

	lm8333_gpio->gpio_chip.label = "lm8333-gpio";
	lm8333_gpio->gpio_chip.dev = &(pdev->dev);
	lm8333_gpio->gpio_chip.owner = THIS_MODULE;
	lm8333_gpio->gpio_chip.get             = lm8333_gpio_get_value;
	lm8333_gpio->gpio_chip.set             = lm8333_gpio_set_value;
	lm8333_gpio->gpio_chip.direction_input = lm8333_gpio_direction_input;
	lm8333_gpio->gpio_chip.direction_output = lm8333_gpio_direction_output;
	lm8333_gpio->gpio_chip.base = pdata->gpio_base;
	lm8333_gpio->gpio_chip.ngpio           = 4;

	err = gpiochip_add(&(lm8333_gpio->gpio_chip));
	if (err) {
		pr_err(DRIVER_NAME ": failed to add gpio chip\n");
		goto fail1;
	}
	
	platform_set_drvdata(pdev, lm8333_gpio);
	pr_info(DRIVER_NAME ": registered\n");

	return 0;
fail1:
	kfree(lm8333_gpio);
	return err;
}

static int __devexit lm8333_gpio_remove(struct platform_device *pdev)
{
	struct lm8333_gpio_data *ddata = platform_get_drvdata(pdev);
	int err;

	err = gpiochip_remove(&(ddata->gpio_chip));
	if (err) {
		pr_err(DRIVER_NAME ": failed to remove gpio chip\n");
	}
	kfree(ddata);

	return 0;
}

#ifdef CONFIG_PM
static int lm8333_gpio_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct lm8333_platform_data *pdata = platform_get_drvdata(pdev);

	if (pdata->irq && device_may_wakeup(&pdev->dev))
		enable_irq_wake(pdata->irq);
	return 0;
}

static int lm8333_gpio_resume(struct platform_device *pdev)
{
	struct lm8333_platform_data *pdata = platform_get_drvdata(pdev);

	if (pdata->irq && device_may_wakeup(&pdev->dev))
		disable_irq_wake(pdata->irq);
	return 0;
}
#else
#define lm8333_gpio_suspend NULL
#define lm8333_gpio_resume NULL
#endif

static struct platform_driver lm8333_gpio_platform_driver = {
	.driver = {
		.name		= "lm8333-gpio",
		.owner		= THIS_MODULE,
	},
	.probe		= lm8333_gpio_probe,
	.remove		= lm8333_gpio_remove,
	.suspend        = lm8333_gpio_suspend,
	.resume         = lm8333_gpio_resume,
};


static int __init lm8333_gpio_init(void)
{
	return platform_driver_register(&lm8333_gpio_platform_driver);
}

static void __exit lm8333_gpio_exit(void)
{
	platform_driver_unregister(&lm8333_gpio_platform_driver);
}

MODULE_DESCRIPTION("National Semiconductor LM8333 gpio driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vladimir Ananiev <vovan888@gmail.com>");
MODULE_ALIAS("platform:lm8333-gpio");

module_init(lm8333_gpio_init);
module_exit(lm8333_gpio_exit);
