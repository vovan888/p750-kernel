/*
 * GPIO Joystick driver
 * Copyright (c) 2009 Vladimir Ananiev <vovan888@gmail.com>
 *
 * Based on:
 * Input driver for Asus A620
 * Based on Input driver for Dell Axim X5.
 *
 * Creates input events for the buttons on the device
 *
 * Copyright 2004 Matthew Garrett
 *                Nicolas Pouillon: Adaptation for A620
 *
 * Copyright 2006 Vincent Benony
 *                Remap all keys in order to be compatible with A716 drivers
 *                Some corrections about declaration of used keys
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/gpio_joystick.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <asm/mach/arch.h>

struct gpio_joystick_drvdata {
	struct input_dev *input;
	int gpios[5];
	int keycodes[5];
	/* Current joystick state bits: ENTER (bit 0), UP (1), RIGHT, DOWN, LEFT */
	u32 joystate;
};

/* This table is used to convert 4-bit joystick state into key indices.
 * If the bit combination have no logical meaning (say NW/SE are set at
 * the same time), we default to 0 (e.g. KEY_ENTER). At any given time
 * joystick cannot generate more than two pressed keys. So we're using
 * the lower nibble for key index 1 and upper nibble for key index 2
 * (if upper nibble is f, the particular bit combination generates just
 * one key press). Also note that a bit value of '1' means key is pressed,
 * while '0' means key is not pressed.
 */
static u8 joydecode[16] = {
/*                 SSNN */
/*                 WEEW */
	0xf0,			/* 0000 */
	0xf0,			/* 0001 */
	0xf0,			/* 0010 */
	0xf1,			/* 0011 */
	0xf0,			/* 0100 */
	0xf0,			/* 0101 */
	0xf4,			/* 0110 */
	0x14,			/* 0111 */
	0xf0,			/* 1000 */
	0xf2,			/* 1001 */
	0xf0,			/* 1010 */
	0x12,			/* 1011 */
	0xf3,			/* 1100 */
	0x32,			/* 1101 */
	0x34,			/* 1110 */
	0xf0,			/* 1111 */
};

static void joypad_decode(struct gpio_joystick_drvdata *ddata)
{
	int i, code, newstate = 0, down;

	/* read joystick state */
	for (i = 0; i < 4; i++) {
		if (gpio_get_value(ddata->gpios[i+1]))
			newstate |= (1 << i);
		else
			newstate &= ~(1 << i);
	}
	/* Decode current joystick bit combination */
	code = joydecode[newstate];
	newstate = gpio_get_value(ddata->gpios[0]) ?
	    ((1 << (code & 0x0f)) | (1 << (code >> 4))) : 0;
	/* We're not interested in other bits (e.g. 1 << 0xf) */
	newstate &= 0x1f;

	/* Find out which bits have changed */
	ddata->joystate ^= newstate;
	for (i = 0; i < 5; i++)
		if (ddata->joystate & (1 << i)) {
			down = (newstate & (1 << i)) ? 1 : 0;
			input_report_key(ddata->input, ddata->keycodes[i],
					 down);
			input_sync(ddata->input);
		}
	ddata->joystate = newstate;

/* 	pr_info("Keys state: %02x\n", ddata->joystate); */
}

irqreturn_t gpio_joystick_handle(int irq, void *dev_id)
{
	struct gpio_joystick_drvdata *ddata = dev_id;

	joypad_decode(ddata);

	return IRQ_HANDLED;
}

static int __devinit gpio_joystick_probe(struct platform_device *pdev)
{
	struct gpio_joystick_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_joystick_drvdata *ddata;
	struct input_dev *input;
	int error, i;

	ddata = kzalloc(sizeof(struct gpio_joystick_drvdata), GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		pr_err("gpio-joystick: Not enough memory\n");
		error = -ENOMEM;
		goto err_free_irq;
	}

	error =
	    request_irq(gpio_to_irq(pdata->gpios[0]), gpio_joystick_handle,
			IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"joystick push", ddata);
	if (error) {
		pr_err("gpio-joystick: Can't allocate irq\n");
		return -EBUSY;
	}

	platform_set_drvdata(pdev, ddata);

	for (i = 0; i < 5; i++) {
		input_set_capability(input, EV_KEY, pdata->keycodes[i]);
		ddata->gpios[i] = pdata->gpios[i];
		ddata->keycodes[i] = pdata->keycodes[i];
		error = gpio_request(ddata->gpios[i], "gpio-joystick");
		if (error) {
			pr_err("gpio-joystick: can't request gpio %x",
			       ddata->gpios[i]);
			goto err_free_dev;
		}
		error = gpio_direction_input(ddata->gpios[i]);
		if (error) {
			pr_err("gpio-joystick: can't set direction for gpio %x",
			       ddata->gpios[i]);
			goto err_free_dev;
		}
	}

//      set_bit(EV_KEY, input->evbit);

	input->name = "gpio-joystick";
	input->phys = "gpio-joystick/input0";
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	ddata->joystate = 0;
	ddata->input = input;

	error = input_register_device(input);
	if (error) {
		pr_err("gpio-joystick: Failed to register device\n");
		goto err_free_dev;
	}

	pr_info("%s installed\n", input->name);
	return 0;
      err_free_dev:
	input_free_device(input);
      err_free_irq:
	free_irq(gpio_to_irq(pdata->gpios[0]), NULL);
	kfree(ddata);
	return error;

}

static int __devexit gpio_joystick_remove(struct platform_device *pdev)
{
	struct gpio_joystick_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_joystick_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < 5; i++)
		gpio_free(pdata->gpios[i]);

	free_irq(gpio_to_irq(pdata->gpios[0]), NULL);
	input_unregister_device(input);
	input_free_device(input);

	return 0;
}

#define gpio_joystick_suspend	NULL
#define gpio_joystick_resume	NULL

static struct platform_driver gpio_keys_device_driver = {
	.probe = gpio_joystick_probe,
	.remove = __devexit_p(gpio_joystick_remove),
	.suspend = gpio_joystick_suspend,
	.resume = gpio_joystick_resume,
	.driver = {
		.name = "gpio-joystick",
		.owner = THIS_MODULE,
	}
};

static int __init gpio_joystick_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_joystick_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_joystick_init);
module_exit(gpio_joystick_exit);

MODULE_AUTHOR("Vladimir Ananiev <vovan888@gmail.com>");
MODULE_DESCRIPTION("GPIO Joystick support");
MODULE_LICENSE("GPL");
