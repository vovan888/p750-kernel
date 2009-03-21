/*
 * Keypad driver for lm8333 chip (keypad/gpio/pwm)
 * Copyright (C) 2009 Vladimir Ananiev <vovan888@gmail.com>
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

#include <linux/i2c.h>
#include <linux/i2c/lm8333.h>
#include <linux/input.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "lm8333-keypad"

struct lm8333_keypad_data {
	struct i2c_client *client;
	/* keypad interface */
	struct input_dev *input;
	struct lm8333_keypad_platform_data *pdata;
};

/* Keypad interface */
/*-----------------------------------------------------------------------*/
static int lm8333_keypad_decode(struct lm8333_keypad_data *lm8333_keypad, 
					u8 lm8333_keypad_event, int *state)
{
	int i, code = 0, event;
	unsigned int	*key_map;

	/* if MSB is set then it is key-press */
	*state = (lm8333_keypad_event & 0x80) ? 1 : 0;
	event = lm8333_keypad_event & 0x7f;
	key_map = lm8333_keypad->pdata->key_map;

	for (i = 0; i < lm8333_keypad->pdata->key_map_size; i++) {
		if ( (key_map[i] >> 24) == event) {
			code = key_map[i] & 0xffffff;
			break;
		}
	}
	return code;
}

static void lm8333_keypad_handler(void *hdata)
{
	int ret, i, code, state;
	u8 fifo[16];
	struct lm8333_keypad_data *lm8333_keypad = hdata;

	/* Read FIFO, returns number of bytes read */
	ret = lm8333_read_data(LM8333_FIFO_READ, 16, &fifo[0]);
	if (ret < 0) {
		pr_info(DRIVER_NAME ": error reading fifo=%d\n", ret);
		return;
	}

	for (i = 0; i < ret; i++) {
		if (fifo[i] == 0)
			break;
		code = lm8333_keypad_decode(lm8333_keypad, fifo[i], &state);
		if (code)
			input_event(lm8333_keypad->input, EV_KEY, code, state);
		else
			pr_info(DRIVER_NAME ": undefined keypad event, matrix code=%x,pressed=%d\n", 
					fifo[i] & 0x7f, (fifo[i] & 0x80)?1:0);
	}
	input_sync(lm8333_keypad->input);
}

static int __init lm8333_keypad_probe(struct platform_device *pdev)
{
	struct lm8333_keypad_platform_data *pdata = pdev->dev.platform_data;
	struct lm8333_keypad_data *lm8333_keypad;
	struct lm8333_plat_data *lm8333_plat;
	int i, ret = -EINVAL, code;
	struct input_dev *input;

	lm8333_keypad = kzalloc(sizeof *lm8333_keypad, GFP_KERNEL);
	if (!lm8333_keypad)
		return -ENOMEM;

	input = input_allocate_device();
	if (!input) {
		pr_err(DRIVER_NAME ": failed to allocate input device\n");
		goto fail2;
	}
	if (!pdata) {
		pr_err(DRIVER_NAME ": no platform data!\n");
		goto fail1;
	}

	if (pdev->dev.parent) {
		lm8333_plat = dev_get_drvdata(pdev->dev.parent);
		if (lm8333_plat) {
			lm8333_keypad->client = lm8333_plat->client;
		} else {
			pr_err(DRIVER_NAME ": no parent drvdata found!\n");
			goto fail1;
		}
	} else {
		pr_err(DRIVER_NAME ": no parent found!\n");
		goto fail1;
	}

	if (!lm8333_keypad->client) {
		pr_err(DRIVER_NAME ": i2c client is NULL!\n");
		goto fail1;
	}

	pr_err(DRIVER_NAME ": i2c client = %p\n", lm8333_keypad->client);

	platform_set_drvdata(pdev, lm8333_keypad);

	input->name = pdev->name;
	input->phys = "lm8333-keypad/input0";
	input->dev.parent = &pdev->dev;
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	lm8333_keypad->input = input;
	lm8333_keypad->pdata = pdata;

	for (i = 0; i < pdata->key_map_size; i++) {
		code = pdata->key_map[i] & 0xffffff;
		input_set_capability(input, EV_KEY, code);
	}

	ret = input_register_device(input);
	if (ret) {
		pr_err(DRIVER_NAME ": Unable to register input device, "
			"error: %d\n", ret);
		goto fail1;
	}

	/* Set debounce time */
	if (pdata->debounce_time) {
		ret = lm8333_write_reg(LM8333_DEBOUNCE, pdata->debounce_time / 3);
		if (ret)
			pr_err(DRIVER_NAME ": Unable to set keypad debounce time\n");
	}

	ret = lm8333_add_irq_work(LM8333_KEYPAD_IRQ, lm8333_keypad_handler, lm8333_keypad);
	if (ret) {
		pr_err(DRIVER_NAME ": failed to add keypad IRQ handler\n");
		goto fail0;
	}

	return 0;
fail0:
	input_unregister_device(input);
fail1:
	input_free_device(input);
fail2:
	kfree(lm8333_keypad);
	return ret;
}

static int __devexit lm8333_keypad_remove(struct platform_device *pdev)
{
	struct lm8333_keypad_data *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;

	device_init_wakeup(&pdev->dev, 0);
	lm8333_remove_irq_work(LM8333_KEYPAD_IRQ);
	input_unregister_device(input);
	kfree(ddata);

	return 0;
}

static struct platform_driver lm8333_keypad_platform_driver = {
	.driver = {
		.name		= "lm8333-keypad",
		.owner		= THIS_MODULE,
	},
	.probe		= lm8333_keypad_probe,
	.remove		= lm8333_keypad_remove,
};


static int __init lm8333_keypad_init(void)
{
	return platform_driver_register(&lm8333_keypad_platform_driver);
}

static void __exit lm8333_keypad_exit(void)
{
	platform_driver_unregister(&lm8333_keypad_platform_driver);
}

module_init(lm8333_keypad_init);
module_exit(lm8333_keypad_exit);

MODULE_DESCRIPTION("National Semiconductor LM8333 keypad driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vladimir Ananiev <vovan888@gmail.com>");
MODULE_ALIAS("platform:lm8333-keypad");
