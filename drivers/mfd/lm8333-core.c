/*
 * i2c driver for lm8333 chip (keypad/gpio/pwm)
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
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/pwm.h>


#define DRIVER_NAME			"lm8333-core"

/* Addresses to scan: none, device can't be detected */
static const unsigned short normal_i2c[] = { I2C_CLIENT_END };

/* Insmod parameters */
I2C_CLIENT_INSMOD;

static void lm8333_work(struct work_struct *_lm8333);

struct lm8333_core_data {
	struct mutex		lock;
	struct i2c_client	*client;
	struct work_struct	work;
	struct lm8333_i2c_platform_data	*pdata;

	u8	mask;
	void	(*handlers[LM8333_NUM_IRQS])(void *);
	void	*handler_data[LM8333_NUM_IRQS];
};

static struct lm8333_core_data *the_lm8333;
static struct i2c_driver lm8333_core_driver;

int lm8333_write_reg(int reg, u8 value)
{
	int val, tries;

	/* try to write 3 times to wake-up lm8333 from HALT mode */
	for (tries = 0; tries < 3; tries++) {
		val = i2c_smbus_write_byte_data(the_lm8333->client, reg, value);
		if (val < 0) {
			msleep(10);
			continue;
		} else
			return 0;
	}

	pr_err(DRIVER_NAME ": write error = %d\n", val);
	return val;
}
EXPORT_SYMBOL_GPL(lm8333_write_reg);

int lm8333_read_reg(int reg)
{
	int val, tries;
	
	/* try to read 3 times to wake-up lm8333 from HALT mode */
	for (tries = 0; tries < 3; tries++) {
		val = i2c_smbus_read_byte_data(the_lm8333->client, reg);
		if (val < 0)
			continue;
		else
			return val;
	}

	pr_err(DRIVER_NAME ": read error = %d\n", val);
	return val;
}
EXPORT_SYMBOL_GPL(lm8333_read_reg);

int lm8333_write_data(int reg, int length, u8 *data)
{
	int val, tries;

	/* try to write 3 times to wake-up lm8333 from HALT mode */
	for (tries = 0; tries < 3; tries++) {
		val = i2c_smbus_write_i2c_block_data(the_lm8333->client, reg, length, data);
		if (val < 0)
			continue;
		else
			return 0;
	}

	pr_err(DRIVER_NAME ": write data error = %d\n", val);
	return val;
}
EXPORT_SYMBOL_GPL(lm8333_write_data);

int lm8333_read_data(int reg, int length, u8 *data)
{
	int val, tries;
	
	/* try to read 3 times to wake-up lm8333 from HALT mode */
	for (tries = 0; tries < 3; tries++) {
		val = i2c_smbus_read_i2c_block_data(the_lm8333->client, reg, length, data);
		if (val < 0)
			continue;
		else
			return val;
	}

	pr_err(DRIVER_NAME ": read data error = %d\n", val);
	return val;
}
EXPORT_SYMBOL_GPL(lm8333_read_data);

static int lm8333_enable_irq(int irq)
{
	int val = 0;
	if (irq == LM8333_EX_0_IRQ || irq == LM8333_EX_1_IRQ) {
		the_lm8333->mask &= ~(1 << (irq - LM8333_EX_0_IRQ));
		val = lm8333_write_reg(LM8333_SET_EXT_INT,
					the_lm8333->mask);
	}
	return val;
}

static int lm8333_disable_irq(int irq)
{
	int val = 0;
	if (irq == LM8333_EX_0_IRQ || irq == LM8333_EX_1_IRQ) {
		the_lm8333->mask |= (1 << (irq - LM8333_EX_0_IRQ));
		val = lm8333_write_reg(LM8333_SET_EXT_INT,
					the_lm8333->mask);
	}
	return val;
}

/* Adds a handler for an interrupt. Does not run in interrupt context */
int lm8333_add_irq_work(int irq,
		void (*handler)(void *), void *data)
{
	int ret = 0;

	mutex_lock(&the_lm8333->lock);
	the_lm8333->handlers[irq] = handler;
	the_lm8333->handler_data[irq] = data;
	ret = lm8333_enable_irq(irq);
	mutex_unlock(&the_lm8333->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(lm8333_add_irq_work);

/* Removes handler for an interrupt */
int lm8333_remove_irq_work(int irq)
{
	int ret = 0;

	mutex_lock(&the_lm8333->lock);
	ret = lm8333_disable_irq(irq);
	the_lm8333->handlers[irq] = NULL;
	the_lm8333->handler_data[irq] = NULL;
	mutex_unlock(&the_lm8333->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(lm8333_remove_irq_work);

/* Error handler */
static void lm8333_error_handler(void *hdata)
{
	struct lm8333_core_data *lm8333_core_data = hdata;

	int error = lm8333_read_reg(LM8333_READ_ERROR);
	pr_err(DRIVER_NAME ": error handler = %d\n", error);
	if (error < 0)
		return;
	if (error & (LM8333_ERROR_CMDOVR | LM8333_ERROR_CMDUNK)) {
		/* Need something here ? */
	}
	if (error & (LM8333_ERROR_KEYOVR | LM8333_ERROR_FIFOOVR)) {
		/* Clear FIFO buffer */
		u8 fifo[16];
		lm8333_read_data(LM8333_FIFO_READ, 16, &fifo[0]);
	}
}

/* Handles lm8333 interrupts. Does not run in interrupt context */
/*-----------------------------------------------------------------------*/
static void lm8333_work(struct work_struct *_lm8333)
{
	struct lm8333_core_data *lm8333 =
			container_of(_lm8333, struct lm8333_core_data, work);
	void (*handler)(void *lm8333);

	while (1) {
		int isr = lm8333_read_reg(LM8333_READ_INT);
		if (isr < 0)
			break;
		isr &= ~lm8333->mask;

		while (isr) {
			int irq = fls(isr) - 1;
			isr &= ~(1 << irq);
			mutex_lock(&lm8333->lock);
			lm8333_disable_irq(irq);
			handler = lm8333->handlers[irq];
			if (handler)
				handler(the_lm8333->handler_data[irq]);
			lm8333_enable_irq(irq);
			mutex_unlock(&lm8333->lock);
		}
	}
	enable_irq(lm8333->client->irq);
}

/*
 * We cannot use I2C in interrupt context, so we just schedule work.
 */
static irqreturn_t lm8333_irq(int irq, void *_lm8333)
{
	struct lm8333_core_data *lm8333 = _lm8333;

	disable_irq_nosync(irq);
	(void)schedule_work(&lm8333->work);

	return IRQ_HANDLED;
}

/*-----------------------------------------------------------------------*/
/* Return 0 if detection is successful, -ENODEV otherwise */
static int lm8333_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* Now, we would do the remaining detection. But the LM8333 is plainly
	   impossible to detect.*/

	strlcpy(info->type, "lm8333", I2C_NAME_SIZE);

	return 0;
}

static int lm8333_core_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct lm8333_core_data	*lm8333;
	int			err = 0;

	if (the_lm8333) {
		pr_err(DRIVER_NAME ": only one device for now\n");
		return -ENODEV;
	}

	lm8333 = kzalloc(sizeof *lm8333, GFP_KERNEL);
	if (!lm8333)
		return -ENOMEM;

	i2c_set_clientdata(client, lm8333);

	the_lm8333 = lm8333;
	lm8333->client = client;

	lm8333->pdata = client->dev.platform_data;

	/* Disable all LM8333 interrupts */
	lm8333_disable_irq(LM8333_EX_0_IRQ);
	lm8333_disable_irq(LM8333_EX_1_IRQ);
	/* Mask only GPIO interrupts as others are always enabled */
	lm8333->mask = 0x06;

	if (client->irq > 0) {
		err = request_irq(client->irq, lm8333_irq, IRQF_DISABLED 
				| IRQF_TRIGGER_FALLING , DRIVER_NAME, lm8333);
		if (err) {
			pr_err(DRIVER_NAME ": can't get IRQ %d, err %d\n",
					client->irq, err);
			goto fail1;
		}
	}

	mutex_init(&lm8333->lock);
	INIT_WORK(&lm8333->work, lm8333_work);

	/* Set active time */
	if (lm8333->pdata->active_time) {
		err = lm8333_write_reg(LM8333_ACTIVE, lm8333->pdata->active_time / 3);
		if (err)
			pr_err(DRIVER_NAME ": Unable to set active time\n");
	}

	/* Disable PWM here until we support it */
	lm8333_write_reg(LM8333_PWM_CTL, 0);

	err = lm8333_add_irq_work(LM8333_ERROR_IRQ, lm8333_error_handler, lm8333);
	if (err) {
		pr_err(DRIVER_NAME ": failed to add error IRQ handler\n");
		goto fail2;
	}

	pr_info(DRIVER_NAME ": registered\n");

	return 0;
fail2:
	free_irq(client->irq, lm8333);
	flush_scheduled_work();
fail1:
	kfree(lm8333);
	return err;
}

static int __exit lm8333_core_remove(struct i2c_client *client)
{
	struct lm8333_core_data	*lm8333 = i2c_get_clientdata(client);

	lm8333_remove_irq_work(LM8333_ERROR_IRQ);
	free_irq(client->irq, lm8333);
	kfree(lm8333);
	i2c_set_clientdata(client, NULL);
	the_lm8333 = NULL;
	return 0;
}

static const struct i2c_device_id lm8333_id[] = {
	{ "lm8333", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm8333_id);

static struct i2c_driver lm8333_core_driver = {
	.driver = {
		.name		= "lm8333",
                .owner		= THIS_MODULE,
	},
	.probe		= lm8333_core_probe,
	.remove		= __exit_p(lm8333_core_remove),
	.id_table	= lm8333_id,

	.class		= I2C_CLASS_HWMON,	/* Nearest choice */
	.detect		= lm8333_detect,
	.address_data	= &addr_data,
};

static int __init lm8333_core_init(void)
{
	int err;

	err = i2c_add_driver(&lm8333_core_driver);
	if (err < 0) {
		pr_err(DRIVER_NAME ": driver registration failed\n");
		return err;
	}

	return 0;
}

static void __exit lm8333_core_exit(void)
{
	i2c_del_driver(&lm8333_core_driver);
}

MODULE_AUTHOR("Vladimir Ananiev <vovan888@gmail.com>");
MODULE_DESCRIPTION("I2C interface for LM8333 chip.");
MODULE_LICENSE("GPL");

module_init(lm8333_core_init);
module_exit(lm8333_core_exit);
