/* Platform data for LM8333 keypad/GPIO/PWM device */

#ifndef _LM8333_H
#define _LM8333_H

#include <linux/i2c.h>

#define LM8333_I2C_ADDRESS		0x51	/*0xA2 >> 1*/

/* Registers */
#define LM8333_FIFO_READ		0x20
#define LM8333_RPT_FIFO_READ		0x21
#define LM8333_DEBOUNCE			0x22
#define LM8333_GEN_IO_IN		0x30
#define LM8333_GEN_IO_OUT		0x31
#define LM8333_GEN_IO_DIR		0x32
#define LM8333_PWM_HI			0x40
#define LM8333_PWM_LO			0x41
#define LM8333_PWM_CTL			0x42
#define LM8333_READ_INT			0xD0
#define LM8333_SET_EXT_INT		0xD1
#define LM8333_READ_STAT		0xE0
#define LM8333_SCAN_REQ			0xE3
#define LM8333_ACTIVE			0xE4
#define LM8333_READ_ERROR		0xF0

/* bit numbers for LM8333_READ_INT */
#define LM8333_KEYPAD_IRQ		0
#define LM8333_EX_0_IRQ			1
#define LM8333_EX_1_IRQ			2
#define LM8333_ERROR_IRQ		3

#define LM8333_NUM_IRQS			4

#define LM8333_ERROR_CMDOVR	1
#define LM8333_ERROR_CMDUNK	2
#define LM8333_ERROR_KEYOVR	4
#define LM8333_ERROR_NOINT	0x10
#define LM8333_ERROR_FIFOOVR	0x40

/* event as returned by LM8333 */
#define LM8333_KEY(event, val)	((event << 24) | (val))

struct lm8333_i2c_platform_data {
	/* Active timeout before enter HALT mode in microseconds */
	unsigned int	active_time;
};

struct lm8333_keypad_platform_data {
	/* array of  LM8333_KEY*/
	unsigned int	*key_map;
	/* number of matrix_key_map entries */
	unsigned int	key_map_size;
	/* Debounce interval in microseconds */
	unsigned int	debounce_time;
};

struct lm8333_gpio_platform_data {
	/* number of the first GPIO */
	unsigned int	gpio_base;
};

struct lm8333_platform_data {
	/* i2c adapter index */
	int i2c_adapter_id;
	/* IRQ of the chip */
	unsigned int	irq;
	/* i2c platform data */
	struct lm8333_i2c_platform_data *lm8333_i2c_pdata;
};

struct lm8333_plat_data {
	struct i2c_client *client;
};

extern int lm8333_remove_irq_work(int irq);
extern int lm8333_add_irq_work(int irq,
		void (*handler)(void *), void *data);
extern int lm8333_read_data(int reg, int length, u8 *data);
extern int lm8333_write_data(int reg, int length, u8 *data);
extern int lm8333_read_reg(int reg);
extern int lm8333_write_reg(int reg, u8 value);

#endif /* _LM8333_H */

