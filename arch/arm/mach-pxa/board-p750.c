/*
 * Machine definition for Asus P750.
 *
 * Copyright (C) 2009 Vladimir Ananiev <vovan888@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/gpio_keys.h>
#include <linux/pwm_backlight.h>
#include <linux/rtc.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jbt6k74.h>
#include <linux/pda_power.h>
#include <linux/power_supply.h>
#include <linux/wm97xx.h>
#include <linux/mtd/physmap.h>
#include <linux/mfd/htc-egpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/audio.h>
#include <mach/i2c.h>
#include <mach/mfp-pxa27x.h>
#include <mach/pxa27x_keypad.h>
#include <mach/pxa2xx_spi.h>
#include <mach/pxafb.h>
#include <mach/pxa2xx-regs.h>
#include <mach/mmc.h>
#include <mach/udc.h>
#include <mach/pxa27x-udc.h>
#include <mach/ohci.h>

#include <mach/board-p750.h>

#include "generic.h"
#include "devices.h"


static unsigned long p750_pin_config[] = {
	/* Global */
	GPIO89_USBH1_PEN,
	GPIO79_nCS_3,

	/* Backlight PWM 0 */
	GPIO16_PWM0_OUT,

	/* MMC */
	GPIO32_MMC_CLK,		/*MMCLK*/
	GPIO92_MMC_DAT_0,
	GPIO109_MMC_DAT_1,
	GPIO110_MMC_DAT_2,
	GPIO111_MMC_DAT_3,
	GPIO112_MMC_CMD,

	/* CF */
	GPIO48_nPOE,
	GPIO49_nPWE,
	GPIO50_nPIOR,
	GPIO51_nPIOW,
	GPIO55_nPREG,
	GPIO56_nPWAIT,
	GPIO57_nIOIS16,

	/* LCD */
	GPIO58_LCD_LDD_0,
	GPIO59_LCD_LDD_1,
	GPIO60_LCD_LDD_2,
	GPIO61_LCD_LDD_3,
	GPIO62_LCD_LDD_4,
	GPIO63_LCD_LDD_5,
	GPIO64_LCD_LDD_6,
	GPIO65_LCD_LDD_7,
	GPIO66_LCD_LDD_8,
	GPIO67_LCD_LDD_9,
	GPIO68_LCD_LDD_10,
	GPIO69_LCD_LDD_11,
	GPIO70_LCD_LDD_12,
	GPIO71_LCD_LDD_13,
	GPIO72_LCD_LDD_14,
	GPIO73_LCD_LDD_15,
	GPIO74_LCD_FCLK,
	GPIO75_LCD_LCLK,
	GPIO76_LCD_PCLK,
	GPIO77_LCD_BIAS,

	/* STUART - Battery */
	GPIO46_STUART_RXD,
	GPIO47_STUART_TXD,

	/* FFUART - GSM */
	GPIO34_FFUART_RXD,
	GPIO35_FFUART_CTS,
	GPIO39_FFUART_TXD,
	GPIO41_FFUART_RTS,

	/* AC97 */
	GPIO28_AC97_BITCLK,
	GPIO29_AC97_SDATA_IN_0,
	GPIO30_AC97_SDATA_OUT,
	GPIO31_AC97_SYNC,
	GPIO45_AC97_SYSCLK,

	/* SSP 2 */
	GPIO36_SSP2_SCLK,
	GPIO37_SSP2_SFRM,
	GPIO38_SSP2_TXD,
	GPIO40_SSP2_RXD,
	
	/* SSP 3 */
	GPIO81_SSP3_TXD,
	GPIO82_SSP3_RXD,
	GPIO83_SSP3_SFRM,
	GPIO84_SSP3_SCLK,
	
	/* CIF */
	GPIO23_CIF_MCLK,
	GPIO24_CIF_FV,
	GPIO25_CIF_LV,
	GPIO26_CIF_PCLK,
	GPIO27_CIF_DD_0,
	GPIO52_CIF_DD_4,
	GPIO93_CIF_DD_6,
	GPIO94_CIF_DD_5,
	GPIO108_CIF_DD_7,
	GPIO114_CIF_DD_1,
	GPIO115_CIF_DD_3,
	GPIO116_CIF_DD_2,
	
};

/* GPIO helper macros and functions */
/******************************************************************************/
#define P750_GPIO_IN(num, _desc) \
	{ .gpio = (num), .dir = 0, .desc = (_desc) }
#define P750_GPIO_OUT(num, _init, _desc) \
	{ .gpio = (num), .dir = 1, .init = (_init), .desc = (_desc) }
struct gpio_ress {
	unsigned gpio : 8;
	unsigned dir : 1;
	unsigned init : 1;
	char *desc;
};

static int p750_gpio_request(struct gpio_ress *gpios, int size)
{
	int i, rc = 0;
	int gpio;
	int dir;

	for (i = 0; (!rc) && (i < size); i++) {
		gpio = gpios[i].gpio;
		dir = gpios[i].dir;
		rc = gpio_request(gpio, gpios[i].desc);
		if (rc) {
			printk(KERN_ERR "Error requesting GPIO %d(%s) : %d\n",
			       gpio, gpios[i].desc, rc);
			continue;
		}
		if (dir)
			gpio_direction_output(gpio, gpios[i].init);
		else
			gpio_direction_input(gpio);
	}
	while ((rc) && (--i >= 0))
		gpio_free(gpios[i].gpio);
	return rc;
}

static void p750_gpio_free(struct gpio_ress *gpios, int size)
{
	int i;

	for (i = 0; i < size; i++)
		gpio_free(gpios[i].gpio);
}

/*
 * BCR (LC4128 CPLD) support using HTC-EGPIO driver
 *
 * 7 32-bit aligned 8-bit registers: all outputs
 */
/******************************************************************************/
static struct resource egpio_resources[] = {
	[0] = {
		.start = PXA_CS3_PHYS,
		.end   = PXA_CS3_PHYS + 0x20,
		.flags = IORESOURCE_MEM,
	},
};

static struct htc_egpio_chip egpio_chips[] = {
	[0] = {
		.reg_start = 0,
		.gpio_base = P750_EGPIO(0, 0),
		.num_gpios = 32,
		.direction = HTC_EGPIO_OUTPUT,
		.initial_values = 0x9845fb18,
	},
	[1] = {
		.reg_start = 4,
		.gpio_base = P750_EGPIO(4, 0),
		.num_gpios = 24,
		.direction = HTC_EGPIO_OUTPUT,
		.initial_values = 0x0920,
	},
};

static struct htc_egpio_platform_data egpio_info = {
	.reg_width    = 8,
	.bus_width    = 32,
	.num_irqs     = 0,
	.chip         = egpio_chips,
	.num_chips    = ARRAY_SIZE(egpio_chips),
};

static struct platform_device egpio = {
	.name          = "htc-egpio",
	.id            = -1,
	.resource      = egpio_resources,
	.num_resources = ARRAY_SIZE(egpio_resources),
	.dev = {
		.platform_data = &egpio_info,
	},
};

/******************************************************************************/
struct pxafb_mode_info p750_fb_modes[] = {
	{
		/* Configuration for 480x640 toppoly TD026TTEA1 */
		.pixclock	= 160000, /* */
		.xres		= 480,
		.yres		= 640,
		.bpp		= 16,
		.left_margin    = 1,
		.right_margin   = 2,
		.hsync_len      = 42,
		.upper_margin   = 93,
		.lower_margin   = 2,
		.vsync_len      = 2,
		.sync		= 0,
		.cmap_greyscale	= 0,
	},
	{
		/* Config for 240x320 LCD */
		.pixclock	= 640000, /**/
		.xres		= 240,
		.yres		= 320,
		.bpp		= 16,
		.left_margin	= 13,
		.right_margin	= 8,
		.hsync_len	= 4,
		.upper_margin	= 2,
		.lower_margin	= 7,
		.vsync_len	= 4,
		.sync		= 0,
		.cmap_greyscale	= 0,
	},
};

static void asusp750_lcd_power(int on, struct fb_var_screeninfo *si)
{
	pr_debug("Toppoly LCD power\n");
	return;
	if (on) {
		pr_debug("on\n");
		gpio_set_value(EGPIO_P750_LCD_RESET_N, 0);
		msleep(10);
		gpio_set_value(EGPIO_P750_LCD_PWREN, 1);
		msleep(10);
		gpio_set_value(EGPIO_P750_LCD_RESET_N, 1);
		msleep(150);
	} else {
		pr_debug("off\n");
		gpio_set_value(EGPIO_P750_LCD_RESET_N, 1);
		msleep(2);
		gpio_set_value(EGPIO_P750_LCD_RESET_N, 0);
		msleep(2);
		gpio_set_value(EGPIO_P750_LCD_PWREN, 0);
	}
}

static struct pxafb_mach_info p750_fb_info  = {
	.modes = p750_fb_modes,
	.num_modes = ARRAY_SIZE(p750_fb_modes),
	.fixed_modes = 1,
	.lccr0          = LCCR0_Color | LCCR0_Sngl | LCCR0_Act,
	.lccr3          = LCCR3_PixRsEdg,
	.lcd_conn		= LCD_COLOR_TFT_16BPP | LCD_PCLK_EDGE_FALL,
	.pxafb_lcd_power	= asusp750_lcd_power,
//	.pxafb_backlight_power	= board_backlight_power,
};

/* LCD Backlight */
/******************************************************************************/
static struct platform_pwm_backlight_data p750_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 100,
	.dft_brightness	= 50,
	.pwm_period_ns	= 4000 * 1024,	/* Fl = 250kHz */
};

/* GPIO Key Configuration */
/******************************************************************************/
static struct gpio_keys_button p750_button_table[] = {
//	{KEY_POWER,	GPIO00_P750_KEY_POWER,		0, "Power button",	EV_KEY, 1}, /* wakeup source */
	{KEY_ESC,	GPIO102_P750_BUTTON_RED,	0, "Hangup button",	EV_KEY},
	{KEY_CAMERA,	GPIO101_P750_BUTTON_CAMERA_FULL,0, "Camera button",	EV_KEY},
	{KEY_F1,	GPIO100_P750_BUTTON_OK,		0, "OK button",		EV_KEY},
	{KEY_MENU,	GPIO99_P750_BUTTON_MODE,	0, "Mode  button",	EV_KEY},
	{KEY_RECORD,	GPIO98_P750_BUTTON_RECORD,	0, "Record button",	EV_KEY},
	{KEY_PHONE,	GPIO97_P750_BUTTON_GREEN,	0, "Green button",	EV_KEY},
	{KEY_F2,	GPIO91_P750_BUTTON_HOLD,	0, "Hold button",	EV_KEY},
	{KEY_CAMERA-1,	GPIO86_P750_BUTTON_CAMERA_HALF,	0, "Camera half press button",	EV_KEY},
//	{KEY_PLAY,	GPIO_P750_BUTTON_HEADPHONE,	0, "Headset button",	EV_KEY},
	{KEY_UP,	GPIO103_P750_BUTTON_JOY_UP,	0, "Up button",		EV_KEY},
	{KEY_DOWN,	GPIO105_P750_BUTTON_JOY_DOWN,	0, "Down button",	EV_KEY},
	{KEY_LEFT,	GPIO106_P750_BUTTON_JOY_LEFT,	0, "Left button",	EV_KEY},
	{KEY_RIGHT,	GPIO104_P750_BUTTON_JOY_RIGHT,	0, "Right button",	EV_KEY},
	{KEY_KPENTER,	GPIO107_P750_BUTTON_JOY_PUSH,	0, "Action button",	EV_KEY},
	{SW_HEADPHONE_INSERT, GPIO14_P750_HEADPHONE_IN,	0, "Headset insert button",EV_SW},
};

static struct gpio_keys_platform_data p750_gpio_keys_data = {
	.buttons  = p750_button_table,
	.nbuttons = ARRAY_SIZE(p750_button_table),
};

/* Leds and vibrator */
/******************************************************************************/
#define ONE_LED(_gpio, _name) \
{ .gpio = (_gpio), .name = (_name), .active_low = false }
static struct gpio_led gpio_leds[] = {
	ONE_LED(EGPIO_P750_LED_GREEN,	"green"),
	ONE_LED(EGPIO_P750_LED_RED,	"red"),
	ONE_LED(EGPIO_P750_LED_BLUE,	"blue"),
	ONE_LED(EGPIO_P750_VIBRA_PWREN,	"vibrator"),
	ONE_LED(EGPIO_P750_KEYPAD_LED_PWREN,"keyboard-backlight")
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds = gpio_leds,
	.num_leds = ARRAY_SIZE(gpio_leds),
};

/*  MMC Card controller */
/******************************************************************************/
static void mci_setpower(struct device *dev, unsigned int vdd)
{
	struct pxamci_platform_data *pdata = dev->platform_data;

	gpio_set_value(EGPIO_P750_SDCARD_PWREN, (1 << vdd) & pdata->ocr_mask);
}

static int mci_get_ro(struct device *dev)
{
	return 0;
}

struct gpio_ress mci_gpios[] = {
	P750_GPIO_IN(GPIO10_P750_SD_DETECT_N,	"SDIO insertion detect"),
};

static void mci_exit(struct device *dev, void *data)
{
	p750_gpio_free(ARRAY_AND_SIZE(mci_gpios));
	free_irq(gpio_to_irq(GPIO10_P750_SD_DETECT_N), data);
}

static struct pxamci_platform_data p750_mci_info;

/**
 * The card detect interrupt isn't debounced so we delay it by 250ms
 * to give the card a chance to fully insert/eject.
 */
static int mci_init(struct device *dev, irq_handler_t detect_int, void *data)
{
	int rc;
	int irq = gpio_to_irq(GPIO10_P750_SD_DETECT_N);

	rc = p750_gpio_request(ARRAY_AND_SIZE(mci_gpios));
	if (rc)
		goto err_gpio;
	/* enable RE/FE interrupt on card insertion and removal */
	rc = request_irq(irq, detect_int,
			 IRQF_DISABLED | IRQF_TRIGGER_RISING |
			 IRQF_TRIGGER_FALLING,
			 "MMC card detect", data);
	if (rc)
		goto err_irq;

	p750_mci_info.detect_delay = msecs_to_jiffies(250);
	return 0;

err_irq:
	dev_err(dev, "p750_mci_init: MMC/SD:"
		" can't request MMC card detect IRQ\n");
	p750_gpio_free(ARRAY_AND_SIZE(mci_gpios));
err_gpio:
	return rc;
}

static struct pxamci_platform_data p750_mci_info = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	.init	  = mci_init,
	.get_ro	  = mci_get_ro,
	.setpower = mci_setpower,
	.exit	  = mci_exit,
};

/* USB UDC */
/******************************************************************************/
static int is_usb_cable_connected(void)
{
	return gpio_get_value(GPIO12_P750_USB_CABLE_DETECT);
}

static int is_ac_adaptor_connected(void)
{
	return gpio_get_value(GPIO13_P750_AC_ADAPTER_DETECT);
}

static void udc_power_command(int cmd)
{
	switch (cmd) {
	case PXA2XX_UDC_CMD_DISCONNECT:
		gpio_set_value(EGPIO_P750_USB_PULL_DPLUS, 0);
		break;
	case PXA2XX_UDC_CMD_CONNECT:
		gpio_set_value(EGPIO_P750_USB_PULL_DPLUS, 1);
		break;
	default:
		printk(KERN_INFO "udc_control: unknown command (0x%x)!\n", cmd);
		break;
	}
}

static struct pxa2xx_udc_mach_info p750_udc_info = {
	.udc_is_connected = is_usb_cable_connected,
	.udc_command	  = udc_power_command,
/*	.gpio_vbus		= GPIO12_P750_USB_CABLE_DETECT,
	.gpio_vbus_inverted	= 0,
	.gpio_pullup		= EGPIO_P750_USB_PULL_DPLUS,
	.gpio_pullup_inverted	= 0,*/
};

/* Power Supply */
/******************************************************************************/
static char *p750_supplicants[] = {
	"ds2780-battery.0",
};

static struct resource power_resources[] = {
	[0] = {
		.name	= "usb",
		.start	= gpio_to_irq(GPIO12_P750_USB_CABLE_DETECT),
		.end	= gpio_to_irq(GPIO12_P750_USB_CABLE_DETECT),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE |
		IORESOURCE_IRQ_LOWEDGE,
	},
	[1] = {
		.name	= "ac",
		.start	= gpio_to_irq(GPIO13_P750_AC_ADAPTER_DETECT),
		.end	= gpio_to_irq(GPIO13_P750_AC_ADAPTER_DETECT),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE |
		IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct pda_power_pdata power_supply_info = {
	.is_ac_online    = is_ac_adaptor_connected,
	.is_usb_online   = is_usb_cable_connected,
	.supplied_to     = p750_supplicants,
	.num_supplicants = ARRAY_SIZE(p750_supplicants),
//	.set_charge = ,
};

static struct platform_device power_dev = {
	.name = "pda-power",
	.id   = -1,
	.resource	= power_resources,
	.num_resources	= ARRAY_SIZE(power_resources),
	.dev  = {
		.platform_data = &power_supply_info,
	},
};

/* AC97 */
/******************************************************************************/
static int p750_audio_startup(struct snd_pcm_substream *substream, void *priv)
{
	gpio_set_value(EGPIO_P750_WM9713_PWREN, 1);
	return 0;
}

static void p750_audio_shutdown(struct snd_pcm_substream *substream, void *priv)
{
	gpio_set_value(EGPIO_P750_WM9713_PWREN, 0);
}

static void p750_audio_suspend(void *priv)
{
	gpio_set_value(EGPIO_P750_WM9713_PWREN, 0);
}

static void p750_audio_resume(void *priv)
{
	gpio_set_value(EGPIO_P750_WM9713_PWREN, 1);
}

static pxa2xx_audio_ops_t p750_audio_ops = {
	.startup	= p750_audio_startup,
	.shutdown	= p750_audio_shutdown,
	.suspend	= p750_audio_suspend,
	.resume		= p750_audio_resume,
};

/* SPI2 - LCD */
/******************************************************************************/
static void p750_jbt6k74_reset(int devidx, int level)
{
	/* empty place holder; p750 does not yet use this */
	printk(KERN_DEBUG "p750_jbt6k74_reset\n");
}

static void p750_jbt6k74_resuming(int devidx)
{
	printk(KERN_DEBUG "p750bl_deferred_resume\n");
	//p750bl_deferred_resume();
}

const struct jbt6k74_platform_data p750_jbt6k74_pdata = {
	.reset		= p750_jbt6k74_reset,
	.resuming	= p750_jbt6k74_resuming,
};

static struct pxa2xx_spi_master pxa_spi2_master_info = {
	.num_chipselect	= 1,
	.enable_dma	= 0,
};

/* SPI3 - GSM */
/******************************************************************************/
static void p750_gsm_cs(u32 command)
{
	gpio_set_value(GPIO20_P750_SPI3_ENABLE, (command == PXA2XX_CS_ASSERT));
}

static struct pxa2xx_spi_chip p750_gsmserial_chip = {
	.cs_control	= p750_gsm_cs,
};

static struct pxa2xx_spi_master pxa_spi3_master_info = {
	.num_chipselect	= 1,
	.enable_dma	= 0,
};

static struct spi_board_info p750_spi_board_info[] = {
	{
		.modalias	= "jbt6k74",
		.platform_data	= &p750_jbt6k74_pdata,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 2,
	}, {
		.modalias	= "spidev",
//		.platform_data	= &p750_gsmserial_pdata,
		.controller_data= &p750_gsmserial_chip,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 3,
		.chip_select	= 0,
	},
};

/* Devices */
/******************************************************************************/
#define P750_PARENT_DEV(var, strname, tparent, pdata)	\
static struct platform_device var = {			\
	.name		= strname,			\
	.id		= -1,				\
	.dev		= {				\
		.platform_data = pdata,			\
		.parent	= tparent,			\
	},						\
};
#define P750_SIMPLE_DEV(var, strname, pdata)	\
	P750_PARENT_DEV(var, strname, NULL, pdata)

P750_SIMPLE_DEV(p750_gpio_keys, "gpio-keys",	    &p750_gpio_keys_data)
P750_PARENT_DEV(p750_backlight, "pwm-backlight",  &pxa27x_device_pwm0.dev,
		&p750_backlight_data);
P750_SIMPLE_DEV(p750_led,	  "leds-gpio",	    &gpio_led_info)
P750_SIMPLE_DEV(p750_board,	  "p750-board",  NULL)


static struct platform_device *p750_devices[] __initdata = {
	&egpio,
	&p750_gpio_keys,
	&p750_backlight,
	&p750_led,
	&power_dev,
	&p750_board,
};

/* USB OHCI */
/******************************************************************************/
static struct pxaohci_platform_data p750_ohci_info = {
	.port_mode	= PMM_PERPORT_MODE,
	.flags		= ENABLE_PORT1 | ENABLE_PORT3 | POWER_CONTROL_LOW,
	.power_budget	= 0,
};

struct gpio_ress global_gpios[] = {
	P750_GPIO_IN(GPIO12_P750_USB_CABLE_DETECT, "USB detect"),
	P750_GPIO_IN(GPIO13_P750_AC_ADAPTER_DETECT, "AC detect"),
	P750_GPIO_IN(GPIO18_P750_MODEM_RDY, "Modem ready"),
	P750_GPIO_OUT(GPIO20_P750_SPI3_ENABLE, 0, "SPI3 enable"),
	P750_GPIO_OUT(GPIO21_P750_BCR_MODEM_ABUSY, 0, "Modem abusy"),
	P750_GPIO_IN(GPIO40_P750_KEYPAD_IRQ, "Keypad IRQ"),
	P750_GPIO_OUT(GPIO78_P750_MODEM_BB_PWR_EN, 0, "Modem bb power en"),
	P750_GPIO_IN(GPIO113_P750_TOUCHSCREEN_IRQ, "Touch IRQ"),
};

/******************************************************************************/
static int p750_hw_init(void)
{
/*	int i;

	GCR &= ~(GCR_PRIRDY_IEN | GCR_SECRDY_IEN);
	GCR |= GCR_ACLINK_OFF;
        PCFR = PCFR_OPDE | PCFR_GPR_EN | PCFR_PI2CEN | PCFR_DC_EN;
*/
	/* Reset DMA registers
	   Useful when kernel is started with Haret/Loadlin8 */
/*	DINT = 0;

	for (i=0;i<32;i++){
		DCSR(i) = 0;
		DCMD(i) = 0;
	}

	for (i=0;i<64;i++){
		DRCMR(i) = 0;
	}
*/
	return 0;
}
/******************************************************************************/
static void __init p750_init (void)
{
	p750_hw_init();
	pxa2xx_mfp_config(ARRAY_AND_SIZE(p750_pin_config));
	p750_gpio_request(ARRAY_AND_SIZE(global_gpios));

	pxa2xx_set_spi_info(2, &pxa_spi2_master_info);
	pxa2xx_set_spi_info(3, &pxa_spi3_master_info);
	spi_register_board_info(p750_spi_board_info, ARRAY_SIZE(p750_spi_board_info));

	set_pxa_fb_info(&p750_fb_info);
	pxa_set_mci_info(&p750_mci_info);
	pxa_set_i2c_info(NULL);
	pxa_set_ac97_info(&p750_audio_ops);
	pxa_set_udc_info(&p750_udc_info);
	pxa_set_ohci_info(&p750_ohci_info);
//	pm_power_off = p750_poweroff;
//	arm_pm_restart = p750_restart;
	platform_add_devices(p750_devices, ARRAY_SIZE(p750_devices));
//	gsm_init();
}
/***********************************************************/
MACHINE_START(ASUSP750, "Asus P750")
	.phys_io	= 0x40000000,
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.boot_params	= 0xa0000100,
	.map_io		= pxa_map_io,
	.init_irq	= pxa27x_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= p750_init,
MACHINE_END
