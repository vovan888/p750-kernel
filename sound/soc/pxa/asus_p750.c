/*
 * asus_p750.c  --  SoC audio for Asus PDAs
 *
 * Vladimir Ananiev <vovan888@gmail.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation; version 2 ONLY.
 *
 *  Revision history
 *	2007-07-19   Vitaly Bursov           Based on tosa.c
 *	2008-12-20   Vladimir Ananiev	     Generic version for Asus P750 PDA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>

#include <sound/ac97_codec.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/delay.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#include <mach/board-p750.h>
#include <mach/pxa-regs.h>
#include <mach/hardware.h>
#include <mach/audio.h>
#include <mach/gpio.h>

#include <sound/ac97_codec.h>
#include <linux/wm97xx.h>

#include "../codecs/wm9713.h"
#include "pxa2xx-pcm.h"
#include "pxa2xx-ac97.h"

static struct snd_soc_machine asus;


extern struct snd_soc_platform pxa2xx_soc_platform;
extern struct snd_soc_codec_device soc_codec_dev_wm9713;
//extern struct snd_soc_codec_dai wm9713_dai;

static DECLARE_MUTEX(jack_mutex);
static DECLARE_MUTEX(jack_codec);
static DECLARE_MUTEX(ts_mutex);
static struct workqueue_struct *phonejack_workqueue;
static struct work_struct phonejack_task;


static struct snd_soc_codec *wq_codec;

#define P750_JACK_SPEAKER 0
#define P750_JACK_HEADPHONE 1

static int asus_jack_func;
static int is_codec_active;

int asus_ac97_dependency;
EXPORT_SYMBOL(asus_ac97_dependency);

static void wm97xx_gpio_func(struct snd_soc_codec *codec, int gpio, int func)
{
	int GEn;
	GEn = soc_ac97_ops.read(codec->ac97, AC97_MISC_AFE);
	if (func) GEn |= gpio;
	else GEn &= ~gpio;
	soc_ac97_ops.write(codec->ac97, AC97_MISC_AFE, GEn);
}
	
static void wm97xx_gpio_mode(struct snd_soc_codec *codec, int gpio, int config, int polarity, int sticky, int wakeup)
{
	int GCn, GPn, GSn, GWn;

	GCn = soc_ac97_ops.read(codec->ac97, AC97_GPIO_CFG);
	GPn = soc_ac97_ops.read(codec->ac97, AC97_GPIO_POLARITY);
	GSn = soc_ac97_ops.read(codec->ac97, AC97_GPIO_STICKY);
	GWn = soc_ac97_ops.read(codec->ac97, AC97_GPIO_WAKEUP);
	
	if (config) GCn |= gpio;
	else GCn &= ~gpio;
	
	if (polarity) GPn |= gpio;
	else GPn &= ~gpio;
	
	if (sticky) GSn |= gpio;
	else GSn &= ~gpio;
	
	if (wakeup) GWn |= gpio;
	else GWn &= ~gpio;
	
	soc_ac97_ops.write(codec->ac97, AC97_GPIO_CFG, GCn);
	soc_ac97_ops.write(codec->ac97, AC97_GPIO_POLARITY, GPn);
	soc_ac97_ops.write(codec->ac97, AC97_GPIO_STICKY, GSn);
	soc_ac97_ops.write(codec->ac97, AC97_GPIO_WAKEUP, GWn);
}


static void asus_ext_control(struct snd_soc_codec *codec)
{
	/* set up jack connection */
	switch (asus_jack_func) {
	case P750_JACK_SPEAKER:
                snd_soc_dapm_enable_pin(codec, "Speaker");
                snd_soc_dapm_disable_pin(codec, "Headphone Jack");
		break;
	case P750_JACK_HEADPHONE:
                snd_soc_dapm_enable_pin(codec, "Headphone Jack");
                snd_soc_dapm_disable_pin(codec, "Speaker");
		break;
	}

	printk (KERN_ERR "%s: activ %d\n", __FUNCTION__, asus_jack_func);

	snd_soc_dapm_sync(codec);
}

static int asus_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->codec;

	down(&jack_codec);

	is_codec_active = 1;
	asus_ext_control(codec);

	up(&jack_codec);

	return 0;
}

static void asus_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->codec;

	down(&jack_codec);

	is_codec_active = 0;

	up(&jack_codec);
}


static struct snd_soc_ops asus_ops = {
	.startup = asus_startup,
	.shutdown = asus_shutdown,
};


/***********************************************************/
/**/
/***********************************************************/
static void phonejack_handler(struct work_struct *p)
{
	struct snd_soc_codec *codec = wq_codec;
	u16 status = 0, polarity = 0;

	if (codec == NULL)
	    return;
		
	status = soc_ac97_ops.read(codec->ac97, AC97_GPIO_STATUS);
	polarity = soc_ac97_ops.read(codec->ac97, AC97_GPIO_POLARITY);

	if (status & WM97XX_GPIO_1)
	{
		if (polarity & WM97XX_GPIO_1)
		{
			polarity &= ~WM97XX_GPIO_1;
			asus_jack_func = P750_JACK_HEADPHONE;
		}
		else
		{
			polarity |= WM97XX_GPIO_1;
			asus_jack_func = P750_JACK_SPEAKER;
		}
		
		/* switch audio output */
		down(&jack_codec);
		if (is_codec_active) {
			asus_ext_control(codec);
		}
		up(&jack_codec);
                status &= ~WM97XX_GPIO_1;

	    mdelay(100);
	
	    soc_ac97_ops.write(codec->ac97, AC97_GPIO_POLARITY, polarity);
	    soc_ac97_ops.write(codec->ac97, AC97_GPIO_STATUS, status);
	}

	up(&jack_mutex);
}

static irqreturn_t jack_irq_handler(int irq, void *p)
{
	if (!down_trylock(&jack_mutex))
	{
		queue_work(phonejack_workqueue, &phonejack_task);
	}
	return IRQ_HANDLED;
}


/***********************************************************/
/**/
/***********************************************************/
static const struct snd_soc_dapm_route audio_map[] = {

	/* headphone connected to HPOUTL, HPOUTR */
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Headphone Jack", NULL, "HPOUTR"},

	/* speaker connected to LOUT2, ROUT2 */
	{"Speaker", NULL, "LOUT2"},
	{"Speaker", NULL, "ROUT2"},

	/* internal mic is connected to mic1, mic2 differential - with bias */
//	{"MIC1", NULL, "Mic Bias"},
//	{"MIC2", NULL, "Mic Bias"},
//	{"Mic Bias", NULL, "Mic (Internal)"},

	{NULL, NULL, NULL},
};


/* tosa dapm event handlers */
static int asus_spk_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
/*	if (SND_SOC_DAPM_EVENT_ON(event))
		gpio_set_value(EGPIO_P750_WM9713_PWREN, 1);
	else
		gpio_set_value(EGPIO_P750_WM9713_PWREN, 0);*/
	return 0;
}

/* tosa machine dapm widgets */
static const struct snd_soc_dapm_widget asus_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Speaker", asus_spk_event),
};


static int asus_ac97_init(struct snd_soc_codec *codec)
{
	snd_soc_dapm_nc_pin(codec, "OUT3");
	snd_soc_dapm_nc_pin(codec, "MONOOUT");

	/* add asus specific widgets */
        snd_soc_dapm_new_controls(codec, asus_dapm_widgets,
                                  ARRAY_SIZE(asus_dapm_widgets));

        /* set up asus specific audio path audio_map */
        snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	wq_codec = codec;

//        soc_ac97_ops.write(codec->ac97, AC97_ADD_FUNC, 0);
        soc_ac97_ops.write(codec->ac97, AC97_GPIO_STICKY, 0);

	// jack sense connected here
	wm97xx_gpio_mode(codec, WM97XX_GPIO_1, WM97XX_GPIO_IN, WM97XX_GPIO_POL_HIGH, WM97XX_GPIO_STICKY, WM97XX_GPIO_WAKE);
	wm97xx_gpio_func(codec, WM97XX_GPIO_1, 0);
	// irq pin
	wm97xx_gpio_mode(codec, WM97XX_GPIO_2, WM97XX_GPIO_OUT, WM97XX_GPIO_POL_HIGH, WM97XX_GPIO_NOTSTICKY, WM97XX_GPIO_NOWAKE);
	wm97xx_gpio_func(codec, WM97XX_GPIO_2, 0);
	// pendown
	wm97xx_gpio_mode(codec, WM97XX_GPIO_3, WM97XX_GPIO_OUT, WM97XX_GPIO_POL_HIGH, WM97XX_GPIO_NOTSTICKY, WM97XX_GPIO_NOWAKE);
	wm97xx_gpio_func(codec, WM97XX_GPIO_3, 0);

	// gpios. no use?
	wm97xx_gpio_mode(codec, WM97XX_GPIO_4, WM97XX_GPIO_IN, WM97XX_GPIO_POL_HIGH, WM97XX_GPIO_NOTSTICKY, WM97XX_GPIO_NOWAKE);
	wm97xx_gpio_func(codec, WM97XX_GPIO_4, 1);
	wm97xx_gpio_mode(codec, WM97XX_GPIO_5, WM97XX_GPIO_IN, WM97XX_GPIO_POL_HIGH, WM97XX_GPIO_STICKY, WM97XX_GPIO_NOWAKE);
	wm97xx_gpio_func(codec, WM97XX_GPIO_5, 1);

	snd_soc_dapm_sync(codec);
	return 0;
}


static struct snd_soc_dai_link asus_dai[] = {
{
	.name = "AC97",
	.stream_name = "AC97 HiFi",
	.cpu_dai = &pxa_ac97_dai[PXA2XX_DAI_AC97_HIFI],
	.codec_dai = &wm9713_dai[WM9713_DAI_AC97_HIFI],
	.init = asus_ac97_init,
	.ops = &asus_ops,
},
{
	.name = "AC97 Aux",
	.stream_name = "AC97 Aux",
	.cpu_dai = &pxa_ac97_dai[PXA2XX_DAI_AC97_AUX],
	.codec_dai = &wm9713_dai[WM9713_DAI_AC97_AUX],
},
};


static struct snd_soc_machine asus = {
	.name = "Asus P750",
	.dai_link = asus_dai,
	.num_links = ARRAY_SIZE(asus_dai),

};

static struct snd_soc_device asus_snd_devdata = {
	.machine = &asus,
	.platform = &pxa2xx_soc_platform,
	.codec_dev = &soc_codec_dev_wm9713,
};

static struct platform_device *asus_snd_device;


static int __init asus_init(void)
{
	int ret, err;
	int irqflag = 0;
#ifdef CONFIG_PREEMPT_RT
	irqflag |= IRQF_NODELAY;
#endif

	if (!machine_is_asusp750())
		return -ENODEV;

	ret = gpio_request(GPIO14_P750_HEADPHONE_IN, "Headphone Jack");
	if (ret)
		return ret;
	gpio_direction_input(GPIO14_P750_HEADPHONE_IN);

	asus_snd_device = platform_device_alloc("soc-audio", -1);
	if (!asus_snd_device)
		return -ENOMEM;

	phonejack_workqueue = create_singlethread_workqueue("phonejackd");
	INIT_WORK(&phonejack_task, phonejack_handler);

	platform_set_drvdata(asus_snd_device, &asus_snd_devdata);
	asus_snd_devdata.dev = &asus_snd_device->dev;
	ret = platform_device_add(asus_snd_device);

	if (ret)
		platform_device_put(asus_snd_device);


	set_irq_type(gpio_to_irq(GPIO14_P750_HEADPHONE_IN), IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
	err = request_irq(gpio_to_irq(GPIO14_P750_HEADPHONE_IN), jack_irq_handler, irqflag, "wm97xx-jack", 0);
	if (err)
	{
		printk(KERN_ERR "%s: Cannot assign GPIO(%d) IRQ\n", __FUNCTION__, IRQ_GPIO(GPIO14_P750_HEADPHONE_IN));
	}

	return ret;
}

static void __exit asus_exit(void)
{
	wq_codec = NULL;

	free_irq(gpio_to_irq(GPIO14_P750_HEADPHONE_IN), NULL);

	destroy_workqueue(phonejack_workqueue);

	platform_device_unregister(asus_snd_device);
}

module_init(asus_init);
module_exit(asus_exit);

/* Module information */
MODULE_AUTHOR("Vladimir Ananiev <vovan888@gmail.com>");
MODULE_DESCRIPTION("ALSA SoC driver for Asus P750");
MODULE_LICENSE("GPL");
