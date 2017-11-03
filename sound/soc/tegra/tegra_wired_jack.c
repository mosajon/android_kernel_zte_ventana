/*
 * sound/soc/tegra/tegra_wired_jack.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/types.h>
#include <linux/gpio.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif
#include <linux/notifier.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <mach/audio.h>

#include "tegra_soc.h"
#include "../codecs/max98095.h"

#define HEAD_DET_GPIO 0

extern struct snd_soc_codec *max98095_codec;

struct wired_jack_conf tegra_wired_jack_conf = {
	-1, -1, -1, -1, 0, NULL, NULL
};

enum headset_state {
	NO_HEADSET = 0,
	HEADSET_MIC =  (1 << 0),
	HEADSET_WITHOUT_MIC = (1 << 1),
};

static struct snd_soc_jack *tegra_wired_jack;

static struct snd_soc_jack_gpio wired_jack_gpios[] = {
	{
		/* gpio pin depends on board traits */
		.name = "headphone-detect-gpio",
		.report = SND_JACK_HEADPHONE,
		.invert = 1,
		.debounce_time = 200,
	},
};

#ifdef CONFIG_SWITCH
static struct switch_dev wired_switch_dev = {
	.name = "h2w",
};

void tegra_switch_set_state(int state)
{
	switch_set_state(&wired_switch_dev, state);
}

static enum headset_state get_headset_state(void)
{
	enum headset_state state = NO_HEADSET;
	int hp_gpio = -1;
	int i;
	volatile int reg_val = 0;

	/* hp_det_n is low active pin */
	if (tegra_wired_jack_conf.hp_det_n != -1){
		hp_gpio = gpio_get_value(tegra_wired_jack_conf.hp_det_n);
		if(hp_gpio == 1){
			state = NO_HEADSET;
			snd_soc_write(max98095_codec, M98095_013_JACK_INT_EN,
			0x00);
			printk("NO_HEADSET\n");
			return state;
		}
	}

	// reset the detect circuit 
	snd_soc_write(max98095_codec, M98095_089_JACK_DET_AUTO, 0);
	snd_soc_write(max98095_codec, M98095_089_JACK_DET_AUTO,  M98095_JDEN|M98095_KEYEN|M98095_PIN5EN);

	/*Make the register stable */	
	for(i=0;i<5;i++){
		reg_val = max98095_codec->hw_read(max98095_codec, M98095_007_JACK_AUTO_STS);		
		printk("reg[M98095_007_JACK_AUTO_STS] = 0x%02X\n", reg_val);
	}
	
	if((0xC0 == reg_val)||(0xA0 == reg_val)) {
		state = HEADSET_WITHOUT_MIC;
		snd_soc_write(max98095_codec, M98095_013_JACK_INT_EN,
			0x03);
		printk("HEADSET_WITHOUT_MIC\n");
	} 
	else{
		state = HEADSET_MIC;
		snd_soc_write(max98095_codec, M98095_013_JACK_INT_EN,
			0x03);
		printk("HEADSET_MIC\n");	
	}			
	return state;
}

static int wired_switch_notify(struct notifier_block *self,
			      unsigned long action, void* dev)
{
	printk("max98095_jack_handler\n");
	tegra_switch_set_state(get_headset_state());

	return NOTIFY_OK;
}

void tegra_jack_suspend(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(wired_jack_gpios); i++)
		disable_irq(gpio_to_irq(wired_jack_gpios[i].gpio));
}

void tegra_jack_resume(void)
{
	int i, val;

	for (i = 0; i < ARRAY_SIZE(wired_jack_gpios); i++) {
		val = gpio_get_value(wired_jack_gpios[i].gpio);
		val = wired_jack_gpios[i].invert ? !val : val;
		val = val ? wired_jack_gpios[i].report : 0;

		snd_soc_jack_report(tegra_wired_jack, val,
			wired_jack_gpios[i].report);

		enable_irq(gpio_to_irq(wired_jack_gpios[i].gpio));
	}
}

static struct notifier_block wired_switch_nb = {
	.notifier_call = wired_switch_notify,
};
#endif

/* platform driver */
static int tegra_wired_jack_probe(struct platform_device *pdev)
{
	int ret;
	int hp_det_n = 0, cdc_irq = 0;
	int en_mic_int = -1, en_mic_ext = -1;
	int en_spkr = 0;
	struct wired_jack_conf *pdata;

	pdata = (struct wired_jack_conf *)pdev->dev.platform_data;

	if (!pdata || !pdata->hp_det_n){
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
	|| !pdata->en_mic_int || !pdata->en_mic_ext
#endif
		pr_err("Please set up gpio pins for jack.\n");
		return -EBUSY;
	}

	hp_det_n = pdata->hp_det_n;
	wired_jack_gpios[HEAD_DET_GPIO].gpio = hp_det_n;

	ret = snd_soc_jack_add_gpios(tegra_wired_jack,
				     ARRAY_SIZE(wired_jack_gpios),
				     wired_jack_gpios);
	if (ret) {
		pr_err("Could NOT set up gpio pins for jack.\n");
		snd_soc_jack_free_gpios(tegra_wired_jack,
					ARRAY_SIZE(wired_jack_gpios),
					wired_jack_gpios);
		return ret;
	}
	/* restore configuration of these pins */
	tegra_wired_jack_conf.hp_det_n = hp_det_n;

	// Communicate the jack connection state at device bootup
	tegra_switch_set_state(get_headset_state());

#ifdef CONFIG_SWITCH
	snd_soc_jack_notifier_register(tegra_wired_jack,
				       &wired_switch_nb);
#endif
	return ret;
}

static int tegra_wired_jack_remove(struct platform_device *pdev)
{
	snd_soc_jack_free_gpios(tegra_wired_jack,
				ARRAY_SIZE(wired_jack_gpios),
				wired_jack_gpios);
	return 0;
}

static struct platform_driver tegra_wired_jack_driver = {
	.probe = tegra_wired_jack_probe,
	.remove = tegra_wired_jack_remove,
	.driver = {
		.name = "tegra_wired_jack",
		.owner = THIS_MODULE,
	},
};


int tegra_jack_init(struct snd_soc_codec *codec)
{
	int ret;

	if (!codec)
		return -1;
	printk("***tegra_jack_init \n");
	tegra_wired_jack = kzalloc(sizeof(*tegra_wired_jack), GFP_KERNEL);
	if (!tegra_wired_jack) {
		pr_err("failed to allocate tegra_wired_jack \n");
		return -ENOMEM;
	}

	/* Add jack detection */
	ret = snd_soc_jack_new(codec->socdev->card, "Wired Accessory Jack",
			       SND_JACK_HEADSET, tegra_wired_jack);
	if (ret < 0)
		goto failed;

#ifdef CONFIG_SWITCH
	/* Addd h2w swith class support */
	ret = switch_dev_register(&wired_switch_dev);
	if (ret < 0)
		goto switch_dev_failed;
#endif

	ret = platform_driver_register(&tegra_wired_jack_driver);
	if (ret < 0)
		goto platform_dev_failed;

	return 0;

#ifdef CONFIG_SWITCH
switch_dev_failed:
	switch_dev_unregister(&wired_switch_dev);
#endif
platform_dev_failed:
	platform_driver_unregister(&tegra_wired_jack_driver);
failed:
	if (tegra_wired_jack) {
		kfree(tegra_wired_jack);
		tegra_wired_jack = 0;
	}
	return ret;
}

void tegra_jack_exit(void)
{
#ifdef CONFIG_SWITCH
	switch_dev_unregister(&wired_switch_dev);
#endif
	platform_driver_unregister(&tegra_wired_jack_driver);

	if (tegra_wired_jack) {
		kfree(tegra_wired_jack);
		tegra_wired_jack = 0;
	}
}
