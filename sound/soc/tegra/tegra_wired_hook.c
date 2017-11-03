/*
 *   sound/soc/tegra/tegra_hook_jack.c
 *   Wang Yunan <wang.yunan@zte.com.cn>
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <mach/audio.h>
#include "tegra_soc.h"
#include "../codecs/max98095.h"
#include <mach/gpio.h>

extern struct snd_soc_codec *max98095_codec;
extern struct wired_jack_conf audio_wr_jack_conf; 

struct max98095_hook_info {
	struct input_dev	*idev;
	//struct i2c_client	*i2c;
	//struct device		*dev;
	int			irq;
};

//struct max98095_hook_info *hook_info;


/* MAX98095 gives us an interrupt when HOOK is held */
static irqreturn_t max98095_hook_handler(int irq, void *data)
{
	struct max98095_hook_info *info = data;
	int ret = 0;
	int hp_status = 0;

	printk("max98095_hook_handler\n");

	max98095_codec->hw_read(max98095_codec, M98095_000_HOST_DATA);	
	snd_soc_write(max98095_codec, M98095_089_JACK_DET_AUTO, 0);
	snd_soc_write(max98095_codec, M98095_089_JACK_DET_AUTO,  M98095_KEYEN|M98095_PIN5EN);
	ret = max98095_codec->hw_read(max98095_codec, M98095_007_JACK_AUTO_STS);
	printk("reg[M98095_007_JACK_AUTO_STS] = 0x%02X\n", ret);
	mdelay(100);
	hp_status = gpio_get_value(audio_wr_jack_conf.hp_det_n);
	if ((0x03 == ret)&&(0 == hp_status)){
		ret = 	0;
		printk( "hook button press down\n");
		input_report_key(info->idev, KEY_MEDIA, 1);
		mdelay(50);
		input_report_key(info->idev, KEY_MEDIA, 0);
		printk( "input_report_key is ok\n");
		input_sync(info->idev);
	}	
	return IRQ_HANDLED;
}

static int __devinit max98095_hook_probe(struct platform_device *pdev)
{
	struct max98095_hook_info *info;
	int			status;
	int irq;

	printk("max98095_hook_probe begin\n");
	info = kzalloc(sizeof (struct max98095_hook_info), GFP_KERNEL);
	info->idev = input_allocate_device();
	info->irq = TEGRA_GPIO_TO_IRQ(audio_wr_jack_conf.cdc_irq);	
	
	if (!info || !info->idev) {
		printk("max98095_hook_probe fail1\n");
		status = -ENOMEM;
		goto fail1;
	}

     	gpio_free(audio_wr_jack_conf.cdc_irq);
	tegra_gpio_enable(audio_wr_jack_conf.cdc_irq);
	gpio_request(audio_wr_jack_conf.cdc_irq, "CODEC_IRQ");	
	gpio_direction_input(audio_wr_jack_conf.cdc_irq);
	status = request_threaded_irq(info->irq, NULL, max98095_hook_handler,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, dev_name(&pdev->dev), info);
	if (status < 0){
		printk("max98095_hook_probe fail1\n");
		goto fail1;
	}
	#if 0
	gpio_free(audio_wr_jack_conf.cdc_irq);
	tegra_gpio_enable(audio_wr_jack_conf.cdc_irq);
	gpio_request(audio_wr_jack_conf.cdc_irq, "CODEC_IRQ");	
	gpio_direction_input(audio_wr_jack_conf.cdc_irq);
	#endif
	#if 1
	input_set_capability(info->idev, EV_KEY, KEY_MEDIA);
	input_set_capability(info->idev, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(info->idev, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(info->idev, EV_SW, SW_HEADPHONE_INSERT);
	input_set_capability(info->idev, EV_SW, SW_MICROPHONE_INSERT);
	input_set_capability(info->idev, EV_KEY, KEY_POWER);
	input_set_capability(info->idev, EV_KEY, KEY_END);
	#endif
	/* register */
	status = input_register_device(info->idev);
	if (status < 0){
		printk("max98095_hook_probe fail3\n");
		goto fail3;
	}	


	platform_set_drvdata(pdev, info);



	return 0;

fail3:
	free_irq(info->irq, info);
fail1:
	input_free_device(info->idev);
	kfree(info);
	dev_err(&pdev->dev, "can't register, err %d\n", status);

	return status;

}

static int __devexit max98095_hook_remove(struct platform_device *pdev)
{
	struct max98095_hook_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq, info);
	input_unregister_device(info->idev);
	kfree(info);
	return 0;
}

static struct platform_driver max98095_hook_driver = {
	.driver		= {
		.name	= "tegra-hook",
		.owner	= THIS_MODULE,
	},
	.probe		= max98095_hook_probe,
	.remove		= __devexit_p(max98095_hook_remove),
};

static int __init max98095_hook_init(void)
{
	return platform_driver_register(&max98095_hook_driver);
}
module_init(max98095_hook_init);

static void __exit max98095_hook_exit(void)
{
	platform_driver_unregister(&max98095_hook_driver);
}
module_exit(max98095_hook_exit);

MODULE_DESCRIPTION("Maxim MAX98095 ONKEY driver");
MODULE_AUTHOR("Wang Yunan<wang.yunan@zte.com.cn>");
MODULE_LICENSE("GPL");

