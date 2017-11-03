/*
 * drivers/misc/keybd-led.c
 *
 * Driver for keyboard led.
 *
 * Copyright (c) 2011, ZTE Corporation. any problem,refer to gao.qiang18@zte.com.cn
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

#include <linux/regulator/consumer.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/reboot.h>

#include "../staging/android/timed_output.h"

//static struct regulator *regulator;
static int timeout;
extern int tps6586x_keybd_led_on(void);
extern int tps6586x_keybd_led_off(void);

/*ZTE: add by tengfei for off led when power off or reset , 2012.3.28, ++*/
static void keybd_led_stop(void);

static int keybd_led_notify_sys(struct notifier_block *this, unsigned long code,
			  void *unused)
{
	if (code == SYS_DOWN || code == SYS_POWER_OFF)
	{
		/* Off led */
		keybd_led_stop();
	}	
	return NOTIFY_DONE;
}

static struct notifier_block keybd_led_notifier = {
	.notifier_call = keybd_led_notify_sys,
};
/*ZTE: add by tengfei for off led when power off or reset , 2012.3.28, --*/

static void keybd_led_start(void)
{
        int ret = 0;
        ret = tps6586x_keybd_led_on();
        if(ret < 0)
        {
            printk("failed to open key_led");
        }
}

static void keybd_led_stop(void)
{
	int ret = 0;
        ret = tps6586x_keybd_led_off();
        if(ret < 0)
        {
            printk("failed to close key_led");
        }
}

/*
 * Timeout value can be changed from sysfs entry
 * created by timed_output_dev.
 * echo 100 > /sys/class/timed_output/vibrator/enable
 */
 /*ZTE: modified by tengfei for key backlight interface, 2011.12.6, ++*/
#if 0
static void keybd_led_enable(struct timed_output_dev *dev, int value)
{
	timeout = value;

	if (value) {
		keybd_led_start();
		msleep(value);
		keybd_led_stop();
	} else {
		keybd_led_stop();
	}
}
#else
static void keybd_led_enable(struct timed_output_dev *dev, int value)
{
	timeout = value;

	if (value) 
		keybd_led_start();
	else 
		keybd_led_stop();
}
#endif
/*ZTE: modified by tengfei for key backlight interface, 2011.12.6, --*/

/*
 * Timeout value can be read from sysfs entry
 * created by timed_output_dev.
 * cat /sys/class/timed_output/vibrator/enable
 */
static int keybd_led_get_time(struct timed_output_dev *dev)
{
	return timeout;
}

static struct timed_output_dev keybd_led_dev = {
	.name		= "keybd_led",
	.get_time	= keybd_led_get_time,
	.enable		= keybd_led_enable,
};

static int __init keybd_led_init(void)
{
	int status;

	status = timed_output_dev_register(&keybd_led_dev);

	/*ZTE: add by tengfei for off led when power off or reset , 2012.3.28*/
	status = register_reboot_notifier(&keybd_led_notifier);	

	if (status) {
            printk("failed to register a timed_ouput dev\n");
    	}
	return status;
}

static void __exit keybd_led_exit(void)
{
	timed_output_dev_unregister(&keybd_led_dev);
	/*ZTE: add by tengfei for off led when power off or reset , 2012.3.28*/	
	unregister_reboot_notifier(&keybd_led_notifier);		
}

MODULE_DESCRIPTION("timed output keybd-led device");
MODULE_AUTHOR("gao.qiang18@zte.com.cn");

module_init(keybd_led_init);
module_exit(keybd_led_exit);
