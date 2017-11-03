/*
 * drivers/misc/zter.c
 *
 * Driver for vibrator motor driver.
 *
 * Copyright (c) 2011, ZTE Corporation.gaoqiang18@zte.com.cn
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

#include "../staging/android/timed_output.h"

static int timeout;
extern int tps6586x_vibrator_on(void);
extern int tps6586x_vibrator_off(void);
extern int tps6586x_vibrate_intensity(unsigned char flag);

static void vibrator_start(void)
{
        int ret = 0;
        ret = tps6586x_vibrator_on();
        if(ret < 0)
        {
            printk("failed to open vibrator");
        }
}

static void vibrator_stop(void)
{
	int ret = 0;
        ret = tps6586x_vibrator_off();
        if(ret < 0)
        {
            printk("failed to close vibrator");
        }
}

/*
 * Timeout value can be changed from sysfs entry
 * created by timed_output_dev.
 * echo 100 > /sys/class/timed_output/vibrator/enable
 */
static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	int ret = 0;
	unsigned char flag = (unsigned char)((value & 0xffffffff) >> 29);
	if(flag)
	{
		printk("intensity modulate: 0x%x",flag);
		ret = tps6586x_vibrate_intensity(flag);
		if(ret < 0)
		{
			printk("failed to set intensity\n");
			return;
		}
	}
	else
	{
		tps6586x_vibrate_intensity(4);  //middle level
	}
	
	timeout = (value & 0x0fffffff);

	if (value) {
		vibrator_start();
		msleep(timeout);
		vibrator_stop();
	} else {
		vibrator_stop();
	}
}

/*
 * Timeout value can be read from sysfs entry
 * created by timed_output_dev.
 * cat /sys/class/timed_output/vibrator/enable
 */
static int vibrator_get_time(struct timed_output_dev *dev)
{
	return timeout;
}

static struct timed_output_dev vibrator_dev = {
	.name		= "vibrator",
	.get_time	= vibrator_get_time,
	.enable		= vibrator_enable,
};

static int __init vibrator_init(void)
{
	int status;

	status = timed_output_dev_register(&vibrator_dev);

	if (status) {
            printk("failed to register a timed_ouput dev\n");
    	}
	return status;
}

static void __exit vibrator_exit(void)
{
	timed_output_dev_unregister(&vibrator_dev);
}

MODULE_DESCRIPTION("timed output vibrator device");
MODULE_AUTHOR("GPL");

module_init(vibrator_init);
module_exit(vibrator_exit);
