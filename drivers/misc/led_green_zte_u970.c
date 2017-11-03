/*
 * drivers/misc/max1749.c
 *
 * Driver for MAX1749, vibrator motor driver.
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

#include <linux/regulator/consumer.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/wakelock.h>

#include "../staging/android/timed_output.h"

int tps6586x_greenlight_ctrl(int onoff);
int tps6586x_redlight_ctrl(int onoff);

/*ZTE: add by tengfei for off led when power off or reset , 2012.3.5*/
static int led_green_notify_sys(struct notifier_block *this, unsigned long code,void *unused);

static void led_start(void);
static void led_stop(void);

static int green_led_timeout;

static void timed_control_init(void);
static void timed_control_remove(void);
static void led_on_timer_fun(unsigned long);
static void led_off_timer_fun(unsigned long);

static int timed_control_work = 0;
static int led_flash_time = 1;

#define MSG_WARN_ON  500
#define MSG_WARN_OFF  500
#define CALL_WARN_ON  500
#define CALL_WARN_OFF  2500

static struct timer_list  led_on_timer;
static struct timer_list  led_off_timer;
static struct work_struct led_on_work;
static struct work_struct led_off_work;

/*ZTE: add by tengfei for wakelock when indicate SMS comes for 5 times, 2012.4.5*/
struct wake_lock led_indicator_wakelock;


/*ZTE: add by tengfei for off led when power off or reset , 2012.3.5, ++*/
static struct notifier_block led_green_notifier = {
	.notifier_call = led_green_notify_sys,
};

static int led_green_notify_sys(struct notifier_block *this, unsigned long code,
			  void *unused)
{
	if (code == SYS_DOWN || code == SYS_POWER_OFF)
	{
		/* Off led */
		timed_control_remove();
		led_stop();
	}	
	return NOTIFY_DONE;
}
/*ZTE: add by tengfei for off led when power off or reset , 2012.3.5, --*/

static void timed_control_init(void)
{      
    if(0 == timed_control_work)
    {
        printk("enter %s ,set_up timer!!\n", __func__);  
        setup_timer(&led_on_timer, led_on_timer_fun,0); 
        setup_timer(&led_off_timer, led_off_timer_fun,0); 
        timed_control_work = 1;
    }
}

static void timed_control_remove(void)
{
    if(1 == timed_control_work)
    {
        printk("enter %s ,del timer!!\n", __func__);   
        del_timer(&led_on_timer);
        del_timer(&led_off_timer);
        timed_control_work = 0;
    }
}

static void led_on_timer_fun(unsigned long data)
{      
    printk("enter %s !!\n", __func__);  
    if(led_flash_time >= 5)
    {
        timed_control_remove();
        led_flash_time = 1;
        schedule_work(&led_off_work); 
		
	 /*ZTE: add by tengfei for wakelock when indicate SMS comes for 5 times, 2012.4.5,++*/	
	if( wake_lock_active(&led_indicator_wakelock) )
		wake_unlock(&led_indicator_wakelock);   	 
	 /*ZTE: add by tengfei for wakelock when indicate SMS comes for 5 times, 2012.4.5,--*/	
    }
    else
    {
        mod_timer(&led_off_timer,jiffies + msecs_to_jiffies(MSG_WARN_OFF));   
        schedule_work(&led_off_work);    
    }    
}

static void led_off_timer_fun(unsigned long data)
{
    printk("enter %s !!\n", __func__);     
    led_flash_time ++;
    mod_timer(&led_on_timer,jiffies + msecs_to_jiffies(MSG_WARN_ON));
    schedule_work(&led_on_work);
}

static void led_on_work_fun(struct work_struct *work)
{
    led_start(); 
}
static void led_off_work_fun(struct work_struct *work)
{
    led_stop(); 
}

static void led_start(void)
{
    int ret = -1;

    printk("start GREEN led \n");
    ret = tps6586x_redlight_ctrl(0);
    if(0 == ret)
    {
    	ret = tps6586x_greenlight_ctrl(1);
        if(ret < 0)
            printk("fail to start GREEN led !!!\n");
    }
    else
    {
        printk("fail to close RED led when start GREEN !!!\n");
    }
}

static void led_stop(void)
{
    int ret = -1;
    
    ret = tps6586x_greenlight_ctrl(0);
    if(ret < 0)
        printk("fail to close GREEN led !!!\n");
}

/*
 * Timeout value can be changed from sysfs entry
 * created by timed_output_dev.
 * echo 100 > /sys/class/timed_output/led_green_zte/enable
 */
static void led_enable(struct timed_output_dev *dev, int value)
{
    printk("enter into %s,led green on_time is %d\n",__func__,value);
    
	green_led_timeout = value;

    if (-1 == value) 
    {
	/*ZTE: add by tengfei for charging led wakelock, 2012.4.11, ++*/	
	if( !wake_lock_active(&led_indicator_wakelock) )
		wake_lock(&led_indicator_wakelock);    
	/*ZTE: add by tengfei for charging led wakelock, 2012.4.11, ++*/		
	
        timed_control_remove();
        led_start();
    } 
    else if(0 == value)
    {
        timed_control_remove();
        led_stop();

	/*ZTE: add by tengfei for charging led wakelock, 2012.4.11, ++*/	
	if( wake_lock_active(&led_indicator_wakelock) )
		wake_unlock(&led_indicator_wakelock);    
	/*ZTE: add by tengfei for charging led wakelock, 2012.4.11, ++*/			
    }
    else
    {
        #if 1   //add timed control by zhanming 20111108
		
	/*ZTE: add by tengfei for wakelock when indicate SMS comes for 5 times, 2012.4.5, ++*/		
	if( !wake_lock_active(&led_indicator_wakelock) )
		wake_lock(&led_indicator_wakelock);   
	/*ZTE: add by tengfei for wakelock when indicate SMS comes for 5 times, 2012.4.5, --*/			
		
        timed_control_remove();
        led_stop();
        led_start();
        timed_control_init();
        mod_timer(&led_on_timer,jiffies + msecs_to_jiffies(MSG_WARN_ON));   

        #else
		led_start();
		msleep_interruptible(value);
		led_stop();
        #endif
	}
}

/*
 * Timeout value can be read from sysfs entry
 * created by timed_output_dev.
 * cat /sys/class/timed_output/led_green_zte/enable
 */
static int led_get_time(struct timed_output_dev *dev)
{
	return green_led_timeout;
}

static struct timed_output_dev led_green_dev = {
	.name		= "led_green_zte",
	.get_time	= led_get_time,
	.enable		= led_enable,
};

static int __init led_green_init(void)
{
	int status;

    INIT_WORK(&led_on_work, led_on_work_fun);
    INIT_WORK(&led_off_work, led_off_work_fun); 

	status = timed_output_dev_register(&led_green_dev);
	/*ZTE: add by tengfei for off led when power off or reset , 2012.3.5*/
	status = register_reboot_notifier(&led_green_notifier);	

	/*ZTE: add by tengfei for wakelock when indicate SMS comes for 5 times, 2012.4.5*/	
	wake_lock_init(&led_indicator_wakelock, WAKE_LOCK_SUSPEND, "Green_led");

	return status;
}

static void __exit led_green_exit(void)
{
    cancel_work_sync(&led_on_work);
    cancel_work_sync(&led_off_work);
    timed_output_dev_unregister(&led_green_dev);
    /*ZTE: add by tengfei for off led when power off or reset , 2012.3.5*/	
    unregister_reboot_notifier(&led_green_notifier);	
	
    /*ZTE: add by tengfei for wakelock when indicate SMS comes for 5 times, 2012.4.5*/		
    wake_lock_destroy(&led_indicator_wakelock);
}

MODULE_DESCRIPTION("timed output led green device");
MODULE_AUTHOR("GPL");

module_init(led_green_init);
module_exit(led_green_exit);
