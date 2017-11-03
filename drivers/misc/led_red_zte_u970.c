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

/*ZTE: add by tengfei for led indicator at PMU side, 2011.12.2, ++*/
int tps6586x_redlight_ctrl(int onoff);
int tps6586x_greenlight_ctrl(int onoff);
int tps6586x_redblink_ctrl(void);
/*ZTE: add by tengfei for led indicator at PMU side, 2011.12.2, --*/

//extern void tps6591x_set_charge_flag(void);//test by zhanming 20111126
static void led_start(void);
static void led_stop(void);

/*ZTE: add by tengfei for off led when power off or reset , 2012.3.5*/
static int led_red_notify_sys(struct notifier_block *this, unsigned long code,void *unused);

static int red_led_timeout;

static void timed_control_init(void);
static void timed_control_remove(void);
static void led_on_timer_fun(unsigned long);
static void led_off_timer_fun(unsigned long);

static int timed_control_work = 0;

#define LOW_BAT_WARN_ON  500
#define LOW_BAT_WARN_OFF  30000

static struct timer_list  led_on_timer;
static struct timer_list  led_off_timer;
static struct work_struct led_on_work;
static struct work_struct led_off_work;

/*ZTE: add by tengfei for charging led wakelock, 2012.4.11*/
struct wake_lock led_charge_wakelock;

/*ZTE: add by tengfei for off led when power off or reset , 2012.3.5, ++*/
static struct notifier_block led_red_notifier = {
	.notifier_call = led_red_notify_sys,
};

static int led_red_notify_sys(struct notifier_block *this, unsigned long code,
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
    mod_timer(&led_off_timer,jiffies + msecs_to_jiffies(LOW_BAT_WARN_OFF));   
    schedule_work(&led_off_work);    
}

static void led_off_timer_fun(unsigned long data)
{
    printk("enter %s !!\n", __func__);     
    mod_timer(&led_on_timer,jiffies + msecs_to_jiffies(LOW_BAT_WARN_ON));
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

    printk("start RED led \n");
    ret = tps6586x_greenlight_ctrl(0);
    if(0 == ret)
    {
    	ret = tps6586x_redlight_ctrl(1);
        if(ret < 0)
            printk("fail to start RED led !!!\n");
    }
    else
    {
        printk("fail to close GREEN led when start RED!!!\n");
    }
}

static void led_stop(void)
{
    int ret = -1;

    ret = tps6586x_redlight_ctrl(0);
    if(ret < 0)
        printk("fail to close RED led !!!\n");
}

/*
 * Timeout value can be changed from sysfs entry
 * created by timed_output_dev.
 * echo 100 > /sys/class/timed_output/led_red_zte/enable
 */
static void led_enable(struct timed_output_dev *dev, int value)
{
    int ret;

    printk("enter into %s,led red on_time is %d\n",__func__,value);    
    red_led_timeout = value;

    if (-1 == value) 
    {

	/*ZTE: add by tengfei for charging led wakelock, 2012.4.11, ++*/	
	if( !wake_lock_active(&led_charge_wakelock) )
		wake_lock(&led_charge_wakelock);    
	/*ZTE: add by tengfei for charging led wakelock, 2012.4.11, ++*/	    

        timed_control_remove();
        led_start();
    } 
    else if(0 == value)
    {

	/*ZTE: add by tengfei for charging led wakelock, 2012.4.11, ++*/	
	if( wake_lock_active(&led_charge_wakelock) )
		wake_unlock(&led_charge_wakelock);    
	/*ZTE: add by tengfei for charging led wakelock, 2012.4.11, ++*/	      
	
        timed_control_remove();
        led_stop();
    }
    else if(10 == value)
    {
        //tps6591x_set_charge_flag();
    }
    else if(30 == value)
    {
        #if 1   //add timed control by zhanming 20111108
        timed_control_remove();
        led_stop();
        led_start();
        timed_control_init();
        mod_timer(&led_on_timer,jiffies + msecs_to_jiffies(LOW_BAT_WARN_ON));   

        #else
		led_start();
		msleep_interruptible(2000);
		led_stop();
        #endif
    }
    /*ZTE: add by tengfei for ledx blink, 2011.11.29, ++*/	
    else if(8 == value)	
    {
    	timed_control_remove();
        ret =tps6586x_redblink_ctrl();
	 if(ret < 0)
	 	printk("Some error happened with return value %d\n",ret);    
    }    
    /*ZTE: add by tengfei for ledx blink, 2011.11.29, --*/	
}

/*
 * Timeout value can be read from sysfs entry
 * created by timed_output_dev.
 * cat /sys/class/timed_output/led_red_zte/enable
 */
static int led_get_time(struct timed_output_dev *dev)
{
	return red_led_timeout;
}

static struct timed_output_dev led_red_dev = {
	.name		= "led_red_zte",
	.get_time	= led_get_time,
	.enable		= led_enable,
};

static int __init led_red_init(void)
{
	int status;

    INIT_WORK(&led_on_work, led_on_work_fun);
    INIT_WORK(&led_off_work, led_off_work_fun);    

	status = timed_output_dev_register(&led_red_dev);

	/*ZTE: add by tengfei for off led when power off or reset , 2012.3.5*/
	status = register_reboot_notifier(&led_red_notifier);	

	/*ZTE: add by tengfei for charging led wakelock, 2012.4.11, ++*/
	wake_lock_init(&led_charge_wakelock, WAKE_LOCK_SUSPEND, "Red_led");	

	return status;
}

static void __exit led_red_exit(void)
{
    cancel_work_sync(&led_on_work);
    cancel_work_sync(&led_off_work);
    timed_output_dev_unregister(&led_red_dev);
    /*ZTE: add by tengfei for off led when power off or reset , 2012.3.5*/	
    unregister_reboot_notifier(&led_red_notifier);		

    /*ZTE: add by tengfei for charging led wakelock, 2012.4.11, ++*/	
    wake_lock_destroy(&led_charge_wakelock);	
}

MODULE_DESCRIPTION("timed output led red device");
MODULE_AUTHOR("GPL");

module_init(led_red_init);
module_exit(led_red_exit);
