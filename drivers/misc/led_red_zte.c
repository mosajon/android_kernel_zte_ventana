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

#include "../staging/android/timed_output.h"

extern int tps6591x_redlight_ctrl(int argument);
extern int tps6591x_greenlight_ctrl(int argument);
extern int tps6591x_map_gpio3_to_led2_blink(void);
extern int tps6591x_map_led2_to_gpio3_out_low(void);

static void led_start(void);
static void led_stop(void);

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
    ret = tps6591x_greenlight_ctrl(0);
    if(0 == ret)
    {
    	ret = tps6591x_redlight_ctrl(1);
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

    ret = tps6591x_redlight_ctrl(0);
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
        timed_control_remove();
        tps6591x_map_led2_to_gpio3_out_low();	
        led_start();
    } 
    else if(0 == value)
    {
        timed_control_remove();
        tps6591x_map_led2_to_gpio3_out_low();	
        led_stop();
    }
    else if(10 == value)
    {
        //tps6591x_set_charge_flag();
    }
    else if(30 == value)
    {
        timed_control_remove();
        tps6591x_map_led2_to_gpio3_out_low();		
        led_stop();
        led_start();
        timed_control_init();
        mod_timer(&led_on_timer,jiffies + msecs_to_jiffies(LOW_BAT_WARN_ON));   
    }
    else if(8 == value)	
    {
        timed_control_remove();
        ret =tps6591x_map_gpio3_to_led2_blink();
	 if(ret < 0)
	 	printk("Some error happened with return value %d\n", ret);    
    }    
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

    return status;
}

static void __exit led_red_exit(void)
{
    cancel_work_sync(&led_on_work);
    cancel_work_sync(&led_off_work);
    timed_output_dev_unregister(&led_red_dev);
}

MODULE_DESCRIPTION("timed output led red device");
MODULE_AUTHOR("GPL");

module_init(led_red_init);
module_exit(led_red_exit);
