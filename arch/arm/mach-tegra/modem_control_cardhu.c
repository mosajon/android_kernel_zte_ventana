/*****************************************************************************
 * Copyright(C) 2011, ZTE Corporation
 *
 *  Module Name	: 
 *  File Name		:  modem_control.c
 *  Description : 
 *  Author      		:  lixingyuan	
 *  Version     : 
 *  Data        		:  2011-07-22
 *  Others      : 
 *  Revision Details1£º
 *     Modify Data£º
 *     Version£º
 *     Author£º
 *     Modification£º
 *  Revision Details2£º
 *****************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>

#include <mach/gpio.h>

#include "board.h"
#include "clock.h"
#include "board-cardhu.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"

static int usb_switch_state = 0;

static int modem_power_enable(int enable)
{
    printk(KERN_INFO "modem_power_enable %d\n", enable);
    if(enable)
        gpio_set_value(MODEM_POWER_ENB, 1);
    else
        gpio_set_value(MODEM_POWER_ENB, 0);

    return 0;
}

static int modem_usb_enable(int enable)
{      
    printk(KERN_INFO "modem_usb_enable %d\n", enable);
    if(enable)
        gpio_set_value(MODEM_USB_CTRL, 0);
    else
        gpio_set_value(MODEM_USB_CTRL, 1);
    
    return 0;
}

/* add by ZTE zhanming for HDMI 20111024 ++*/
int modem_otg_power_enable(int enable)
{      
    printk(KERN_INFO "HDMI_DCDC_ENB %d\n", enable);
    if(enable)
        gpio_set_value(HDMI_DCDC_ENB, 1);
    else
        gpio_set_value(HDMI_DCDC_ENB, 0);
    
    return 0;
}
/* add by ZTE zhanming for HDMI 20111024 --*/

static int modem_usb3v3_enable(int enable)
{
    printk(KERN_INFO "modem_usb3v3_enable %d\n", enable);
    if(enable)
        gpio_set_value(MODEM_USB3V3_ENB, 1);
    else
        gpio_set_value(MODEM_USB3V3_ENB, 0);
    
    return 0;
}

static ssize_t modem_usb_switch_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t status;
	long value;
    char *usb_ap[2] = { "USB_SWITCH=AP", NULL };
    char *usb_modem[2] = { "USB_SWITCH=MODEM", NULL };    
    
	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
        status = 1;
        if (value == 0) {
            printk(KERN_NOTICE "switch usb to modem\n");
            kobject_uevent_env( &dev->kobj, KOBJ_CHANGE, usb_modem);
            modem_usb_enable(0);
        } else if (value == 1) {
            printk(KERN_NOTICE "switch usb to ap\n");
            kobject_uevent_env( &dev->kobj, KOBJ_CHANGE, usb_ap);
            modem_usb_enable(1);
        } else {
            status = -EINVAL;
        }
	}

    return status;
}
static const DEVICE_ATTR(usb_switch, 0600, NULL, modem_usb_switch_store);

static ssize_t modem_usb_ldo_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t status;
	long value;

	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
        status = 1;
        if (value == 0) {
            printk(KERN_NOTICE "disable usb3v3 ldo\n");
            modem_usb3v3_enable(0);
        } else if (value == 1) {
            printk(KERN_NOTICE "enable usb3v3 ldo\n");
            modem_usb3v3_enable(1);
        } else {
            status = -EINVAL;
        }
	}

    return status;
}
static const DEVICE_ATTR(usb_ldo, 0600, NULL, modem_usb_ldo_store);

static ssize_t modem_power_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t status;
	long value;

	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
        status = 1;
        if (value == 0) {
            printk(KERN_NOTICE "poweroff modem\n");
            
            modem_power_enable(0);
        } else if (value == 1) {
            printk(KERN_NOTICE "poweron modem\n");
            modem_power_enable(1);
        } else {
            status = -EINVAL;
        }
	}

    return status;
}
static const DEVICE_ATTR(poweron, 0600, NULL, modem_power_store);

static ssize_t modem_reset_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    ssize_t status;
    long value;
    char *modem_reset[2] = { "MODEM_STATE=RESET", NULL };

    status = strict_strtol(buf, 0, &value);
    if (status == 0) {
        status = 1;
        if (value == 1) {
            kobject_uevent_env( &dev->kobj, KOBJ_CHANGE, modem_reset);
            
            printk(KERN_NOTICE "reset modem\n");
            modem_power_enable(0);
            modem_usb3v3_enable(0);
            msleep(1000);
            modem_usb3v3_enable(1);
            modem_power_enable(1);
        } else {
            status = -EINVAL;
        }
    }

    return status;
}
static const DEVICE_ATTR(reset, 0600, NULL, modem_reset_store);

static ssize_t modem_sleep_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    int value;
    
    value = gpio_get_value(MODEM_TO_AP_SLP);
    if (value < 0)
        return value;

    if (value)
        strcpy(buf, "1\n");
    else
        strcpy(buf, "0\n");
    
    return 2;
}
static const DEVICE_ATTR(sleep, 0600, modem_sleep_show, NULL);

 /*ZTE:add by caixiaoguang 20111021 for modem dead++*/
static ssize_t modem_dead_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    int value;
    
    value = gpio_get_value(MODEM_DEAD_STAT);
    if (value < 0)
        return value;

    if (value)
        strcpy(buf, "0\n");//modem not dead
    else
        strcpy(buf, "1\n");//modem is dead
    
    return 2;
}
static const DEVICE_ATTR(dead, 0600, modem_dead_show, NULL);
/*ZTE:add by caixiaoguang 20111021 for modem dead--*/
static ssize_t modem_wakeup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t status;
	long value;

	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
        status = 1;
        if (value == 1) {
            printk(KERN_NOTICE "wakeup modem\n");
            gpio_set_value(AP_TO_MODEM_WKUP, 1);
            msleep(100);
            gpio_set_value(AP_TO_MODEM_WKUP, 0);
            msleep(100);
            gpio_set_value(AP_TO_MODEM_WKUP, 1);
        } else {
            status = -EINVAL;
        }
	}

    return status;
}
static const DEVICE_ATTR(wakeup, 0600, NULL, modem_wakeup_store);

static ssize_t modem_apwakeup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    int value;
    
    value = gpio_get_value(MODEM_TO_AP_WKUP);
    if (value < 0)
        return value;

    if (value)
        strcpy(buf, "1\n");
    else
        strcpy(buf, "0\n");
    
    return 2;
}
static const DEVICE_ATTR(apwakeup, 0600, modem_apwakeup_show, NULL);

static ssize_t modem_apsleep_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t status;
	long value;

	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
        status = 1;
        if (value == 1) {
            gpio_set_value(AP_TO_MODEM_SLP, 1);
        } else if (value == 0) {
            gpio_set_value(AP_TO_MODEM_SLP, 0);
        } else {
            status = -EINVAL;
        }
	}

    return status;
}
static const DEVICE_ATTR(apsleep, 0600, NULL, modem_apsleep_store);

static ssize_t modem_download_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t status;
	long value;
        char *modem_download[2] = { "MODEM_STATE=DOWNLOAD", NULL };

	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
        status = 1;
        if (value == 1) {
            kobject_uevent_env( &dev->kobj, KOBJ_CHANGE, modem_download);
            
            modem_power_enable(0);
            modem_usb3v3_enable(0);
            modem_usb_enable(0);
            mdelay(500);
            modem_usb3v3_enable(1);
            modem_power_enable(1);
        } else {
            status = -EINVAL;
        }
	}

    return status;
}
static const DEVICE_ATTR(download, 0600, NULL, modem_download_store);

static ssize_t modem_otgpower_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t status;
	long value;

	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
        status = 1;
        if (value == 1) {
            printk(KERN_INFO "otg_power enable\n");
            gpio_set_value(HDMI_DCDC_ENB, 1);
        } else if (value == 0) {
            printk(KERN_INFO "otg_power disable\n");
            gpio_set_value(HDMI_DCDC_ENB, 0);
        } else {
            status = -EINVAL;
        }
	}

    return status;
}
static const DEVICE_ATTR(otg_power, 0600, NULL, modem_otgpower_store);

static const struct attribute *modem_control_attrs[] = {
	&dev_attr_usb_switch.attr,
	&dev_attr_usb_ldo.attr,
	&dev_attr_poweron.attr,
	&dev_attr_reset.attr,
	&dev_attr_sleep.attr,
	&dev_attr_wakeup.attr,
	&dev_attr_apsleep.attr,
	&dev_attr_apwakeup.attr,
	&dev_attr_download.attr,
	&dev_attr_otg_power.attr,
	&dev_attr_dead.attr,/*ZTE:add by caixiaoguang 20111021 for modem dead*/
	NULL,
};

static const struct attribute_group modem_control_attr_group = {
	.attrs = (struct attribute **) modem_control_attrs,
};

static struct wake_lock modem_wake_lock;
static struct work_struct modem_env_work;
static struct kobject *pkobj;
static int suspended;
static int wakeup_irq;

static void modem_dead_work(struct work_struct *work)
{
    char *modem_dead[2] = { "MODEM_STATE=DEAD", NULL };

    kobject_uevent_env( pkobj, KOBJ_CHANGE, modem_dead);    
}

int modem_dead_check(void){
    int value;
    value = !gpio_get_value(MODEM_DEAD_STAT);
    return value;
}
EXPORT_SYMBOL(modem_dead_check);

static irqreturn_t modem_wakeup_irq(int irq, void *data)
{
    struct platform_device *pdev = data;
 	int value;

    //printk(KERN_INFO "modem_wakeup_irq......\n");   
    value = gpio_get_value(MODEM_DEAD_STAT);
    if (!value){
        printk(KERN_INFO "modem_dead!\n");
        pkobj = &pdev->dev.kobj;
        if (!work_pending(&modem_env_work))
    		schedule_work(&modem_env_work);
    }
    wake_lock_timeout(&modem_wake_lock, msecs_to_jiffies(5000));
    
    return IRQ_HANDLED;
}

static int __init parse_usb_swtich(char *arg)
{
	ssize_t status;
	long value;

    printk(KERN_INFO "parse_usb_swtich: %s\n", arg);

	status = strict_strtol(arg, 0, &value);
	if (status == 0) {
		if (value == 1)
			usb_switch_state = 1;
	}
	return 1;
}
__setup("usb_port_mode=", parse_usb_swtich);

static int __devinit modem_control_probe(struct platform_device *pdev)
{
    int ret;
    printk(KERN_INFO "modem_control_probe\n");

    ret = gpio_request(MODEM_POWER_ENB, "modem_power");
    if (ret < 0) {
        printk(KERN_WARNING "failed to request gpio MODEM_POWER_ENB %d\n", ret);
    }
    gpio_direction_output(MODEM_POWER_ENB, 1);
    tegra_gpio_enable(MODEM_POWER_ENB);
    
    ret = gpio_request(MODEM_USB_CTRL, "usb_switch");	
    if (ret < 0) {
        printk(KERN_WARNING "failed to request gpio MODEM_USB_CTRL %d\n", ret);
    }
    tegra_gpio_enable(MODEM_USB_CTRL);
    printk(KERN_INFO "usb_switch_state: %d\n", usb_switch_state);
    gpio_direction_output(MODEM_USB_CTRL, usb_switch_state);
    
    ret =  gpio_request(MODEM_USB3V3_ENB, "modem_usb3v3");	
    if (ret < 0) {
        printk(KERN_WARNING "failed to request gpio MODEM_USB3V3_ENB %d\n", ret);
    }
    tegra_gpio_enable(MODEM_USB3V3_ENB);
    gpio_direction_output(MODEM_USB3V3_ENB, 1);

    ret =  gpio_request(MODEM_RST_SET, "modem_reset");	
    if (ret < 0) {
        printk(KERN_WARNING "failed to request gpio MODEM_RST_SET %d\n", ret);
    }
    tegra_gpio_enable(MODEM_RST_SET);
    gpio_direction_input(MODEM_RST_SET);
    
    ret =  gpio_request(MODEM_TO_AP_SLP, "modem_sleep");	
    if (ret < 0) {
        printk(KERN_WARNING "failed to request gpio MODEM_TO_AP_SLP %d\n", ret);
    }
    tegra_gpio_enable(MODEM_TO_AP_SLP);
    gpio_direction_input(MODEM_TO_AP_SLP);

    /*++ cuijian modify init sequence, must keep ap_sleep init before modem_wakeup 20120401++*/
    ret =  gpio_request(AP_TO_MODEM_SLP, "ap_sleep");	
    if (ret < 0) {
        printk(KERN_WARNING "failed to request gpio AP_TO_MODEM_SLP %d\n", ret);
    }
    tegra_gpio_enable(AP_TO_MODEM_SLP);
    gpio_direction_output(AP_TO_MODEM_SLP, 0);

    ret =  gpio_request(AP_TO_MODEM_WKUP, "modem_wakeup");	
    if (ret < 0) {
        printk(KERN_WARNING "failed to request gpio AP_TO_MODEM_WKUP %d\n", ret);
    }
    tegra_gpio_enable(AP_TO_MODEM_WKUP);
    gpio_direction_output(AP_TO_MODEM_WKUP, 1);
    /*-- cuijian modify init sequence, must keep ap_sleep init before modem_wakeup 20120401--*/
    ret =  gpio_request(MODEM_TO_AP_WKUP, "ap_wakeup");	
    if (ret < 0) {
        printk(KERN_WARNING "failed to request gpio MODEM_TO_AP_WKUP %d\n", ret);
    }
    tegra_gpio_enable(MODEM_TO_AP_WKUP);
    gpio_direction_input(MODEM_TO_AP_WKUP);

    ret =  gpio_request(HDMI_DCDC_ENB, "otg_power");	
    if (ret < 0) {
        printk(KERN_WARNING "failed to request gpio HDMI_DCDC_ENB %d\n", ret);
    }
    tegra_gpio_enable(HDMI_DCDC_ENB);
    gpio_direction_output(HDMI_DCDC_ENB, 1);

    /*ZTE:add by caixiaoguang 20111021 for modem dead++*/   
    ret =  gpio_request(MODEM_DEAD_STAT, "modem_dead");	
    if (ret < 0) {
        printk(KERN_WARNING "failed to request gpio MODEM_DEAD_STAT %d\n", ret);
    }
    tegra_gpio_enable(MODEM_DEAD_STAT);
    gpio_direction_input(MODEM_DEAD_STAT);
    /*ZTE:add by caixiaoguang 20111021 for modem dead--*/

    wake_lock_init(&modem_wake_lock, WAKE_LOCK_SUSPEND, "modem_lock");
    INIT_WORK(&modem_env_work, modem_dead_work);
    
    wakeup_irq = gpio_to_irq(MODEM_TO_AP_WKUP);
	ret = request_irq(wakeup_irq, modem_wakeup_irq,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "modem_wakeup_irq", pdev);
    if (ret < 0) {
        printk(KERN_WARNING "failed to request modem_wakeup_irq %d\n", ret);
    }

    printk(KERN_INFO "create sysfs interface\n");
    ret = sysfs_create_group(&pdev->dev.kobj, &modem_control_attr_group);
    if (ret)
    printk(KERN_WARNING "sysfs_create_group ret %d\n", ret);
    device_init_wakeup(&pdev->dev, 1);
    suspended = 0;

    return ret;
}

static int __devexit modem_control_remove(struct platform_device *pdev)
{
    printk(KERN_NOTICE "modem_control_remove\n");
    device_init_wakeup(&pdev->dev, 0);
    sysfs_remove_group(&pdev->dev.kobj, &modem_control_attr_group);
    gpio_free(MODEM_POWER_ENB);
    gpio_free(MODEM_USB_CTRL);
    gpio_free(MODEM_USB3V3_ENB);
    
    return 0;
}

static int modem_control_suspend(struct platform_device *pdev, pm_message_t state)
{
    if (device_may_wakeup(&pdev->dev))
    {
        enable_irq_wake(wakeup_irq);
    }
    suspended = 1;
    return 0;
}

static int modem_control_resume(struct platform_device *pdev)
{
    if (device_may_wakeup(&pdev->dev))
    {
        disable_irq_wake(wakeup_irq);
    }
    suspended = 0;
    return 0;
}

static struct platform_driver modem_control_driver = {
	.remove = modem_control_remove,
    .suspend = modem_control_suspend,
    .resume = modem_control_resume,
	.driver = {
		   .name = "modem_control",
	},
};

static int __devinit modem_control_init(void)
{
    printk(KERN_NOTICE "modem_control_init\n");
	return platform_driver_probe(&modem_control_driver, modem_control_probe);
}

static void __devexit modem_control_exit(void)
{
    printk(KERN_NOTICE "modem_control_exit\n");
	platform_driver_unregister(&modem_control_driver);
}

module_init(modem_control_init);
module_exit(modem_control_exit);
