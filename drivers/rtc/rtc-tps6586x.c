/*
 * drivers/rtc/rtc-tps6586x.c
 *
 * RTC driver for TI TPS6586x
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/tps6586x.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>

#define RTC_CTRL	0xc0
#define POR_RESET_N	BIT(7)
#define OSC_SRC_SEL	BIT(6)
#define RTC_ENABLE	BIT(5)	/* enables alarm */
#define RTC_BUF_ENABLE	BIT(4)	/* 32 KHz buffer enable */
#define PRE_BYPASS	BIT(3)	/* 0=1KHz or 1=32KHz updates */
#define CL_SEL_MASK	(BIT(2)|BIT(1))
#define RTC_ENABLE_ALARM2_POWERON  BIT(0)
#define CL_SEL_POS	1
#define RTC_ALARM1_HI	0xc1
#define RTC_ALARM2_HI  0xc4
#define RTC_COUNT4	0xc6
#define RTC_COUNT4_DUMMYREAD 0xc5  /* start a PMU RTC access by reading the register prior to the RTC_COUNT4 */
#define ALM1_VALID_RANGE_IN_SEC 0x3FFF /*only 14-bits width in second*/
#define ALM2_VALID_RANGE_IN_SEC 0x7FFFF /*only 18-bits width in second*/

struct tps6586x_rtc {
	unsigned long		epoch_start;
	int			irq;
	struct rtc_device	*rtc;
	bool			irq_en;
};

static inline struct device *to_tps6586x_dev(struct device *dev)
{
	return dev->parent;
}

static int tps6586x_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	unsigned long long ticks = 0;
	unsigned long seconds;
	u8 buff[6];
	int err;
	int i;

	err = tps6586x_reads(tps_dev, RTC_COUNT4_DUMMYREAD, sizeof(buff), buff);
	if (err < 0) {
		dev_err(dev, "failed to read counter\n");
		return err;
	}

	for (i = 1; i < sizeof(buff); i++) {
		ticks <<= 8;
		ticks |= buff[i];
	}

	seconds = ticks >> 10;

	seconds += rtc->epoch_start;
	rtc_time_to_tm(seconds, tm);
	printk("read rtc time: %d:%d:%d  %d:%d:%d\n",(tm->tm_year + 1900),(tm->tm_mon + 1),tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);
	return rtc_valid_tm(tm);
}

static int tps6586x_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	unsigned long long ticks;
	unsigned long seconds;
	u8 buff[5];
	int err;

	printk("get time from upper: %d:%d:%d  %d:%d:%d\n",(tm->tm_year + 1900),(tm->tm_mon + 1),tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);
	rtc_tm_to_time(tm, &seconds);

	if (WARN_ON(seconds < rtc->epoch_start)) {
		dev_err(dev, "requested time unsupported\n");
		return -EINVAL;
	}

	seconds -= rtc->epoch_start;

	ticks = (unsigned long long)seconds << 10;
	buff[0] = (ticks >> 32) & 0xff;
	buff[1] = (ticks >> 24) & 0xff;
	buff[2] = (ticks >> 16) & 0xff;
	buff[3] = (ticks >> 8) & 0xff;
	buff[4] = ticks & 0xff;

	err = tps6586x_clr_bits(tps_dev, RTC_CTRL, RTC_ENABLE);
	if (err < 0) {
		dev_err(dev, "failed to clear RTC_ENABLE\n");
		return err;
	}

	err = tps6586x_writes(tps_dev, RTC_COUNT4, sizeof(buff), buff);
	if (err < 0) {
		dev_err(dev, "failed to program new time\n");
		return err;
	}

	err = tps6586x_set_bits(tps_dev, RTC_CTRL, RTC_ENABLE);
	if (err < 0) {
		dev_err(dev, "failed to set RTC_ENABLE\n");
		return err;
	}

	return 0;
}

static int tps6586x_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	unsigned long seconds;
	unsigned long ticks;
	unsigned long rtc_current_time;
	unsigned long long rticks = 0;
	u8 buff[3];
	u8 buff_2[2];
	u8 rbuff[6];
	int err;
	int i;
	u8 val;

	if (rtc->irq == -1)
		return -EIO;

	rtc_tm_to_time(&alrm->time, &seconds);

	if (WARN_ON(alrm->enabled && (seconds < rtc->epoch_start))) {
		dev_err(dev, "can't set alarm to requested time\n");
		return -EINVAL;
	}

	if (alrm->enabled && !rtc->irq_en) {
		enable_irq(rtc->irq);
		//able_irq(rtc->irq + 10);	   //this is enable rtc_alarm_2 interrup
		rtc->irq_en = true;
	} else if (!alrm->enabled && rtc->irq_en) {
		disable_irq(rtc->irq);
		rtc->irq_en = false;
	}

	err = tps6586x_read(tps_dev, RTC_CTRL, &val);
	if (err < 0) {
		dev_err(dev, "failed to read RTC_CTRL\n");
		return err;
	}

	printk("alrm->enabled = %d,rtc->irq = %d,reg 0xc0 = 0x%x\n",alrm->enabled,rtc->irq,val);

	tps6586x_set_bits(tps_dev, RTC_CTRL, RTC_ENABLE_ALARM2_POWERON);   

	seconds -= rtc->epoch_start;

	err = tps6586x_reads(tps_dev, RTC_COUNT4_DUMMYREAD, sizeof(rbuff), rbuff);
	if (err < 0) {
		dev_err(dev, "failed to read counter\n");
		return err;
	}

	for (i = 1; i < sizeof(rbuff); i++) {
		rticks <<= 8;
		rticks |= rbuff[i];
	}

	rtc_current_time = rticks >> 10;
	if ((seconds - rtc_current_time) > ALM2_VALID_RANGE_IN_SEC)
		seconds = rtc_current_time - 1;

//	ticks = (unsigned long long)seconds << 10;
	ticks = (unsigned long long)(seconds + 4) << 10;

	buff[0] = (ticks >> 16) & 0xff;
	buff[1] = (ticks >> 8) & 0xff;
	buff[2] = ticks & 0xff;

	err = tps6586x_writes(tps_dev, RTC_ALARM1_HI, sizeof(buff), buff);
	if (err)
		dev_err(tps_dev, "unable to program alarm\n");

       buff_2[0] = (ticks >> 20) & 0xff;
       buff_2[1] = (ticks >> 12) & 0xff;
/*
       err = tps6586x_writes(tps_dev,RTC_ALARM2_HI,sizeof(buff_2),buff_2);
       if(err)
                dev_err(tps_dev,"unable to program alarm 2\n");
*/
	err = tps6586x_write(tps_dev, (RTC_ALARM2_HI + 1),buff_2[1]);
	if (err)
		dev_err(tps_dev, "unable to program alarm\n");
	err = tps6586x_write(tps_dev, RTC_ALARM2_HI,buff_2[0]);
	if (err)
		dev_err(tps_dev, "unable to program alarm\n");

	return err;
}

static int tps6586x_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	unsigned long ticks;
	unsigned long seconds;
	u8 buff[3];
	int err;

	err = tps6586x_reads(tps_dev, RTC_ALARM1_HI, sizeof(buff), buff);
	if (err)
		return err;

	ticks = (buff[0] << 16) | (buff[1] << 8) | buff[2];
	seconds = ticks >> 10;
	seconds += rtc->epoch_start;

	rtc_time_to_tm(seconds, &alrm->time);

	return 0;
}

static int tps6586x_rtc_alarm_irq_enable(struct device *dev,
					 unsigned int enabled)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);
	struct device *tps_dev = to_tps6586x_dev(dev);
	u8 buff;
	int err;

	if (rtc->irq == -1)
		return -EIO;

	err = tps6586x_read(tps_dev, RTC_CTRL, &buff);
	if (err < 0) {
		dev_err(dev, "failed to read RTC_CTRL\n");
		return err;
	}

	if ((enabled && (buff & RTC_ENABLE)) ||
	    (!enabled && !(buff & RTC_ENABLE)))
		return 0;

	if (enabled) {
		err = tps6586x_set_bits(tps_dev, RTC_CTRL, RTC_ENABLE);
		if (err < 0) {
			dev_err(dev, "failed to set RTC_ENABLE\n");
			return err;
		}
		err = tps6586x_set_bits(tps_dev, RTC_CTRL, RTC_ENABLE_ALARM2_POWERON);
		if (err < 0) {
			dev_err(dev, "failed to clear bit 0\n");
			return err;
		}
		
		if (!rtc->irq_en) {
//	enable_irq(rtc->irq + 10);
			enable_irq(rtc->irq);
			rtc->irq_en = true;
		}
	} else {
	
		err = tps6586x_clr_bits(tps_dev, RTC_CTRL, RTC_ENABLE_ALARM2_POWERON);
		if (err < 0) {
			dev_err(dev, "failed to clear RTC_ENABLE\n");
			return err;
		}
	
		if (rtc->irq_en) {
//	disable_irq(rtc->irq + 10);
			disable_irq(rtc->irq);
			rtc->irq_en = false;
		}
	}

	return 0;
}

static const struct rtc_class_ops tps6586x_rtc_ops = {
	.read_time	= tps6586x_rtc_read_time,
	.set_time	= tps6586x_rtc_set_time,
	.set_alarm	= tps6586x_rtc_set_alarm,
	.read_alarm	= tps6586x_rtc_read_alarm,
	.alarm_irq_enable = tps6586x_rtc_alarm_irq_enable,
};

static irqreturn_t tps6586x_rtc_irq(int irq, void *data)
{
	struct device *dev = data;
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);

	rtc_update_irq(rtc->rtc, 1, RTC_IRQF | RTC_AF);
	return IRQ_HANDLED;
}
static irqreturn_t tps6586x_rtc_alarm2_irq(int irq, void *data)
{
	struct device *dev = data;
	struct tps6586x_rtc *rtc = dev_get_drvdata(dev);

	rtc_update_irq(rtc->rtc, 1, RTC_IRQF | RTC_AF);
	return IRQ_HANDLED;
}

extern void gpio_keys_set_report_keypower_state(int state);
extern int zte_get_board_id(void);

#if 0
static irqreturn_t tps6586x_on_key_irq(void)
{
    int board_id = -1;
    
    printk("enter into tps6586x_on_key_irq");

    board_id = zte_get_board_id();
    if((board_id == 0) || (board_id == 1))
        {
           gpio_keys_set_report_keypower_state(1);
        }
    return IRQ_HANDLED;
}

#endif
extern irqreturn_t tps6586x_chg_stat_irq(int irq, void *data);
struct device * p_dev = NULL;
static int create_rtc_proc_file(void);
/*ZTE:add by caixiaoguang for set a flag before power off++*/
extern void tps6586x_set_power_off_flag(void);
static ssize_t set_power_off_flag_proc_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
    tps6586x_set_power_off_flag();
    return 1;
}

static struct file_operations set_power_off_flag_proc_ops = {

   	.owner   = THIS_MODULE, 
   	.write = set_power_off_flag_proc_write,
};

static int create_set_power_off_flag_proc_file(void)	
{  
	struct proc_dir_entry *entry;        
       entry = create_proc_entry("driver/set_power_off_flag", 0666, NULL);     
   	if (entry)  
       	entry->proc_fops = &set_power_off_flag_proc_ops;           
       return 0;
}
/*ZTE:add by caixiaoguang for set a flag before power off--*/
static int __devinit tps6586x_rtc_probe(struct platform_device *pdev)
{
	struct tps6586x_rtc_platform_data *pdata = pdev->dev.platform_data;
	struct device *tps_dev = to_tps6586x_dev(&pdev->dev);
	struct tps6586x_rtc *rtc;
	int err;
	struct tps6586x_epoch_start *epoch;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform_data specified\n");
		return -EINVAL;
	}

	rtc = kzalloc(sizeof(*rtc), GFP_KERNEL);

	if (!rtc)
		return -ENOMEM;

	rtc->irq = -1;

	if (pdata->irq < 0)
		dev_warn(&pdev->dev, "no IRQ specified, wakeup is disabled\n");

	epoch = &pdata->start;
	rtc->epoch_start = mktime(epoch->year, epoch->month, epoch->day,
				  epoch->hour, epoch->min, epoch->sec);

	dev_set_drvdata(&pdev->dev, rtc);

	device_init_wakeup(&pdev->dev, 1);

	rtc->rtc = rtc_device_register("tps6586x-rtc", &pdev->dev,
				       &tps6586x_rtc_ops, THIS_MODULE);

	if (IS_ERR(rtc->rtc)) {
		err = PTR_ERR(rtc->rtc);
		goto fail;
	}

	/* 1 kHz tick mode, enable tick counting */
	err = tps6586x_update(tps_dev, RTC_CTRL,
		RTC_ENABLE | OSC_SRC_SEL | ((pdata->cl_sel << CL_SEL_POS) &
					    CL_SEL_MASK),
		RTC_ENABLE | OSC_SRC_SEL | PRE_BYPASS | CL_SEL_MASK);
	if (err < 0) {
		dev_err(&pdev->dev, "unable to start counter\n");
		goto fail;
	}

	if (pdata && (pdata->irq >= 0)) {
		rtc->irq = pdata->irq;
		printk("in tps6586x_rtc_probe():pdate->irq = 0x%x\n",pdata->irq);
		err = request_threaded_irq(pdata->irq, NULL, tps6586x_rtc_irq,
					   IRQF_ONESHOT, "tps6586x-rtc",
					   &pdev->dev);
		if (err) {
			dev_warn(&pdev->dev, "unable to request IRQ(%d)\n", rtc->irq);
			rtc->irq = -1;
		} else {
			enable_irq_wake(rtc->irq);
			disable_irq(rtc->irq);
		}
/*
		err = request_threaded_irq(442, NULL, tps6586x_rtc_irq,
					   IRQF_ONESHOT, "tps6586x-alarm-2",
					   &pdev->dev);
		if (err) {
			dev_warn(&pdev->dev, "unable to request IRQ(%d)\n", 442);
			rtc->irq = -1;
		} else {
			device_init_wakeup(&pdev->dev, 1);
			enable_irq_wake(442);
			//sable_irq(440);
			disable_irq(442);
		} 
        tps6586x_rtc_alarm_irq_enable(&pdev->dev,1);
*/
        err = request_threaded_irq(437, NULL, tps6586x_chg_stat_irq,
					   IRQF_ONESHOT, "tps6586x-chg-stat",
					   &pdev->dev);
		if (err) {
			dev_warn(&pdev->dev, "unable to request IRQ(%d)\n", 437);
			//rtc->irq = -1;
		} else {
			device_init_wakeup(&pdev->dev, 1);
			enable_irq_wake(437);
			//sable_irq(440);
			//enable_irq(437);
		}
	}

        p_dev = &pdev->dev;
        create_rtc_proc_file();        
        create_set_power_off_flag_proc_file();
	return 0;

fail:
	if (!IS_ERR_OR_NULL(rtc->rtc))
		rtc_device_unregister(rtc->rtc);
	device_init_wakeup(&pdev->dev, 0);
	kfree(rtc);
	return err;
}

static int __devexit tps6586x_rtc_remove(struct platform_device *pdev)
{
	struct tps6586x_rtc *rtc = dev_get_drvdata(&pdev->dev);

	if (rtc->irq != -1)
		free_irq(rtc->irq, rtc);
	rtc_device_unregister(rtc->rtc);
	kfree(rtc);
	return 0;
}

static struct platform_driver tps6586x_rtc_driver = {
	.driver	= {
		.name	= "tps6586x-rtc",
		.owner	= THIS_MODULE,
	},
	.probe	= tps6586x_rtc_probe,
	.remove	= __devexit_p(tps6586x_rtc_remove),
};
static ssize_t rtc_proc_read(struct file *filp, const char *buff, size_t len, loff_t *off)
{

    int i = 0;
    uint8_t reg_val[21];
      
        for(i = 0;i < 11;i++)
            {
                tps6586x_read(p_dev->parent, (0xc0+i), &reg_val[i]);
                printk("pmu int reg address: 0x%x,value:0x%x\n",(0xc0+i),reg_val[i]);
            }
        for(i = 0;i < 9;i++)
            {
                tps6586x_read(p_dev->parent, (0xb0+i), &reg_val[11+i]);                
                printk("pmu int reg address: 0x%x,value:0x%x\n",(0xb0+i),reg_val[11+i]);                
            }

	return (ssize_t)0;
}

static struct file_operations rtc_proc_ops = {

   	.owner   = THIS_MODULE, 
   	.read = rtc_proc_read,
};

static int create_rtc_proc_file(void)	
{  
	struct proc_dir_entry *entry;          
 
   	entry = create_proc_entry("driver/rtc_status", 0, NULL);     
   	if (entry)  
       	entry->proc_fops = &rtc_proc_ops;            
 
   	return 0;
}

static int __init tps6586x_rtc_init(void)
{
	return platform_driver_register(&tps6586x_rtc_driver);
}
module_init(tps6586x_rtc_init);

static void __exit tps6586x_rtc_exit(void)
{
	platform_driver_unregister(&tps6586x_rtc_driver);
}
module_exit(tps6586x_rtc_exit);

MODULE_DESCRIPTION("TI TPS6586x RTC driver");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtc-tps6586x");
