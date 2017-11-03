/*
 * Core driver for TI TPS6586x PMIC family
 *
 * Copyright (c) 2010 CompuLab Ltd.
 * Mike Rapoport <mike@compulab.co.il>
 *
 * Based on da903x.c.
 * Copyright (C) 2008 Compulab, Ltd.
 * Mike Rapoport <mike@compulab.co.il>
 * Copyright (C) 2006-2008 Marvell International Ltd.
 * Eric Miao <eric.miao@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>

#include <linux/mfd/core.h>
#include <linux/mfd/tps6586x.h>
//ZTE:add by gaoq for time delay
#include <linux/time.h>
#include <linux/delay.h>
//ZTE:add by gaoq for time delay

#define TPS6586X_SUPPLYENE  0x14
#define EXITSLREQ_BIT       BIT(1) /* Exit sleep mode request */
#define SLEEP_MODE_BIT      BIT(3) /* Sleep mode */

/* GPIO control registers */
#define TPS6586X_GPIOSET1	0x5d
#define TPS6586X_GPIOSET2	0x5e

/* interrupt control registers */
#define TPS6586X_INT_ACK1	0xb5
#define TPS6586X_INT_ACK2	0xb6
#define TPS6586X_INT_ACK3	0xb7
#define TPS6586X_INT_ACK4	0xb8

/* interrupt mask registers */
#define TPS6586X_INT_MASK1	0xb0
#define TPS6586X_INT_MASK2	0xb1
#define TPS6586X_INT_MASK3	0xb2
#define TPS6586X_INT_MASK4	0xb3
#define TPS6586X_INT_MASK5	0xb4

/* device id */
#define TPS6586X_VERSIONCRC	0xcd

struct tps6586x_irq_data {
	u8	mask_reg;
	u8	mask_mask;
};

#define TPS6586X_IRQ(_reg, _mask)				\
	{							\
		.mask_reg = (_reg) - TPS6586X_INT_MASK1,	\
		.mask_mask = (_mask),				\
	}
//ZTE: add by gaoq for charger on_lien status test
bool charger_on_line = 0;
//ZTE: add by gaoq for charger on_lien status test
/*ZTE:add by caixiaoguang for set a flag before power off++*/
bool power_off_flag = 0;
/*ZTE:add by caixiaoguang for set a flag before power off--*/
static const struct tps6586x_irq_data tps6586x_irqs[] = {
	[TPS6586X_INT_PLDO_0]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 0),
	[TPS6586X_INT_PLDO_1]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 1),
	[TPS6586X_INT_PLDO_2]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 2),
	[TPS6586X_INT_PLDO_3]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 3),
	[TPS6586X_INT_PLDO_4]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 4),
	[TPS6586X_INT_PLDO_5]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 5),
	[TPS6586X_INT_PLDO_6]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 6),
	[TPS6586X_INT_PLDO_7]	= TPS6586X_IRQ(TPS6586X_INT_MASK1, 1 << 7),
	[TPS6586X_INT_COMP_DET]	= TPS6586X_IRQ(TPS6586X_INT_MASK4, 1 << 0),
	[TPS6586X_INT_ADC]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 1),
	[TPS6586X_INT_PLDO_8]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 2),
	[TPS6586X_INT_PLDO_9]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 3),
	[TPS6586X_INT_PSM_0]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 4),
	[TPS6586X_INT_PSM_1]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 5),
	[TPS6586X_INT_PSM_2]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 6),
	[TPS6586X_INT_PSM_3]	= TPS6586X_IRQ(TPS6586X_INT_MASK2, 1 << 7),
	[TPS6586X_INT_RTC_ALM1]	= TPS6586X_IRQ(TPS6586X_INT_MASK5, 1 << 4),
	[TPS6586X_INT_ACUSB_OVP] = TPS6586X_IRQ(TPS6586X_INT_MASK5, 0x03),
	[TPS6586X_INT_USB_DET]	= TPS6586X_IRQ(TPS6586X_INT_MASK5, 1 << 2),
	[TPS6586X_INT_AC_DET]	= TPS6586X_IRQ(TPS6586X_INT_MASK5, 1 << 3),
	[TPS6586X_INT_BAT_DET]	= TPS6586X_IRQ(TPS6586X_INT_MASK3, 1 << 0),
	[TPS6586X_INT_CHG_STAT]	= TPS6586X_IRQ(TPS6586X_INT_MASK4, 0xfc),
	[TPS6586X_INT_CHG_TEMP]	= TPS6586X_IRQ(TPS6586X_INT_MASK3, 0x06),
	[TPS6586X_INT_PP]	= TPS6586X_IRQ(TPS6586X_INT_MASK3, 0xf0),
	[TPS6586X_INT_RESUME]	= TPS6586X_IRQ(TPS6586X_INT_MASK5, 1 << 5),
	[TPS6586X_INT_LOW_SYS]	= TPS6586X_IRQ(TPS6586X_INT_MASK5, 1 << 6),
	[TPS6586X_INT_RTC_ALM2] = TPS6586X_IRQ(TPS6586X_INT_MASK4, 1 << 1),
};

struct tps6586x {
	struct mutex		lock;
	struct device		*dev;
	struct i2c_client	*client;

	struct gpio_chip	gpio;
	struct irq_chip		irq_chip;
	struct mutex		irq_lock;
	int			irq_base;
	u32			irq_en;
	u8			mask_cache[5];
	u8			mask_reg[5];
};

/*ZTE: add by tengfei for U880F led indicator, 2012.4.26,++*/
#ifdef CONFIG_U880F_FLAG
static int is_u880f = 1;
#else
static int is_u880f = 0;
#endif
/*ZTE: add by tengfei for U880F led indicator, 2012.4.26,--*/

static inline int __tps6586x_read(struct i2c_client *client,
				  int reg, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
		return ret;
	}

	*val = (uint8_t)ret;

	return 0;
}

static inline int __tps6586x_reads(struct i2c_client *client, int reg,
				   int len, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, len, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading from 0x%02x\n", reg);
		return ret;
	}

	return 0;
}

static inline int __tps6586x_write(struct i2c_client *client,
				 int reg, uint8_t val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writing 0x%02x to 0x%02x\n",
				val, reg);
		return ret;
	}

	return 0;
}

static inline int __tps6586x_writes(struct i2c_client *client, int reg,
				  int len, uint8_t *val)
{
	int ret, i;

	for (i = 0; i < len; i++) {
		ret = __tps6586x_write(client, reg + i, *(val + i));
		if (ret < 0)
			return ret;
	}

	return 0;
}

int tps6586x_write(struct device *dev, int reg, uint8_t val)
{
	return __tps6586x_write(to_i2c_client(dev), reg, val);
}
EXPORT_SYMBOL_GPL(tps6586x_write);

int tps6586x_writes(struct device *dev, int reg, int len, uint8_t *val)
{
	return __tps6586x_writes(to_i2c_client(dev), reg, len, val);
}
EXPORT_SYMBOL_GPL(tps6586x_writes);

int tps6586x_read(struct device *dev, int reg, uint8_t *val)
{
	return __tps6586x_read(to_i2c_client(dev), reg, val);
}
EXPORT_SYMBOL_GPL(tps6586x_read);

int tps6586x_reads(struct device *dev, int reg, int len, uint8_t *val)
{
	return __tps6586x_reads(to_i2c_client(dev), reg, len, val);
}
EXPORT_SYMBOL_GPL(tps6586x_reads);

int tps6586x_set_bits(struct device *dev, int reg, uint8_t bit_mask)
{
	struct tps6586x *tps6586x = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&tps6586x->lock);

	ret = __tps6586x_read(to_i2c_client(dev), reg, &reg_val);
	if (ret)
		goto out;

	if ((reg_val & bit_mask) == 0) {
		reg_val |= bit_mask;
		ret = __tps6586x_write(to_i2c_client(dev), reg, reg_val);
	}
out:
	mutex_unlock(&tps6586x->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tps6586x_set_bits);

int tps6586x_clr_bits(struct device *dev, int reg, uint8_t bit_mask)
{
	struct tps6586x *tps6586x = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&tps6586x->lock);

	ret = __tps6586x_read(to_i2c_client(dev), reg, &reg_val);
	if (ret)
		goto out;

	if (reg_val & bit_mask) {
		reg_val &= ~bit_mask;
		ret = __tps6586x_write(to_i2c_client(dev), reg, reg_val);
	}
out:
	mutex_unlock(&tps6586x->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tps6586x_clr_bits);

int tps6586x_update(struct device *dev, int reg, uint8_t val, uint8_t mask)
{
	struct tps6586x *tps6586x = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&tps6586x->lock);

	ret = __tps6586x_read(tps6586x->client, reg, &reg_val);
	if (ret)
		goto out;

	if ((reg_val & mask) != val) {
		reg_val = (reg_val & ~mask) | val;
		ret = __tps6586x_write(tps6586x->client, reg, reg_val);
	}
out:
	mutex_unlock(&tps6586x->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tps6586x_update);

static struct i2c_client *tps6586x_i2c_client = NULL;

int tps6586x_write_data_sm3(int reg, unsigned int val)
{
        struct device *dev = NULL;
        int ret = -EINVAL;
        printk("enter tps6586x_write_data_sm3 reg %d val %d\r\n",reg , val);
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;
        ret = tps6586x_write(dev,reg,val);
        return 0;

}
EXPORT_SYMBOL_GPL(tps6586x_write_data_sm3);

int tps6586x_enable_sm3()
{
        struct device *dev = NULL;
        int ret = -EINVAL;
       unsigned int val = 0;
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        printk("enter tps6586x_enable_sm3\r\n");
        ret = tps6586x_write(dev, 0x4E, val);
       val |= 0x80;
    	ret = tps6586x_write(dev, 0x4E, val);
        return 0;
}
EXPORT_SYMBOL_GPL(tps6586x_enable_sm3);
int tps6586x_power_off(void)
{
	struct device *dev = NULL;
	int ret = -EINVAL;
       uint8_t val[3];

	if (!tps6586x_i2c_client)
		return ret;

	dev = &tps6586x_i2c_client->dev;

	ret = tps6586x_clr_bits(dev, TPS6586X_SUPPLYENE, EXITSLREQ_BIT);
	if (ret)
		return ret;
    //  ZTE: add by gaoq for test++
	ret = tps6586x_read(dev, 0xc0, &val[0]);
	ret = tps6586x_read(dev, 0xb3, &val[1]);
	ret = tps6586x_read(dev, 0xb4, &val[2]);
	ret = tps6586x_write(dev,0xb4,val[2]|0x14);//mask the rtc-alarm1 and usb det interrupt
	ret = tps6586x_read(dev, 0xb4, &val[2]);
	printk("reg 0xc0 val = 0x%x,reg 0xb3 val = 0x%x,reg 0xb4 val = 0x%x\n",val[0],val[1],val[2]);

    //  ZTE: add by gaoq for test++

	ret = tps6586x_set_bits(dev, TPS6586X_SUPPLYENE, SLEEP_MODE_BIT);
	if (ret)
		return ret;

	return 0;
}

//ZTE: add by gaoq for charger power sm2 ON and OFF 20110808
int tps6586x_en_sm2(void)
{
        struct device *dev = NULL;
        uint8_t val = 0;
        int ret = -EINVAL;

        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x12,&val);
        val |= 0x80;
        ret = tps6586x_write(dev,0x12,val);

        ret = tps6586x_read(dev,0x13,&val);
        val |= 0x80;
        ret = tps6586x_write(dev,0x13,val);
        return ret;
}
EXPORT_SYMBOL(tps6586x_en_sm2);

int tps686x_en_chg_norm(void)
{
        struct device *dev = NULL;
        uint8_t  val = 0;
        int ret = -EINVAL;

        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x4a,&val);
        val |= 0x02;
        ret = tps6586x_write(dev,0x4a,val);

        ret = tps6586x_read(dev,0x4b,&val);
        val |= 0x80;
        ret = tps6586x_write(dev,0x4b,val);
        return ret;
}
EXPORT_SYMBOL(tps686x_en_chg_norm);


int tps6586x_dis_sm2(void)
{
        struct device *dev = NULL;
        uint8_t  val = 0;
        int ret = -EINVAL;

        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x12,&val);
        val &= (~(0x80));
        ret = tps6586x_write(dev,0x12,val);

        ret = tps6586x_read(dev,0x13,&val);
        val &= (~(0x80));
        ret = tps6586x_write(dev,0x13,val);
        return ret;
}
EXPORT_SYMBOL(tps6586x_dis_sm2);

int tps686x_dis_chg_norm(void)
{
        struct device *dev = NULL;
        uint8_t  val = 0;
        int ret = -EINVAL;

        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x4a,&val);
        val &= (~(0x02));
        ret = tps6586x_write(dev,0x4a,val);

        return 0;        
}
/*ZTE:add by caixiaoguang 20111012 for get charge status++*/
int tps6586x_get_charge_status(void)
{
        struct device *dev = NULL;
        uint8_t  val = 0;
//        int is_charge_enable = 0;       
        
        dev = &tps6586x_i2c_client->dev;

        tps6586x_read(dev,0xBA,&val);//STAT2 reg
        val &= 0x06;//get BIT[1:2]        
        val = val >>1;
        //00:Pre-charge ,01:charge done,10:Fast charge,11:charge suspend,timer fault,charger off
#if 0
        if(val==0x0 ||val==0x2 )
            is_charge_enable =1;
        else
            is_charge_enable=0;
#endif        
        
       // printk("[%s]:CXG x1b tps6586x_get_charge_status is_charge_enable=%d\n",__FUNCTION__,is_charge_enable);
       //return is_charge_enable;  
        return 1;//ZTE:add by caixiaoguang for set is_charge_enable 1 when usb or ac is in
        
}
EXPORT_SYMBOL(tps6586x_get_charge_status);

int tps6586x_get_charger_present_status(void)
{
        struct device *dev = NULL;
        uint8_t  val = 0;
        int is_charger_present = 0;
        
        dev = &tps6586x_i2c_client->dev;

        tps6586x_read(dev,0xBB,&val);//STAT2 reg
        val &= 0x04;//get BIT[2]        
        val = val >>2;
        //0:charger not detect,1 :charger detect
        if(val==0x1  )
            is_charger_present =1;
        else
            is_charger_present=0;
        //printk("[%s]:CXG x1b tps6586x_get_charger_present_status is_charger_present=%d\n",__FUNCTION__,is_charger_present);

        return is_charger_present;
        
}
EXPORT_SYMBOL(tps6586x_get_charger_present_status);
/*ZTE:add by caixiaoguang 20111012 for get charge status--*/

/*ZTE:add by caixiaoguang 20120331 for set charge timeout 8 hours++*/
void tps6586x_set_charge_timer_fault(int long_time)
{
        struct device *dev = NULL;
        uint8_t  val = 0;  
        dev = &tps6586x_i2c_client->dev;

        tps6586x_read(dev,0x49,&val);//CHG1 reg        
        val &= 0x3F;
        if(1==long_time)
            {
                val |= 0xC0;
            }
        else
            {
                val |= 0x80;
            }
        tps6586x_write(dev,0x49,val);        
        //printk("[%s]:CXG x2 get 0x49 reg =%x\n",__func__,val);
        return;
        
}
EXPORT_SYMBOL(tps6586x_set_charge_timer_fault);
/*ZTE:add by caixiaoguang 20120331 for set charge timeout 8 hours--*/
void tps6586x_set_power_path(int auto_mode)
{
        struct device *dev = NULL;
        uint8_t  val = 0;  
        dev = &tps6586x_i2c_client->dev;

        tps6586x_read(dev,0x4C,&val);//CHG1 reg       
        if(1 == auto_mode)
            {
                val |= 0x01;
            }
        else
            {
                val &= 0xFE;
            }
        tps6586x_write(dev,0x4C,val);        
        //printk("[%s]:CXG x2 get 0x49 reg =%x\n",__func__,val);
        return;
        
}
EXPORT_SYMBOL(tps6586x_set_power_path);
void tps6586x_set_sys_dppm_threshold(int low_or_high)
{
        struct device *dev = NULL;
        uint8_t  val = 0;  
        dev = &tps6586x_i2c_client->dev;
        
        tps6586x_read(dev,0x4b,&val);  
        printk("[%s]CXG get reg 0x4b val=%x\n",__func__,val);
        val &= 0x9f;          
        if(1 == low_or_high)
            {
                val |= 0x40;
            }
        else
            {
                val |= 0x20; 
            }
        tps6586x_write(dev,0x4b,val);        
        tps6586x_read(dev,0x4b,&val);  
		printk("[%s]CXG x1 get reg 0x4b val=%x\n",__func__,val);
        return;
        
}
EXPORT_SYMBOL(tps6586x_set_sys_dppm_threshold);
//ZTE: add by gao for ac charge setting++
extern int fsl_charger_detect(void);
int tps6586x_set_input_current_500(void);
int set_ac_charge_current(void)
{  
    struct device *dev = NULL;
    uint8_t  val = 0;
    int ret = -1;
    printk("enter into %s\n",__func__);

    if(!tps6586x_i2c_client)
    return ret;

    dev = &tps6586x_i2c_client->dev;    

    //sleep 100ms to wait USB is ready
    mdelay(50);

    //check if it is AC charge
    ret = fsl_charger_detect();
    if(ret == -1)
    {
        printk("check ac charge:USB is not ready\n");
        return 0;
    }
    else if(ret == 0)
    {
        printk("pc host charge\n");
        /*ZTE:add by caixiaoguang for USB charge 20111215++*/            
        tps6586x_set_input_current_500();
		#if 0
        ret = tps6586x_read(dev,0x49,&val);
        //val &= (~0x08);
        //temp version
        val |= 0x08;
        val |= 0x04;
        ret = tps6586x_write(dev,0x49,val);

        ret = tps6586x_read(dev,0x4c,&val);
        val |= 0x02;
        ret = tps6586x_write(dev,0x4c,val);
		#endif
        tps6586x_en_sm2();
        tps686x_en_chg_norm();
        /*ZTE:add by caixiaoguang for USB charge 20111215--*/   
        return 0;
    }
    else if(ret == 1)
    {
        printk("set ac charge currrent to 2.1A\n");
        ret = tps6586x_read(dev,0x49,&val);
        //val &= (~0x08);
        //temp version
        val |= 0x08;
        val |= 0x04;
        ret = tps6586x_write(dev,0x49,val);

        ret = tps6586x_read(dev,0x4c,&val);
        val |= 0x02;
        ret = tps6586x_write(dev,0x4c,val);
            
            /*ZTE:add by caixiaoguang for  charge current 20111228++*/            
            tps6586x_en_sm2();
            tps686x_en_chg_norm();           
            /*ZTE:add by caixiaoguang for  charge current 20111228--*/ 
    }
    
    return 0;    
}
EXPORT_SYMBOL(set_ac_charge_current);
//ZTE: add by gao for ac charge setting--
int tps6586x_set_capa(void)
{
	    struct device *dev = NULL;
        uint8_t  val = 0;
        int ret = -EINVAL;

        printk("enter into tps6586x_set_capa\n");
    
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0xc0,&val);
		printk("get 0xc0 reg value: %x",val);
        val |= 0x4;
        ret = tps6586x_write(dev,0xc0,val);
        ret = tps6586x_read(dev,0xc0,&val);
		printk("after write,get 0xc0 reg value: %x",val);
        
        return 0;
}

int tps6586x_set_vrtc(void)
{
	    struct device *dev = NULL;
        uint8_t  val = 0;
        int ret = -EINVAL;

        printk("enter into tps6586x_set_vrtc\n");
    
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x44,&val);
		printk("get 0x44 reg value: %x",val);
        val |= 0x20;
        ret = tps6586x_write(dev,0x44,val);
        ret = tps6586x_read(dev,0x44,&val);
		printk("after write,get 0x44 reg value: %x",val);
        
        return 0;
}

int tps6586x_force_normal(void)
{
        struct device *dev = NULL;
        uint8_t  val = 0;
        int ret = -EINVAL;

        printk("neter into tps6586x_force_normal\n");
    
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x14,&val);
        val |= 0x02;
        ret = tps6586x_write(dev,0x14,val);
        
        return 0;
}

int tps6586x_onkey_irq_enable(void)
{
        struct device *dev = NULL;
        uint8_t  val = 0;
        int ret = -EINVAL;

        printk("neter into tps6586x_onkey_irq_enable\n");
    
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        //ret = tps6586x_read(dev,0xb4,&val);
        //printk("get mask reg: 0x%x\n",val);
        //val &= (~(0x20));
        //ret = tps6586x_write(dev,0xb4,val);
        //ret = tps6586x_write(dev,0xb0,0);
        //ret = tps6586x_write(dev,0xb1,0);
        ret = tps6586x_write(dev,0xb2,0);
        ret = tps6586x_write(dev,0xb3,0);
        ret = tps6586x_write(dev,0xb4,0);

        ret = tps6586x_read(dev,0x14,&val);
        printk("get pmu 0x14 reg: 0x%x\n",val);
        val |= 0x2;
        ret = tps6586x_write(dev,0x14,val);
        
         //ret = tps6586x_read(dev,0xb4,&val);
         //printk("5 secs later will trigger an int*******************************\n");

        //msleep(5000);
        // trigger an int
        //tps6586x_write(dev,0xb8,1);
         
        printk("write mask reg: 0x%x",val);
         
        return 0;         
}

int tps6586x_keybd_led_on(void)
{
        struct device *dev = NULL;
        uint8_t  val = 0;
        int ret = -EINVAL;
    
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x59,&val);
        val |= 0xff;
        ret = tps6586x_write(dev,0x59,val);
        ret = tps6586x_read(dev,0x59,&val);
        
        return 0;
}
EXPORT_SYMBOL(tps6586x_keybd_led_on);
int tps6586x_keybd_led_off(void)
{
    struct device *dev = NULL;
        uint8_t  val = 0;
        int ret = -EINVAL;
    
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x59,&val);
        ret = tps6586x_write(dev,0x59,0x0);

        return 0;
}
EXPORT_SYMBOL(tps6586x_keybd_led_off);

//ZTE: add by gaoq for set input charge current to 500mA 
int tps6586x_set_input_current_500(void)
{
        struct device *dev = NULL;
        uint8_t  val = 0;
        int ret = -EINVAL;
    
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;

        ret = tps6586x_read(dev,0x4c,&val);
        val &= ~0x02;/*ZTE:add by caixiaoguang for  charge current 20111228++*/
        val |= 0x04;
        ret = tps6586x_write(dev,0x4c,val);

        printk("have set the charger input current to 500MA\n");

        return 0;
}
EXPORT_SYMBOL(tps6586x_set_input_current_500);
//ZTE: add by gaoq for charger power sm2 ON and OFF 20110808

/*ZTE: add by tengfei for led indicator at PMU side, 2011.12.2, ++*/
extern int zte_get_board_id(void);

int tps6586x_greenlight_ctrl(int onoff)
{
	struct device *dev = NULL;
	uint8_t val = 0;
	int ret = -EINVAL;
    
	if(!tps6586x_i2c_client)
		goto err;

	if((onoff != 0) && (onoff != 1))
		goto err;

	dev = &tps6586x_i2c_client->dev;

	/*#1: RGB driver OFF*/		
	ret = tps6586x_read(dev,0x52,&val);	
	if(ret < 0)	
		goto err;

	val &= (~0x80);   	
	val |= (0x00 << 7);	 //RGB1 driver OFF first;

	ret = tps6586x_write(dev,0x52,val);	 	
	if(ret < 0)	
		goto err;
	/*#1: ---End*/		
	
       /*#2: Disable flash mode*/	
	ret = tps6586x_read(dev,0x50,&val);	
	if(ret < 0)	
		goto err;	

	val &= (~0x0f);   	
	val |= (0x0f << 0);	 //Disable flash mode;;

	ret = tps6586x_write(dev,0x50,val);	 	
	if(ret < 0)	
		goto err;
	/*#2: ---End*/	

	/*#3: Set Current Sink*/
	ret = tps6586x_read(dev,0x51,&val);	
	if(ret < 0)	
		goto err;

	val &= (~0x60);   	
	val |= (0x3 << 5);	 //Set current sink:11.1mA;
	ret = tps6586x_write(dev,0x51,val);	 	
	if(ret < 0)	
		goto err;
	/*#3: ---End*/		
	
       /*#4: Set ==RED== intensity*/	
	if ((zte_get_board_id() == 0) && (is_u880f == 0))
	{
	    //Ver1-Red for green led
	    ret = tps6586x_read(dev,0x51,&val);	
	    if(ret < 0)	
  	        goto err;

	    val &= (~0x1f);   	
	    val |= ((onoff) ? (0x1f << 0) : (0x00 << 0)); //Set Intensity
	
	    ret = tps6586x_write(dev,0x51,val);	 	
	    if(ret < 0)	
	        goto err;
	}
	else
	{
	    //Ver2-Green for green led
	    ret = tps6586x_read(dev,0x52,&val);	
	    if(ret < 0)	
  	        goto err;

	    val &= (~0x1f);   	
	    val |= ((onoff) ? (0x1f << 0) : (0x00 << 0)); //Set Intensity
	
	    ret = tps6586x_write(dev,0x52,val);	 	
	    if(ret < 0)	
	        goto err;
	}		
	/*#4: ---End*/			

       /*#5: ON or OFF RGB dirver*/	
	ret = tps6586x_read(dev,0x52,&val);	
	if(ret < 0)	
		goto err;	

	val &= (~0x80);   	
	val |= (onoff << 7); //ON or OFF;

	ret = tps6586x_write(dev,0x52,val);	 	
	if(ret < 0)	
		goto err;	
	/*#5: ---End*/			

	return 0;	
err:
	return ret;
	   
}
EXPORT_SYMBOL(tps6586x_greenlight_ctrl);

int tps6586x_redlight_ctrl(int onoff)
{
	struct device *dev = NULL;
	uint8_t val = 0;
	int ret = -EINVAL;
    
	if(!tps6586x_i2c_client)
		goto err;

	if((onoff != 0) && (onoff != 1))
		goto err;

	dev = &tps6586x_i2c_client->dev;

	/*#1: RGB driver OFF*/		
	ret = tps6586x_read(dev,0x52,&val);	
	if(ret < 0)	
		goto err;

	val &= (~0x80);   	
	val |= (0x0 << 7);	 //RGB1 driver OFF first;

	ret = tps6586x_write(dev,0x52,val);	 	
	if(ret < 0)	
		goto err;
	/*#1: ---End*/		
	
       /*#2: Disable flash mode*/	
	ret = tps6586x_read(dev,0x50,&val);	
	if(ret < 0)	
		goto err;	

	val &= (~0x0f);   	
	val |= (0x0f << 0);	 //Disable flash mode;;

	ret = tps6586x_write(dev,0x50,val);	 	
	if(ret < 0)	
		goto err;
	/*#2: ---End*/	

       /*#3: Set Current Sink*/	
	ret = tps6586x_read(dev,0x51,&val);	
	if(ret < 0)	
		goto err;

	val &= (~0x60);   	
	val |= (0x3 << 5);	 //Set current sink:11.1mA;
	ret = tps6586x_write(dev,0x51,val);	 	
	if(ret < 0)	
		goto err;
	/*#3: ---End*/		
	
       /*#4: Set ==GREEN==intensity*/	
	if ((zte_get_board_id() == 0) && (is_u880f == 0))
	{
	    //Ver1-Green for Red led
	    ret = tps6586x_read(dev,0x52,&val);	
	    if(ret < 0)	
  	        goto err;

	    val &= (~0x1f);   	
	    val |= ((onoff) ? (0x1f << 0) : (0x00 << 0)); //Set Intensity
	
	    ret = tps6586x_write(dev,0x52,val);	 	
	    if(ret < 0)	
	        goto err;
	}
	else
	{
	    //Ver2-Red for Red led
	    ret = tps6586x_read(dev,0x51,&val);	
	    if(ret < 0)	
  	        goto err;

	    val &= (~0x1f);   	
	    val |= ((onoff) ? (0x1f << 0) : (0x00 << 0)); //Set Intensity
	
	    ret = tps6586x_write(dev,0x51,val);	 	
	    if(ret < 0)	
	        goto err;
	}	
	/*#4: ---End*/			

       /*#5: ON or OFF RGB dirver*/	
	ret = tps6586x_read(dev,0x52,&val);	
	if(ret < 0)	
		goto err;	

	val &= (~0x80);   	
	val |= (onoff << 7); //ON or OFF;

	ret = tps6586x_write(dev,0x52,val);	 	
	if(ret < 0)	
		goto err;	
	/*#5: ---End*/			

	return 0;	
err:
	return ret;
	   
}
EXPORT_SYMBOL(tps6586x_redlight_ctrl);

int tps6586x_redblink_ctrl(void)
{
	struct device *dev = NULL;
	uint8_t  val = 0;
	int ret = -EINVAL;
    
	if(!tps6586x_i2c_client)
		goto err;

	dev = &tps6586x_i2c_client->dev;
	
	/*#1: RGB driver OFF*/		
	ret = tps6586x_read(dev,0x52,&val);	
	if(ret < 0)	
		goto err;
	
	val &= (~0x80);   	
	val |= (0x0 << 7);	 //RGB1 driver OFF first;

	ret = tps6586x_write(dev,0x52,val);	 	
	if(ret < 0)	
		goto err;
	/*#1: ---End*/		
	
       /*#2:Enable flash mode with 500ms on in 8sec period*/	
	ret = tps6586x_read(dev,0x50,&val);	
	if(ret < 0)	
		goto err;	

	val &= (~0x7f);   	
	val |= (0x0e << 0);	 //Peroid: 8sec;
	val |= (0x06 << 4);	 //ON: 500mS;	

	ret = tps6586x_write(dev,0x50,val);	 	
	if(ret < 0)	
		goto err;
	/*#2: ---End*/	

       /*#3: Set Current Sink*/	
	ret = tps6586x_read(dev,0x51,&val);	
	if(ret < 0)	
		goto err;

	val &= (~0x60);   	
	val |= (0x3 << 5);	 //Set current sink:11.1mA;
	ret = tps6586x_write(dev,0x51,val);	 	
	if(ret < 0)	
		goto err;
	/*#3: ---End*/	
	
       /*#4: Set intensity*/	
	if ((zte_get_board_id() == 0) && (is_u880f == 0))
	{
	    //Ver1-Green for Red led
	    ret = tps6586x_read(dev,0x52,&val);	
	    if(ret < 0)	
  	        goto err;

	    val &= (~0x1f);   	
	    val |= (0x1f << 0);	 //Set intensity;
	
	    ret = tps6586x_write(dev,0x52,val);	 	
	    if(ret < 0)	
	        goto err;
	}
	else
	{
	    //Ver2-Red for Red led
	    ret = tps6586x_read(dev,0x51,&val);	
	    if(ret < 0)	
  	        goto err;

	    val &= (~0x1f);   	
	    val |= (0x1f << 0);	 //Set intensity;	    	    
	
	    ret = tps6586x_write(dev,0x51,val);	 	
	    if(ret < 0)	
	        goto err;
	}		
	/*#4: ---End*/			

       /*#5: ON RGB dirver*/	
	ret = tps6586x_read(dev,0x52,&val);	
	if(ret < 0)	
		goto err;	

	val &= (~0x80);   	
	val |= (0x01 << 7); //ON;

	ret = tps6586x_write(dev,0x52,val);	 	
	if(ret < 0)	
		goto err;	
	/*#5: ---End*/			

	return 0;	
err:
	return ret;
}

EXPORT_SYMBOL(tps6586x_redblink_ctrl);
/*ZTE: add by tengfei for led indicator at PMU, 2011.12.2, --*/
static int tps6586x_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct tps6586x *tps6586x = container_of(gc, struct tps6586x, gpio);
	uint8_t val;
	int ret;

	ret = __tps6586x_read(tps6586x->client, TPS6586X_GPIOSET2, &val);
	if (ret)
		return ret;

	return !!(val & (1 << offset));
}


static void tps6586x_gpio_set(struct gpio_chip *chip, unsigned offset,
			      int value)
{
	struct tps6586x *tps6586x = container_of(chip, struct tps6586x, gpio);

#if 0
	tps6586x_update(tps6586x->dev, TPS6586X_GPIOSET2,
			value << offset, 1 << offset);
#endif
__tps6586x_write(tps6586x->client, TPS6586X_GPIOSET2,
			 value << offset);
}

static int tps6586x_gpio_input(struct gpio_chip *gc, unsigned offset)
{
	/* FIXME: add handling of GPIOs as dedicated inputs */
	return -ENOSYS;
}

static int tps6586x_gpio_output(struct gpio_chip *gc, unsigned offset,
				int value)
{
	struct tps6586x *tps6586x = container_of(gc, struct tps6586x, gpio);
	uint8_t val, mask;
	int ret;

	val = value << offset;
	mask = 0x1 << offset;
	ret = tps6586x_update(tps6586x->dev, TPS6586X_GPIOSET2, val, mask);
	if (ret)
		return ret;

	val = 0x1 << (offset * 2);
	mask = 0x3 << (offset * 2);

	return tps6586x_update(tps6586x->dev, TPS6586X_GPIOSET1, val, mask);
}

static int tps6586x_gpio_init(struct tps6586x *tps6586x, int gpio_base)
{
	if (!gpio_base)
		return 0;

	tps6586x->gpio.owner		= THIS_MODULE;
	tps6586x->gpio.label		= tps6586x->client->name;
	tps6586x->gpio.dev		= tps6586x->dev;
	tps6586x->gpio.base		= gpio_base;
	tps6586x->gpio.ngpio		= 4;
	tps6586x->gpio.can_sleep	= 1;

	tps6586x->gpio.direction_input	= tps6586x_gpio_input;
	tps6586x->gpio.direction_output	= tps6586x_gpio_output;
	tps6586x->gpio.set		= tps6586x_gpio_set;
	tps6586x->gpio.get		= tps6586x_gpio_get;

	return gpiochip_add(&tps6586x->gpio);
}

static int __remove_subdev(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int tps6586x_remove_subdevs(struct tps6586x *tps6586x)
{
	return device_for_each_child(tps6586x->dev, NULL, __remove_subdev);
}

static void tps6586x_irq_lock(struct irq_data *data)
{
	struct tps6586x *tps6586x = irq_data_get_irq_chip_data(data);

	mutex_lock(&tps6586x->irq_lock);
}

static void tps6586x_irq_enable(struct irq_data *irq_data)
{
	struct tps6586x *tps6586x = irq_data_get_irq_chip_data(irq_data);
	unsigned int __irq = irq_data->irq - tps6586x->irq_base;
	const struct tps6586x_irq_data *data = &tps6586x_irqs[__irq];

	tps6586x->mask_reg[data->mask_reg] &= ~data->mask_mask;
	tps6586x->irq_en |= (1 << __irq);
}

static void tps6586x_irq_disable(struct irq_data *irq_data)
{
	struct tps6586x *tps6586x = irq_data_get_irq_chip_data(irq_data);

	unsigned int __irq = irq_data->irq - tps6586x->irq_base;
	const struct tps6586x_irq_data *data = &tps6586x_irqs[__irq];

	tps6586x->mask_reg[data->mask_reg] |= data->mask_mask;
	tps6586x->irq_en &= ~(1 << __irq);
}

static void tps6586x_irq_sync_unlock(struct irq_data *data)
{
	struct tps6586x *tps6586x = irq_data_get_irq_chip_data(data);
	int i;

	for (i = 0; i < ARRAY_SIZE(tps6586x->mask_reg); i++) {
		if (tps6586x->mask_reg[i] != tps6586x->mask_cache[i]) {
			if (!WARN_ON(tps6586x_write(tps6586x->dev,
						    TPS6586X_INT_MASK1 + i,
						    tps6586x->mask_reg[i])))
				tps6586x->mask_cache[i] = tps6586x->mask_reg[i];
		}
	}

	mutex_unlock(&tps6586x->irq_lock);
}
extern void gpio_keys_report_keypower(int state);
/*ZTE:add by caixiaoguang for power on mode++*/
int tps6586x_set_power_on_flag(void)
{
        struct device *dev = NULL;
        uint8_t  val = 0;
        int ret = -EINVAL;
        //printk("[tps6586x_set_power_on_flag]:CXG x0 get reg 0x53 = 0x%x\n",val);
        if(!tps6586x_i2c_client)
            return ret;

        dev = &tps6586x_i2c_client->dev;
        ret = tps6586x_read(dev,0x53,&val);
        printk("[tps6586x_set_power_on_flag]:CXG x1 get reg 0x53 = 0x%x\n",val);
        val |= 0x01;
        ret = tps6586x_write(dev,0x53,val);        
        //ret = tps6586x_read(dev,0x53,&val);
        //printk("[tps6586x_set_power_on_flag]:CXG x2 get reg 0x53 = 0x%x\n",val);
        return ret;
}
EXPORT_SYMBOL(tps6586x_set_power_on_flag);
/*ZTE:add by caixiaoguang for set a flag before power off++*/
void tps6586x_set_power_off_flag(void)
{
    power_off_flag = 1;
}
/*ZTE:add by caixiaoguang for set a flag before power off--*/
int tps6586x_set_reboot_flag(void)
{
        struct device *dev = NULL;
        uint8_t  val = 0;
        int ret = -EINVAL;
        if(!tps6586x_i2c_client)
            return ret;
        dev = &tps6586x_i2c_client->dev;
        if(power_off_flag==0)
            {
                val =0x01 ;
                printk("[tps6586x_set_reboot_flag]:CXG x1set_reboot_flag val =1 !\n"); 
            }
        else
            {
                        
                if(1==tps6586x_get_charger_present_status())
                    {
                val =0x02 ;
                        printk("[tps6586x_set_reboot_flag]:CXG x2 set_reboot_flag val =2 !\n"); 
                    }
                else
                    {
                        val =0x03 ;
                        printk("[tps6586x_set_reboot_flag]:CXG x3 set_reboot_flag val =3 !\n"); 
                    }
                
            }        
        ret = tps6586x_write(dev,0x53,val);        
        if(ret<0)
            printk("[tps6586x_set_reboot_flag]:CXG tps6586x_set_reboot_flag error!\n");        
        return ret;
}
EXPORT_SYMBOL(tps6586x_set_reboot_flag);
/*ZTE:add by caixiaoguang for power on mode--*/


/*ZTE: add by tengfei for adapter interrupt lost when wake from sleep, 2012.3.24, ++*/
#ifdef CONFIG_MACH_VENTANA  
u32 interrupt_status_wake =0;
#endif
/*ZTE: add by tengfei for adapter interrupt lost when wake from sleep, 2012.3.24, --*/


static irqreturn_t tps6586x_irq(int irq, void *data)
{
	struct tps6586x *tps6586x = data;
	u32 acks;
	int ret = 0;
       uint8_t  val1=0 ;
       uint8_t  val2 = 0;

//       tps6586x_read(tps6586x->dev, 0xb7, &val1);
//       tps6586x_read(tps6586x->dev, 0xb8, &val2);
       printk("enter into %s %s reg 0xb7 = 0x%x,0xb8 = 0x%x\n",__FILE__,__func__,val1,val2);

	ret = tps6586x_reads(tps6586x->dev, TPS6586X_INT_ACK1,
			     sizeof(acks), (uint8_t *)&acks);

	if (ret < 0) {
		dev_err(tps6586x->dev, "failed to read interrupt status\n");
		return IRQ_NONE;
	}

	acks = le32_to_cpu(acks);
/*ZTE:delete by caixiaoguang for charge current++*/    
#if 0
    if(acks&0x00040000)
    {
        if(charger_on_line == 0)
            {
                tps6586x_set_input_current_500();
                tps6586x_en_sm2();
                tps686x_en_chg_norm();
                charger_on_line = 1;
            }
            else
            {
                tps6586x_dis_sm2();
                tps686x_dis_chg_norm();
                charger_on_line = 0;
            }
    }
#endif
/*ZTE:delete by caixiaoguang for charge current--*/ 


	/*ZTE: add by tengfei for adapter interrupt lost when wake from sleep, 2012.3.24,++*/
	#ifdef CONFIG_MACH_VENTANA  
	if((acks !=0) && (tps6586x->irq_en == 0))
	{
		interrupt_status_wake =acks;
	}
	#endif
	/*ZTE: add by tengfei for adapter interrupt lost when wake from sleep, 2012.3.24,--*/	
	

	while (acks) {
		int i = __ffs(acks);

		if (tps6586x->irq_en & (1 << i))
			handle_nested_irq(tps6586x->irq_base + i);

		acks &= ~(1 << i);
	}

	return IRQ_HANDLED;
}

static int __devinit tps6586x_irq_init(struct tps6586x *tps6586x, int irq,
				       int irq_base)
{
	int i, ret;
	u8 tmp[4];

	if (!irq_base) {
		dev_warn(tps6586x->dev, "No interrupt support on IRQ base\n");
		return -EINVAL;
	}

	mutex_init(&tps6586x->irq_lock);
	for (i = 0; i < 5; i++) {
		tps6586x->mask_cache[i] = 0xff;
		tps6586x->mask_reg[i] = 0xff;
		tps6586x_write(tps6586x->dev, TPS6586X_INT_MASK1 + i, 0xff);
	}

	tps6586x_reads(tps6586x->dev, TPS6586X_INT_ACK1, sizeof(tmp), tmp);

	tps6586x->irq_base = irq_base;

	tps6586x->irq_chip.name = "tps6586x";
	tps6586x->irq_chip.irq_enable = tps6586x_irq_enable;
	tps6586x->irq_chip.irq_disable = tps6586x_irq_disable;
	tps6586x->irq_chip.irq_bus_lock = tps6586x_irq_lock;
	tps6586x->irq_chip.irq_bus_sync_unlock = tps6586x_irq_sync_unlock;

	for (i = 0; i < ARRAY_SIZE(tps6586x_irqs); i++) {
		int __irq = i + tps6586x->irq_base;
		irq_set_chip_data(__irq, tps6586x);
		irq_set_chip_and_handler(__irq, &tps6586x->irq_chip,
					 handle_simple_irq);
		irq_set_nested_thread(__irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(__irq, IRQF_VALID);
#endif
	}

	ret = request_threaded_irq(irq, NULL, tps6586x_irq, IRQF_ONESHOT,
				   "tps6586x", tps6586x);

	if (!ret) {
		device_init_wakeup(tps6586x->dev, 1);
		enable_irq_wake(irq);
	}

	return ret;
}

static int __devinit tps6586x_add_subdevs(struct tps6586x *tps6586x,
					  struct tps6586x_platform_data *pdata)
{
	struct tps6586x_subdev_info *subdev;
	struct platform_device *pdev;
	int i, ret = 0;

	for (i = 0; i < pdata->num_subdevs; i++) {
		subdev = &pdata->subdevs[i];

		pdev = platform_device_alloc(subdev->name, subdev->id);
		if (!pdev) {
			ret = -ENOMEM;
			goto failed;
		}

		pdev->dev.parent = tps6586x->dev;
		pdev->dev.platform_data = subdev->platform_data;

		ret = platform_device_add(pdev);
		if (ret) {
			platform_device_put(pdev);
			goto failed;
		}
	}
	return 0;

failed:
	tps6586x_remove_subdevs(tps6586x);
	return ret;
}

static int __devinit tps6586x_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct tps6586x_platform_data *pdata = client->dev.platform_data;
	struct tps6586x *tps6586x;
	int ret;

	if (!pdata) {
		dev_err(&client->dev, "tps6586x requires platform data\n");
		return -ENOTSUPP;
	}

	ret = i2c_smbus_read_byte_data(client, TPS6586X_VERSIONCRC);
	if (ret < 0) {
		dev_err(&client->dev, "Chip ID read failed: %d\n", ret);
		return -EIO;
	}

	dev_info(&client->dev, "VERSIONCRC is %02x\n", ret);

	tps6586x = kzalloc(sizeof(struct tps6586x), GFP_KERNEL);
	if (tps6586x == NULL)
		return -ENOMEM;

	tps6586x->client = client;
	tps6586x->dev = &client->dev;
	i2c_set_clientdata(client, tps6586x);

	mutex_init(&tps6586x->lock);

	if (client->irq) {
		ret = tps6586x_irq_init(tps6586x, client->irq,
					pdata->irq_base);
		if (ret) {
			dev_err(&client->dev, "IRQ init failed: %d\n", ret);
			goto err_irq_init;
		}
	}

	ret = tps6586x_gpio_init(tps6586x, pdata->gpio_base);
	if (ret) {
		dev_err(&client->dev, "GPIO registration failed: %d\n", ret);
		goto err_gpio_init;
	}

	ret = tps6586x_add_subdevs(tps6586x, pdata);
	if (ret) {
		dev_err(&client->dev, "add devices failed: %d\n", ret);
		goto err_add_devs;
	}

	tps6586x_gpio_init(tps6586x, pdata->gpio_base);

	tps6586x_i2c_client = client;
	//ZTE: add by gaoq for force pmu enter into normal state
    tps6586x_force_normal();
	tps6586x_set_vrtc();
	//tps6586x_set_capa();
	//ZTE: add by gaoq for force pmu enter into normal state
	return 0;

err_add_devs:
	if (pdata->gpio_base) {
		ret = gpiochip_remove(&tps6586x->gpio);
		if (ret)
			dev_err(&client->dev, "Can't remove gpio chip: %d\n",
				ret);
	}
err_gpio_init:
	if (client->irq)
		free_irq(client->irq, tps6586x);
err_irq_init:
	kfree(tps6586x);
	return ret;
}

static int __devexit tps6586x_i2c_remove(struct i2c_client *client)
{
	struct tps6586x *tps6586x = i2c_get_clientdata(client);
	struct tps6586x_platform_data *pdata = client->dev.platform_data;
	int ret;

	if (client->irq)
		free_irq(client->irq, tps6586x);

	if (pdata->gpio_base) {
		ret = gpiochip_remove(&tps6586x->gpio);
		if (ret)
			dev_err(&client->dev, "Can't remove gpio chip: %d\n",
				ret);
	}

	tps6586x_remove_subdevs(tps6586x);
	kfree(tps6586x);
	return 0;
}

static const struct i2c_device_id tps6586x_id_table[] = {
	{ "tps6586x", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, tps6586x_id_table);

static struct i2c_driver tps6586x_driver = {
	.driver	= {
		.name	= "tps6586x",
		.owner	= THIS_MODULE,
	},
	.probe		= tps6586x_i2c_probe,
	.remove		= __devexit_p(tps6586x_i2c_remove),
	.id_table	= tps6586x_id_table,
};

static int __init tps6586x_init(void)
{
	return i2c_add_driver(&tps6586x_driver);
}
subsys_initcall(tps6586x_init);

static void __exit tps6586x_exit(void)
{
	i2c_del_driver(&tps6586x_driver);
}
module_exit(tps6586x_exit);

MODULE_DESCRIPTION("TPS6586X core driver");
MODULE_AUTHOR("Mike Rapoport <mike@compulab.co.il>");
MODULE_LICENSE("GPL");
