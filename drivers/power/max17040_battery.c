/*
 *  max17040_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/max17040_battery.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
//ZTE:add by caixiaoguang 20110819++
#include <linux/irq.h>
#include "../gpio-names.h"
#include <mach/gpio.h>
//ZTE:add by caixiaoguang 20110819--

#define MAX17040_VCELL_MSB	0x02
#define MAX17040_VCELL_LSB	0x03
#define MAX17040_SOC_MSB	0x04
#define MAX17040_SOC_LSB	0x05
#define MAX17040_MODE_MSB	0x06
#define MAX17040_MODE_LSB	0x07
#define MAX17040_VER_MSB	0x08
#define MAX17040_VER_LSB	0x09
#define MAX17040_RCOMP_MSB	0x0C
#define MAX17040_RCOMP_LSB	0x0D
/** ZTE_MODIFY liuzhongzhi added for load customer model, liuzhongzhi0008 */
#define MAX17040_OCV_MSB	0x0E
#define MAX17040_OCV_LSB	0x0F
/** ZTE_MODIFY end */
#define MAX17040_CMD_MSB	0xFE
#define MAX17040_CMD_LSB	0xFF

/* ZTE:origin MAX17040_DELAY is 1000*/
#define MAX17040_DELAY		1000
#define MAX17040_BATTERY_FULL	95
//ZTE:add by caixiaoguang 20110831, 3.5V is too low,3.6V is low
#define MAX17040_BATTERY_LEVEL_TOO_LOW  2800
#define MAX17040_BATTERY_LEVEL_LOW 2880
#define MAX17040_WARN_CHARGE_PERIOD 30000
#define MAX17040_LED_LIGHT_TIME 200

struct max17040_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
#ifdef CONFIG_MACH_CARDHU  
	struct delayed_work		dc_work;    //add by zhanming 20111115
	struct work_struct     low_power_work1;//ZTE:add by caixiaoguang 20111129
	struct wake_lock       bat_lock;   //add by zhanming 20111124
	struct wake_lock       low_power_work1_lock; //ZTE:add by caixiaoguang 20111129
#endif
#ifdef CONFIG_MACH_VENTANA  
	struct work_struct		charge_indicate_work;//ZTE:add by caixiaoguang 20120301
	struct work_struct     low_power_int_work;   //add by zhanming 20120301
	struct work_struct      portable_power_det_work;//ZTE:add by caixiaoguang 20120524
	struct delayed_work		charger_error_filter_work; 
#endif
	struct power_supply		battery;
    struct power_supply		ac;  
    struct power_supply		usb;  
	struct max17040_platform_data	*pdata;

	/* State Of Connect */
	int online;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;
	/** ZTE_MODIFY liuzhongzhi added for load customer model, liuzhongzhi0008 */
	/* State Of load model */
	int load_status;
	/** ZTE_MODIFY end */
};

static struct max17040_chip *charger_chip;  
static int dc_work_init_flag = 0;  
struct wake_lock load_model_wakelock;

#ifdef CONFIG_MACH_CARDHU 
extern int tps6591x_power_off(void);//ZTE:add by caixiaoguang 20111129 
/* revised by zhanming 20111122 ++ */
#define CHARGE_IRQ_DELAY		100   
#define CHARGE_DETECT_USB     0
#define CHARGE_DETECT_AC      1
#define CHARGE_DETECT_OTG     2
#define CHARGE_DETECT_NONE    3
#define POWER_MODE_ERROR      -1
#define POWER_MODE_NORMAL     0
#define POWER_MODE_CHARGE     1
#define DOCK_MODE_ERROR      -1
#define DOCK_MODE_NORMAL     0
#define DOCK_MODE_CHARGE     1

extern bool check_is_otg_mode(void);
extern int fsl_charger_detect(void);

static int get_chg_status(void)
{
    if(0 == dc_work_init_flag)
        return POWER_MODE_ERROR;
    
    if(charger_chip->pdata->charger_online())
        return POWER_MODE_CHARGE;
    else
        return POWER_MODE_NORMAL;
}

static int get_dock_status(void)
{   
    
    if(0 == dc_work_init_flag)
        return DOCK_MODE_ERROR;
    
    if(charger_chip->pdata->dock_online) {
       if(charger_chip->pdata->dock_online())
           return DOCK_MODE_CHARGE;
       else
           return DOCK_MODE_ERROR;
     }
    else
        return DOCK_MODE_NORMAL;
}

static int get_chg_otg_status(void)
{
    if(1 == get_chg_status())
    {
        if(1 == get_dock_status())
        {
            //printk("power status is DOCK!!!\n");
            return CHARGE_DETECT_AC;
        }
    
        if(1 == check_is_otg_mode())
        {
            //printk("power status is OTG!!!\n");
            return CHARGE_DETECT_OTG;
        }
        else
        {
            if(1 == fsl_charger_detect())
            {
                //printk("power status is AC!!!\n");
                return CHARGE_DETECT_AC;
            }
            else
            {
                //printk("power status is USB!!!\n");
                return CHARGE_DETECT_USB;
            }
        }
    }
    else
    {
        //printk("power status is normal!!!\n");
        return CHARGE_DETECT_NONE;
    }
}
/* revised by zhanming 20111122-- */
#endif
#ifdef CONFIG_MACH_VENTANA
static int charge_full_flag = 0;
static int irq_count = 0;
static int charge_error = 0;
static int charge_error_filter_flag = 0;
extern int fsl_charger_detect(void);
//ZTE: add by gao for ac charge setting
extern int set_ac_charge_current(void);
extern void tps6586x_set_charge_timer_fault(int long_time);
extern void tps6586x_set_power_path(int auto_mode);
void tps6586x_set_sys_dppm_threshold(int low_or_high);
struct wake_lock det_port_power_wakelock;
//ZTE: add by gao for ac charge setting
#endif
int max17040_get_vcell(struct i2c_client *client) ; 
int max17040_vcell_filter(void)
{
    int vol_local  = 0;  
    int min_vol = {0};
    int iTemp[6] = {0};  
    int i,j;
        
    for(i=0;i<6;i++)
    {        
        iTemp[i]= max17040_get_vcell(charger_chip->client);      
        msleep(100);
    }

    for(i=0; i<5; i++)
        for(j=i+1; j<6; j++)
        {
            if(iTemp[j] > iTemp[i])
            {
                min_vol   = iTemp[i];
                iTemp[i]  = iTemp[j];
                iTemp[j] = min_vol;
            }
        }
   
    for(i=1; i<5; i++)
    {
        vol_local +=iTemp[i];
    }
    vol_local = vol_local/4;
    
    return vol_local;    
}
#ifdef CONFIG_MACH_CARDHU 
/*ZTE:add by caixiaoguang for low power  interrupt 20111116++*/
int max17040_get_soc(struct i2c_client *client) ;
//static struct work_struct low_power_work2; 
extern int tps6591x_set_comp2_threshold(int threshold );
extern int tps6591x_clr_comp2_int_status(void);
extern int tps6591x_comp2_int_enable(int enable);
static void low_power_work1_fun(void)
{   
    int voltage  = 0;  
    
    wake_lock(&charger_chip->low_power_work1_lock); 
     
    voltage = max17040_get_vcell(charger_chip->client);    
  
    if(voltage > 3450)
    {
        tps6591x_set_comp2_threshold(0);
        msleep(2000);
        tps6591x_comp2_int_enable(1);
    }
    else if(voltage > 3350)
    {
        tps6591x_set_comp2_threshold(1);
        msleep(2000);
        tps6591x_comp2_int_enable(1);
    }
    else
    {
        max17040_get_soc(charger_chip->client);
        printk("low power int happened, vol = %d mv level = %d %%\n",voltage,charger_chip->soc);
        power_supply_changed(&charger_chip->battery); 
        msleep(2000);
        if(voltage<3200)
        {
            tps6591x_power_off();
        }
    } 
    
    wake_unlock(&charger_chip->low_power_work1_lock);    
}
/*ZTE:add by caixiaoguang for low power  interrupt 20111116--*/
#endif

static int max17040_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17040_chip *chip = container_of(psy,
				struct max17040_chip, battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
        case POWER_SUPPLY_PROP_TEMP:
            {
                if(1==charge_error)                    
                    val->intval = 260;
                else
		val->intval = 250;
		break;        
            }
	default:
		return -EINVAL;
	}
	return 0;
}
#ifdef CONFIG_MACH_CARDHU 
/*ZTE:add by caixiaoguang 20111107 for power supply++*/
static int max17040_get_ac_status(void)
{
    if(1 == get_chg_otg_status())
        return 1;
    else
        return 0;
}

static int max17040_get_usb_status(void)
{
    if(0 == get_chg_otg_status())
        return 1;
    else
        return 0;
}
#endif

static int max17040_ac_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
#ifdef CONFIG_MACH_VENTANA
	struct max17040_chip *chip = container_of(psy,
				struct max17040_chip, ac);
#endif

	switch (psp) {	
	case POWER_SUPPLY_PROP_ONLINE:
{
#ifdef CONFIG_MACH_CARDHU  
	val->intval = max17040_get_ac_status();
		break;
#endif
#ifdef CONFIG_MACH_VENTANA
if((1==chip->pdata->charger_online() )&&(1==fsl_charger_detect()))
                                val->intval=1;
                            else
                                val->intval=0;
    		                break;
#endif
}
		
	default:
		return -EINVAL;
	}
	return 0;
}
static int max17040_usb_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
#ifdef CONFIG_MACH_VENTANA
	struct max17040_chip *chip = container_of(psy,
				struct max17040_chip, usb);
#endif

	switch (psp) {	
	case POWER_SUPPLY_PROP_ONLINE:
{
#ifdef CONFIG_MACH_CARDHU 
	val->intval = max17040_get_usb_status();
		break;
#endif
#ifdef CONFIG_MACH_VENTANA
     if((1==chip->pdata->charger_online() ) &&(0==fsl_charger_detect()))
                    val->intval=1;
                else
                    val->intval=0;		
    		    break;
#endif
}
		
	default:
		return -EINVAL;
	}
	return 0;
}

enum supply_type {
	SUPPLY_TYPE_BATTERY = 0,
	SUPPLY_TYPE_AC,
	SUPPLY_TYPE_USB,
};

static char *power_supplied_to[] = {
	"battery",
};
static enum power_supply_property max17040_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	/*battery temp*/
	POWER_SUPPLY_PROP_TEMP,	
};
static enum power_supply_property max17040_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,		
};
static enum power_supply_property max17040_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,		
};
/*ZTE:add by caixiaoguang 20111107 for power supply--*/

/** ZTE_MODIFY liuzhongzhi delete not used function, 20110825 */
#if 0
static int max17040_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
#endif
/** ZTE_MODIFY end */

static int max17040_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

/** ZTE_MODIFY liuzhongzhi added for load customer model, liuzhongzhi0008 */
/*
 * max17040_write_word() - Write a word to max17040 register
 * @client:	The i2c client
 * @reg	  : Register to be write
 * @value : New register value
 */
static int max17040_write_word(struct i2c_client *client, u8 reg, u16 value)
{
	int ret;
	u8 data[2];
	data[0] = (value >> 8) & 0xFF;
	data[1] = value & 0xFF;

	ret = i2c_smbus_write_i2c_block_data(client, reg, 2, data);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

/*
 * max17040_write_word() - Read a word from max17040 register
 * @client:	The i2c client
 * @reg	  : Register to be read
 */
static int max17040_read_word(struct i2c_client *client, u8 reg)
{
	int ret;
	u8 data[2];

	ret = i2c_smbus_read_i2c_block_data(client, reg, 2, data);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ((data[0] << 8) | data[1]);
}

/*
 * max17040_write_block_data() - Write a blcok data to max17040 memory
 * @client:	The i2c client
 * @reg	  : Start memory addr to be write
 * @len   : Block data length
 * @value : Block data addr
 */
static int max17040_write_block_data(struct i2c_client *client, u8 reg, u8 len, const u8 *value)
{
	int ret;
	
	ret = i2c_smbus_write_i2c_block_data(client, reg, len, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
/** ZTE_MODIFY end */

/** ZTE_MODIFY liuzhongzhi should not reset the device so this function is not used, 20110825*/
#if 0
static void max17040_reset(struct i2c_client *client)
{
	max17040_write_reg(client, MAX17040_CMD_MSB, 0x54);
	max17040_write_reg(client, MAX17040_CMD_LSB, 0x00);
}
#endif
/** ZTE_MODIFY end */

//static void max17040_get_vcell(struct i2c_client *client)
int max17040_get_vcell(struct i2c_client *client)   //huxb fixed, 2011.02.06
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_VCELL_MSB);
	lsb = max17040_read_reg(client, MAX17040_VCELL_LSB);

	chip->vcell = (((msb << 4) + (lsb >> 4)) * 1250)/1000; //vcell one unit is 1.25 mv
	//huxb fixed, 2011.02.04
    if(chip->vcell > 4200)
    {
    	  chip->vcell =4200;
    }	
    //end by huxb fixed, 2011.02.04
    //printk("[%s]:CXG get vell=%d !!\n",__FUNCTION__,chip->vcell);
    return chip->vcell;
}

/*ZTE:  add by tengfei for optimization of battery capacity display, 2012.4.12, ++*/

/*********************************************************
*            Real                     Display
*         100~95                  100~95     Cd=Cr;
*         94~30                    94~50       Cd=Cr + 20;
*         29~7                      49~14       Cd=2Cr;
*         6~0                        13~0        Cd=3Cr;
**********************************************************/
#ifdef CONFIG_MACH_VENTANA
int bat_cap_conversion(int cap)
{
	int cap_real =cap;
	int cap_disp = 0;

	if (cap_real > 100)
	{
		cap_disp = 100;
	}
	else if( cap_real >= 30 )
	{
		cap_disp = 50 + ((cap_real - 30)*5)/7;
	}
	else if(cap_real >= 7)
	{
		cap_disp = 14 + ((cap_real - 7)*35)/22;
	}
	else
	{
		cap_disp = (cap_real*13)/6;
	}
	printk("%s: Real(%d) and Dispay(%d)...\n", __func__,cap_real, cap_disp);
	return cap_disp;
}
#endif
/*ZTE:  add by tengfei for optimization of battery capacity display, 2012.4.12, --*/

//static void max17040_get_soc(struct i2c_client *client)
int max17040_get_soc(struct i2c_client *client)  //huxb fixed, 2011.02.06
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	u8 msb;
	u8 lsb;
        #ifdef CONFIG_MACH_VENTANA
		static int soc_last = 100;
        #endif
    int vell_local =0;/*ZTE:add by caixiaoguang for low power  interrupt 20111116*/
        
	msb = max17040_read_reg(client, MAX17040_SOC_MSB);
	lsb = max17040_read_reg(client, MAX17040_SOC_LSB);
	/** ZTE_MODIFY liuzhongzhi added for load customer model, liuzhongzhi0008 */
#if 0
	chip->soc = msb;
#else
	if (chip->pdata->ini_data.bits == 19)
		chip->soc = msb / 2;
	else
	chip->soc = msb;
#ifdef CONFIG_MACH_CARDHU 
/*ZTE:add by caixiaoguang for low power  interrupt 20111116++*/
    if(chip->soc < 5)
    {
        vell_local= max17040_vcell_filter();
        //printk("low power level = %d %%, voltage = %d mv\n",chip->soc,vell_local);
        if(vell_local >= 3400)
        {
            chip->soc = 2;
        }
        else if(vell_local >= 3350)
        {
            chip->soc = 1;
        }
        else 
        {
            chip->soc = 0;
        }
    }
/*ZTE:add by caixiaoguang for low power  interrupt 20111116--*/
 #endif
#ifdef CONFIG_MACH_VENTANA
printk("%s: get the unrevised soc =%d\n", __func__,chip->soc);
    if (chip->soc > 100)
        chip->soc = soc_last;
    else
        soc_last = chip->soc;

    chip->soc = bat_cap_conversion(chip->soc);
    
    if(chip->soc < 5)
    {
        vell_local= max17040_vcell_filter();
        //printk("low power level = %d %%, voltage = %d mv\n",chip->soc,vell_local);
        if(vell_local > 3400)
        {
            chip->soc =1;
        }   
        else 
        {
            chip->soc = 0;
        }
    }
    
    if(charge_full_flag==1)
    {
        chip->soc = 100;
    }
#endif 
	if(chip->soc > 100)
		chip->soc = 100;
#endif
	/** ZTE_MODIFY end */
    return chip->soc;
}

#ifdef CONFIG_MACH_CARDHU 
/*ZTE:add by caixiaoguang for low power  interrupt 20111116++*/
irqreturn_t tps6591x_low_power_irq(int irq, void *data)
{
    int ret;
    
    ret = tps6591x_comp2_int_enable(0);//mask comp2 int    
    ret = tps6591x_clr_comp2_int_status();
    schedule_work(&charger_chip->low_power_work1);  

    return IRQ_HANDLED;
}
/*ZTE:add by caixiaoguang for low power  interrupt 20111116--*/
#endif
static void max17040_get_version(struct i2c_client *client)
{
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_VER_MSB);
	lsb = max17040_read_reg(client, MAX17040_VER_LSB);

	dev_info(&client->dev, "MAX17040 Fuel-Gauge Ver %d%d\n", msb, lsb);
}

/** ZTE_MODIFY liuzhongzhi added for load customer model, liuzhongzhi0008 */
/*
 * max17040_load_model() - load a customer model to max17040
 * @client:	The i2c client
 *
 * It is recommended to periodically reload the model in case
 * an event occurred that might corrupt the custom model.
 * 
 * The model can be refreshed once per hour.
 */
static void max17040_load_model(struct i2c_client *client)
{
	struct max17040_chip *chip;
	struct max17040_ini_data *ini_data;	/* customer model data */
	int rcomp, ocv;		/* saved value of rcomp and ocv register */
	int soc1;			/* saved SOC register high and low byte */
	int i;

	wake_lock(&load_model_wakelock);
	chip = i2c_get_clientdata(client);
	ini_data = &chip->pdata->ini_data;
	dev_info(&client->dev, "@ini_data=%p, title=%s\n", ini_data, ini_data->title);
    
	/* 
	 * 1. Unlock Model Access
	 *
	 * To unlock access to the model the host software must write 0x4A to
 	 * memory location 0x3E and write 0x57 to memory location 0x3F.
	 */
	max17040_write_word(client, 0x3E, 0x4A57);

	/* 
	 * 2. Read RCOMP and OCV
	 */
	rcomp = max17040_read_word(client, MAX17040_RCOMP_MSB);
	ocv = max17040_read_word(client, MAX17040_OCV_MSB);
	dev_info(&client->dev, "rcomp=0x%x, ocv=0x%x\n", rcomp, ocv);

	/* 
	 * 3. Write OCV
	 */
	max17040_write_word(client, MAX17040_OCV_MSB, ini_data->ocvtest);

	/* 
	 * 4. Write RCOMP to a Maximum value of 0xFF00
	 */
	max17040_write_word(client, MAX17040_RCOMP_MSB, 0xFF00);

	/* 
	 * 5. Write Model
	 *
	 * The 64byte model is located between memory locations 0x40 and 0x7F
	 */
	for (i = 0x40; i < 0x80; i += 0x10){
		max17040_write_block_data(client, i, 16, &ini_data->data[i - 0x20]); /* model data from 32byte */
	}

	/* 
	 * 6. Delay at least 150ms
	 *
	 */
	mdelay(200);

	/* 
	 * 7. Write OCV
	 */
	max17040_write_word(client, MAX17040_OCV_MSB, ini_data->ocvtest);

	/* 
	 * 8. Delay between 150ms and 600ms
	 *
	 */
	mdelay(200);

	/* 
	 * 9. Read SOC register and compare to expected result
	 *
	 */
	soc1 = max17040_read_reg(client, MAX17040_SOC_MSB);	
	if ((soc1 >= ini_data->socchecka) && (soc1 <= ini_data->soccheckb)){
		chip->load_status = 1;
	}else{
		chip->load_status = 0;
            printk("%s: CXG get  load_status = 0 so reload model !!\n", __func__);
            /*reload model*/
        	max17040_write_word(client, MAX17040_OCV_MSB, ini_data->ocvtest);
        	max17040_write_word(client, MAX17040_RCOMP_MSB, 0xFF00);
        	for (i = 0x40; i < 0x80; i += 0x10){
        		max17040_write_block_data(client, i, 16, &ini_data->data[i - 0x20]); /* model data from 32byte */
        	}	
        	mdelay(200);	
        	max17040_write_word(client, MAX17040_OCV_MSB, ini_data->ocvtest);
        	mdelay(200);            
	}

	/* 
	 * 10. Restore RCOMP and OCV
	 *
	 */
	max17040_write_word(client, MAX17040_RCOMP_MSB, rcomp);
	max17040_write_word(client, MAX17040_OCV_MSB, ocv);

	/* 
	 * 11. Lock Model Access
	 *
	 * To lock access to the model the host software must write 0x00 to
 	 * memory location 0x3E and write 0x00 to memory location 0x3F.
	 */
	max17040_write_word(client, 0x3E, 0x0000);

	wake_unlock(&load_model_wakelock);
       printk("%s: CXG load model is done!!\n", __func__);
	return;
}

/*
 * show_load_model_status() - Show last load model status
 */
static ssize_t show_load_model_status(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct max17040_chip *chip;
	struct power_supply *psy;
	
	psy = dev_get_drvdata(dev);
	chip = container_of(psy, struct max17040_chip, battery);

	return sprintf(buf, "%u\n", chip->load_status);
}

/*
 * store_load_model() - Force to load model
 */
static ssize_t store_load_model(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct max17040_chip *chip;
	struct power_supply *psy;
	char *after;
	unsigned long num;
	
	psy = dev_get_drvdata(dev);
	chip = container_of(psy, struct max17040_chip, battery);
	
	num = simple_strtoul(buf, &after, 10);
	if (num)
	{
		/* adjust RCOMP register first */
		max17040_write_word(chip->client, MAX17040_RCOMP_MSB, (chip->pdata->ini_data.rcomp0 << 8));
		max17040_load_model(chip->client);
	}

	return count;
}

static struct device_attribute load_model_attr =
	__ATTR(load_model, S_IRUGO | S_IWUSR, show_load_model_status, store_load_model);

/** ZTE_MODIFY end */
/*ZTE:add by caixiaoguang 20120507 for init OCV revise++*/
static void max17040_revise_ocv(struct i2c_client *client)
{
    u8 Volt_MSB, Volt_LSB;   
    int VCell1, VCell2, OCV, Desired_OCV;    
    printk("%s:CXG begin max17040_revise_ocv!!\n ",__func__);      
    /******************************************************************************
    Step 1.  Read First VCELL Sample
    */  
    Volt_MSB = max17040_read_reg(client, MAX17040_VCELL_MSB);
    Volt_LSB = max17040_read_reg(client, MAX17040_VCELL_LSB);
    VCell1 = ((Volt_MSB << 8) + Volt_LSB);
    /******************************************************************************
    Step 2.  Delay 500ms
     
       Delay at least 500ms to ensure a new reading in the VCELL register.
    */
     mdelay(500);
     /******************************************************************************
    Step 3.  Read Second VCELL Sample
    */
    Volt_MSB = max17040_read_reg(client, MAX17040_VCELL_MSB);
    Volt_LSB = max17040_read_reg(client, MAX17040_VCELL_LSB);
    VCell2 = ((Volt_MSB << 8) + Volt_LSB);
    /******************************************************************************
    Step 4.  Unlock Model Access
     
       To unlock access to the model the host software must write 0x4Ah to memory 
       location 0x3E and write 0x57 to memory location 0x3F.  
       Model Access must be unlocked to read and write the OCV register.
    */    
     max17040_write_word(client, 0x3E, 0x4A57);

    /******************************************************************************
    Step 5.  Read OCV
    */
    Volt_MSB = max17040_read_reg(client, MAX17040_OCV_MSB);
    Volt_LSB = max17040_read_reg(client, MAX17040_OCV_LSB);
    OCV = ((Volt_MSB << 8) + Volt_LSB);
     
    /******************************************************************************
    Step 6.  Determine maximum value of VCell1, VCell2, and OCV
    */
    if((VCell1 > VCell2) && (VCell1 > OCV)){
        Desired_OCV = VCell1;
    }
    else if((VCell2 > VCell1) && (VCell2 > OCV)){
        Desired_OCV = VCell2;
    }
    else{
        Desired_OCV = OCV;
    }
     
    /******************************************************************************
    Step 7.  Write OCV
    */    
    max17040_write_word(client, MAX17040_OCV_MSB, Desired_OCV);
    /******************************************************************************
    Step 8. Lock Model Access
    */    
    max17040_write_word(client, 0x3E, 0x0000);
    /******************************************************************************
    Step 9.  Delay 125ms
     
       This delay must be at least 150mS before reading the SOC Register to allow 
       the correct value to be calculated by the device.
    */
    mdelay(125);    
    printk("%s:CXG end max17040_revise_ocv!!\n ",__func__);
}
/*ZTE:add by caixiaoguang 20120507 for init OCV revise--*/

static void max17040_get_online(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata->battery_online)
		chip->online = chip->pdata->battery_online();
	else
		chip->online = 1;
}

static void max17040_get_status(struct i2c_client *client)
{
    struct max17040_chip *chip = i2c_get_clientdata(client);
    
    #ifdef CONFIG_MACH_CARDHU      
    int power_status = -1;
    
    power_status = get_chg_otg_status();
    if((0 == power_status) || (1 == power_status))
    {
        //printk("power status is charge!!!\n");
        if(chip->pdata->charger_settype)
        {
            if(1 == power_status )
                chip->pdata->charger_settype(1);
            else 
                chip->pdata->charger_settype(0);
        }
        
        chip->status = POWER_SUPPLY_STATUS_CHARGING;
    }
    else
    {
        //printk("power status is normal!!!\n");
        chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
        chip->pdata->charger_settype(0);
    	if (chip->soc > MAX17040_BATTERY_FULL)
        {
            printk("power status is full!!!\n");
    		chip->status = POWER_SUPPLY_STATUS_FULL;
        }
    }
    #endif 
    
    #ifdef CONFIG_MACH_VENTANA  
    if (!chip->pdata->charger_online || !chip->pdata->charger_enable) 
    {
		chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
		return;
	}

	if (chip->pdata->charger_online()) 
    {        
		if (chip->pdata->charger_enable())
        {
			chip->status = POWER_SUPPLY_STATUS_CHARGING;
            set_ac_charge_current();
                    if(chip->soc==100)
                        charge_full_flag=1;
        }
		else
			chip->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
    else 
    {
        charge_full_flag=0;
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
        if (chip->soc > MAX17040_BATTERY_FULL)
        {
            printk("power status is full!!!\n");
    		chip->status = POWER_SUPPLY_STATUS_FULL;
        }
	}    
    #endif
        
    dev_dbg(&client->dev, "MAX17040 Fuel-Gauge status: %d\n", chip->status);
}

 #ifdef CONFIG_MACH_CARDHU 
void dc_present_start(void)
{
    if(0 == dc_work_init_flag)
        return;
    
    wake_lock(&charger_chip->bat_lock);

    schedule_delayed_work(&charger_chip->dc_work, CHARGE_IRQ_DELAY);
}
EXPORT_SYMBOL(dc_present_start);
/* add by zhanming 20111115 -- */    


static void dc_present_work(struct work_struct *dc_work)
{
	struct max17040_chip *chip;

	chip = container_of(dc_work, struct max17040_chip, dc_work.work);

    //should first update the status, then inform the userspace
    max17040_get_status(chip->client);
    power_supply_changed(&chip->battery);     
    power_supply_changed(&chip->ac); 
    power_supply_changed(&chip->usb); 
    msleep(2000);
    wake_unlock(&chip->bat_lock);
}
#endif
#ifdef CONFIG_MACH_VENTANA
void dc_present_start(void)
{
    if(1 == dc_work_init_flag)
    {
        schedule_work(&charger_chip->low_power_int_work);
    }
}
EXPORT_SYMBOL(dc_present_start);
/*ZTE add by caixiaoguang for charge indicate++*/
irqreturn_t tps6586x_chg_stat_irq(int irq, void *data)
{
    if(1 == dc_work_init_flag)
    {
        schedule_work(&charger_chip->charge_indicate_work);  
    }

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(tps6586x_chg_stat_irq);

static void charge_indicate_work_func(struct work_struct *work)
{
    printk("CXG  enter  into charge_indicate_work_func!!\n"); 
    #ifdef CONFIG_MACH_VENTANA
    max17040_get_soc(charger_chip->client);
    #endif
    max17040_get_status(charger_chip->client);    
    power_supply_changed(&charger_chip->battery);     
    power_supply_changed(&charger_chip->ac); 
    power_supply_changed(&charger_chip->usb);
}
/*ZTE add by caixiaoguang for charge indicate--*/

static void low_power_int_work_func(struct work_struct *work)
{
    max17040_get_vcell(charger_chip->client);
    if(charger_chip->vcell <= 3400)
    {
        max17040_get_soc(charger_chip->client);
        printk("CXG low power int, level = %d %%,voltage = %d mV!!\n",charger_chip->soc,charger_chip->vcell);
        power_supply_changed(&charger_chip->battery);
    }
}
/*ZTE:add by caixiaoguang 20120526 for bad charger++*/
static void charger_error_filter_work_func(struct work_struct *work)
{
   printk("[%s]CXG wait until 2s after charger and resume power path!\n ",__func__);   
   tps6586x_set_power_path(1);//resume power path auto mode 
   charge_error_filter_flag = 1;
}

static void portable_power_det_work_fun(struct work_struct *work)
{    
    int ret= 0;    
    int charger_present = -1;   
    wake_lock(&det_port_power_wakelock);
    msleep(1000);  //wait for set power path execute    
    do
    {  
        msleep(500);  //inteval        
        charger_present = tps6586x_get_charger_present_status();
        printk("[portable_power_det_work_fun]CXG  get charger_present=%d\n ",charger_present);
        ret = charger_present;
    }
    while(ret);      
    printk("[portable_power_det_work_fun]CXG get ret=%d and resume power path!\n ",ret);     
    charge_error = 0;
    tps6586x_set_power_path(1);//resume power path auto mode
    msleep(200);  
    schedule_work(&charger_chip->charge_indicate_work);  
    irq_count = 0;  
    wake_unlock(&det_port_power_wakelock);  
     return;  
}
void detect_bad_charger(void)
{    
    static  struct timeval begin_time ;
    static  struct timeval end_time;        
    long time_interval_ms = 0;        
	int charger_present = -1; 
	charger_present = tps6586x_get_charger_present_status();
	if(1==charger_present)
		{
    irq_count++;
		    printk("[%s]CXG %d times enter into detect_bad_charger get charger_present=%d!!\n",__func__,irq_count,charger_present);
    if(1 == irq_count)
        {
            do_gettimeofday(&begin_time);
        }
		    else if(3== irq_count)
        {
            do_gettimeofday(&end_time);           
            if((end_time.tv_sec - begin_time.tv_sec) == 0)
                {
                    time_interval_ms  = (end_time.tv_usec - begin_time.tv_usec)/1000;
                }
            else if((end_time.tv_sec - begin_time.tv_sec) > 0)
                {                    
                    time_interval_ms = (end_time.tv_sec - begin_time.tv_sec)*1000+(end_time.tv_usec - begin_time.tv_usec)/1000;
                } 
            else
                {
                   printk("[%s]CXG get end_time earlier than begin_time, so error !!\n",__func__);
                }
            printk("[%s]CXG get end_time.tv_sec =%lu,tv_usec=%lu,begin_time.tv_sec=%lu,tv_usec=%lu,time_interval_ms=%lu\n",__func__,end_time.tv_sec ,end_time.tv_usec,begin_time.tv_sec,begin_time.tv_usec,time_interval_ms);
            if(( time_interval_ms > 0)&&(time_interval_ms < 200))
                {                        
							printk("[%s]CXG %d times detect usb in %lu ms,charge_error_filter_flag=%d,so set power path 0 !!\n",__func__,irq_count,time_interval_ms,charge_error_filter_flag);
							if(0==charge_error_filter_flag)
		                        {
		                            tps6586x_set_power_path(0);
		                            schedule_delayed_work(&charger_chip->charger_error_filter_work, 200);
		                        }
		                    else
		                        {
                    charge_error = 1;                    
                    power_supply_changed(&charger_chip->battery);  
                    tps6586x_set_power_path(0);
                    schedule_work(&charger_chip->portable_power_det_work);                        
                } 
            
		                } 
		            
            irq_count = 0;
        }
		}
	else
		{
			charge_error_filter_flag=0;	
		}
    return ;            
}
/*ZTE:add by caixiaoguang 20120526 for bad charger--*/
#endif
static void max17040_work(struct work_struct *work)
{
	struct max17040_chip *chip;
       static int load_time = MAX17040_DELAY;	/** ZTE_MODIFY liuzhongzhi added for load customer model, liuzhongzhi0008 */

	chip = container_of(work, struct max17040_chip, work.work);

	max17040_get_vcell(chip->client);
	max17040_get_soc(chip->client);
    printk("CXG get battery voltage =%d mV, level = %d %%\n",chip->vcell,chip->soc);
    
/** ZTE_MODIFY liuzhongzhi added for load customer model, liuzhongzhi0008 */
	max17040_get_online(chip->client);
	max17040_get_status(chip->client);

	if ((load_time % (3600 * HZ)) == 0)	/* load the model per hour */
	{
		max17040_load_model(chip->client);
		load_time = 0;
	}
	load_time += MAX17040_DELAY;
   	/** ZTE_MODIFY end */
       
    //ZTE, zzq add
    //power supply infact does not change, we just use the Uevent to trigger the user space Uevent observer to update the battery infos 
    power_supply_changed(&chip->battery);     
    power_supply_changed(&chip->ac); 
    power_supply_changed(&chip->usb); 

    schedule_delayed_work(&chip->work, MAX17040_DELAY);
}

#ifdef CONFIG_MACH_CARDHU
extern int zte_get_board_id(void); //add by zhanming for charge 20111101
#endif

static int __devinit max17040_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17040_chip *chip;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);
    /*ZTE:add by caixiaoguang 20111107 for power supply++*/    

	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17040_get_property;
	chip->battery.properties	= max17040_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17040_battery_props);
    
    ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		kfree(chip);
		return ret;
	}
    
    chip->ac.name = "ac",
    chip->ac.type = POWER_SUPPLY_TYPE_MAINS,
    chip->ac.supplied_to = power_supplied_to,
    chip->ac.num_supplicants = ARRAY_SIZE(power_supplied_to),
    chip->ac.properties = max17040_ac_props,
    chip->ac.num_properties = ARRAY_SIZE(max17040_ac_props),
    chip->ac.get_property = max17040_ac_get_property,
    
	ret = power_supply_register(&client->dev, &chip->ac);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		kfree(chip);
		return ret;
	}
    
    chip->usb.name = "usb",
    chip->usb.type = POWER_SUPPLY_TYPE_USB,
    chip->usb.supplied_to = power_supplied_to,
    chip->usb.num_supplicants = ARRAY_SIZE(power_supplied_to),
    chip->usb.properties = max17040_usb_props,
    chip->usb.num_properties = ARRAY_SIZE(max17040_usb_props),
    chip->usb.get_property = max17040_usb_get_property,	
    
    ret = power_supply_register(&client->dev, &chip->usb);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		kfree(chip);
		return ret;
	}
    /*ZTE:add by caixiaoguang 20111107 for power supply--*/

    /** ZTE_MODIFY liuzhongzhi added for load customer model, liuzhongzhi0008 */
	ret = device_create_file(chip->battery.dev, &load_model_attr);
	if (ret) {
		dev_err(&client->dev, "failed: create loadmodel file\n");
	}

	//max17040_reset(client); /* as maxim suggest, should not reset the device */
	max17040_write_word(client, MAX17040_RCOMP_MSB, (chip->pdata->ini_data.rcomp0 << 8));
      max17040_load_model(chip->client);
      #ifdef CONFIG_MACH_VENTANA
      if(0==tps6586x_get_charger_present_status())
        {
      max17040_revise_ocv(chip->client);
        }      
      #endif
	/** ZTE_MODIFY end */   

    //max17040_reset(client);
	max17040_get_version(client);
    
    charger_chip = chip;
    
#ifdef CONFIG_MACH_CARDHU 

	wake_lock_init(&chip->bat_lock, WAKE_LOCK_SUSPEND, "bat_lock");
    
    INIT_DELAYED_WORK(&chip->dc_work, dc_present_work); 
    
/*ZTE:add by caixiaoguang for low power  interrupt 20111116++*/
    #if 1   //closed by zhanming 20111206
    printk("enablel low_power_irq!\n"); 
    enable_irq(752);
    wake_lock_init(&chip->low_power_work1_lock, WAKE_LOCK_SUSPEND, "low_power_work1_lock");  
    INIT_WORK(&chip->low_power_work1, low_power_work1_fun);   
    #endif
/*ZTE:add by caixiaoguang for low power  interrupt 20111116--*/
#endif

#ifdef CONFIG_MACH_VENTANA

    INIT_WORK(&chip->charge_indicate_work, charge_indicate_work_func);/*ZTEadd by caixiaoguang for charge indicate*/
    INIT_WORK(&chip->low_power_int_work, low_power_int_work_func);//add for low power int by zhanming 20120229
    INIT_WORK(&chip->portable_power_det_work, portable_power_det_work_fun);/*ZTEadd by caixiaoguang for charge indicate*/
    INIT_DELAYED_WORK(&chip->charger_error_filter_work, charger_error_filter_work_func);
    tps6586x_set_charge_timer_fault(1);
	tps6586x_set_sys_dppm_threshold(0);
    //ZTE: add by gaoq to check if charger exist then start charge 
    if(tps6586x_get_charger_present_status())
    {   
        printk("detect charger,start charge\n");
        tps6586x_set_input_current_500();
        tps6586x_en_sm2();
        tps686x_en_chg_norm();
    }
    //ZTE: add by gaoq for force pmu enter into normal state
    //printk("test schedule_delayed_work\n");
#endif

    dc_work_init_flag = 1;

	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17040_work);
	schedule_delayed_work(&chip->work, MAX17040_DELAY);

	return 0;
}

static int __devexit max17040_remove(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	device_remove_file(chip->battery.dev, &load_model_attr);	/** ZTE_MODIFY liuzhongzhi added for load customer model, liuzhongzhi0008 */
	power_supply_unregister(&chip->battery);
    power_supply_unregister(&chip->ac);
    power_supply_unregister(&chip->usb);
	cancel_delayed_work(&chip->work);
#ifdef CONFIG_MACH_CARDHU 
    cancel_delayed_work(&chip->dc_work);    //add by zhanming 20111115
#endif
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int max17040_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->work);
	return 0;
}

/*ZTE: add by tengfei for adapter interrupt lost when wake from sleep, 2012.3.24, ++*/
#ifdef CONFIG_MACH_VENTANA  
extern u32 interrupt_status_wake ;
#endif
/*ZTE: add by tengfei for adapter interrupt lost when wake from sleep, 2012.3.24, --*/

static int max17040_resume(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	schedule_delayed_work(&chip->work, HZ/2);

	return 0;
}

#else

#define max17040_suspend NULL
#define max17040_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id max17040_id[] = {
	{ "max17040", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17040_id);

static struct i2c_driver max17040_i2c_driver = {
	.driver	= {
		.name	= "max17040",
	},
	.probe		= max17040_probe,
	.remove		= __devexit_p(max17040_remove),
	.suspend	= max17040_suspend,
	.resume		= max17040_resume,
	.id_table	= max17040_id,
};

static int __init max17040_init(void)
{
	wake_lock_init(&load_model_wakelock, WAKE_LOCK_SUSPEND, "Load_model");
    wake_lock_init(&det_port_power_wakelock, WAKE_LOCK_SUSPEND, "det_port_power");
	return i2c_add_driver(&max17040_i2c_driver);
}
module_init(max17040_init);

static void __exit max17040_exit(void)
{
	wake_lock_destroy(&load_model_wakelock);
    wake_lock_destroy(&det_port_power_wakelock);
	i2c_del_driver(&max17040_i2c_driver);
}
module_exit(max17040_exit);

MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("MAX17040 Fuel Gauge");
MODULE_LICENSE("GPL");
