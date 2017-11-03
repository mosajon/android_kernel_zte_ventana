/*
 * adp1655.c - adp1655 flash/torch kernel driver
 *
 * Copyright (C) 2011 NVIDIA Corp.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */


#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <media/adp1655.h>

#define ADP1655_I2C_REG_AMP		0x00
#define ADP1655_I2C_REG_TMR                 0x02
#define ADP1655_I2C_REG_CUR                0x03
#define ADP1655_I2C_REG_FLASH               0x04
/* ZTE: add by yaoling for flash reg val level set 20110902 ++ */
#define FLASH_TIME_LEVEL_ONE   0x0f
#define FLASH_TIME_LEVEL_TWO   0x0e
#define FLASH_TIME_LEVEL_THREE   0x0d

#define FLASH_CURRENT_LEVEL_ONE  0x69
#define FLASH_CURRENT_LEVEL_TWO 0x59
#define FLASH_CURRENT_LEVEL_THREE  0x49
/* ZTE: add by yaoling for flash reg val level set 20110902 ++ */


enum {
	ADP1655_GPIO_ACT,   //  做ADP1655的I2C 使能管脚
	ADP1655_GPIO_EN1,
	ADP1655_GPIO_EN2,
	ADP1655_GPIO_STRB,
	/* ZTE: add by yaoling for ADP1655 */
	ADP1655_GPIO_TORCH,
};

struct adp1655_info {
	struct i2c_client *i2c_client;
	struct adp1655_platform_data *pdata;
};

static struct adp1655_info *info;

static int adp1655_gpio(u8 gpio, u8 val)
{
	int prev_val;

	switch (gpio) {
	case ADP1655_GPIO_ACT:
		if (info->pdata && info->pdata->gpio_act) {
			prev_val = info->pdata->gpio_act(val);
			if (val && (prev_val ^ val))
				mdelay(1); /*delay for device ready*/
			return 0;
		}
		return -1;

	case ADP1655_GPIO_EN1:
		if (info->pdata && info->pdata->gpio_en1) {
			info->pdata->gpio_en1(val);
			return 0;
		}
		return -1;

	case ADP1655_GPIO_EN2:
		if (info->pdata && info->pdata->gpio_en2) {
			info->pdata->gpio_en2(val);
			return 0;
		}
		return -1;

	case ADP1655_GPIO_STRB:
		if (info->pdata && info->pdata->gpio_strb) {
			info->pdata->gpio_strb(val);
			return 0;
		}
        /* ZTE: add by yaoling for adp1655 20110711 ++ */
        case ADP1655_GPIO_TORCH:
        if (info->pdata && info->pdata->gpio_tor) {
        info->pdata->gpio_tor(val);
        return 0;
        }
          /* ZTE: add by yaoling for adp1655 20110711 -- */
	default:
		return -1;
	}
}

static int adp1655_get_reg(u8 addr, u8 *val)
{
	struct i2c_client *client = info->i2c_client;
	struct i2c_msg msg[2];
	unsigned char data[2];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = data;

	data[0] = (u8) (addr);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 1;

	*val = 0;

	if (i2c_transfer(client->adapter, msg, 2) == 2) {
		*val = data[1];
		return 0;
	} else {
		return -1;
	}
}

static int adp1655_set_reg(u8 addr, u8 val)
{
	struct i2c_client *client = info->i2c_client;
	struct i2c_msg msg;
	unsigned char data[2];

	data[0] = (u8) (addr);
	data[1] = (u8) (val);
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;

	if (i2c_transfer(client->adapter, &msg, 1) == 1)
		return 0;
	else
		return -1;
}

static long adp1655_ioctl(
	struct file *file,
	unsigned int cmd,
	unsigned long arg)
{
	u8 val = (u8)arg;
	u8 reg;
        u8  time_reg, current_reg ;

	switch (cmd) {
	case ADP1655_IOCTL_MODE_SHUTDOWN:
		//adp1655_gpio(ADP1655_GPIO_ACT, 1);  
             
                adp1655_gpio(ADP1655_GPIO_TORCH, 0);
                
           
		return 0;

	case ADP1655_IOCTL_MODE_STANDBY:
		if (info->pdata->config & 0x01) {	/*0:0 0=I2C, 1=GPIO*/
			adp1655_gpio(ADP1655_GPIO_EN1, 0);
			adp1655_gpio(ADP1655_GPIO_EN2, 0);
			return 0;
		} else 
            {
               /* ZTE modify by yaoling for adp1655 20110711 */
                //return adp1655_set_reg(ADP1655_I2C_REG_AMP, 0x00);
                return adp1655_set_reg(0x04, 0x00);
		}

/* Amp limit for torch, flash, and LED is controlled by external circuitry in
 * GPIO mode.  In I2C mode amp limit is controlled by chip registers and the
 * limit values are in the board-sensors file.
 */
	case ADP1655_IOTCL_MODE_TORCH:
		//adp1655_gpio(ADP1655_GPIO_ACT, 1);
		if (info->pdata->config & 0x01) {	/*0:0 0=I2C, 1=GPIO*/
			adp1655_gpio(ADP1655_GPIO_EN1, 0);
			adp1655_gpio(ADP1655_GPIO_EN2, 1);
			return 0;
		} else 
		{
		        /* ZTE: add by yaoling for ADP1655 20110711  */
                adp1655_gpio(ADP1655_GPIO_TORCH, 1);
			if (val > info->pdata->max_amp_torch)
				val = info->pdata->max_amp_torch;
			val = ((val << 3) & 0xF8);	/*7:3=torch amps*/
			if (!adp1655_get_reg(ADP1655_I2C_REG_AMP, &reg)) {
				val = val | (reg & 0x07); /*shared w/ LED 2:0*/
                            return adp1655_set_reg(ADP1655_I2C_REG_AMP,val);
			} else {
				return -1;
			}
		}

	case ADP1655_IOCTL_MODE_FLASH:
              #if 0
                pr_info("ADP1655_IOCTL_MODE_FLASH\n");
                if (info->pdata->config & 0x01) 
                    {   /*0:0 0=I2C, 1=GPIO*/
			adp1655_gpio(ADP1655_GPIO_EN1, 1);
			adp1655_gpio(ADP1655_GPIO_EN2, 1);
			return 0;
		} 
             else 
                {
                   /* ZTE: modify by yaoling 20110711 ++ */
                        if (!adp1655_get_reg(ADP1655_I2C_REG_FLASH, &reg)) 
                    {    
                            return adp1655_set_reg(ADP1655_I2C_REG_FLASH,
                            0xbb);
                    } 
                     /* ZTE: modify by yaoling 20110711 -- */
                        else 
                        {
				return -1;
			}
		}
                #endif
                if(val == 1)
                {
                    adp1655_gpio(ADP1655_GPIO_STRB, 1);
                }
                else 
                {
                    adp1655_gpio(ADP1655_GPIO_STRB, 0);  
                }
                return 0;

	case ADP1655_IOCTL_MODE_LED:
                //adp1655_gpio(ADP1655_GPIO_ACT, 1);
		if (info->pdata->config & 0x01) {	/*0:0 0=I2C, 1=GPIO*/
			adp1655_gpio(ADP1655_GPIO_EN1, 1);
			adp1655_gpio(ADP1655_GPIO_EN2, 0);
			return 0;
                }
                else 
                {
                   #if 0
			if (val > info->pdata->max_amp_indic)
				val = info->pdata->max_amp_indic;
			val = (val & 0x07);		/*2:0=LED amps*/
			if (!adp1655_get_reg(ADP1655_I2C_REG_AMP, &reg)) {
				val = val | (reg & 0xF8); /*shared w/ 7:3*/
				return adp1655_set_reg(ADP1655_I2C_REG_AMP,
							val);
                    } 
                    #endif
                    if (!adp1655_get_reg(0x04, &reg)) {
                    reg = reg| 0xba; /*shared w/ 7:3*/
                    return adp1655_set_reg(0x04,
                    reg);
                    } 
                    else 
                        {
				return -1;
			}
		}

	case ADP1655_IOCTL_STRB:
           
             pr_info("ADP1655_IOCTL_STRB val=%d\n",val);
           
            if (!info->pdata->config & 0x01)	/*0:0 0=I2C, 1=GPIO*/
            {
                pr_info(" if (!info->pdata->config & 0x01) val=%d\n",val); 
                if(val == 0x01 ||val == 0x02||val == 0x03||val == 0x04)
                {
                    time_reg = FLASH_TIME_LEVEL_ONE;
                    current_reg = FLASH_CURRENT_LEVEL_ONE;
                }
                else if(val == 0x05 ||val == 0x06||val == 0x07||val == 0x08||val ==0x09)
                {
                    time_reg = FLASH_TIME_LEVEL_TWO;
                    current_reg = FLASH_CURRENT_LEVEL_TWO;
                }
                else
                {
                    time_reg = FLASH_TIME_LEVEL_THREE;
                    current_reg = FLASH_CURRENT_LEVEL_THREE; 
                }
                pr_info(" ADP1655_IOCTL_STRB time_reg=%x,current_reg=%x \n",time_reg,current_reg); 
                adp1655_set_reg(ADP1655_I2C_REG_TMR, time_reg);
                adp1655_set_reg(ADP1655_I2C_REG_CUR, current_reg);
			return 0;
            }
		else
            {
                pr_info(" ielse\n"); 
			return -1;
            }

	case ADP1655_IOCTL_TIMER:
		if (!info->pdata->config & 0x01)	/*if I2C mode*/
			//return adp1655_set_reg(ADP1655_I2C_REG_TMR, val);
			return adp1655_set_reg(0x02, val);

	default:
		return -1;
	}
}


static int adp1655_open(struct inode *inode, struct file *file)
{
	int err;
	//u8 reg;
	file->private_data = info;

	pr_info("%s\n", __func__);
	if (info->pdata && info->pdata->init) {
		err = info->pdata->init();
		if (err)
			pr_err("adp1655_open: Board init failed\n");
	}
	return 0;
}

int adp1655_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->exit)
		info->pdata->exit();
	file->private_data = NULL;
	return 0;
}


static const struct file_operations adp1655_fileops = {
	.owner = THIS_MODULE,
	.open = adp1655_open,
	.unlocked_ioctl = adp1655_ioctl,
	.release = adp1655_release,
};

static struct miscdevice adp1655_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "adp1655",
	.fops = &adp1655_fileops,
};

static int adp1655_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int err;
	info = kzalloc(sizeof(struct adp1655_info), GFP_KERNEL);
	if (!info) {
		pr_err("adp1655: Unable to allocate memory!\n");
		return -ENOMEM;
	}
	err = misc_register(&adp1655_device);
	if (err) {
		pr_err("adp1655: Unable to register misc device!\n");
		kfree(info);
		return err;
	}
	info->pdata = client->dev.platform_data;
	info->i2c_client = client;
	i2c_set_clientdata(client, info);
	return 0;
}

static int adp1655_remove(struct i2c_client *client)
{
	info = i2c_get_clientdata(client);
	misc_deregister(&adp1655_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id adp1655_id[] = {
	{ "adp1655", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, adp1655_id);

static struct i2c_driver adp1655_i2c_driver = {
	.driver = {
		.name = "adp1655",
		.owner = THIS_MODULE,
	},
	.probe = adp1655_probe,
	.remove = adp1655_remove,
	.id_table = adp1655_id,
};

static int __init adp1655_init(void)
{
	return i2c_add_driver(&adp1655_i2c_driver);
}

static void __exit adp1655_exit(void)
{
	i2c_del_driver(&adp1655_i2c_driver);
}

module_init(adp1655_init);
module_exit(adp1655_exit);

