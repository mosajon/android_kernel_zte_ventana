/*
 * ov7692.c - ov7692 sensor driver
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * Contributors:
 *      Abhinav Sinha <absinha@nvidia.com>
 *
 * Leverage OV2710.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/**
 * SetMode Sequence for 640x480. Phase 0. Sensor Dependent.
 * This sequence should put sensor in streaming mode for 640x480
 * This is usually given by the FAE or the sensor vendor.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/ov7692.h>
extern int zte_get_board_id(void);

#define OV7692_PRINT 0

struct ov7692_reg {
    u16 addr;
    u16 val;
};

struct ov7692_info {
    int mode;
    struct i2c_client *i2c_client;
    struct ov7692_platform_data *pdata;
};

#define OV7692_TABLE_WAIT_MS 0
#define OV7692_TABLE_END 1
#define OV7692_MAX_RETRIES 3

static struct ov7692_reg  ov7692_brightness_level1[] = {

    {0xd3, 0x10}, 
    {0xd2, 0x04}, 
    {0xdc, 0x08},
    {OV7692_TABLE_END, 0x0000}
   

};
static  struct ov7692_reg  ov7692_brightness_level2[] = {
    {0xd3, 0x00}, 
    {0xd2, 0x04}, 
    {0xdc, 0x00},
    {OV7692_TABLE_END, 0x0000}
     
};
static  struct ov7692_reg  ov7692_brightness_level3[] = {
    {0xd3, 0x10}, 
    {0xd2, 0x04}, 
    {0xdc, 0x00},
    {OV7692_TABLE_END, 0x0000}
};
static  struct ov7692_reg  ov7692_brightness_level4[] = {
    {0xd3, 0x20}, 
    {0xd2, 0x04}, 
    {0xdc, 0x00},
    {OV7692_TABLE_END, 0x0000}
     
};
static  struct ov7692_reg   ov7692_brightness_level5[] = {
  
    {0xd3, 0x30}, 
    {0xd2, 0x04}, 
    {0xdc, 0x00},
    {OV7692_TABLE_END, 0x0000}
};


static struct ov7692_reg mode_640x480[] = {
    {OV7692_TABLE_WAIT_MS, 1},
    {0x12, 0x80},
    // delay 5ms
    {OV7692_TABLE_WAIT_MS, 5},
    {0x0e, 0x08},
    {0x69, 0x52},
    {0x1e, 0xb3},
    {0x48, 0x42},
    {0xff, 0x01},
    {0xae, 0xa0},
    {0xa8, 0x26},
    {0xb4, 0xc0},
    {0xb5, 0x40},
    {0xff, 0x00},
	{0x0c, 0x10},
    {0x62, 0x10},
    {0x12, 0x00},
    {0x17, 0x65},
    {0x18, 0xa4},
    {0x19, 0x0a},
    {0x1a, 0xf6},
    {0x3e, 0x30},
    {0x64, 0x0a},
    {0xff, 0x01},
    {0x80, 0x24},
    {0xb4, 0xc0},
    {0x86, 0x48},
    {0xff, 0x00},
    {0x67, 0x20},
    {0x81, 0x3f},
    {0xcc, 0x02},
    {0xcd, 0x80},
    {0xce, 0x01},
    {0xcf, 0xe0},
    {0xc8, 0x02},
    {0xc9, 0x80},
    {0xca, 0x01},
    {0xcb, 0xe0},
    {0xd0, 0x48},
    {0x82, 0x03},
    {0x70, 0x00},
    {0x71, 0x34},
    {0x74, 0x28},
    {0x75, 0x98},
    {0x76, 0x00},
    {0x77, 0x64},
    {0x78, 0x01},
    {0x79, 0xc2},
    {0x7a, 0x4e},
    {0x7b, 0x1f},
    {0x7c, 0x00},
    {0x11, 0x00},
    {0x20, 0x00},
	{0x21, 0x57},
	{0x50, 0x4d},
	{0x51, 0x40},
	{0x4c, 0x7d},
	{0x41, 0x43},
    {0x80, 0x7f},
	{0x15, 0xb0},
    //lens correction
	{0x85, 0x00}, //;neil update with office light 
    {0x86, 0x00},
	{0x87, 0x00},
	{0x88, 0x00}, 
	{0x89, 0x19}, 
	{0x8a, 0x11}, 
	{0x8b, 0x11}, 
	//lens correction
	{0xbb, 0xac}, //; a17 db file set ab
	{0xbc, 0xae}, //; a17 db file set 84
	{0xbd, 0x02}, //; a17 db file set 27 
	{0xbe, 0x1f}, //; a17 db file set 0e
	{0xbf, 0x93}, //; a17 db file set b8
	{0xc0, 0xb1}, //; a17 db file set c5
	{0xc1, 0x1a}, //; a17 db file set 1e
    //edge/denoise/exposure
	{0xb7, 0x02}, //; a17 db file set 05
	{0xb8, 0x0b}, //; a17 db file set 09
    {0xb9, 0x00},
    {0xba, 0x18},
    // UV adjust
    {0x81, 0xff},                  
	{0x5A, 0x10}, //; a17 db file set 1f 
	{0x5B, 0xa1}, //; a17 db file set 9f
	{0x5C, 0x3a}, //; a17 db file set 69
	{0x5d, 0x20}, //; a17 db file set 42
    // AE target
	{0x24, 0x88},
	{0x25, 0x7a},
	{0x26, 0xc4},
    //gamma
	{0xa3, 0x05}, //; a17 db file set 0b
	{0xa4, 0x10}, //; a17 db file set 15
	{0xa5, 0x25}, //; a17 db file set 29
	{0xa6, 0x46}, //; a17 db file set 4a
	{0xa7, 0x57}, //; a17 db file set 58
	{0xa8, 0x64}, //; a17 db file set 65
	{0xa9, 0x70}, //
	{0xaa, 0x7c}, //; a17 db file set 7b
	{0xab, 0x87}, //; a17 db file set 85
	{0xac, 0x90}, //; a17 db file set 8e
	{0xad, 0x9f}, //; a17 db file set a0
	{0xae, 0xac}, //; a17 db file set b0
	{0xaf, 0xc1}, //; a17 db file set cb
	{0xb0, 0xd5}, //; a17 db file set e1
	{0xb1, 0xe7}, //; a17 db file set f1
	{0xb2, 0x21}, //; a17 db file set 14
    //awb
	{0x8e, 0x92},
	{0x96, 0xff},
	{0x97, 0x00},
	{0x8c, 0x52}, //;neil update with macbeth light resource
    {0x8d, 0x11},
    {0x8e, 0x12},
    {0x8f, 0x19},
    {0x90, 0x50},
    {0x91, 0x20},
	{0x92, 0x94}, 
	{0x93, 0x8e}, 
	{0x94, 0x18}, 
    {0x95, 0x1b},
	{0x96, 0xf0}, 
	{0x97, 0x10}, 
	{0x98, 0x7a}, 
	{0x99, 0x75}, 
	{0x9a, 0x92}, 
	{0x9b, 0x8f}, 
    {0x9c, 0xf0},
    {0x9d, 0xf0},
    {0x9e, 0xf0},
    {0x9f, 0xff},
	{0xa0, 0xc0}, 
	{0xa1, 0xb1}, 
	{0xa2, 0x1c}, 
	{0x14, 0x2b}, //; a17 db file set 3b
	{0xd2, 0x06}, //;zte need enable saturation and contrast function,diable hue
    {0x0e, 0x00},

    // delay 60ms
    {OV7692_TABLE_WAIT_MS, 60},
    {OV7692_TABLE_END,0x00},
};
  
static struct ov7692_reg mode_640x480_for_version2[] = {
    {OV7692_TABLE_WAIT_MS, 1},
	{0x12, 0x80},
	// delay 5ms
    {OV7692_TABLE_WAIT_MS, 5},
    {0x0e, 0x08},
    {0x69, 0x52},
    {0x1e, 0xb3},
    {0x48, 0x42},
    {0xff, 0x01},
    {0xae, 0xa0},
    {0xa8, 0x26},
    {0xb4, 0xc0},
    {0xb5, 0x40},
    {0xff, 0x00},
	{0x0c, 0x10},
    {0x62, 0x10},
    {0x12, 0x00},
    {0x17, 0x65},
    {0x18, 0xa4},
    {0x19, 0x0a},
    {0x1a, 0xf6},
    {0x3e, 0x30},
    {0x64, 0x0a},
    {0xff, 0x01},
    {0x80, 0x24},
    {0xb4, 0xc0},
    {0x86, 0x48},
    {0xff, 0x00},
    {0x67, 0x20},
    {0x81, 0x3f},
    {0xcc, 0x02},
    {0xcd, 0x80},
    {0xce, 0x01},
    {0xcf, 0xe0},
    {0xc8, 0x02},
    {0xc9, 0x80},
    {0xca, 0x01},
    {0xcb, 0xe0},
    {0xd0, 0x48},
    {0x82, 0x03},
    {0x70, 0x00},
    {0x71, 0x34},
    {0x74, 0x28},
    {0x75, 0x98},
    {0x76, 0x00},
    {0x77, 0x64},
    {0x78, 0x01},
    {0x79, 0xc2},
    {0x7a, 0x4e},
    {0x7b, 0x1f},
    {0x7c, 0x00},
    {0x11, 0x00},
    {0x20, 0x00},
	{0x21, 0x57},
	{0x50, 0x4d},
	{0x51, 0x40},
	{0x4c, 0x7d},
	{0x41, 0x43},
    {0x80, 0x7f},
	{0x15, 0xb0},
    //lens correction
	{0x85, 0x00}, //;neil update with office light 
    {0x86, 0x00},
	{0x87, 0x00},
	{0x88, 0x00}, 
	{0x89, 0x19}, 
	{0x8a, 0x11}, 
	{0x8b, 0x11}, 
	//lens correction
	{0xbb, 0xac}, //; a17 db file set ab
	{0xbc, 0xae}, //; a17 db file set 84
	{0xbd, 0x02}, //; a17 db file set 27 
	{0xbe, 0x1f}, //; a17 db file set 0e
	{0xbf, 0x93}, //; a17 db file set b8
	{0xc0, 0xb1}, //; a17 db file set c5
	{0xc1, 0x1a}, //; a17 db file set 1e
    //edge/denoise/exposure
	{0xb7, 0x02}, //; a17 db file set 05
	{0xb8, 0x0b}, //; a17 db file set 09
    {0xb9, 0x00},
    {0xba, 0x18},
    // UV adjust
    {0x81, 0xff},                  
	{0x5A, 0x10}, //; a17 db file set 1f 
	{0x5B, 0xa1}, //; a17 db file set 9f
	{0x5C, 0x3a}, //; a17 db file set 69
	{0x5d, 0x20}, //; a17 db file set 42
	// AE target
	{0x24, 0x88},
	{0x25, 0x7a},
	{0x26, 0xc4},
    //gamma
	{0xa3, 0x05}, //; a17 db file set 0b
	{0xa4, 0x10}, //; a17 db file set 15
	{0xa5, 0x25}, //; a17 db file set 29
	{0xa6, 0x46}, //; a17 db file set 4a
	{0xa7, 0x57}, //; a17 db file set 58
	{0xa8, 0x64}, //; a17 db file set 65
	{0xa9, 0x70}, //
	{0xaa, 0x7c}, //; a17 db file set 7b
	{0xab, 0x87}, //; a17 db file set 85
	{0xac, 0x90}, //; a17 db file set 8e
	{0xad, 0x9f}, //; a17 db file set a0
	{0xae, 0xac}, //; a17 db file set b0
	{0xaf, 0xc1}, //; a17 db file set cb
	{0xb0, 0xd5}, //; a17 db file set e1
	{0xb1, 0xe7}, //; a17 db file set f1
	{0xb2, 0x21}, //; a17 db file set 14
    //awb
	{0x8e, 0x92},
	{0x96, 0xff},
	{0x97, 0x00},
	{0x8c, 0x52}, //;neil update with macbeth light resource
    {0x8d, 0x11},
    {0x8e, 0x12},
    {0x8f, 0x19},
    {0x90, 0x50},
    {0x91, 0x20},
	{0x92, 0x94}, 
	{0x93, 0x8e}, 
	{0x94, 0x18}, 
    {0x95, 0x1b},
	{0x96, 0xf0}, 
	{0x97, 0x10}, 
	{0x98, 0x7a}, 
	{0x99, 0x75}, 
	{0x9a, 0x92}, 
	{0x9b, 0x8f}, 
    {0x9c, 0xf0},
    {0x9d, 0xf0},
    {0x9e, 0xf0},
    {0x9f, 0xff},
	{0xa0, 0xc0}, 
	{0xa1, 0xb1}, 
	{0xa2, 0x1c}, 
	{0x14, 0x2b}, //; a17 db file set 3b
	{0xd2, 0x06}, //;zte need enable saturation and contrast function,diable hue
    {0x0e, 0x00},

    // delay 60ms
    {OV7692_TABLE_WAIT_MS, 60},
    {OV7692_TABLE_END,0x00},
};
enum {
    OV7692_MODE_640x480,
};

static struct ov7692_reg *mode_table[] = {
    [OV7692_MODE_640x480] = mode_640x480,
};
static struct ov7692_reg *mode_table_for_version2[] = {
    [OV7692_MODE_640x480] = mode_640x480_for_version2,
};
static int ov7692_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
        int err;
        struct i2c_msg msg[2];
        unsigned char data[3];

        if (!client->adapter)
            return -ENODEV;
       client->addr = 0x78>>1;
        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].len =1; // //2;
        msg[0].buf = data;

        /* high byte goes out first */
        data[0] =(u8) (addr );
  

        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = 1;
        msg[1].buf = data + 1;

        err = i2c_transfer(client->adapter, msg, 2);

        if (err != 2)
            return -EINVAL;

        *val = data[1];

        return 0;

}

static int ov7692_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
       int err;
    struct i2c_msg msg;
    unsigned char data[2];
    int retry = 0;

    if (!client->adapter)
        return -ENODEV;
        client->addr = 0x78>>1;
     
        data[0] = (u8) (addr);
        data[1] = (u8) (val);
        msg.addr = client->addr;
        msg.flags = 0;
        msg.len = 2;
        msg.buf = data;
       #if OV7692_PRINT
        printk("ov7692: i2c transfer, retrying %x %x\n",
            addr, val);
       #endif
        do {
                err = i2c_transfer(client->adapter, &msg, 1);
                if (err == 1)
                    return 0;
                retry++;
                printk("ov7692: i2c transfer failed, retrying %x %x\n",
                    addr, val);
                msleep(3);
        } while (retry <= OV7692_MAX_RETRIES);

        return err;

       
}

static int ov7692_write_table(struct i2c_client *client,
                const struct ov7692_reg table[],
                const struct ov7692_reg override_list[],
                int num_override_regs)
{
        int err;
        const struct ov7692_reg *next;
        int i;
        u16 val;

        for (next = table; next->addr != OV7692_TABLE_END; next++) {
            if (next->addr == OV7692_TABLE_WAIT_MS) {
                    msleep(next->val);
                    continue;
            }

            val = next->val;

            /* When an override list is passed in, replace the reg */
            /* value to write if the reg is in the list            */
            if (override_list) {
                    for (i = 0; i < num_override_regs; i++) {
                            if (next->addr == override_list[i].addr) {
                                val = override_list[i].val;
                                break;
                            }
                    }
                }

                err = ov7692_write_reg(client, next->addr, val);
                if (err)
                    return err;
            }
            return 0;
            }

static int ov7692_set_mode(struct ov7692_info *info, struct ov7692_mode *mode)
{
        int sensor_mode;
        int err;
        u16 status;
        #if OV7692_PRINT
        pr_info("%s: xres %u yres %u\n", __func__, mode->xres, mode->yres);
        #endif
        if (mode->xres == 640 && mode->yres == 480)
            sensor_mode = OV7692_MODE_640x480;
        else {
            pr_err("%s: invalid resolution supplied to set mode %d %d\n",
                    __func__, mode->xres, mode->yres);
            return -EINVAL;
        }
       msleep(100);

        {
        ov7692_read_reg(info->i2c_client, 0x0a, &status);
      
        }
   
   
         #if OV7692_PRINT
       printk("%s 380_read_reg 1%x\n", __func__, status);
         #endif
         #ifdef  CONFIG_U880F_FLAG
                printk("CONFIG_VIDEO_U880F_OV7692\n");
                err = ov7692_write_table(info->i2c_client, mode_table_for_version2[sensor_mode],
                NULL, 0);
        #else
                printk("else CONFIG_VIDEO_U880F_OV7692\n");
        if(zte_get_board_id()>0)
        {
                       printk("  if(zte_get_board_id()>0)\n");
            err = ov7692_write_table(info->i2c_client, mode_table_for_version2[sensor_mode],
            NULL, 0);
        }
        else
        {
                         printk("else \n");
        err = ov7692_write_table(info->i2c_client, mode_table[sensor_mode],
            NULL, 0);
        }
        #endif
        if (err)
            return err;

        info->mode = sensor_mode;
        return 0;
}

static int ov7692_get_status(struct ov7692_info *info,
            struct ov7692_status *dev_status)
{
    return 0;
}

static long ov7692_ioctl(struct file *file,
        unsigned int cmd, unsigned long arg)
{
        int err;
        struct ov7692_info *info = file->private_data;
         #if OV7692_PRINT
        printk("yuv ov7692_ioctl cmd %d\n",cmd);
         #endif
        switch (cmd) {
        case OV7692_IOCTL_SET_MODE:
        {
            struct ov7692_mode mode;
                #if OV7692_PRINT
                 pr_info("YAOLING ov7692_ioctl case OV7692_IOCTL_SET_MODE\n");
                #endif
            if (copy_from_user(&mode,
                        (const void __user *)arg,
                            sizeof(struct ov7692_mode))) {
                       pr_info("YAOLING ov7692_ioctl case OV7692_IOCTL_SET_MODE ERR\n");     
                    return -EFAULT;
                }

                return ov7692_set_mode(info, &mode);
        }
      
           /* ZTE: add by yaoling for  brightness function 20110928 ++ */
         case OV7692_IOCTL_SET_BRIGHTNESS:
        {
            u8 brightness;
            if (copy_from_user(&brightness,
                (const void __user *)arg,
                sizeof(brightness))) {
                return -EFAULT;
            }
            switch(brightness)
            {
                case ov7692_Brightness_Level1:
                    #if OV7692_PRINT
                    printk("yuv SET_Brightness 0\n");
                    #endif
                    err = ov7692_write_table(info->i2c_client, ov7692_brightness_level1,NULL, 0);
                    if(err)
                        return err;
                    break;
                case ov7692_Brightness_Level2:
                     #if OV7692_PRINT
                    printk("yuv SET_Brightness 1\n");
                     #endif
                    err = ov7692_write_table(info->i2c_client, ov7692_brightness_level2,NULL, 0);
                    if(err)
                        return err;
                    break;
                case ov7692_Brightness_Level3:
                     #if OV7692_PRINT
                    printk("yuv SET_Brightness 2\n");
                     #endif
                    err = ov7692_write_table(info->i2c_client, ov7692_brightness_level3,NULL, 0);
                    if(err)
                        return err;
                    break;
                case ov7692_Brightness_Level4:
                     #if OV7692_PRINT
                    printk("yuv SET_Brightness 3\n");
                     #endif
                    err = ov7692_write_table(info->i2c_client, ov7692_brightness_level4,NULL, 0);
                    if(err)
                        return err;
                    break;
                case ov7692_Brightness_Level5:
                     #if OV7692_PRINT
                    printk("yuv SET_Brightness 4\n");
                     #endif
                    err = ov7692_write_table(info->i2c_client, ov7692_brightness_level5,NULL, 0);
                    if(err)
                        return err;
                    break;
                default:
                    break;
            }
            return 0;
        }
           /* ZTE: add by yaoling for 9v114 brightness function 20110928 -- */
        case OV7692_IOCTL_GET_STATUS:
        {
                return 0;
        }
        default:
             return -EFAULT;


        }
     
}

static struct ov7692_info *info;

static int ov7692_open(struct inode *inode, struct file *file)
{
        struct ov7692_status dev_status;
        int err;
         #if OV7692_PRINT
        pr_info("ov7692_open\n");
         #endif
        file->private_data = info;
        if (info->pdata && info->pdata->power_on)
            info->pdata->power_on();

        dev_status.data = 0;
        dev_status.status = 0;
        err = ov7692_get_status(info, &dev_status);
        return err;
}

int ov7692_release(struct inode *inode, struct file *file)
{
        #if OV7692_PRINT
        pr_info("ov7692_release\n");
        #endif
        if (info->pdata && info->pdata->power_off)
                info->pdata->power_off();
        file->private_data = NULL;
        return 0;
}

static const struct file_operations ov7692_fileops = {
        .owner = THIS_MODULE,
        .open = ov7692_open,
        .unlocked_ioctl = ov7692_ioctl,
        .release = ov7692_release,
};

static struct miscdevice ov7692_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "ov7692",
        .fops = &ov7692_fileops,
};

static int ov7692_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
        int err;
        #if OV7692_PRINT
        pr_info("ov7692: probing sensor.\n");
        #endif

        info = kzalloc(sizeof(struct ov7692_info), GFP_KERNEL);
        if (!info) {
                pr_err("ov7692: Unable to allocate memory!\n");
                return -ENOMEM;
        }

        err = misc_register(&ov7692_device);
        if (err) {
                pr_err("ov7692: Unable to register misc device!\n");
                kfree(info);
                return err;
        }

        info->pdata = client->dev.platform_data;
        info->i2c_client = client;

        i2c_set_clientdata(client, info);
        return 0;
}

static int ov7692_remove(struct i2c_client *client)
{
        struct ov7692_info *info;
        info = i2c_get_clientdata(client);
        misc_deregister(&ov7692_device);
        kfree(info);
        return 0;
}

static const struct i2c_device_id ov7692_id[] = {
        { "ov7692", 0 },
        { },
};

MODULE_DEVICE_TABLE(i2c, ov7692_id);

static struct i2c_driver ov7692_i2c_driver = {
        .driver = {
                .name = "ov7692",
                .owner = THIS_MODULE,
        },
        .probe = ov7692_probe,
        .remove = ov7692_remove,
        .id_table = ov7692_id,
};

static int __init ov7692_init(void)
{
        #if OV7692_PRINT
        pr_info("ov7692 sensor driver loading\n");
        #endif
        return i2c_add_driver(&ov7692_i2c_driver);
}

static void __exit ov7692_exit(void)
{
        i2c_del_driver(&ov7692_i2c_driver);
}

module_init(ov7692_init);
module_exit(ov7692_exit);
