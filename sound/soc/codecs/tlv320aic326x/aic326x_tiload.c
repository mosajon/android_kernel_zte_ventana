/*
 * linux/sound/soc/codecs/AIC3262_tiload.c
 *
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 *
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * Rev 0.1 	 Tiload support    		Mistral         16-09-2010
 *
 *          The Tiload programming support is added to AIC3262.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/control.h>
#include <linux/slab.h>  //for kmalloc

#include "tlv320aic326x.h"
#include "aic326x_tiload.h"
#include "tlv320aic326x_mini-dsp.h"

/* enable debug prints in the driver */
//#define DEBUG
//#undef DEBUG

//added for tiload debugging 
 
#ifdef DEBUG
#define dprintk(x...) 	printk(x)
#else
#define dprintk(x...)
#endif

#ifdef AIC3262_TiLoad

int aic3262_driver_init(struct snd_soc_codec *codec);
/************** Dynamic aic3262 driver, TI LOAD support  ***************/

static struct cdev *aic3262_cdev;
static int aic3262_major = 0;	/* Dynamic allocation of Mjr No. */
static int aic3262_opened = 0;	/* Dynamic allocation of Mjr No. */
static struct snd_soc_codec *aic3262_codec;
struct class *tiload_class;
static unsigned int magic_num = 0xE0;

/******************************** Debug section *****************************/

/*
 *----------------------------------------------------------------------------
 * Function : tiload_open
 *
 * Purpose  : open method for aic3262-tiload programming interface
 *----------------------------------------------------------------------------
 */
static int tiload_open(struct inode *in, struct file *filp)
{
	dprintk("TiLoad DRIVER : %s\n", __FUNCTION__);
	if (aic3262_opened) {
		printk("%s device is already opened\n", "aic3262");
		printk("%s: only one instance of driver is allowed\n",
		       "aic3262");
		return -1;
	}
	aic3262_opened++;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : tiload_release
 *
 * Purpose  : close method for aic3262_tilaod programming interface
 *----------------------------------------------------------------------------
 */
static int tiload_release(struct inode *in, struct file *filp)
{
	dprintk("TiLoad DRIVER : %s\n", __FUNCTION__);
	aic3262_opened--;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : tiload_read
 *
 * Purpose  : read method for mini dsp programming interface
 *----------------------------------------------------------------------------
 */
static ssize_t tiload_read(struct file *file, char __user * buf,
			   size_t count, loff_t * offset)
{
	static char rd_data[128];
	char reg_addr;
	size_t size;
	#ifdef DEBUG
	int i;
	#endif

	dprintk("TiLoad DRIVER : %s\n", __FUNCTION__);
	if (count > 128) {
		printk("Max 128 bytes can be read\n");
		count = 128;
	}

	/* copy register address from user space  */
	size = copy_from_user(&reg_addr, buf, 1);
	if (size != 0) {
		printk("read: copy_from_user failure\n");
		return -1;
	}
	/* Send the address to device thats is to be read */
        aic3262_codec->hw_series_read(aic3262_codec, reg_addr, rd_data, count);
    #ifdef DEBUG
	printk("[tiload]read reg_addr=%x, count=%x\n", reg_addr, count);
	for (i=0; i<count; i++)
	{
	    printk("[tiload]read rd_data[%d]=%x\n", reg_addr+i, rd_data[i]);
	}
	#endif
	size = count;
	if (copy_to_user(buf, rd_data, size) != 0) {
		dprintk("copy_to_user failed\n");
		return -1;
	}

	return size;
}

/*
 *----------------------------------------------------------------------------
 * Function : tiload_write
 *
 * Purpose  : write method for aic3262_tiload programming interface
 *----------------------------------------------------------------------------
 */
static ssize_t tiload_write(struct file *file, const char __user * buf,
			    size_t count, loff_t * offset)
{
	static char wr_data[128];
	u8 pg_no;
	#ifdef DEBUG
	int i;
	#endif
	struct aic3262_priv *aic3262_private = snd_soc_codec_get_drvdata(aic3262_codec);
	int ret = 0;

	dprintk("TiLoad DRIVER : %s\n", __FUNCTION__);
	/* copy buffer from user space  */
	if (copy_from_user(wr_data, buf, count)) {
		printk("copy_from_user failure\n");
		return -1;
	}
	#ifdef DEBUG
	printk("[tiload] write size = %d\n", (int)count);
	for (i = 0; i < (int)count; i++) {
		printk("[tiload]write wr_data[%d]=%x\n", i, wr_data[i]);
	}
	#endif
	//this lock cannot completely solve the tiload bug, there are two reasons
	//1. on codec driver side: when got the lock, codec driver always asume its on book0, 
	//  but here the book can be changed.
	// 2. on tiload side, page and book may be changed by codec driver
	mutex_lock(&codec_io_mutex);
	if (wr_data[0] == 0) {
		aic3262_change_page(aic3262_codec, wr_data[1]);
		mutex_unlock(&codec_io_mutex);
		return count;
	}
	pg_no = aic3262_private->page_no;

	if ((wr_data[0] == 127) && (pg_no == 0)) {
		aic3262_change_book(aic3262_codec, wr_data[1]);
		mutex_unlock(&codec_io_mutex);
		return count;
	}
	
	if (0 == aic3262_codec->hw_write(aic3262_codec, wr_data, count))
	{
		ret = count;
	}
	else
	{
	    ret = 0;
	}
	mutex_unlock(&codec_io_mutex);
	return ret;
}
#define codec_page_size 128
#define PAGE_NUM 5
/*
 *----------------------------------------------------------------------------
 * Function : aic3262_dump_page
 * Purpose  : Read and display one codec register page, for debugging purpose
 *----------------------------------------------------------------------------
 */
static void aic3262_dump_page(struct snd_soc_codec *codec, u8 page, u8 *buf)
{
	//u8 test_page_array[256];

    mutex_lock(&codec_io_mutex);
	aic3262_change_page(codec, page);
       codec->hw_series_read(codec, 0, buf, codec_page_size);
    mutex_unlock(&codec_io_mutex);

}

static long tiload_ioctl( struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	int num = 0;
	unsigned long flow_size = 0;
	u8 *buf;
	u8 *miniDSP_codes_ptr = NULL; 
	bool enable_miniDSP = true;
	int i,j;
	void __user *argp = (void __user *)arg;
	
	if (_IOC_TYPE(cmd) != aic3262_IOC_MAGIC)
		return -ENOTTY;

	dprintk("TiLoad DRIVER : %s\n", __FUNCTION__);
	switch (cmd) {
	case aic3262_IOMAGICNUM_GET:
		num = copy_to_user(argp, &magic_num, sizeof(int));
		break;
	case aic3262_IOMAGICNUM_SET:
		num = copy_from_user(&magic_num, argp, sizeof(int));
		break;
	case aic3262_IODSP_NUM_GET:
		flow_size = get_current_flow_size(aic3262_codec);
		if (!copy_to_user(argp, &flow_size, sizeof(int)))
			num = sizeof(int);
		break;
	case aic3262_DUMPREG_GET:
		buf = kmalloc(PAGE_NUM*codec_page_size, GFP_KERNEL);
		for (i = 0, j=0; i < PAGE_NUM; i++, j++){
			//if(i == 2)
			//	i= 4;
			aic3262_dump_page(aic3262_codec, i, (u8*)(buf+j*codec_page_size));
		}
		num = copy_to_user(argp, buf, PAGE_NUM*codec_page_size);
		kfree(buf);
		break;
	case aic3262_IODSP1_CODE_GET:
		enable_miniDSP = false;
		//go though, no break;
	case aic3262_IODSP2_CODE_GET:
		flow_size = get_current_flow_size(aic3262_codec);
		miniDSP_codes_ptr = (u8 *)kzalloc(flow_size, GFP_KERNEL);
		if (miniDSP_codes_ptr != NULL)
		{
			get_current_flow_arrays(aic3262_codec, (reg_value *)miniDSP_codes_ptr, enable_miniDSP);
			num = flow_size - copy_to_user(argp, miniDSP_codes_ptr, flow_size);
			kfree(miniDSP_codes_ptr);
		}
		
		break;
		
	}
	return num;
}

/*********** File operations structure for aic3262-tiload programming *************/
static struct file_operations aic3262_fops = {
	.owner = THIS_MODULE,
	.open = tiload_open,
	.release = tiload_release,
	.read = tiload_read,
	.write = tiload_write,
	.unlocked_ioctl = tiload_ioctl,
};

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_driver_init
 *
 * Purpose  : Register a char driver for dynamic aic3262-tiload programming
 *----------------------------------------------------------------------------
 */
int aic3262_driver_init(struct snd_soc_codec *codec)
{
	int result;

	dev_t dev;
	dprintk("TiLoad DRIVER : %s\n", __FUNCTION__);
	aic3262_codec = codec;

	dprintk("allocating dynamic major number\n");

	result = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	if (result < 0) {
		dprintk("cannot allocate major number %d\n", aic3262_major);
		return result;
	}
	
	aic3262_cdev = cdev_alloc();
	cdev_init(aic3262_cdev, &aic3262_fops);
	aic3262_cdev->owner = THIS_MODULE;
	aic3262_cdev->ops = &aic3262_fops;

	if (cdev_add(aic3262_cdev, dev, 1) < 0) {
		dprintk("aic3262_driver: cdev_add failed \n");
		unregister_chrdev_region(dev, 1);
		aic3262_cdev = NULL;
		return 1;
	}

	tiload_class = class_create(THIS_MODULE, DEVICE_NAME);
	device_create(tiload_class, NULL, MKDEV(MAJOR(dev), 0), NULL, DEVICE_NAME); 
	printk("Registered aic3262 TiLoad driver, Major number: %d \n",
	       aic3262_major);
	//class_device_create(tiload_class, NULL, dev, NULL, DEVICE_NAME, 0);
	return 0;
}

#endif
