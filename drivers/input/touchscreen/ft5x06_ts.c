/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION          DATE            AUTHOR        Note
 *    1.0         2010-01-05            WenFS    only support mulititouch   Wenfs 2010-10-01
 *    2.0          2011-09-05                   Duxx      Add touch key, and project setting update, auto CLB command
 *    3.0         2011-09-09            Luowj   
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/ft5x06_ts.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>   
#include <mach/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>

#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#ifdef FT5x0x_EXTEND_FUN
#include <linux/input/ft5x06_ex_fun.h>
#endif

static struct i2c_client *this_client;

#define CONFIG_FT5X0X_MULTITOUCH 1
#if 0
struct ts_event {
    u16 au16_x[CFG_MAX_TOUCH_POINTS];              //x coordinate
    u16 au16_y[CFG_MAX_TOUCH_POINTS];              //y coordinate
    u8  au8_touch_event[CFG_MAX_TOUCH_POINTS];     //touch event:  0 -- down; 1-- contact; 2 -- contact
    u8  au8_finger_id[CFG_MAX_TOUCH_POINTS];       //touch ID
    u16 pressure;
    u8  touch_point;
};

struct ft5x0x_ts_data {
    struct input_dev    *input_dev;
    struct ts_event     event;
    struct work_struct  pen_event_work;
    struct workqueue_struct *ts_workqueue;
//  struct early_suspend    early_suspend;
//  struct mutex device_mode_mutex;   /* Ensures that only one function can specify the Device Mode at a time. */
};
#endif
static int probe_success_flag = 0;
#if POLLING_OR_INTERRUPT
static void ft5x0x_polling(unsigned long data);
static struct timer_list test_timer;
#define POLLING_CYCLE               10
#define POLLING_CHECK_TOUCH         0x01
#define POLLING_CHECK_NOTOUCH       0x00
#endif

#if CFG_SUPPORT_TOUCH_KEY
int tsp_keycodes[CFG_NUMOFKEYS] ={
        KEY_MENU,
        //KEY_HOME,
        KEY_HOMEPAGE,
        KEY_BACK,
        KEY_SEARCH
};
char *tsp_keyname[CFG_NUMOFKEYS] ={
        "Menu",
        "Home",
        "Back",
        "Search"
};
static bool tsp_keystatus[CFG_NUMOFKEYS];
#endif


/***********************************************************************************************
Name    :   ft5x0x_i2c_rxdata 

Input   :   *rxdata
                     *length

Output  :   ret

function    :   

***********************************************************************************************/
int ft5x0x_i2c_Read(char * writebuf, int writelen, char *readbuf, int readlen)
{
    int ret;

    if(writelen > 0)
    {
        struct i2c_msg msgs[] = {
            {
                .addr   = this_client->addr,
                .flags  = 0,
                .len    = writelen,
                .buf    = writebuf,
            },
            {
                .addr   = this_client->addr,
                .flags  = I2C_M_RD,
                .len    = readlen,
                .buf    = readbuf,
            },
        };
        ret = i2c_transfer(this_client->adapter, msgs, 2);
        if (ret < 0)
            DbgPrintk("[FTS] msg %s i2c read error: %d\n", __func__, ret);
    }
    else
    {
        struct i2c_msg msgs[] = {
            {
                .addr   = this_client->addr,
                .flags  = I2C_M_RD,
                .len    = readlen,
                .buf    = readbuf,
            },
        };
        ret = i2c_transfer(this_client->adapter, msgs, 1);
        if (ret < 0)
            DbgPrintk("[FTS] msg %s i2c read error: %d\n", __func__, ret);
    }
    return ret;
}EXPORT_SYMBOL(ft5x0x_i2c_Read);
/***********************************************************************************************
Name    :    ft5x0x_i2c_Write

Input   :   
                     

Output  :0-write success    
        other-error code    
function    :   write data by i2c 

***********************************************************************************************/
int ft5x0x_i2c_Write(char *writebuf, int writelen)
{
    int ret;

    struct i2c_msg msg[] = {
        {
            .addr   = this_client->addr,
            .flags  = 0,
            .len    = writelen,
            .buf    = writebuf,
        },
    };

    ret = i2c_transfer(this_client->adapter, msg, 1);
    if (ret < 0)
        DbgPrintk("[FTS] %s i2c write error: %d\n", __func__, ret);

    return ret;
}EXPORT_SYMBOL(ft5x0x_i2c_Write);

void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}

/***********************************************************************************************
Name    :    

Input   :   
                     

Output  :   

function    :   

***********************************************************************************************/
static void ft5x0x_ts_release(void)
{
    struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
    int i;
  printk("[FTS]:ft5x0x_ts_release\n");

    for(i = 0; i <CFG_NUMOFKEYS; i++ )
    {
            if(tsp_keystatus[i])
            {
                  input_report_key(data->input_dev, tsp_keycodes[i], 0);
      
            //      DbgPrintk("[FTS] %s key is release. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);

                  tsp_keystatus[i] = KEY_RELEASE;      
            }
    }
    input_report_key(data->input_dev, BTN_TOUCH, 0);
    input_mt_sync(data->input_dev);
    input_sync(data->input_dev);
}


//read touch point information
static int ft5x0x_read_Touchdata(void)
{
    struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
    struct ts_event *event = &data->event;
    u8 buf[CFG_POINT_READ_BUF] = {0};
    int ret = -1;
    int i;

    ret = ft5x0x_i2c_Read(buf, 1, buf, CFG_POINT_READ_BUF);
    if (ret < 0) {
        DbgPrintk("[FTS] %s read_data i2c_rxdata failed: %d\n", __func__, ret);
        return ret;
    }
    memset(event, 0, sizeof(struct ts_event));
#if USE_EVENT_POINT
    event->touch_point = buf[2] & 0x07; 
#else
    event->touch_point = buf[2] >>4;
#endif
    if (event->touch_point > CFG_MAX_TOUCH_POINTS)
    {
        event->touch_point = CFG_MAX_TOUCH_POINTS;
    }
//    DbgPrintk("[FTS] %s event->touch_point: %d\n", __func__, event->touch_point);
    for (i = 0; i < event->touch_point; i++)
    {
        event->au16_x[i] = (u16)(buf[3 + 6*i] & 0x0F)<<8 | (u16)buf[4 + 6*i];
        event->au16_y[i] = (u16)(buf[5 + 6*i] & 0x0F)<<8 | (u16)buf[6 + 6*i];
        event->au8_touch_event[i] = buf[3 + 6*i] >> 6;
        event->au8_finger_id[i]   = buf[5 + 6*i] >> 4;
    }

    event->pressure = 200;

    return 0;
}

/***********************************************************************************************
Name    :    

Input   :   
                     

Output  :   

function    :   

***********************************************************************************************/


#if CFG_SUPPORT_TOUCH_KEY
int ft5x0x_touch_key_process(struct input_dev *dev, int x, int y, int touch_event)
{
    int i;
    int key_id;
    DbgPrintk("[FTS] %s x %d  y %d touch_event %d\n", __func__, x, y, touch_event);   
    if ( x < 70)
    {
        key_id = 0;
    }
    else if ( x > 160 && x < 210)
    {
        key_id = 1;
    }
    
    else if ( x > 280 && x < 360)
    {
        key_id = 2;
    }  
    else if (x > 410)
    {
        key_id = 3;
    }
    else
    {
        key_id = 0xf;
    }
    
    for(i = 0; i <CFG_NUMOFKEYS; i++ )
    {
        if( key_id == i )
        {
            if( touch_event == 0)                                  // detect
            {

                if(!tsp_keystatus[i])
                {
                    input_report_key(dev, tsp_keycodes[i], 1);
                    DbgPrintk( "[FTS] %s key is pressed. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
                    tsp_keystatus[i] = KEY_PRESS;
               }
            }
        }
    }
    return 0;
    
}    
#endif

static void ft5x0x_report_value(void)
{
    struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
    struct ts_event *event = &data->event;
    int i;
    //   DbgPrintk("[FTS] ft5x0x_report_value event->touch_point %d.\n", event->touch_point);
    input_report_key(data->input_dev, BTN_TOUCH, 1);

    for (i  = 0; i < event->touch_point; i++)
    {
        // LCD view area
        //   DbgPrintk("[FTS] ft5x0x_report_value %d %d %d %d .\n", event->au16_x[i], event->au16_y[i], event->au8_finger_id[i], event->pressure);
        if (event->au16_x[i] < SCREEN_MAX_X && event->au16_y[i] < SCREEN_MAX_Y)
        {
            input_report_abs(data->input_dev, ABS_X, event->au16_x[i]);
            input_report_abs(data->input_dev, ABS_Y, event->au16_y[i]);
            input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
            input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
            input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->au8_finger_id[i]);



            if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
            {
                input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure);
            }
            else
            {
                input_report_abs(data->input_dev, ABS_MT_PRESSURE, 0);
            }
        }
        else //maybe the touch key area
        {
#if CFG_SUPPORT_TOUCH_KEY
            if (event->au16_y[i] >= (SCREEN_MAX_Y+20))
            {
                ft5x0x_touch_key_process(data->input_dev, event->au16_x[i], event->au16_y[i], event->au8_touch_event[i]);
            }
#endif
        }
        input_mt_sync(data->input_dev);
    }
    input_sync(data->input_dev);
    if (event->touch_point == 0) {
        ft5x0x_ts_release();
    }
}   /*end ft5x0x_report_value*/


/***********************************************************************************************
Name    :    

Input   :   
                     

Output  :   

function    :   

***********************************************************************************************/
static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{

    int ret = -1;

    ret = ft5x0x_read_Touchdata();  
    if (ret == 0 && probe_success_flag==1) { 
        ft5x0x_report_value();
    }
#if POLLING_OR_INTERRUPT
    del_timer(&test_timer);
    add_timer(&test_timer);
#else
    enable_irq(this_client->irq);  
#endif

}

#if POLLING_OR_INTERRUPT
static void ft5x0x_polling(unsigned long data)
{
    struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(this_client);

    if (!work_pending(&ft5x0x_ts->pen_event_work)) {
        queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
    }
}
#endif
/***********************************************************************************************
Name    :    

Input   :   
                     

Output  :   

function    :   

***********************************************************************************************/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
    struct ft5x0x_ts_data *ft5x0x_ts = dev_id;

  	//printk("[FTS] ts ft5x0x_ts_interrupt\n");
    disable_irq_nosync(this_client->irq);
    if (!work_pending(&ft5x0x_ts->pen_event_work)) {
        queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
    }

    return IRQ_HANDLED;
}
static struct mutex ft5x0x_ts_lock;
static int ft5x0x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
        unsigned char uc_reg_value; 
        unsigned char uc_reg_addr; 
        unsigned char  i2c_write_buf[10]; 

    	printk("[FTS] ts suspend\n");
	mutex_lock(&ft5x0x_ts_lock);

     	disable_irq(client->irq); 
     i2c_write_buf[0] = FT5x0x_REG_PW_MODE;
      i2c_write_buf[1] = 3;
	ft5x0x_i2c_Write(i2c_write_buf, 2);    
      uc_reg_addr = FT5x0x_REG_PW_MODE;
        ft5x0x_i2c_Read(&uc_reg_addr, 1, &uc_reg_value, 1);
     DbgPrintk("[FTS] suspend touch PWR_MODE is %d.\n", uc_reg_value);
	mutex_unlock(&ft5x0x_ts_lock);
	return 0;
}
extern int  ft_touch_wake(void);
extern int  ft_touch_reset(void);
static int ft5x0x_ts_resume(struct i2c_client *client)
{
        unsigned char uc_reg_value; 
        unsigned char uc_reg_addr; 
 //       unsigned char  i2c_write_buf[10]; 
    	printk("[FTS] ts resume\n");
	mutex_lock(&ft5x0x_ts_lock);
       ft_touch_wake();     
       uc_reg_addr = FT5x0x_REG_PW_MODE;
        ft5x0x_i2c_Read(&uc_reg_addr, 1, &uc_reg_value, 1);
        DbgPrintk("[FTS] resume touch PWR_MODE is %d.\n", uc_reg_value);   
        if(uc_reg_value != 0)
        {
            ft_touch_reset();
         ft5x0x_i2c_Read(&uc_reg_addr, 1, &uc_reg_value, 1);
        DbgPrintk("[FTS] resume touch read status after reset PWR_MODE is %d.\n", uc_reg_value);   
       }
        enable_irq(client->irq);
	mutex_unlock(&ft5x0x_ts_lock);

      	return 0;
}
static void ft5x0x_ts_early_suspend(struct early_suspend *h)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
       int ret;
    	printk("[FTS] ts ft5x0x_ts_early_suspend\n");
	ft5x0x_ts = container_of(h, struct ft5x0x_ts_data, early_suspend);
    	ret = cancel_work_sync(&ft5x0x_ts->pen_event_work);
    	printk("[FTS] ts ft5x0x_ts_early_suspend cancel_work_sync ret %d\n", ret);

	ft5x0x_ts_suspend(ft5x0x_ts->client, PMSG_SUSPEND);
}

static void ft5x0x_ts_later_resume(struct early_suspend *h)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
    	printk("[FTS] ts ft5x0x_ts_later_resume\n");
	ft5x0x_ts = container_of(h, struct ft5x0x_ts_data, early_suspend);

	ft5x0x_ts_resume(ft5x0x_ts->client);

      queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);

}
extern int prop_add( struct device *dev, char *item, char *value);

/***********************************************************************************************
Name    :    

Input   :   
                     

Output  :   

function    :   

***********************************************************************************************/
static int 
ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ft5x0x_ts_data *ft5x0x_ts;
    struct input_dev *input_dev;
    int err = 0;
    unsigned char uc_reg_value; 
    unsigned char uc_reg_addr;
    char versionbuf[5];
#if CFG_SUPPORT_TOUCH_KEY
        int i;
#endif
      struct Ft5x06_ts_platform_data *pdata = pdata = client->dev.platform_data;

     	if (pdata->init_platform_hw)
		pdata->init_platform_hw();   
    DbgPrintk("[FTS] ft5x0x_ts_probe, driver version is %s.\n", CFG_FTS_CTP_DRIVER_VERSION);

    
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        err = -ENODEV;
        goto exit_check_functionality_failed;
    }

    ft5x0x_ts = kzalloc(sizeof(struct ft5x0x_ts_data), GFP_KERNEL);

    if (!ft5x0x_ts) {
        err = -ENOMEM;
        goto exit_alloc_data_failed;
    }

    this_client = client;
    ft5x0x_ts->client = client;
    i2c_set_clientdata(client, ft5x0x_ts);
    this_client->irq = client->irq;
    DbgPrintk("[FTS] INT irq=%d\n", client->irq);
//  mutex_init(&ft5x0x_ts->device_mode_mutex);
    INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

    ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
    if (!ft5x0x_ts->ts_workqueue) {
        err = -ESRCH;
        goto exit_create_singlethread;
    }
#if POLLING_OR_INTERRUPT
    DbgPrintk("[FTS] Read TouchData by Polling\n");
#else
    err = request_irq(this_client->irq, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING, "ft5x0x_ts", ft5x0x_ts);
    if (err < 0) {
        dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
        goto exit_irq_request_failed;
    }
    disable_irq(this_client->irq);
#endif
    input_dev = input_allocate_device();
    if (!input_dev) {
        err = -ENOMEM;
        dev_err(&client->dev, "failed to allocate input device\n");
        goto exit_input_dev_alloc_failed;
    }
    
    ft5x0x_ts->input_dev = input_dev;

    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_SYN, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(BTN_TOUCH, input_dev->keybit);
    __set_bit(BTN_2, input_dev->keybit);
    input_dev->mscbit[0] = BIT_MASK(MSC_GESTURE);
    /* x-axis acceleration */
    input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
    /* y-axis acceleration */
    input_set_abs_params(input_dev, ABS_Y,  0, SCREEN_MAX_Y, 0, 0);
    input_set_abs_params(input_dev,
            ABS_MT_POSITION_X,  0, SCREEN_MAX_X, 0, 0);
    input_set_abs_params(input_dev,
    ABS_MT_POSITION_Y,  0, SCREEN_MAX_Y, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);   
    input_set_abs_params(input_dev,
            ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
    input_set_abs_params(input_dev,
            ABS_MT_TRACKING_ID, 0, 5, 0, 0);

#if CFG_SUPPORT_TOUCH_KEY
    //setup key code area
    input_dev->keycode = tsp_keycodes;
    for(i = 0; i < CFG_NUMOFKEYS; i++)
    {
        input_set_capability(input_dev, EV_KEY, ((int*)input_dev->keycode)[i]);
        tsp_keystatus[i] = KEY_RELEASE;
    }
#endif
    input_dev->name     = "Ft5x_dev";      //dev_name(&client->dev)
    err = input_register_device(input_dev);
    if (err) {
        dev_err(&client->dev,
                "ft5x0x_ts_probe: failed to register input device: %s\n",
                dev_name(&client->dev));
        goto exit_input_register_device_failed;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    DbgPrintk("[FTS] ==register_early_suspend =\n");
    ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_early_suspend;
    ft5x0x_ts->early_suspend.resume   = ft5x0x_ts_later_resume;
    register_early_suspend(&ft5x0x_ts->early_suspend);
#endif
     mutex_init(&ft5x0x_ts_lock);

     msleep(150);  //make sure CTP already finish startup process
    
    //get some register information
    uc_reg_addr = FT5x0x_REG_FW_VER;
    ft5x0x_i2c_Read(&uc_reg_addr, 1, &uc_reg_value, 1);
    if(uc_reg_value == 0x0a)
    DbgPrintk("[FTS] Firmware version = 0x%x\n", uc_reg_value);
    sprintf(versionbuf, "0x%x", uc_reg_value);
    prop_add(&input_dev->dev, "fw_version", versionbuf);
    uc_reg_addr = FT5x0x_REG_POINT_RATE;
    ft5x0x_i2c_Read(&uc_reg_addr, 1, &uc_reg_value, 1);
    DbgPrintk("[FTS] report rate is %dHz.\n", uc_reg_value * 10);

    uc_reg_addr = FT5X0X_REG_THGROUP;
    ft5x0x_i2c_Read(&uc_reg_addr, 1, &uc_reg_value, 1);
     DbgPrintk("[FTS] touch threshold is %d.\n", uc_reg_value * 4);
     
    uc_reg_addr = FT5x0x_REG_PW_MODE;
    ft5x0x_i2c_Read(&uc_reg_addr, 1, &uc_reg_value, 1);
     DbgPrintk("[FTS] touch PWR_MODE is %d.\n", uc_reg_value);
     #if POLLING_OR_INTERRUPT
    test_timer.function = ft5x06_polling;
    test_timer.expires = jiffies + HZ*2;//POLLING_CYCLE*100; // 100/10
    test_timer.data = 1;
    init_timer(&test_timer);
    add_timer(&test_timer);
#else
    enable_irq(this_client->irq);
#endif
    //you can add sysfs for test
    //ft5x0x_create_sysfs(client);
    probe_success_flag= 1;
    DbgPrintk("[FTS] ==probe over =\n");
    return 0;

exit_input_register_device_failed:
    input_free_device(input_dev);
    
exit_input_dev_alloc_failed:
    free_irq(this_client->irq, ft5x0x_ts);
    
exit_irq_request_failed:
    cancel_work_sync(&ft5x0x_ts->pen_event_work);
    destroy_workqueue(ft5x0x_ts->ts_workqueue);
    
exit_create_singlethread:
    DbgPrintk("[FTS] ==singlethread error =\n");
    i2c_set_clientdata(client, NULL);
    kfree(ft5x0x_ts);
    
exit_alloc_data_failed:
exit_check_functionality_failed:
    return err;
}
/***********************************************************************************************
Name    :    

Input   :   
                     

Output  :   

function    :   

***********************************************************************************************/
static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
    struct ft5x0x_ts_data *ft5x0x_ts;
    DbgPrintk("[FTS] ==ft5x0x_ts_remove=\n");
    ft5x0x_ts = i2c_get_clientdata(client);
    //unregister_early_suspend(&ft5x0x_ts->early_suspend);
    //mutex_destroy(&ft5x0x_ts->device_mode_mutex); 
    input_unregister_device(ft5x0x_ts->input_dev);
    kfree(ft5x0x_ts);
    cancel_work_sync(&ft5x0x_ts->pen_event_work);
    destroy_workqueue(ft5x0x_ts->ts_workqueue);
    i2c_set_clientdata(client, NULL); 
#if POLLING_OR_INTERRUPT
    del_timer(&test_timer);
#else
    free_irq(client->irq, ft5x0x_ts);
#endif

    return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
    { FT5X0X_NAME, 0 },{ }
};


MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
    .probe      = ft5x0x_ts_probe,
    .remove     = __devexit_p(ft5x0x_ts_remove),
       .id_table   = ft5x0x_ts_id,
    .driver = {
        .name   = FT5X0X_NAME,
        .owner  = THIS_MODULE,
    },
};

/***********************************************************************************************
Name    :    

Input   :   
                     

Output  :   

function    :   

***********************************************************************************************/
static int __init ft5x0x_ts_init(void)
{
    int ret;
    DbgPrintk("[FTS] ==ft5x0x_ts_init==\n");
    ret = i2c_add_driver(&ft5x0x_ts_driver);
    DbgPrintk("[FTS] ret=%d\n",ret);
    return ret;
}

/***********************************************************************************************
Name    :    

Input   :   
                     

Output  :   

function    :   

***********************************************************************************************/
static void __exit ft5x0x_ts_exit(void)
{
    DbgPrintk("[FTS] ==ft5x0x_ts_exit==\n");
    i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<luowj@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
