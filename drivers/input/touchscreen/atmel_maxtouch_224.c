/*
 *  Atmel maXTouch Touchscreen Controller Driver
 *
 *
 *  Copyright (C) 2010 Atmel Corporation
 *  Copyright (C) 2009 Raphael Derosso Pereira <raphaelpereira@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*zte lipeng10094834 add at 2011.07.11*/
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>
#include <linux/i2c/atmel_maxtouch.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#define	TOUCHSCREEN_TIMEOUT	(msecs_to_jiffies(1000))

/* Returns the start address of object in mXT memory. */
#define	MXT_BASE_ADDR(object_type, mxt)					\
	get_object_address(object_type, 0, mxt->object_table,           \
			   mxt->device_info.num_objs)

#define I2C_RETRY_COUNT		5

/* Maps a report ID to an object type (object type number). */
#define	REPORT_ID_TO_OBJECT(rid, mxt)			\
	(((rid) == 0xff) ? 0 : mxt->rid_map[rid].object)

/* Maps a report ID to an object type (string). */
#define	REPORT_ID_TO_OBJECT_NAME(rid, mxt)			\
	object_type_name[REPORT_ID_TO_OBJECT(rid, mxt)]

/* Returns non-zero if given object is a touch object */
#define IS_TOUCH_OBJECT(object) \
	((object == MXT_GEN_POWERCONFIG_T7) || \
	 (object == MXT_GEN_ACQUIRECONFIG_T8) || \
	 (object == MXT_TOUCH_MULTITOUCHSCREEN_T9) || \
	 (object == MXT_TOUCH_KEYARRAY_T15) || \
	 (object == MXT_SPT_COMMSCONFIG_T18) || \
	 (object == MXT_PROCI_GRIPFACESUPPRESSION_T20) || \
	 (object == MXT_PROCG_NOISESUPPRESSION_T22) ||	\
	 (object == MXT_TOUCH_PROXIMITY_T23) || \
	 (object == MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24) || \
	 (object == MXT_SPT_SELFTEST_T25) || \
	 (object == MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27) || \
	 (object == MXT_SPT_CTECONFIG_T28) ? 1 : 0)

#define mxt_debug(level, ...) \
	do { \
			pr_debug(__VA_ARGS__); \
	} while (0)


static int debug = DEBUG_INFO;
static int comms = 1;

#define	CAL_NODATA		0
#define	CAL_SAMPLED		1
#define	CAL_CONFIRMED		2

module_param(debug, int, 0644);
module_param(comms, int, 0644);

MODULE_PARM_DESC(debug, "Activate debugging output");
MODULE_PARM_DESC(comms, "Select communications mode");

static int8_t cal_check_flag = 1u;
//static uint8_t good_check_flag = 0;
//static unsigned int qt_time_point=0;
//static unsigned int qt_time_diff=0;
//static unsigned int qt_timer_state =0;

/* Device Info descriptor */
/* Parsed from maXTouch "Id information" inside device */
struct mxt_device_info {
	u8   family_id;
	u8   variant_id;
	u8   major;
	u8   minor;
	u8   build;
	u8   num_objs;
	u8   x_size;
	u8   y_size;
	char family_name[16];	 /* Family name */
	char variant_name[16];    /* Variant name */
	u16  num_nodes;           /* Number of sensor nodes */
};

/* object descriptor table, parsed from maXTouch "object table" */
struct mxt_object {
	u16 chip_addr;
	u8  type;
	u8  size;
	u8  instances;
	u8  num_report_ids;
};


/* Mapping from report id to object type and instance */
struct report_id_map {
	u8  object;
	u8  instance;
/*
 * This is the first report ID belonging to object. It enables us to
 * find out easily the touch number: each touch has different report
 * ID (which are assigned to touches in increasing order). By
 * subtracting the first report ID from current, we get the touch
 * number.
 */
	u8  first_rid;
};

struct atmel_finger_data {
	int x;
	int y;
	int w;
	int z;
};

/* Driver datastructure */
struct mxt_data {
	struct i2c_client			*client;
	struct input_dev			*input;
	char						phys_name[32];
	int						irq;
	struct early_suspend		early_suspend;
	u16						last_read_addr;
	struct workqueue_struct	*atmel_wq;
	struct work_struct			work;
	u8						numtouch;
	u8						finger_count;
      struct atmel_finger_data	finger_data[10];
      u16						finger_pressed;
	struct mxt_device_info		device_info;
	u32						info_block_crc;
	u32						configuration_crc;
	u16						report_id_count;
	struct report_id_map		*rid_map;
	struct mxt_object			*object_table;
	u16						msg_proc_addr;
	u8						message_size;
	u16						max_x_val;
	u16						max_y_val;
/* mengzf add for atmel 224 touchscreen calibration begin */
	int                                   pre_data[3];
	uint64_t                           timestamp;
/* mengzf end*/
	int						(*init_hw)(void);
	int						(*exit_hw)(void);
	u8						(*valid_interrupt)(void);
	int 						(*read_chg)(void);
	struct timer_list			timer;
	u8						t9_status;
	uint64_t					init_timestamp;
	u8						init_time_check;
       u8                                      watchdog;

};
static struct mxt_data          *mxt_proc;
struct atmel_config_struct {

        int8_t  config_T6[6];
        int8_t  config_T7[3];
        int8_t  config_T8[10];
        int8_t  config_T9[34];
        int8_t  config_T15[11];
        int8_t  config_T18[2];
        int8_t  config_T20[12];
        int8_t  config_T22[17];
        int8_t  config_T23[15];
        int8_t  config_T24[19];
        int8_t  config_T25[14];
        int8_t  config_T27[7];
        int8_t  config_T28[6];
};




/* modify touchscreen controller parameters*/
struct atmel_config_struct atmel_config_data = {
        .config_T7 = {255, 255, 25},
        .config_T8 = {8, 0, 20, 20, 0, 0, 5, 15},
        .config_T9 = {139, 0, 0, 18, 11, 1, 0, 30, 2, 6,10, 2, 2, 48, 6, 10, 30, 30, 255, 4,31, 3, 0, 0, 0, 0, 64, 0, 0, 0, 18},
        .config_T15 = {3, 0, 11, 3, 1, 2, 1, 30, 2, 0, 0},
        .config_T18 = {4, 0},
        .config_T20 = {25, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
        .config_T22 = {13, 0, 0, 20, 0, 236, 255, 8, 27, 1, 0, 15, 16, 20, 25, 30, 16},
        .config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        .config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        .config_T25 = {0, 0, 224, 46, 88, 27, 224, 46, 88, 27, 0, 0, 0, 0},
        .config_T27 = {0, 0, 0, 0, 0, 0, 0},
        .config_T28 = {0, 0, 2, 16, 16, 30},
};

/*origin config param*/
/*
        .config_T7 = {32, 255, 25},
        .config_T8 = {8, 0, 10, 5, 0, 0, 5, 26},
        .config_T9 = {139, 0, 0, 18, 11, 1, 1, 38, 1, 3, 0, 10, 2, 0, 4, 30, 10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        .config_T15 = {3, 0, 11, 3, 1, 2, 1, 30, 2, 0, 0},
        .config_T18 = {4, 0},
        .config_T20 = {7, 100, 100, 100, 100, 5, 0, 20, 25, 6, 6, 0},
        .config_T22 = {13, 0, 0, 20, 0, 236, 255, 8, 27, 1, 0, 5, 10, 15, 20, 25, 16},
        .config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        .config_T24 = {0, 2, 255, 3, 0, 100, 100, 1, 10, 20, 40, 75, 0, 2, 0, 100, 0, 25, 0},
        .config_T25 = {0, 0, 224, 46, 88, 27, 224, 46, 88, 27, 0, 0, 0, 0},
        .config_T27 = {0, 2, 0, 224, 3, 35, 0},
        .config_T28 = {0, 0, 2, 8, 8, 30},
*/

static const u8	*object_type_name[] = {
	[0]  = "Reserved",
	[5]  = "GEN_MESSAGEPROCESSOR_T5",
	[6]  = "GEN_COMMANDPROCESSOR_T6",
	[7]  = "GEN_POWERCONFIG_T7",
	[8]  = "GEN_ACQUIRECONFIG_T8",
	[9]  = "TOUCH_MULTITOUCHSCREEN_T9",
	[15] = "TOUCH_KEYARRAY_T15",
	[18] = "SPT_COMMSCONFIG_T18",
	[19] = "SPT_GPIOPWM_T19",
	[20] = "PROCI_GRIPFACESUPPRESSION_T20",
	[22] = "PROCG_NOISESUPPRESSION_T22",
	[23] = "TOUCH_PROXIMITY_T23",
	[24] = "PROCI_ONETOUCHGESTUREPROCESSOR_T24",
	[25] = "SPT_SELFTEST_T25",
	[27] = "PROCI_TWOTOUCHGESTUREPROCESSOR_T27",
	[28] = "SPT_CTECONFIG_T28",
	[37] = "DEBUG_DIAGNOSTICS_T37",
	[38] = "SPT_USER_DATA_T38",
};

static inline void report_gesture(int data, struct mxt_data *mxt)
{
	input_event(mxt->input, EV_MSC, MSC_GESTURE, data);
}


static u8 mxt_valid_interrupt_dummy(void)
{
	return 1;
}

/*
 * Reads a block of bytes from given address from mXT chip. If we are
 * reading from message window, and previous read was from message window,
 * there's no need to write the address pointer: the mXT chip will
 * automatically set the address pointer back to message window start.
 */
static int mxt_read_block(struct i2c_client *client,
		   u16 addr,
		   u16 length,
		   u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16	le_addr;
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);

	if (mxt != NULL) {
		if ((mxt->last_read_addr == addr) &&
			(addr == mxt->msg_proc_addr)) {
			if  (i2c_master_recv(client, value, length) == length)
				return length;
			else
				return -EIO;
		} else {
			mxt->last_read_addr = addr;
		}
	}

	mxt_debug(DEBUG_TRACE, "Writing address pointer & reading %d bytes "
		"in on i2c transaction...\n", length);
	le_addr = cpu_to_le16(addr);

	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &le_addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = length;
	msg[1].buf   = (u8 *) value;
	if  (i2c_transfer(adapter, msg, 2) == 2)
		return length;
	else
		return -EIO;

}


/* Writes one byte to given address in mXT chip. */
static int mxt_write_byte(struct i2c_client *client, u16 addr, u8 value)
{
	struct {
		__le16 le_addr;
		u8 data;

	} i2c_byte_transfer;

	struct mxt_data *mxt;

    //   printk(KERN_DEBUG "[TSP]Writing 1 byte to %d...\n",  addr);
	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;
	i2c_byte_transfer.le_addr = cpu_to_le16(addr);
	i2c_byte_transfer.data = value;
	if  (i2c_master_send(client, (u8 *) &i2c_byte_transfer, 3) == 3)
	{
            return 0;
	}
	else
	{
	    return -EIO;
	}
}


/* Writes a block of bytes (max 256) to given address in mXT chip. */
static int mxt_write_block(struct i2c_client *client,
		    u16 addr,
		    u16 length,
		    u8 *value)
{
	int i;
	struct {
		__le16	le_addr;
		u8	data[256];

	} i2c_block_transfer;

	struct mxt_data *mxt;

  //  printk(KERN_DEBUG "Writing %d bytes to %d...", length, addr);
	if (length > 256)
		return -EINVAL;
	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;
	for (i = 0; i < length; i++)
		i2c_block_transfer.data[i] = *value++;
	i2c_block_transfer.le_addr = cpu_to_le16(addr);
	i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
	if (i == (length + 2))
	{
            return length;
	}
	else
	{
	    return -EIO;
	}
}

static u8 get_object_size(struct mxt_data *mxt, u8 object_type)
{
        u8 loop_i;
        for (loop_i = 0; loop_i < mxt->device_info.num_objs; loop_i++)
        {
                if (mxt->object_table[loop_i].type == object_type)
                        return mxt->object_table[loop_i].size;
        }
        return 0;
}

/* Returns object address in mXT chip, or zero if object is not found */
static u16 get_object_address(struct mxt_data *mxt, uint8_t object_type)
{
        uint8_t object_table_index = 0;
        uint8_t address_found = 0;
        uint16_t address = 0;
        uint8_t instance = 0;
        struct mxt_object *object_table = mxt->object_table;
        struct mxt_object *obj;
        int max_objs = mxt->device_info.num_objs;

        while ((object_table_index < max_objs) && !address_found)
        {
                obj = &object_table[object_table_index];
                if (obj->type == object_type)
                {
                        address_found = 1;
                        /* Are there enough instances defined in the FW? */
                        if (obj->instances >= instance)
                        {
                                address = obj->chip_addr +
                                        (obj->size + 1) * instance;
                        }
                        else
                        {
                                return 0;
                        }
                }
                object_table_index++;
        }
        return address;
}

/* Calculates the 24-bit CRC sum. */
static u32 CRC_24(u32 crc, u8 byte1, u8 byte2)
{
	static const u32 crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = ((((u16) byte2) << 8u) | byte1);
	result = ((crc << 1u) ^ data_word);
	if (result & 0x1000000)
		result ^= crcpoly;
	return result;
}

/* Calculates the CRC value for mXT infoblock. */
int calculate_infoblock_crc(u32 *crc_result, u8 *data, int crc_area_size)
{
	u32 crc = 0;
	int i;

	for (i = 0; i < (crc_area_size - 1); i = i + 2)
		crc = CRC_24(crc, *(data + i), *(data + i + 1));
	/* If uneven size, pad with zero */
	if (crc_area_size & 0x0001)
		crc = CRC_24(crc, *(data + i), 0);
	/* Return only 24 bits of CRC. */
	*crc_result = (crc & 0x00FFFFFF);

	return 0;
}

/* Calculates the CRC value for mXT config data. */
int calculate_config_data_crc(u32 *crc_result, u8 *data, int crc_area_size)
{
        u32 crc = 0;
        int i;

        for (i = 0; i < (crc_area_size - 1); i = i + 2)
                crc = CRC_24(crc, *(data + i), *(data + i + 1));
        /* If uneven size, pad with zero */
        if (crc_area_size & 0x0001)
                crc = CRC_24(crc, *(data + i), 0);
        /* Return only 24 bits of CRC. */
        *crc_result = (crc & 0x00FFFFFF);

        return 0;
}

static void mxt_clean_touch_state(struct mxt_data *mxt)
{
        u8 loop_i;
	printk("[TSP] mxt_clean_touch_state\n");

	 //mxt->pre_data[1] = 0;
	 //mxt->pre_data[2] = 0;

        mxt->finger_pressed = 0;
        mxt->finger_count = 0;
        memset(&mxt->finger_data, 0, sizeof(struct atmel_finger_data) * mxt->numtouch);

        if(!mxt->finger_count)
        {
                mxt->finger_pressed = 0;
                return;
         }

        for (loop_i = 0; loop_i < mxt->numtouch; loop_i++)
        {
                if (mxt->finger_pressed & BIT(loop_i))
                {
                        #ifdef ABS_MT_TRACKING_ID
                        printk("[TSP] ABS_MT_TRACKING_ID input_report_abs\n");
                        input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, 0);
                        input_report_key(mxt->input, BTN_TOUCH, 0);
                        #else
                        input_report_key(mxt->input, BTN_TOUCH, 0);
                        #endif

                }
        }

}

/*comented by mengzf for calibration*/
#if 0
static int check_abs_time(void)
{
	unsigned int total_time;

	if (!qt_time_point)
		return 0;

	qt_time_diff = jiffies_to_msecs(jiffies) - qt_time_point;
	if(qt_time_diff >0)
		return 1;
	else
		return 0;

}
#endif

uint8_t calibrate_chip(struct mxt_data *mxt)
{
    //uint8_t data = 1u;
	int ret = 1;
	//uint8_t atchcalst, atchcalsthr;
      uint8_t config_T8_cal_clear[2]	= {5, 45};

	printk(KERN_WARNING"[TSP][%s] \n", __FUNCTION__);
	if(cal_check_flag == 0)
	{
		/* Write temporary acquisition config to chip. */
	      mxt_write_block(mxt->client, get_object_address(mxt, MXT_GEN_ACQUIRECONFIG_T8)+6,
                                            2, config_T8_cal_clear);
	}

	/* send calibration command to the chip */
        /* change calibration suspend settings to zero until calibration confirmed good */
	mxt_write_byte(mxt->client, get_object_address(mxt, MXT_GEN_COMMANDPROCESSOR_T6) +
                          MXT_ADR_T6_CALIBRATE, 1);

	/* set flag to show we must still confirm if calibration was good or bad */
	//cal_check_flag = 1u;
	return ret;
}


void check_chip_calibration(struct mxt_data *mxt)
{
      //uint8_t config_T8_cal[2]		= {5, 15};
	uint8_t data_buffer[100] = { 0 };
	uint8_t try_ctr = 0;
	uint8_t data_byte = 0xF3; /* dianostic command to get touch flags */
	uint16_t diag_address;
	uint8_t tch_ch = 0, atch_ch = 0;
	uint8_t check_mask;
	uint8_t i;
	uint8_t j;
	uint8_t x_line_limit;
	//int ret;

		/* we have had the first touchscreen or face suppression message
		 * after a calibration - check the sensor state and try to confirm if
		 * cal was good or bad */

		/* get touch flags from the chip using the diagnostic object */
		/* write command to command processor to get touch flags - 0xF3 Command required to do this */
		mxt_write_byte(mxt->client, get_object_address(mxt, MXT_GEN_COMMANDPROCESSOR_T6) +
	                          DIAGNOSTIC_OFFSET, data_byte);
		/* get the address of the diagnostic object so we can get the data we need */
		diag_address = get_object_address(mxt,MXT_DEBUG_DIAGNOSTIC_T37);

		msleep(10);

		/* read touch flags from the diagnostic object - clear buffer so the while loop can run first time */
		memset( data_buffer , 0xFF, sizeof( data_buffer ) );

		/* wait for diagnostic object to update */
		while(!((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00)))
		{
			//printk("[TSP] try_ctr = %d\n",try_ctr);
			/* wait for data to be valid  */
			if(try_ctr > 10) //0318 hugh 100-> 10
			{
				/* Failed! */
				printk("[TSP] Diagnostic Data did not update!!\n");
				//qt_timer_state = 0;//0430 hugh
				break;
			}
			msleep(2); //0318 hugh  3-> 2
			try_ctr++; /* timeout counter */
			mxt_read_block(mxt->client,diag_address,2,data_buffer);
			//printk("[TSP] Waiting for diagnostic data to update, try %d\n", try_ctr);
		}


		/* data is ready - read the detection flags */
		mxt_read_block(mxt->client,diag_address,82,data_buffer);
		/* data array is 20 x 16 bits for each set of flags, 2 byte header, 40 bytes for touch flags 40 bytes for antitouch flags*/

		/* count up the channels/bits if we recived the data properly */
		if((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00))
		{
			/* mode 0 : 16 x line, mode 1 : 17 etc etc upto mode 4.*/
			x_line_limit = 16 + 3; // cte_config.mode = 3
			if(x_line_limit > 20)
			{
				/* hard limit at 20 so we don't over-index the array */
				x_line_limit = 20;
			}

			/* double the limit as the array is in bytes not words */
			x_line_limit = x_line_limit << 1;

			/* count the channels and print the flags to the log */
			for(i = 0; i < x_line_limit; i+=2) /* check X lines - data is in words so increment 2 at a time */
			{
				/* print the flags to the log - only really needed for debugging */
				//printk("[TSP] Detect Flags X%d, %x%x, %x%x \n", i>>1,data_buffer[3+i],data_buffer[2+i],data_buffer[43+i],data_buffer[42+i]);

				/* count how many bits set for this row */
				for(j = 0; j < 8; j++)
				{
					/* create a bit mask to check against */
					check_mask = 1 << j;

					/* check detect flags */
					if(data_buffer[2+i] & check_mask)
					{
						tch_ch++;
					}
					if(data_buffer[3+i] & check_mask)
					{
						tch_ch++;
					}

					/* check anti-detect flags */
					if(data_buffer[42+i] & check_mask)
					{
						atch_ch++;
					}
					if(data_buffer[43+i] & check_mask)
					{
						atch_ch++;
					}
				}
			}


			/* print how many channels we counted */
			//printk("[TSP] Flags Counted channels: t:%d a:%d \n", tch_ch, atch_ch);

			/* send page up command so we can detect when data updates next time,
			 * page byte will sit at 1 until we next send F3 command */
			data_byte = 0x01;
			mxt_write_byte(mxt->client, get_object_address(mxt, MXT_GEN_COMMANDPROCESSOR_T6)+DIAGNOSTIC_OFFSET,
			                             data_byte);
			/* process counters and decide if we must re-calibrate or if cal was good */
		#if 0
			if((tch_ch>0) && (atch_ch == 0))  //jwlee change.
			{
				/* cal was good - don't need to check any more */
				if(!check_abs_time())
					qt_time_diff=501;

				if(qt_timer_state == 1) {
					if(qt_time_diff > 500)	{
						printk("[TSP] calibration was good\n");
						cal_check_flag = 0;
						good_check_flag = 0;
						qt_timer_state =0;
						qt_time_point = jiffies_to_msecs(jiffies);
						/* Write normal acquisition config back to the chip. */
						mxt_write_block(mxt->client, get_object_address(mxt, MXT_GEN_ACQUIRECONFIG_T8)+6,
						                            2, config_T8_cal);
					} else  {
						cal_check_flag = 1;
					}
				} else {
					qt_timer_state=1;
					qt_time_point = jiffies_to_msecs(jiffies);
					cal_check_flag=1;
				}
			}
			//else if((tch_ch + CAL_THR /*10*/ ) <= atch_ch)
			else
		#endif
			if(atch_ch >= 2)
			{
				printk( "[TSP] calibration was bad\n");
				/* cal was bad - must recalibrate and check afterwards */
				calibrate_chip(mxt);
				//qt_timer_state=0;
				//qt_time_point = jiffies_to_msecs(jiffies);
			}
			else {
				//printk("[TSP] calibration was not decided yet\n");
				/* we cannot confirm if good or bad - we must wait for next touch  message to confirm */
				cal_check_flag = 1u;
				/* Reset the 100ms timer */
				//qt_timer_state=0;// 1 --> 0
				//qt_time_point = jiffies_to_msecs(jiffies);
				}

		}
}


static void confirm_calibration(struct mxt_data *mxt)
{
	uint8_t ATCH_NOR[2] = {0, 1};
	mxt_write_block(mxt->client, get_object_address(mxt, MXT_GEN_ACQUIRECONFIG_T8)+6,
	                            2, ATCH_NOR);
	mxt->pre_data[0] = CAL_CONFIRMED;
	cal_check_flag = 0;
	//good_check_flag = 0;
	printk( "[TSP] calibration confirm\n");
}

void report_muti_ts_key(u8 *message, struct mxt_data *mxt)
{
	u8 key_status;
	u8 key0_7;
	static u8 key_flag = 0;
	key_status = (message[MXT_MSG_T15_STATUS] >>7)& 0x01;
	key0_7 = message[MXT_MSG_T15_KEY0_7];
	if(key0_7 ==1)
	{
	    input_report_key(mxt->input, KEY_HOME,  1);
	    key_flag = 1;
	}
	else if(key0_7 ==2)
	{
	    input_report_key(mxt->input, KEY_MENU,   1);
	    key_flag = 2;
	}
	else if(key0_7 ==4)
	{
	    input_report_key(mxt->input, KEY_BACK,   1);
	    key_flag = 4;
	}
	else
	{
	    if( key_flag == 1)
	    {
	            input_report_key(mxt->input, KEY_HOME,   0);
	    }
	    if( key_flag == 2)
	    {
	            input_report_key(mxt->input, KEY_MENU,   0);
	    }
	    if( key_flag == 4)
	    {
	            input_report_key(mxt->input, KEY_BACK,   0);

	    }
	    key_flag = 0;
	}
	input_sync(mxt->input);
}

static void msg_process_finger_data(struct atmel_finger_data *fdata, u8  *data)
{
	fdata->x =  data[MXT_MSG_T9_XPOSMSB] * 16+
			((data[MXT_MSG_T9_XYPOSLSB] >> 4) & 0xF);
	fdata->y = data[MXT_MSG_T9_YPOSMSB] * 4 +
			((data[MXT_MSG_T9_XYPOSLSB] >> 2) & 0x3);
	fdata->w = data[MXT_MSG_T9_TCHAREA];
	fdata->z = data[MXT_MSG_T9_TCHAMPLITUDE];
}

/*mengzf comented for unused function*/
#if 0
static void compatible_input_report(struct input_dev *idev,
				struct atmel_finger_data *fdata, uint8_t press, uint8_t last)
{
	if (!press) {
		input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
		input_mt_sync(idev);
	}
	else {
		input_report_abs(idev, ABS_MT_TOUCH_MAJOR, fdata->z);
		input_report_abs(idev, ABS_MT_WIDTH_MAJOR, fdata->w);
		input_report_abs(idev, ABS_MT_POSITION_X, fdata->x);
		input_report_abs(idev, ABS_MT_POSITION_Y, fdata->y);
		input_mt_sync(idev);
	}
}


static void multi_input_report(struct mxt_data *mxt)
{
	u8 loop_i, finger_report = 0;
	for (loop_i = 0; loop_i < mxt->numtouch; loop_i++) {
		if (mxt->finger_pressed & BIT(loop_i)) {
			compatible_input_report(mxt->input, &mxt->finger_data[loop_i],
									1, (mxt->finger_count == ++finger_report));
		}
	}
	input_sync(mxt->input);
}
#endif

static void report_muti_touchs(struct mxt_data *mxt)
{
#ifdef ABS_MT_TRACKING_ID
        u8 loop_i;
        u32 loopcount = 0;
        if(!mxt->finger_count)
        {
      //          input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, 0);
   //             input_sync(mxt->input);
                input_mt_sync(mxt->input);
            input_report_key(mxt->input, BTN_TOUCH, 0);
        }
        else
        {		
            if(cal_check_flag)
				check_chip_calibration(mxt);

                for (loop_i = 0; loop_i < mxt->numtouch; loop_i++)
                {
                        if(loopcount >= mxt->finger_count)
                            break;
                        if (mxt->finger_pressed & BIT(loop_i))
                        {
                        loopcount ++;

                        /*       if(loopcount % 10000 == 0)
              		      {
              		      struct timex txc;
                                 do_gettimeofday(&(txc.time));
              		   //     printk("%s,loop_counti %d  time %d:%d\n",__FUNCTION__,loopcount,txc.time.tv_sec, txc.time.tv_usec);
                                    }*/
 		             //  printk("%s,loop_i %d [%d,%d]:[%d,%d] mxt->t9_status %d\n",__FUNCTION__,loop_i,mxt->finger_data[loop_i].x,mxt->finger_data[loop_i].y, mxt->finger_data[loop_i].z,mxt->finger_data[loop_i].w,mxt->t9_status);
 		                 
                          //     input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, mxt->finger_data[loop_i].z);
                         //       input_report_abs(mxt->input, ABS_MT_WIDTH_MAJOR, mxt->finger_data[loop_i].w);
                                input_report_abs(mxt->input, ABS_MT_PRESSURE, mxt->finger_data[loop_i].z);
                                input_report_abs(mxt->input, ABS_MT_POSITION_X,    mxt->finger_data[loop_i].x);
                                input_report_abs(mxt->input, ABS_MT_POSITION_Y,    mxt->finger_data[loop_i].y);
                                input_mt_sync(mxt->input);
                        }
                }
                input_report_key(mxt->input, BTN_TOUCH, 1);

        }
/* mengzf add for atmel 224 touchscreen calibration ++ */
	if (mxt->t9_status & (MXT_MSGB_T9_RELEASE|MXT_MSGB_T9_SUPPRESS) ) {
		if ((mxt->device_info.family_id == MXT224_FAMILYID) && (mxt->pre_data[0] == CAL_SAMPLED) )
		{
		//	printk("[TSP] finger[%d,%d],pre[%d,%d],cal_check_flag:%d\n",mxt->finger_data[0].x,mxt->finger_data[0].y,mxt->pre_data[1],mxt->pre_data[2],cal_check_flag);
		        if (mxt->finger_count == 0 &&
		                (jiffies_to_msecs(jiffies) > (jiffies_to_msecs(mxt->timestamp) + 200) &&
		                 ((abs(mxt->finger_data[0].y - mxt->pre_data[2]) > 100)
		                || (abs(mxt->finger_data[0].x - mxt->pre_data[1]) > 100))))
		         {
		                        confirm_calibration(mxt);
		         }
		        if (mxt->finger_count)
		        {
		        	#if 1
					calibrate_chip(mxt);
				#else
		                mxt_write_byte(mxt->client,
		                        get_object_address(mxt, MXT_GEN_COMMANDPROCESSOR_T6) +
		                        MXT_ADR_T6_CALIBRATE, 0x55);
				#endif
			}
		        else if ((mxt->finger_pressed != 0x01) && (mxt->pre_data[0] == CAL_SAMPLED))
		        {
		                mxt->pre_data[0] = CAL_NODATA;
		        }
		}
	}
/* mengzf add for atmel 224 touchscreen calibration -- */

#else
        if(mxt->finger_count)
        {
                input_report_key(mxt->input, BTN_TOUCH, 1);
                input_report_abs(mxt->input, ABS_X, mxt->finger_data[0].x);
                input_report_abs(mxt->input, ABS_Y, mxt->finger_data[0].y);
                input_report_abs(mxt->input, ABS_PRESSURE, mxt->finger_data[0].z);
                input_report_abs(mxt->input, ABS_TOOL_WIDTH, mxt->finger_data[0].w);
        }
        else
        {
                input_report_key(mxt->input, BTN_TOUCH, 0);
        }
#endif
        input_sync(mxt->input);
}

static void process_T9_message(u8 *message, struct mxt_data *mxt)
{

	struct	input_dev *input;
	u8  status;
	u8  touch_number;
	u8  report_id;
       // static u32 loopcountA = 0;
	input = mxt->input;
	status = message[MXT_MSG_T9_STATUS];
	report_id = message[0];
	touch_number = message[MXT_MSG_REPORTID] -
	mxt->rid_map[report_id].first_rid;
	if((touch_number >= 0) && (touch_number < mxt->numtouch))
	{
		mxt->t9_status = status;
	        msg_process_finger_data(&mxt->finger_data[touch_number], message);
	        if (status & (MXT_MSGB_T9_RELEASE|MXT_MSGB_T9_SUPPRESS) )
	        {

                   /*     loopcountA ++;
                                if(loopcountA % 10000 == 0)
              		      {
              		      struct timex txc;
                                 do_gettimeofday(&(txc.time));
              		//        printk("%s,A loop_countiA %d  time %d:%d\n",__FUNCTION__,loopcountA,txc.time.tv_sec, txc.time.tv_usec);
                                    }*/
                      //          printk("[TSP] release [x,y] : [%d,%d]\n",mxt->finger_data[touch_number].x,mxt->finger_data[touch_number].y);
			if(mxt->finger_pressed & BIT(touch_number))
			{
			        mxt->finger_count--;
			        mxt->finger_pressed &= ~BIT(touch_number);
			}
/* mengzf add for atmel 224 touchscreen calibration ++ */
#if 0
			if ((mxt->device_info.family_id == MXT224_FAMILYID) && (mxt->pre_data[0] < 2) )
			{
			        if (mxt->finger_count == 0 &&
			                (jiffies > (mxt->timestamp + 10) &&
			                (touch_number == 0 && ((abs(mxt->finger_data[0].y - mxt->pre_data[2]) > 135)
			                || (abs(mxt->finger_data[0].x - mxt->pre_data[1]) > 135)))))
			         {
			                        confirm_calibration(mxt);
			         }
			        if (mxt->finger_count)
			        {
			                mxt_write_byte(mxt->client,
			                        get_object_address(mxt, MXT_GEN_COMMANDPROCESSOR_T6) +
			                        MXT_ADR_T6_CALIBRATE, 0x55);
			        }
			        else if (!mxt->finger_count && mxt->pre_data[0] == 1)
			        {
			                mxt->pre_data[0] = 0;
			        }
			}
#endif
/* mengzf add for atmel 224 touchscreen calibration -- */
	        }
		else if (status & (MXT_MSGB_T9_DETECT|MXT_MSGB_T9_PRESS))
		{
		/*static u32 loopcount = 0;
		                        loopcount ++;
                                if(loopcount % 10000 == 0)
              		      {
              		      struct timex txc;
                                 do_gettimeofday(&(txc.time));
              		   //     printk("%s,B loop_counti %d  time %d:%d\n",__FUNCTION__,loopcount,txc.time.tv_sec, txc.time.tv_usec);
                                    }
                                if((loopcount+loopcountA) % 10000 == 0)
              		      {
              		      struct timex txc;
                                 do_gettimeofday(&(txc.time));
              		     //   printk("%s,B loop_counti (loopcount+loopcountA)%d  time %d:%d\n",__FUNCTION__,(loopcount+loopcountA),txc.time.tv_sec, txc.time.tv_usec);
                                    }	*/
                           //       	printk("[TSP] press [x,y] : [%d,%d]\n",mxt->finger_data[touch_number].x,mxt->finger_data[touch_number].y);
			if (!(mxt->finger_pressed & BIT(touch_number)))
			{
			    mxt->finger_count++;
			    mxt->finger_pressed |= BIT(touch_number);
			}
			/* mengzf add for atmel 224 touchscreen calibration ++ */
			if(!mxt->init_time_check)
				if(jiffies_to_msecs(jiffies) - mxt->init_timestamp >= 30000)
					mxt->init_time_check = 1;
			if(!mxt->init_time_check)
				return;
			if ((mxt->device_info.family_id == MXT224_FAMILYID)&& (mxt->pre_data[0] < CAL_CONFIRMED))
			{
			        if (!mxt->pre_data[0] && mxt->finger_pressed == 0x01)
			        {
			                mxt->pre_data[0] = CAL_SAMPLED;
			                mxt->pre_data[1] = mxt->finger_data[0].x;
			                mxt->pre_data[2] = mxt->finger_data[0].y;
			                mxt->timestamp = jiffies;
			        }
				  if( mxt->finger_pressed != 0x01)
				  	mxt->pre_data[0] = CAL_NODATA;

			}
       /*             input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, mxt->finger_data[touch_number].z);
                    input_report_abs(mxt->input, ABS_MT_WIDTH_MAJOR, mxt->finger_data[touch_number].w);
                    input_report_abs(mxt->input, ABS_MT_POSITION_X,    mxt->finger_data[touch_number].x);
                    input_report_abs(mxt->input, ABS_MT_POSITION_Y,    mxt->finger_data[touch_number].y);
                    input_report_abs(mxt->input, ABS_MT_TRACKING_ID, touch_number);
                    input_mt_sync(mxt->input); */
			//if(cal_check_flag)
			//	check_chip_calibration(mxt);
			//printk("[TSP] finger count: %d   pre data[0]: %d, finger_pressed: 0x%x\n", mxt->finger_count, mxt->pre_data[0],mxt->finger_pressed);
		/* mengzf add for atmel 224 touchscreen calibration -- */
		}
	//printk("[TSP] finger_count=%d,pressed=0x%x,status=0x%x\n",mxt->finger_count,mxt->finger_pressed,status);
	}

	return;
}


int process_message(u8 *message, u8 object, struct mxt_data *mxt)
{
	struct i2c_client *client;
	u8  status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8  event;
	u8  direction;
	u16 distance;
	u8  length;
	u8  report_id;

	client = mxt->client;
	length = mxt->message_size;
	report_id = message[0];
    // printk(KERN_DEBUG"[TSP] Receiving process_message %d\n", object);
	switch (object) {
	case MXT_GEN_COMMANDPROCESSOR_T6:
		status = message[1];
		if (status & MXT_MSGB_T6_COMSERR) {
			printk(
				"[TSP] maXTouch checksum error\n");
		}
		if (status & MXT_MSGB_T6_CFGERR) {
			/*
			 * Configuration error. A proper configuration
			 * needs to be written to chip and backed up. Refer
			 * to protocol document for further info.
			 */
			printk(
				"[TSP] maXTouch configuration error\n");
		}
		if (status & MXT_MSGB_T6_CAL) {
			cal_check_flag = 1u;
			mxt->pre_data[0] = CAL_NODATA;
			mxt_clean_touch_state(mxt);
			/* Calibration in action, no need to react */
			printk(
				"[TSP] maXTouch calibration in progress\n");
		}
		if (status & MXT_MSGB_T6_SIGERR) {
			/*
			 * Signal acquisition error, something is seriously
			 * wrong, not much we can in the driver to correct
			 * this
			 */
			printk(
				"[TSP] maXTouch acquisition error\n");
		}
		if (status & MXT_MSGB_T6_OFL) {
			/*
			 * Cycle overflow, the acquisition is too short.
			 * Can happen temporarily when there's a complex
			 * touch shape on the screen requiring lots of
			 * processing.
			 */
			printk(
				"[TSP] maXTouch cycle overflow\n");
		}
		if (status & MXT_MSGB_T6_RESET) {
			/* Chip has reseted, no need to react. */
			printk(
				"[TSP] maXTouch chip reset\n");
		}
		if (status == 0) {
			/* Chip status back to normal. */
			mxt_clean_touch_state(mxt);
			printk(
				"[TSP] maXTouch status normal\n");
		}
		break;

	case MXT_TOUCH_MULTITOUCHSCREEN_T9:
		//dev_info(&client->dev,"mxt touch multitouchscreen\n");
		process_T9_message(message, mxt);
		break;

	case MXT_SPT_GPIOPWM_T19:
			printk("[TSP] Receiving GPIO message\n");
		break;
      case MXT_TOUCH_KEYARRAY_T15:
		report_muti_ts_key(message, mxt);

		break;
	case MXT_PROCI_GRIPFACESUPPRESSION_T20:
			printk("[TSP] Receiving face suppression msg\n");
		break;

	case MXT_PROCG_NOISESUPPRESSION_T22:
			printk("[TSP] Receiving noise suppression msg\n");
		status = message[MXT_MSG_T22_STATUS];
		if (status & MXT_MSGB_T22_FHCHG) {
				printk("[TSP] maXTouch: Freq changed\n");
		}
		if (status & MXT_MSGB_T22_GCAFERR) {
				printk("[TSP] maXTouch: High noise level\n");
		}
		if (status & MXT_MSGB_T22_FHERR) {
				printk("[TSP] maXTouch: Freq changed - "
					"Noise level too high\n");
		}
		break;

	case MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:
		event = message[MXT_MSG_T24_STATUS] & 0x0F;
		xpos = message[MXT_MSG_T24_XPOSMSB] * 16 +
			((message[MXT_MSG_T24_XYPOSLSB] >> 4) & 0x0F);
		ypos = message[MXT_MSG_T24_YPOSMSB] * 16 +
			((message[MXT_MSG_T24_XYPOSLSB] >> 0) & 0x0F);
		//xpos >>= 2; //zte lipeng10094834 delete
		ypos >>= 2;
		direction = message[MXT_MSG_T24_DIR];
		distance = message[MXT_MSG_T24_DIST] +
			   (message[MXT_MSG_T24_DIST + 1] << 16);

			dev_info(&client->dev,"Receiving one-touch gesture msg event %d\n", event);
		report_gesture((event << 24) | (direction << 16) | distance,
			mxt);
		report_gesture((xpos << 16) | ypos, mxt);
		break;

	case MXT_SPT_SELFTEST_T25:
			printk(
				"[TSP] Receiving Self-Test msg\n");

		if (message[MXT_MSG_T25_STATUS] == MXT_MSGR_T25_OK) {
				dev_info(&client->dev,
					"maXTouch: Self-Test OK\n");

		} else  {
			printk(
				"[TSP] maXTouch: Self-Test Failed [%02x]:"
				"{%02x,%02x,%02x,%02x,%02x}\n",
				message[MXT_MSG_T25_STATUS],
				message[MXT_MSG_T25_STATUS + 0],
				message[MXT_MSG_T25_STATUS + 1],
				message[MXT_MSG_T25_STATUS + 2],
				message[MXT_MSG_T25_STATUS + 3],
				message[MXT_MSG_T25_STATUS + 4]
				);
		}
		break;

	case MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27:

		event = message[MXT_MSG_T27_STATUS] & 0xF0;
		xpos = message[MXT_MSG_T27_XPOSMSB] * 16 +
			((message[MXT_MSG_T27_XYPOSLSB] >> 4) & 0x0F);
		ypos = message[MXT_MSG_T27_YPOSMSB] * 16 +
			((message[MXT_MSG_T27_XYPOSLSB] >> 0) & 0x0F);
		//xpos >>= 2; //zte lipeng10094834 delete
		ypos >>= 2;
		direction = message[MXT_MSG_T27_ANGLE];
		distance = message[MXT_MSG_T27_SEPARATION] +
			   (message[MXT_MSG_T27_SEPARATION + 1] << 16);
			printk("[TSP] Receiving 2-touch gesture message event %d\n", event);
		report_gesture((event << 24) | (direction << 16) | distance,
			mxt);
		report_gesture((xpos << 16) | ypos, mxt);
		break;

	case MXT_SPT_CTECONFIG_T28:
			printk(
				"[TSP] Receiving CTE message...\n");
		status = message[MXT_MSG_T28_STATUS];
		if (status & MXT_MSGB_T28_CHKERR)
			printk(
				"[TSP] maXTouch: Power-Up CRC failure\n");

		break;
	default:
		printk(
			"[TSP] maXTouch: Unknown message!\n");

		break;
	}

	return 0;
}


/*
 * Processes messages when the interrupt line (CHG) is asserted. Keeps
 * reading messages until a message with report ID 0xFF is received,
 * which indicates that there is no more new messages.
 *
 */
static void mxt_worker(struct work_struct *work)
{
	struct	mxt_data *mxt;
	struct	i2c_client *client;

	u8	*message;
	u16	message_length;
	u16	message_addr;
	u8	report_id;
	u8	object;
	int	error = 0;
	int	i, j = 0;


       message = NULL;
	mxt = container_of(work, struct mxt_data, work);
	client = mxt->client;
	message_addr = 	mxt->msg_proc_addr;
	message_length = mxt->message_size;

	if (message_length < 256) {
		message = kmalloc(message_length, GFP_KERNEL);
		if (message == NULL) {
                    enable_irq(mxt->client->irq);
			printk(KERN_WARNING"[TSP] Error allocating memory\n");
			return;
		}
	} else {
	      enable_irq(mxt->client->irq);
		printk(
			KERN_WARNING"[TSP] Message length larger than 256 bytes not supported\n");
		return;
	}
begin_of_mxt_work:
	do {
		/* Read next message, reread on failure. */
		for (i = 1; i < I2C_RETRY_COUNT; i++) {
			error = mxt_read_block(client,
					       message_addr,
					       message_length,
					       message);
			if (error >= 0)
				break;
			printk(
				KERN_WARNING"[TSP] Failure reading maxTouch device\n");
		}
		if (error < 0) {
                     enable_irq(mxt->client->irq);
			kfree(message);
			return;
		}

		report_id = message[0];
              j ++;
	 //      printk(KERN_WARNING"[TSP]mxt work report_id %d j %d\n", report_id, j);
		if ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)) {
			smp_wmb();
			/* Get type of object and process the message */
			object = mxt->rid_map[report_id].object;
			process_message(message, object, mxt);
	               report_muti_touchs(mxt);
		}
            mxt->watchdog = 0;

	}  while (comms ? (gpio_get_value(146) == 0) :
		((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)));
	//      printk(KERN_WARNING"[TSP]end mxt work report_id %d j %d\n", report_id, j);
	if(gpio_get_value(146))
	    enable_irq(mxt->client->irq);
	else
		goto begin_of_mxt_work;
	kfree(message);
}


/*
 * The maXTouch device will signal the host about a new message by asserting
 * the CHG line. This ISR schedules a worker routine to read the message when
 * that happens.
 */
static irqreturn_t mxt_irq_handler(int irq, void *_mxt)
{
	struct mxt_data *mxt = _mxt;

	disable_irq_nosync(mxt->client->irq);
	queue_work(mxt->atmel_wq, &mxt->work);

	return IRQ_HANDLED;
}



/*
 * timer handler,when no touch happen in 1s,read mxt224 once
 * incase a irq is missed
 */
#if 0
static void mxt_timer_readback(struct mxt_data *mxt)
{
	struct	i2c_client *client;

	u8	*message;
	u16	message_length;
	u16	message_addr;
	u8	report_id;
	u8	object;
	int	error = 0;
	int	i;

	message = NULL;
	client = mxt->client;
	message_addr = 	mxt->msg_proc_addr;
	message_length = mxt->message_size;

	if (message_length < 256) {
		message = kmalloc(message_length, GFP_KERNEL);
		if (message == NULL) {
			printk( "[TSP] Error allocating memory\n");
			return;
		}
	} else {
		printk(
			"[TSP] Message length larger than 256 bytes not supported\n");
		return;
	}

	do {
		/* Read next message, reread on failure. */
		for (i = 1; i < I2C_RETRY_COUNT; i++) {
			error = mxt_read_block(client,
					       message_addr,
					       message_length,
					       message);
			if (error >= 0)
				break;
			printk(
				"[TSP] Failure reading maxTouch device\n");
		}
		if (error < 0) {
			kfree(message);
			return;
		}
		#if 1
		report_id = message[0];

		if ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)) {
			smp_wmb();
			/* Get type of object and process the message */
			object = mxt->rid_map[report_id].object;
			process_message(message, object, mxt);
		}
		#endif
	} while (comms ? (mxt->read_chg() == 0) :
		((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)));

	kfree(message);
}
#endif 
static void mxt_timer(unsigned long handle)
{
	struct mxt_data *mxt = (struct mxt_data *) handle;
       
	if(gpio_get_value(146) || mxt_proc==NULL){
		//printk("[TSP] irq level: %d\n",gpio_get_value(146));
		mxt_proc->watchdog = 0;
              mod_timer(&mxt->timer, jiffies + TOUCHSCREEN_TIMEOUT);
		return;
		}

	mxt_proc->watchdog++;
       if(mxt_proc->watchdog == 5)
       {
	      //  printk("[TSP] irq level: 0, %d\n", mxt_proc->watchdog);
               cal_check_flag = 1u;
               disable_irq_nosync(mxt_proc->client->irq);
	        queue_work(mxt_proc->atmel_wq, &mxt_proc->work);          
               mxt_proc->watchdog = 0;
       }
	mod_timer(&mxt->timer, jiffies + TOUCHSCREEN_TIMEOUT);

/*	mxt_timer_readback(mxt);
	mxt_clean_touch_state(mxt);*/	
	return;
}


/******************************************************************************/
/* Initialization of driver                                                   */
/******************************************************************************/

static int __devinit mxt_identify(struct i2c_client *client,
				  struct mxt_data *mxt,
				  u8 *id_block_data)
{
	u8 buf[7];
	int error;
	int identified;

	identified = 0;

	/* Read Device info to check if chip is valid */
	error = mxt_read_block(client, MXT_ADDR_INFO_BLOCK, MXT_ID_BLOCK_SIZE,
			       (u8 *) buf);
	if (error < 0) {
		printk( "[TSP] Failure accessing maXTouch device\n");
		return -EIO;
	}

	memcpy(id_block_data, buf, MXT_ID_BLOCK_SIZE);

	mxt->device_info.family_id		= buf[0];
	mxt->device_info.variant_id		= buf[1];
	mxt->device_info.major		= ((buf[2] >> 4) & 0x0F);
	mxt->device_info.minor		= (buf[2] & 0x0F);
	mxt->device_info.build			= buf[3];
	mxt->device_info.x_size		= buf[4];
	mxt->device_info.y_size	    	= buf[5];
	mxt->device_info.num_objs   	= buf[6];
	mxt->device_info.num_nodes  	= mxt->device_info.x_size *
				      				mxt->device_info.y_size;
	/*
         * Check Family & Variant Info; warn if not recognized but
         * still continue.
         */
	/* MXT224 */
	if (mxt->device_info.family_id == MXT224_FAMILYID) {
		strcpy(mxt->device_info.family_name, "mXT224");

		if (mxt->device_info.variant_id == MXT224_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else if (mxt->device_info.variant_id ==
			MXT224_UNCAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Uncalibrated");
		} else {
			dev_err(&client->dev,
				"Warning: maXTouch Variant ID [%d] not "
				"supported\n",
				mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}

	/* MXT1386 */
	} else if (mxt->device_info.family_id == MXT1386_FAMILYID) {
		strcpy(mxt->device_info.family_name, "mXT1386");

		if (mxt->device_info.variant_id == MXT1386_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else {
			dev_err(&client->dev,
				"Warning: maXTouch Variant ID [%d] not "
				"supported\n",
				mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}
	/* Unknown family ID! */
	} else {
		dev_err(&client->dev,
			"Warning: maXTouch Family ID [%d] not supported\n",
			mxt->device_info.family_id);
		strcpy(mxt->device_info.family_name, "UNKNOWN");
		strcpy(mxt->device_info.variant_name, "UNKNOWN");
		/* identified = -ENXIO; */
	}

	dev_info(
		&client->dev,
		"Atmel maXTouch (Family %s (%X), Variant %s (%X)) Firmware "
		"version [%d.%d] Build %d\n",
		mxt->device_info.family_name,
		mxt->device_info.family_id,
		mxt->device_info.variant_name,
		mxt->device_info.variant_id,
		mxt->device_info.major,
		mxt->device_info.minor,
		mxt->device_info.build
	);
	dev_info(
		&client->dev,
		"Atmel maXTouch Configuration "
		"[X: %d] x [Y: %d]\n",
		mxt->device_info.x_size,
		mxt->device_info.y_size
	);
	return identified;
}

 static int  mxt_check_config_data(struct i2c_client *client,
                                                                struct mxt_data *mxt)
{
        u8 *mem_crc = NULL;
        u32 config_crc = 0;
        u32 my_config_crc = 0;
        u8	object_type;
        u16	object_address;
        u16	object_size;
        int	i;
        int	error;
        int maxsize_object = 0;

        for (i = 0; i < mxt->device_info.num_objs; i++)
        {
                object_size = mxt->object_table[i].size;
                if(object_size > maxsize_object)
                {
                        maxsize_object = object_size;
                }
        }
        mxt->configuration_crc = 1;
        mem_crc = kzalloc(maxsize_object, GFP_KERNEL);
        if (mem_crc == NULL)
        {
                printk( "[TSP] maXTouch: Can't allocate memory!\n");
                error = -ENOMEM;
                goto err_allocate_mem;
        }
        for (i = 0; i < mxt->device_info.num_objs; i++)
        {
                object_type = mxt->object_table[i].type;
                object_address = mxt->object_table[i].chip_addr;
                object_size = mxt->object_table[i].size;
                error = mxt_read_block(client, object_address,
                		       object_size, mem_crc);
                if (error < 0)
                {
                        printk( "[TSP] maXTouch Object %d could not be read!\n", i);
                        error = -EIO;
                        goto err_read_mxt_touch;
                }

                if(!IS_TOUCH_OBJECT(object_type))
		{
                        continue;
		}
                printk(  "[TSP] calculate config crc from chip!\n");
                calculate_config_data_crc(&config_crc, mem_crc, object_size);

                switch(object_type){
                        case MXT_GEN_POWERCONFIG_T7:
                                calculate_config_data_crc(&my_config_crc, atmel_config_data.config_T7, object_size);
                                break;

                        case MXT_GEN_ACQUIRECONFIG_T8:
                                calculate_config_data_crc(&my_config_crc, atmel_config_data.config_T8, object_size);
                                break;

                        case MXT_TOUCH_MULTITOUCHSCREEN_T9:
                                calculate_config_data_crc(&my_config_crc, atmel_config_data.config_T9, object_size);
                                break;

                        case MXT_TOUCH_KEYARRAY_T15:
                                calculate_config_data_crc(&my_config_crc, atmel_config_data.config_T15, object_size);
                                break;

                        case MXT_SPT_COMMSCONFIG_T18:
                                calculate_config_data_crc(&my_config_crc, atmel_config_data.config_T18, object_size);
                                break;

                        case MXT_PROCI_GRIPFACESUPPRESSION_T20:
                                calculate_config_data_crc(&my_config_crc, atmel_config_data.config_T20, object_size);
                                break;

                        case MXT_PROCG_NOISESUPPRESSION_T22:
                                calculate_config_data_crc(&my_config_crc, atmel_config_data.config_T22, object_size);
                                break;

                        case MXT_TOUCH_PROXIMITY_T23:
                                calculate_config_data_crc(&my_config_crc, atmel_config_data.config_T23, object_size);
                                break;

                        case MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:
                                calculate_config_data_crc(&my_config_crc, atmel_config_data.config_T24, object_size);
                                break;

                        case MXT_SPT_SELFTEST_T25:
                                calculate_config_data_crc(&my_config_crc, atmel_config_data.config_T25, object_size);
                                break;

                        case MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
                                calculate_config_data_crc(&my_config_crc, atmel_config_data.config_T27, object_size);
                                break;

                        case MXT_SPT_CTECONFIG_T28:
                                calculate_config_data_crc(&my_config_crc, atmel_config_data.config_T28, object_size);
                                break;

                        default:
                                printk(  "[TSP] warning: there are other object!\n");
                                break;
                }
                if(my_config_crc != config_crc)
                {
                        printk(  "[TSP] the config data is not match, object_type:%d, chip crc:%d, crc:%d\n",
                                           object_type, config_crc, my_config_crc);
                        mxt->configuration_crc = 0;
                        break;
                }
        }
        error = 0;
	err_read_mxt_touch:
        if(mem_crc != NULL)
        {
                kfree(mem_crc);
        }
	err_allocate_mem:

        return error;
}

/* v11 wangwp0004::modified by wangweiping 2011/03/17 for delete some useful message */
/*
 * Reads the object table from maXTouch chip to get object data like
 * address, size, report id. For Info Block CRC calculation, already read
 * id data is passed to this function too (Info Block consists of the ID
 * block and object table).
 *
 */
static int __devinit mxt_read_object_table(struct i2c_client *client,
					   struct mxt_data *mxt,
					   u8 *raw_id_data)
{
	u16	report_id_count;
	u8	buf[MXT_OBJECT_TABLE_ELEMENT_SIZE];
	u8      *raw_ib_data;
	u8	object_type;
	u16	object_address;
	u16	object_size;
	u8	object_instances;
	u8	object_report_ids;
	u16	object_info_address;
	u32	crc;
	u32     calculated_crc;
	int	i;
	int	error;
	u8	object_instance;
	u8	object_report_id;
	u8	report_id;
	int     first_report_id;
	int     ib_pointer;
	struct mxt_object *object_table;

	object_table = kzalloc(sizeof(struct mxt_object) *
			       mxt->device_info.num_objs,
			       GFP_KERNEL);
	if (object_table == NULL) {
		printk( "[TSP] maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_object_table_alloc;
	}

	raw_ib_data = kmalloc(MXT_OBJECT_TABLE_ELEMENT_SIZE *
			mxt->device_info.num_objs + MXT_ID_BLOCK_SIZE,
			GFP_KERNEL);
	if (raw_ib_data == NULL) {
		printk( "[TSP] maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_ib_alloc;
	}

	/* Copy the ID data for CRC calculation. */
	memcpy(raw_ib_data, raw_id_data, MXT_ID_BLOCK_SIZE);
	ib_pointer = MXT_ID_BLOCK_SIZE;

	mxt->object_table = object_table;
	object_info_address = MXT_ADDR_OBJECT_TABLE;

	report_id_count = 0;
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		printk("[TSP] Reading maXTouch at [0x%04x]: ",
			  object_info_address);

		error = mxt_read_block(client, object_info_address,
				       MXT_OBJECT_TABLE_ELEMENT_SIZE, buf);

		if (error < 0) {
			dev_err(&client->dev,
				"[TSP] maXTouch Object %d could not be read\n", i);
			error = -EIO;
			goto err_object_read;
		}

		memcpy(raw_ib_data + ib_pointer, buf,
		       MXT_OBJECT_TABLE_ELEMENT_SIZE);
		ib_pointer += MXT_OBJECT_TABLE_ELEMENT_SIZE;

		object_type       =  buf[0];
		object_address    = (buf[2] << 8) + buf[1];
		object_size       =  buf[3] + 1;
		object_instances  =  buf[4] + 1;
		object_report_ids =  buf[5];
		printk("[TSP] Type=%03d, Address=0x%04x, "
			  "Size=0x%02x, %d instances, %d report id's\n",
			  object_type,
			  object_address,
			  object_size,
			  object_instances,
			  object_report_ids
		);

		/* TODO: check whether object is known and supported? */

		/* Save frequently needed info. */
		if (object_type == MXT_GEN_MESSAGEPROCESSOR_T5) {
			mxt->msg_proc_addr = object_address;
			mxt->message_size = object_size;
		}

		object_table[i].type            = object_type;
		object_table[i].chip_addr       = object_address;
		object_table[i].size            = object_size;
		object_table[i].instances       = object_instances;
		object_table[i].num_report_ids  = object_report_ids;
		report_id_count += object_instances * object_report_ids;

		object_info_address += MXT_OBJECT_TABLE_ELEMENT_SIZE;
	}

	mxt->rid_map =
		kzalloc(sizeof(struct report_id_map) * (report_id_count + 1),
			/* allocate for report_id 0, even if not used */
			GFP_KERNEL);
	if (mxt->rid_map == NULL) {
		printk(KERN_WARNING "[TSP] maXTouch: Can't allocate memory!\n");
		error = -ENOMEM;
		goto err_rid_map_alloc;
	}

	mxt->report_id_count = report_id_count;
	if (report_id_count > 254) {	/* 0 & 255 are reserved */
			dev_err(&client->dev,
				"Too many maXTouch report id's [%d]\n",
				report_id_count);
			error = -ENXIO;
			goto err_max_rid;
	}

	/* Create a mapping from report id to object type */
	report_id = 1; /* Start from 1, 0 is reserved. */

	/* Create table associating report id's with objects & instances */
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		for (object_instance = 0;
		     object_instance < object_table[i].instances;
		     object_instance++){
			first_report_id = report_id;
			for (object_report_id = 0;
			     object_report_id < object_table[i].num_report_ids;
			     object_report_id++) {
				mxt->rid_map[report_id].object =
					object_table[i].type;
				mxt->rid_map[report_id].instance =
					object_instance;
				mxt->rid_map[report_id].first_rid =
					first_report_id;
				report_id++;
			}
		}
	}

	/* Read 3 byte CRC */
	error = mxt_read_block(client, object_info_address, 3, buf);
	if (error < 0) {
		dev_err(&client->dev, "Error reading CRC\n");
	}

	crc = (buf[2] << 16) | (buf[1] << 8) | buf[0];

	if (calculate_infoblock_crc(&calculated_crc, raw_ib_data,
				    ib_pointer)) {
		printk( "[TSP] Error while calculating CRC!\n");
		calculated_crc = 0;
	}
	kfree(raw_ib_data);

	mxt_debug(DEBUG_TRACE, "\nReported info block CRC = 0x%6X\n", crc);
	mxt_debug(DEBUG_TRACE, "Calculated info block CRC = 0x%6X\n\n",
		       calculated_crc);

	if (crc == calculated_crc) {
		mxt->info_block_crc = crc;
	} else {
		mxt->info_block_crc = 0;
		printk( "[TSP] maXTouch: Info block CRC invalid!\n");
	}

	if (debug >= DEBUG_VERBOSE) {

		dev_info(&client->dev, "maXTouch: %d Objects\n",
				mxt->device_info.num_objs);

		for (i = 0; i < mxt->device_info.num_objs; i++) {
			dev_warn(&client->dev, "Type:\t\t\t[%d]: %s\n",
				 object_table[i].type,
				 object_type_name[object_table[i].type]);
			dev_warn(&client->dev, "\tAddress:\t0x%04X\n",
				object_table[i].chip_addr);
			dev_warn(&client->dev, "\tSize:\t\t%d Bytes\n",
				 object_table[i].size);
			dev_warn(&client->dev, "\tInstances:\t%d\n",
				 object_table[i].instances);
			dev_warn(&client->dev, "\tReport Id's:\t%d\n",
				 object_table[i].num_report_ids);
		}
	}

	return 0;

err_max_rid:
	kfree(mxt->rid_map);
err_rid_map_alloc:
err_object_read:
	kfree(raw_ib_data);
err_ib_alloc:
	kfree(object_table);
err_object_table_alloc:
	return error;
}

static int mxt_suspend(struct device *dev)
{
        unsigned ret = -ENODEV;
        struct mxt_data *mxt = dev_get_drvdata(dev);
         int8_t config_T7_sleep[3] = {0, 0, 0};
 		printk(KERN_WARNING"[TSP] %s\n",__FUNCTION__);

        if (device_may_wakeup(dev))
        {
                if (mxt->client->irq)
                        enable_irq_wake(mxt->client->irq);
        }

        /* disable worker */
		disable_irq_nosync(mxt->client->irq);
	ret = cancel_work_sync(&mxt->work);
	if (ret){
		printk(KERN_WARNING"[TSP] %s,enable irq\n",__FUNCTION__);
		enable_irq(mxt->client->irq);
	}
      mxt_write_block(mxt->client, get_object_address(mxt, MXT_GEN_POWERCONFIG_T7),
                                                get_object_size(mxt, MXT_GEN_POWERCONFIG_T7), config_T7_sleep);
        if (mxt->exit_hw != NULL)
	        mxt->exit_hw();
        mxt->pre_data[0] = CAL_NODATA;
	//qt_timer_state=0;
        return 0;
}

static int mxt_resume(struct device *dev)

{
        struct mxt_data *mxt = dev_get_drvdata(dev);
        if (mxt->init_hw != NULL)
	        mxt->init_hw();

 		printk(KERN_WARNING"[TSP] %s\n",__FUNCTION__);

        mxt_write_block(mxt->client, get_object_address(mxt, MXT_GEN_POWERCONFIG_T7),
                                               get_object_size(mxt, MXT_GEN_POWERCONFIG_T7), atmel_config_data.config_T7);

        if (device_may_wakeup(dev))
        {
                if (mxt->client->irq){
				printk(KERN_WARNING"[TSP] %s,disable_irq_wake\n",__FUNCTION__);
                        disable_irq_wake(mxt->client->irq);
	        }
        }
	//good_check_flag=0;
	msleep(20);
	calibrate_chip(mxt);
	msleep(20);


	queue_work(mxt->atmel_wq, &mxt->work);
	return 0;
        /* re-enable the interrupt prior to wake device */
	#if 0
        if (mxt->client->irq){
		    printk("%s,enable_irq\n",__FUNCTION__);
                enable_irq(mxt->client->irq);
		}
        return 0;
	#endif

}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxt_early_suspend(struct early_suspend *h)
{
	struct mxt_data *ts = container_of(h, struct mxt_data, early_suspend);

	mxt_suspend(&ts->client->dev);
}

static void mxt_late_resume(struct early_suspend *h)
{
	struct mxt_data *ts = container_of(h, struct mxt_data, early_suspend);
	printk("[TSP] %s\n", __FUNCTION__);
	mxt_resume(&ts->client->dev);
}
#endif

static void  mxt_reload_config_data(struct mxt_data *mxt)
{

	  printk("[TSP] enter %s\n",__FUNCTION__);
        mxt_write_block(mxt->client, get_object_address(mxt, MXT_GEN_POWERCONFIG_T7),
                                        get_object_size(mxt, MXT_GEN_POWERCONFIG_T7),
                                        atmel_config_data.config_T7);
        mdelay(10);
        mxt_write_block(mxt->client, get_object_address(mxt, MXT_GEN_ACQUIRECONFIG_T8),
                                        get_object_size(mxt, MXT_GEN_ACQUIRECONFIG_T8),
                                        atmel_config_data.config_T8);
        mdelay(10);
        mxt_write_block(mxt->client, get_object_address(mxt, MXT_TOUCH_MULTITOUCHSCREEN_T9),
                                        get_object_size(mxt, MXT_TOUCH_MULTITOUCHSCREEN_T9),
                                        atmel_config_data.config_T9);
        mdelay(10);
        mxt_write_block(mxt->client, get_object_address(mxt, MXT_TOUCH_KEYARRAY_T15),
                                        get_object_size(mxt, MXT_TOUCH_KEYARRAY_T15),
                                        atmel_config_data.config_T15);
        mxt_write_block(mxt->client, get_object_address(mxt, MXT_PROCG_NOISESUPPRESSION_T22),
                                        get_object_size(mxt, MXT_PROCG_NOISESUPPRESSION_T22),
                                        atmel_config_data.config_T22);
        mdelay(10);
        mxt_write_block(mxt->client, get_object_address(mxt, MXT_SPT_COMMSCONFIG_T18),
                                        get_object_size(mxt, MXT_SPT_COMMSCONFIG_T18),
                                        atmel_config_data.config_T18);
        mdelay(10);
        mxt_write_block(mxt->client, get_object_address(mxt, MXT_PROCI_GRIPFACESUPPRESSION_T20),
                                        get_object_size(mxt, MXT_PROCI_GRIPFACESUPPRESSION_T20),
                                        atmel_config_data.config_T20);
        mdelay(10);
        mxt_write_block(mxt->client, get_object_address(mxt, MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24),
                                        get_object_size(mxt, MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24),
                                        atmel_config_data.config_T24);
        mdelay(10);
        mxt_write_block(mxt->client, get_object_address(mxt, MXT_SPT_SELFTEST_T25),
                                        get_object_size(mxt, MXT_SPT_SELFTEST_T25),
                                        atmel_config_data.config_T25);
        mdelay(10);
        mxt_write_block(mxt->client, get_object_address(mxt, MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27),
                                        get_object_size(mxt, MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27),
                                        atmel_config_data.config_T27);
        mdelay(10);
        mxt_write_block(mxt->client, get_object_address(mxt, MXT_SPT_CTECONFIG_T28),
                                        get_object_size(mxt, MXT_SPT_CTECONFIG_T28),
                                        atmel_config_data.config_T28);
        mdelay(10);
        mxt_write_byte(mxt->client, get_object_address(mxt, MXT_GEN_COMMANDPROCESSOR_T6) +
                                        MXT_ADR_T6_BACKUPNV, 0x55);
        mdelay(10);
        mxt_write_byte(mxt->client, get_object_address(mxt, MXT_GEN_COMMANDPROCESSOR_T6) +
                                        MXT_ADR_T6_RESET, 0x11);
        mdelay(200);

}



static ssize_t mxt224_proc_write(struct file *filp,
				       const char *buff, size_t len,
				       loff_t * off)
{
	char messages[256];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	printk("[TSP] %s\n",__FUNCTION__);
	disable_irq_nosync(mxt_proc->client->irq);
	queue_work(mxt_proc->atmel_wq, &mxt_proc->work);


	return len;

}

static struct proc_dir_entry *mxt224_proc_file;

static struct file_operations mxt224_proc_ops = {
	.write = mxt224_proc_write,
};

static void create_mxt224_proc_file(void)
{
	mxt224_proc_file = create_proc_entry("driver/mxt224", 0644, NULL);
	if (mxt224_proc_file) {
		mxt224_proc_file->proc_fops = &mxt224_proc_ops;
	} else
		printk(KERN_INFO "[TSP] proc file create failed!\n");
}
#if CONFIG_ZTE_PROP_BRIDGE
extern int prop_add(char *devname, char *item, char *value);/*ZTE: added by lipeng10094834 增加读取触摸屏型号接口 20110926 */
#endif

static int __init mxt_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct mxt_data          *mxt;
	struct mxt_platform_data *pdata;
	struct input_dev         *input;
	u8 *id_data;
       int error;
       char versionbuf[5];
	mxt_debug(DEBUG_INFO, "mXT224: mxt_probe\n");

	if (client == NULL) {
		pr_debug("maXTouch: client == NULL\n");
		return	-EINVAL;
	} else if (client->adapter == NULL) {
		pr_debug("maXTouch: client->adapter == NULL\n");
		return	-EINVAL;
	} else if (&client->dev == NULL) {
		pr_debug("maXTouch: client->dev == NULL\n");
		return	-EINVAL;
	} else if (&client->adapter->dev == NULL) {
		pr_debug("maXTouch: client->adapter->dev == NULL\n");
		return	-EINVAL;
	} else if (id == NULL) {
		pr_debug("maXTouch: id == NULL\n");
		return	-EINVAL;
	}

	/* Check if the I2C bus supports BYTE transfer */
	error = i2c_check_functionality(client->adapter,
			I2C_FUNC_I2C);
        if (!error)
        {
		dev_err(&client->dev, "%s adapter not supported\n",
				dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}

        printk(KERN_WARNING "[TSP] maXTouch driver functionality OK\n");

	/* Allocate structure - we need it to identify device */
	mxt = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	mxt_proc = mxt;
        if (mxt == NULL)
        {
		dev_err(&client->dev, "insufficient memory\n");
		error = -ENOMEM;
		goto err_mxt_alloc;
	}

	id_data = kmalloc(MXT_ID_BLOCK_SIZE, GFP_KERNEL);
        if (id_data == NULL)
        {
		dev_err(&client->dev, "insufficient memory\n");
		error = -ENOMEM;
		goto err_id_alloc;
	}

	input = input_allocate_device();
        if (!input)
        {
		dev_err(&client->dev, "error allocating input device\n");
		error = -ENOMEM;
		goto err_input_dev_alloc;
	}

	/* Initialize Platform data */
	pdata = client->dev.platform_data;
        if (pdata == NULL)
        {
		dev_err(&client->dev, "platform data is required!\n");
		error = -EINVAL;
		goto err_pdata;
	}
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "Platform OK: pdata = 0x%08x\n",
		       (unsigned int) pdata);

	mxt->max_x_val         = pdata->max_x;
	mxt->max_y_val         = pdata->max_y;

	/* Get data that is defined in board specific code. */
	mxt->init_hw = pdata->init_platform_hw;
	mxt->exit_hw = pdata->exit_platform_hw;
	mxt->read_chg = pdata->read_chg;
      mxt->numtouch   = pdata->numtouch;
      mxt->watchdog = 0;
	if (pdata->valid_interrupt != NULL)
		mxt->valid_interrupt = pdata->valid_interrupt;
	else
		mxt->valid_interrupt = mxt_valid_interrupt_dummy;

	if (mxt->init_hw != NULL)
		mxt->init_hw();

	printk(KERN_INFO "maXTouch driver identifying chip\n");

        if (mxt_identify(client, mxt, id_data) < 0)
        {
		dev_err(&client->dev, "Chip could not be identified\n");
		error = -ENODEV;
		goto err_identify;
	}
	/* Chip is valid and active. */
        printk(  KERN_WARNING"[TSP] maXTouch driver allocating input device\n");

	mxt->client = client;
	mxt->input  = input;

        mxt->atmel_wq = create_singlethread_workqueue("atmel_wq");
	if (!mxt->atmel_wq)
	{
		printk(KERN_ERR"[TSP] %s: create workqueue failed\n", __func__);
		error = -ENOMEM;
		goto err_cread_wq_failed;
	}
	INIT_WORK(&mxt->work, mxt_worker);
	mxt_debug(DEBUG_TRACE, "maXTouch driver creating device name\n");

	input->name = "atmel-maxtouch";
	input->phys = mxt->phys_name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	mxt->pre_data[0] = CAL_NODATA;  /* mengzf added for calibration */
	mxt->init_timestamp = jiffies_to_msecs(jiffies);
	mxt->init_time_check = 0;
	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
//	__set_bit(EV_MSC, input->evbit);
      __set_bit(BTN_TOUCH, input->keybit);
      __set_bit(BTN_2, input->keybit);


      input->mscbit[0] = BIT_MASK(MSC_GESTURE);

#ifdef ABS_MT_TRACKING_ID
	/* Multitouch */
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, mxt->max_x_val, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, mxt->max_y_val, 0, 0);
    	input_set_abs_params(input, ABS_MT_PRESSURE, 0, MXT_MAX_REPORTED_PRESSURE,
			     0, 0);
/*	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, MXT_MAX_TOUCH_SIZE,
			     0, 0);
      input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, MXT_MAX_WIDTH_SIZE,
			     0, 0);*/
#else
	/* Single touch */
	input_set_abs_params(input, ABS_X, 0, mxt->max_x_val, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, mxt->max_y_val, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, MXT_MAX_REPORTED_PRESSURE,
			     0, 0);
	input_set_abs_params(input, ABS_TOOL_WIDTH, 0, MXT_MAX_REPORTED_WIDTH,
			     0, 0);
#endif


	mxt_debug(DEBUG_TRACE, "maXTouch driver setting client data\n");
	i2c_set_clientdata(client, mxt);

	mxt_debug(DEBUG_TRACE, "maXTouch driver input register device\n");
	error = input_register_device(mxt->input);
        if (error < 0)
        {
		dev_err(&client->dev,
			"Failed to register input device\n");
		goto err_register_device;
	}

	error = mxt_read_object_table(client, mxt, id_data);

	if (error < 0)
		goto err_read_ot;

        error = mxt_check_config_data(client, mxt);
        if(error < 0)
        {
                printk(  "[TSP] Error check config data CRC\n");
        }
        printk( KERN_WARNING"[TSP] Atmel Muti-Touchscreen CRC: %d\n", mxt->configuration_crc);

        if(!mxt->configuration_crc)
        {
                printk( KERN_WARNING "KERN_WARNING[TSP] Touchscreen CRC Check Fail!, reload the Config data to chip\n");
                mxt_reload_config_data(mxt);
        }

	mxt_debug(DEBUG_TRACE, "maXTouch driver allocating interrupt...\n");
	mxt->irq = client->irq;
	if (mxt->irq) {
		/* Try to request IRQ with falling edge first. This is
		 * not always supported. If it fails, try with any edge. */
		error = request_irq(mxt->irq,
				    mxt_irq_handler,
				    IRQF_TRIGGER_FALLING,
				    client->dev.driver->name,
				    mxt);
		if (error < 0) {
			/* TODO: why only 0 works on STK1000? */
			error = request_irq(mxt->irq,
					    mxt_irq_handler,
					    0,
					    client->dev.driver->name,
					    mxt);
		}
		if (error < 0) {
			dev_err(&client->dev,
				"failed to allocate irq %d\n", mxt->irq);
			goto err_irq;
		}
	}
	error = mxt_write_byte(mxt->client, get_object_address(mxt, MXT_GEN_COMMANDPROCESSOR_T6) +
	                          MXT_ADR_T6_RESET, 0x11);
      if(error < 0)
        {
            printk(KERN_WARNING "[TSP] maXTouch driver reset error!\n");
        }
	mdelay(200);

	if(!gpio_get_value(146))
	{
        	 printk(KERN_WARNING "[TSP] maXTouch driver gpio_get_value 0\n");
              // mxt_timer_readback(mxt);
              mxt->watchdog ++;
              disable_irq_nosync(mxt->client->irq);
	        queue_work(mxt->atmel_wq, &mxt->work);


	}
	else
	{
		printk(KERN_WARNING "[TSP] maXTouch driver gpio_get_value 1\n");
	}
	create_mxt224_proc_file();
       dev_info(&client->dev, "touchscreen, irq %d\n", mxt->irq);


	/* Schedule a worker routine to read any messages that might have
	 * been sent before interrupts were enabled. */
	setup_timer(&mxt->timer, mxt_timer, (unsigned long) mxt);
	mod_timer(&mxt->timer, jiffies + TOUCHSCREEN_TIMEOUT);

	kfree(id_data);

#ifdef CONFIG_HAS_EARLYSUSPEND
	mxt->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2;
	mxt->early_suspend.suspend = mxt_early_suspend;
	mxt->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&mxt->early_suspend);
#endif

#ifdef CONFIG_ZTE_PROP_BRIDGE
      /*ZTE: added by lipeng10094834 增加读取触摸屏型号接口 20110927 ++*/
      sprintf(versionbuf, "0x%x", mxt->device_info.family_id);
      if(prop_add("Touchscreen", "fw-version", versionbuf))
      {
            printk("[TSP]: prop_add touchscreen failed!");
      }
      /*ZTE: added by lipeng10094834 增加读取触摸屏型号接口 20110927 --*/
#endif
	return 0;

	err_irq:
		kfree(mxt->rid_map);
		kfree(mxt->object_table);
	err_read_ot:
	err_register_device:
		destroy_workqueue(mxt->atmel_wq);
	err_identify:
	err_pdata:
	              if (input)
		        input_free_device(input);
	err_input_dev_alloc:
		kfree(id_data);
	err_cread_wq_failed:
	err_id_alloc:
		if (mxt->exit_hw != NULL)
			mxt->exit_hw();
		kfree(mxt);
	err_mxt_alloc:
		return error;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);

	if (mxt != NULL) {

		if (mxt->exit_hw != NULL)
			mxt->exit_hw();

		if (mxt->irq) {
			free_irq(mxt->irq, mxt);
		}

		destroy_workqueue(mxt->atmel_wq);
		input_unregister_device(mxt->input);

		kfree(mxt->rid_map);
		kfree(mxt->object_table);
	}
	kfree(mxt);

	i2c_set_clientdata(client, NULL);
	if (debug >= DEBUG_TRACE)
		dev_info(&client->dev, "Touchscreen unregistered\n");

	return 0;
}

static const struct i2c_device_id mxt_idtable[] = {
	{"AtmelMxt_224_i2c", 0,},
	{ }
};

MODULE_DEVICE_TABLE(i2c, mxt_idtable);

static const struct dev_pm_ops mxt_pm_ops = {
	.suspend = mxt_suspend,
	.resume = mxt_resume,
};

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "AtmelMxt_224_i2c",
		.owner  = THIS_MODULE,
#ifndef CONFIG_HAS_EARLYSUSPEND
        	.pm = &mxt_pm_ops,
#endif
	},

	.id_table	= mxt_idtable,
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
};

static int __init mxt_init(void)
{
	int err;
	err = i2c_add_driver(&mxt_driver);
	if (err) {
		printk(KERN_WARNING "[TSP] Adding maXTouch driver failed "
		       "(errno = %d)\n", err);
	} else {
		printk(KERN_WARNING "[TSP] Successfully added driver %s\n",
		          mxt_driver.driver.name);
	}
	return err;
}

static void __exit mxt_cleanup(void)
{
	i2c_del_driver(&mxt_driver);
}


module_init(mxt_init);
module_exit(mxt_cleanup);

MODULE_AUTHOR("Iiro Valkonen");
MODULE_DESCRIPTION("Driver for Atmel maXTouch Touchscreen Controller");
MODULE_LICENSE("GPL");
