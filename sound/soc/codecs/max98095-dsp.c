/*
 * max98095 codec DSP firmware download and control interface functions
 * Copyright 2011 Maxim Integrated Products
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/slab.h>

#include "max98095.h"
#include <sound/max98095.h>
#include "max98095-dsp.h"

#include <linux/version.h>

/***************************************************************/
/* DSP firmware data file                                      */
/***************************************************************/
#include "max98095.fw"

//#define M98095_ENCRYPTION
//#define M98095_DBGPRINT

#ifdef M98095_DBGPRINT
  #define DBG_PRINT(...) printk(KERN_DEBUG __VA_ARGS__)
#else
  #define DBG_PRINT(...) ;
#endif

#ifdef M98095_DBGDUMP
  #define DBG_DUMP(...) max98095_hex_dump(__VA_ARGS__)
#else
  #define DBG_DUMP(...) ;
#endif

#ifdef M98095_DBGDUMP
static void max98095_hex_dump(uint8_t *ptr, int bytelen)
{
	int i;
	int first_time = 1;
	
	for (i = 0; i < bytelen; i++) {
		if (i % 8 == 0 || first_time) {
			printk(KERN_DEBUG "\n0x%04x:", i);
			first_time = 0;
		}
		printk(KERN_DEBUG " 0x%02x", ptr[i] & 0xff);
	}
	printk(KERN_DEBUG "\n");
}
#endif

/* 
 * Read a known number of available bytes.
 * return <0 on error or returns number actually read
 */
static int max98095_xfer_read(struct snd_soc_codec *codec, int i2c_reg, uint8_t *buf,
						int avail_len, int max_to_read)
{
	struct i2c_client *i2c = codec->control_data;
	int ret;
	u8 register_number = i2c_reg;
	int len = min(avail_len, max_to_read);
	struct i2c_msg xfer[2];

	/* first transfer register number */
	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = sizeof(register_number);
	xfer[0].buf = &register_number;
	/* transfer buffer */
	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = buf;

	DBG_PRINT("reading %d bytes of max %d to reg0x%02x\n", len, max_to_read, i2c_reg);
	DBG_DUMP(buf, max_to_read);

	ret = i2c_transfer(i2c->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret != ARRAY_SIZE(xfer)) {
		dev_err(&i2c->dev, "i2c_transfer() returned %d\n", ret);
		return ret;
	}

	return len;
}

/* Read data from the DSP based on number of available bytes to read */
#define MSEC_TIMEOUT 60
#define MSEC_PER_PAUSE 1
static int max98095_fifo_read(struct snd_soc_codec *codec, uint8_t *buf, int read_len)
{
	int ret;
	int len;
	int indx = 0;
	int delay = MSEC_TIMEOUT / MSEC_PER_PAUSE;

	while (indx < read_len) {
		/* check number of bytes (0..8) available to read */
		len = snd_soc_read(codec, M98095_002_HOST_RSP_STS);
		if (len < 0)
			return len;

		DBG_PRINT("Avail %d bytes reading %d bytes ind=%d\n", len, read_len, indx);

		if (len) {
			ret = max98095_xfer_read(codec, M98095_000_HOST_DATA,
					&buf[indx], len, read_len - indx);
			if (ret < 0)
				return ret;

			DBG_PRINT("reg=0x00 indx=%d len=%d ret=0x%x\n", indx, read_len, ret);
			indx += ret; /* update collected amount */
			/* restart timeout */
			delay = MSEC_TIMEOUT / MSEC_PER_PAUSE;
		}
		if (indx < read_len) {
			msleep(MSEC_PER_PAUSE);
			if (!(delay--))
				return -7;
		}
	}
	return 0;
}

/* 
 * Write data to the DSP 
 * Here we know how much is guaranteed available, write only what I need
 * Return amount written, or <0 on error
 */
static int max98095_xfer_write(struct snd_soc_codec *codec, int i2c_reg,
			uint8_t *buf, int avail_len, int max_to_write)
{
	struct i2c_client *i2c = codec->control_data;
	int ret;
	int len = min(avail_len, max_to_write);
	uint8_t xferbuf[65];

	xferbuf[0] = i2c_reg;
	memcpy(&xferbuf[1], buf, len);

	DBG_PRINT("writing %d bytes of max %d to reg0x%02x\n", len, max_to_write, i2c_reg);
	DBG_DUMP(xferbuf, len + 1);

	ret = i2c_master_send(i2c, xferbuf, len + 1);
	if (ret != len + 1) {
		DBG_PRINT("i2c_transfer() returned %d\n", ret);
		return -8;
	}

	return len;
}

/* Write data to the DSP through the FIFO */
static int max98095_fifo_write(struct snd_soc_codec *codec, uint8_t *buf, int write_len)
{
	int ret;
	int len;
	int avail;
	int index = 0;
	int delay = MSEC_TIMEOUT / MSEC_PER_PAUSE;

	do {
		/* check number of bytes free in FIFO available to write (up to 64) */
		avail = snd_soc_read(codec, M98095_003_HOST_CMD_STS);
		if (avail < 0)
			return avail; /* return read error status */

		len = min(avail, (write_len - index));

		if (len)
			DBG_PRINT("avail %d bytes writing %d bytes (of %d) ind=%d\n", avail, len, write_len, index);

		if (avail) {
			ret = max98095_xfer_write(codec, M98095_000_HOST_DATA, &buf[index], avail, write_len - index);
			if (ret < 0)
				return ret;

			index += ret;
			delay = MSEC_TIMEOUT / MSEC_PER_PAUSE; /* restart timeout */
		}
		
		if (index < write_len) {
			msleep(MSEC_PER_PAUSE);
			if (!(delay--))
				return -7;
		}
	} while (index < write_len);

	return 0;
}

/* Read len bytes from FIFO into buffer. Returns <0 on error */ 
static int max98095_fifo_block_read(struct snd_soc_codec *codec, uint8_t *buf, int len)
{
	int ret = 0;
	int offs = 0;
	int xferlen;
	
	while (len) {
		xferlen = min(len, I2C_SMBUS_BLOCK_MAX);
		ret = max98095_fifo_read(codec, &buf[offs], xferlen);
		if (ret < 0) {
			printk(KERN_ERR "FIFO read error %d\n", ret);
			return ret;
		}

		DBG_DUMP(&buf[offs], xferlen);
		len  -= xferlen;
		offs += xferlen;
	}
	return ret;
}

/* Write len bytes from buf. Returns <0 on error */
int max98095_fifo_block_write(struct snd_soc_codec *codec, uint8_t *buf, int len)
{
	int ret = 0;
	int offs = 0;
	int xferlen;

	while (len) {
		xferlen = min(len, I2C_SMBUS_BLOCK_MAX);
		ret = max98095_fifo_write(codec, &buf[offs], xferlen);
		if (ret < 0) {
			printk(KERN_ERR "FIFO write error %d\n", ret);
			return ret;
		}
		
		DBG_DUMP(&buf[offs], xferlen);
		len  -= xferlen;
		offs += xferlen;
	}
	return ret;
}

/* 
 * read flush the response queue hardware FIFO
 * return <0 if error, else 0
 */
static int max98095_fifo_read_flush(struct snd_soc_codec *codec)
{
	int cmd;
	int ret;
	uint8_t buf[8];
	
	do {
		/* check number of bytes (0..8) available to read */
		cmd = snd_soc_read(codec, M98095_002_HOST_RSP_STS);
		ret = cmd;

		if (cmd > 0) {
			ret = max98095_fifo_block_read(codec, buf, cmd);
			if (ret < 0) {
				printk(KERN_ERR "error %d purging input buffer\n", ret);
				break;
			}
		}
	} while (cmd > 0);
	
	return ret;
}

int max98095_read_command_status(struct snd_soc_codec *codec)
{
	int ret;
	uint8_t status[4];
	
	ret = max98095_fifo_block_read(codec, status, 4);
	DBG_PRINT("Command status=0x%x 0x%x 0x%x 0x%x\n", status[0], status[1], status[2], status[3]);
	if (ret >= 0) {	
		ret = status[2] + (status[3] << 8);
		ret = (ret == 0xffff) ? -1 : ret;
	}

	return ret;
}

/* 
 * Send a dsp command, and read back response.
 * buf is the source and destination buffer for commands.
 */
int max98095_command(struct snd_soc_codec *codec, uint8_t *buf, int txlen, int rxlen)
{
	int ret;

	DBG_PRINT("\n%s sending %d bytes waiting for %d bytes\n", __func__, txlen, rxlen);
	DBG_DUMP(buf, txlen);

	max98095_fifo_read_flush(codec);

	mutex_lock(&codec->mutex);
	ret = max98095_fifo_block_write(codec, buf, txlen);
	if (ret < 0) {
		printk(KERN_ERR "error %d writing data\n", ret);
		goto done;
	}
	ret = max98095_fifo_block_read(codec, buf, rxlen);
	if (ret < 0)
		printk(KERN_ERR "error %d reading data\n", ret);
done:
	mutex_unlock(&codec->mutex);
	return ret;
}

/* 
 * Issue a get ROM ID (version) command, where id is module number, or 0 for romid
 * returns 16 bit id number of <0 if error accessing id
 */
int max98095_get_rom_id(struct snd_soc_codec *codec, int id)
{
	int ret;
	uint8_t buf[] = { MID_GET_ID, id, 0, 0 };

	ret = max98095_command(codec, buf, sizeof(buf), sizeof(buf));
	if (ret < 0) {
		ret = -1;
	}
	else {
		ret = (buf[2] | buf[3]<<8);
	}
	
	return ret;
}

/* 
 * Issue a set param command, where
 * id is module number
 * len is number of uint32_t's in a uint32_t array of params
 * returns negative status if failed
 */
int max98095_set_param(struct snd_soc_codec *codec, int id, int len, uint32_t *in_params)
{
	int ret;
	uint8_t buf[DSP_XFER_MAX];

	buf[0] = MID_SET_PARAMETER; /* get param */
	buf[1] = id;                /* module id (middleware) */
	buf[2] = len;               /* length, low */
	buf[3] = 0x0;               /* length, high */

	if ((len * 4) > ((int)sizeof(buf) - 4)) {
		printk(KERN_ERR "Too many params %d\n", len);
		return -1;
	}

	memcpy(&buf[4], in_params, len * 4);
	DBG_PRINT("Setting 0x%02X param 0x%08X\n", id, *(uint32_t*)buf);

	ret = max98095_command(codec, buf, 4 + (len * 4), 4);
	if (ret < 0)
		DBG_PRINT(KERN_ERR "can't set params\n");

	DBG_DUMP(buf, len * 4);
	return ret;
}

/* 
 * Handle code downloads and checks return status
 * returns 0 for success, <0 for error
 * if chkstatus is true, fetch and check the xtensa status message
 */
int max98095_mem_download(struct snd_soc_codec *codec, uint8_t *bptr, int len, int chkstatus)
{
	int ret;

	mutex_lock(&codec->mutex);

	ret = max98095_fifo_block_write(codec, bptr, len);
	if (ret >= 0 && chkstatus)
		ret = max98095_read_command_status(codec);

	mutex_unlock(&codec->mutex);

	return ret;
}

static int offset_config(int cfg_no)
{
	int i;
	int size = 0;
	
	for (i = 0; i < cfg_no; i++)
		size += fw_config_sizes[i];

	return size;
}

/*
 * every config has a initial index in file_sizes
 * And a number of files after this initial config
 */
static int offset_file_base(int cfg_no)
{
	int i;
	int size = 0;

	for (i = 0; i < cfg_no; i++)
		size += fw_config_file_count[i];

	return size;
}

static int offset_file_size(int cfg_no, int file_no)
{
	return fw_config_file_index[offset_file_base(cfg_no) + file_no];
}

static int offset_file_binary(int cfg_no, int file_no)
{
	int i;
	int size = 0;

	for (i = 0; i < offset_file_size(cfg_no, file_no); i++)
		size += fw_file_sizes[i];

	return size;
}

/*
 * Every module or tile has a unique control count
 */
static int offset_module(int cfg_no)
{
	int i;
	int size = 0;
	
	for (i = 0; i < cfg_no; i++)
		size += fw_config_module_count[i];

	return size;
}

/*
 * Skip to requested control in requested config 
 * need to skip number of controls until hit the desired config.
 * From the fw_config_module_count per config,
 * need to consume that many entries in fw_controls_per_tile
 * to point to the base control of the config, then skip to module_no in config
 */
static int offset_control(int cfg_no, int module_no)
{
	int i;
	int size = 0;
	
	for (i = 0; i < offset_module(cfg_no) + module_no; i++)
		size +=	fw_controls_per_tile[i];

	return size;
}

/*
 * Calculate index into the reg values table using
 * config# times the length of each mode's register table.
 * The table contains only the value and not the reg number.
 */
static int offset_reg(int cfg_no)
{
	return cfg_no * fw_reglist_len;
}

#ifdef M98095_ENCRYPTION

#define PREAMBLE_LENGTH 12
#define POSTAMBLE_LENGTH 9
#define KEY_LENGTH 6
static int max98095_send_preamble(struct snd_soc_codec *codec, int cfg_no, uint8_t *data)
{
	int i;

	for (i = 0; i < KEY_LENGTH; i++) {
		snd_soc_write(codec, M98095_018_KEYCODE3 + i,
			fw_registers[offset_reg(cfg_no) + M98095_018_KEYCODE3 + i]);
	}

	for (i = 0; i < PREAMBLE_LENGTH; i++)
		snd_soc_write(codec, M98095_015_DEC, data[i]);

	return 0;
}

static int max98095_send_postamble(struct snd_soc_codec *codec, uint8_t *data)
{
	int i;
	
	for (i = 0; i < POSTAMBLE_LENGTH; i++)
		snd_soc_write(codec, M98095_015_DEC, data[i]);

	return 0;
}

static int max98095_load_encrypted(struct snd_soc_codec *codec, int cfg_no, int filesize, int myfile)
{
	int ret;

	/* handle encrypted download */
	snd_soc_update_bits(codec, M98095_00F_HOST_CFG, M98095_DCRYPTEN, M98095_DCRYPTEN);

	ret = max98095_send_preamble(codec, cfg_no, &fw_binary[myfile]);
	if (ret < 0)
		return ret;

	DBG_DUMP(&fw_binary[myfile + PREAMBLE_LENGTH], filesize - POSTAMBLE_LENGTH - PREAMBLE_LENGTH);

	ret = max98095_mem_download(codec, &fw_binary[myfile + PREAMBLE_LENGTH],
			filesize - POSTAMBLE_LENGTH - PREAMBLE_LENGTH, 0);
	if (ret < 0)
		return ret;

	ret = max98095_send_postamble(codec, &fw_binary[myfile + filesize - PREAMBLE_LENGTH]);
	if (ret < 0)
		return ret;

	snd_soc_update_bits(codec, M98095_00F_HOST_CFG, M98095_DCRYPTEN, 0);
	ret = max98095_read_command_status(codec);
	return ret;
}
#endif


#define PARAM_STEP_SIZE  10

/* Iterate through all modules in this config */
static int max98095_load_params(struct snd_soc_codec *codec, enum fw_config_enum cfg_no)
{
	int i,p, j;
	int ret = 0;
	int param_cnt; /* number of parameters to send in this module */
	int stepcnt;   /* number of parameters in this packet to send */
	uint32_t param[PARAM_STEP_SIZE];
	
	for (j = 0; j < fw_config_module_count[cfg_no]; j++) {
		/* iterate through all controls in this module */
		param_cnt = fw_controls_per_tile[offset_module(cfg_no) + j];

		i = 0;
		while (i < param_cnt) {
	 		stepcnt = (param_cnt - i);
	 		if (stepcnt > PARAM_STEP_SIZE)
	 			stepcnt = PARAM_STEP_SIZE;

			for (p = 0; p < stepcnt; p++) {
				param[p] = (fw_control_values[offset_control(cfg_no, j) + i + p] & 0x00ffffff)
				   		    + (fw_control_ids[offset_control(cfg_no, j) + i + p] << 24);
				DBG_PRINT("Set param module=%x, param[]=%x j=%d, i=%d p=%d\n", fw_pmid[offset_module(cfg_no) + j], param[p], j, i, p);
			}
			ret = max98095_set_param(codec, fw_pmid[offset_module(cfg_no) + j], stepcnt, param);
			if (ret < 0)
				goto done;
			msleep(1);
	 		i = i + stepcnt;
	 	}
	}
done:
	return ret;
}

static int max98095_hw_write(struct snd_soc_codec *codec, unsigned int reg,
			     unsigned int value)
{
	u8 data[2];

	data[0] = reg;
	data[1] = value;
	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;
	else
		return -EIO;
}

/*
 * Write len bytes from buf starting register number reg
 * returns <0 on error
 */
int max98095_load_dsp_regs(struct snd_soc_codec *codec, int reg, uint8_t *buf, int len)
{
	int ret = 0;
	int i;

	if (reg + len > 0xFF) {
		DBG_PRINT("Error: register out of bound\n");
		return -1;
	}

	for (i=0; i<len; i++)
	{
		switch (reg + i) {
		case M98095_00F_HOST_CFG: 	  /* xtp0dis, dcrypten, dsp clock enables */
		case M98095_010_HOST_INT_CFG: /* host interrupt config */
		case M98095_011_HOST_INT_EN:  /* host interrupt enable */
			ret = snd_soc_write(codec, reg + i, buf[i]);
			if (ret < 0) {
				DBG_PRINT("Error: Write failed, return code %d\n", ret);
				return ret;
			}
			break;
		default:
			break;
		}
	}
	return ret;
}

/* Write regs (0x1E..0x23) needed to turn on FIFOs 	*/
int max98095_fifo_enable(struct snd_soc_codec *codec, enum fw_config_enum cfg_no)
{
	int i;
	int ret = 0;
	uint8_t *buf = &fw_registers[offset_reg(cfg_no) + (0x1E - 0x0F)];

	for (i=0; i<6; i++) {
		ret = snd_soc_write(codec, 0x1E + i, buf[i]);
		if (ret < 0) {
			DBG_PRINT("Error: Write failed, return code %d\n", ret);
			break;
		}
	}
	return ret;
}

/* data FIFO to bypasses DSP while we download */
int max98095_fifo_disable(struct snd_soc_codec *codec)
{
	int i;
	int ret = 0;

	for (i=0; i<6; i++) {
		ret = snd_soc_write(codec, 0x1E + i, 0);
		if (ret < 0) {
			DBG_PRINT("Error: Write failed, return code %d\n", ret);
			return ret;
		}
	}
	return ret;
}

/*
 * Install a new firmware image, send it a system configuration and
 * then send it all the required configs and params to get it running.
 */
int max98095_dsp_download(struct snd_soc_codec *codec, enum fw_config_enum cfg_no)
{
	struct max98095_priv *max98095 = snd_soc_codec_get_drvdata(codec);
	int myfile, filesize;
	uint32_t filetype;
	int i;
	int ret = 0;
	volatile int status;
	int r48, r49;

	if (cfg_no >= FLEXSOUND_NUMBER_OF_CONFIGS)
		return -ENODEV;

	/* load the hex image if this same image is not currently in the dsp */
	myfile = offset_file_binary(cfg_no, 0);
	dev_info(codec->dev, "DSP download %s %d\n", fw_filenames[cfg_no], myfile);

	if (myfile != max98095->cur_dsp) {
		max98095->cur_dsp = myfile;

		snd_soc_update_bits(codec, M98095_097_PWR_SYS, M98095_SHDNRUN, 0);

		if ((ret = max98095_fifo_disable(codec)) < 0)
			return ret;

		snd_soc_update_bits(codec, M98095_097_PWR_SYS, M98095_SHDNRUN, M98095_SHDNRUN);

		/* clear any xsts bits by reading status reg, and reset the DSP for download */
		status = snd_soc_read(codec, M98095_001_HOST_INT_STS);
		snd_soc_update_bits(codec, M98095_00F_HOST_CFG,
			M98095_XTCLKEN | M98095_MDLLEN | M98095_XTEN | M98095_SEG, 0);

		/* load dsp related registers from configuration starting at reg number (fw_reglist_first + 1) */
		DBG_DUMP(&fw_registers[offset_reg(cfg_no)], M98095_097_PWR_SYS + 1 - fw_reglist_first);
		ret = max98095_load_dsp_regs(codec,
				fw_reglist_first + 1,
				&fw_registers[offset_reg(cfg_no) + 1],
				M98095_097_PWR_SYS - fw_reglist_first - 1);
		if (ret < 0)
			goto done;

		/* write the FIFO enable configuration for this firmware image */ 
		ret = max98095_fifo_enable(codec, cfg_no);
		if (ret < 0)
			goto done;

		/* turn on the dsp one bit at a time, with XTEN enable bit last, separate from clocking */
		snd_soc_update_bits(codec, M98095_00F_HOST_CFG, M98095_MDLLEN, M98095_MDLLEN);
		snd_soc_update_bits(codec, M98095_00F_HOST_CFG, M98095_XTCLKEN, M98095_XTCLKEN);
		snd_soc_update_bits(codec, M98095_00F_HOST_CFG, M98095_XTEN, M98095_XTEN);

		/* verify reg 1 xsts[7:6] bits are 1s */
		status = snd_soc_read(codec, M98095_001_HOST_INT_STS);
		if ((status & 0xC0) != 0xC0) {
			/* reset dsp core again - toggling xten is sufficient */
			snd_soc_update_bits(codec, M98095_00F_HOST_CFG, M98095_XTEN, 0);
			DBG_PRINT(KERN_ERR "Toggle XTEN. INT_STS status: 0x%02X\n", status);
			/* wait at least 50us */
			msleep(1);
			snd_soc_update_bits(codec, M98095_00F_HOST_CFG, M98095_XTEN, M98095_XTEN);
		}

		/* get rom id , where id=module#, or 0 for romid
	 	 * returns 16 bit id number or neg value if error */
		if ((status = max98095_get_rom_id(codec, 0)) < 0) {
			printk(KERN_ERR "get_rom_id=%d\n", status);
			ret = -1;
			goto done;
		} 		 

		/* download the program code - a list of files to load */
		for (i = 0; i < fw_config_file_count[cfg_no]; i++) {
			myfile = offset_file_binary(cfg_no, i);
			filetype = fw_binary[myfile] + (fw_binary[myfile + 1] << 8);
			filesize = fw_file_sizes[offset_file_size(cfg_no, i)];			
	
			#define FLEXSOUND_ENCRYPTED	0x2118
			#define FLEXSOUND_DOWNLOAD	0
			#define FLEXSOUND_CONFIG	0x003
			
			switch (filetype) {
#ifdef M98095_ENCRYPTION
			case FLEXSOUND_ENCRYPTED:
				ret = max98095_load_encrypted(codec, cfg_no, filesize, myfile);
				break;
#endif
			case FLEXSOUND_DOWNLOAD:
			case FLEXSOUND_CONFIG:
				mutex_lock(&codec->mutex);
				ret = max98095_fifo_block_write(codec, &fw_binary[myfile], filesize);
				if (ret >= 0)
					ret = max98095_read_command_status(codec);
				mutex_unlock(&codec->mutex);
				break;
	
			default:
				ret = -44; /* unknown binary */
			}
			if (ret < 0)
				break;
		}
		if (ret < 0) {
			printk(KERN_ERR "do_download=%d\n", ret);
			goto done;
		}

		/* download the optional config */
		DBG_DUMP((uint8_t *)&fw_all_config[offset_config(cfg_no)], fw_config_sizes[cfg_no]);
		if (fw_config_sizes[cfg_no] > 1) {
			ret =  max98095_mem_download(codec, &fw_all_config[offset_config(cfg_no)],
						fw_config_sizes[cfg_no], 1);
			if (ret < 0) {
				printk(KERN_ERR "mem_download=%d\n", ret);
				goto done;
			}
		}
	}

	/* download the parameters */
	ret = max98095_load_params(codec, cfg_no);
	if (ret < 0) {
		printk(KERN_ERR "Send parameter error %d\n", ret);
		goto done;
	}

	mutex_lock(&codec->mutex);
	r48 = snd_soc_read(codec, M98095_048_MIX_DAC_LR);
	r49 = snd_soc_read(codec, M98095_049_MIX_DAC_M);
	max98095_hw_write(codec, M98095_048_MIX_DAC_LR, 0);
	max98095_hw_write(codec, M98095_049_MIX_DAC_M, 0);
	max98095_hw_write(codec, M98095_048_MIX_DAC_LR, r48);
	max98095_hw_write(codec, M98095_049_MIX_DAC_M, r49);
	mutex_unlock(&codec->mutex);
done:
	return ret;
}

char **max98095_get_cfg_names(void)
{
	return &fw_filenames[0];
}

int max98095_get_cfg_count(void)
{
	return FLEXSOUND_NUMBER_OF_CONFIGS;
}

static int max98095_find_cfg(const char *name)
{
	int i;
	
	for (i=0; i<FLEXSOUND_NUMBER_OF_CONFIGS; i++)
	{
		if (strcmp(name, fw_filenames[i]) == 0) {
			return i;
		}
	}
	return -EINVAL;
}

static int max98095_put_fs_enum(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct max98095_priv *max98095 = snd_soc_codec_get_drvdata(codec);
	int sel = ucontrol->value.integer.value[0];
	char **cfg_str_addr;
	int count;
	int ret;

	count = max98095_get_cfg_count();
	cfg_str_addr = max98095_get_cfg_names();

	if (!max98095->fs_textcnt)
		return 0;

	if (sel >= count)
		return -EINVAL;

	max98095->fs_sel = sel;

	ret = max98095_dsp_download(codec, sel);
	if (ret < 0) {
		printk(KERN_ERR "start DSP error %d\n", ret);
		return -EINVAL;
	}

	return 0;
}

static int max98095_get_fs_enum(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct max98095_priv *max98095 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = max98095->fs_sel;
	return 0;
}

/* Modify string name based on current silicon revision ID */
/* Detect for trailing _B or _C pattern in the string.     */
/* If this is rev C silicon, then we will convert string   */
/* "SPK_C" to "SPK", while leaving "SPK_B" unmodified.     */ 
static int max98095_fs_match_revision(char *t, char ver)
{
	int len;
	
	len = strlen(t);
	if (len <= 2)
		return 0;
	
	if (t[len-2]=='_') {
		if (t[len-1] == ver) {
			t[len-2] = 0;
			return 1;
		}
	}

	return 0;	
}

static void max98095_fs_setup(struct snd_soc_codec *codec)
{
	struct max98095_priv *max98095 = snd_soc_codec_get_drvdata(codec);
	int i, count, ret;
	char **t;
	char **cfg_str_addr;
	char revchar;

	struct snd_kcontrol_new controls[] = {
		SOC_ENUM_EXT("Flexsound",
			max98095->fs_enum,
			max98095_get_fs_enum,
			max98095_put_fs_enum),
	};

	ret = snd_soc_read(codec, M98095_0FF_REV_ID);
	if (ret == 0x42) /* if REV C silicon */
		revchar = 'C';
	else if (ret == 0x40) /* if REV A/B silicon */
		revchar = 'B';
	else
		revchar = '*';

	max98095->fs_textcnt = 0;
	max98095->fs_texts = NULL;

	count = max98095_get_cfg_count();
	cfg_str_addr = max98095_get_cfg_names();

	if (cfg_str_addr == NULL) {
		printk(KERN_ERR "Invalid flexsound configuration\n");
		return;
	}

	for (i = 0; i < count; i++) {
		
		t = krealloc(max98095->fs_texts,
			     sizeof(char *) * (max98095->fs_textcnt + 1),
			     GFP_KERNEL);
		if (t == NULL)
			continue;

		t[max98095->fs_textcnt] = cfg_str_addr[i];
		max98095_fs_match_revision( t[max98095->fs_textcnt], revchar);
		max98095->fs_textcnt++;
		max98095->fs_texts = (const char **)t;
	}

	max98095->fs_enum.texts = max98095->fs_texts;
	max98095->fs_enum.max = max98095->fs_textcnt;

	ret = snd_soc_add_controls(codec, controls, ARRAY_SIZE(controls));
	if (ret != 0)
		printk(KERN_ERR "Failed to add control: %d\n", ret);
}

/* Initialize the Flexsound DSP firmware */
int max98095_flexsound_init(struct snd_soc_codec *codec)
{
	int ret = 0; 

	/* turn on the codec and enable the clocks to the DSP */
	snd_soc_update_bits(codec, M98095_097_PWR_SYS, M98095_SHDNRUN, M98095_SHDNRUN);
	snd_soc_write(codec, M98095_025_XCLKCFG, 0x6D);      //XTCLK=DLLCK     //0x0E);//XTCLK = (PCLK/2)*6, XTclk=DLLCLK*14/16
	snd_soc_write(codec, M98095_026_SYS_CLK, 0x10);	     //PSCLK, MCLKSEL
	snd_soc_write(codec, M98095_010_HOST_INT_CFG, 0xC0); //Host Int Config
	snd_soc_write(codec, M98095_011_HOST_INT_EN, 0x06);  //Host Int Enable
	snd_soc_write(codec, M98095_00F_HOST_CFG, 0x0C);     //XTCLKEN, MDLLEN
	snd_soc_write(codec, M98095_00F_HOST_CFG, 0x0E);     //+XTEN

	max98095_fs_setup(codec);

	/* pre-load a DSP image to start with to enable fast mode switching */
	ret = max98095_dsp_download(codec, max98095_find_cfg("SPK"));
	if (ret < 0)
		printk(KERN_ERR "Initial DSP download error %d\n", ret);

	return ret;
}
