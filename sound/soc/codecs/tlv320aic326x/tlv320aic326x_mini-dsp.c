/*
 * linux/sound/soc/codecs/tlv320aic326x_mini-dsp.c
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * The TLV320AIC3262 is a flexible, low-power, low-voltage stereo audio
 * codec with digital microphone inputs and programmable outputs.
 *
 * History:
 *
 * Rev 0.1   Added the miniDSP Support     Mistral         01-03-2011
 *
 * Rev 0.2   Updated the code-base for miniDSP switching and
 *     mux control update.    Mistral         21-03-2011
 *
 * Rev 0.3   Updated the code-base to support Multi-Configuration feature
 *           of PPS GDE
   Rev 0.4   add codec->mutex protection. For future new controls, remember 
             to add the mutex! zuoziqiang 16-2-2012
 */

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */
//#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/core.h>
#include <sound/soc-dapm.h>
#include <sound/control.h>
#include <linux/time.h>		/* For timing computations */
#include "tlv320aic326x.h"
#include "tlv320aic326x_mini-dsp.h"

//#include "second_rate_pps_driver.h"
#include "base_main_Rate8_pps_driver_TD_simpleflow.h"
//#include "base_main_Rate8_pps_driver_1mic_A3b16_2.h"
//#include "base_main_Rate8_pps_driver_A3B21.h"
//#include "base_main_Rate8_pps_driver_A3B22.h"
#include "base_main_Rate8_pps_driver_1mic_A3B29.h"
//#include "base_main_Rate8_pps_driver_A3B19.h"
/*patches for 1mic nc=====*/
#include "Patch_base_main_Rate8_pps_driver.h"
#include "Patch_base_Headphone_Rate8_pps_driver.h"
#include "Patch_base_speakerphone_Rate8_pps_driver.h"
//two mic nc for  voice call
//#include "base_main_Rate8_pps_driver_2mic_A4B23.h"
//#include "base_main_Rate8_pps_driver_A4B31.h"
//#include "base_main_Rate8_pps_driver_A4B32.h"
//#include "base_main_Rate8_pps_driver_A4B33.h"
//#include "base_main_Rate8_pps_driver_A4B38.h"
#include "base_main_Rate8_pps_driver_A4B43.h"
//#include "base_main_Rate8_pps_driver_PrimaryMic_RefMic.h"
//#include "base_main_Rate8_pps_driver_refMic.h"
//#include "base_main_Rate8_pps_driver_A4B29.h"
/* New Version of Music PFW with Multiple SRS Patches */
//#include "base_main_Rate48_pps_driver_0417v5.h"
//#include "base_main_Rate48_pps_driver_0418v6.h"
//remove two dsp sync for music play
#include "base_main_Rate48_pps_driver_0518v9.h"
#include "Patch_base_main_Rate48_pps_driver.h"
#include "Patch_base_SRSON_Rate48_pps_driver.h"
#include "Patch_base_SRSOFF_Rate48_pps_driver.h"
#include "Patch_base_SRS_1_Rate48_pps_driver.h"
#include "Patch_base_SRS_2_Rate48_pps_driver.h"
#include "Patch_base_SRS_3_Rate48_pps_driver.h"
#include "Patch_base_SRS_4_Rate48_pps_driver.h"
//for bt voice call
#include "base_main_Rate8_BT_R3.h"
#ifdef CONFIG_MINI_DSP

/*
 *****************************************************************************
 * LOCAL STATIC DECLARATIONS
 *****************************************************************************
 */
static int m_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo);
static int m_control_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);
static int m_control_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);

/*
 *****************************************************************************
 * MINIDSP RELATED GLOBALS
 *****************************************************************************
 */
/* The below variable is used to maintain the I2C Transactions
 * to be carried out during miniDSP switching.
 */
minidsp_parser_data dsp_parse_data[MINIDSP_PARSER_ARRAY_SIZE*2];

/* kcontrol structure used to register with ALSA Core layer */
//static struct snd_kcontrol_new snd_mux_controls[MAX_MUX_CONTROLS];

/* mode variables */
static int amode;
static int dmode;

/* k-control macros used for miniDSP related Kcontrols */
#define SOC_SINGLE_VALUE_M(xmax, xinvert) \
	((unsigned long)&(struct soc_mixer_control) \
	{.max = xmax, \
	.invert = xinvert})

#define SOC_SINGLE_AIC3262_M(xname) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = m_control_info, .get = m_control_get,\
	.put = m_control_put, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
}
#define SOC_SINGLE_AIC3262_M2(xname, xmax, xinvert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = m_control_info, .get = m_control_get,\
	.put = m_control_put, \
	.private_value =  ((unsigned long)&(struct soc_mixer_control) \
	{.max = xmax, \
	 .invert = xinvert}) }
/*
 * aic3262_minidsp_controls
 *
 * Contains the list of the Kcontrol macros required for modifying the
 * miniDSP behavior at run-time.
 */
static const char *minidsp_mode[] = {"Music", "one mic nc","Bluetooth",
								"Two mic aec-nc","simple voice call"}; 

/* SOC_ENUM Declaration and kControl for switching Configurations
 * at run-time.
 */
static const struct soc_enum dsp_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(minidsp_mode),
			minidsp_mode);

static const struct snd_kcontrol_new aic3262_minidsp_mode_control[] = {

	SOC_ENUM_EXT("Minidsp mode",
		dsp_enum, m_control_get, m_control_put),

};

static const struct snd_kcontrol_new aic3262_minidsp_controls[] = {
	SOC_SINGLE_AIC3262_M("ADC Adaptive mode Enable") ,
	SOC_SINGLE_AIC3262_M("DAC Adaptive mode Enable") ,
	//SOC_SINGLE_AIC3262_M("Dump Regs Book0") ,
	SOC_SINGLE_AIC3262_M("Verify minidsp program") ,
	SOC_SINGLE_AIC3262_M2("Apply Multiconfig Patch", 10, 0),
	SOC_ENUM_EXT("Minidsp mode",
		dsp_enum, m_control_get, m_control_put),
	SOC_SINGLE_AIC3262_M("Codec Test") ,
	SOC_SINGLE_AIC3262_M("DSP resync"),
};

//directly call hw_write with lock, not cached, so dont use it to write to book 0
int dsp_write(struct snd_soc_codec *codec, u8 book, unsigned int reg, unsigned int val) 
{   
	u8 buf[2];
	int ret = 0;
	//never use this function to write book 0, instead use snd_soc_write
    BUG_ON(book==0);
	mutex_lock(&codec_io_mutex);
	aic3262_change_book(codec, book);
	buf[AIC3262_REG_OFFSET_INDEX] = reg;
	buf[AIC3262_REG_DATA_INDEX] = val;
	if (codec->hw_write(codec, buf, 2) < 0) {
		printk(KERN_ERR "#%s: Error in hw write\n", __func__);
		ret = -EIO;
	}
	aic3262_change_book(codec, 0);	
	mutex_unlock(&codec_io_mutex);
	return ret;
}
unsigned int dsp_read(struct snd_soc_codec *codec, u8 book, unsigned int reg)
{
	u8 val;
	mutex_lock(&codec_io_mutex);
	aic3262_change_book(codec, book);
    val = codec->hw_read(codec, reg);
	aic3262_change_book(codec, 0);	
	mutex_unlock(&codec_io_mutex);
	return val;
}

//directly call hw_write with lock, not cached, so dont use it to write to book 0
int dsp_update_bits(struct snd_soc_codec *codec, u8 book, unsigned short reg,
				unsigned int mask, unsigned int value)
{   
	int change;
	unsigned int old, new;
	int ret = 0;
	u8 buf[2];
	
	BUG_ON(book==0);
	mutex_lock(&codec_io_mutex);
	aic3262_change_book(codec, book);
	ret = codec->hw_read(codec, reg);
	if (ret < 0) 
		goto out;

	old = ret;
	new = (old & ~mask) | value;
	change = old != new;
	if (change) {
		buf[AIC3262_REG_OFFSET_INDEX] = reg;
		buf[AIC3262_REG_DATA_INDEX] = new;
		if (codec->hw_write(codec, buf, 2) < 0) {
			printk(KERN_ERR "#%s: Error in hw write\n", __func__);
			ret = -EIO;
		}
	}

out:  
	
	aic3262_change_book(codec, 0);	
	mutex_unlock(&codec_io_mutex);
	return ret;
}
/*
 *----------------------------------------------------------------------------
 * Function : byte_array_transfer
 * Purpose  : Function used only for debugging purpose. This function will
 *            be used while switching miniDSP Modes register by register.
 *            This needs to be used only during development.
 *-----------------------------------------------------------------------------
 */
int byte_array_transfer(struct snd_soc_codec *codec,
				reg_value *program_ptr,
				int size)
{	
	int j;
	u8 buf[3];
	int ret = 0;
		
	if(codec == NULL || codec->hw_write == NULL) {
		printk(KERN_INFO "%s:codec NULL pointer. Returning error\n", __func__);
		return -1;
	}

	mutex_lock(&codec_io_mutex);
	for (j = 0; j < size; j++) {
		buf[AIC3262_REG_OFFSET_INDEX] = program_ptr[j].reg_off;
		buf[AIC3262_REG_DATA_INDEX] = program_ptr[j].reg_val;

		if (codec->hw_write(codec, buf, 2) < 0) {
			printk(KERN_ERR "#%s: Error in hw write\n", __func__);
			ret = -EIO;
		    goto out;
		}
	}
out:
	aic3262_change_book(codec, 0);
	mutex_unlock(&codec_io_mutex);
	//printk(KERN_INFO "%s: ended\n", __func__);
	return ret;

}

/*
 *----------------------------------------------------------------------------
 * Function : byte_array_read
 * Purpose  : This function is used to perform Byte I2C Read. This is used
 *            only for debugging purposes to read back the Codec Page
 *            Registers after miniDSP Configuration.
 *----------------------------------------------------------------------------
 */
int byte_array_read(struct snd_soc_codec *codec,
			reg_value *program_ptr, int size)
{
	int j;
	u8 val;
	u8 cur_page = 0;
	u8 cur_book = 0;
	
	mutex_lock(&codec_io_mutex);
	for (j = 0; j < size; j++) {
		/* Check if current Reg offset is zero */
		if (program_ptr[j].reg_off == 0) {
			/* Check for the Book Change Request */
			if ((j < (size - 1)) &&
				(program_ptr[j+1].reg_off == 127)) {
				aic3262_change_book(codec,
					program_ptr[j+1].reg_val);
				cur_book = program_ptr[j+1].reg_val;
			/* Increment for loop counter across Book Change */
				j++;
				continue;
			}
			/* Check for the Page Change Request in Current book */
			aic3262_change_page(codec, program_ptr[j].reg_val);
			cur_page = program_ptr[j].reg_val;
			continue;
		}

		val = codec->hw_read(codec, program_ptr[j].reg_off);
		if (val < 0)
			printk(KERN_ERR "Error in smbus read\n");

//		DBG(KERN_INFO "[%d][%d][%d]= %x\n",
//			cur_book, cur_page, program_ptr[j].reg_off, val);
	}
	aic3262_change_book(codec, 0);
	mutex_unlock(&codec_io_mutex);
	return 0;
}


/*
 *----------------------------------------------------------------------------
 * Function : byte_debug_read
 * Purpose  : This function is used to read valules of the miniDSP registers. This is used
 *            only for debugging purposes to read back the Codec Page
 *            Registers after miniDSP Configuration.
 *----------------------------------------------------------------------------
 */
int byte_debug_read(struct snd_soc_codec *codec,
			reg_value *program_ptr,reg_value *read_ptr, int size)
{
	int j;
	u8 cur_page = 0;
	u8 cur_book = 0;
	
	mutex_lock(&codec_io_mutex);
	for (j = 0; j < size; j++) {
		read_ptr[j].reg_off = program_ptr[j].reg_off;
		/* Check if current Reg offset is zero */
		if (program_ptr[j].reg_off == 0) {
			read_ptr[j].reg_val = program_ptr[j].reg_val;
			/* Check for the Book Change Request */
			if ((j < (size - 1)) &&
				(program_ptr[j+1].reg_off == 127)) {
				aic3262_change_book(codec,
					program_ptr[j+1].reg_val);
				cur_book = program_ptr[j+1].reg_val;
			/* Increment for loop counter across Book Change */
				j++;
				read_ptr[j].reg_off = program_ptr[j].reg_off;
			       read_ptr[j].reg_val = program_ptr[j].reg_val;
				continue;
			}
			/* Check for the Page Change Request in Current book */
			aic3262_change_page(codec, program_ptr[j].reg_val);
			cur_page = program_ptr[j].reg_val;
			continue;
		}

		read_ptr[j].reg_val = codec->hw_read(codec, program_ptr[j].reg_off);
		
	//	printk("[%d][%d][%02x]= %02x\n",
	//		cur_book, cur_page, program_ptr[j].reg_off, read_ptr[j].reg_val);
	}
	aic3262_change_book(codec, 0);
	mutex_unlock(&codec_io_mutex);
	return 0;
}


/*
 *----------------------------------------------------------------------------
 * Function : zte_minidsp_get_burst
 * Purpose  : Format one I2C burst for transfer from mini dsp program array.
 *            This function will parse the program array and get next burst
 *            data for doing an I2C bulk transfer.
 *----------------------------------------------------------------------------
 */
static void zte_minidsp_get_burst(reg_value *program_ptr,
				int program_size,
				minidsp_parser_data *parse_data)
{
	int index = parse_data->current_loc;
	int burst_write_count = 0;
	
	parse_data->burst_array[burst_write_count++] =
			program_ptr[index].reg_off;
	parse_data->burst_array[burst_write_count++] =
			program_ptr[index].reg_val;
	index++;
       for (; index < program_size; index++) 
       {
	    if (program_ptr[index].reg_off != (program_ptr[index - 1].reg_off + 1)) {
                 break;
            }
		else{
	          parse_data->burst_array[burst_write_count++] =
				program_ptr[index].reg_val;
	     }
       }
	parse_data->burst_size = burst_write_count;
	if (index == program_size)
		/* parsing completed */
		parse_data->current_loc = MINIDSP_PARSING_END;
	else
		parse_data->current_loc = index;
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_multibyte_transfer
 * Purpose  : Function used to perform multi-byte I2C Writes. Used to configure
 *            the miniDSP Pages.
 *----------------------------------------------------------------------------
 */
int
minidsp_multibyte_transfer(struct snd_soc_codec *codec,
					reg_value *program_ptr,
					int program_size)
{
	minidsp_parser_data parse_data;

	/* point the current location to start of program array */
	parse_data.current_loc = 0;
	parse_data.page_num = 0;
	parse_data.book_change = 0;
	parse_data.book_no = 0;

	mutex_lock(&codec_io_mutex);
	pr_info("%s start \n", __func__);
	do
	{
	    zte_minidsp_get_burst(program_ptr, program_size, &parse_data);
	    codec->hw_write(codec, parse_data.burst_array, parse_data.burst_size);
	} while (parse_data.current_loc != MINIDSP_PARSING_END);
	aic3262_change_book(codec, 0);
    mutex_unlock(&codec_io_mutex);
	pr_info("%s end \n", __func__);
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : set_minidsp_mode
 * Purpose  : Switch to the first minidsp mode.
 *----------------------------------------------------------------------------
 */
struct process_flow{
	int init_size;
	reg_value *miniDSP_init;
	int A_size;
	reg_value *miniDSP_A_values;
	int D_size;
	reg_value *miniDSP_D_values;
	int post_size;
	reg_value *miniDSP_post;
	struct minidsp_config {
		int a_patch_size;
		reg_value *a_patch;
		int d_patch_size;
		reg_value *d_patch;
	} configs[MAXCONFIG];
	
} miniDSP_programs[]  = {
{
	ARRAY_SIZE(base_speaker_SRS_REG_init_Section_program),base_speaker_SRS_REG_init_Section_program,
  	ARRAY_SIZE(base_speaker_SRS_miniDSP_A_reg_values),base_speaker_SRS_miniDSP_A_reg_values,
  	ARRAY_SIZE(base_speaker_SRS_miniDSP_D_reg_values),base_speaker_SRS_miniDSP_D_reg_values,
  	ARRAY_SIZE(base_speaker_SRS_REG_post_Section_program),base_speaker_SRS_REG_post_Section_program,

	{
			#if 1
			{	ARRAY_SIZE(SRS_OFF_miniDSP_A_reg_values),	
				SRS_OFF_miniDSP_A_reg_values,
				ARRAY_SIZE(SRS_OFF_miniDSP_D_reg_values),
				SRS_OFF_miniDSP_D_reg_values},
			/* for SRS diffrent mode*/
			/*	
			{	ARRAY_SIZE(SRS_ON_miniDSP_A_reg_values),	
				SRS_ON_miniDSP_A_reg_values,
				ARRAY_SIZE(SRS_ON_miniDSP_D_reg_values),
				SRS_ON_miniDSP_D_reg_values},
			*/
			{	ARRAY_SIZE(SRS_1_miniDSP_A_reg_values),
				SRS_1_miniDSP_A_reg_values,
				ARRAY_SIZE(SRS_1_miniDSP_D_reg_values),
				SRS_1_miniDSP_D_reg_values},
			{	ARRAY_SIZE(SRS_2_miniDSP_A_reg_values),
				SRS_2_miniDSP_A_reg_values,
				ARRAY_SIZE(SRS_2_miniDSP_D_reg_values),
				SRS_2_miniDSP_D_reg_values},
			{	ARRAY_SIZE(SRS_3_miniDSP_A_reg_values),
				SRS_3_miniDSP_A_reg_values,
				ARRAY_SIZE(SRS_3_miniDSP_D_reg_values),
				SRS_3_miniDSP_D_reg_values},
			{	ARRAY_SIZE(SRS_4_miniDSP_A_reg_values),
				SRS_4_miniDSP_A_reg_values,
				ARRAY_SIZE(SRS_4_miniDSP_D_reg_values),
				SRS_4_miniDSP_D_reg_values},
			/*for speaker*/
			{	ARRAY_SIZE(music_main_miniDSP_A_reg_values),
				music_main_miniDSP_A_reg_values,
				ARRAY_SIZE(music_main_miniDSP_D_reg_values),
				music_main_miniDSP_D_reg_values},
			/* for DAC instruction croption */
			{	0, 0,
				ARRAY_SIZE(base_speaker_SRS_miniDSP_D_reg_values),
				base_speaker_SRS_miniDSP_D_reg_values},
			/* for ADC instruction croption */
			{	0, 0,
				ARRAY_SIZE(base_speaker_SRS_miniDSP_A_reg_values),
				base_speaker_SRS_miniDSP_A_reg_values},
			#else
			{0,	0,	ARRAY_SIZE(SRS_OFF_miniDSP_D_reg_values),
				SRS_OFF_miniDSP_D_reg_values},
			{0,	0,	ARRAY_SIZE(SRS_ON_miniDSP_D_reg_values),
				SRS_ON_miniDSP_D_reg_values},
			{0,0,	ARRAY_SIZE(SPK_ON_miniDSP_D_reg_values),
				SPK_ON_miniDSP_D_reg_values},
			{0,0,	ARRAY_SIZE(SPK_OFF_miniDSP_D_reg_values),
				SPK_OFF_miniDSP_D_reg_values},
			#endif
	},
},
  	{
	ARRAY_SIZE(main44_REG_Section_init_program), main44_REG_Section_init_program,
  	ARRAY_SIZE(main44_miniDSP_A_reg_values),main44_miniDSP_A_reg_values,
  	ARRAY_SIZE(main44_miniDSP_D_reg_values),main44_miniDSP_D_reg_values,
  	ARRAY_SIZE(main44_REG_Section_post_program),main44_REG_Section_post_program,
  	{
  		#if 0
  		{ 0, 0, 0, 0},
  		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		#else
	  		{ ARRAY_SIZE(one_mic_main_miniDSP_A_reg_values), one_mic_main_miniDSP_A_reg_values, ARRAY_SIZE(one_mic_main_miniDSP_D_reg_values), 
			one_mic_main_miniDSP_D_reg_values},
			{ ARRAY_SIZE(one_mic_hp_miniDSP_A_reg_values), one_mic_hp_miniDSP_A_reg_values, ARRAY_SIZE(one_mic_hp_miniDSP_D_reg_values), 
			one_mic_hp_miniDSP_D_reg_values},
			{ ARRAY_SIZE(one_mic_spk_miniDSP_A_reg_values), one_mic_spk_miniDSP_A_reg_values, ARRAY_SIZE(one_mic_spk_miniDSP_D_reg_values), 
			one_mic_spk_miniDSP_D_reg_values},
		{ 0, 0, 0, 0},
		#endif
	},
},
{
	ARRAY_SIZE(Third_Rate_REG_Section_init_program), Third_Rate_REG_Section_init_program,
  	ARRAY_SIZE(Third_Rate_miniDSP_A_reg_values),Third_Rate_miniDSP_A_reg_values,
  	ARRAY_SIZE(Third_Rate_miniDSP_D_reg_values),Third_Rate_miniDSP_D_reg_values,
  	ARRAY_SIZE(Third_Rate_REG_Section_post_program),Third_Rate_REG_Section_post_program,
  	{
  		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		
	},
},

{
	ARRAY_SIZE(aecnc_2mic_REG_Section_init_program), aecnc_2mic_REG_Section_init_program,
  	ARRAY_SIZE(aecnc_2mic_miniDSP_A_reg_values),aecnc_2mic_miniDSP_A_reg_values,
  	ARRAY_SIZE(aecnc_2mic_miniDSP_D_reg_values),aecnc_2mic_miniDSP_D_reg_values,
  	ARRAY_SIZE(aecnc_2mic_REG_Section_post_program),aecnc_2mic_REG_Section_post_program,
  	{
  		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		
	},
},

{
	ARRAY_SIZE(simple_call_REG_Section_init_program), simple_call_REG_Section_init_program,
  	ARRAY_SIZE(simple_call_miniDSP_A_reg_values),simple_call_miniDSP_A_reg_values,
  	ARRAY_SIZE(simple_call_miniDSP_D_reg_values),simple_call_miniDSP_D_reg_values,
  	ARRAY_SIZE(simple_call_REG_Section_post_program),simple_call_REG_Section_post_program,
  	{
  		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		
	},
},
#if 0
{
	ARRAY_SIZE(base_speaker_SRS_REG_init_Section_program),base_speaker_SRS_REG_init_Section_program,
  	ARRAY_SIZE(base_speaker_SRS_miniDSP_A_reg_values),base_speaker_SRS_miniDSP_A_reg_values,
  	ARRAY_SIZE(base_speaker_SRS_miniDSP_D_reg_values),base_speaker_SRS_miniDSP_D_reg_values,
  	ARRAY_SIZE(base_speaker_SRS_REG_post_Section_program),base_speaker_SRS_REG_post_Section_program,
	



	{
			{0,	0,
				ARRAY_SIZE(SRS_ON_miniDSP_D_reg_values),
				SRS_ON_miniDSP_D_reg_values},
			{0,	0,
				ARRAY_SIZE(SRS_OFF_miniDSP_D_reg_values),
				SRS_OFF_miniDSP_D_reg_values},
			{0,0,0,0},
			{0,0,0,0},
	},
},

	{ARRAY_SIZE(spkr_srs_REG_Section_init_program),spkr_srs_REG_Section_init_program,
  	ARRAY_SIZE(spkr_srs_miniDSP_A_reg_values),spkr_srs_miniDSP_A_reg_values,
  	ARRAY_SIZE(spkr_srs_miniDSP_D_reg_values),spkr_srs_miniDSP_D_reg_values,
  	ARRAY_SIZE(spkr_srs_REG_Section_post_program),spkr_srs_REG_Section_post_program,
	{
  		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		{ 0, 0, 0, 0},
		
	},

},
#endif
};

/*function to change minidsp power status
* off_restore: 0 - powers off the minidsp
* 			1 - restores the minidsp to the state indicated by the bit mask
* return: bit mask of old power state of the minidsp
*		<minidspA-right.D4><minidspA-left.D3><minidspD-right.D1><minidspD-left.D0>
* usage: we use the function to stop the minidsp temporarely and later restore it to its former value.
* 		To retore, the onus of maintaining the return value and passing it again is with the calling function
* 		It can also be used by calling the function with absolute bitmask to power on desired minidsp
*/
/*TODO : revisit the logic for checking the power on status of each DAC & ADC*/
int change_minidsp_power_status(struct snd_soc_codec *codec, 
						int off_restore, int power_mask)
{
	int minidsp_power_mask;
	u8 dac_status, adc_status;

	minidsp_power_mask=0;

	switch(off_restore) {

		case 0:/*for powering off*/

				dac_status = snd_soc_read(codec, DAC_FLAG_R1);					
				/*power down dac only if dac is already on*/
				/*left dac*/
				if(dac_status & 0x80){
					minidsp_power_mask |= 0x01;
					snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0x80, 0x00);
				}
				/*right dac*/
				if(dac_status & 0x08){
					minidsp_power_mask |= 0x02;
					snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0x40, 0x00);
				}
				/*power down ADC only if ADC is already running*/
				adc_status=snd_soc_read(codec,ADC_FLAG_R1);
				//pr_info("adc_status: 0x%x\n", adc_status );
				/*left adc*/
				if(adc_status & 0x40) {
					minidsp_power_mask |= 0x04;
					snd_soc_update_bits(codec, ADC_CHANNEL_POW, 0x80, 0x00);
				}
				/*right adc*/
				if(adc_status & 0x04) {
					minidsp_power_mask |= 0x08;
					snd_soc_update_bits(codec, ADC_CHANNEL_POW, 0x40, 0x00);
				}

				/*check power satatus*/
				if(dac_status & 0x80){
					poll_dac(codec, 0, 0);
				}
				if(dac_status & 0x08){
					poll_dac(codec, 1, 0);
				}
				if(adc_status & 0x40) {
					poll_adc(codec, 0, 0);
				}
				if(adc_status & 0x04) {
					poll_adc(codec, 1, 0);
		}
				break;
		case 1:/*for restoring minidsp to former state as per bit mask*/
				if(power_mask & 0x01) {
					snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0x80, 0x80);
				}
				if(power_mask & 0x02) {
					snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0x40, 0x40);
				}
				if(power_mask & 0x04) {
					snd_soc_update_bits(codec, ADC_CHANNEL_POW, 0x80, 0x80);
				}
				if(power_mask & 0x08) {
					snd_soc_update_bits(codec, ADC_CHANNEL_POW, 0x40, 0x40);
				}
				/*check power satatus*/
				if(power_mask & 0x01) {
					poll_dac(codec, 0, 1);
				}
				if(power_mask & 0x02) {
					poll_dac(codec, 1, 1);	
		}
				if(power_mask & 0x04) {
					poll_adc(codec, 0, 1);
				}
				if(power_mask & 0x08) {
					poll_adc(codec, 1, 1);	
	}
				break;
		default:
				printk(KERN_ERR "%s:Unknown power state requested\n", __func__);

	};

	return minidsp_power_mask;

		}

int
set_minidsp_mode(struct snd_soc_codec * codec, int new_mode, int new_config, int force)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	struct process_flow *pflows = &miniDSP_programs[new_mode];
	int minidsp_stat;
	u8 pfw_changed = 0;
	u8 dac_mute = 0;
	//after process flow changed, set this flag to 1, after patched, set to 0; initial is 1;
	static u8 last_patch_invalid = 1;

	
	int (*ptransfer)(struct snd_soc_codec *codec,
				reg_value *program_ptr,
				int size);

	if (new_mode >= ARRAY_SIZE(miniDSP_programs)) {
		printk("%s: new_mode >= ARRAY_SIZE of minidsp\n", __func__);
		return 0; //  error condition
	}
	if (new_config > MAXCONFIG)
		return 0;
	#ifndef MULTIBYTE_TRANS
	ptransfer = byte_array_transfer;
	#else
	ptransfer = minidsp_multibyte_transfer;
	#endif

	mutex_lock(&codec->mutex);
	if (((new_mode !=  aic3262->process_flow) && (new_mode >= 0) )|| force) {

		pr_info( "== From PFW %d to PFW %d==\n", 
		aic3262->process_flow , new_mode);

		minidsp_stat = change_minidsp_power_status(codec, 0, 3);

	      /* I/o mutext used in this ptransfer below */
	 	ptransfer(codec, pflows->miniDSP_init,      pflows->init_size);
		ptransfer(codec, pflows->miniDSP_A_values,  pflows->A_size);
		ptransfer(codec, pflows->miniDSP_D_values,  pflows->D_size);
		ptransfer(codec, pflows->miniDSP_post,      pflows->post_size);

		/* Whenever we switch to Music Mode back from an earlier Voice
		* or BT Call Mode, revert the MINIDSP_SYNC_CTRL_REG back
		* to 0x00.
		* Without this step, further music playback may not work. 
		*/
		printk("%s:  Previous ASI SYNC REG value %x \n",
				__func__, snd_soc_read(codec, (PAGE_4+119)));
              /*new_mode = 0, means music mode, for music mode, miniDSP sync 
              * logic is not required, hence we write 0xF0 to sync reg(P4Reg119).
              * However, after TI Suggested to have Sync on ASI1, the same
              * was changed below to a value of 0x00 which means MiniDSP_D
              * and miniDSP_A will sync on the clock on ASI1 Interface.
              */
		if (new_mode == MODE_MUSIC)  {
			snd_soc_update_bits (codec, MINIDSP_ASI_SYNC_REG, 0xF0, 0x00);
		}
		#if 0
		/* miniDSP sync logic is not required for both the above modes       */
		/* hence we write 0xF0 to sync reg(P4Reg119)  */
		if((new_mode ==MODE_BT_CALL) || (new_mode ==MODE_SIMPLE_CALL))
			snd_soc_update_bits (codec, MINIDSP_ASI_SYNC_REG, 0xF0, 0xF0);
		#endif
		aic3262->process_flow = new_mode;
		pfw_changed = 1;
		last_patch_invalid = 1;

		/*
		*	start sync logic for call process flows
		*/
		/* new_mode = 1,  means spkr/headset mode, new_mode=3 means recvr mode*/
		/* for spkr/headset mode we use, 1micNC+AEC pwf, for recvr mode 2micNC+AEC*/
		/* pfw is used, we need miniDSP sync in these modes, hence we do the following  */
		/* Sync logic.  explanation of sync logic is as below:                                           */
		/* switch off ADC and DAC. Download pfw. Before swithinng on ADC and DAC     */
		/* write 0x66 to B0_P4_R48 so that DAC and ADC wait for sync clock on GPIO4   */
		/* And write 0xA0 to Sync Reg(P4Reg119) so that we wait on clock from ASI3 on GPIO4*/
		/* then 'explicitly' switch ON both ADC and DAC.                                                */
		/* After switching ON ADC and DAC, revert P4Reg119 to 0x50 to sync on ASI2     */
		/* and B0_P4_R48 to 0 which is default value for this register                              */
		#if 0
		if ((new_mode == MODE_ONE_MIC_NC) || (new_mode == MODE_TWO_MIC_AEC_NC)) {
			printk("\nsettting sync logic for voice call\n");
			/* Configure B0_P4_R48 = 0x66 to lock ASI3 on GPI4 */
			snd_soc_update_bits(codec, ASI3_ADC_CNTRL_REG, 0x77, 0x66);
			snd_soc_update_bits (codec, MINIDSP_ASI_SYNC_REG, 0xF0, 0xA0);
			mdelay(5);
		}
		if((aic3262->process_flow == MODE_MUSIC) ||(aic3262->process_flow == MODE_BT_CALL)  ||
			(aic3262->process_flow == MODE_SIMPLE_CALL))
			change_minidsp_power_status(codec, 1, minidsp_stat);
		#else
		if (new_mode != MODE_MUSIC) {
			snd_soc_update_bits(codec, ASI3_ADC_CNTRL_REG, 0x77, 0x66);
			snd_soc_update_bits (codec, MINIDSP_ASI_SYNC_REG, 0xF0, 0xA0);
			mdelay(5);
			}
		if(aic3262->process_flow == MODE_MUSIC)
			change_minidsp_power_status(codec, 1, minidsp_stat);
		#endif
		

		#if 0
		/*
		*	revert & complete sync logic for call process flows
		*/
		if ((new_mode == MODE_ONE_MIC_NC) || (new_mode == MODE_TWO_MIC_AEC_NC)) {
			/*Temporary hack to deal with adc power on. -Review later*/
			if(new_mode == MODE_ONE_MIC_NC) {
				snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0xC0, 0xC0);
				snd_soc_update_bits(codec, ADC_CHANNEL_POW, 0x80, 0x80);
				}
			if(new_mode == MODE_TWO_MIC_AEC_NC) {
				snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0xC0, 0xC0);
				snd_soc_update_bits(codec, ADC_CHANNEL_POW, 0xC0, 0xC0);
			}
			//printk("\n reverting the sync logic\n\n");
			/*Important: The following mdelay is required after switching ON DAC/ADC*/
			/* and before reverting back the Sync registers                                         */
			mdelay(5);
			snd_soc_update_bits(codec, MINIDSP_ASI_SYNC_REG, 0xF0, 0x50);
			snd_soc_update_bits(codec, ASI3_ADC_CNTRL_REG, 0x77, 0x00);
		}
		#else
		if (new_mode != MODE_MUSIC) {
			/*Temporary hack to deal with adc power on. -Review later*/
			//for simple process flow, only use left ADC, including loopback test
			if(new_mode == MODE_ONE_MIC_NC || new_mode == MODE_SIMPLE_CALL || new_mode == MODE_BT_CALL) {
				snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0xC0, 0xC0);
				snd_soc_update_bits(codec, ADC_CHANNEL_POW, 0x80, 0x80);
				}
			else {
				snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0xC0, 0xC0);
				snd_soc_update_bits(codec, ADC_CHANNEL_POW, 0xC0, 0xC0);
			}
			//printk("\n reverting the sync logic\n\n");
			/*Important: The following mdelay is required after switching ON DAC/ADC*/
			/* and before reverting back the Sync registers                                         */
			mdelay(5);
			snd_soc_update_bits(codec, MINIDSP_ASI_SYNC_REG, 0xF0, 0x50);
			snd_soc_update_bits(codec, ASI3_ADC_CNTRL_REG, 0x77, 0x00);
		}
		#endif
	         //printk("%s: ASI SYNC REG value %x", __func__, snd_soc_read(codec, (PAGE_4+119)));
	}

	#ifdef MULTI_CONFIG_SUPPORT
	if (new_config < 0 )
		goto out;// No configs supported in this pfw
		
	/* If the Process-flow has changed, then we have to switch the Configuration. 
	* If there is no change in Process-flow, only then check if the new and old
	* configuration matches or not. 
	*/	
	/*  if set the process flow and patch seprately, pfw_change will be 0, and cannot dectect
	the process flow change
	*/
	if((!last_patch_invalid) && (new_config == aic3262->current_config)){
		pr_info("config are same, exit \n");
		goto out;
	}

	pr_info("apply patch, new cofig: %d, old: %d,process_flow: %d \n", new_config, aic3262->current_config, new_mode);
	/*zuoziqiang add, solve srs switch pop, should also enable soft-stepping */
	dac_mute = snd_soc_read(codec, DAC_MVOL_CONF);
	snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0x03, 0x01);
	snd_soc_update_bits(codec, DAC_MVOL_CONF, 0x0C, 0x0C);

	if (pflows->configs[new_config].a_patch_size || pflows->configs[new_config].d_patch_size){
		//powerdown ADC and dac for music mode, so avoid buffer swap, which 
		//will possbile fail if clocks of ASI is not ready
		#if 1
		int minidsp_stat2 = 0;
		if (aic3262->process_flow == MODE_MUSIC)
		{
			minidsp_stat2 = change_minidsp_power_status(codec, 0, 3);
		}
		#endif
		minidsp_multiconfig(codec, 
			pflows->configs[new_config].a_patch, pflows->configs[new_config].a_patch_size,
			pflows->configs[new_config].d_patch,  pflows->configs[new_config].d_patch_size);
		#if 1
		if (aic3262->process_flow == MODE_MUSIC)
		{
			change_minidsp_power_status(codec, 1, minidsp_stat2);
		}
		#endif
	}
	/*unmuting DAC*/
	snd_soc_update_bits(codec, DAC_MVOL_CONF, 0x0C, dac_mute);
	//re-disable the soft stepping
	snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0x03, 0x02);

	last_patch_invalid = 0;
	#endif

    aic3262->current_config = new_config;
	//aic3262->patched_flow = new_mode;
out:
	mutex_unlock(&codec->mutex);
	return 0;
}

int recover_coeff(struct snd_soc_codec * codec, int new_config)
{
	struct process_flow *pflows = &miniDSP_programs[MODE_MUSIC];
	int minidsp_stat2 = 0;
	minidsp_stat2 = change_minidsp_power_status(codec, 0, 3);
	minidsp_multiconfig(codec, 
			pflows->configs[new_config].a_patch, pflows->configs[new_config].a_patch_size,
			pflows->configs[new_config].d_patch,  pflows->configs[new_config].d_patch_size);
	change_minidsp_power_status(codec, 1, minidsp_stat2);
	return 0;
}
/*
 *----------------------------------------------------------------------------
 * Function : get_current_flow_size
 * Purpose  : Get the size of the current flow,  miniDSP A + D. 
 *----------------------------------------------------------------------------
 */
 
 unsigned long get_current_flow_size(struct snd_soc_codec * codec)
 {
 	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	struct process_flow *pflows = &miniDSP_programs[aic3262->process_flow];

	unsigned long size = 0; 
	size = (pflows->A_size + pflows->D_size) * sizeof(reg_value);
	
	return size;
 }


/*
 *----------------------------------------------------------------------------
 * Function : get_current_flow_arrays
 * Purpose  : Dump the contents of the current flow,  miniDSP A + D. 
 *----------------------------------------------------------------------------
 */
 
 unsigned long  get_current_flow_arrays(struct snd_soc_codec * codec, reg_value *miniDSP_codes_ptr, bool enable_miniDSP)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	struct process_flow *pflows = &miniDSP_programs[aic3262->process_flow];
   	reg_value *buf; 
	int minidsp_stat; 
	unsigned long miniDsp_codes_size = 0;
	miniDsp_codes_size = pflows->A_size + pflows->D_size;
	 buf = miniDSP_codes_ptr;
	 
	mutex_lock(&codec->mutex);

	if (enable_miniDSP)
		minidsp_stat = change_minidsp_power_status(codec, 0, 3);
	
	byte_debug_read(codec, pflows->miniDSP_A_values,  buf, pflows->A_size);
	buf += pflows->A_size;

	byte_debug_read(codec, pflows->miniDSP_D_values,  buf, pflows->D_size);

	if (enable_miniDSP)
		change_minidsp_power_status(codec, 1, minidsp_stat);
	
	mutex_unlock(&codec->mutex);
	
	return miniDsp_codes_size;
}

static void codec_rw_test(struct snd_soc_codec * codec)
{
	int i = 0;
	u8 val1, val2, valold1, valold2;
	unsigned long long cnt = 0;
	pr_info("codec_rw_test \n");
	valold1 = snd_soc_read(codec, DAC_ADC_CLKIN_REG);
	valold2 = snd_soc_read(codec, ASI2_BCLK_N_CNTL);
	while(1){
	cnt++;
	msleep(100);
	pr_info("codec_rw_test, cnt: %lld \n", cnt);
	for(i=0; i< 1000; i++){
		snd_soc_write(codec, DAC_ADC_CLKIN_REG, 1);
		val1 = snd_soc_read(codec, DAC_ADC_CLKIN_REG);
		if(val1 != 1)
		{
			pr_err("1 error rw i: %d.............. \n", i);
			goto out;
		}
		snd_soc_write(codec, ASI2_BCLK_N_CNTL, 1);
		val2 = snd_soc_read(codec, ASI2_BCLK_N_CNTL);
		if(val2 != 1)
		{
			pr_err("2 error rw i: %d..............\n", i);
			goto out;
		}

		snd_soc_write(codec, DAC_ADC_CLKIN_REG, 0);
		val1 = snd_soc_read(codec, DAC_ADC_CLKIN_REG);
		if(val1 != 0)
		{
			pr_err("3 error rw  i: %d..............\n", i);
			goto out;
		}
		snd_soc_write(codec, ASI2_BCLK_N_CNTL, 0);
		val2 = snd_soc_read(codec, ASI2_BCLK_N_CNTL);
		if(val2 != 0)
		{
			pr_err("4 error rw i %d..............\n", i);
			goto out;
		}
	}
	}
out:
	snd_soc_write(codec, DAC_ADC_CLKIN_REG, valold1);
	snd_soc_write(codec, ASI2_BCLK_N_CNTL, valold2);
}
/*
 *----------------------------------------------------------------------------
 * Function : m_control_info
 * Purpose  : This function is to initialize data for new control required to
 *            program the AIC3262 registers.
 *
 *----------------------------------------------------------------------------
 */
static int m_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	#if 1
	int max;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	max = mc->max;
	#endif
	uinfo->count = 1;
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->value.integer.min = 0;
	#if 1
	uinfo->value.integer.max = 10;
	#else
	uinfo->value.integer.max = (long) max;
	#endif
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : m_control_get
 * Purpose  : This function is to read data of new control for
 *            program the AIC3262 registers.
 *
 *----------------------------------------------------------------------------
 */
static int m_control_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	u32 val;
	u8 val1;

	if (!strcmp(kcontrol->id.name, "Minidsp mode")) {
		val = aic3262->process_flow;
		ucontrol->value.integer.value[0] = val;
	}
	if (!strcmp(kcontrol->id.name, "DAC Adaptive mode Enable")) {
		val1 = dsp_read(codec, 80, 1);
		ucontrol->value.integer.value[0] = ((val1>>1)&0x01);
	}
	if (!strcmp(kcontrol->id.name, "ADC Adaptive mode Enable")) {
		val1 = dsp_read(codec, 40, 1);
		ucontrol->value.integer.value[0] = ((val1>>1)&0x01);
	}
	if (!strcmp(kcontrol->id.name, "Apply Multiconfig Patch")) {
		ucontrol->value.integer.value[0] = aic3262->current_config;
	}

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : m_new_control_put
 * Purpose  : new_control_put is called to pass data from user/application to
 *            the driver.
 *
 *----------------------------------------------------------------------------
 */
static int m_control_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	int mode = aic3262->process_flow;
	u32 val;
	#if 0
	struct process_flow *  pflows = &miniDSP_programs[mode];
	#endif
	unsigned long  pfw_chg_j1, pfw_chg_j2;
	
	DBG("n_control_put: %s\n", kcontrol->id.name);
	val = ucontrol->value.integer.value[0];
	if (!strcmp(kcontrol->id.name, "Minidsp mode")) {
		if (val != mode) {
			#ifndef NO_PROCESS_SWTITCH
			pfw_chg_j1 = jiffies;
			set_minidsp_mode(codec, val, -1, 0);
			pfw_chg_j2 = jiffies;
			printk("pfw change time %d\n",
				jiffies_to_msecs(pfw_chg_j2 - pfw_chg_j1));
			#endif
		}
	}

	if (!strcmp(kcontrol->id.name, "DAC Adaptive mode Enable")) {
		if (val != amode) {
			dsp_update_bits(codec, 80, 1, 0xfb, 0x04);
		}
		amode = val;
	}

	if (!strcmp(kcontrol->id.name, "ADC Adaptive mode Enable")) {
		if (val != dmode) {
			dsp_update_bits(codec, 40, 1, 0xfb, 0x04);
		}
		dmode = val;
	}


#if 0
	if (!strcmp(kcontrol->id.name, "Verify minidsp program")) {
		byte_array_read(codec, pflows->miniDSP_init,           pflows->init_size);
		byte_array_read(codec, pflows->miniDSP_A_values,  pflows->A_size);
		byte_array_read(codec, pflows->miniDSP_D_values,  pflows->D_size);
		byte_array_read(codec, pflows->miniDSP_post,         pflows->post_size);
	}
#endif
	if (!strcmp(kcontrol->id.name, "Apply Multiconfig Patch")) {
		set_minidsp_mode(codec, aic3262->process_flow, val, 0);
	}
	if (!strcmp(kcontrol->id.name, "Codec Test")) {
		codec_rw_test(codec);
	}
	if (!strcmp(kcontrol->id.name, "DSP resync")) {
		set_minidsp_mode(codec, aic3262->process_flow, aic3262->current_config, 1);
	}
	return val;
}
#if 0
/************************** MUX CONTROL section *****************************/
/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info_minidsp_mux
 * Purpose  : info routine for mini dsp mux control amixer kcontrols
 *----------------------------------------------------------------------------
 */
static int __new_control_info_minidsp_mux(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_info *uinfo)
{
	int index,index2;
	int ret_val = -1;
	

	for (index = 0; index < ARRAY_SIZE(main44_MUX_controls); index++) {
		if (strstr(kcontrol->id.name, main44_MUX_control_names[index]))
			break;
	}
	if (index < ARRAY_SIZE(main44_MUX_controls))
		{
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = MIN_MUX_CTRL;
		uinfo->value.integer.max = MAX_MUX_CTRL;
		ret_val = 0;
	}

	#if 1
	else{
		printk(" The second rate kcontrol id name is====== %s\n",kcontrol->id.name);


	for (index2 = 0; index < ARRAY_SIZE(base_speaker_SRS_MUX_controls); index2++) {
		if (strstr(kcontrol->id.name, base_speaker_SRS_MUX_control_names[index2]))
			break;
		}
		if (index < ARRAY_SIZE(base_speaker_SRS_MUX_controls))
		{
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = MIN_MUX_CTRL;
		uinfo->value.integer.max = MAX_MUX_CTRL;
		ret_val = 0;
		}
	}
	
	#endif
	
	return ret_val;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get_minidsp_mux
 *
 * Purpose  : get routine for  mux control amixer kcontrols,
 *   read current register values to user.
 *   Used for for mini dsp 'MUX control' amixer controls.
 *----------------------------------------------------------------------------
 */
static int __new_control_get_minidsp_mux(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	
	ucontrol->value.integer.value[0] = kcontrol->private_value;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put_minidsp_mux
 *
 * Purpose  : put routine for amixer kcontrols, write user values to registers
 *            values. Used for for mini dsp 'MUX control' amixer controls.
 *----------------------------------------------------------------------------
 */
static int __new_control_put_minidsp_mux(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u8 data[MUX_CTRL_REG_SIZE + 1];
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int index = 1;
	int user_value = ucontrol->value.integer.value[0];
	u8 value[2], swap_reg_pre, swap_reg_post;
	u8 page;
	int ret_val = -1, array_size;
	control *array;
	char **array_names;
	char *control_name, *control_name1, *control_name2;
	struct aic3262_priv *aic326x = snd_soc_codec_get_drvdata(codec);


	if (aic326x->process_flow == 0) {
		DBG("#the current process flow is %d", aic326x->process_flow);
		array = main44_MUX_controls;
		array_size = ARRAY_SIZE(main44_MUX_controls);
		array_names = main44_MUX_control_names;
		control_name = "Stereo_Mux_TwoToOne_1";
		control_name1 = "Mono_Mux_1_1";
		}

#if 0

		/* Configure only for process flow  1 controls */
		if (strcmp(kcontrol->id.name, control_name) &&
		    strcmp(kcontrol->id.name, control_name1))
			return 0;
	} else {
		array = Second_Rate_MUX_controls;
		array_size = ARRAY_SIZE(Second_Rate_MUX_controls);
		array_names = Second_Rate_MUX_control_names;
		control_name = "Stereo_Mux_TwoToOne_1_Second";
		control_name1 = "Mono_Mux_1_Second";
		control_name2 = "Mono_Mux_4_Second";

		/* Configure only for process flow 2 controls */
		if (strcmp(kcontrol->id.name, control_name1) &&
		    strcmp(kcontrol->id.name, control_name2))
			return 0;
	}

#endif

	page = array[index].control_page;

	DBG("#user value = 0x%x\n", user_value);
	for (index = 0; index < array_size; index++) {
		if (strstr(kcontrol->id.name, array_names[index]))
			break;
	}
	if (index < array_size) {
		DBG(KERN_INFO "#Index %d Changing to Page %d\n", index,
			array[index].control_page);

		aic3262_change_book(codec,
				    array[index].control_book);
		aic3262_change_page(codec,
				    array[index].control_page);

		if (!strcmp(array_names[index], control_name)) {
			if (user_value > 0) {
				data[1] = 0x00;
				data[2] = 0x00;
				data[3] = 0x00;
			} else {
				data[1] = 0xFF;
				data[2] = 0xFf;
				data[3] = 0xFF;
			}
		} else {
			if (user_value > 0) {
				data[1] =
					(u8) ((user_value >> 16) &
					      AIC3262_8BITS_MASK);
				data[2] =
					(u8) ((user_value >> 8) &
					      AIC3262_8BITS_MASK);
				data[3] =
					(u8)((user_value) & AIC3262_8BITS_MASK);
			}
		}
		/* start register address */
		data[0] = array[index].control_base;

		DBG(KERN_INFO
		"#Writing %d %d %d \r\n", data[0], data[1], data[2]);
                ret_val = codec->hw_write(codec, data, MUX_CTRL_REG_SIZE + 1);
		if (ret_val != 0)
			printk(KERN_ERR "__new_control_put_minidsp_mux1 transfer failed\n");
		else {
			/* store the current level */
			kcontrol->private_value = user_value;
			ret_val = 0;
			/* Enable adaptive filtering for ADC/DAC */
		}

		/* Perform a BUFFER SWAP Command. Check if we are currently not
		 * in Page 8, if so, swap to Page 8 first
		 */

		value[0] = 1;
	        value[0] = codec->hw_read(codec, value[0]);
		swap_reg_pre = value[0];
		/* Write the Register bit updates */
		value[1] = value[0] | 1;
		value[0] = 1;
	       if(codec->hw_write(codec, value, 2) != 0)
               {
			printk(KERN_ERR "Can not write register address\n");
               }	

		value[0] = 1;
		/* verify buffer swap */
		swap_reg_post = codec->hw_read(codec, value[0]);
		if ((swap_reg_pre == 4 && swap_reg_post == 6)
		    || (swap_reg_pre == 6 && swap_reg_post == 4))
			DBG("Buffer swap success\n");
		else
			printk(KERN_ERR
			"Buffer swap...FAILED\nswap_reg_pre=%x, \
			swap_reg_post=%x\n", swap_reg_pre, swap_reg_post);

	}
	/* update the new buffer value in the old, just swapped out buffer */
	aic3262_change_book(codec, array[index].control_book);
	aic3262_change_page(codec, array[index].control_page);

	ret_val = codec->hw_write(codec, data, MUX_CTRL_REG_SIZE + 1);
	ret_val = 0;
	aic3262_change_book(codec, 0);
	return ret_val;
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_mux_ctrl_mixer_controls
 *
 * Purpose  : Add amixer kcontrols for mini dsp mux controls,
 *----------------------------------------------------------------------------
 */
static int minidsp_mux_ctrl_mixer_controls(struct snd_soc_codec *codec,
						int size, control *cntl,
						char **name)
{
	int i, err;
	int val1;

	DBG("mixer controls for mini dsp MUX\n");

	if (size) {
		for (i = 0; i < size; i++) {

			snd_mux_controls[i].name = name[i];
			snd_mux_controls[i].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
			snd_mux_controls[i].access =
				SNDRV_CTL_ELEM_ACCESS_READWRITE;
			snd_mux_controls[i].info =
				__new_control_info_minidsp_mux;
			snd_mux_controls[i].get = __new_control_get_minidsp_mux;
			snd_mux_controls[i].put = __new_control_put_minidsp_mux;
			/*
			 *  TBD: read volume reg and update the index number
			 */
			aic3262_change_book(codec, cntl[i].control_book);
			aic3262_change_page(codec, cntl[i].control_page);
			val1 = codec->hw_read(codec, cntl[i].control_base);
			DBG(KERN_INFO "Control data %x\n", val1);
			/*
			if( val1 >= 0 )
				snd_mux_controls[i].private_value = val1;
			else
				snd_mux_controls[i].private_value = 0;
			*/
			DBG(KERN_INFO
				"the value of amixer control mux=%d", val1);
			if (val1 >= 0 && val1 != 255)
				snd_mux_controls[i].private_value = val1;
			else
				snd_mux_controls[i].private_value = 0;

			snd_mux_controls[i].count = 0;

			err = snd_ctl_add(codec->card->snd_card,
				snd_soc_cnew(&snd_mux_controls[i],
					       codec, NULL));
			if (err < 0)
				printk(
					"%s:Invalid control %s\n", __FILE__,
					snd_mux_controls[i].name);
		}
	}
	return 0;
}

/*-------------------------  Volume Controls  -----------------------*/
static int volume_lite_table[] = {

	0x00000D, 0x00000E, 0x00000E, 0x00000F,
	0x000010, 0x000011, 0x000012, 0x000013,
	0x000015, 0x000016, 0x000017, 0x000018,
	0x00001A, 0x00001C, 0x00001D, 0x00001F,
	0x000021, 0x000023, 0x000025, 0x000027,
	0x000029, 0x00002C, 0x00002F, 0x000031,
	0x000034, 0x000037, 0x00003B, 0x00003E,
	0x000042, 0x000046, 0x00004A, 0x00004F,
	0x000053, 0x000058, 0x00005D, 0x000063,
	0x000069, 0x00006F, 0x000076, 0x00007D,
	0x000084, 0x00008C, 0x000094, 0x00009D,
	0x0000A6, 0x0000B0, 0x0000BB, 0x0000C6,
	0x0000D2, 0x0000DE, 0x0000EB, 0x0000F9,
	0x000108, 0x000118, 0x000128, 0x00013A,
	0x00014D, 0x000160, 0x000175, 0x00018B,
	0x0001A3, 0x0001BC, 0x0001D6, 0x0001F2,
	0x000210, 0x00022F, 0x000250, 0x000273,
	0x000298, 0x0002C0, 0x0002E9, 0x000316,
	0x000344, 0x000376, 0x0003AA, 0x0003E2,
	0x00041D, 0x00045B, 0x00049E, 0x0004E4,
	0x00052E, 0x00057C, 0x0005D0, 0x000628,
	0x000685, 0x0006E8, 0x000751, 0x0007C0,
	0x000836, 0x0008B2, 0x000936, 0x0009C2,
	0x000A56, 0x000AF3, 0x000B99, 0x000C49,
	0x000D03, 0x000DC9, 0x000E9A, 0x000F77,
	0x001062, 0x00115A, 0x001262, 0x001378,
	0x0014A0, 0x0015D9, 0x001724, 0x001883,
	0x0019F7, 0x001B81, 0x001D22, 0x001EDC,
	0x0020B0, 0x0022A0, 0x0024AD, 0x0026DA,
	0x002927, 0x002B97, 0x002E2D, 0x0030E9,
	0x0033CF, 0x0036E1, 0x003A21, 0x003D93,
	0x004139, 0x004517, 0x00492F, 0x004D85,
	0x00521D, 0x0056FA, 0x005C22, 0x006197,
	0x006760, 0x006D80, 0x0073FD, 0x007ADC,
	0x008224, 0x0089DA, 0x009205, 0x009AAC,
	0x00A3D7, 0x00B7D4, 0x00AD8C, 0x00C2B9,
	0x00CE43, 0x00DA7B, 0x00E76E, 0x00F524,
	0x0103AB, 0x01130E, 0x01235A, 0x01349D,
	0x0146E7, 0x015A46, 0x016ECA, 0x018486,
	0x019B8C, 0x01B3EE, 0x01CDC3, 0x01E920,
	0x02061B, 0x0224CE, 0x024553, 0x0267C5,
	0x028C42, 0x02B2E8, 0x02DBD8, 0x030736,
	0x033525, 0x0365CD, 0x039957, 0x03CFEE,
	0x0409C2, 0x044703, 0x0487E5, 0x04CCA0,
	0x05156D, 0x05628A, 0x05B439, 0x060ABF,
	0x066666, 0x06C77B, 0x072E50, 0x079B3D,
	0x080E9F, 0x0888D7, 0x090A4D, 0x09936E,
	0x0A24B0, 0x0ABE8D, 0x0B6188, 0x0C0E2B,
	0x0CC509, 0x0D86BD, 0x0E53EB, 0x0F2D42,
	0x101379, 0x110754, 0x1209A3, 0x131B40,
	0x143D13, 0x157012, 0x16B543, 0x180DB8,
	0x197A96, 0x1AFD13, 0x1C9676, 0x1E481C,
	0x201373, 0x21FA02, 0x23FD66, 0x261F54,
	0x28619A, 0x2AC625, 0x2D4EFB, 0x2FFE44,
	0x32D646, 0x35D96B, 0x390A41, 0x3C6B7E,
	0x400000, 0x43CAD0, 0x47CF26, 0x4C106B,
	0x50923B, 0x55586A, 0x5A6703, 0x5FC253,
	0x656EE3, 0x6B7186, 0x71CF54, 0x788DB4,
	0x7FB260,
};

static struct snd_kcontrol_new snd_vol_controls[MAX_VOLUME_CONTROLS];
/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info_main44_minidsp_volume
 * Purpose  : info routine for volumeLite amixer kcontrols
 *----------------------------------------------------------------------------
 */

static int
__new_control_info_minidsp_volume(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	int index, index8;
	int ret_val = -1;

	for (index = 0; index < ARRAY_SIZE(main44_VOLUME_controls); index++) {
		if (strstr
		    (kcontrol->id.name, main44_VOLUME_control_names[index]))
			break;
	}

	for (index8 = 0; index8 < ARRAY_SIZE(base_speaker_SRS_VOLUME_controls);
			index8++) {
		if (strstr
		    (kcontrol->id.name,
		    base_speaker_SRS_VOLUME_control_names[index]))
			break;
	}
 
	if ((index < ARRAY_SIZE(main44_VOLUME_controls))
		
	    || (index8 < ARRAY_SIZE(base_speaker_SRS_VOLUME_controls))) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = MIN_VOLUME;
		uinfo->value.integer.max = MAX_VOLUME;
		ret_val = 0;
	}
	return ret_val;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get_main44_minidsp_vol
 * Purpose  : get routine for amixer kcontrols, read current register
 *         values. Used for for mini dsp 'VolumeLite' amixer controls.
 *----------------------------------------------------------------------------
 */
static int
__new_control_get_minidsp_volume(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = kcontrol->private_value;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put_main44_minidsp_volume
 * Purpose  : put routine for amixer kcontrols, write user values to registers
 *        values. Used for for mini dsp 'VolumeLite' amixer controls.
 *----------------------------------------------------------------------------
 */
static int
__new_control_put_minidsp_volume(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	u8 data[4];
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int user_value = ucontrol->value.integer.value[0];
	int ret_val = -1;
	int coeff;
	u8 value[2], swap_reg_pre, swap_reg_post;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	control *volume_controls = NULL;
	printk(KERN_INFO "user value = 0x%x\n", user_value);

	if (aic3262->process_flow == 0)
		volume_controls = main44_VOLUME_controls;

	else
		volume_controls = base_speaker_SRS_VOLUME_controls;
 

	aic3262_change_book(codec, volume_controls->control_book);
	aic3262_change_page(codec, volume_controls->control_page);

	coeff = volume_lite_table[user_value << 1];

	data[1] = (u8) ((coeff >> 16) & AIC3262_8BITS_MASK);
	data[2] = (u8) ((coeff >> 8) & AIC3262_8BITS_MASK);
	data[3] = (u8) ((coeff) & AIC3262_8BITS_MASK);

	/* Start register address */
	data[0] = volume_controls->control_base;
	ret_val = codec->hw_write(codec, data, VOLUME_REG_SIZE + 1);
	if (ret_val != 0)
		printk(KERN_ERR "__new_control_put_minidsp_volume transfer failed\n");
	else {
		/* store the current level */
		kcontrol->private_value = user_value;
		ret_val = 0;
	}
	/* Initiate buffer swap */
	value[0] = 1;
	value[0] = codec->hw_read(codec, value[0]);
	swap_reg_pre = value[0];
	/* Write the Register bit updates */
	value[1] = value[0] | 1;
	value[0] = 1;
	if(codec->hw_write(codec, value, 2) != 0)
        {
		printk(KERN_ERR "Can not write register address\n");
        }	

	value[0] = 1;
	/* verify buffer swap */
	swap_reg_post = codec->hw_read(codec, value[0]);

	if ((swap_reg_pre == 4 && swap_reg_post == 6)
	    || (swap_reg_pre == 6 && swap_reg_post == 4))
		DBG("Buffer swap success\n");
	else
		DBG("Buffer swap...FAILED\nswap_reg_pre=%x, swap_reg_post=%x\n",
			 swap_reg_pre, swap_reg_post);

	/* update the new buffer value in the old, just swapped out buffer */
	aic3262_change_book(codec, volume_controls->control_book);
	aic3262_change_page(codec, volume_controls->control_page);
	codec->hw_write(codec, data, MUX_CTRL_REG_SIZE + 1);

	aic3262_change_book(codec, 0);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_volume_main44_mixer_controls
 * Purpose  : Add amixer kcontrols for mini dsp volume Lite controls,
 *----------------------------------------------------------------------------
 */
static int minidsp_volume_mixer_controls(struct snd_soc_codec *codec)
{
	int i, err, no_volume_controls;
	static char volume_control_name[MAX_VOLUME_CONTROLS][40];

	/*      ADD first process volume controls       */
	no_volume_controls = ARRAY_SIZE(main44_VOLUME_controls);

	printk(KERN_INFO " %d mixer controls for mini dsp 'volumeLite'\n",
		no_volume_controls);

	if (no_volume_controls) {

		for (i = 0; i < no_volume_controls; i++) {
			strcpy(volume_control_name[i],
			       main44_VOLUME_control_names[i]);
			strcat(volume_control_name[i], VOLUME_KCONTROL_NAME);

			printk(KERN_ERR "Volume controls: %s\n",
				volume_control_name[i]);

			snd_vol_controls[i].name = volume_control_name[i];
			snd_vol_controls[i].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
			snd_vol_controls[i].access =
				SNDRV_CTL_ELEM_ACCESS_READWRITE;
			snd_vol_controls[i].info =
				__new_control_info_minidsp_volume;
			snd_vol_controls[i].get =
				__new_control_get_minidsp_volume;
			snd_vol_controls[i].put =
				__new_control_put_minidsp_volume;
			/*
			 *      TBD: read volume reg and update the index number
			 */
			snd_vol_controls[i].private_value = 0;
			snd_vol_controls[i].count = 0;

			err = snd_ctl_add(codec->card->snd_card,
					  snd_soc_cnew(&snd_vol_controls[i],
						       codec, NULL));
			if (err < 0) {
				printk(KERN_ERR
					"%s:Invalid control %s\n", __FILE__,
					snd_vol_controls[i].name);
			}
		}
	}

	
	/*      ADD second process volume controls      */
	no_volume_controls = ARRAY_SIZE(base_speaker_SRS_VOLUME_controls);

	printk(KERN_ERR " %d mixer controls for mini dsp 'volumeLite'\n",
		no_volume_controls);

	if (no_volume_controls) {

		for (i = 0; i < no_volume_controls; i++) {
			strcpy(volume_control_name[i],
			       base_speaker_SRS_VOLUME_control_names[i]);
			strcat(volume_control_name[i], VOLUME_KCONTROL_NAME);

			printk(KERN_ERR "Volume controls: %s\n",
				volume_control_name[i]);

			snd_vol_controls[i].name = volume_control_name[i];
			snd_vol_controls[i].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
			snd_vol_controls[i].access =
				SNDRV_CTL_ELEM_ACCESS_READWRITE;
			snd_vol_controls[i].info =
				__new_control_info_minidsp_volume;
			snd_vol_controls[i].get =
				__new_control_get_minidsp_volume;
			snd_vol_controls[i].put =
				__new_control_put_minidsp_volume;
			/*
			 *      TBD: read volume reg and update the index number
			 */
			snd_vol_controls[i].private_value = 0;
			snd_vol_controls[i].count = 0;

			err = snd_ctl_add(codec->card->snd_card,
					  snd_soc_cnew(&snd_vol_controls[i],
						       codec, NULL));
			if (err < 0) {
				printk(KERN_ERR
					"%s:Invalid control %s\n", __FILE__,
				       snd_vol_controls[i].name);
			}
		}
	}
	
	return 0;
}
#endif
/*
 *--------------------------------------------------------------------------
 * Function : aic3262_add_minidsp_controls
 * Purpose :  Configures the AMIXER Control Interfaces that can be exercised by
 *            the user at run-time. Utilizes the  the snd_adaptive_controls[]
 *            array to specify two run-time controls.
 *---------------------------------------------------------------------------
 */
int aic3262_add_minidsp_controls(struct snd_soc_codec *codec)
{
#ifdef ADD_MINI_DSP_CONTROLS
	int i, err;
	/* add mode k control */
	for (i = 0; i < ARRAY_SIZE(aic3262_minidsp_controls); i++) {
		err = snd_ctl_add(codec->card->snd_card,
		snd_ctl_new1(&aic3262_minidsp_controls[i], codec));
		if (err < 0) {
			printk(KERN_ERR "Invalid control\n");
			return err;
		}
	}

	#if 0
	/* add mux controls */
	no_mux_controls = ARRAY_SIZE(main44_MUX_controls);
	minidsp_mux_ctrl_mixer_controls(codec, no_mux_controls,
		main44_MUX_controls, main44_MUX_control_names);

	
	no_mux_controls1 = ARRAY_SIZE(base_speaker_SRS_MUX_controls);
	minidsp_mux_ctrl_mixer_controls(codec, no_mux_controls1,
		base_speaker_SRS_MUX_controls, base_speaker_SRS_MUX_control_names);


	/* add volume controls*/
	minidsp_volume_mixer_controls(codec);
	#endif
#endif /* ADD_MINI_DSP_CONTROLS */
	return 0;
}

MODULE_DESCRIPTION("ASoC TLV320AIC3262 miniDSP driver");
MODULE_AUTHOR("Y Preetam Sashank Reddy <preetam@mistralsolutions.com>");
MODULE_LICENSE("GPL");
#endif /* End of CONFIG_MINI_DSP */
