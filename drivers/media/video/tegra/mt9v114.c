/*
 * mt9v114.c - mt9v114 sensor driver
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
#include <media/mt9v114.h>

struct mt9v114_reg {
    u16 addr;
    u16 val;
};

struct mt9v114_info {
    int mode;
    struct i2c_client *i2c_client;
    struct mt9v114_platform_data *pdata;
};

#define MT9V114_TABLE_WAIT_MS 0
#define MT9V114_TABLE_END 1
#define MT9V114_MAX_RETRIES 3
/* ZTE: add by yaoling for add balance,exp function 20110815 ++ */
static struct mt9v114_reg Whitebalance_Auto[] = {
{ 0x3406, 0x00}, 
{0x5183, 0x94 },     
{ 0x5191, 0xFF}, 
{0x5192, 0x00 }, 
{MT9V114_TABLE_END, 0x0000}
};
/* ZTE: add by yaoling for 9v114 brightness 20110930 ++ */
static struct mt9v114_reg  mt9v114_brightness_level1[] = {
    { 0x098E, 0xA01F },
    { 0xA01F, 0x8019},
    {MT9V114_TABLE_END, 0x0000}

};
static  struct mt9v114_reg  mt9v114_brightness_level2[] = {
    { 0x098E, 0xA01F },
    { 0xA01F, 0x8023},
    {MT9V114_TABLE_END, 0x0000}
};
static  struct mt9v114_reg  mt9v114_brightness_level3[] = {
    { 0x098E, 0xA01F },
    { 0xA01F, 0x8037 },
    {MT9V114_TABLE_END, 0x0000}
};
static  struct mt9v114_reg  mt9v114_brightness_level4[] = {
    { 0x098E, 0xA01F},
    { 0xA01F, 0x804B},
    {MT9V114_TABLE_END, 0x0000}
};
static  struct mt9v114_reg   mt9v114_brightness_level5[] = {
    { 0x098E, 0xA01F},
    { 0xA01F, 0x805F},
    {MT9V114_TABLE_END, 0x0000}
};
/* ZTE: add by yaoling for 9v114 brightness 20110930 -- */
//°×³ã
static struct mt9v114_reg Whitebalance_Incandescent[] = {

{MT9V114_TABLE_END, 0x0000}
};
// ÈÕ¹â 
static struct mt9v114_reg Whitebalance_Daylight[] = {

{MT9V114_TABLE_END, 0x0000 }
};
// Ó«¹â
static struct mt9v114_reg Whitebalance_Fluorescent[] = {

{MT9V114_TABLE_END, 0x0000}
};

static struct mt9v114_reg exp_negative2[] = {
 // -1.7EV
    { 0x098e, 0xa01f}, 
    {0xa01f, 0x8023 },     
    {MT9V114_TABLE_END, 0x0000}
};
static struct mt9v114_reg exp_negative1[] = {
  // -1.0EV
    { 0x098e, 0xa01f}, 
    { 0xa01f, 0x802d}, 
    {MT9V114_TABLE_END, 0x0000}
};
static struct mt9v114_reg exp_zero[] = {
    { 0x098e, 0xa01f}, 
    {0xa01f, 0x8037 },     
    
    {MT9V114_TABLE_END, 0x0000}
};
static struct mt9v114_reg exp_one[] = {

    { 0x098e, 0xa01f}, 
    {0xa01f, 0x8041 },     
    {MT9V114_TABLE_END, 0x0000}
};
static struct mt9v114_reg exp_two[] = {

    { 0x098e, 0xa01f}, 
    {0xa01f, 0x804b},     
    {MT9V114_TABLE_END, 0x0000}
};

static struct mt9v114_reg ColorEffect_None[] = { 
    { 0x098e, 0x8400 },
    {0x8400, 0x0101}, 
    { 0xa010, 0x0041 },
    {0x8400, 0x0202}, 
    {MT9V114_TABLE_END, 0x0000}
};

static struct mt9v114_reg ColorEffect_Mono[] = {

        { 0x098e, 0x8400 },
        {0x8400, 0x0101}, 
        {0xa010, 0x0141}, 
        {0x8400, 0x0202}, 
        {MT9V114_TABLE_END, 0x0000}
};

static struct mt9v114_reg ColorEffect_Sepia[] = {
        { 0x098e, 0x8400 },
        {0x8400, 0x0101}, 
        {0xa010, 0x0241}, 
        {0xa012, 0x01ED8}, 
        {0x8400, 0x0202}, 
        {MT9V114_TABLE_END, 0x0000}
};

static struct mt9v114_reg ColorEffect_Negative[] = {
        { 0x098e, 0x8400 },
        {0x8400, 0x0101}, 
        { 0xa010, 0x0341 },
        {0x8400, 0x0202}, 
        {MT9V114_TABLE_END, 0x0000}
};

static struct mt9v114_reg ColorEffect_Solarize[] = {
    { 0x098e, 0x8400 },
    {0x8400, 0x0101}, 
    { 0xa010, 0x0441 },
    {0x8400, 0x0202}, 
    {MT9V114_TABLE_END, 0x0000}
};

//Sensor ISP Not Support this function
static struct mt9v114_reg ColorEffect_Posterize[] = {
   { 0x098e, 0x8400 },
    {0x8400, 0x0101}, 
    { 0xa010, 0x0541 },
    {0x8400, 0x0202}, 
    {MT9V114_TABLE_END, 0x0000}
};
/* ZTE: add by yaoling for add balance,exp function 20110815 -- */
static struct mt9v114_reg mode_640x480[] = {
      
    //[Step1-Reset] //Reset
    {0x001A, 0x0325}, // RESET_AND_MISC_CONTROL
    //DELAY=5 //5ms
    {MT9V114_TABLE_WAIT_MS, 5},
    {0x001A, 0x0124}, // RESET_AND_MISC_CONTROL
    //[Step2-PLL_Timing] //PLL, Timing & Flicker
    //PLL_settings


    {0x0010,0x0116},
    {0x0012,0x0500},
    {0x001e,0x0401},
    {0x0018,0x0006},
    //DELAY=5 //5ms
    {MT9V114_TABLE_WAIT_MS, 5},
    // Required patch, must be done at this point.
    // Delta dark patch
    {0x098A, 0x0000}, // PHYSICAL_ADDRESS_ACCESS
    {0x8082, 0x0194},
    {0x8084, 0x0163},
    {0x8086, 0x0107},
    {0x8088, 0x01C7},
    {0x808A, 0x01A1},
    {0x808C, 0x022A},
    {0x8098, 0x3C3C},
    {0x809A, 0xD600},
    {0x809C, 0xC101},
    {0x809E, 0x2659},
    {0x80A0, 0xCC31},
    {0x80A2, 0x8230},
    {0x80A4, 0xED02},
    {0x80A6, 0xCC00},
    {0x80A8, 0x90ED},
    {0x80AA, 0x00C6},
    {0x80AC, 0x04BD},
    {0x80AE, 0xDDBD},
    {0x80B0, 0xFC02},
    {0x80B2, 0xF330},
    {0x80B4, 0xED00},
    {0x80B6, 0xDC90},
    {0x80B8, 0xBD03},
    {0x80BA, 0x30DD},
    {0x80BC, 0x90FC},
    {0x80BE, 0x02F5},
    {0x80C0, 0x30ED},
    {0x80C2, 0x00DC},
    {0x80C4, 0x92BD},
    {0x80C6, 0x0330},
    {0x80C8, 0xDD92},
    {0x80CA, 0xFC02},
    {0x80CC, 0xF730},
    {0x80CE, 0xED00},
    {0x80D0, 0xDC94},
    {0x80D2, 0xBD03},
    {0x80D4, 0x30DD},
    {0x80D6, 0x94FC},
    {0x80D8, 0x02F3},
    {0x80DA, 0x30ED},
    {0x80DC, 0x00DC},
    {0x80DE, 0x96BD},
    {0x80E0, 0x0330},
    {0x80E2, 0xDD96},
    {0x80E4, 0xCC07},
    {0x80E6, 0xFFDD},
    {0x80E8, 0x8ECC},
    {0x80EA, 0x3180},
    {0x80EC, 0x30ED},
    {0x80EE, 0x02CC},
    {0x80F0, 0x008E},
    {0x80F2, 0xED00},
    {0x80F4, 0xC605},
    {0x80F6, 0xBDDE},
    {0x80F8, 0x13BD},
    {0x80FA, 0xC278},
    {0x80FC, 0x3838},
    {0x80FE, 0x3900},
    {0x8330, 0x3736},
    {0x8332, 0x3C30},
    {0x8334, 0xEC06},
    {0x8336, 0xCE00},
    {0x8338, 0x18BD},
    {0x833A, 0xE451},
    {0x833C, 0x164F},
    {0x833E, 0x30ED},
    {0x8340, 0x00EC},
    {0x8342, 0x02A3},
    {0x8344, 0x00DD},
    {0x8346, 0x8E2C},
    {0x8348, 0x045F},
    {0x834A, 0x4F20},
    {0x834C, 0x02DC},
    {0x834E, 0x8E38},
    {0x8350, 0x3839},
    {0x83E4, 0x3CD6},
    {0x83E6, 0x00C1},
    {0x83E8, 0x0126},
    {0x83EA, 0x0CCC},
    {0x83EC, 0x3180},
    {0x83EE, 0x30ED},
    {0x83F0, 0x00CC},
    {0x83F2, 0x87FF},
    {0x83F4, 0xBDDD},
    {0x83F6, 0xF5BD},
    {0x83F8, 0xC2A9},
    {0x83FA, 0x3839},
    {0x098E, 0x0000 },// LOGICAL_ADDRESS_ACCESS
    //delta dark patch enable
    {0x098A, 0x0000}, // PHYSICAL_ADDRESS_ACCESS
    {0x83E0, 0x0098},
    {0x83E2, 0x03E4},
    {0x098E, 0x0000}, // LOGICAL_ADDRESS_ACCESS
    //Timing_settings]
    {0x3002, 0x000},    //Row Start = 0
    { 0x3004, 0x000},   //Column Start = 0
    {0x3006, 0x1E7},    //Row End = 487
    {0x3008, 0x287},    //Column End = 647
    {0x300A, 0x020B},   //Frame Lines = 523
    {0x300C, 0x02D6},   //Line Length = 726
    {0x3010, 0x0012},   //Sensor_fine_correction = 18
    {0x3012, 0x0008},   //Coarse_IT_Time = 8
    {0x3014, 0x00C0},   //Fine_IT_Time = 192
    {0x3016, 0x1},  //Row Speed = 1
    {0x3040, 0x4041},   //Read Mode = 65
    {MT9V114_TABLE_WAIT_MS, 50},
    {0x98E, 0x1000},
    {0xC800, 0x0280},   //Output Width = 640
    {0xC802, 0x01E0},   //Output Height = 480
    {0xB803, 0x0004},     //FD Zone Height = 4
    {0xC86E, 0x0053},   //FD Period 50Hz = 83
    {0xC870, 0x0045},   //FD Period 60Hz = 69
    {0xC872, 0x0010},   //FD Search F1 50Hz = 16
    {0xC873, 0x0012},   //FD Search F2 50Hz = 18
    {0xC874, 0x0013},   //FD Search F1 60Hz = 19
    {0xC875, 0x0015},   //FD Search F2 60Hz = 21
    {0xC876, 0x000A},  //Max FD Zone 50Hz = 10
    {0xC878, 0x000C},   //Max FD Zone 60Hz = 12
    //[Step3-Recommended] //Char settings
    {0x3E22, 0x3307}, // SAMP_BOOST_ROW
    {0x3ECE, 0x4311}, // DAC_LD_2_3
    {0x3ED0, 0x16AF}, // DAC_LD_4_5
    //[Step4-PGA] //PGA
    //[CWF_95_LSC]
    {0x364A, 0x00F0}, // P_R_P0Q0
    {0x364C, 0x8DEB}, // P_R_P0Q1
    {0x364E, 0x7B0E}, // P_R_P0Q2
    {0x3650, 0x7CC8}, // P_R_P0Q3
    {0x3652, 0x9D0C}, // P_R_P0Q4
    {0x368A, 0x2E8A}, // P_R_P1Q0
    {0x368C, 0x5AEB}, // P_R_P1Q1
    {0x368E, 0xC14C}, // P_R_P1Q2
    {0x3690, 0xB3AF}, // P_R_P1Q3
    {0x3692, 0xD2AE}, // P_R_P1Q4
    {0x36CA, 0x4CF0}, // P_R_P2Q0
    {0x36CC, 0x1670 },// P_R_P2Q1
    {0x36CE, 0x024E}, // P_R_P2Q2
    {0x36D0, 0x8DF3}, // P_R_P2Q3
    {0x36D2, 0x80D3}, // P_R_P2Q4
    {0x370A, 0xDBEC },// P_R_P3Q0
    {0x370C, 0x8F70}, // P_R_P3Q1
    {0x370E, 0xAC8D}, // P_R_P3Q2
    {0x3710, 0x7872}, // P_R_P3Q3
    {0x3712, 0x5C12}, // P_R_P3Q4
    {0x374A, 0x9E0E}, // P_R_P4Q0
    {0x374C, 0x80F2}, // P_R_P4Q1
    {0x374E, 0x1FD1}, // P_R_P4Q2
    {0x3750, 0x13D5}, // P_R_P4Q3
    {0x3752, 0x0435 },// P_R_P4Q4
    {0x3640, 0x00F0 },// P_G1_P0Q0
    {0x3642, 0xD40B}, // P_G1_P0Q1
    {0x3644, 0x46EE}, // P_G1_P0Q2
    {0x3646, 0xB56D}, // P_G1_P0Q3
    {0x3648, 0xE92D}, // P_G1_P0Q4
    {0x3680, 0x1DE9}, // P_G1_P1Q0
    {0x3682, 0x8CAC}, // P_G1_P1Q1
    {0x3684, 0x4D2A}, // P_G1_P1Q2
    {0x3686, 0xABAD}, // P_G1_P1Q3
    {0x3688, 0xAAEC}, // P_G1_P1Q4
    {0x36C0, 0x3E30}, // P_G1_P2Q0
    {0x36C2, 0x46D0}, // P_G1_P2Q1
    {0x36C4, 0xE36F}, // P_G1_P2Q2
    {0x36C6, 0xD153}, // P_G1_P2Q3
    {0x36C8, 0xA9B3}, // P_G1_P2Q4
    {0x3700, 0x898D}, // P_G1_P3Q0
    {0x3702, 0x076D}, // P_G1_P3Q1
    {0x3704, 0xC5EE}, // P_G1_P3Q2
    {0x3706, 0x64D1 },// P_G1_P3Q3
    {0x3708, 0x0912}, // P_G1_P3Q4
    {0x3740, 0x3C8D}, // P_G1_P4Q0
    {0x3742, 0x98D2}, // P_G1_P4Q1
    {0x3744, 0x3BB2}, // P_G1_P4Q2
    {0x3746, 0x6935}, // P_G1_P4Q3
    {0x3748, 0x4975}, // P_G1_P4Q4
    {0x3654, 0x0150}, // P_B_P0Q0
    {0x3656, 0x982C },// P_B_P0Q1
    {0x3658, 0x5D4E}, // P_B_P0Q2
    {0x365A, 0x8BAD}, // P_B_P0Q3
    {0x365C, 0xA84E}, // P_B_P0Q4
    {0x3694, 0x0E0A}, // P_B_P1Q0
    {0x3696, 0x5E4C}, // P_B_P1Q1
    {0x3698, 0x9E8C}, // P_B_P1Q2
    { 0x369A, 0xEA2F}, // P_B_P1Q3
    {0x369C, 0x9F6F}, // P_B_P1Q4
    {0x36D4, 0x4CB0}, // P_B_P2Q0
    {0x36D6, 0x5B6F}, // P_B_P2Q1
    {0x36D8, 0xA92F}, // P_B_P2Q2
    {0x36DA, 0x9B33}, // P_B_P2Q3
    {0x36DC, 0x8973}, // P_B_P2Q4
    {0x3714, 0x852C}, // P_B_P3Q0
    {0x3716, 0xA5B0}, // P_B_P3Q1
    {0x3718, 0x054C}, // P_B_P3Q2
    {0x371A, 0x2B53}, // P_B_P3Q3
    {0x371C, 0x2173}, // P_B_P3Q4
    {0x3754, 0xF6EF}, // P_B_P4Q0
    {0x3756, 0xF8B1}, // P_B_P4Q1
    {0x3758, 0x1E12}, // P_B_P4Q2
    {0x375A, 0x3DD5}, // P_B_P4Q3
    {0x375C, 0x2F75}, // P_B_P4Q4
    {0x365E, 0x0490}, // P_G2_P0Q0
    {0x3660, 0xB2CB}, // P_G2_P0Q1
    {0x3662, 0x524E}, // P_G2_P0Q2
    {0x3664, 0xBCAD}, // P_G2_P0Q3
    {0x3666, 0xEC4D}, // P_G2_P0Q4
    {0x369E, 0x1586}, // P_G2_P1Q0
    {0x36A0, 0xBDAB}, // P_G2_P1Q1
    {0x36A2, 0x6F2C}, // P_G2_P1Q2
    {0x36A4, 0x9CAD}, // P_G2_P1Q3
    {0x36A6, 0x8A6D },// P_G2_P1Q4
    {0x36DE, 0x4650 },// P_G2_P2Q0
    {0x36E0, 0x5350 },// P_G2_P2Q1
    {0x36E2, 0xFE4F },// P_G2_P2Q2
    {0x36E4, 0xD6D3 },// P_G2_P2Q3
    {0x36E6, 0xAC53 },// P_G2_P2Q4
    {0x371E, 0xEA6B}, // P_G2_P3Q0
    {0x3720, 0x0269}, // P_G2_P3Q1
    {0x3722, 0x85B0 },// P_G2_P3Q2
    {0x3724, 0x60B1 },// P_G2_P3Q3
    {0x3726, 0x1992}, // P_G2_P3Q4
    {0x375E, 0x1FCC}, // P_G2_P4Q0
    {0x3760, 0xAAB2}, // P_G2_P4Q1
    {0x3762, 0x4AF2 },// P_G2_P4Q2
    {0x3764, 0x7AB5}, // P_G2_P4Q3
    {0x3766, 0x57F5}, // P_G2_P4Q4
    {0x3784, 0x01C4}, // CENTER_COLUMN
    {0x3782, 0x0108}, // CENTER_ROW
    { 0x3210, 0x00B8}, //PGA_ENABLE
    //[Step5-AWB_CCM] //AWB & CCM
    //CCM
    {0x098E, 0x202F}, // LOGICAL_ADDRESS_ACCESS [CAM_AWB_CONFIG_CCM_L_0]
    {0xA02F, 0x0279}, // CAM_AWB_CONFIG_CCM_L_0
    {0xA031, 0xFE68}, // CAM_AWB_CONFIG_CCM_L_1
    {0xA033, 0x0020 },// CAM_AWB_CONFIG_CCM_L_2
    {0xA035, 0xFFB3}, // CAM_AWB_CONFIG_CCM_L_3
    {0xA037, 0x0167}, // CAM_AWB_CONFIG_CCM_L_4
    {0xA039, 0xFFE6 },// CAM_AWB_CONFIG_CCM_L_5
    {0xA03B, 0xFF55}, // CAM_AWB_CONFIG_CCM_L_6
    { 0xA03D, 0xFE86}, // CAM_AWB_CONFIG_CCM_L_7
    {0xA03F, 0x0325 },// CAM_AWB_CONFIG_CCM_L_8
    {0xA041, 0x0021 },// CAM_AWB_CONFIG_CCM_L_9
    {0xA043, 0x004A}, // CAM_AWB_CONFIG_CCM_L_10
    {0xA045, 0xFF97}, // CAM_AWB_CONFIG_CCM_RL_0
    {0xA047, 0x00C0 },// CAM_AWB_CONFIG_CCM_RL_1
    {0xA049, 0xFFAA}, // CAM_AWB_CONFIG_CCM_RL_2
    {0xA04B, 0xFFF0}, // CAM_AWB_CONFIG_CCM_RL_3
    {0xA04D, 0x0006}, // CAM_AWB_CONFIG_CCM_RL_4
    { 0xA04F, 0x000A}, // CAM_AWB_CONFIG_CCM_RL_5
    {0xA051, 0x0096}, // CAM_AWB_CONFIG_CCM_RL_6
    {0xA053, 0x00E5}, // CAM_AWB_CONFIG_CCM_RL_7
    {0xA055, 0xFE85}, // CAM_AWB_CONFIG_CCM_RL_8
    { 0xA057, 0x0010}, // CAM_AWB_CONFIG_CCM_RL_9
    {0xA059, 0xFFDD}, // CAM_AWB_CONFIG_CCM_RL_10
    //AWB
    {0x098E, 0xA065}, // LOGICAL_ADDRESS_ACCESS [CAM_AWB_CONFIG_X_SCALE]
    { 0xA065, 0x0402}, // CAM_AWB_CONFIG_X_SCALE&&&CAM_AWB_CONFIG_Y_SCALE
    {0x2112, 0x2855}, // AWB_WEIGHT_R0
    {0x2114, 0x1C0E}, // AWB_WEIGHT_R1
    {0x2116, 0x9A82 },// AWB_WEIGHT_R2
    {0x2118, 0x9D4A}, // AWB_WEIGHT_R3
    {0x211A, 0x80D4}, // AWB_WEIGHT_R4
    { 0x211C, 0x0EC9}, // AWB_WEIGHT_R5
    {0x211E, 0x8F9B}, // AWB_WEIGHT_R6
    {0x2120, 0x462B}, // AWB_WEIGHT_R7
    {0xA061, 0x0039}, // CAM_AWB_CONFIG_X_SHIFT_PRE_ADJ
    {0xA063, 0x0032}, // CAM_AWB_CONFIG_Y_SHIFT_PRE_ADJ
    {0x9416, 0x4060}, // AWB_R_SCENE_RATIO_LOWER&&&AWB_R_SCENE_RATIO_UPPER
    {0x9418, 0x174F },// AWB_B_SCENE_RATIO_LOWER&&&AWB_B_SCENE_RATIO_UPPER
    //[Step6-CPIPE_Calibration] //Color Pipe Calibration settings, if any
    //[Step7-CPIPE_Preference] //Color Pipe preference settings, if any
    //NR
    {0x326E, 0x0006}, // LOW_PASS_YUV_FILTER
    {0x33F4, 0x000B },// KERNEL_CONFIG
    //GAMMA
    {0x098E, 0xA087}, // LOGICAL_ADDRESS_ACCESS [CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_0]
    {0xA087, 0x0007}, // CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_0&&&CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_1
    {0xA089, 0x1630}, // CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_2&&&CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_3
    {0xA08B, 0x526D}, // CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_4&&&CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_5
    {0xA08D, 0x869B}, // CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_6&&&CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_7
    { 0xA08F, 0xABB9}, // CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_8&&&CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_9
    {0xA091, 0xC5CF}, // CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_10&&&CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_11
    {0xA093, 0xD8E0}, // CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_12&&&CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_13
    {0xA095, 0xE7EE}, // CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_14&&&CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_15
    {0xA097, 0xF4FA}, //CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_16&&&CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_17
    {0xA0AD, 0x0001}, // CAM_LL_CONFIG_GAMMA_START_BM
    {0xA0AF, 0x0338}, // CAM_LL_CONFIG_GAMMA_STOP_BM
    ////AE Preferred
    {0x098E, 0xA0B1}, // LOGICAL_ADDRESS_ACCESS [CAM_LL_CONFIG_NR_RED_START]
    {0xA0B1, 0x102D },// CAM_LL_CONFIG_NR_RED_START&&&CAM_LL_CONFIG_NR_RED_STOP
    {0xA0B3, 0x102D}, // CAM_LL_CONFIG_NR_GREEN_START&&&CAM_LL_CONFIG_NR_GREEN_STOP
    {0xA0B5, 0x102D}, // CAM_LL_CONFIG_NR_BLUE_START&&&CAM_LL_CONFIG_NR_BLUE_STOP
    {0xA0B7, 0x102D}, // CAM_LL_CONFIG_NR_MIN_MAX_START&&&CAM_LL_CONFIG_NR_MIN_MAX_STOP
    {0xA0B9, 0x0040}, // CAM_LL_CONFIG_START_GAIN_METRIC
    {0xA0BB, 0x00C8}, // CAM_LL_CONFIG_STOP_GAIN_METRIC
    {0xA07A, 0x040F}, // CAM_LL_CONFIG_AP_THRESH_START&&&CAM_LL_CONFIG_AP_THRESH_STOP
    {0xA07C, 0x0300 },// CAM_LL_CONFIG_AP_GAIN_START&&&CAM_LL_CONFIG_AP_GAIN_STOP
    {0xA07E, 0x0078}, // CAM_LL_CONFIG_CDC_THRESHOLD_BM
    {0xA080, 0x050F}, // CAM_LL_CONFIG_CDC_GATE_PERCENTAGE&&&CAM_LL_CONFIG_DM_EDGE_TH_START
    {0xA081, 0x0F28}, //CAM_LL_CONFIG_DM_EDGE_TH_START&&&CAM_LL_CONFIG_DM_EDGE_TH_STOP
    {0xA083, 0x000F },// CAM_LL_CONFIG_FTB_AVG_YSUM_START
    {0xA085, 0x0000}, // CAM_LL_CONFIG_FTB_AVG_YSUM_STOP
    {0xA020, 0x4B00 },// CAM_AE_CONFIG_BASE_TARGET&&&CAM_AE_CONFIG_MIN_VIRT_INT_TIME_HIGHBYTE
    {0xA027, 0x0050 },// CAM_AE_CONFIG_MIN_VIRT_AGAIN
    {0xA029, 0x00A8 },// CAM_AE_CONFIG_MAX_VIRT_AGAIN
    {0xA025, 0x0080}, // CAM_AE_CONFIG_MAX_VIRT_DGAIN
    {0xA01C, 0x004B },// CAM_AE_CONFIG_TARGET_AGAIN
    {0xA01E, 0x0080 },// CAM_AE_CONFIG_TARGET_DGAIN
    {0xA01A, 0x0018}, // CAM_AE_CONFIG_TARGET_FDZONE
    {0xA05F, 0x8C37 },// CAM_AWB_CONFIG_START_SATURATION&&&CAM_AWB_CONFIG_END_SATURATION
    {0xA05B, 0x0005}, // CAM_AWB_CONFIG_START_BRIGHTNESS_BM
    { 0xA05D, 0x000A },// CAM_AWB_CONFIG_STOP_BRIGHTNESS_BM
    {0x9C01, 0x3202}, // LL_GAMMA_MANUAL_CONTRAST_PERC&&&LL_GAMMA_SELECT
    {0x9001, 0x0564 },// AE_TRACK_MODE&&&AE_TRACK_SPOT_PERCENTAGE
    {0x9007, 0x0A09 },// AE_TRACK_TARGET_GATE&&&AE_TRACK_WEIGHT_TABLE_0
    {0x9003, 0x2D02 },// AE_TRACK_BLACK_LEVEL_MAX&&&AE_TRACK_BLACK_LEVEL_STEP_SIZE
    {0x9005, 0x2308 },// AE_TRACK_BLACK_CLIP_PERCENT&&&AE_TRACK_BLACK_GATE_PERCENT
    {0x8C03, 0x0103}, // FD_STAT_MIN&&&FD_STAT_MAX
    {0x8C04, 0x0305 },// FD_STAT_MAX&&&FD_MIN_AMPLITUDE
    //[Step8-Features] //Ports, special features, etc., if any
    //EXIT Standby
    {0x0018, 0x0002 },// STANDBY_CONTROL_AND_STATUS
    //DELAY=100 //100ms
        {MT9V114_TABLE_WAIT_MS, 100},
    {MT9V114_TABLE_END, 0x0000}


};
      


enum {
    MT9V114_MODE_680x480,
};

static struct mt9v114_reg *mode_table[] = {
    [MT9V114_MODE_680x480] = mode_640x480,
};

static int mt9v114_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
        int err;
        struct i2c_msg msg[2];
        unsigned char data[4];

        if (!client->adapter)
        return -ENODEV;

        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].len = 2;
        msg[0].buf = data;

        /* high byte goes out first */
        data[0] = (u8) (addr >> 8);
        data[1] = (u8) (addr & 0xff);

        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = 2;
        msg[1].buf = data + 2;


        err = i2c_transfer(client->adapter, msg, 2);

        if (err != 2)
        return -EINVAL;

        *val = data[2] << 8 | data[3];

        return 0;
}

static int mt9v114_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
        int err;
        struct i2c_msg msg;
        unsigned char data[4];
        int retry = 0;

        if (!client->adapter)
        return -ENODEV;

        data[0] = (u8) (addr >> 8);
        data[1] = (u8) (addr & 0xff);
        data[2] = (u8) (val >> 8);
        data[3] = (u8) (val & 0xff);

        msg.addr = client->addr;
        msg.flags = 0;
        msg.len = 4;
        msg.buf = data;

        do {
            err = i2c_transfer(client->adapter, &msg, 1);
            if (err == 1)
            return 0;
            retry++;
            pr_err("mt9v114: i2c transfer failed, retrying %x %x\n",
            addr, val);
            msleep(3);
        } while (retry <= MT9V114_MAX_RETRIES);

        return err;
}

static int mt9v114_write_table(struct i2c_client *client,
                const struct mt9v114_reg table[],
                const struct mt9v114_reg override_list[],
                int num_override_regs)
{
        int err;
        const struct mt9v114_reg *next;
        int i;
        u16 val;

        for (next = table; next->addr != MT9V114_TABLE_END; next++) {
            if (next->addr == MT9V114_TABLE_WAIT_MS) {
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

        err = mt9v114_write_reg(client, next->addr, val);
            if (err)
            return err;
        }
        return 0; 
}

static int mt9v114_set_mode(struct mt9v114_info *info, struct mt9v114_mode *mode)
{
        int sensor_mode;
        int err;
        u16 status;
     
        pr_info("%s: xres %u yres %u\n", __func__, mode->xres, mode->yres);
        if (mode->xres == 640 && mode->yres == 480)
            sensor_mode = MT9V114_MODE_680x480;
        else {
            pr_err("%s: invalid resolution supplied to set mode %d %d\n",
                    __func__, mode->xres, mode->yres);
            return -EINVAL;
        }
       msleep(100);

        {
        mt9v114_read_reg(info->i2c_client, 0x0000, &status);
      
        }
       printk("%s 380_read_reg 1%x\n", __func__, status);
        err = mt9v114_write_table(info->i2c_client, mode_table[sensor_mode],
            NULL, 0);
        if (err)
            return err;

        info->mode = sensor_mode;
        return 0;
}

static int mt9v114_get_status(struct mt9v114_info *info,
            struct mt9v114_status *dev_status)
{
    return 0;
}

static long mt9v114_ioctl(struct file *file,
        unsigned int cmd, unsigned long arg)
{
        int err;
        struct mt9v114_info *info = file->private_data;
        printk("yuv mt9v114_ioctl cmd %d\n",cmd);
        switch (cmd) {
        case MT9V114_IOCTL_SET_MODE:
        {
            struct mt9v114_mode mode;
                 pr_info("YAOLING mt9v114_ioctl case MT9V114_IOCTL_SET_MODE\n");
            if (copy_from_user(&mode,
                        (const void __user *)arg,
                            sizeof(struct mt9v114_mode))) {
                       pr_info("YAOLING mt9v114_ioctl case MT9V114_IOCTL_SET_MODE ERR\n");     
                    return -EFAULT;
                }

                return mt9v114_set_mode(info, &mode);
        }
        /* ZTE: add by yaoling for exposure function 20110812 ++ */
        case MT9V114_IOCTL_SET_COLOR_EFFECT:
        {
            u8 coloreffect;

            if (copy_from_user(&coloreffect,
                (const void __user *)arg,
                sizeof(coloreffect))) {
                pr_info("MT9V114_IOCTL_SET_COLOR_EFFECT COPY  ERR\n");
                return -EFAULT;
            }
            pr_info("yuv coloreffect %d\n",coloreffect);

            switch(coloreffect)
            {
                case MT9V114_ColorEffect_None:
                    err = mt9v114_write_table(info->i2c_client, ColorEffect_None,NULL, 0);
                    break;
                case MT9V114_ColorEffect_Mono:
                    err = mt9v114_write_table(info->i2c_client, ColorEffect_Mono,NULL, 0);
                    break;
                case MT9V114_ColorEffect_Sepia:
                    err = mt9v114_write_table(info->i2c_client, ColorEffect_Sepia,NULL, 0);
                    break;
                case MT9V114_ColorEffect_Negative:
                    err = mt9v114_write_table(info->i2c_client, ColorEffect_Negative,NULL, 0);
                    break;
                case MT9V114_ColorEffect_Solarize:
                    err = mt9v114_write_table(info->i2c_client, ColorEffect_Solarize,NULL, 0);
                    break;
                case MT9V114_ColorEffect_Posterize:
                err = mt9v114_write_table(info->i2c_client, ColorEffect_Posterize,NULL, 0);
                    break;
                default:
                    break;
            }

            if (err)
            {
            //pr_info("yuv coloreffect ERR%d\n");
            return err;
            }
            /* ZTE: add by yaoling for switch effect  20110818 */
            msleep(50);
            /* ZTE: add by yaoling for switch effect  20110818 */
            return 0;
        }
         case MT9V114_IOCTL_SET_WHITE_BALANCE:
        {
            u8 whitebalance;

            pr_info("yuv whitebalance %lu\n",arg);
            if (copy_from_user(&whitebalance,
                (const void __user *)arg,
                sizeof(whitebalance))) {
                pr_info("yuv whitebalance ERR\n");	   
                return -EFAULT;
            }
            pr_info("yuv whitebalance %d\n",whitebalance);

            switch(whitebalance)
            {
                case MT9V114_Whitebalance_Auto:
                    err = mt9v114_write_table(info->i2c_client, Whitebalance_Auto,NULL, 0);
                    break;
                case MT9V114_Whitebalance_Incandescent:
                    err = mt9v114_write_table(info->i2c_client, Whitebalance_Incandescent,NULL, 0);
                    break;
                case MT9V114_Whitebalance_Daylight:
                    err = mt9v114_write_table(info->i2c_client, Whitebalance_Daylight,NULL, 0);
                    break;
                case MT9V114_Whitebalance_Fluorescent:
                    err = mt9v114_write_table(info->i2c_client, Whitebalance_Fluorescent,NULL, 0);
                    break;
                default:
                    break;
            }

            if (err)
            {
                pr_info("yuv whitebalance set err %d\n",err);
                return err;
            }

            return 0;
        }
      

         case MT9V114_IOCTL_SET_EXPOSURE:
        {
            pr_info("yuv EXP %lu\n",arg);
            switch(arg)
            {
                case MT9V114_Exposure_0:
                    pr_info("yuv SET_EXPOSURE 0\n");
                    err = mt9v114_write_table(info->i2c_client, exp_zero,NULL, 0);
                    break;
                case MT9V114_Exposure_1:
                    pr_info("yuv SET_EXPOSURE 1\n");
                    err = mt9v114_write_table(info->i2c_client, exp_one,NULL, 0);
                    break;
                case MT9V114_Exposure_2:
                    pr_info("yuv SET_EXPOSURE 2\n");
                    err = mt9v114_write_table(info->i2c_client, exp_two,NULL, 0);
                    break;
                case MT9V114_Exposure_Negative_1:
                    pr_info("yuv SET_EXPOSURE -1\n");
                    err = mt9v114_write_table(info->i2c_client, exp_negative1,NULL, 0);
                    break;
                case MT9V114_Exposure_Negative_2:
                    pr_info("yuv SET_EXPOSURE -2\n");
                    err = mt9v114_write_table(info->i2c_client, exp_negative2,NULL, 0);
                    break;
                default:
                    break;
            }
             if (err)
            {
              //  pr_info("yuv exp set err %d\n",err);
                return err;
            }

            return 0;
            
        }
          /* ZTE: add by yaoling for exposure function 20110812 -- */
           /* ZTE: add by yaoling for 9v114 brightness function 20110928 ++ */
         case MT9V114_IOCTL_SET_BRIGHTNESS:
        {
            u8 brightness;
            if (copy_from_user(&brightness,
                (const void __user *)arg,
                sizeof(brightness))) {
                return -EFAULT;
            }
            switch(brightness)
            {
                case mt9v114_Brightness_Level1:
                    printk("yuv SET_Brightness 0\n");
                    err = mt9v114_write_table(info->i2c_client, mt9v114_brightness_level1,NULL, 0);
                    if(err)
                        return err;
                    break;
                case mt9v114_Brightness_Level2:
                    printk("yuv SET_Brightness 1\n");
                    err = mt9v114_write_table(info->i2c_client, mt9v114_brightness_level2,NULL, 0);
                    if(err)
                        return err;
                    break;
                case mt9v114_Brightness_Level3:
                    printk("yuv SET_Brightness 2\n");
                    err = mt9v114_write_table(info->i2c_client, mt9v114_brightness_level3,NULL, 0);
                    if(err)
                        return err;
                    break;
                case mt9v114_Brightness_Level4:
                    printk("yuv SET_Brightness 3\n");
                    err = mt9v114_write_table(info->i2c_client, mt9v114_brightness_level4,NULL, 0);
                    if(err)
                        return err;
                    break;
                case mt9v114_Brightness_Level5:
                    printk("yuv SET_Brightness 4\n");
                    err = mt9v114_write_table(info->i2c_client, mt9v114_brightness_level5,NULL, 0);
                    if(err)
                        return err;
                    break;
                default:
                    break;
            }
            return 0;
        }
           /* ZTE: add by yaoling for 9v114 brightness function 20110928 -- */
        case MT9V114_IOCTL_GET_STATUS:
        {
                return 0;
        }
        default:
             return -EFAULT;


        }
     
}

static struct mt9v114_info *info;

static int mt9v114_open(struct inode *inode, struct file *file)
{
        struct mt9v114_status dev_status;
        int err;
        pr_info("mt9v114_open\n");
        file->private_data = info;
        if (info->pdata && info->pdata->power_on)
            info->pdata->power_on();

        dev_status.data = 0;
        dev_status.status = 0;
        err = mt9v114_get_status(info, &dev_status);
        return err;
}

int mt9v114_release(struct inode *inode, struct file *file)
{
        pr_info("mt9v114_release\n");
        if (info->pdata && info->pdata->power_off)
                info->pdata->power_off();
        file->private_data = NULL;
        return 0;
}

static const struct file_operations mt9v114_fileops = {
        .owner = THIS_MODULE,
        .open = mt9v114_open,
        .unlocked_ioctl = mt9v114_ioctl,
        .release = mt9v114_release,
};

static struct miscdevice mt9v114_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "mt9v114",
        .fops = &mt9v114_fileops,
};

static int mt9v114_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
        int err;

        pr_info("mt9v114: probing sensor.\n");

        info = kzalloc(sizeof(struct mt9v114_info), GFP_KERNEL);
        if (!info) {
                pr_err("mt9v114: Unable to allocate memory!\n");
                return -ENOMEM;
        }

        err = misc_register(&mt9v114_device);
        if (err) {
                pr_err("mt9v114: Unable to register misc device!\n");
                kfree(info);
                return err;
        }

        info->pdata = client->dev.platform_data;
        info->i2c_client = client;

        i2c_set_clientdata(client, info);
        return 0;
}

static int mt9v114_remove(struct i2c_client *client)
{
        struct mt9v114_info *info;
        info = i2c_get_clientdata(client);
        misc_deregister(&mt9v114_device);
        kfree(info);
        return 0;
}

static const struct i2c_device_id mt9v114_id[] = {
        { "mt9v114", 0 },
        { },
};

MODULE_DEVICE_TABLE(i2c, mt9v114_id);

static struct i2c_driver mt9v114_i2c_driver = {
        .driver = {
                .name = "mt9v114",
                .owner = THIS_MODULE,
        },
        .probe = mt9v114_probe,
        .remove = mt9v114_remove,
        .id_table = mt9v114_id,
};

static int __init mt9v114_init(void)
{
        pr_info("mt9v114 sensor driver loading\n");
        return i2c_add_driver(&mt9v114_i2c_driver);
}

static void __exit mt9v114_exit(void)
{
        i2c_del_driver(&mt9v114_i2c_driver);
}

module_init(mt9v114_init);
module_exit(mt9v114_exit);
