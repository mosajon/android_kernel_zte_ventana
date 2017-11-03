/*
 * arch/arm/mach-tegra/board-ventana-sensors.c
 *
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mpu.h>
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <linux/nct1008.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/max17040_battery.h>//ZTE:add by caixiaoguang 20111008


#include <mach/gpio.h>

#include <media/ov5650.h>
#include <media/ov2710.h>
/* ZTE: add by yaoling for new camera 20110809 ++ */
#include <media/ov5640.h>
#include <media/mt9v114.h>
#include <media/adp1655.h>
/* ZTE: add by yaoling for new camera 20110809 -- */
/* ZTE: add by yaoling for 970 new camera */
#include <media/ov7692.h>
#include <media/ssl3250a.h>
#include <generated/mach-types.h>

#include "gpio-names.h"
#include "board.h"
#include "board-ventana.h"
#include "cpu-tegra.h"
#include <media/sh532u.h>

#ifdef CONFIG_SENSORS_TMD2771
#include <linux/i2c/tmd2771x.h>
#endif
#ifdef CONFIG_SENSORS_L3G4200D
#include <linux/i2c/l3g4200d.h>
#endif
#ifdef CONFIG_SENSORS_AK8962
#include <linux/i2c/akm8962.h>
#endif
#ifdef CONFIG_ACCEL_SENSORS
#include <linux/accel.h>
#endif

#define ISL29018_IRQ_GPIO	TEGRA_GPIO_PZ2
#define AKM8975_IRQ_GPIO	TEGRA_GPIO_PN5
#define CAMERA_POWER_GPIO	TEGRA_GPIO_PV4
#define CAMERA_CSI_MUX_SEL_GPIO	TEGRA_GPIO_PBB4
#define CAMERA_FLASH_ACT_GPIO	TEGRA_GPIO_PD2
#define NCT1008_THERM2_GPIO	TEGRA_GPIO_PN6
/* ZTE: add by yaoling for new cam 20110810 ++ */
#define CAMERA1_PWDN_GPIO		 TEGRA_GPIO_PBB1 
#define CAMERA1_RESET_GPIO		TEGRA_GPIO_PBB4 
#define CAMERA2_PWDN_GPIO		TEGRA_GPIO_PBB5
#define CAMERA2_RESET_GPIO		TEGRA_GPIO_PBB5 
#define CAMERA_AF_PD_GPIO		TEGRA_GPIO_PT3
#define CAMERA_FLASH_EN1_GPIO  TEGRA_GPIO_PBB2//TEGRA_GPIO_PD2 
#define CAMERA_FLASH_EN2_GPIO	  TEGRA_GPIO_PBB3 
#define CAMERA_FLASH_TOR_GPIO       TEGRA_GPIO_PT3
/* ZTE: add by yaoling for new cam 20110810 -- */
/* for 970 new front camera start */
#define CAMERA2_OV7692_PWDN_GPIO TEGRA_GPIO_PD2
/* for 970 new front camera start */
/* ZTE: modify  by yaoling for flash 20110711  */
//#define CAMERA_FLASH_STRB_GPIO	TEGRA_GPIO_PA0
#define CAMERA_FLASH_STRB_GPIO      TEGRA_GPIO_PT2
#define CAMERA_FLASH_OP_MODE		1//0 /*0=I2C mode, 1=GPIO mode*/
#define CAMERA_FLASH_MAX_LED_AMP	7
#define CAMERA_FLASH_MAX_TORCH_AMP	11
#define CAMERA_FLASH_MAX_FLASH_AMP	31

/* ZTE: modify  by yaoling for flash 20110711  */
#ifdef CONFIG_MHL_SII8334
#define CI2CA  true  // CI2CA depend on the CI2CA pin's level
#ifdef CI2CA 
#define SII8334_plus 0x02  //Define sii8334's I2c Address of all pages by the status of CI2CA.
#else
#define SII8334_plus 0x00  //Define sii8334's I2c Address of all pages by the status of CI2CA.
#endif

struct MHL_platform_data {
	void (*reset) (void);
};

static void Sii8334_reset(void)
{	
	printk("[MHL] sii8334 reset sequence\n");
	
	tegra_gpio_enable(AP_MHL_RESET);
	
	gpio_request(AP_MHL_RESET, "mhl_reset");
	
	gpio_direction_output(AP_MHL_RESET, 0);
	gpio_export(AP_MHL_RESET, false);
	gpio_set_value(AP_MHL_RESET, 0);
	mdelay(100);
	gpio_set_value(AP_MHL_RESET, 1);
	
	gpio_free(AP_MHL_RESET);
}
static struct MHL_platform_data Sii8334_data = {
	.reset = Sii8334_reset,
};

static int __init ventana_mhl_init(void)
{    
//	int ret;
	gpio_request(TEGRA_GPIO_PG0, "mhl-irq");	  
      gpio_direction_input(TEGRA_GPIO_PG0); 
      tegra_gpio_enable(TEGRA_GPIO_PG0);
#if 0
	  printk("[MHL] init irq & power\n");
      	if (!ventana_LDO8) {
		ventana_LDO8 = regulator_get(NULL, "vdd_ldo8"); /* LD03 */
		if (IS_ERR_OR_NULL(ventana_LDO8)) {
			printk(KERN_ERR "dsi: couldn't get regulator vdd_ldo8\n");
			ventana_LDO8 = NULL;
			return PTR_ERR(ventana_LDO8);
		}
        	/* set HDMI voltage to 3V3*/
       ret = regulator_set_voltage(ventana_LDO8, 3300*1000, 3300*1000);
	if (ret) {
		printk(KERN_ERR "%s: Failed to set vdd_ldo8 to 3.3v\n", __func__);
              regulator_put(ventana_LDO8);
		return PTR_ERR(ventana_LDO8);
	}
            regulator_enable(ventana_LDO8); 

    }
#endif
     return 0;
}
#endif
static int ventana_camera_init(void)
{
      
        tegra_gpio_enable(CAMERA1_PWDN_GPIO);
        gpio_request(CAMERA1_PWDN_GPIO, "camera1_powerdown");
        gpio_direction_output(CAMERA1_PWDN_GPIO, 0);
        gpio_export(CAMERA1_PWDN_GPIO, false);

        tegra_gpio_enable(CAMERA1_RESET_GPIO);
        gpio_request(CAMERA1_RESET_GPIO, "camera1_reset");
        gpio_direction_output(CAMERA1_RESET_GPIO, 0);
        gpio_export(CAMERA1_RESET_GPIO, false);


        tegra_gpio_enable(CAMERA2_RESET_GPIO);
        gpio_request(CAMERA2_RESET_GPIO, "camera2_reset");
        gpio_direction_output(CAMERA2_RESET_GPIO, 0);
        gpio_export(CAMERA2_RESET_GPIO, false);
       
         tegra_gpio_enable(CAMERA2_OV7692_PWDN_GPIO);
        gpio_request(CAMERA2_OV7692_PWDN_GPIO, "camera2_ov7692_powerdown");
        gpio_direction_output(CAMERA2_OV7692_PWDN_GPIO, 0);
        gpio_export(CAMERA2_OV7692_PWDN_GPIO, false);

        mdelay(5);
	return 0;
}

/* left ov5650 is CAM2 which is on csi_a */
static int ventana_left_ov5650_power_on(void)
{
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 0);
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 1);
	gpio_direction_output(CAM2_LDO_SHUTDN_L_GPIO, 1);
	mdelay(5);
	gpio_direction_output(CAM2_PWR_DN_GPIO, 0);
	mdelay(5);
	gpio_direction_output(CAM2_RST_L_GPIO, 0);
	mdelay(1);
	gpio_direction_output(CAM2_RST_L_GPIO, 1);
	mdelay(20);
	return 0;
}

static int ventana_left_ov5650_power_off(void)
{
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 0);
	gpio_direction_output(CAM2_RST_L_GPIO, 0);
	gpio_direction_output(CAM2_PWR_DN_GPIO, 1);
	gpio_direction_output(CAM2_LDO_SHUTDN_L_GPIO, 0);
	return 0;
}

struct ov5650_platform_data ventana_left_ov5650_data = {
	.power_on = ventana_left_ov5650_power_on,
	.power_off = ventana_left_ov5650_power_off,
};

/* right ov5650 is CAM1 which is on csi_b */
static int ventana_right_ov5650_power_on(void)
{
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 1);
	gpio_direction_output(CAM1_LDO_SHUTDN_L_GPIO, 1);
	mdelay(5);
	gpio_direction_output(CAM1_PWR_DN_GPIO, 0);
	mdelay(5);
	gpio_direction_output(CAM1_RST_L_GPIO, 0);
	mdelay(1);
	gpio_direction_output(CAM1_RST_L_GPIO, 1);
	mdelay(20);
	return 0;
}

static int ventana_right_ov5650_power_off(void)
{
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 0);
	gpio_direction_output(CAM1_RST_L_GPIO, 0);
	gpio_direction_output(CAM1_PWR_DN_GPIO, 1);
	gpio_direction_output(CAM1_LDO_SHUTDN_L_GPIO, 0);
	return 0;
}

struct ov5650_platform_data ventana_right_ov5650_data = {
	.power_on = ventana_right_ov5650_power_on,
	.power_off = ventana_right_ov5650_power_off,
};
/* ZTE: add by yaoling camera 9v114 and ov5640 power on /off  20110809 ++ */
static int ventana_ov5640_power_on(void)
{
    //printk("ventana_ov5640_power_on\n");
   // gpio_set_value(CAMERA2_OV7692_PWDN_GPIO, 1);
    gpio_direction_output(CAM_1V8_GPIO_EN, 1);
    gpio_direction_output(CAM_2V8_GPIO_EN, 1);
    gpio_set_value(CAMERA1_PWDN_GPIO, 0);
    mdelay(10);  
    gpio_set_value(CAMERA1_RESET_GPIO, 1);
    mdelay(10);
    gpio_set_value(CAMERA1_RESET_GPIO, 0);
    mdelay(5);
    gpio_set_value(CAMERA1_RESET_GPIO, 1);
    mdelay(20);
 
    return 0;
}

static int ventana_ov5640_power_off(void)
{
       
        gpio_set_value(CAMERA1_PWDN_GPIO, 1);
       // gpio_set_value(CAMERA1_RESET_GPIO, 0);
        gpio_direction_output(CAM_1V8_GPIO_EN, 0);
        gpio_direction_output(CAM_2V8_GPIO_EN, 0);

        return 0;
}

static int ventana_mt9v114_power_on(void)
{
        pr_err("ventana_mt9v114_power_on\n");
        gpio_set_value(CAMERA1_PWDN_GPIO, 1);
        mdelay(10);
        gpio_direction_output(CAM_1V8_GPIO_EN, 1);
        gpio_direction_output(CAM_2V8_GPIO_EN, 1);
        mdelay(5);
        mdelay(10);
        gpio_set_value(CAMERA2_RESET_GPIO, 0);

        mdelay(200);

        return 0;

}

static int ventana_mt9v114_power_off(void)
{
     
        gpio_set_value(CAMERA2_RESET_GPIO, 1);
        gpio_direction_output(CAM_1V8_GPIO_EN, 0);
        gpio_direction_output(CAM_2V8_GPIO_EN, 0);
        return 0;
}
/* ZTE: add by yaoling for 970 new front camera start */
static int ventana_ov7692_power_on(void)
{
        //pr_err("ventana_ov7692_power_on\n");
      //  gpio_set_value(CAMERA1_PWDN_GPIO, 1);
     //   mdelay(10);
        gpio_direction_output(CAM_1V8_GPIO_EN, 1);
        gpio_direction_output(CAM_2V8_GPIO_EN, 1);
        mdelay(10);
        gpio_set_value(CAMERA2_OV7692_PWDN_GPIO, 0);

        mdelay(200);

        return 0;
}

static int ventana_ov7692_power_off(void)
{
     
        gpio_set_value(CAMERA2_OV7692_PWDN_GPIO, 1);
        gpio_direction_output(CAM_1V8_GPIO_EN, 0);
        gpio_direction_output(CAM_2V8_GPIO_EN, 0);
        return 0;
}
/* ZTE: add by yaolingfor 970 new front camera end */
struct ov5640_platform_data ventana_ov5640_data = {
	.power_on = ventana_ov5640_power_on,
	.power_off = ventana_ov5640_power_off,
};
struct mt9v114_platform_data ventana_mt9v114_data = {
	.power_on = ventana_mt9v114_power_on,
	.power_off = ventana_mt9v114_power_off,
};
/* ZTE : add by yaoling for 970 ++ */
struct ov7692_platform_data ventana_ov7692_data = {
	.power_on = ventana_ov7692_power_on,
	.power_off = ventana_ov7692_power_off,
};
/* ZTE : add by yaoling for 970 -- */
static struct i2c_board_info ventana_i2c3_board_info[] = {
    {
        I2C_BOARD_INFO("mt9v114", 0x7A>>1),
        .platform_data = &ventana_mt9v114_data,
    },
    { 
        I2C_BOARD_INFO("ov5640", 0x78>>1), 
        .platform_data = &ventana_ov5640_data,
    },
/* ZTE:add by yaoling for front camera ++ */
    {
        I2C_BOARD_INFO("ov7692", 0x3f),
        .platform_data = &ventana_ov7692_data,
    },
/* ZTE:add by yaoling for front camera -- */
};
/* ZTE: add by yaoling camera 9v114 and ov5640 power on /off20110809 -- */
static int ventana_ov2710_power_on(void)
{
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 1);
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 1);
	gpio_direction_output(CAM3_LDO_SHUTDN_L_GPIO, 1);
	mdelay(5);
	gpio_direction_output(CAM3_PWR_DN_GPIO, 0);
	mdelay(5);
	gpio_direction_output(CAM3_RST_L_GPIO, 0);
	mdelay(1);
	gpio_direction_output(CAM3_RST_L_GPIO, 1);
	mdelay(20);
	return 0;
}

static int ventana_ov2710_power_off(void)
{
	gpio_direction_output(CAM3_RST_L_GPIO, 0);
	gpio_direction_output(CAM3_PWR_DN_GPIO, 1);
	gpio_direction_output(CAM3_LDO_SHUTDN_L_GPIO, 0);
	gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 0);
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 0);
	return 0;
}

struct ov2710_platform_data ventana_ov2710_data = {
	.power_on = ventana_ov2710_power_on,
	.power_off = ventana_ov2710_power_off,
};

#if 0
static struct sh532u_platform_data sh532u_left_pdata = {
	.num		= 1,
	.sync		= 2,
	.dev_name	= "focuser",
	.gpio_reset	= CAM2_RST_L_GPIO,
	.gpio_en	= CAM2_LDO_SHUTDN_L_GPIO,
};

static struct sh532u_platform_data sh532u_right_pdata = {
	.num		= 2,
	.sync		= 1,
	.dev_name	= "focuser",
	.gpio_reset	= CAM1_RST_L_GPIO,
	.gpio_en	= CAM1_LDO_SHUTDN_L_GPIO,
};
#endif

static struct nvc_torch_pin_state ventana_ssl3250a_pinstate = {
	.mask		= 0x0040, /* VGP6 */
	.values		= 0x0040,
};

static struct ssl3250a_platform_data ventana_ssl3250a_pdata = {
	.dev_name	= "torch",
	.pinstate	= &ventana_ssl3250a_pinstate,
	.gpio_act	= CAMERA_FLASH_ACT_GPIO,
};

/* ZTE: add by yaoling for new camera flash 20110816 ++ */
static int ventana_adp1655_init(void)
{
       
        gpio_request(CAMERA_FLASH_STRB_GPIO, "torch_gpio_strb");
        gpio_direction_output(CAMERA_FLASH_STRB_GPIO, 0);
        tegra_gpio_enable(CAMERA_FLASH_STRB_GPIO);
        gpio_export(CAMERA_FLASH_STRB_GPIO, false);
        /* ZTE del by yaoling for 970 20111123 ++ */
        #if 0
        gpio_request(CAMERA_FLASH_TOR_GPIO, "torch_gpio_tor");
        gpio_direction_output(CAMERA_FLASH_TOR_GPIO, 0);
        tegra_gpio_enable(CAMERA_FLASH_TOR_GPIO);
        gpio_export(CAMERA_FLASH_TOR_GPIO, false); 
        gpio_set_value(CAMERA_FLASH_STRB_GPIO, 1);
        #endif 
        /* ZTE del by yaoling for 970 20111123 -- */

	return 0;
}

static void ventana_adp1655_exit(void)
{
	gpio_set_value(CAMERA_FLASH_STRB_GPIO, 0);
	gpio_free(CAMERA_FLASH_STRB_GPIO);
	tegra_gpio_disable(CAMERA_FLASH_STRB_GPIO);
         /* ZTE del by yaoling for 970 20111123 ++ */
        #if 0
        gpio_set_value(CAMERA_FLASH_TOR_GPIO, 0);
        gpio_free(CAMERA_FLASH_TOR_GPIO);
         tegra_gpio_disable(CAMERA_FLASH_TOR_GPIO);
         #endif 
        /* ZTE del by yaoling for 970 20111123 -- */
     
}

static int ventana_adp1655_gpio_strb(int val)
{
	int prev_val;
	prev_val = gpio_get_value(CAMERA_FLASH_STRB_GPIO);
	gpio_set_value(CAMERA_FLASH_STRB_GPIO, val);
	return prev_val;
};
#if 0
static int ventana_adp1655_gpio_act(int val)
{
	int prev_val;
	//prev_val = gpio_get_value(CAMERA_FLASH_ACT_GPIO);
	//gpio_set_value(CAMERA_FLASH_ACT_GPIO, val);
	return prev_val;
};

static int ventana_adp1655_gpio_tor(int val)
{
	int prev_val;
         /* ZTE del by yaoling for 970 20111123 ++ */
        #if 0
	prev_val = gpio_get_value(CAMERA_FLASH_TOR_GPIO);
	gpio_set_value(CAMERA_FLASH_TOR_GPIO, val);
        #endif
        /* ZTE del by yaoling for 970 20111123 -- */
	return prev_val;
};
#endif
static struct adp1655_platform_data ventana_adp1655_data = {
	.config		= CAMERA_FLASH_OP_MODE,
	.max_amp_indic	= CAMERA_FLASH_MAX_LED_AMP,
	.max_amp_torch	= CAMERA_FLASH_MAX_TORCH_AMP,
	.max_amp_flash	= CAMERA_FLASH_MAX_FLASH_AMP,
        .init = ventana_adp1655_init,
        .exit = ventana_adp1655_exit,
        .gpio_act	= NULL,
	.gpio_en1	= NULL,
	.gpio_en2	= NULL,
        .gpio_strb	= ventana_adp1655_gpio_strb,
        .gpio_tor  =NULL,// ventana_adp1655_gpio_tor,

};
static const struct i2c_board_info ventana_i2c3_board_info_adp1655[] = {
    {
        I2C_BOARD_INFO("adp1655", 0x30),
        .platform_data = &ventana_adp1655_data,
    },
};
/* ZTE: add by yaoling for new camera flash 20110816 -- */
/*
static void ventana_isl29018_init(void)
{
	tegra_gpio_enable(ISL29018_IRQ_GPIO);
	gpio_request(ISL29018_IRQ_GPIO, "isl29018");
	gpio_direction_input(ISL29018_IRQ_GPIO);
}*/

#ifdef CONFIG_SENSORS_AK8975
static void ventana_akm8975_init(void)
{
	tegra_gpio_enable(AKM8975_IRQ_GPIO);
	gpio_request(AKM8975_IRQ_GPIO, "akm8975");
	gpio_direction_input(AKM8975_IRQ_GPIO);
}
#endif

static void ventana_nct1008_init(void)
{
	tegra_gpio_enable(NCT1008_THERM2_GPIO);
	gpio_request(NCT1008_THERM2_GPIO, "temp_alert");
	gpio_direction_input(NCT1008_THERM2_GPIO);
}

static struct nct1008_platform_data ventana_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.conv_rate = 0x08,
	.offset = 0,
	.hysteresis = 0,
	.shutdown_ext_limit = 115,
	.shutdown_local_limit = 120,
	.throttling_ext_limit = 90,
	.alarm_fn = tegra_throttling_enable,
};

static const struct i2c_board_info ventana_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("bq20z75", 0x0B),
	},
};

static struct pca953x_platform_data ventana_tca6416_data = {
	.gpio_base      = TEGRA_NR_GPIOS + 4, /* 4 gpios are already requested by tps6586x */
};

static struct pca954x_platform_mode ventana_pca9546_modes[] = {
	{ .adap_id = 6, .deselect_on_exit = 1 }, /* REAR CAM1 */
	{ .adap_id = 7, .deselect_on_exit = 1 }, /* REAR CAM2 */
	{ .adap_id = 8, .deselect_on_exit = 1 }, /* FRONT CAM3 */
};

static struct pca954x_platform_data ventana_pca9546_data = {
	.modes	  = ventana_pca9546_modes,
	.num_modes      = ARRAY_SIZE(ventana_pca9546_modes),
};

static const struct i2c_board_info ventana_i2c3_board_info_tca6416[] = {
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &ventana_tca6416_data,
	},
};

static const struct i2c_board_info ventana_i2c3_board_info_pca9546[] = {
	{
		I2C_BOARD_INFO("pca9546", 0x70),
		.platform_data = &ventana_pca9546_data,
	},
};

static const struct i2c_board_info ventana_i2c3_board_info_ssl3250a[] = {
	{
		I2C_BOARD_INFO("ssl3250a", 0x30),
		.platform_data = &ventana_ssl3250a_pdata,
	},
};


/*ZTE:add by caixiaoguang for max17040++*/
#ifdef CONFIG_BATTERY_MAX17040
static int  battery_is_online(void)
{
     return 1;  //currently we have no way to get this infomation
}
extern int fsl_charger_detect(void);
extern int tps6586x_get_charge_status(void);
extern int tps6586x_get_charger_present_status(void);

static int  charger_is_online(void)
{      
    int   ret =0;
    ret =tps6586x_get_charger_present_status();
    //printk("[%s]:CXG get chager online flag ret=%d\n",__FUNCTION__,ret);
    return ret ;
}
static int  charger_is_enabled(void)
{
    int   ret =0;
    ret =tps6586x_get_charge_status();
    //printk("[%s]:CXG get chager enable  flag ret=%d\n",__FUNCTION__,ret);
    return ret;
}

static struct max17040_platform_data vetana_max17040_pdata = {
	.battery_online = battery_is_online,   
	.charger_online = charger_is_online,  
	.charger_enable = charger_is_enabled,	
	.ini_data={
                        .title = "839_1_032612",
                        .emptyadjustment = 0,
                        .fulladjustment = 100,
                        .rcomp0 = 70,				/* Starting RCOMP value */
                        .tempcoup = -0.375,			/* Temperature (hot) coeffiecient for RCOMP, should div 10 */
                        .tempcodown = -3.225,			/* Temperature (cold) coeffiecient for RCOMP, should div 10 */
                        .ocvtest = 55888,			/* OCVTest vaule */
                        .socchecka = 233,			/* SOCCheck low value */
                        .soccheckb = 235,			/* SOCCheck high value */
                        .bits = 19,				/* 18 or 19 bit model */
                        .data = {   0x4F,0x5F,0x92,0x9D,0xB7,0xC1,0xC8,0xD4,
                                        0xE2,0x0B,0x31,0x57,0x80,0xA9,0xD9,0x05,
                                        0xE1,0x05,0x01,0x00,0x31,0x00,0x51,0x00,
                                        0x81,0x00,0xA1,0x01,0xD0,0x00,0x00,0x00,
                                        0xA4,0xF0,0xB5,0xF0,0xB9,0x20,0xB9,0xD0,
                                        0xBB,0x70,0xBC,0x10,0xBC,0x80,0xBD,0x40,
                                        0xBE,0x20,0xC0,0xB0,0xC3,0x10,0xC5,0x70,
                                        0xC8,0x00,0xCA,0x90,0xCD,0x90,0xD0,0x50,
                                        0x01,0x90,0x1D,0xD0,0x0F,0xF0,0x14,0xD0,
                                        0x4E,0x40,0x6E,0x30,0x5F,0x00,0x4B,0x60,
                                        0x1C,0x50,0x1B,0x00,0x1B,0x00,0x14,0x00,
                                        0x14,0x00,0x13,0x10,0x0E,0x00,0x0E,0x00,
                                        0x40,0x09,0x51,0x0D,0x90,0x0F,0x91,0x0D,
                                        0xB4,0x04,0xC6,0x03,0xC5,0x00,0xD4,0x06,
                                        0x19,0xDD,0xFF,0x4D,0xE4,0xE3,0xF0,0xB6,
                                        0xC5,0xB0,0xB0,0x40,0x40,0x31,0xE0,0xE0,
                                        } ,	
	                #if 0
                    //1650mAh
                        .title = "718_1_102511",
                        .emptyadjustment = 0,
                        .fulladjustment = 100,
                        .rcomp0 = 81,				/* Starting RCOMP value */
                        .tempcoup = -0.825,			/* Temperature (hot) coeffiecient for RCOMP, should div 10 */
                        .tempcodown = -4.05,			/* Temperature (cold) coeffiecient for RCOMP, should div 10 */
                        .ocvtest = 55904,			/* OCVTest vaule */
                        .socchecka = 241,			/* SOCCheck low value */
                        .soccheckb = 243,			/* SOCCheck high value */
                        .bits = 19,				/* 18 or 19 bit model */
                        .data = {   0x55,0x74,0x8A,0xA9,0xBB,0xC3,0xC7,0xD1,
                                        0xDF,0xED,0x14,0x38,0x5F,0x86,0xC6,0x06,
                                        0xD4,0x0F,0xE1,0x0F,0x11,0x0F,0x31,0x01,
                                        0x51,0x05,0x81,0x00,0xC1,0x00,0x01,0x00,
                                        0xA5,0x50,0xB7,0x40,0xB8,0xA0,0xBA,0x90,
                                        0xBB,0xB0,0xBC,0x30,0xBC,0x70,0xBD,0x10,
                                        0xBD,0xF0,0xBE,0xD0,0xC1,0x40,0xC3,0x80,
                                        0xC5,0xF0,0xC8,0x60,0xCC,0x60,0xD0,0x60,
                                        0x01,0xF0,0x33,0x50,0x0F,0xF0,0x11,0xF0,
                                        0x54,0xF0,0x75,0xF0,0x68,0x40,0x48,0x90,
                                        0x48,0xF0,0x19,0xF0,0x1B,0xF0,0x18,0x10,
                                        0x18,0x50,0x11,0x00,0x11,0x00,0x11,0x00,
                                        0x50,0x0F,0x73,0x05,0x80,0x0F,0xA1,0x0F,
                                        0xB5,0x0F,0xC7,0x0F,0xC6,0x04,0xD4,0x09,
                                        0x1F,0x35,0xFF,0x1F,0x4F,0x5F,0x84,0x89,
                                        0x8F,0x9F,0xBF,0x81,0x85,0x10,0x10,0x10,
                                        } ,	
                    #endif
                    
	},
};
#endif
/*ZTE:add by caixiaoguang for max17040--*/

static struct i2c_board_info ventana_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(NCT1008_THERM2_GPIO),
		.platform_data = &ventana_nct1008_pdata,
	},

#ifdef CONFIG_SENSORS_AK8975
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.irq = TEGRA_GPIO_TO_IRQ(AKM8975_IRQ_GPIO),
	},
#endif

/*ZTE:add by caixiaoguang for max17040++*/
#ifdef CONFIG_BATTERY_MAX17040
       {
		I2C_BOARD_INFO("max17040", 0x36),  
               .platform_data = &vetana_max17040_pdata,
        
	},
#endif
/*ZTE:add by caixiaoguang for max17040--*/
};

#ifdef CONFIG_SENSORS_TMD2771
static struct prox_config prox_config_pdata = {
	.pulse_count = 4,
	.led_drive = PROX_LED_DRIVE_25MA,
	.threshold_lo = 500,
	.threshold_hi = 560,
	.thresh_offset = 50,
	.thresh_gain_hi = 200,
	.thresh_gain_lo = 170,
};

static int tmd2771_irq_setup(void){
       /*configure the interrupt pin*/
	gpio_request(TEGRA_GPIO_PK2, "taos_irq");
	tegra_gpio_enable(TEGRA_GPIO_PK2);
	gpio_direction_input(TEGRA_GPIO_PK2);
	return 0;
}

static int tmd2771_power_control(int en){
	static struct regulator *pmu_ldo3_reg = NULL;

	if(!pmu_ldo3_reg)
		pmu_ldo3_reg = regulator_get(NULL, "vdd_ldo3");

	if(en){
		regulator_enable(pmu_ldo3_reg);
		gpio_set_value(TEGRA_GPIO_PR2, 1);
	}else{
		gpio_set_value(TEGRA_GPIO_PR2, 0);
		regulator_disable(pmu_ldo3_reg);
	}

	return 0;
}

static struct tmd2771x_platform_data tmd2771_pdata = {
	.prox_config = &prox_config_pdata,
	.irq_gpio_setup = tmd2771_irq_setup,
	.power_control = tmd2771_power_control,
};
#endif

#ifdef CONFIG_ACCEL_SENSORS
static unsigned short accel_addr_list[] = {
    0x0F, // kionix
    0x19, // st
    0x1D, // freescale
    I2C_CLIENT_END
};

static struct accel_platform_data accel_pdata ={
	.adapt_nr = 0,
	.accel_list = accel_addr_list,
	.poll_inerval = 10,
	.g_range = ACCEL_FS_2G,
};

static struct platform_device accel_platform_device = {
    .name		= "accel_platform",
    .id = -1,
    .dev = {
        .platform_data = &accel_pdata,
    },
};
#endif

#ifdef CONFIG_SENSORS_L3G4200D
static struct l3g4200d_gyr_platform_data l3g4200d_pdata = {
	.poll_interval = 10,
	.min_interval = 2,
	.fs_range      = L3G4200D_FS_2000DPS,

	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,

	.negate_x = 1,
	.negate_y = 1,
	.negate_z = 0,

	.gpio_DRDY = TEGRA_GPIO_PY1,
};
#endif

#ifdef CONFIG_SENSORS_AK8962
static struct akm8962_platform_data akm8962_pdata = {
	.gpio_DRDY = TEGRA_GPIO_PS2,
};
#endif

static const struct i2c_board_info ventana_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO("isl29018", 0x44),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ2),
	},
#ifdef CONFIG_MHL_SII8334
	{
	  I2C_BOARD_INFO("sii8334_PAGE_TPI", 0x39 + SII8334_plus),
	 .irq = TEGRA_GPIO_TO_IRQ(AP_MHL_INT),  //define the interrupt signal input pin
	 .platform_data = &Sii8334_data,
	},
	{
	  I2C_BOARD_INFO("sii8334_PAGE_TX_L0", 0x39 + SII8334_plus),
	},
	{
	  I2C_BOARD_INFO("sii8334_PAGE_TX_L1", 0x3d + SII8334_plus),
	},
	{
	  I2C_BOARD_INFO("sii8334_PAGE_TX_2", 0x49 + SII8334_plus),
	},
	{
	  I2C_BOARD_INFO("sii8334_PAGE_TX_3", 0x4d + SII8334_plus),
	},
	{
	  I2C_BOARD_INFO("sii8334_PAGE_CBUS", 0x64 + SII8334_plus),
	},
#endif
#ifdef CONFIG_SENSORS_TMD2771
	{
		I2C_BOARD_INFO("tmd2771x", 0x39),
		.platform_data = &tmd2771_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2),
	},
#endif
#ifdef CONFIG_SENSORS_L3G4200D
	{
		I2C_BOARD_INFO(L3G4200D_I2C_NAME, 0x69),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV1),
		.platform_data = &l3g4200d_pdata,
	},
#endif
#ifdef CONFIG_SENSORS_AK8962
	{
		I2C_BOARD_INFO(AKM8962_I2C_NAME, 0x0C),
		.platform_data = &akm8962_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS2),
	},
#endif
};
#if 0
static struct i2c_board_info ventana_i2c6_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650R", 0x36),
		.platform_data = &ventana_right_ov5650_data,
	},
	{
		I2C_BOARD_INFO("sh532u", 0x72),
		.platform_data = &sh532u_right_pdata,
	},
};

static struct i2c_board_info ventana_i2c7_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650L", 0x36),
		.platform_data = &ventana_left_ov5650_data,
	},
	{
		I2C_BOARD_INFO("sh532u", 0x72),
		.platform_data = &sh532u_left_pdata,
	},
};

static struct i2c_board_info ventana_i2c8_board_info[] = {
	{
		I2C_BOARD_INFO("ov2710", 0x36),
		.platform_data = &ventana_ov2710_data,
	},
};
#endif
#ifdef CONFIG_MPU_SENSORS_MPU3050
static struct mpu_platform_data mpu3050_data = {
	.int_config  = 0x10,
	.level_shifter = 0,
	.orientation	= MPU_GYRO_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct ext_slave_platform_data mpu3050_accel_data = {
	.address	= MPU_ACCEL_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_ACCEL_BUS_NUM,
	.bus         = EXT_SLAVE_BUS_SECONDARY,
	.orientation	= MPU_ACCEL_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct ext_slave_platform_data mpu_compass_data = {
	.address	= MPU_COMPASS_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_COMPASS_BUS_NUM,
	.bus         = EXT_SLAVE_BUS_PRIMARY,
	.orientation	= MPU_COMPASS_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct i2c_board_info __initdata inv_mpu_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.irq = TEGRA_GPIO_TO_IRQ(MPU_GYRO_IRQ_GPIO),
		.platform_data = &mpu3050_data,
	},
	{
		I2C_BOARD_INFO(MPU_ACCEL_NAME, MPU_ACCEL_ADDR),
#if	MPU_ACCEL_IRQ_GPIO
		.irq = TEGRA_GPIO_TO_IRQ(MPU_ACCEL_IRQ_GPIO),
#endif
		.platform_data = &mpu3050_accel_data,
	},
};

static struct i2c_board_info __initdata inv_mpu_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
#if	MPU_COMPASS_IRQ_GPIO
		.irq = TEGRA_GPIO_TO_IRQ(MPU_COMPASS_IRQ_GPIO),
#endif
		.platform_data = &mpu_compass_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;

	pr_info("*** MPU START *** mpuirq_init...\n");

#if	MPU_ACCEL_IRQ_GPIO
	/* ACCEL-IRQ assignment */
	tegra_gpio_enable(MPU_ACCEL_IRQ_GPIO);
	ret = gpio_request(MPU_ACCEL_IRQ_GPIO, MPU_ACCEL_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_ACCEL_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_ACCEL_IRQ_GPIO);
		return;
	}
#endif

	/* MPU-IRQ assignment */
	tegra_gpio_enable(MPU_GYRO_IRQ_GPIO);
	ret = gpio_request(MPU_GYRO_IRQ_GPIO, MPU_GYRO_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_GYRO_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_GYRO_IRQ_GPIO);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	i2c_register_board_info(MPU_GYRO_BUS_NUM, inv_mpu_i2c2_board_info,
		ARRAY_SIZE(inv_mpu_i2c2_board_info));
	i2c_register_board_info(MPU_COMPASS_BUS_NUM, inv_mpu_i2c4_board_info,
		ARRAY_SIZE(inv_mpu_i2c4_board_info));
}
#endif

int __init ventana_sensors_init(void)
{
   /* ZTE: del by yaoling for no use 20110810 ++ */
   /*
	struct board_info BoardInfo;

	ventana_isl29018_init();
#ifdef CONFIG_SENSORS_AK8975
	ventana_akm8975_init();
#endif
#ifdef CONFIG_MPU_SENSORS_MPU3050
	mpuirq_init();
#endif
	ventana_camera_init();
	ventana_nct1008_init();  */
	

 /*	i2c_register_board_info(0, ventana_i2c0_board_info,
		ARRAY_SIZE(ventana_i2c0_board_info));

	tegra_get_board_info(&BoardInfo); */

	/*
	 * battery driver is supported on FAB.D boards and above only,
	 * since they have the necessary hardware rework
	 */
	
	/*if (BoardInfo.sku > 0) {
		i2c_register_board_info(2, ventana_i2c2_board_info,
			ARRAY_SIZE(ventana_i2c2_board_info));
	}

	i2c_register_board_info(3, ventana_i2c3_board_info_ssl3250a,
		ARRAY_SIZE(ventana_i2c3_board_info_ssl3250a));

	i2c_register_board_info(4, ventana_i2c4_board_info,
		ARRAY_SIZE(ventana_i2c4_board_info));

	i2c_register_board_info(6, ventana_i2c6_board_info,
		ARRAY_SIZE(ventana_i2c6_board_info));

	i2c_register_board_info(7, ventana_i2c7_board_info,
		ARRAY_SIZE(ventana_i2c7_board_info));

	i2c_register_board_info(8, ventana_i2c8_board_info,
		ARRAY_SIZE(ventana_i2c8_board_info));

  */
    /* ZTE: del by yaoling for no use 20110810 -- */
	/*sensor power enable*/
	tegra_gpio_enable(TEGRA_GPIO_PQ0);
	gpio_request(TEGRA_GPIO_PQ0, "SENSOR_ON");
	gpio_direction_output(TEGRA_GPIO_PQ0, 1);

	gpio_request(TEGRA_GPIO_PR2, "tmd2771x_power");
	tegra_gpio_enable(TEGRA_GPIO_PR2);
	gpio_direction_output(TEGRA_GPIO_PR2, 1);

#ifdef CONFIG_ACCEL_SENSORS
	gpio_request(TEGRA_GPIO_PY2, "accel_int1");
	tegra_gpio_enable(TEGRA_GPIO_PY2);
	gpio_direction_input(TEGRA_GPIO_PY2);

	gpio_request(TEGRA_GPIO_PY3, "accel_int2");
	tegra_gpio_enable(TEGRA_GPIO_PY3);
	gpio_direction_input(TEGRA_GPIO_PY3);
#endif	

#ifdef CONFIG_SENSORS_L3G4200D
	gpio_request(TEGRA_GPIO_PY1, "l3g4200dl_ready");
	tegra_gpio_enable(TEGRA_GPIO_PY1);
	gpio_direction_input(TEGRA_GPIO_PY1);

	gpio_request(TEGRA_GPIO_PV1, "l3g4200dl_ready");
	tegra_gpio_enable(TEGRA_GPIO_PV1);
	gpio_direction_input(TEGRA_GPIO_PV1);
#endif

#ifdef CONFIG_SENSORS_AK8962
    /*akm int pin config*/
    tegra_gpio_enable(TEGRA_GPIO_PS2);
    gpio_request(TEGRA_GPIO_PS2, "akm_int");
    gpio_direction_input(TEGRA_GPIO_PS2);
#endif

 #ifdef CONFIG_MHL_SII8334
	ventana_mhl_init();
        #endif

    i2c_register_board_info(0, ventana_i2c0_board_info,
        ARRAY_SIZE(ventana_i2c0_board_info));

#ifdef CONFIG_ACCEL_SENSORS
    platform_device_register(&accel_platform_device);
#endif
    /*cpu tempreture sensor*/
    ventana_nct1008_init();
    i2c_register_board_info(4, ventana_i2c4_board_info,
        ARRAY_SIZE(ventana_i2c4_board_info));

        /* ZTE: added by yaoling for new camera 20110810 ++ */
    ventana_camera_init();
    i2c_register_board_info(3, ventana_i2c3_board_info,
        ARRAY_SIZE(ventana_i2c3_board_info));
    i2c_register_board_info(3, ventana_i2c3_board_info_adp1655,
        ARRAY_SIZE(ventana_i2c3_board_info_adp1655));
    /* ZTE: added by yaoling for new camera 20110810 -- */
	return 0;
}

#ifdef CONFIG_TEGRA_CAMERA

struct tegra_camera_gpios {
	const char *name;
	int gpio;
	int enabled;
};

#define TEGRA_CAMERA_GPIO(_name, _gpio, _enabled)		\
	{						\
		.name = _name,				\
		.gpio = _gpio,				\
		.enabled = _enabled,			\
	}

static struct tegra_camera_gpios ventana_camera_gpio_keys[] = {
	//[0] = TEGRA_CAMERA_GPIO("en_avdd_csi", AVDD_DSI_CSI_ENB_GPIO, 0),
	[0] = TEGRA_CAMERA_GPIO("camera_1v8_enble", CAM_1V8_GPIO_EN, 0),
	[1] = TEGRA_CAMERA_GPIO("camera_2v8_enble", CAM_2V8_GPIO_EN, 0),

	/*[3] = TEGRA_CAMERA_GPIO("cam2_af_pwdn_lo", CAM2_AF_PWR_DN_L_GPIO, 0),
	[4] = TEGRA_CAMERA_GPIO("cam2_pwdn", CAM2_PWR_DN_GPIO, 0),
	[5] = TEGRA_CAMERA_GPIO("cam2_rst_lo", CAM2_RST_L_GPIO, 1),

	[6] = TEGRA_CAMERA_GPIO("cam3_ldo_shdn_lo", CAM3_LDO_SHUTDN_L_GPIO, 0),
	[7] = TEGRA_CAMERA_GPIO("cam3_af_pwdn_lo", CAM3_AF_PWR_DN_L_GPIO, 0),
	[8] = TEGRA_CAMERA_GPIO("cam3_pwdn", CAM3_PWR_DN_GPIO, 0),
	[9] = TEGRA_CAMERA_GPIO("cam3_rst_lo", CAM3_RST_L_GPIO, 1),

	[10] = TEGRA_CAMERA_GPIO("cam1_ldo_shdn_lo", CAM1_LDO_SHUTDN_L_GPIO, 0),
	[11] = TEGRA_CAMERA_GPIO("cam1_af_pwdn_lo", CAM1_AF_PWR_DN_L_GPIO, 0),
	[12] = TEGRA_CAMERA_GPIO("cam1_pwdn", CAM1_PWR_DN_GPIO, 0),
	[13] = TEGRA_CAMERA_GPIO("cam1_rst_lo", CAM1_RST_L_GPIO, 1), */
};

int __init ventana_camera_late_init(void)
{
        int ret;
        int i;
        //struct regulator *cam_ldo6 = NULL;

        if (!machine_is_ventana())
        return 0;
       ;
    #if 0
   
	cam_ldo6 = regulator_get(NULL, "vdd_ldo6");
	if (IS_ERR_OR_NULL(cam_ldo6)) {
		pr_err("%s: Couldn't get regulator ldo6\n", __func__);
		return PTR_ERR(cam_ldo6);
	}

	ret = regulator_enable(cam_ldo6);
	if (ret){
		pr_err("%s: Failed to enable ldo6\n", __func__);
		goto fail_put_regulator;
	}
  
 
	i2c_new_device(i2c_get_adapter(3), ventana_i2c3_board_info_tca6416);

	for (i = 0; i < ARRAY_SIZE(ventana_camera_gpio_keys); i++) {
		ret = gpio_request(ventana_camera_gpio_keys[i].gpio,
			ventana_camera_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail_free_gpio;
		}
		gpio_direction_output(ventana_camera_gpio_keys[i].gpio,
			ventana_camera_gpio_keys[i].enabled);
		gpio_export(ventana_camera_gpio_keys[i].gpio, false);
	}

	i2c_new_device(i2c_get_adapter(3), ventana_i2c3_board_info_pca9546);

	ventana_ov2710_power_off();
	ventana_left_ov5650_power_off();
	ventana_right_ov5650_power_off();

	ret = regulator_disable(cam_ldo6);
	if (ret){
		pr_err("%s: Failed to disable ldo6\n", __func__);
		goto fail_free_gpio;
	}

	regulator_put(cam_ldo6);
    #endif
    #if 1
        printk("yaoling ventana_camera_late_init\n");
        for (i = 0; i < ARRAY_SIZE(ventana_camera_gpio_keys); i++) {
            ret = gpio_request(ventana_camera_gpio_keys[i].gpio,
            ventana_camera_gpio_keys[i].name);
            if (ret < 0) {
            pr_err("%s: gpio_request failed for gpio #%d\n",
             __func__, i);
                goto fail_free_gpio;
        }
        gpio_direction_output(ventana_camera_gpio_keys[i].gpio,
        ventana_camera_gpio_keys[i].enabled);
        gpio_export(ventana_camera_gpio_keys[i].gpio, false);
        }
        ventana_ov7692_power_off();
        ventana_ov5640_power_off();
    #endif
     return 0;
fail_free_gpio:
	while (i--)
		gpio_free(ventana_camera_gpio_keys[i].gpio);
	return ret;
}

late_initcall(ventana_camera_late_init);

#endif /* CONFIG_TEGRA_CAMERA */
