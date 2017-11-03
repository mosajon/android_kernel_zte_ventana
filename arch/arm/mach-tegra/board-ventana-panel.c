/*
 * arch/arm/mach-tegra/board-ventana-panel.c
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/pwm_backlight.h>
#include <linux/nvhost.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "devices.h"
#include "gpio-names.h"
#include "board.h"

/*ZTE: added by tong.weili for print debug 20120202 ++*/
static int debug = 0;
module_param(debug, int, 0600);
/*ZTE: added by tong.weili for print debug 20120202 ++*/

//#define ventana_pnl_pwr_enb	TEGRA_GPIO_PC6  /*ZTE: deleted by tong.weili for  backlight power  unused 20111117*/
//#define ventana_bl_enb		TEGRA_GPIO_PD4  /*ZTE: deleted by tong.weili for  backlight enable unused 20111117*/
//#define ventana_lvds_shutdown	TEGRA_GPIO_PB2  /*ZTE: deleted by tong.weili for  LVDS  unused 20111117*/

#ifdef CONFIG_BACKLIGHT_1_WIRE_MODE
#define TPS61165_CTRL_PIN		TEGRA_GPIO_PB2  /*ZTE:added by tong.weili for TPS61165 CTRL pin 20111117*/
#else
#define ventana_bl_pwm			TEGRA_GPIO_PU5 /*ZTE:added by tong.weili for PWM2 pin 20120309*/
#endif

#define ventana_hdmi_hpd	TEGRA_GPIO_PN7
#define ventana_hdmi_enb	TEGRA_GPIO_PV5
//zte lipeng10094834 add at 2011.07.04
#define ventana_lcd_1v8_enable	    TEGRA_GPIO_PE4
#define ventana_lcd_id	    TEGRA_GPIO_PF2
#define ventana_lcd_te	    TEGRA_GPIO_PJ1
#define ventana_lcd_reset	TEGRA_GPIO_PV7
static struct regulator *ventana_LDO9 = NULL; 
static struct regulator *ventana_LDO0 = NULL; 
#define dsi_panel_reset TEGRA_GPIO_PV7
#define dsi_panel_id	TEGRA_GPIO_PB2
#define dsi_panel_te	TEGRA_GPIO_PN6
////static struct regulator *iovcc_lcd_regulator = NULL;
////static struct regulator *vcc_lcd_regulator = NULL;
extern int tps6586x_write_data_sm3(int reg, unsigned int val);
extern int tps6586x_enable_sm3(void);
static  int g_lcdid = 0;
//zte lipeng10094834 add end
/*panel power on sequence timing*/
#define ventana_pnl_to_lvds_ms	0
#define ventana_lvds_to_bl_ms	200
int s_SuspendFlag = 0;
#ifdef CONFIG_TEGRA_DC
static struct regulator *ventana_hdmi_reg = NULL;
static struct regulator *ventana_hdmi_pll = NULL;
#endif

#ifdef CONFIG_MHL_SII8334
static struct regulator *ventana_LDO8 = NULL;
#endif

#define DEFAULT_BRIGHTNESS (132)      /*ZTE: added by tong.weili 初始默认亮度20111130*/
#ifdef CONFIG_BACKLIGHT_1_WIRE_MODE
/*ZTE: added by tong.weili for 1-wire backlight 20111122 ++*/
static spinlock_t s_tps61165_lock;
static struct mutex s_tps61165_mutex;  /*ZTE: added by tong.weili 增加背光设置稳定性 20111128*/
static bool s_tps61165_is_inited = true;  /*ZTE: modified  by tong.weili for 已在bootloader中初始化20111130*/
static bool s_bNeedSetBacklight = false; /*ZTE: added by tong.weili 解决开机刚进入kernel 设置默认背光会闪一下20111130*/
static int tps61165_init(void);
static int tps61165_write_bit(u8 b);
static int tps61165_write_byte(u8 bytedata);
static int tps61165_config_ES_timing(void);
int tps61165_set_backlight(int brightness);
static int tps61165_shutdown(void);
/*ZTE: added by tong.weili for 1-wire backlight 20111122 --*/
#endif

static int ventana_backlight_init(struct device *dev) {
	int ret = 0;
/*nvidia danny delete at 2011.07.29*/
/*
	ret = gpio_request(ventana_bl_enb, "backlight_enb");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(ventana_bl_enb, 1);
	if (ret < 0)
		gpio_free(ventana_bl_enb);
	else
		tegra_gpio_enable(ventana_bl_enb);
*/
/*nvidia danny delete end at 2011.07.29*/


#if CONFIG_LCD_480_800  
#else
       printk("[%s] 970\n", __FUNCTION__);
#ifdef CONFIG_BACKLIGHT_1_WIRE_MODE
      /*ZTE: added by tong.weili for 1-wire backlight 20111122 ++*/
      ret = gpio_request(TPS61165_CTRL_PIN, "tps61165 ctrl pin");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(TPS61165_CTRL_PIN, 1);
	if (ret < 0)
      {
          gpio_free(TPS61165_CTRL_PIN);
          return ret;
      }   		
	else
      {
          tegra_gpio_enable(TPS61165_CTRL_PIN);
      }   
   
      s_tps61165_is_inited = false;
      spin_lock_init(&s_tps61165_lock); /*ZTE: added by tong.weili 解决背光设置稳定性问题 20111123*/
      mutex_init(&s_tps61165_mutex); /*ZTE: added by tong.weili 增加背光设置稳定性 20111128*/

      /*ZTE: added by tong.weili for 1-wire backlight 20111122 --*/
#else
       tegra_gpio_disable(ventana_bl_pwm);/*ZTE: added by tong.weili for backlight mode: PWM*/	
#endif
#endif
	return ret;
};

static void ventana_backlight_exit(struct device *dev) {
/*nvidia danny delete at 2011.07.29*/
/*	gpio_set_value(ventana_bl_enb, 0);
	gpio_free(ventana_bl_enb);
	tegra_gpio_disable(ventana_bl_enb);
*/
/*nvidia danny delete end at 2011.07.29*/

      printk("[%s] \n", __FUNCTION__);
#if CONFIG_LCD_480_800  
#else
#ifdef CONFIG_BACKLIGHT_1_WIRE_MODE
      /*ZTE: added by tong.weili for 1-wire backlight 20111122 ++*/
      gpio_set_value(TPS61165_CTRL_PIN, 0);
      gpio_free(TPS61165_CTRL_PIN);
      tegra_gpio_disable(TPS61165_CTRL_PIN);
      s_tps61165_is_inited = false;
      /*ZTE: added by tong.weili for 1-wire backlight 20111122 --*/
#endif
#endif
      return;   
}

static int ventana_backlight_notify(struct device *unused, int brightness)
{
/*nvidia danny delete at 2011.07.29*/
//	gpio_set_value(ventana_bl_enb, !!brightness);
/*nvidia danny delete end at 2011.07.29*/
       static int old_brightness = 0;
/*ZTE: deleted by tong.weili for 1-wire backlight 20111122 ++*/

/*ZTE: deleted by tong.weili for 1-wire backlight 20111122 --*/
// printk("[%s] brightness%d s_SuspendFlag %d\n", __FUNCTION__, brightness, s_SuspendFlag);
        if(s_SuspendFlag == 1)
            brightness = 0;
#if CONFIG_LCD_480_800  
//zte lipeng10094834 add at 2011.07.04

{
    uint8_t val0 = 0;
     
       uint8_t val1 = 0;

       val0 = brightness << 3;
       if(brightness != 0)
           val1 = 0x08 | ((brightness >> 5) & 0x07);
       else
           val1 = 0;
       tps6586x_write_data_sm3(0x57, val0);
       tps6586x_write_data_sm3(0x58, val1);   
       printk("[%s] 880F val0 %x, val1 %x\n", __FUNCTION__, val0, val1);
}
//zte lipeng10094834 add end at 2011.07.04
#else
       if(old_brightness==0 && brightness>0 && g_lcdid==1)
        {
            msleep(50);
        }
        old_brightness = brightness;
        if(s_SuspendFlag == 1)
            brightness = 0;
#endif
	return brightness;
}

static int ventana_disp1_check_fb(struct device *dev, struct fb_info *info);

static struct platform_pwm_backlight_data ventana_backlight_data = {
	.pwm_id		= 2,
	.max_brightness	= 255,
	//.dft_brightness	= 224,
	.dft_brightness	= DEFAULT_BRIGHTNESS, /*ZTE: modified by tong.weili 更改初始默认亮度20111130*/
	//.pwm_period_ns	= 5000000,
	.pwm_period_ns	= 1000000,//tong test
	.init		= ventana_backlight_init,
	.exit		= ventana_backlight_exit,
	.notify		= ventana_backlight_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb   = ventana_disp1_check_fb,
};

static struct platform_device ventana_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &ventana_backlight_data,
	},
};

#ifdef CONFIG_TEGRA_DC
static int ventana_panel_enable(void)
{
    int ret;

    printk("+++++++++++++++++++++++++++enter Panel_enable\n");

    #if 1
        if (!ventana_LDO0) {
            ventana_LDO0 = regulator_get(NULL, "vdd_ldo0"); /* LD00 */
            if (IS_ERR_OR_NULL(ventana_LDO0)) {
            	pr_err("dsi: couldn't get regulator vdd_ldo0\n");
            	ventana_LDO0 = NULL;
            	return PTR_ERR(ventana_LDO0);
            }

            regulator_enable(ventana_LDO0);

        }

#endif
 #if 1 

    if (!ventana_LDO9) {
        ventana_LDO9 = regulator_get(NULL, "vdd_ldo9"); /* LD09 */
        if (IS_ERR_OR_NULL(ventana_LDO9)) {
        	pr_err("dsi: couldn't get regulator vdd_ldo9\n");
        	ventana_LDO9 = NULL;
        	return PTR_ERR(ventana_LDO9);
        }
        	/* set vddio_vi voltage to 3.3v */
        ret = regulator_set_voltage(ventana_LDO9, 3300*1000, 3300*1000);
        if (ret) {
              pr_err("%s: Failed to set vdd_ldo3 to 2.85v\n", __func__);
              regulator_put(ventana_LDO9);
             return PTR_ERR(ventana_LDO9);
        }
        regulator_enable(ventana_LDO9); 

    }



#endif   

    printk("+++++++++++++++++++++++++++Panel_enable\n");
    //zte lipeng10094834 add end at 2011.07.04
    return 0;
}

static int ventana_panel_disable(void)
{
   //// int ret;
    printk("+++++++++++++++++++++++++++Panel_disable\n");
    return 0;

}

static int ventana_hdmi_enable(void)
{
#ifdef CONFIG_MHL_SII8334

    printk("[MHL] init irq & power ventana_hdmi_enable\n");
    if (!ventana_LDO8) {
		ventana_LDO8 = regulator_get(NULL, "vdd_ldo8"); /* LD03 */
		if (IS_ERR_OR_NULL(ventana_LDO8)) {
			printk(KERN_ERR "dsi: couldn't get regulator vdd_ldo8\n");
			ventana_LDO8 = NULL;
			return PTR_ERR(ventana_LDO8);
		}
        }
        /* set HDMI voltage to 3V3*/
        regulator_enable(ventana_LDO8); 
#endif
        #if 0
	if (!ventana_hdmi_reg) {
		ventana_hdmi_reg = regulator_get(NULL, "avdd_hdmi"); /* LD07 */
		if (IS_ERR_OR_NULL(ventana_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			ventana_hdmi_reg = NULL;
			return PTR_ERR(ventana_hdmi_reg);
		}
	}
	regulator_enable(ventana_hdmi_reg);

	if (!ventana_hdmi_pll) {
		ventana_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll"); /* LD08 */
		if (IS_ERR_OR_NULL(ventana_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			ventana_hdmi_pll = NULL;
			regulator_disable(ventana_hdmi_reg);
			ventana_hdmi_reg = NULL;
			return PTR_ERR(ventana_hdmi_pll);
		}
	}
	regulator_enable(ventana_hdmi_pll);
	#endif
	return 0;
}

static int ventana_hdmi_disable(void)
{
#if 0
	regulator_disable(ventana_hdmi_reg);
	regulator_disable(ventana_hdmi_pll);
#endif

#ifdef CONFIG_MHL_SII8334
    printk("[MHL] power disable,ventana_hdmi_disable\n");
    regulator_disable(ventana_LDO8);
#endif
	return 0;
}

static struct resource ventana_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
	},
/*nvidia danny add  at 2011.07.29*/
        {
                .name   = "dsi_regs",
                .start  = TEGRA_DSI_BASE,
                .end    = TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
                .flags  = IORESOURCE_MEM,
        },
/*nvidia danny add  end at 2011.07.29*/
};

static struct resource ventana_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
static u8 OTM_9608[] = {0xFF,0x96,0x08,0x01};			   // Enable cmd
static u8 OTM_9608_0[] = {0x00,0x80};			   // Enable cmd
static u8 OTM_9608_1[] = {0xFF,0x96,0x08}; 
static u8 OTM_9608_2[] = {0x00,0x00}; //OTP select region
static u8 OTM_9608_3[] = {0xA0,0x00};			

static u8 OTM_9608_4[] = {0x00,0x80}; //Command Set Option Parameter
static u8 OTM_9608_5[] = {0xB3,0x00,0x00,0x20,0x00,0x00};	

static u8 OTM_9608_6[] = {0x00,0xC0}; //SRAM Setting 
static u8 OTM_9608_7[] = {0xB3,0x09};																										

static u8 OTM_9608_8[] = {0x00,0x80};	//TCON Setting Parameters																																		
static u8 OTM_9608_9[] = {0xC0,0x00,0x48,0x00,0x10,0x10,0x00,0x47,0x10,0x10}; 
	
static u8 OTM_9608_10[] = {0x00,0x92};	//Panel Timing Setting Parameter.
static u8 OTM_9608_11[] = {0xC0,0x00,0x10,0x00,0x13};	
	
static u8 OTM_9608_12[] = {0x00,0xA2};	//Panel Timing Setting Parameter.
static u8 OTM_9608_13[] = {0xC0,0x0C,0x05,0x02};	
	
static u8 OTM_9608_14[] = {0x00,0xB3};	//Interval Scan Frame Setting																																
static u8 OTM_9608_15[] = {0xC0,0x00,0x50};																																
	
static u8 OTM_9608_16[] = {0x00,0x81}; //Oscillator Adjustment for Idle/Normal Mode
static u8 OTM_9608_17[] = {0xC1,0x55}; 
	
static u8 OTM_9608_18[] = {0x00,0x80};		
static u8 OTM_9608_19[] = {0xC4,0x00,0x84,0xFC};

static u8 OTM_9608_20[] = {0x00,0xA0};		
static u8 OTM_9608_21[] = {0xB3,0x10,0x00};

static u8 OTM_9608_22[] = {0x00,0xA0};		
static u8 OTM_9608_23[] = {0xC0,0x00};
	
static u8 OTM_9608_24[] = {0x00,0xA0}; //DC2DC Setting
static u8 OTM_9608_25[] = {0xC4,0x33,0x09,0x90,0x2B,0x33,0x09,0x90,0x54};
	
static u8 OTM_9608_26[] = {0x00,0x80};																																			
static u8 OTM_9608_27[] = {0xC5,0x08,0x00,0xA0,0x11};																													
	
static u8 OTM_9608_28[] = {0x00,0x90};	//Power Control Setting2 for Normal Mode
static u8 OTM_9608_29[] = {0xC5,0x96,0x57,0x01,0x57,0x33,0x33,0x34};	
	
static u8 OTM_9608_30[] = {0x00,0xA0}; //Power Control Setting3 for Idle Mode
static u8 OTM_9608_31[] = {0xC5,0x96,0x57,0x00,0x57,0x33,0x33,0x34};
	
static u8 OTM_9608_32[] = {0x00,0xB0};	//Power Control Setting3 for DC Voltage Settings
static u8 OTM_9608_33[] = {0xC5,0x04,0xAC,0x01,0x00,0x71,0xB1,0x83};
	
static u8 OTM_9608_34[] = {0x00,0x00};	//Vcom setting
static u8 OTM_9608_35[] = {0xD9,0x5e}; //0x61

static u8 OTM_9608_36[] = {0x00,0x80};	//ABC Parameter.
static u8 OTM_9608_37[] = {0xC6,0x64};

static u8 OTM_9608_38[] = {0x00,0xB0};	//ABC Parameter.
static u8 OTM_9608_39[] = {0xC6,0x03,0x10,0x00,0x1F,0x12};

static u8 OTM_9608_40[] = {0x00,0x00}; //ID1
static u8 OTM_9608_41[] = {0xD0,0x40};

static u8 OTM_9608_42[] = {0x00,0x00};	//ID2&ID3
static u8 OTM_9608_43[] = {0xD1,0x00,0x00};


static u8 OTM_9608_44[] = {0x00,0xB7};	
static u8 OTM_9608_45[] = {0xB0,0x10};
	
static u8 OTM_9608_46[] = {0x00,0xC0};
static u8 OTM_9608_47[] = {0xB0,0x55};	

static u8 OTM_9608_48[] = {0x00,0xB1};
static u8 OTM_9608_49[] = {0xB0,0x03};
	
static u8 OTM_9608_50[] = {0x00,0x81};							 //Sharpness Off	
static u8 OTM_9608_51[] = {0xD6,0x00};


static u8 OTM_9608_52[] = {0x00,0x00};																						//Address Shift
static u8 OTM_9608_53[] = {0xE1,0x01,0x0D,0x13,0x0F,0x07,0x11,0x0B,0x0A,0x03,0x06,0x0B,0x08,0x0D,0x0E,0x09,0x01};  
																						//Wait 10ms
		
static u8 OTM_9608_54[] = {0x00,0x00};																						//Address Shift
static u8 OTM_9608_55[] = {0xE2,0x02,0x0F,0x15,0x0E,0x08,0x10,0x0B,0x0C,0x02,0x04,0x0B,0x04,0x0E,0x0D,0x08,0x00};  
				


static u8 OTM_9608_56[] = {0x00,0x80};	//TCON GOA WAVE(Panel timing state control)
static u8 OTM_9608_57[] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};															
	
static u8 OTM_9608_58[] = {0x00,0x90};	//TCON GOA WAVE(Panel timing state control)																																					
static u8 OTM_9608_59[] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};    
	
static u8 OTM_9608_60[] = {0x00,0xA0};	//TCON GOA WAVE(Panel timing state control)																																					
static u8 OTM_9608_61[] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};    
	
static u8 OTM_9608_62[] = {0x00,0xB0};	//TCON GOA WAVE(Panel timing state control)																									
static u8 OTM_9608_63[] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};															
	
static u8 OTM_9608_64[] = {0x00,0xC0};	//TCON_GOA_WAVE (panel timing state control)																																				
static u8 OTM_9608_65[] = {0xCB,0x04,0x04,0x04,0x04,0x08,0x04,0x08,0x04,0x08,0x04,0x08,0x04,0x04,0x04,0x08};    

static u8 OTM_9608_66[] = {0x00,0xD0};	//TCON GOA WAVE(Panel timing state control)																																					
static u8 OTM_9608_67[] = {0xCB,0x08,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x08,0x04,0x08,0x04,0x08,0x04};    
	
static u8 OTM_9608_68[] = {0x00,0xE0}; //TCON GOA WAVE(Panel timing state control)	
static u8 OTM_9608_69[] = {0xCB,0x08,0x04,0x04,0x04,0x08,0x08,0x00,0x00,0x00,0x00};
	
static u8 OTM_9608_70[] = {0x00,0xF0}; //TCON GOA WAVE(Panel timing state control)
static u8 OTM_9608_71[] = {0xCB,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};															
	
static u8 OTM_9608_72[] = {0x00,0x80};	//TCON GOA WAVE(Panel timing state control)
static u8 OTM_9608_73[] = {0xCC,0x26,0x25,0x23,0x24,0x00,0x0F,0x00,0x0D,0x00,0x0B};															
	
static u8 OTM_9608_74[] = {0x00,0x90};	//TCON GOA WAVE(Panel timing state control)																																
static u8 OTM_9608_75[] = {0xCC,0x00,0x09,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x26,0x25,0x21,0x22,0x00};    
	
static u8 OTM_9608_76[] = {0x00,0xA0};	//TCON GOA WAVE(Panel timing state control)																																			
static u8 OTM_9608_77[] = {0xCC,0x10,0x00,0x0E,0x00,0x0C,0x00,0x0A,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00};    
	
static u8 OTM_9608_78[] = {0x00,0xB0};	//TCON GOA WAVE(Panel timing state control)																								
static u8 OTM_9608_79[] = {0xCC,0x25,0x26,0x21,0x22,0x00,0x0A,0x00,0x0C,0x00,0x0E};															
	
static u8 OTM_9608_80[] = {0x00,0xC0};	//TCON GOA WAVE(Panel timing state control)																																		
static u8 OTM_9608_81[] = {0xCC,0x00,0x10,0x04,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x25,0x26,0x23,0x24,0x00};    
	
static u8 OTM_9608_82[] = {0x00,0xD0};	//TCON GOA WAVE(Panel timing state control)																																		
static u8 OTM_9608_83[] = {0xCC,0x09,0x00,0x0B,0x00,0x0D,0x00,0x0F,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00};    
	
static u8 OTM_9608_84[] = {0x00,0x80};	//GOA VST Setting																																				
static u8 OTM_9608_85[] = {0xCE,0x8A,0x03,0x06,0x89,0x03,0x06,0x88,0x03,0x06,0x87,0x03,0x06};    								
	
static u8 OTM_9608_86[] = {0x00,0x90}; //GOA VEND and Group Setting
static u8 OTM_9608_87[] = {0xCE,0xF0,0x00,0x00,0xF0,0x00,0x00,0xF0,0x00,0x00,0xF0,0x00,0x00,0x00,0x00};    			
	
static u8 OTM_9608_88[] = {0x00,0xA0};	//GOA CLK1 and GOA CLK2 Setting
static u8 OTM_9608_89[] = {0xCE,0x38,0x02,0x03,0xC1,0x00,0x06,0x00,0x38,0x01,0x03,0xC2,0x00,0x06,0x00};    			
	
static u8 OTM_9608_90[] = {0x00,0xB0};	//GOA CLK3 and GOA CLK4 Setting																																					//Address Shift
static u8 OTM_9608_91[] = {0xCE,0x38,0x00,0x03,0xC3,0x00,0x06,0x00,0x30,0x00,0x03,0xC4,0x00,0x06,0x00};    			
	
static u8 OTM_9608_92[] = {0x00,0xC0};	//GOA CLKB1 and GOA CLKB2 Setting																																				
static u8 OTM_9608_93[] = {0xCE,0x38,0x06,0x03,0xBD,0x00,0x06,0x00,0x38,0x05,0x03,0xBE,0x00,0x06,0x00};    			
	
static u8 OTM_9608_94[] = {0x00,0xD0};	//GOA CLKB3 and GOA CLKB4 Setting																																				
static u8 OTM_9608_95[] = {0xCE,0x38,0x04,0x03,0xBF,0x00,0x06,0x00,0x38,0x03,0x03,0xC0,0x00,0x06,0x00};    			

static u8 OTM_9608_96[] = {0x00,0x80};	//GOA CLKC1 and GOA CLKC2 Setting																																				
static u8 OTM_9608_97[] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};    			
	
static u8 OTM_9608_98[] = {0x00,0x90};	//GOA CLKC3 and GOA CLKC4 Setting																																					
static u8 OTM_9608_99[] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};    			
	
static u8 OTM_9608_100[] = {0x00,0xA0};	//GOA CLKD1 and GOA CLKD2 Setting
static u8 OTM_9608_101[] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};    			
	
static u8 OTM_9608_102[] = {0x00,0xB0};	//GOA CLKD3 and GOA CLKD4 Setting
static u8 OTM_9608_103[] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};    			
	
static u8 OTM_9608_104[] = {0x00,0xC0};	//GOA ECLK Setting and GOA Other Options1 and GOA Signal Toggle Option Setting																																				
static u8 OTM_9608_105[] = {0xCF,0x02,0x02,0x20,0x20,0x00,0x00,0x01,0x00,0x00,0x02};    	


static u8 OTM_9608_106[] = {0x00,0x00};			   
static u8 OTM_9608_107[] = {0xD8,0xA7,0xA7}; 

static u8 OTM_9608_108[] = {0x00,0x88};			   
static u8 OTM_9608_109[] = {0xc4,0x40}; 
static u8 OTM_9608_110[] = {0x00,0x00};			   
static u8 OTM_9608_111[] = {0xff,0xff,0xff,0xff}; 

static struct tegra_dsi_cmd otm_9608_ventana_dsi_init_cmd[]= {
	DSI_CMD_LONG(0x39, OTM_9608),   
	DSI_CMD_LONG(0x39, OTM_9608_0),  
	DSI_CMD_LONG(0x39, OTM_9608_1),   
	DSI_CMD_LONG(0x39, OTM_9608_2),   
	DSI_CMD_LONG(0x39, OTM_9608_3),   
	DSI_CMD_LONG(0x39, OTM_9608_4),   
	DSI_CMD_LONG(0x39, OTM_9608_5),   
	DSI_CMD_LONG(0x39, OTM_9608_6),   
	DSI_CMD_LONG(0x39, OTM_9608_7),   
	DSI_CMD_LONG(0x39, OTM_9608_8),  
	DSI_CMD_LONG(0x39, OTM_9608_9),   
	DSI_CMD_LONG(0x39, OTM_9608_10),   
	DSI_CMD_LONG(0x39, OTM_9608_11),   
	DSI_CMD_LONG(0x39, OTM_9608_12),  
	DSI_CMD_LONG(0x39, OTM_9608_13),   
	DSI_CMD_LONG(0x39, OTM_9608_14),   
	DSI_CMD_LONG(0x39, OTM_9608_15),   
	DSI_CMD_LONG(0x39, OTM_9608_16),   
	DSI_CMD_LONG(0x39, OTM_9608_17),   
	DSI_CMD_LONG(0x39, OTM_9608_18),   
	DSI_CMD_LONG(0x39, OTM_9608_19),   
	DSI_CMD_LONG(0x39, OTM_9608_20),   
	DSI_CMD_LONG(0x39, OTM_9608_21),   
	DSI_CMD_LONG(0x39, OTM_9608_22),   
	DSI_CMD_LONG(0x39, OTM_9608_23),  
	DSI_CMD_LONG(0x39, OTM_9608_24),   
	DSI_CMD_LONG(0x39, OTM_9608_25),   
	DSI_CMD_LONG(0x39, OTM_9608_26),   
	DSI_CMD_LONG(0x39, OTM_9608_27),  
	DSI_CMD_LONG(0x39, OTM_9608_28),   
	DSI_CMD_LONG(0x39, OTM_9608_29),   
	DSI_CMD_LONG(0x39, OTM_9608_30),  
	DSI_CMD_LONG(0x39, OTM_9608_31),   
	DSI_CMD_LONG(0x39, OTM_9608_32),   
	DSI_CMD_LONG(0x39, OTM_9608_33),  
	DSI_CMD_LONG(0x39, OTM_9608_34),   
	DSI_CMD_LONG(0x39, OTM_9608_35),   
	DSI_CMD_LONG(0x39, OTM_9608_36),   
	DSI_CMD_LONG(0x39, OTM_9608_37),  
	DSI_CMD_LONG(0x39, OTM_9608_38),   
	DSI_CMD_LONG(0x39, OTM_9608_39),   
	DSI_CMD_LONG(0x39, OTM_9608_40),  
	DSI_CMD_LONG(0x39, OTM_9608_41),   
	DSI_CMD_LONG(0x39, OTM_9608_42),   
	DSI_CMD_LONG(0x39, OTM_9608_43),  
	DSI_CMD_LONG(0x39, OTM_9608_44),   
	DSI_CMD_LONG(0x39, OTM_9608_45),   
	DSI_CMD_LONG(0x39, OTM_9608_46),   
	DSI_CMD_LONG(0x39, OTM_9608_47),  
	DSI_CMD_LONG(0x39, OTM_9608_48),   
	DSI_CMD_LONG(0x39, OTM_9608_49),   
	DSI_CMD_LONG(0x39, OTM_9608_50),  
	DSI_CMD_LONG(0x39, OTM_9608_51),   
	DSI_CMD_LONG(0x39, OTM_9608_52),   
	DSI_CMD_LONG(0x39, OTM_9608_53),  
//	DSI_DLY_MS(10),
	DSI_CMD_LONG(0x39, OTM_9608_54),   
	DSI_CMD_LONG(0x39, OTM_9608_55),   
//	DSI_DLY_MS(10),
	DSI_CMD_LONG(0x39, OTM_9608_56),   
	DSI_CMD_LONG(0x39, OTM_9608_57),  
	DSI_CMD_LONG(0x39, OTM_9608_58),   
	DSI_CMD_LONG(0x39, OTM_9608_59),   
	DSI_CMD_LONG(0x39, OTM_9608_60),  
	DSI_CMD_LONG(0x39, OTM_9608_61),   
	DSI_CMD_LONG(0x39, OTM_9608_62),   
	DSI_CMD_LONG(0x39, OTM_9608_63),  
	DSI_CMD_LONG(0x39, OTM_9608_64),   
	DSI_CMD_LONG(0x39, OTM_9608_65),   
	DSI_CMD_LONG(0x39, OTM_9608_66),   
	DSI_CMD_LONG(0x39, OTM_9608_67),  
	DSI_CMD_LONG(0x39, OTM_9608_68),   
	DSI_CMD_LONG(0x39, OTM_9608_69),   
	DSI_CMD_LONG(0x39, OTM_9608_70),      
 	DSI_CMD_LONG(0x39, OTM_9608_71),   
	DSI_CMD_LONG(0x39, OTM_9608_72),   
	DSI_CMD_LONG(0x39, OTM_9608_73),  
	DSI_CMD_LONG(0x39, OTM_9608_74),   
	DSI_CMD_LONG(0x39, OTM_9608_75),   
	DSI_CMD_LONG(0x39, OTM_9608_76),   
	DSI_CMD_LONG(0x39, OTM_9608_77),  
	DSI_CMD_LONG(0x39, OTM_9608_78),   
	DSI_CMD_LONG(0x39, OTM_9608_79),   
	DSI_CMD_LONG(0x39, OTM_9608_80),  
 	DSI_CMD_LONG(0x39, OTM_9608_81),   
	DSI_CMD_LONG(0x39, OTM_9608_82),   
	DSI_CMD_LONG(0x39, OTM_9608_83),  
	DSI_CMD_LONG(0x39, OTM_9608_84),   
	DSI_CMD_LONG(0x39, OTM_9608_85),   
	DSI_CMD_LONG(0x39, OTM_9608_86),   
	DSI_CMD_LONG(0x39, OTM_9608_87),  
	DSI_CMD_LONG(0x39, OTM_9608_88),   
	DSI_CMD_LONG(0x39, OTM_9608_89),   
	DSI_CMD_LONG(0x39, OTM_9608_90),  
 	DSI_CMD_LONG(0x39, OTM_9608_91),   
	DSI_CMD_LONG(0x39, OTM_9608_92),   
	DSI_CMD_LONG(0x39, OTM_9608_93),  
	DSI_CMD_LONG(0x39, OTM_9608_94),   
	DSI_CMD_LONG(0x39, OTM_9608_95),   
	DSI_CMD_LONG(0x39, OTM_9608_96),   
	DSI_CMD_LONG(0x39, OTM_9608_97),  
	DSI_CMD_LONG(0x39, OTM_9608_98),   
	DSI_CMD_LONG(0x39, OTM_9608_99),   
	DSI_CMD_LONG(0x39, OTM_9608_100),  
	DSI_CMD_LONG(0x39, OTM_9608_101),  
	DSI_CMD_LONG(0x39, OTM_9608_102),      
 	DSI_CMD_LONG(0x39, OTM_9608_103),  
	DSI_CMD_LONG(0x39, OTM_9608_104),  
	DSI_CMD_LONG(0x39, OTM_9608_105),      
 	DSI_CMD_LONG(0x39, OTM_9608_106),  
 	DSI_CMD_LONG(0x39, OTM_9608_107),      
	DSI_CMD_LONG(0x39, OTM_9608_108),  
	DSI_CMD_LONG(0x39, OTM_9608_109),      
 	DSI_CMD_LONG(0x39, OTM_9608_110),  
 	DSI_CMD_LONG(0x39, OTM_9608_111),      
#if(1)
        DSI_CMD_SHORT(0x15, 0x35, 0x00),
#endif

	DSI_CMD_SHORT(0x05, 0x11, 0x00),

	DSI_DLY_MS(120),
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(5),
	DSI_DLY_MS(5),
	
};

/*static u8 s_command1[] = {0xFF, 0xAA, 0x55, 0x25, 0x01};
static u8 s_command2[] = {0xF4, 0x00, 0x48, 0x00, 0x00,0x40, 0x00, 0x00, 0x00,0x00, 0x30, 0x45, 0x04,0x31, 0x21, 0x34, 0x32};
static u8 s_command3[] = {0xFF, 0xAA, 0x55, 0x25, 0x01};
static u8 s_command4[] = {0xF8, 0x08, 0x03, 0x20, 0x33,0x13, 0x00, 0x24, 0x40,0xc6, 0xA3, 0x00, 0x00,0x11};
static u8 s_command_24[] = {0x36, 0x08}; 
static u8 init_cmd[] = {0xB9,0xFF,0x83,0x92};
static u8 eq_cmd[] = {0xD5,0x00,0x00,0x02};
static u8 ptbf_cmd[] = {0xBF,0x05,0x60,0x02};*/
static u8 s_command1[] = {0xF0, 0x55, 0xAA, 0x52, 0x08,0x00};
static u8 s_command2[] = {0xBC, 0x00, 0x00, 0x00}; 
static u8 s_0[] = { 0x11};
static u8 s_1[] = {0xFF,0xAA,0x55,0x25,0x01};
static u8 s_2[] = {0xF3, 0x02,0x03,0x07,0x15};
static u8 s_3[] = {0xF0, 0x55,0xAA,0x52,0x08,0x00};
static u8 s_4[] = {0xB8,0x01,0x02,0x02,0x02};
static u8 s_5[] = {0xBC,0x05,0x05,0x05};
static u8 s_6[] = {0xF0,0x55,0xAA,0x52,0x08,0x01};
static u8 s_7[] = {0xB0,0x05,0x05,0x05};
static u8 s_8[] = {0xB1,0x05,0x05,0x05};
static u8 s_9[] = {0xB6,0x44,0x44,0x44};
static u8 s_10[] = {0xB7,0x34,0x34,0x34};
static u8 s_11[] = {0xBA,0x24,0x24,0x24};
static u8 s_12[] = {0xBC, 0x00,0x88,0x00};
static u8 s_13[] = {0xBD,0x00,0x88,0x00};
static u8 s_14[] = {0xBE, 0x4F};
static u8 s_15[] = {0xD1,0x00,0x29,0x00,0x32,0x00,0x46,0x00,0x59,0x00,0x69,0x00,0x86,0x00,0x9C,0x00,0xC3};
static u8 s_15_1[] = {0xD2,0x00,0xE6,0x01,0x1B,0x01,0x43,0x01,0x83,0x01,0xB2,0x01,0xB3,0x01,0xDE,0x02,0x07}; 
static u8 s_15_2[] = {0xD3,0x02,0x20,0x02,0x3F,0x02,0x53,0x02,0x78,0x02,0x95,0x02,0xC6,0x02,0xEE,0x03,0x30}; 
static u8 s_15_3[] = {0xD4, 0x03,0xF0,0x03,0xF4 };
static u8 s_16[] = {0xD5,0x00,0x29,0x00,0x32,0x00,0x46,0x00,0x59,0x00,0x69,0x00,0x86,0x00,0x9C,0x00,0xC3};
static u8 s_16_1[] = {0xD6,0x00,0xE6,0x01,0x1B,0x01,0x43,0x01,0x83,0x01,0xB2,0x01,0xB3,0x01,0xDE,0x02,0x07};
static u8 s_16_2[] = {0xD7,0x02,0x20,0x02,0x3F,0x02,0x53,0x02,0x78,0x02,0x95,0x02,0xC6,0x02,0xEE,0x03,0x30};
static u8 s_16_3[] = {0xD8, 0x03,0xF0,0x03,0xF4  };
static u8 s_17[] = {0xD9, 0x00,0x29,0x00,0x32,0x00,0x46,0x00,0x59,0x00,0x69,0x00,0x86,0x00,0x9C,0x00,0xC3};
static u8 s_17_1[] = {0xDD, 0x00,0xE6,0x01,0x1B,0x01,0x43,0x01,0x83,0x01,0xB2,0x01,0xB3,0x01,0xDE,0x02,0x07};
static u8 s_17_2[] = {0xDE, 0x02,0x20,0x02,0x3F,0x02,0x53,0x02,0x78,0x02,0x95,0x02,0xC6,0x02,0xEE,0x03,0x30};
static u8 s_17_3[] = {0xDF,0x03,0xF0,0x03,0xF4   };
static u8 s_18[] = {0xE0, 0x00,0x29,0x00,0x32,0x00,0x46,0x00,0x59,0x00,0x69,0x00,0x86,0x00,0x9C,0x00,0xC3};
static u8 s_18_1[] = {0xE1, 0x00,0xE6,0x01,0x1B,0x01,0x43,0x01,0x83,0x01,0xB2,0x01,0xB3,0x01,0xDE,0x02,0x07};
static u8 s_18_2[] = {0xE2,0x02,0x20,0x02,0x3F,0x02,0x53,0x02,0x78,0x02,0x95,0x02,0xC6,0x02,0xEE,0x03,0x30};
static u8 s_18_3[] = {0xE3, 0x03,0xF0,0x03,0xF4   };
static u8 s_19[] = {0xE4,0x00,0x29,0x00,0x32,0x00,0x46,0x00,0x59,0x00,0x69,0x00,0x86,0x00,0x9C,0x00,0xC3};
static u8 s_19_1[] = {0xE5, 0x00,0xE6,0x01,0x1B,0x01,0x43,0x01,0x83,0x01,0xB2,0x01,0xB3,0x01,0xDE,0x02,0x07};
static u8 s_19_2[] = {0xE6,0x02,0x20,0x02,0x3F,0x02,0x53,0x02,0x78,0x02,0x95,0x02,0xC6,0x02,0xEE,0x03,0x30};
static u8 s_19_3[] = {0xE7,0x03,0xF0,0x03,0xF4   };
static u8 s_20[] = {0xE8, 0x00,0x29,0x00,0x32,0x00,0x46,0x00,0x59,0x00,0x69,0x00,0x86,0x00,0x9C,0x00,0xC3};
static u8 s_20_1[] = {0xE9, 0x00,0xE6,0x01,0x1B,0x01,0x43,0x01,0x83,0x01,0xB2,0x01,0xB3,0x01,0xDE,0x02,0x07};
static u8 s_20_2[] = {0xEA, 0x02,0x20,0x02,0x3F,0x02,0x53,0x02,0x78,0x02,0x95,0x02,0xC6,0x02,0xEE,0x03,0x30};
static u8 s_20_3[] = {0xEB, 0x03,0xF0,0x03,0xF4   };

static u8 s_lg_1[] = {0xFF,0xAA,0x55,0x25,0x01,0x01};
static u8 s_lg_2[] = {0xF2,0x00,0x00,0x4A,0x0A,0xA8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x01,0x51,0x00,0x01,0x00,0x01};
static u8 s_lg_3[] = {0xF3,0x02,0x03,0x07,0x45,0x88,0xD1,0x0D};
static u8 s_lg_4[] = {0xF0,0x55,0xAA,0x52,0x08,0x00};
static u8 s_lg_5[] = {0xB1,0xCC,0x00,0x00};
static u8 s_lg_6[] = {0xB8,0x01,0x02,0x02,0x02};
static u8 s_lg_7[] = {0xC9,0x63,0x06,0x0D,0x1A,0x17,0x00};
static u8 s_lg_8[] = {0xF0,0x55,0xAA,0x52,0x08,0x01};
static u8 s_lg_9[] = {0xB0,0x05,0x05,0x05};
static u8 s_lg_10[] = {0xB1,0x05,0x05,0x05};
static u8 s_lg_11[] = {0xB2,0x01,0x01,0x01};
static u8 s_lg_12[] = {0xB3,0x0E,0x0E,0x0E};
static u8 s_lg_13[] = {0xB4,0x08,0x08,0x08};
static u8 s_lg_14[] = {0xB6,0x44,0x44,0x44};
static u8 s_lg_15[] = {0xB7,0x34,0x34,0x34};
static u8 s_lg_16[] = {0xB8,0x10,0x10,0x10};
static u8 s_lg_17[] = {0xB9,0x26,0x26,0x26};
static u8 s_lg_18[] = {0xBA,0x34,0x34,0x34};
static u8 s_lg_19[] = {0xBC,0x00,0xC8,0x00};
static u8 s_lg_20[] = {0xBD,0x00,0xC8,0x00};
static u8 s_lg_21[] = {0xBE,0x75};
static u8 s_lg_22[] = {0xC0,0x04,0x00};
static u8 s_lg_23[] = {0xCA,0x00};
static u8 s_lg_24[] = {0xD0,0x0A,0x10,0x0D,0x0F};
static u8 s_lg_25[] = {0xD1,0x00,0x70,0x00,0xCE,0x00,0xF7,0x01,0x10,0x01,0x21,0x01,0x44,0x01,0x62,0x01,0x8D};
static u8 s_lg_26[] = {0xD2,0x01,0xAF,0x01,0xE4,0x02,0x0C,0x02,0x4D,0x02,0x82,0x02,0x84,0x02,0xB8,0x02,0xF0};
static u8 s_lg_27[] = {0xD3,0x03,0x14,0x03,0x42,0x03,0x5E,0x03,0x80,0x03,0x97,0x03,0xB0,0x03,0xC0,0x03,0xDF};
static u8 s_lg_28[] = {0xD4,0x03,0xFD,0x03,0xFF};
static u8 s_lg_29[] = {0xD5,0x00,0x70,0x00,0xCE,0x00,0xF7,0x01,0x10,0x01,0x21,0x01,0x44,0x01,0x62,0x01,0x8D};
static u8 s_lg_30[] = {0xD6,0x01,0xAF,0x01,0xE4,0x02,0x0C,0x02,0x4D,0x02,0x82,0x02,0x84,0x02,0xB8,0x02,0xF0};
static u8 s_lg_31[] = {0xD7,0x03,0x14,0x03,0x42,0x03,0x5E,0x03,0x80,0x03,0x97,0x03,0xB0,0x03,0xC0,0x03,0xDF};
static u8 s_lg_32[] = {0xD8,0x03,0xFD,0x03,0xFF};
static u8 s_lg_33[] = {0xD9,0x00,0x70,0x00,0xCE,0x00,0xF7,0x01,0x10,0x01,0x21,0x01,0x44,0x01,0x62,0x01,0x8D};
static u8 s_lg_34[] = {0xDD,0x01,0xAF,0x01,0xE4,0x02,0x0C,0x02,0x4D,0x02,0x82,0x02,0x84,0x02,0xB8,0x02,0xF0};
static u8 s_lg_35[] = {0xDE,0x03,0x14,0x03,0x42,0x03,0x5E,0x03,0x80,0x03,0x97,0x03,0xB0,0x03,0xC0,0x03,0xDF};
static u8 s_lg_36[] = {0xDF,0x03,0xFD,0x03,0xFF};
static u8 s_lg_37[] = {0xE0,0x00,0x70,0x00,0xCE,0x00,0xF7,0x01,0x10,0x01,0x21,0x01,0x44,0x01,0x62,0x01,0x8D};
static u8 s_lg_38[] = {0xE1,0x01,0xAF,0x01,0xE4,0x02,0x0C,0x02,0x4D,0x02,0x82,0x02,0x84,0x02,0xB8,0x02,0xF0};
static u8 s_lg_39[] = {0xE2,0x03,0x14,0x03,0x42,0x03,0x5E,0x03,0x80,0x03,0x97,0x03,0xB0,0x03,0xC0,0x03,0xDF};
static u8 s_lg_40[] = {0xE3,0x03,0xFD,0x03,0xFF};
static u8 s_lg_41[] = {0xE4,0x00,0x70,0x00,0xCE,0x00,0xF7,0x01,0x10,0x01,0x21,0x01,0x44,0x01,0x62,0x01,0x8D};
static u8 s_lg_42[] = {0xE5,0x01,0xAF,0x01,0xE4,0x02,0x0C,0x02,0x4D,0x02,0x82,0x02,0x84,0x02,0xB8,0x02,0xF0};
static u8 s_lg_43[] = {0xE6,0x03,0x14,0x03,0x42,0x03,0x5E,0x03,0x80,0x03,0x97,0x03,0xB0,0x03,0xC0,0x03,0xDF};
static u8 s_lg_44[] = {0xE7,0x03,0xFD,0x03,0xFF};
static u8 s_lg_45[] = {0xE8,0x00,0x70,0x00,0xCE,0x00,0xF7,0x01,0x10,0x01,0x21,0x01,0x44,0x01,0x62,0x01,0x8D};
static u8 s_lg_46[] = {0xE9,0x01,0xAF,0x01,0xE4,0x02,0x0C,0x02,0x4D,0x02,0x82,0x02,0x84,0x02,0xB8,0x02,0xF0};
static u8 s_lg_47[] = {0xEA,0x03,0x14,0x03,0x42,0x03,0x5E,0x03,0x80,0x03,0x97,0x03,0xB0,0x03,0xC0,0x03,0xDF};
static u8 s_lg_48[] = {0xEB,0x03,0xFD,0x03,0xFF};

static u8 hx8369_tianma_tn_para_0xB9[]={0xB9,0xFF,0x83,0x69};
//static NvU8 hx8369_tianma_tn_para_0xb0[3] = {0x01,0x0B};
static u8 hx8369_tianma_tn_para_0xb1[]={0xb1,0x01,0x00,0x34,0x0a,0x00,0x11,0x11,0x32,0x2f,0x3F,0x3F,
	0x01,0x3a,0x01,0xE6,0xE6,0xE6,0xE6,0xE6}; 
static u8 hx8369_tianma_tn_para_0xb2[]={0xb2,0x00,0x20,0x05,0x05,0x70,0x00,0xFF,0x00,0x00,0x00,0x00,
	0x03,0x03,0x00,0x01};  //Command MODE

static u8 hx8369_tianma_tn_para_0xb4[]={0xb4,0x02,0x1d,0x80,0x06,0x02}; 
//static NvU8 hx8369_tianma_tn_para_0x36[2]={0x36,0xc0};
static u8 hx8369_tianma_tn_para_0xcc[]={0xcc,0x00};
static u8 hx8369_tianma_tn_para_0xb6[]={0xb6,0x4d,0x4d};
static u8 hx8369_tianma_tn_para_0xd5[]={0xd5,0x00,0x01,0x03,0x34,0x01,0x06,0x00,0x70,0x11,0x13,0x00,
	0x00,0x60,0x24,0x71,0x35,0x00,0x00,0x71,0x05,0x60,0x24,0x07,0x0F,0x04,0x04}; 
static u8 hx8369_tianma_tn_para_0xe0[]={0xe0,0x00,0x02,0x0b,0x13,0x0b,0x3f,0x24,0x30,
	0x09,0x13,0x0f,0x15,0x15,0x14,0x15,0x0c,0x0c,
	0x00,0x02,0x0b,0x13,0x0b,0x3f,0x24,0x30,
	0x09,0x13,0x0f,0x15,0x15,0x14,0x15,0x0c,0x0c};
static u8 hx8369_tianma_tn_para_0x3a[]={0x3a,0x77}; 
static u8 hx8369_tianma_tn_para_0xba[]={0xba,0x00,0xA0,0xC6,0x00,0x0A,0x00,0x10,0x30,0x6F,0x02,0x11,0x18,0x40};

static struct tegra_dsi_cmd hx8369_tm_ventana_dsi_init_cmd[]= {
	DSI_CMD_LONG(0x39, hx8369_tianma_tn_para_0xB9),   
	DSI_CMD_LONG(0x39, hx8369_tianma_tn_para_0xb1),   
	DSI_CMD_LONG(0x39, hx8369_tianma_tn_para_0xb2),   
	DSI_CMD_LONG(0x39, hx8369_tianma_tn_para_0xb4),   
	DSI_CMD_LONG(0x39, hx8369_tianma_tn_para_0xcc),   
	DSI_CMD_LONG(0x39, hx8369_tianma_tn_para_0xb6),   
	DSI_CMD_LONG(0x39, hx8369_tianma_tn_para_0xd5),   
	DSI_CMD_LONG(0x39, hx8369_tianma_tn_para_0xe0),  
	DSI_CMD_LONG(0x39, hx8369_tianma_tn_para_0x3a),   
	DSI_CMD_LONG(0x39, hx8369_tianma_tn_para_0xba),   
    
	DSI_CMD_SHORT(0x05, 0x11, 0x00),
	DSI_DLY_MS(120),
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(5),
	DSI_DLY_MS(5),

};

static u8 hx8369_0xB9[]={0xB9,0xFF,0x83,0x69}; 
static u8 hx8369_0xb0[]={0xb0,0x01,0x09};
static u8 hx8369_0xb1[]={0xb1,0x01,0x00,0x34,0x07,0x00,0x0E,0x0E,
	0x21,0x29,0x3F,0x3F,0x01,0x63,0x01,0xE6,0xE6,0xE6,0xE6,0xE6};  
static u8 hx8369_0xb2[]={0xb2,0x00,0x20,0x07,0x07,0x70,0x00,0xFF,
	0x00,0x00,0x00,0x00,0x03,0x03,0x00,0x01};  //Command MODE
static u8 hx8369_0xb4[]={0xb4,0x02,0x18,0x80,0x10,0x01}; 
static u8 hx8369_0xb6[]={0xb6,0x1F,0x1F};
static u8 hx8369_0xcc[]={0xcc,0x00}; 
static u8 hx8369_0xd5[]={0xd5,0x00,0x01,0x00,0x00,0x01,0x06,0x10,
	0x80,0x73,0x37,0x23,0x01,0xB9,0x75,0xA8,0x64,0x00,0x00,0x41,0x06,
	0x50,0x07,0x07,0x0F,0x07,0x00}; 

static u8 hx8369_0xe0[]={0xe0,0x00,0x03,0x00,0x09,0x09,0x21,0x1B,0x2D,
	0x06,0x0c,0x10,0x15,0x16,0x14,0x16,0x12,0x18,0x00,0x03,0x00,0x09,0x09,
	0x21,0x1B,0x2D,0x06,0x0c,0x10,0x15,0x16,0x14,0x16,0x12,0x18};
static u8 hx8369_0x3a[]={0x3a,0x77}; 
static u8 hx8369_0xba[]={0xba,0x00,0xA0,0xC6,0x00,0x0A,0x02,0x10,0x30,
	0x6F,0x02,0x11,0x18,0x40}; 
static struct tegra_dsi_cmd hx8369_ventana_dsi_init_cmd[]= {
	DSI_CMD_LONG(0x39, hx8369_0xB9),   
	DSI_CMD_LONG(0x39, hx8369_0xb0),   
	DSI_CMD_LONG(0x39, hx8369_0xb1),   
	DSI_CMD_LONG(0x39, hx8369_0xb2),   
	DSI_CMD_LONG(0x39, hx8369_0xb4),   
	DSI_CMD_LONG(0x39, hx8369_0xb6),   
	DSI_CMD_LONG(0x39, hx8369_0xcc),   
	DSI_CMD_LONG(0x39, hx8369_0xd5),  
	DSI_CMD_LONG(0x39, hx8369_0xe0),   
	DSI_CMD_LONG(0x39, hx8369_0x3a),   
 	DSI_CMD_LONG(0x39, hx8369_0xba),     
	DSI_CMD_SHORT(0x05, 0x11, 0x00),
	DSI_DLY_MS(120),
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(5),
	DSI_DLY_MS(5),

};

static struct tegra_dsi_cmd lg_ventana_dsi_init_cmd[]= {
	DSI_CMD_LONG(0x39, s_lg_1),   
	DSI_CMD_LONG(0x39, s_lg_2),   
	DSI_CMD_LONG(0x39, s_lg_3),   
	DSI_CMD_LONG(0x39, s_lg_4),   
	DSI_CMD_LONG(0x39, s_lg_5),   
	DSI_CMD_LONG(0x39, s_lg_6),   
	DSI_CMD_LONG(0x39, s_lg_7),   
	DSI_CMD_LONG(0x39, s_lg_8),  
	DSI_CMD_LONG(0x39, s_lg_9),   
	DSI_CMD_LONG(0x39, s_lg_10),   
	DSI_CMD_LONG(0x39, s_lg_11),   
	DSI_CMD_LONG(0x39, s_lg_12),  
	DSI_CMD_LONG(0x39, s_lg_13),   
	DSI_CMD_LONG(0x39, s_lg_14),   
	DSI_CMD_LONG(0x39, s_lg_15),   
	DSI_CMD_LONG(0x39, s_lg_16),   
	DSI_CMD_LONG(0x39, s_lg_17),   
	DSI_CMD_LONG(0x39, s_lg_18),   
	DSI_CMD_LONG(0x39, s_lg_19),   
	DSI_CMD_LONG(0x39, s_lg_20),  
	DSI_CMD_LONG(0x39, s_lg_21),   
	DSI_CMD_LONG(0x39, s_lg_22),   
	DSI_CMD_LONG(0x39, s_lg_23),   
	DSI_CMD_LONG(0x39, s_lg_24), 
	DSI_CMD_LONG(0x39, s_lg_25),   
	DSI_CMD_LONG(0x39, s_lg_26),
	DSI_CMD_LONG(0x39, s_lg_27),   
	DSI_CMD_LONG(0x39, s_lg_28), 
	DSI_CMD_LONG(0x39, s_lg_29),   
	DSI_CMD_LONG(0x39, s_lg_30),
	DSI_CMD_LONG(0x39, s_lg_31),   
	DSI_CMD_LONG(0x39, s_lg_32), 
	DSI_CMD_LONG(0x39, s_lg_33),   
	DSI_CMD_LONG(0x39, s_lg_34),
	DSI_CMD_LONG(0x39, s_lg_35),   
	DSI_CMD_LONG(0x39, s_lg_36), 
	DSI_CMD_LONG(0x39, s_lg_37),   
	DSI_CMD_LONG(0x39, s_lg_38),
	DSI_CMD_LONG(0x39, s_lg_39),   
	DSI_CMD_LONG(0x39, s_lg_40),
	DSI_CMD_LONG(0x39, s_lg_41),   
	DSI_CMD_LONG(0x39, s_lg_42), 
	DSI_CMD_LONG(0x39, s_lg_43),   
	DSI_CMD_LONG(0x39, s_lg_44),
	DSI_CMD_LONG(0x39, s_lg_45),   
	DSI_CMD_LONG(0x39, s_lg_46), 
	DSI_CMD_LONG(0x39, s_lg_47),   
	DSI_CMD_LONG(0x39, s_lg_48),
    
	DSI_CMD_SHORT(0x05, 0x11, 0x00),
	DSI_DLY_MS(120),
	DSI_CMD_SHORT(0x05, 0x2c, 0x00),
	DSI_CMD_SHORT(0x05, 0x13, 0x00),
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(5),
	DSI_DLY_MS(5),

};

static struct tegra_dsi_cmd auo_ventana_dsi_init_cmd[]= {
	DSI_CMD_LONG(0x39, s_1),   
	DSI_CMD_LONG(0x39, s_2),   
	DSI_CMD_LONG(0x39, s_3),   
	DSI_CMD_LONG(0x39, s_4),   
	DSI_CMD_LONG(0x39, s_5),   
	DSI_CMD_LONG(0x39, s_6),   
	DSI_CMD_LONG(0x39, s_7),   
	DSI_CMD_LONG(0x39, s_8),  
	DSI_CMD_LONG(0x39, s_9),   
	DSI_CMD_LONG(0x39, s_10),   
	DSI_CMD_LONG(0x39, s_11),   
	DSI_CMD_LONG(0x39, s_12),  
	DSI_CMD_LONG(0x39, s_13),   
	DSI_CMD_LONG(0x39, s_14),   
	DSI_CMD_LONG(0x39, s_15),   
	DSI_CMD_LONG(0x39, s_15_1),   
	DSI_CMD_LONG(0x39, s_15_2),   
	DSI_CMD_LONG(0x39, s_15_3),   
	DSI_CMD_LONG(0x39, s_16),   
	DSI_CMD_LONG(0x39, s_16_1),  
	DSI_CMD_LONG(0x39, s_16_2),   
	DSI_CMD_LONG(0x39, s_16_3),   
	DSI_CMD_LONG(0x39, s_17),   
	DSI_CMD_LONG(0x39, s_17_1), 
	DSI_CMD_LONG(0x39, s_17_2),   
	DSI_CMD_LONG(0x39, s_17_3),
	DSI_CMD_LONG(0x39, s_18),   
	DSI_CMD_LONG(0x39, s_18_1), 
	DSI_CMD_LONG(0x39, s_18_2),   
	DSI_CMD_LONG(0x39, s_18_3),
	DSI_CMD_LONG(0x39, s_19),   
	DSI_CMD_LONG(0x39, s_19_1), 
	DSI_CMD_LONG(0x39, s_19_2),   
	DSI_CMD_LONG(0x39, s_19_3),
	DSI_CMD_LONG(0x39, s_20),   
	DSI_CMD_LONG(0x39, s_20_1), 
	DSI_CMD_LONG(0x39, s_20_2),   
	DSI_CMD_LONG(0x39, s_20_3),
#if(1)
        DSI_CMD_SHORT(0x15, 0x35, 0x00),
#endif
	DSI_CMD_SHORT(0x05, 0x11, 0x00),

	DSI_DLY_MS(120),
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(5),
	DSI_DLY_MS(5),
	
};

static struct tegra_dsi_cmd auo_ventana_dis_exit_cmd[] = {
//	DSI_CMD_LONG( 0x39,  s_command3),
//	DSI_CMD_LONG( 0x39,  s_command4),
//	DSI_CMD_LONG( 0x39,  s_command2),
	DSI_CMD_SHORT(0x05, 0x28, 0x00),
	DSI_DLY_MS(40),
//	DSI_CMD_SHORT(0x05, 0x28, 0x00),
//	DSI_DLY_MS(20),
#if(1)
        DSI_CMD_SHORT(0x15, 0x34, 0x00),
#endif
	DSI_CMD_SHORT(0x05, 0x10, 0x00),
	DSI_DLY_MS(120),
};


static struct tegra_dsi_cmd ventana_dsi_init_cmd[]= {
//	DSI_CMD_LONG( 0x39,  s_command3),
//	DSI_CMD_LONG( 0x39,  s_command4),
    //
//	DSI_CMD_LONG( 0x39,  s_command2),
//DSI_CMD_SHORT(0x05, 0x11, 0x00),
//                DSI_DLY_MS(105),
/*DSI_CMD_SHORT(0x05, 0x11, 0x00),
                DSI_DLY_MS(105),
                DSI_CMD_LONG(0x39, init_cmd),
                DSI_CMD_SHORT(0x15, 0xD4, 0x00),
                DSI_CMD_LONG(0x39, eq_cmd),
                DSI_CMD_LONG(0x39, ptbf_cmd),
#if(1)
        DSI_CMD_SHORT(0x15, 0x35, 0x00),
#endif
                DSI_CMD_SHORT(0x05, 0x29, 0x00),
                DSI_DLY_MS(42),*/
	DSI_CMD_LONG(0x39, s_command1),    //lipeng10094834 add  for reduce power at 2012.03.07
	DSI_CMD_LONG(0x39, s_command2),   //lipeng10094834 add  for reduce power at 2012.03.07
	DSI_CMD_SHORT(0x05, 0x11, 0x00),

	DSI_DLY_MS(120),
#if(1)
        DSI_CMD_SHORT(0x15, 0x35, 0x00),
#endif
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(5),
	DSI_DLY_MS(5),
	
};
/*
static struct tegra_dsi_cmd dsi_early_suspend_cmd[] = {
	DSI_CMD_SHORT(0x05, 0x28, 0x00),
	DSI_DLY_MS(20),
};

static struct tegra_dsi_cmd dsi_late_resume_cmd[] = {
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(20),
};
*/
static struct tegra_dsi_cmd ventana_dis_exit_cmd[] = {
//	DSI_CMD_LONG( 0x39,  s_command3),
//	DSI_CMD_LONG( 0x39,  s_command4),
//	DSI_CMD_LONG( 0x39,  s_command2),
	DSI_CMD_SHORT(0x05, 0x28, 0x00),
	DSI_DLY_MS(40),
//	DSI_CMD_SHORT(0x05, 0x28, 0x00),
//	DSI_DLY_MS(20),
#if(1)
        DSI_CMD_SHORT(0x15, 0x34, 0x00),
#endif
	DSI_CMD_SHORT(0x05, 0x10, 0x00),
	DSI_DLY_MS(120),
};



static struct tegra_dc_mode ventana_panel_modes[] = {
	{
      #if 0
		.pclk = 12927000, //
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 4,
		.h_sync_width = 4,
		.v_sync_width = 8,/*3, FIX ME! zte lipeng10094834 modify at 2011.07.04*/
		.h_back_porch = 52,
		.v_back_porch = 12,/*3, FIX ME! zte lipeng10094834 modify at 2011.07.04*/
		.h_active = 540,
		.v_active = 960,
		.h_front_porch = 60,
		.v_front_porch = 12,/*3, FIX ME! zte lipeng10094834 modify at 2011.07.04*/
		#else
        	.pclk = 12927000,
		.h_ref_to_sync = 2,
		.v_ref_to_sync = 1,
		.h_sync_width = 2,
		.v_sync_width = 2,
		.h_back_porch = 50,
		.v_back_porch = 10,

 #if CONFIG_LCD_480_800
 	.h_active		= 480,
	.v_active		= 800,
 #else
		.h_active = 540,
		.v_active = 960,
#endif
		.h_front_porch = 60,
		.v_front_porch = 10,
        #endif

	},
};

static struct tegra_fb_data ventana_fb_data = {
	.win		= 0,
 #if CONFIG_LCD_480_800
 	.xres		= 480,
	.yres		= 800,
 #else
	.xres		= 540,
	.yres		= 960,
#endif
	.bits_per_pixel	= 32,
	 .flags          = TEGRA_FB_FLIP_ON_PROBE,  //Nvidia Martin add at 2011.0823
};

static struct tegra_fb_data ventana_hdmi_fb_data = {
	.win		= 0,
 #if CONFIG_LCD_480_800
 	.xres		= 480,
	.yres		= 800,
 #else
	.xres		= 540,
	.yres		= 960,
#endif
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};
#define DC_CTRL_MODE    TEGRA_DC_OUT_ONE_SHOT_MODE
static struct tegra_dsi_out ventana_dsi_out = {
#if 1
	.n_data_lanes = 2,	
     .pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
     .refresh_rate = 62,
     .virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,	
	.panel_has_frame_buffer = true,

        .panel_reset = 0,
     .power_saving_suspend = true,	
 #if CONFIG_LCD_480_800
      .n_init_cmd = ARRAY_SIZE(hx8369_tm_ventana_dsi_init_cmd),
     .dsi_init_cmd = hx8369_tm_ventana_dsi_init_cmd,	
 #else
     .n_init_cmd = ARRAY_SIZE(ventana_dsi_init_cmd),
     .dsi_init_cmd = ventana_dsi_init_cmd,	
#endif
        .n_early_suspend_cmd = ARRAY_SIZE(ventana_dis_exit_cmd),
        .dsi_early_suspend_cmd = ventana_dis_exit_cmd,
 #if CONFIG_LCD_480_800
         .n_late_resume_cmd = ARRAY_SIZE(hx8369_tm_ventana_dsi_init_cmd),
        .dsi_late_resume_cmd = hx8369_tm_ventana_dsi_init_cmd,
  #else
        .n_late_resume_cmd = ARRAY_SIZE(ventana_dsi_init_cmd),
        .dsi_late_resume_cmd = ventana_dsi_init_cmd,
#endif
     .n_suspend_cmd = ARRAY_SIZE(ventana_dis_exit_cmd),	
     .dsi_suspend_cmd = ventana_dis_exit_cmd,	

     .video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,	
        .video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
	.lp_cmd_mode_freq_khz = 10000,

	/* TODO: Get the vender recommended freq */
 #if CONFIG_LCD_480_800
	.lp_read_cmd_mode_freq_khz = 178500, //230500, //246000,//246000,//230000, //
#else
	.lp_read_cmd_mode_freq_khz = 230500, //246000,//246000,//230000, //
#endif
      //  .hs_cmd_mode_supported = true,
       .phy_timing = {
	.t_hsprepr_ns = 6,
       } 
    #else
	.n_data_lanes = 2,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.refresh_rate =60, //40       //zte lipeng10094834 FIX ME at 2011.07.04
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,
       .panel_has_frame_buffer = true,	
       .dsi_init_cmd = &ventana_dsi_init_cmd,		/* init cmd*/
	.n_init_cmd = ARRAY_SIZE(ventana_dsi_init_cmd),	
	.dsi_suspend_cmd = &ventana_dis_exit_cmd,   /*zte lipeng10094834 add for disable panel*/
	.n_suspend_cmd = ARRAY_SIZE(ventana_dis_exit_cmd),	 /*zte lipeng10094834 add for disable panel*/
      .video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,//TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,//		//zte lipeng10094834 FIX ME at 2011.07.04
       .video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
      .lp_cmd_mode_freq_khz = 5000,
      	.dsi_instance = 0,
      	#endif
    //   .hs_cmd_mode_supported = true,
/*  #if 1
      .phy_timing = {
       .t_hsdexit_ns = 4,
	.t_hstrail_ns = 6,
	.t_hsprepr_ns = 2,
	.t_datzero_ns = 3,
	.t_clktrail_ns = 3,
	.t_clkpost_ns = 9,
	.t_clkzero_ns = 7,
	.t_tlpx_ns = 2,
       }
  #else
        .phy_timing = {
        .t_hsdexit_ns = 6,
	.t_hstrail_ns = 8,
	.t_hsprepr_ns = 3,
	.t_datzero_ns = 4,
	.t_clktrail_ns = 4,
	.t_clkpost_ns = 11,
	.t_clkzero_ns = 10,
	.t_tlpx_ns = 3,
       }
  #endif*/
};
static struct tegra_dc_out ventana_disp1_out = {
	.type		= TEGRA_DC_OUT_DSI,     //nvidia danny modify at 2011.07.29

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	//.depth		= 18,                      //nvidia danny modify at 2011.07.29
	//.dither		= TEGRA_DC_ORDERED_DITHER,  //nvidia danny modify at 2011.07.29
	  .flags		= 0, //TEGRA_DC_OUT_ONE_SHOT_MODE, 
	.modes	 	= ventana_panel_modes,
	.n_modes 	= ARRAY_SIZE(ventana_panel_modes),
	.dsi		= &ventana_dsi_out,     //zte lipeng10094834 modify end at 2011.07.04
	.enable		= ventana_panel_enable,
	.disable	= ventana_panel_disable,

};

static struct tegra_dc_out ventana_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 1,
	.hotplug_gpio	= ventana_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= ventana_hdmi_enable,
	.disable	= ventana_hdmi_disable,
};

static struct tegra_dc_platform_data ventana_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &ventana_disp1_out,
	.fb		= &ventana_fb_data,
       .emc_clk_rate	= 150000000,
};

static struct tegra_dc_platform_data ventana_disp2_pdata = {
	.flags		= 0,
	.default_out	= &ventana_disp2_out,
	.fb		= &ventana_hdmi_fb_data,
       .emc_clk_rate	= 150000000,
};

static struct nvhost_device ventana_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= ventana_disp1_resources,
	.num_resources	= ARRAY_SIZE(ventana_disp1_resources),
	.dev = {
		.platform_data = &ventana_disp1_pdata,
	},
};

static int ventana_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &ventana_disp1_device.dev;
}

static struct nvhost_device ventana_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= ventana_disp2_resources,
	.num_resources	= ARRAY_SIZE(ventana_disp2_resources),
	.dev = {
		.platform_data = &ventana_disp2_pdata,
	},
};
#else
static int ventana_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return 0;
}
#endif

static struct nvmap_platform_carveout ventana_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data ventana_nvmap_data = {
	.carveouts	= ventana_carveouts,
	.nr_carveouts	= ARRAY_SIZE(ventana_carveouts),
};

static struct platform_device ventana_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &ventana_nvmap_data,
	},
};

static struct platform_device *ventana_gfx_devices[] __initdata = {
	&ventana_nvmap_device,
#ifdef CONFIG_TEGRA_GRHOST
	&tegra_grhost_device,
#endif
	&tegra_pwfm2_device,
	&ventana_backlight_device,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend ventana_panel_early_suspender;

static void ventana_panel_early_suspend(struct early_suspend *h)
{
	////unsigned i;
	 printk("[%s] SuspendFlag %d\n", __FUNCTION__, s_SuspendFlag);

	s_SuspendFlag = 1;
	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_NORMAL);
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_save_default_governor();
	cpufreq_set_conservative_governor();
	cpufreq_set_conservative_governor_param(
		SET_CONSERVATIVE_GOVERNOR_UP_THRESHOLD,
		SET_CONSERVATIVE_GOVERNOR_DOWN_THRESHOLD);
#endif
}

static void ventana_panel_late_resume(struct early_suspend *h)
{
	unsigned i;



#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_restore_default_governor();
#endif
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
	 printk("[%s] SuspendFlag %d\n", __FUNCTION__, s_SuspendFlag);
}
#endif
int ventana_panel_set_lcd_type(int panelid)
{
    int  ret;
    int lcdid;
   printk("+++++++++++++++++++++++++++ventana_panel_set_lcd_type\n");
    ret = gpio_request(ventana_lcd_id, "ventana_lcdid");
    if (ret < 0)
    {
	pr_err(" gpio_request pn6 error %d\n", ret);
        return ret;
        }

   
    ret = gpio_direction_input(ventana_lcd_id);
    if (ret < 0) {
        pr_err("gpio_direction_output pn6 error %d\n", ret);
            gpio_free(ventana_lcd_id);
            return ret;
    }   
    tegra_gpio_enable(ventana_lcd_id);  
    lcdid = gpio_get_value(ventana_lcd_id);
    if(lcdid == 0)
    {
        printk("+++++++++++++++++++++++++++ventana_panel_set_lcd_type auo\n");
        if(panelid == 0x0)
        {
            ventana_dsi_out.n_init_cmd = ARRAY_SIZE(auo_ventana_dsi_init_cmd);
            ventana_dsi_out.dsi_init_cmd = auo_ventana_dsi_init_cmd;	
            ventana_dsi_out.n_late_resume_cmd = ARRAY_SIZE(auo_ventana_dsi_init_cmd);
            ventana_dsi_out.dsi_late_resume_cmd = auo_ventana_dsi_init_cmd;
            g_lcdid = 0;
         }
    }
    else
    {

        if(panelid == 0x88)
        {
            printk("+++++++++++++++++++++++++++ventana_panel_set_lcd_type lg\n");
            ventana_dsi_out.n_init_cmd = ARRAY_SIZE(lg_ventana_dsi_init_cmd);
            ventana_dsi_out.dsi_init_cmd = lg_ventana_dsi_init_cmd;	
            ventana_dsi_out.n_late_resume_cmd = ARRAY_SIZE(lg_ventana_dsi_init_cmd);
            ventana_dsi_out.dsi_late_resume_cmd = lg_ventana_dsi_init_cmd;       
            g_lcdid = 1;
        }
        else if(panelid == 0x40)
        {
             printk("+++++++++++++++++++++++++++ventana_panel_set_lcd_type otm\n");
            ventana_dsi_out.n_init_cmd = ARRAY_SIZE(otm_9608_ventana_dsi_init_cmd);
            ventana_dsi_out.dsi_init_cmd = otm_9608_ventana_dsi_init_cmd;	
            ventana_dsi_out.n_late_resume_cmd = ARRAY_SIZE(otm_9608_ventana_dsi_init_cmd);
            ventana_dsi_out.dsi_late_resume_cmd = otm_9608_ventana_dsi_init_cmd;     
            g_lcdid = 1;
        }
        else
        {
            printk("+++++++++++++++++++++++++++ventana_panel_set_lcd_type sharp\n");
            g_lcdid = 2;
        }
    }    
      return 0;
}
EXPORT_SYMBOL(ventana_panel_set_lcd_type);

int ventana_panel_set_lcd_type_for_hx8369(int panelid)
{
    int  ret;
    int lcdid;
   printk("+++++++++++++++++++++++++++ventana_panel_set_lcd_type_for_hx8369\n");
    ret = gpio_request(ventana_lcd_id, "ventana_lcdid");
    if (ret < 0)
    {
	pr_err(" gpio_request pn6 error %d\n", ret);
        return ret;
        }

   
    ret = gpio_direction_input(ventana_lcd_id);
    if (ret < 0) {
        pr_err("gpio_direction_output pn6 error %d\n", ret);
            gpio_free(ventana_lcd_id);
            return ret;
    }   
    tegra_gpio_enable(ventana_lcd_id);  
    lcdid = gpio_get_value(ventana_lcd_id);
   printk("+++++++++++++++++++++++++++ventana_panel_set_lcd_type_for_hx8369 lcdid %d\n", lcdid);
  if(panelid == 0x88)
    {
        printk("+++++++++++++++++++++++++++ventana_panel_set_lcd_type_for_hx8369 lead\n");
        ventana_dsi_out.n_init_cmd = ARRAY_SIZE(hx8369_ventana_dsi_init_cmd);
        ventana_dsi_out.dsi_init_cmd = hx8369_ventana_dsi_init_cmd;	
        ventana_dsi_out.n_late_resume_cmd = ARRAY_SIZE(hx8369_ventana_dsi_init_cmd);
        ventana_dsi_out.dsi_late_resume_cmd = hx8369_ventana_dsi_init_cmd;    
    }
  else
    {
        printk("+++++++++++++++++++++++++++ventana_panel_set_lcd_type_for_hx8369 tianma\n");
    }
  return 0;
}
EXPORT_SYMBOL(ventana_panel_set_lcd_type_for_hx8369);

int __init ventana_panel_init(void)
{
	int err, ret;
	struct resource *res;
//nvidia danny delete at 2011.07.29
/*
	gpio_request(ventana_pnl_pwr_enb, "pnl_pwr_enb");
	gpio_direction_output(ventana_pnl_pwr_enb, 1);
	tegra_gpio_enable(ventana_pnl_pwr_enb);

	gpio_request(ventana_lvds_shutdown, "lvds_shdn");
	gpio_direction_output(ventana_lvds_shutdown, 1);
	tegra_gpio_enable(ventana_lvds_shutdown);
*/
     printk("+++++++++++++++++++++++++++ventana_panel_init\n");
#if CONFIG_LCD_480_800
     printk("+++++++++++++++++++++++++++ventana_panel_init U880F\n");
ret = gpio_request(ventana_lcd_1v8_enable, "ventana_lcd_1v8_enable");
    if (ret < 0)
    {
	 pr_err("gpio_request pe4 error %d\n", ret);
        return ret;
        }

    ret = gpio_direction_output(ventana_lcd_1v8_enable, 1);
    if (ret < 0) {
        pr_err("gpio_direction_output pe4 error %d\n",  ret);
            gpio_free(ventana_lcd_1v8_enable);
            return ret;
    }
   tegra_gpio_enable(ventana_lcd_1v8_enable);  
#endif


  ret = gpio_request(ventana_lcd_te, "ventana_te");
    if (ret < 0)
    {
	pr_err(" gpio_request pn6 error %d\n", ret);
        return ret;
        }

   
    ret = gpio_direction_input(ventana_lcd_te);
    if (ret < 0) {
        pr_err("gpio_direction_output pn6 error %d\n", ret);
            gpio_free(ventana_lcd_reset);
            return ret;
    }   
    tegra_gpio_enable(ventana_lcd_te);   



     	tegra_gpio_enable(ventana_hdmi_enb);
	gpio_request(ventana_hdmi_enb, "hdmi_5v_en");
	gpio_direction_output(ventana_hdmi_enb, 1);

	tegra_gpio_enable(ventana_hdmi_hpd);
	gpio_request(ventana_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(ventana_hdmi_hpd);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ventana_panel_early_suspender.suspend = ventana_panel_early_suspend;
	ventana_panel_early_suspender.resume = ventana_panel_late_resume;
	ventana_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&ventana_panel_early_suspender);
#endif

	ventana_carveouts[1].base = tegra_carveout_start;
	ventana_carveouts[1].size = tegra_carveout_size;

	err = platform_add_devices(ventana_gfx_devices,
				   ARRAY_SIZE(ventana_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&ventana_disp1_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	res = nvhost_get_resource_byname(&ventana_disp2_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
		min(tegra_fb_size, tegra_bootloader_fb_size));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	if (!err)
		err = nvhost_device_register(&ventana_disp1_device);

	if (!err)
		err = nvhost_device_register(&ventana_disp2_device);
#endif

	return err;
}

#ifdef CONFIG_BACKLIGHT_1_WIRE_MODE
/*ZTE: added by tong.weili for 1-wire backlight 20111122 ++*/
#define TPS61165_DEVICE_ADDR  (0x72)
#define BIT_DELAY_UNIT (50)
#define LOGIC_FACTOR (3)
#define CONDITION_DELAY (50)
#define ES_DETECT_DELAY  (200) 
#define ES_DETECT_TIME  (300) 
#define ES_TIMING_WINDOW  (1000)

#define TPS61165_DELAY(n) udelay(n)

static int tps61165_write_bit(u8 b)
{
    if(1 == b)
    {
        gpio_set_value(TPS61165_CTRL_PIN, 0);
        TPS61165_DELAY(BIT_DELAY_UNIT);
        gpio_set_value(TPS61165_CTRL_PIN, 1);
        TPS61165_DELAY(LOGIC_FACTOR*BIT_DELAY_UNIT);
        //gpio_set_value(TPS61165_CTRL_PIN, 0);
    }
    else if(0 == b)
    {
        gpio_set_value(TPS61165_CTRL_PIN, 0);
        TPS61165_DELAY(LOGIC_FACTOR*BIT_DELAY_UNIT);
        gpio_set_value(TPS61165_CTRL_PIN, 1);
        TPS61165_DELAY(BIT_DELAY_UNIT);
        //gpio_set_value(TPS61165_CTRL_PIN, 0);
    }
    else
    {
        printk("[tong]:tps61165_write_bit: error param!\n");
        return -1;
    }
    
    return 0;
}

static int tps61165_write_byte(u8 bytedata)
{
    u8 bit_cnt = 8;
    u8 val = bytedata;
    int ret = 0;
    unsigned long flags;
    
    //spin_lock(&s_tps61165_lock); /*ZTE: added by tong.weili 解决待机唤醒时偶发背光设置出错 20111128*/
    spin_lock_irqsave(&s_tps61165_lock, flags);/*ZTE: modified by tong.weili 增加背光设置稳定性 20120310*/
    gpio_set_value(TPS61165_CTRL_PIN, 1);
    TPS61165_DELAY(CONDITION_DELAY); //Start condition,  at least 2us

    bit_cnt = 8;
		
    while(bit_cnt)
    {
        bit_cnt--;
        if((val >> bit_cnt) & 1)
        {          
            ret = tps61165_write_bit(1);
        }
        else
        {
            ret = tps61165_write_bit(0);
        } 

        if(ret)
        {
            printk("[tong]:tps61165_write_byte:failed!\n");
            //spin_unlock(&s_tps61165_lock); /*ZTE: added by tong.weili 解决待机唤醒时偶发背光设置出错 20111128*/
            spin_unlock_irqrestore(&s_tps61165_lock, flags);
            return ret;
        }     
    }

    gpio_set_value(TPS61165_CTRL_PIN, 0);
    TPS61165_DELAY(CONDITION_DELAY); //EOS condition, at least 2us
    gpio_set_value(TPS61165_CTRL_PIN, 1);
    
    //spin_unlock(&s_tps61165_lock); /*ZTE: added by tong.weili 解决待机唤醒时偶发背光设置出错 20111128*/
    spin_unlock_irqrestore(&s_tps61165_lock, flags);
    
    return 0;
}

static int tps61165_config_ES_timing(void)
{
    unsigned long flags;//tong test
    
    printk("[tong]:tps61165_config_ES_timing\n");
    
    //spin_lock(&s_tps61165_lock); /*ZTE: added by tong.weili 解决待机唤醒时偶发背光设置出错 20111128*/
    spin_lock_irqsave(&s_tps61165_lock, flags);/*ZTE: modified by tong.weili 增加背光设置稳定性 20120310*/
    
    gpio_set_value(TPS61165_CTRL_PIN, 0);
    TPS61165_DELAY(CONDITION_DELAY);
    
    gpio_set_value(TPS61165_CTRL_PIN, 1);  //start ES Timing Window
    TPS61165_DELAY(ES_DETECT_DELAY); //at least 100us

    gpio_set_value(TPS61165_CTRL_PIN, 0);
    TPS61165_DELAY(ES_DETECT_TIME); //at least 260us

    gpio_set_value(TPS61165_CTRL_PIN, 1);
    TPS61165_DELAY(ES_TIMING_WINDOW - ES_DETECT_DELAY - ES_DETECT_TIME);

    mdelay(1);
    
    //spin_unlock(&s_tps61165_lock); /*ZTE: added by tong.weili 解决待机唤醒时偶发背光设置出错 20111128*/
    spin_unlock_irqrestore(&s_tps61165_lock, flags);
    
    return 0;
}

static int tps61165_shutdown(void)
{
    printk("[tong]:tps61165_shutdown\n");
    
    gpio_set_value(TPS61165_CTRL_PIN, 0);
    mdelay(4); //enter shutdown mode, at least 2.5ms
    return 0;
}

static int tps61165_init(void)
{
    tps61165_shutdown();
    tps61165_config_ES_timing();
    return 0;
}

int tps61165_set_backlight(int brightness)
{
    u8 tps61165_level;
    static u8 old_level = -1;
 
    /*ZTE: added by tong.weili 开机刚进入kernel 不设默认背光，因为已在bootloader中设置过 20111130 ++*/
    if(DEFAULT_BRIGHTNESS == brightness)
    {
        if(!s_bNeedSetBacklight)
        {
            s_bNeedSetBacklight = true;
            return 0;
        }      
    }
    /*ZTE: added by tong.weili 开机刚进入kernel 不设默认背光，因为已在bootloader中设置过 20111130 --*/
    
    tps61165_level = (brightness & 0xFF) >> 3;/*convert level 0~255  to  0~31*/

    if(old_level == tps61165_level)
    {
        //printk("[tong]:tps61165_set_backlight: the same level as before, nothing done!level=%d\n", tps61165_level);
        return 0;
    }

    
    /*ZTE: modified by tong.weili for backlight print 20120202 ++*/
    if(debug)
    {
        printk("[tong]:tps61165_set_backlight: brightness=%d, tps61165_level=%d\n", brightness, tps61165_level);
    }
    /*ZTE: modified by tong.weili for backlight print 20120202 --*/

    /*ZTE: added by tong.weili 优化背光处理流程 20111125 ++*/
    if(tps61165_level)
    {
        if(!s_tps61165_is_inited)
        {
            tps61165_init();
            s_tps61165_is_inited = true;
        }      
    }
    else
    {
        tps61165_shutdown();
        old_level = tps61165_level;
        s_tps61165_is_inited = false;
        return 0;
    }
    /*ZTE: added by tong.weili 优化背光处理流程 20111125 --*/

    mutex_lock(&s_tps61165_mutex); /*ZTE: added by tong.weili 增加背光设置稳定性 20111128*/
    
    tps61165_write_byte(TPS61165_DEVICE_ADDR);
    tps61165_write_byte(tps61165_level);

    mutex_unlock(&s_tps61165_mutex); /*ZTE: added by tong.weili 增加背光设置稳定性 20111128*/

    old_level = tps61165_level;
    return 0;    
}
/*ZTE: added by tong.weili for 1-wire backlight 20111122 --*/
#endif
