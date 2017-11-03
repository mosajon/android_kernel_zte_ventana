/*
 * arch/arm/mach-tegra/board-ventana.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c/panjit_ts.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#ifdef CONFIG_ZTE_CMMB
#include <linux/spi/spi.h>  
#endif
#include <linux/mfd/tps6586x.h>
#include <linux/memblock.h>
//#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/tegra_uart.h>
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI
#include <linux/synaptics_i2c_rmi.h> 
#endif
#ifdef CONFIG_TOUCHSCREEN_FT5X06
#include <linux/input/ft5x06_ts.h>    /*ZTE: modified by tong.weili for Ft5X06 Touch 20120411 */
#endif
#include <linux/regulator/consumer.h>  /*ZTE: modified by tong.weili for Touch Power 20120411 */
#include <sound/wm8903.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#if defined(CONFIG_SND_SOC_WM8903)
#include <mach/tegra_wm8903_pdata.h>
#elif defined (CONFIG_SND_SOC_TLV320AIC326X)
#include <mach/tegra_aic326x_pdata.h>
#include <linux/spi-tegra.h>
#else
#error
#endif
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <sound/tlv320aic326x.h>
#include "board.h"
#include "clock.h"
#include "board-ventana.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "wakeups-t2.h"
#include "pm.h"
#include "zte_board_ext_func.h" // ZTE：added by pengtao

/*ZTE: modified by tong.weili for Touch Power 20120411 ++*/
#define TOUCH_POWER_2V8_EN    TEGRA_GPIO_PR3
#define TOUCH_POWER_1V8_EN    TEGRA_GPIO_PR4
static struct regulator *ventana_LDO3 = NULL;
/*ZTE: modified by tong.weili for Touch Power 20120411 --*/

static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[1] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
};

static struct tegra_ulpi_config ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PG2,
	.clk = "cdev2",
};

static struct resource ventana_bcm4329_rfkill_resources[] = {
	{
		.name   = "bcm4329_nshutdown_gpio",
		.start  = TEGRA_GPIO_PU0,
		.end    = TEGRA_GPIO_PU0,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device ventana_bcm4329_rfkill_device = {
	.name = "bcm4329_rfkill",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ventana_bcm4329_rfkill_resources),
	.resource       = ventana_bcm4329_rfkill_resources,
};

static void __init ventana_bt_rfkill(void)
{
	/*Add Clock Resource*/
	clk_add_alias("bcm4329_32k_clk", ventana_bcm4329_rfkill_device.name, \
				"blink", NULL);
	return;
}

static struct resource ventana_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PV3,
			.end    = TEGRA_GPIO_PV3,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = TEGRA_GPIO_PU1,
			.end    = TEGRA_GPIO_PU1,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV3),
			.end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV3),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device ventana_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ventana_bluesleep_resources),
	.resource       = ventana_bluesleep_resources,
};

static void __init ventana_setup_bluesleep(void)
{
	platform_device_register(&ventana_bluesleep_device);
	tegra_gpio_enable(TEGRA_GPIO_PV3);
	tegra_gpio_enable(TEGRA_GPIO_PU1);
	return;
}

static __initdata struct tegra_clk_init_table ventana_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "blink",	"clk_32k",	32768,		false},
	{ "pll_p_out4",	"pll_p",	24000000,	true },
	{ "pwm",	"clk_32k",	32768,		false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s2",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_ulpi_config ventana_ehci2_ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PV1,
	.clk = "cdev2",
};

static struct tegra_ehci_platform_data ventana_ehci2_ulpi_platform_data = {
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 1,
	.phy_config = &ventana_ehci2_ulpi_phy_config,
	.phy_type = TEGRA_USB_PHY_TYPE_LINK_ULPI,
};

static struct tegra_i2c_platform_data ventana_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.slave_addr = 0x00FC,
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup	= TEGRA_PINGROUP_PTA,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data ventana_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	//.bus_clk_rate	= { 100000, 10000 },
	.bus_clk_rate	= { 100000, 400000 },/*ZTE: modified by tong.weili 解决触摸上报率低 20120515*/
	.bus_mux	= { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len	= { 1, 1 },
	.slave_addr = 0x00FC,
	.scl_gpio		= {0, TEGRA_GPIO_PT5},
	.sda_gpio		= {0, TEGRA_GPIO_PT6},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data ventana_i2c3_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.slave_addr = 0x00FC,
	.scl_gpio		= {TEGRA_GPIO_PBB2, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB3, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data ventana_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_dvc		= true,
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};
#if defined(CONFIG_SND_SOC_WM8903) 
static struct wm8903_platform_data ventana_wm8903_pdata = {
	.irq_active_low = 0,
	.micdet_cfg = 0,
	.micdet_delay = 100,
	.gpio_base = VENTANA_GPIO_WM8903(0),
	.gpio_cfg = {
		(WM8903_GPn_FN_DMIC_LR_CLK_OUTPUT << WM8903_GP1_FN_SHIFT),
		(WM8903_GPn_FN_DMIC_LR_CLK_OUTPUT << WM8903_GP2_FN_SHIFT) |
			WM8903_GP2_DIR,
		0,
		WM8903_GPIO_NO_CONFIG,
		WM8903_GPIO_NO_CONFIG,
	},
};

static struct i2c_board_info __initdata wm8903_board_info = {
	I2C_BOARD_INFO("wm8903", 0x1a),
	.platform_data = &ventana_wm8903_pdata,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ),
};
#endif
#if defined(CONFIG_SND_SOC_TLV320AIC326X)
static struct aic326x_pdata ventana_aic3262_pdata = {
            .first_version_reset_pin = CODEC_FIRST_VERSION_RESET_PIN,
	     .second_version_reset_pin = CODEC_SECOND_VERSION_RESET_PIN,
};
static struct i2c_board_info __initdata aic3262_board_info = {
	I2C_BOARD_INFO("tlv320aic3262", 0x18),
	.platform_data = &ventana_aic3262_pdata,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ),
};

static struct spi_clk_parent spi_codec_parent_clk[] = {
        [0] = {.name = "pll_p"},        
};

static struct tegra_spi_platform_data spi_codec_pdata = {
        .is_dma_based           = true,
	.is_dynamic_dma		= false,
        .max_dma_buffer         = (16 * 1024),
        .is_clkon_always        = true,
        .max_rate               = 5000000,
};

static struct tegra_spi_device_controller_data codec_spi_control_info = {
	.is_hw_based_cs = true,
	.cs_setup_clk_count = 2,
	.cs_hold_clk_count = 2,
};

static struct spi_board_info codec_spi_board_info[] = {
	/* spi master */
	{
		.modalias = "tlv320aic3262",
		.bus_num = 2,
		.chip_select = 2,
		.mode = SPI_MODE_1,
		.max_speed_hz = 12000000,
		.platform_data = &ventana_aic3262_pdata,
		.controller_data = &codec_spi_control_info,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ),
	},
};
#endif
static void ventana_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &ventana_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &ventana_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &ventana_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &ventana_dvc_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device4);

	#if defined(CONFIG_SND_SOC_WM8903) 
	i2c_register_board_info(0, &wm8903_board_info, 1);
	#endif
	#if defined(CONFIG_SND_SOC_TLV320AIC326X)
	i2c_register_board_info(0, &aic3262_board_info, 1);
	#endif
}
static struct platform_device *ventana_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
};

static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "pll_p"},
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
};

static struct tegra_uart_platform_data ventana_uart_pdata;
static struct tegra_uart_platform_data modem_uart_pdata;
static void wake_modem(struct uart_port *port)
{
    int value = 1;
    int retry;
    
    //printk(KERN_DEBUG "%s: wakeup modem start\n", dev_name(port->dev));
    gpio_set_value(AP_TO_MODEM_WKUP, 1);
    gpio_set_value(AP_TO_MODEM_WKUP, 0);

    //totally wait for 100ms
    retry = 10000;
    while(retry){
        value = gpio_get_value(MODEM_TO_AP_SLP);
        if (0 > value) {
            printk(KERN_ERR "%s: failed to get modem sleep status %d\n", 
                    dev_name(port->dev), value);
            return;
        }
        
        if (0 == value){
            //printk(KERN_DEBUG "%s: wakeup modem succ, retry left %d\n", dev_name(port->dev), retry);
            break;
        }
        udelay(10);
        retry--;
    }
    
    gpio_set_value(AP_TO_MODEM_WKUP, 1);

    if (0 == retry && 1 == value)
        printk(KERN_ERR "%s: wakeup modem failed\n", dev_name(port->dev));  
}

static void modem_sleep_control(struct uart_port *port, int val, int step)
{
    if (1 == val)
    {
    	if (step == 0) {
    		pr_info("%s: set ap sleep state 1\n", __func__);
        	gpio_set_value(AP_TO_MODEM_SLP, 1);
	    } else if (step == 1) {
    		pr_info("%s: set uart port switch 0\n", __func__);
        	//gpio_set_value(p903u_uart_port_switch, 0);
    	}
    }
    else if (0 == val)
    {
    	if (step == 0) {
			pr_info("%s: set uart port switch 1\n", __func__);
        	//gpio_set_value(p903u_uart_port_switch, 1);
	    } else if (step == 1) {
    		pr_info("%s: set ap sleep state 0\n", __func__);
        	gpio_set_value(AP_TO_MODEM_SLP, 0);
    	}
    }
    return;
}

static void __init uart_debug_init(void)
{
	unsigned long rate;
	struct clk *c;

	/* UARTD is the debug port. */
	pr_info("Selecting UARTD as the debug console\n");
	ventana_uart_devices[3] = &debug_uartd_device;
	debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
	debug_uart_clk = clk_get_sys("serial8250.0", "uartd");

	/* Clock enable for the debug channel */
	if (!IS_ERR_OR_NULL(debug_uart_clk)) {
		rate = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->uartclk;
		pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
		c = tegra_get_clock_by_name("pll_p");
		if (IS_ERR_OR_NULL(c))
			pr_err("Not getting the parent clock pll_p\n");
		else
			clk_set_parent(debug_uart_clk, c);

		clk_enable(debug_uart_clk);
		clk_set_rate(debug_uart_clk, rate);
	} else {
		pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
	}
}

static void __init ventana_uart_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	ventana_uart_pdata.parent_clk_list = uart_parent_clk;
	ventana_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);

	modem_uart_pdata = ventana_uart_pdata;
	modem_uart_pdata.wake_peer = wake_modem;
	modem_uart_pdata.sleep_ctrl = modem_sleep_control;

	tegra_uarta_device.dev.platform_data = &modem_uart_pdata;

	tegra_uartb_device.dev.platform_data = &ventana_uart_pdata;
	tegra_uartc_device.dev.platform_data = &ventana_uart_pdata;
	tegra_uartd_device.dev.platform_data = &ventana_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(ventana_uart_devices,
				ARRAY_SIZE(ventana_uart_devices));
}

#ifdef CONFIG_KEYBOARD_GPIO
#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button ventana_keys[] = {
/*
	[0] = GPIO_KEY(KEY_FIND, PQ3, 0),
	[1] = GPIO_KEY(KEY_HOME, PQ1, 0),
	[2] = GPIO_KEY(KEY_BACK, PQ2, 0),
	[3] = GPIO_KEY(KEY_VOLUMEUP, PQ5, 0),
	[4] = GPIO_KEY(KEY_VOLUMEDOWN, PQ4, 0),
	[5] = GPIO_KEY(KEY_POWER, PV2, 1),
	[6] = GPIO_KEY(KEY_MENU, PC7, 0),*/
	[0]= GPIO_KEY(KEY_VOLUMEUP, PI1, 1),
	[1]= GPIO_KEY(KEY_VOLUMEDOWN, PR0, 1),
	[2] = GPIO_KEY(KEY_POWER, PV2, 1),
};

#define PMC_WAKE_STATUS 0x14

static int ventana_wakeup_key(void)
{
	unsigned long status =
		readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);

	return status & TEGRA_WAKE_GPIO_PV2 ? KEY_POWER : KEY_RESERVED;
}

static struct gpio_keys_platform_data ventana_keys_platform_data = {
	.buttons	= ventana_keys,
	.nbuttons	= ARRAY_SIZE(ventana_keys),
	.wakeup_key	= ventana_wakeup_key,
};

static struct platform_device ventana_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev	= {
		.platform_data	= &ventana_keys_platform_data,
	},
};

static void ventana_keys_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ventana_keys); i++)
		tegra_gpio_enable(ventana_keys[i].gpio);
}
#endif

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct platform_device modem_control_device = {
	.name = "modem_control",
	.id = -1,
};
#if defined(CONFIG_SND_SOC_WM8903)
static struct tegra_wm8903_platform_data ventana_audio_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= TEGRA_GPIO_INT_MIC_EN,
	.gpio_ext_mic_en	= TEGRA_GPIO_EXT_MIC_EN,
};
#elif defined (CONFIG_SND_SOC_TLV320AIC326X)
static struct tegra_aic326x_platform_data ventana_audio_pdata = {
	.gpio_spkr_en		= -1,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1,
	.gpio_ext_mic_en	= -1,
	.gpio_cdc_irq       = TEGRA_GPIO_CDC_IRQ,
};
#else
#error
#endif

static struct platform_device ventana_audio_device = {
#if defined(CONFIG_SND_SOC_WM8903) 
	.name	= "tegra-snd-wm8903",
#elif defined (CONFIG_SND_SOC_TLV320AIC326X)
     .name	= "tegra-snd-aic326x",
#else
#error
#endif
	.id	= 0,
	.dev	= {
		.platform_data  = &ventana_audio_pdata,
	},
};

static struct platform_device *ventana_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_gart_device,
	&tegra_aes_device,
#ifdef CONFIG_KEYBOARD_GPIO
	&ventana_keys_device,
#endif
	&tegra_wdt_device,
	&tegra_avp_device,
	&tegra_camera,
	&tegra_i2s_device1,
	&tegra_i2s_device2,
	&tegra_spdif_device,
	&tegra_das_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&ventana_bcm4329_rfkill_device,
	&tegra_pcm_device,
	&ventana_audio_device,
	&modem_control_device,
};

/*ZTE: added by tong.weili for touch 20111215 ++*/
#if 0
static struct mxt_platform_data atmel_mxt_info = {
	.x_line		= 27,
	.y_line		= 42,
	.x_size		= 768,
	.y_size		= 1366,
	.blen		= 0x20,
	.threshold	= 0x3C,
	.voltage	= 3300000,
	.orient		= MXT_ROTATED_90,
	.irqflags	= IRQF_TRIGGER_FALLING,
};

static struct i2c_board_info __initdata i2c_info[] = {
	{
	 I2C_BOARD_INFO("atmel_mxt_ts", 0x5A),
	 .irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6),
	 .platform_data = &atmel_mxt_info,
	 },
};

static int __init ventana_touch_init_atmel(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PV6);
	tegra_gpio_enable(TEGRA_GPIO_PQ7);

	gpio_request(TEGRA_GPIO_PV6, "atmel-irq");
	gpio_direction_input(TEGRA_GPIO_PV6);

	gpio_request(TEGRA_GPIO_PQ7, "atmel-reset");
	gpio_direction_output(TEGRA_GPIO_PQ7, 0);
	msleep(1);
	gpio_set_value(TEGRA_GPIO_PQ7, 1);
	msleep(100);

	i2c_register_board_info(0, i2c_info, 1);

	return 0;
}

static struct panjit_i2c_ts_platform_data panjit_data = {
	.gpio_reset = TEGRA_GPIO_PQ7,
};

static struct i2c_board_info __initdata ventana_i2c_bus1_touch_info[] = {
	{
		I2C_BOARD_INFO("panjit_touch", 0x3),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6),
		.platform_data = &panjit_data,
	},
};

static int __init ventana_touch_init_panjit(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PV6);

	tegra_gpio_enable(TEGRA_GPIO_PQ7);
	i2c_register_board_info(0, ventana_i2c_bus1_touch_info, 1);

	return 0;
}
#else

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI
static struct i2c_board_info synaptics_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("synaptics-rmi-ts", 0x22),
		//.platform_data = &synaptics_i2c_data,
		.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PC6),		
	},
};
#endif

/*ZTE: modified by tong.weili for Ft5X06 Touch 20120411 ++*/
#ifdef CONFIG_TOUCHSCREEN_FT5X06
int  ft_touch_wake(void)
{
        printk("[FTS]ft_touch_wake enter \n");
        gpio_direction_output(TEGRA_GPIO_PC6, 1); 
        msleep(3);
        gpio_set_value(TEGRA_GPIO_PC6, 0);
        msleep(3);
        gpio_set_value(TEGRA_GPIO_PC6, 1);
        msleep(3);
        gpio_direction_input(TEGRA_GPIO_PC6); 
        printk("[FTS]ft_touch_wake exit \n");
        return 0;
}
EXPORT_SYMBOL(ft_touch_wake);

int  ft_touch_reset(void)
{
        printk("[FTS]ft_touch_reset enter \n");
        gpio_set_value(TEGRA_GPIO_PW0, 1);
        msleep(3);
        gpio_set_value(TEGRA_GPIO_PW0, 0);
        msleep(3);
        gpio_set_value(TEGRA_GPIO_PW0, 1);
        msleep(250);  
        printk("[FTS]ft_touch_reset exit \n");
        return 0;
}
EXPORT_SYMBOL(ft_touch_reset);
static int touch_hw_init(void)
{
      gpio_request(TEGRA_GPIO_PC6, "touch-irq");	  
      gpio_direction_input(TEGRA_GPIO_PC6); 
      tegra_gpio_enable(TEGRA_GPIO_PC6);    
      
      if (!ventana_LDO3) 
      {
          ventana_LDO3 = regulator_get(NULL, "vdd_ldo3"); /* LD03 */
          if (IS_ERR_OR_NULL(ventana_LDO3)) 
          {
              pr_err("touch_hw_init: couldn't get regulator vdd_ldo3\n");
              ventana_LDO3 = NULL;
              return -1;
          }
          regulator_enable(ventana_LDO3); 
      }

      gpio_request(TOUCH_POWER_2V8_EN, "touch_power_2v8_enb");
      gpio_direction_output(TOUCH_POWER_2V8_EN, 1);
      tegra_gpio_enable(TOUCH_POWER_2V8_EN);

      gpio_request(TOUCH_POWER_1V8_EN, "touch_power_1v8_enb");
      gpio_direction_output(TOUCH_POWER_1V8_EN, 1);
      tegra_gpio_enable(TOUCH_POWER_1V8_EN);

      gpio_request(TEGRA_GPIO_PW0, "touch-reset");	  
      gpio_direction_output(TEGRA_GPIO_PW0, 1);
      tegra_gpio_enable(TEGRA_GPIO_PW0);

      return 0;
}

static struct Ft5x06_ts_platform_data ft5x06_i2c_data = {
              .maxx = 319,      
              .maxy = 479,       
              .model = 2010,
              .x_plate_ohms = 300,
              .irq_flags    = IRQF_TRIGGER_LOW,
              .init_platform_hw = touch_hw_init,
};

static struct i2c_board_info ft5x06_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("ft5x0x_ts", 0x3E),
             //I2C_BOARD_INFO("ft5x0x_ts", 0x38),//tong test
		.platform_data = &ft5x06_i2c_data,
		.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PC6),
	},
};  
#endif
/*ZTE: modified by tong.weili for Ft5X06 Touch 20120411 --*/

static int __init ventana_touch_init(void)
{    
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI
      gpio_request(TEGRA_GPIO_PC6, "touch-irq");	  
      gpio_direction_input(TEGRA_GPIO_PC6); 
      tegra_gpio_enable(TEGRA_GPIO_PC6);
      i2c_register_board_info(2, synaptics_i2c_board_info, ARRAY_SIZE(synaptics_i2c_board_info));
      //i2c_register_board_info(2, atmxt_224_i2c_info, ARRAY_SIZE(atmxt_224_i2c_info));  
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5X06
	i2c_register_board_info(2, ft5x06_i2c_board_info,
							ARRAY_SIZE(ft5x06_i2c_board_info));
#endif

     return 0;
}
#endif
/*ZTE: added by tong.weili for touch 20111215 --*/


static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_irq = TPS6586X_INT_BASE + TPS6586X_INT_USB_DET,
			.vbus_gpio = TEGRA_GPIO_PD0,
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = TEGRA_GPIO_PD3,
	},
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_HOST,
			/* ++ ZTE: denglibin 20120110 */
			.power_down_on_bus_suspend = 0,
			/* -- ZTE: denglibin 20120110 */
	},
	[1] = {
			.phy_config = &ulpi_phy_config,
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
			.phy_type = TEGRA_USB_PHY_TYPE_LINK_ULPI,
	},
	[2] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
			.hotplug = 1,
	},
};

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci_pdata[0],
};

static int __init ventana_gps_init(void)
{   
	int ret;
	struct clk *clk32 = clk_get_sys(NULL, "blink");
	if (!IS_ERR(clk32)) {
		clk_set_rate(clk32,clk32->parent->rate);
		clk_enable(clk32);
	}
	tegra_gpio_enable(TEGRA_GPIO_PU3);   /*163 RST*/
	tegra_gpio_enable(TEGRA_GPIO_PU4);   /*164 EN*/
	//gpio_export(TEGRA_GPIO_PU3, false);
	//gpio_export(TEGRA_GPIO_PU4, false);

	printk("[GPS]ventana_gps_init  PJ6 input\n");
    	ret = gpio_request(TEGRA_GPIO_PJ6, "GPS_UART_CTS");
	if (ret) {
		gpio_free(TEGRA_GPIO_PJ6);
	    	ret = gpio_request(TEGRA_GPIO_PJ6, "GPS_UART_CTS");
		if (ret) {
			printk("[GPS]request TEGRA_GPIO_PJ6 fail\n");
		}
	}
    	gpio_direction_input(TEGRA_GPIO_PJ6);
    	tegra_gpio_enable(TEGRA_GPIO_PJ6);

	printk("[GPS]ventana_gps_init ok\n");
	return 0;
}

static void ventana_power_off(void)
{
	int ret;

	ret = tps6586x_power_off();
	if (ret)
		pr_err("ventana: failed to power off\n");

	while(1);
}

static void __init ventana_power_off_init(void)
{
	pm_power_off = ventana_power_off;
}

static void ventana_usb_init(void)
{
	tegra_usb_phy_init(tegra_usb_phy_pdata, ARRAY_SIZE(tegra_usb_phy_pdata));
	/* OTG should be the first to be registered */
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	platform_device_register(&tegra_udc_device);

	tegra_ehci3_device.dev.platform_data=&tegra_ehci_pdata[2];
}

/*ZTE:add by wangtao for CMMB 20110702  ++*/
#ifdef CONFIG_ZTE_CMMB
/* static struct tegra_clk_init_table cmmb_clk[] = {
	{ "sbc1",	"clk_p", 6200000,	true},
}; */

static struct spi_board_info cmmb_spi_board_info[] = {
	/* spi master */
	{
		.modalias = "spi_sms2186",
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.max_speed_hz = 10000000,
		.platform_data = NULL,
		.irq = 0,
	},
};

static struct spi_clk_parent spi_cmmb_parent_clk[] = {
        [0] = {.name = "pll_p"},        
};

static struct tegra_spi_platform_data spi_cmmb_pdata = {
        .is_dma_based           = true,
        .is_dynamic_dma		= true,
        .max_dma_buffer         = (16 * 1024),
        .is_clkon_always        = true,
        .max_rate               = 5000000,
};

int ventana_cmmb_init(void)
{
    int err;
    int i;
    struct clk *c;

    printk("%s: [cmmb] cardhu_cmmb_init entry \n",__func__);

    for (i = 0; i < ARRAY_SIZE(spi_cmmb_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_cmmb_parent_clk[i].name);
              if (IS_ERR_OR_NULL(c)) {
                   pr_err("Not able to get the clock for %s\n",
                                                spi_cmmb_parent_clk[i].name);
                        continue;
                }
                spi_cmmb_parent_clk[i].parent_clk = c;
                spi_cmmb_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
   }
   spi_cmmb_pdata.parent_clk_list = spi_cmmb_parent_clk;
   spi_cmmb_pdata.parent_clk_count = ARRAY_SIZE(spi_cmmb_parent_clk);
   tegra_spi_device1.dev.platform_data = &spi_cmmb_pdata;
    
    platform_device_register(&tegra_spi_device1);
    err = spi_register_board_info(cmmb_spi_board_info,
    		                    ARRAY_SIZE(cmmb_spi_board_info));
    if (err < 0)
    {
        pr_err("%s: spi_register_board returned error %d\n",__func__, err);
    }
    return 0;
}
#endif
/*ZTE:add by wangtao for CMMB 20110702 --*/
#ifdef CONFIG_SND_SOC_TLV320AIC326X
static void audio_codec_I2S_en(bool enable)
{
        gpio_request(CODEC_I2S_EN_PIN, "CODEC_I2S_EN");	
	 tegra_gpio_enable(CODEC_I2S_EN_PIN);
        if (enable)
        {
            gpio_direction_output(CODEC_I2S_EN_PIN, 1);
        }
        else
        {
            gpio_direction_output(CODEC_I2S_EN_PIN, 0);
        }
}

static void audio_codec_3V3_EN(void)
{
        int ret = 0;
        
        ret = gpio_request(CODEC_3V3_EN_PIN, "CODEC_3V3_EN");
        if (ret < 0)
        {
             printk("[codec] audio_codec_3V3_EN\n");
        }
	 tegra_gpio_enable(CODEC_3V3_EN_PIN);
        gpio_direction_output(CODEC_3V3_EN_PIN, 1);
}

static void audio_bt_pcm_init(void)
{
        int ret = 0;
	 ret = gpio_request(CODEC_SECOND_VERSION_BT_PCM_EN_PIN, "bt_pcm_en");
        if (ret < 0)
        {
             printk("[codec] bt_pcm_en error\n");
        }
	 tegra_gpio_enable(CODEC_SECOND_VERSION_BT_PCM_EN_PIN);
        gpio_direction_output(CODEC_SECOND_VERSION_BT_PCM_EN_PIN, 0);
}
static int ventana_codec_init(void)
{
    int err;
    int i;
    struct clk *c;

    for (i = 0; i < ARRAY_SIZE(spi_codec_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_codec_parent_clk[i].name);
              if (IS_ERR_OR_NULL(c)) {
                   pr_err("Not able to get the clock for %s\n",
                                                spi_codec_parent_clk[i].name);
                        continue;
                }
                spi_codec_parent_clk[i].parent_clk = c;
                spi_codec_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
   }
   spi_codec_pdata.parent_clk_list = spi_codec_parent_clk;
   spi_codec_pdata.parent_clk_count = ARRAY_SIZE(spi_codec_parent_clk);
   tegra_spi_device3.dev.platform_data = &spi_codec_pdata;
    
    platform_device_register(&tegra_spi_device3);
    err = spi_register_board_info(codec_spi_board_info,
    		                    ARRAY_SIZE(codec_spi_board_info));
    if (err < 0)
    {
        pr_err("%s: spi_register_board returned error %d\n",__func__, err);
    }
    //ZTE: disable I2S port
    audio_codec_I2S_en(0);
    //ZTE: power up 3v3
    audio_codec_3V3_EN();
    #if CONFIG_TLV320AIC326x_U970_VERSION
    if (0 != zte_get_board_id())
    {
         audio_bt_pcm_init();
    }
    #else
    audio_bt_pcm_init();
    #endif
    return 0;
}
#endif

static void __init tegra_ventana_init(void)
{
	//struct board_info BoardInfo; /*ZTE: deleted by tong.weili for touch 20111215 */

	tegra_clk_init_from_table(ventana_clk_init_table);
	ventana_pinmux_init();
	ventana_i2c_init();
	ventana_uart_init();
	tegra_ehci2_device.dev.platform_data
		= &ventana_ehci2_ulpi_platform_data;
	platform_add_devices(ventana_devices, ARRAY_SIZE(ventana_devices));
	tegra_ram_console_debug_init();
	ventana_sdhci_init();
	ventana_charge_init();
	ventana_regulator_init();
	//ventana_charger_init();

 /*ZTE: modified by tong.weili for touch 20111215 ++*/
#if 0 
	tegra_get_board_info(&BoardInfo);

	/* boards with sku > 0 have atmel touch panels */
	if (BoardInfo.sku) {
		pr_info("Initializing Atmel touch driver\n");
		ventana_touch_init_atmel();
	} else {
		pr_info("Initializing Panjit touch driver\n");
		ventana_touch_init_panjit();
	}
#else
      ventana_touch_init();
#endif
 /*ZTE: modified by tong.weili for touch 20111215 --*/

#ifdef CONFIG_KEYBOARD_GPIO
	ventana_keys_init();
#endif

	ventana_usb_init();
	ventana_gps_init();
	ventana_panel_init();
	ventana_sensors_init();
	ventana_bt_rfkill();
	ventana_power_off_init();
	ventana_emc_init();

	ventana_setup_bluesleep();
	tegra_release_bootloader_fb();
#ifdef CONFIG_SND_SOC_TLV320AIC326X
       ventana_codec_init();
#endif
      #ifdef CONFIG_ZTE_CMMB
      ventana_cmmb_init();
      #endif
#ifdef CONFIG_ZTE_PROP_BRIDGE
     zte_hver_proc_init();
#endif
    zte_fm_state_proc_init();
}

int __init tegra_ventana_protected_aperture_init(void)
{
	if (!machine_is_ventana())
		return 0;

	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}
late_initcall(tegra_ventana_protected_aperture_init);

void __init tegra_ventana_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	tegra_reserve(SZ_256M, SZ_8M + SZ_1M, SZ_16M);

	tegra_ram_console_debug_reserve(SZ_1M);
}

MACHINE_START(VENTANA, "ventana")
	.boot_params    = 0x00000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_ventana_reserve,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine	= tegra_ventana_init,
MACHINE_END
