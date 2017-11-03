/*
 * arch/arm/mach-tegra/board-cardhu.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
//#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>
#include <linux/nfc/pn544.h>

//#include <sound/wm8903.h>
#include <media/tegra_dtv.h>
#include <sound/max98095.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
//#include <mach/tegra_wm8903_pdata.h>
//#include <mach/tegra_rt5640_pdata.h>
#include <mach/tegra_max98095_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/thermal.h>
#include <mach/pci.h>

#include "board.h"
#include "clock.h"
#include "board-cardhu.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "baseband-xmm-power.h"
#include "wdt-recovery.h"
#include "zte_board_ext_func.h" // ZTE£ºadded by pengtao

/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	.temp_throttle = 85000,
	.temp_shutdown = 90000,
	.temp_offset = TDIODE_OFFSET, /* temps based on tdiode */
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	.tc1 = 0,
	.tc2 = 1,
	.passive_delay = 2000,
#else
	.hysteresis_throttle = 1000,
#endif
};

/* ZTE: added by lipeng for bring up 20110719 ++ */
#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT224)
#include <linux/i2c/atmel_maxtouch.h>
#endif
/* ZTE: added by lipeng for bring up 20110719 -- */

/* !!!TODO: Change for cardhu (Taken from Ventana) */
static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 0,
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
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[2] = {
			.hssync_start_delay = 0,
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

static struct resource cardhu_bcm4329_rfkill_resources[] = {
	{
		.name   = "bcm4329_nshutdown_gpio",
		.start  = TEGRA_GPIO_PU0,
		.end    = TEGRA_GPIO_PU0,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device cardhu_bcm4329_rfkill_device = {
	.name = "bcm4329_rfkill",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(cardhu_bcm4329_rfkill_resources),
	.resource       = cardhu_bcm4329_rfkill_resources,
};

static struct resource cardhu_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PU6,
			.end    = TEGRA_GPIO_PU6,
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
			.start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device cardhu_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(cardhu_bluesleep_resources),
	.resource       = cardhu_bluesleep_resources,
};

static noinline void __init cardhu_setup_bluesleep(void)
{
	platform_device_register(&cardhu_bluesleep_device);
	tegra_gpio_enable(TEGRA_GPIO_PU6);
	tegra_gpio_enable(TEGRA_GPIO_PU1);
	return;
}

static __initdata struct tegra_clk_init_table cardhu_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x","pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	3187500,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"pll_a_out0",	0,		false},
	{ "dam0",	"pll_a_out0",	0,		false},
	{ "dam1",	"pll_a_out0",	0,		false},
	{ "dam2",	"pll_a_out0",	0,		false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ NULL,		NULL,		0,		0},
};

static struct pn544_i2c_platform_data nfc_pdata = {
	.irq_gpio = TEGRA_GPIO_PX0,
	.ven_gpio = TEGRA_GPIO_PP3,
	.firm_gpio = TEGRA_GPIO_PO7,
	};

static struct i2c_board_info __initdata cardhu_i2c_bus3_board_info[] = {
	{
		I2C_BOARD_INFO("pn544", 0x28),
		.platform_data = &nfc_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PX0),
	},
};
static struct tegra_i2c_platform_data cardhu_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data cardhu_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data cardhu_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data cardhu_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data cardhu_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};

#if 1
/*ZTE: added for CODEC, Begin*/
/* Equalizer filter coefs generated from the MAXIM MAX98095
 * Evaluation Kit (EVKIT) software tool */
static struct max98095_eq_cfg eq_cfg[] = {
	{ /* Flat response */
	.name = "SPK_MUSIC",
	.rate = 48000,
	.band1 = {0x268B, 0xC008, 0x3F08, 0x01EB, 0x0B16},
	.band2 = {0x6601, 0xC5C2, 0x3506, 0x1A87, 0x23D6},
	.band3 = {0x0A50, 0xC35F, 0x2146, 0x147E, 0x36AB},
	.band4 = {0x7FFE, 0xD606, 0x1E77, 0x304F, 0x3848},
	.band5 = {0x2594, 0xC01D, 0x3E37, 0x03C2, 0x0F02},
	},
	{ /* Low pass Fc=1KHz */
	.name = "HP_MUSIC",
	.rate = 48000,
	.band1 = {0x2997, 0xC002, 0x3F7E, 0x00E3, 0x0804},
	.band2 = {0x2405, 0xC009, 0x3F1D, 0x0218, 0x0A9D},
	.band3 = {0x2045, 0xC06B, 0x3F1A, 0x0745, 0x0AAA},
	.band4 = {0x2638, 0xC3AC, 0x32FF, 0x155D, 0x26AB},
	.band5 = {0x293E, 0xF89B, 0x0DE2, 0x3F92, 0x3E79},
	},
	{ /* BASS=-12dB, TREBLE=+9dB, Fc=5KHz */
	.name = "HIBOOST", 
	.rate = 44100,
	.band1 = {0x0815, 0xC001, 0x3AA4, 0x0003, 0x19A2},
	.band2 = {0x0815, 0xC103, 0x092F, 0x0B55, 0x3F56},
	.band3 = {0x0E0A, 0xC306, 0x1E5C, 0x136E, 0x3856},
	.band4 = {0x2459, 0xF665, 0x0CAA, 0x3F46, 0x3EBB},
	.band5 = {0x5BBB, 0x3FFF, 0xCEB0, 0x0000, 0x28CA},
	},
	{ /* BASS=12dB, TREBLE=+12dB */
	.name = "LOUD12DB",
	.rate = 44100,
	.band1 = {0x7FC1, 0xC001, 0x3EE8, 0x0020, 0x0BC7},
	.band2 = {0x51E9, 0xC016, 0x3C7C, 0x033F, 0x14E9},
	.band3 = {0x1745, 0xC12C, 0x1680, 0x0C2F, 0x3BE9},
	.band4 = {0x4536, 0xD7E2, 0x0ED4, 0x31DD, 0x3E42},
	.band5 = {0x7FEF, 0x3FFF, 0x0BAB, 0x0000, 0x3EED},
	},
	{
	.name = "FLAT",
	.rate = 16000,
	.band1 = {0x2000, 0xC004, 0x4000, 0x0141, 0x0000},
	.band2 = {0x2000, 0xC033, 0x4000, 0x0505, 0x0000},
	.band3 = {0x2000, 0xC268, 0x4000, 0x115F, 0x0000},
	.band4 = {0x2000, 0xDA62, 0x4000, 0x33C6, 0x0000},
	.band5 = {0x2000, 0x4000, 0x4000, 0x0000, 0x0000},
	},
	{
	.name = "LOWPASS1K",
	.rate = 16000,
	.band1 = {0x2000, 0xC004, 0x4000, 0x0141, 0x0000},
	.band2 = {0x5BE8, 0xC3E0, 0x3307, 0x15ED, 0x26A0},
	.band3 = {0x0F71, 0xD15A, 0x08B3, 0x2BD0, 0x3F67},
	.band4 = {0x0815, 0x3FFF, 0xCF78, 0x0000, 0x29B7},
	.band5 = {0x0815, 0x3FFF, 0xCF78, 0x0000, 0x29B7},
	},
	{ /* BASS=-12dB, TREBLE=+9dB, Fc=2KHz */
	.name = "HIBOOST", 
	.rate = 16000,
	.band1 = {0x0815, 0xC001, 0x3BD2, 0x0009, 0x16BF},
	.band2 = {0x080E, 0xC17E, 0xF653, 0x0DBD, 0x3F43},
	.band3 = {0x0F80, 0xDF45, 0xEE33, 0x36FE, 0x3D79},
	.band4 = {0x590B, 0x3FF0, 0xE882, 0x02BD, 0x3B87},
	.band5 = {0x4C87, 0xF3D0, 0x063F, 0x3ED4, 0x3FB1},
	},
	{ /* BASS=12dB, TREBLE=+12dB */
	.name = "LOUD12DB", 
	.rate = 16000,	
	.band1 = {0x7FC1, 0xC001, 0x3D07, 0x0058, 0x1344},
	.band2 = {0x2DA6, 0xC013, 0x3CF1, 0x02FF, 0x138B},
	.band3 = {0x18F1, 0xC08E, 0x244D, 0x0863, 0x34B5},
	.band4 = {0x2BE0, 0xF385, 0x04FD, 0x3EC5, 0x3FCE},
	.band5 = {0x7FEF, 0x4000, 0x0BAB, 0x0000, 0x3EED},
	},
};

static struct max98095_biquad_cfg bq_cfg[] = {
	{
	.name = "LP4K",
	.rate = 44100,
	.band1 = {0x5019, 0xe0de, 0x03c2, 0x0784, 0x03c2},
	.band2 = {0x5013, 0xe0e5, 0x03c1, 0x0783, 0x03c1},
	},
	{
	.name = "HP4K",
	.rate = 44100,
	.band1 = {0x5019, 0xe0de, 0x2e4b, 0xa36a, 0x2e4b},
	.band2 = {0x5013, 0xe0e5, 0x2e47, 0xa371, 0x2e47},
	},
};

static struct max98095_pdata cardhu_max98095_pdata = {
	/* equalizer configuration */
	.eq_cfg = eq_cfg,
	.eq_cfgcnt = ARRAY_SIZE(eq_cfg), 

	/* biquad filter configuration */
	.bq_cfg = bq_cfg,
	.bq_cfgcnt = ARRAY_SIZE(bq_cfg), 

	/* microphone configuration */
	.digmic_left_mode = 0,  /* 0 = normal analog mic */
	.digmic_right_mode = 0, /* 0 = normal analog mic */
};


#endif
//ZTE: added for CODEC, end



// Modify by Nvidia for Android 3.2
#if 0
static struct tegra_audio_platform_data tegra_i2s_pdata[] = {
	[0] = {
		.i2s_master	= true,
		.dma_on		= true,  /* use dma by default */
		.i2s_master_clk = 48000,
		.dev_clk_rate	= 12288000,
		.mode		= AUDIO_FRAME_FORMAT_I2S,
		.fifo_fmt	= AUDIO_FIFO_PACK_16,
		.bit_size	= AUDIO_BIT_SIZE_16,
		.i2s_bus_width	= 32,
		.dsp_bus_width	= 16,
	},
	[1] = {
		.i2s_master	= true,
		.dma_on		= true,  /* use dma by default */
		.i2s_master_clk = 8000,
		.dev_clk_rate	= 1024000,
		.mode		= AUDIO_FRAME_FORMAT_DSP,
		.fifo_fmt	= AUDIO_FIFO_NOP,
		.bit_size	= AUDIO_BIT_SIZE_16,
		.i2s_bus_width	= 32,
		.dsp_bus_width	= 16,
	},
	[2] = {
		.i2s_master	= true,
		.dma_on		= true,  /* use dma by default */
		.i2s_master_clk = 8000,
		.dev_clk_rate	= 1024000,
		.mode		= AUDIO_FRAME_FORMAT_DSP,
		.fifo_fmt	= AUDIO_FIFO_NOP,
		.bit_size	= AUDIO_BIT_SIZE_16,
		.i2s_bus_width	= 32,
		.dsp_bus_width	= 16,
	},
};

static struct tegra_audio_platform_data tegra_spdif_pdata = {
	.dma_on = true,  /* use dma by default */
	.dev_clk_rate = 6144000,
	.mode = SPDIF_BIT_MODE_MODE16BIT,
	.fifo_fmt = AUDIO_FIFO_PACK_16,
};
#endif


#if 0
struct tegra_wired_jack_conf audio_wr_jack_conf = {
	.hp_det_n = TEGRA_GPIO_PW2,
	.en_mic_ext = TEGRA_GPIO_PX1,
	.en_mic_int = TEGRA_GPIO_PX0,
};
#endif

#if 0
static struct wm8903_platform_data cardhu_wm8903_pdata = {
	.irq_active_low = 0,
	.micdet_cfg = 0,
	.micdet_delay = 100,
	.gpio_base = CARDHU_GPIO_WM8903(0),
	.gpio_cfg = {
		(WM8903_GPn_FN_DMIC_LR_CLK_OUTPUT << WM8903_GP1_FN_SHIFT),
		(WM8903_GPn_FN_DMIC_LR_CLK_OUTPUT << WM8903_GP2_FN_SHIFT) |
			WM8903_GP2_DIR,
		0,
		WM8903_GPIO_NO_CONFIG,
		WM8903_GPIO_NO_CONFIG,
	},
};

static struct i2c_board_info __initdata cardhu_codec_info[] = {
	{
		I2C_BOARD_INFO("wm8903", 0x1a),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ),
		.platform_data = &cardhu_wm8903_pdata,
	},
	{
		 I2C_BOARD_INFO("rt5640", 0x1c),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ),
	},
};
#endif

static struct i2c_board_info __initdata max98095_board_info = {
	I2C_BOARD_INFO("max98095", 0x10),
	.platform_data = &cardhu_max98095_pdata,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ),
};

static void cardhu_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &cardhu_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &cardhu_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &cardhu_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &cardhu_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &cardhu_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);

	/*i2c_register_board_info(4, cardhu_codec_info,
					ARRAY_SIZE(cardhu_codec_info));*/
	i2c_register_board_info(4, &max98095_board_info, 1);
	i2c_register_board_info(2, cardhu_i2c_bus3_board_info, 1);
}

static struct platform_device *cardhu_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};
static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data cardhu_uart_pdata;
static struct tegra_uart_platform_data cardhu_loopback_uart_pdata;

static void __init uart_debug_init(void)
{
	struct board_info board_info;
	int debug_port_id;

	tegra_get_board_info(&board_info);

	debug_port_id = get_tegra_uart_debug_port_id();
	if (debug_port_id < 0) {
		debug_port_id = 0;
			/* UARTB is debug port
			 *       for SLT - E1186/E1187/PM269
			 *       for E1256/E1257
			 */
		if (((board_info.sku & SKU_SLT_ULPI_SUPPORT) &&
			((board_info.board_id == BOARD_E1186) ||
			(board_info.board_id == BOARD_E1187) ||
			(board_info.board_id == BOARD_PM269))) ||
			(board_info.board_id == BOARD_E1256) ||
			(board_info.board_id == BOARD_E1257))
				debug_port_id = 1;
	}
	switch (debug_port_id) {
	case 0:
		/* UARTA is the debug port. */
		pr_info("Selecting UARTA as the debug console\n");
		cardhu_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;

	case 1:
		/* UARTB is the debug port. */
		pr_info("Selecting UARTB as the debug console\n");
		cardhu_uart_devices[1] = &debug_uartb_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartb");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartb_device.dev.platform_data))->mapbase;
		break;

	case 2:
		/* UARTC is the debug port. */
		pr_info("Selecting UARTC as the debug console\n");
		cardhu_uart_devices[2] = &debug_uartc_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartc");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartc_device.dev.platform_data))->mapbase;
		break;

	case 3:
		/* UARTD is the debug port. */
		pr_info("Selecting UARTD as the debug console\n");
		cardhu_uart_devices[3] = &debug_uartd_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartd");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
		break;

	case 4:
		/* UARTE is the debug port. */
		pr_info("Selecting UARTE as the debug console\n");
		cardhu_uart_devices[4] = &debug_uarte_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uarte");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarte_device.dev.platform_data))->mapbase;
		break;

	default:
		pr_info("The debug console id %d is invalid, Assuming UARTA", debug_port_id);
		cardhu_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;
	}
	return;
}

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
	    }
    }
    else if (0 == val)
    {
    	if (step == 1) {
    		pr_info("%s: set ap sleep state 0\n", __func__);
	        gpio_set_value(AP_TO_MODEM_SLP, 0);
	    }
    }
    return;
}

static void __init cardhu_uart_init(void)
{
	struct clk *c;
	int i;

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
	cardhu_uart_pdata.parent_clk_list = uart_parent_clk;
	cardhu_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	cardhu_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	cardhu_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	cardhu_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &cardhu_uart_pdata;
	tegra_uartb_device.dev.platform_data = &cardhu_uart_pdata;
	tegra_uartc_device.dev.platform_data = &cardhu_uart_pdata;

	modem_uart_pdata = cardhu_uart_pdata;
	modem_uart_pdata.wake_peer = wake_modem;
	modem_uart_pdata.sleep_ctrl = modem_sleep_control;

	tegra_uartd_device.dev.platform_data = &modem_uart_pdata;
	/* UARTE is used for loopback test purpose */
	tegra_uarte_device.dev.platform_data = &cardhu_loopback_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, clk_get_rate(c));
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	platform_add_devices(cardhu_uart_devices,
				ARRAY_SIZE(cardhu_uart_devices));
}

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct platform_device *cardhu_spi_devices[] __initdata = {
	&tegra_spi_device4,
};

struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data cardhu_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 100000000,
};

static void __init cardhu_spi_init(void)
{
	int i;
	struct clk *c;
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	cardhu_spi_pdata.parent_clk_list = spi_parent_clk;
	cardhu_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	tegra_spi_device4.dev.platform_data = &cardhu_spi_pdata;
	platform_add_devices(cardhu_spi_devices,
				ARRAY_SIZE(cardhu_spi_devices));

	if (board_info.board_id == BOARD_E1198) {
		tegra_spi_device2.dev.platform_data = &cardhu_spi_pdata;
		platform_device_register(&tegra_spi_device2);
		tegra_spi_slave_device1.dev.platform_data = &cardhu_spi_pdata;
		platform_device_register(&tegra_spi_slave_device1);
	}
}

static void __init cardhu_dtv_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	if (board_info.board_id == BOARD_E1186)
		platform_device_register(&tegra_dtv_device);
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

#if 0
static struct tegra_wm8903_platform_data cardhu_audio_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1,
	.gpio_ext_mic_en	= -1,
};

static struct tegra_rt5640_platform_data cardhu_audio_rt5640_pdata = {
	.gpio_spkr_en		= -1,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1,
	.gpio_ext_mic_en	= -1,
};

static struct platform_device cardhu_audio_device[] = {
	{
		.name	= "tegra-snd-wm8903",
		.id	= 0,
		.dev	= {
				.platform_data = &cardhu_audio_pdata,
			  }
	},
	{
		.name	= "tegra-snd-rt5640",
		.id	= 0,
		.dev	= {
				.platform_data = &cardhu_audio_rt5640_pdata,
			  }
	},
};
#endif

struct tegra_max98095_platform_data cardhu_audio_pdata = {
	.gpio_spkr_en		= -1,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1,
	.gpio_ext_mic_en	= -1,
	.gpio_mic_det		= TEGRA_GPIO_CDC_IRQ,	
	.audio_port_id		= {
		[HIFI_CODEC] = 0,
		[BASEBAND] = 2,
		[BT_SCO] = 3,
	},
};


static struct platform_device cardhu_audio_device = {
	.name	= "tegra-snd-max98095",
	.id	= 0,
	.dev	= {
		.platform_data  = &cardhu_audio_pdata,
	},
};

static struct platform_device modem_control_device = {
	.name = "modem_control",
	.id = -1,
};

static struct platform_device *cardhu_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_i2s_device2,
	&tegra_i2s_device3,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&cardhu_bcm4329_rfkill_device,
	&tegra_pcm_device,
	//&cardhu_audio_device[0],
	//&cardhu_audio_device[1],
	&cardhu_audio_device,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
	&modem_control_device,
};

#define MXT_CONFIG_CRC  0xD62DE8
static const u8 config[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x32, 0x0A, 0x00, 0x14, 0x14, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0x00, 0x00,
	0x1B, 0x2A, 0x00, 0x20, 0x3C, 0x04, 0x05, 0x00,
	0x02, 0x01, 0x00, 0x0A, 0x0A, 0x0A, 0x0A, 0xFF,
	0x02, 0x55, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x64, 0x02, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23,
	0x00, 0x00, 0x00, 0x05, 0x0A, 0x15, 0x1E, 0x00,
	0x00, 0x04, 0xFF, 0x03, 0x3F, 0x64, 0x64, 0x01,
	0x0A, 0x14, 0x28, 0x4B, 0x00, 0x02, 0x00, 0x64,
	0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x08, 0x10, 0x3C, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define MXT_CONFIG_CRC_SKU2000  0xA24D9A
static const u8 config_sku2000[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x32, 0x0A, 0x00, 0x14, 0x14, 0x19,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0x00, 0x00,
	0x1B, 0x2A, 0x00, 0x20, 0x3A, 0x04, 0x05, 0x00,  //23=thr  2 di
	0x04, 0x04, 0x41, 0x0A, 0x0A, 0x0A, 0x0A, 0xFF,
	0x02, 0x55, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,  //0A=limit
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23,
	0x00, 0x00, 0x00, 0x05, 0x0A, 0x15, 0x1E, 0x00,
	0x00, 0x04, 0x00, 0x03, 0x3F, 0x64, 0x64, 0x01,
	0x0A, 0x14, 0x28, 0x4B, 0x00, 0x02, 0x00, 0x64,
	0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x08, 0x10, 0x3C, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#if 0
static struct mxt_platform_data atmel_mxt_info = {
	.x_line         = 27,
	.y_line         = 42,
	.x_size         = 768,
	.y_size         = 1366,
	.blen           = 0x20,
	.threshold      = 0x3C,
	.voltage        = 3300000,              /* 3.3V */
	.orient         = 5,
	.config         = config,
	.config_length  = 157,
	.config_crc     = MXT_CONFIG_CRC,
	.irqflags       = IRQF_TRIGGER_FALLING,
/*	.read_chg       = &read_chg, */
	.read_chg       = NULL,
};

static struct i2c_board_info __initdata atmel_i2c_info[] = {
	{
		I2C_BOARD_INFO("atmel_mxt_ts", 0x5A),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH4),
		.platform_data = &atmel_mxt_info,
	}
};
#endif



static struct mxt_platform_data atmxt_224_i2c_data = {
              .max_x = 1280,
              .max_y = 800,
              .numtouch = 6,
              .init_platform_hw = NULL,
              .exit_platform_hw = NULL,
};

static struct i2c_board_info atmxt_224_i2c_info[] = {
	{
		I2C_BOARD_INFO("AtmelMxt_224_i2c", MXT_I2C_ADDRESS),
		.platform_data = &atmxt_224_i2c_data,
		.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS2),		
	},
};

static int __init cardhu_touch_init(void)
{
#if 0
	struct board_info BoardInfo;

	tegra_gpio_enable(TEGRA_GPIO_PH4);
	tegra_gpio_enable(TEGRA_GPIO_PH6);

	gpio_request(TEGRA_GPIO_PH4, "atmel-irq");
	gpio_direction_input(TEGRA_GPIO_PH4);

	gpio_request(TEGRA_GPIO_PH6, "atmel-reset");
	gpio_direction_output(TEGRA_GPIO_PH6, 0);
	msleep(1);
	gpio_set_value(TEGRA_GPIO_PH6, 1);
	msleep(100);

	tegra_get_board_info(&BoardInfo);
	if ((BoardInfo.sku & SKU_TOUCH_MASK) == SKU_TOUCH_2000) {
		atmel_mxt_info.config = config_sku2000;
		atmel_mxt_info.config_crc = MXT_CONFIG_CRC_SKU2000;
	}

	i2c_register_board_info(1, atmel_i2c_info, 1);

	return 0;
#endif
	int ret = 0;
	pr_info("### zte Touch Init...\n");
	i2c_register_board_info(0, atmxt_224_i2c_info, ARRAY_SIZE(atmxt_224_i2c_info));    
	ret = gpio_request(TEGRA_GPIO_PS2, "touch-irq");	
	if (ret < 0) 
        {		
            pr_err("%s(): gpio_request() fails for gpio %d (touch-irq)\n",	 __func__, TEGRA_GPIO_PS2);
            return ret;
        }
	gpio_direction_input(TEGRA_GPIO_PS2);

	tegra_gpio_enable(TEGRA_GPIO_PS2);
	pr_info("### zte Touch Init exit\n");

	return ret;
}

static struct tegra_uhsic_config uhsic_phy_config = {
	.enable_gpio = EN_HSIC_GPIO,
	.reset_gpio = -1,
	.sync_start_delay = 9,
	.idle_wait_delay = 17,
	.term_range_adj = 0,
	.elastic_underrun_limit = 16,
	.elastic_overrun_limit = 16,
};

static struct tegra_ehci_platform_data tegra_ehci_uhsic_pdata = {
	.phy_type = TEGRA_USB_PHY_TYPE_HSIC,
	.phy_config = &uhsic_phy_config,
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 1,
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
	},
	[1] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
	},
	[2] = {
			.phy_config = &utmi_phy_config[2],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
			.hotplug = 1,
	},
};

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci_pdata[0],
};

#ifdef CONFIG_USB_SUPPORT
static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_gpio = -1,
			.vbus_reg_supply = "vdd_vbus_micro_usb",
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = -1,
			.vbus_reg_supply = "vdd_vbus_typea_usb",
	},
};

static int cardhu_usb_hsic_postsupend(void)
{
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2);
#endif
	return 0;
}

static int cardhu_usb_hsic_preresume(void)
{
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
#endif
	return 0;
}

static int cardhu_usb_hsic_phy_ready(void)
{
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L0);
#endif
	return 0;
}

static int cardhu_usb_hsic_phy_off(void)
{
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L3);
#endif
	return 0;
}

static void cardhu_usb_init(void)
{
	struct board_info bi;

	tegra_get_board_info(&bi);

	tegra_usb_phy_init(tegra_usb_phy_pdata,
			ARRAY_SIZE(tegra_usb_phy_pdata));

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);
	if (bi.board_id == BOARD_PM267) {
		uhsic_phy_config.reset_gpio =
			PM267_SMSC4640_HSIC_HUB_RESET_GPIO;
		tegra_ehci2_device.dev.platform_data = &tegra_ehci_uhsic_pdata;
		platform_device_register(&tegra_ehci2_device);
	} else if (bi.board_id == BOARD_E1256) {
		tegra_ehci2_device.dev.platform_data = &tegra_ehci_uhsic_pdata;
		platform_device_register(&tegra_ehci2_device);
	} else if (bi.board_id == BOARD_E1186) {
		/* for baseband devices do not switch off phy during suspend */
		tegra_ehci_uhsic_pdata.power_down_on_bus_suspend = 0;
		uhsic_phy_config.postsuspend = cardhu_usb_hsic_postsupend;
		uhsic_phy_config.preresume = cardhu_usb_hsic_preresume;
		uhsic_phy_config.usb_phy_ready = cardhu_usb_hsic_phy_ready;
		uhsic_phy_config.post_phy_off = cardhu_usb_hsic_phy_off;
		tegra_ehci2_device.dev.platform_data = &tegra_ehci_uhsic_pdata;
		/* baseband registration happens in baseband-xmm-power  */
	} else {
		tegra_ehci2_device.dev.platform_data = &tegra_ehci_pdata[1];
		platform_device_register(&tegra_ehci2_device);
	}

	tegra_ehci3_device.dev.platform_data = &tegra_ehci_pdata[2];
	platform_device_register(&tegra_ehci3_device);
}
#else
static void cardhu_usb_init(void) { }
#endif

static void cardhu_gps_init(void)
{
	int ret;	
	ret = gpio_request(TEGRA_GPIO_PR5, "GPS_PWR");
	if (ret){
		printk("[GPS]TEGRA_GPIO_PR5 fail\n");
		goto fail;
	}
	gpio_direction_output(TEGRA_GPIO_PR5, 1);
	tegra_gpio_enable(TEGRA_GPIO_PR5);
	gpio_export(TEGRA_GPIO_PR5, false); /*141*/

	tegra_gpio_enable(TEGRA_GPIO_PU2);/*EN*/
	tegra_gpio_enable(TEGRA_GPIO_PU3);/*RST*/
	//gpio_export(TEGRA_GPIO_PU2, false);
	//gpio_export(TEGRA_GPIO_PU3, false);
	
	printk("[GPS]cardhu_gps_init ok\n");
		return ;
fail:
	printk("[GPS]cardhu_gps_init fail\n");
		return;
}

static void cardhu_nfc_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PX0);
	tegra_gpio_enable(TEGRA_GPIO_PP3);
	tegra_gpio_enable(TEGRA_GPIO_PO7);
}

static struct baseband_power_platform_data tegra_baseband_power_data = {
	.baseband_type = BASEBAND_XMM,
	.modem = {
	.xmm = {
			.bb_rst = XMM_GPIO_BB_RST,
			.bb_on = XMM_GPIO_BB_ON,
			.ipc_bb_wake = XMM_GPIO_IPC_BB_WAKE,
			.ipc_ap_wake = XMM_GPIO_IPC_AP_WAKE,
			.ipc_hsic_active = XMM_GPIO_IPC_HSIC_ACTIVE,
			.ipc_hsic_sus_req = XMM_GPIO_IPC_HSIC_SUS_REQ,
			.hsic_device = &tegra_ehci2_device,
		},
	},
};

static struct platform_device tegra_baseband_power_device = {
	.name = "baseband_xmm_power",
	.id = -1,
	.dev = {
		.platform_data = &tegra_baseband_power_data,
	},
};

static struct platform_device tegra_baseband_power2_device = {
	.name = "baseband_xmm_power2",
	.id = -1,
	.dev = {
		.platform_data = &tegra_baseband_power_data,
	},
};


static struct tegra_pci_platform_data cardhu_pci_platform_data = {
	.port_status[0]	= 1,
	.port_status[1]	= 1,
	.port_status[2]	= 1,
	.use_dock_detect	= 0,
	.gpio		= 0,
};

static void cardhu_pci_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	if (board_info.board_id == BOARD_E1291) {
		cardhu_pci_platform_data.port_status[0] = 0;
		cardhu_pci_platform_data.port_status[1] = 0;
		cardhu_pci_platform_data.port_status[2] = 1;
		cardhu_pci_platform_data.use_dock_detect = 1;
		cardhu_pci_platform_data.gpio = DOCK_DETECT_GPIO;
	}
	tegra_pci_device.dev.platform_data = &cardhu_pci_platform_data;
	platform_device_register(&tegra_pci_device);
}

static void cardhu_modem_init(void)
{
#if 0
	struct board_info board_info;
	int w_disable_gpio, ret;

	tegra_get_board_info(&board_info);
	switch (board_info.board_id) {
	case BOARD_E1291:
	case BOARD_E1198:
		if (((board_info.board_id == BOARD_E1291) &&
				(board_info.fab < BOARD_FAB_A03)) ||
			((board_info.board_id == BOARD_E1198) &&
					(board_info.fab < BOARD_FAB_A02))) {
			w_disable_gpio = TEGRA_GPIO_PH5;
		} else {
			w_disable_gpio = TEGRA_GPIO_PDD5;
		}
		tegra_gpio_enable(w_disable_gpio);
		ret = gpio_request(w_disable_gpio, "w_disable_gpio");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %d\n",
				__func__, w_disable_gpio);
		else
			gpio_direction_input(w_disable_gpio);

		/* E1291-A04 & E1198:A02: Set PERST signal to high */
		if (((board_info.board_id == BOARD_E1291) &&
				(board_info.fab >= BOARD_FAB_A04)) ||
			((board_info.board_id == BOARD_E1198) &&
					(board_info.fab >= BOARD_FAB_A02))) {
			ret = gpio_request(TEGRA_GPIO_PH7, "modem_perst");
			if (ret < 0) {
				pr_err("%s(): Error in allocating gpio "
					"TEGRA_GPIO_PH7\n", __func__);
				break;
			}
			gpio_direction_output(TEGRA_GPIO_PH7, 1);
			tegra_gpio_enable(TEGRA_GPIO_PH7);
		}
		break;
	case BOARD_E1186:
		tegra_gpio_enable(
			tegra_baseband_power_data.modem.xmm.bb_rst);
		tegra_gpio_enable(
			tegra_baseband_power_data.modem.xmm.bb_on);
		tegra_gpio_enable(
			tegra_baseband_power_data.modem.xmm.ipc_bb_wake);
		tegra_gpio_enable(
			tegra_baseband_power_data.modem.xmm.ipc_ap_wake);
		tegra_gpio_enable(
			tegra_baseband_power_data.modem.xmm.ipc_hsic_active);
		tegra_gpio_enable(
			tegra_baseband_power_data.modem.xmm.ipc_hsic_sus_req);
		platform_device_register(&tegra_baseband_power_device);
		platform_device_register(&tegra_baseband_power2_device);
		break;
	default:
		break;
	}
#endif
}

#ifdef CONFIG_SATA_AHCI_TEGRA
static void cardhu_sata_init(void)
{
	platform_device_register(&tegra_sata_device);
}
#else
static void cardhu_sata_init(void) { }
#endif

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
		.max_speed_hz = 12000000,
		.platform_data = NULL,
		.irq = 0,
	},
};

int cardhu_cmmb_init(void)
{
    int err;
    printk("%s: [cmmb] cardhu_cmmb_init entry \n",__func__);
   /* if (cmmb_clk)
    tegra_clk_init_from_table(cmmb_clk);*/
    
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


//ZTE: Modify by wangyunan for Codec Enable Begin
//extern void create_codec_proc_file(void);
static void audio_codec_enbale(bool enable)
{
    gpio_request(CODEC_CEN_GPIO, "CODEC_ENABLE");	
    tegra_gpio_enable(CODEC_CEN_GPIO);
    gpio_direction_output(CODEC_CEN_GPIO, 0);
    if (enable)
    {
        gpio_set_value(CODEC_CEN_GPIO, 1);
    }
    else
    {
        gpio_set_value(CODEC_CEN_GPIO, 0);
    }
    //gpio_free(CODEC_CEN_GPIO);
}
//ZTE: Modify by wangyunan for Codec Enable End

static void __init tegra_cardhu_init(void)
{
	tegra_thermal_init(&thermal_data);
	tegra_clk_init_from_table(cardhu_clk_init_table);
	cardhu_pinmux_init();
	cardhu_i2c_init();
	cardhu_spi_init();
	cardhu_usb_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	cardhu_edp_init();
#endif
	cardhu_uart_init();
	cardhu_tsensor_init();
	platform_add_devices(cardhu_devices, ARRAY_SIZE(cardhu_devices));
	tegra_ram_console_debug_init();
	cardhu_sdhci_init();
	cardhu_regulator_init();
	cardhu_dtv_init();
	cardhu_gpio_switch_regulator_init();
	cardhu_suspend_init();
	cardhu_power_off_init();
	cardhu_touch_init();
	cardhu_gps_init();
	cardhu_modem_init();
	cardhu_kbc_init();
	cardhu_scroll_init();
	cardhu_keys_init();
	cardhu_panel_init();
	//cardhu_pmon_init();
	cardhu_sensors_init();
	cardhu_setup_bluesleep();
	cardhu_sata_init();
	//audio_wired_jack_init();
	//create_codec_proc_file();
        audio_codec_enbale(1);
	cardhu_pins_state_init();
	cardhu_emc_init();
	tegra_release_bootloader_fb();
	//cardhu_nfc_init();/*ZTE:modify by wangtao there is no nfc on T98 and the pin P03 used  by I2S2*/
	cardhu_pci_init();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
#ifdef CONFIG_ZTE_CMMB
	cardhu_cmmb_init();/*ZTE:add by wangtao for CMMB 20110711 */ 
#endif
#ifdef CONFIG_ZTE_PROP_BRIDGE
	zte_hver_proc_init(); // ZTE:added by pengtao fro hardware version 20110921
#endif
}

static void __init cardhu_ramconsole_reserve(unsigned long size)
{
	//struct resource *res; /*ZTE: deleted by tong.weili for unused 20120203 */
	//long ret;                 /*ZTE: deleted by tong.weili for unused 20120203 */

	tegra_ram_console_debug_reserve(SZ_1M);
}

static void __init tegra_cardhu_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* support 1920X1200 with 24bpp */
	tegra_reserve(0, SZ_8M + SZ_1M, SZ_8M + SZ_1M);
#else
	tegra_reserve(SZ_128M, SZ_8M, SZ_8M);
#endif
	cardhu_ramconsole_reserve(SZ_1M);
}

MACHINE_START(CARDHU, "cardhu")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_cardhu_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_cardhu_init,
MACHINE_END
