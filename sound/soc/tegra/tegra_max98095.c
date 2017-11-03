/*
 * tegra_soc_max98085.c  --  SoC audio for tegra
 *
 * Copyright  2010-2011 NVIDIA Corporation
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

#define DEBUG

#define DRV_NAME "tegra-snd-max98095"

#include <asm/mach-types.h>

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <sound/asound.h>
#include "tegra30_ahub.h"
#include "tegra30_i2s.h"
#include "tegra30_dam.h"
#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"
#include <mach/tegra_max98095_pdata.h>
#include "../codecs/max98095.h"

#include <linux/clk.h>
extern struct snd_soc_codec *max98095_codec;
extern  int rate_value(int rate, u8 *value);

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)
#define GPIO_HP_DET     BIT(4)

struct tegra_max98095 {
	struct tegra_asoc_utils_data util_data;
	struct tegra_max98095_platform_data *pdata;
	struct regulator *spk_reg;
	struct regulator *dmic_reg;
	int gpio_requested;
	bool init_done;
	int is_call_mode;
	int is_device_bt;
	struct codec_config codec_info[NUM_I2S_DEVICES];
#ifdef CONFIG_SWITCH
	int jack_status;
#endif
};
static int tegra_call_mode_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int tegra_call_mode_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_max98095 *machine = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = machine->is_call_mode;

	return 0;
}

static int tegra_call_mode_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_max98095 *machine = snd_kcontrol_chip(kcontrol);
	int is_call_mode_new = ucontrol->value.integer.value[0];
	int codec_index;

	if (machine->is_call_mode == is_call_mode_new)
		return 0;

	if (machine->is_device_bt)
		codec_index = BT_SCO;

	if (is_call_mode_new) {
		if (machine->codec_info[codec_index].rate == 0 ||
			machine->codec_info[codec_index].channels == 0)
				return -EINVAL;
        	tegra30_make_voice_call_connections(
			&machine->codec_info[codec_index],
			&machine->codec_info[BASEBAND]);
	} else {
		tegra30_break_voice_call_connections(
			&machine->codec_info[codec_index],
			&machine->codec_info[BASEBAND]);
	}

	machine->is_call_mode = is_call_mode_new;
	return 1;
}

struct snd_kcontrol_new tegra_call_mode_control = {
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Call Mode Switch",
	.private_value = 0xffff,
	.info = tegra_call_mode_info,
	.get = tegra_call_mode_get,
	.put = tegra_call_mode_put
};

static int tegra_max98095_set_dam_cif(int dam_ifc, int srate,
			int channels, int bit_size, int src_on, int src_srate,
			int src_channels, int src_bit_size)
{
	tegra30_dam_set_gain(dam_ifc, TEGRA30_DAM_CHIN1, 0x1000);
	tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHOUT,
				srate);
	tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHIN1,
				srate);
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN1,
		channels, bit_size, channels,
				bit_size);
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHOUT,
		channels, bit_size, channels,
				bit_size);

	if (src_on) {
		tegra30_dam_set_gain(dam_ifc, TEGRA30_DAM_CHIN0_SRC, 0x1000);
		tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			src_srate);
		tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			src_channels, src_bit_size, 1, 16);
	}

	return 0;
}

static int tegra_max98095_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, i2s_daifmt;
	int err;
	printk("[audio]   tegra_max98095_hw_params in\n");
	srate = params_rate(params);
	switch (srate) {
	case 64000:
	case 88200:
	case 96000:
		mclk = 128 * srate;
		break;
	default:
		mclk = 256 * srate;
		break;
	}
	/* FIXME: Codec only requires >= 3MHz if OSR==0 */
	while (mclk < 6000000)
		mclk *= 2;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	i2s_daifmt = SND_SOC_DAIFMT_NB_NF |
		     SND_SOC_DAIFMT_CBS_CFS;

	/* Use DSP mode for mono on Tegra20 */
	if ((params_channels(params) != 2) &&machine_is_ventana() )
		i2s_daifmt |= SND_SOC_DAIFMT_DSP_A;
	else	
		i2s_daifmt |= SND_SOC_DAIFMT_I2S;

	err = snd_soc_dai_set_fmt(codec_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC1,
					TEGRA20_DAS_DAP_ID_1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dap-dac path\n");
		return err;
	}

	err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_1,
					TEGRA20_DAS_DAP_SEL_DAC1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}
#endif
	printk("[audio]   tegra_max98095_hw_params ok\n");
	return 0;
}

#if 0
static int tegra_bt_sco_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 64 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	err = snd_soc_dai_set_fmt(cpu_dai,
					SND_SOC_DAIFMT_DSP_A |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC2,
					TEGRA20_DAS_DAP_ID_4);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}

	err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_4,
					TEGRA20_DAS_DAP_SEL_DAC2);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}
#endif
	return 0;
}

static int tegra_spdif_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 128 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	return 0;
}
#endif 

static int tegra_voice_call_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk;
	int err;
	printk("[audio]   tegra_voice_call_hw_params in\n");
	srate = params_rate(params);
	switch (srate) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break; 
	default:
		return -EINVAL;
		break;
	}

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	err = snd_soc_dai_set_fmt(codec_dai,
					SND_SOC_DAIFMT_I2S |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

	printk(" tegra_voice_call_hw_params params_rate(params): %d\n",params_rate(params));
	printk(" tegra_voice_call_hw_paramsparams_channels(params): %d\n",params_channels(params));
	/* voice configuration */
	machine->codec_info[1].rate = params_rate(params);
	machine->codec_info[1].channels = params_channels(params);	
	machine->codec_info[1].bitsize = 16;
	machine->codec_info[1].is_i2smaster = 1;
	machine->codec_info[1].is_format_dsp = 1;

	machine->is_device_bt = 0;

	return 0;
}

static void tegra_voice_call_shutdown(struct snd_pcm_substream *substream)
{
#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_max98095 *machine  =
			snd_soc_card_get_drvdata(rtd->codec->card);
	return 0;
#endif
}

static int tegra_bt_voice_call_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);

	int err, srate, mclk, min_mclk;
	printk("[bt_voice]	tegra_bt_voice_call_hw_params in\n");
	printk("[bt_voice]	tegra_bt_voice_call_hw_params card is %s\n",card->name);
	printk("[bt_voice]	tegra_bt_voice_call_hw_params codec is %s\n",rtd->codec->name);
	printk("[bt_voice]	tegra_bt_voice_call_hw_params codec_dai is %s\n",rtd->codec_dai->name);
	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 64 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		printk("[bt_voice]	tegra_bt_voice_call_hw_params	err = %d",err);
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	err = snd_soc_dai_set_fmt(codec_dai,
					SND_SOC_DAIFMT_I2S |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}
	printk("[bt_voice]	tegra_bt_voice_call_hw_param codec_dai ->id= %d\n",codec_dai ->id);

	err = snd_soc_dai_set_fmt(rtd->cpu_dai,
			SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		dev_err(rtd->codec->card->dev, "cpu_dai fmt not set\n");
		return err;
	}
	printk("[bt_voice]	tegra_bt_voice_call_hw_params rtd->cpu_dai->id = %d\n",rtd->cpu_dai->id);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);


	/* codec configuration */
	machine->codec_info[BT_SCO].rate = params_rate(params);
	machine->codec_info[BT_SCO].channels = params_channels(params);
	machine->codec_info[BT_SCO].bitsize = 16;
	machine->codec_info[BT_SCO].is_i2smaster = 1;
	machine->codec_info[BT_SCO].is_format_dsp = 1;

	/* baseband configuration */
	machine->codec_info[BASEBAND].rate = params_rate(params);
	machine->codec_info[BASEBAND].channels =params_channels(params);
	machine->codec_info[BASEBAND].bitsize = 16;
	machine->codec_info[BASEBAND].is_i2smaster = 1;
	machine->codec_info[BASEBAND].is_format_dsp = 1;

	machine->is_device_bt = 1;

	/*codec register config Begin*/
	printk("[bt_voice] codec register config\n");

	snd_soc_write(codec, M98095_026_SYS_CLK,0x10);//10Mhz<MCLK<20Mhz
	/*snd_soc_write(codec, M98095_031_DAI2_CLKMODE,
		0x01);				// Sample Rate is 8kHz
	snd_soc_write(codec, M98095_032_DAI2_CLKCFG_HI,
		0x10);
	snd_soc_write(codec, M98095_033_DAI2_CLKCFG_LO,
		0x00);			//  Set Codec AnyClock
	snd_soc_write(codec, M98095_034_DAI2_FORMAT,
		M98095_DAI_MAS |M98095_DAI_TDM);	// Set DAI2 is Master and TDM format
	snd_soc_write(codec, M98095_035_DAI2_CLOCK,
		0x27);				// Set DAI2 Clock 
	snd_soc_write(codec, M98095_037_DAI2_TDM,
		0x10);				// Set TDM Right Time Slot
	snd_soc_update_bits(codec, M98095_038_DAI2_FILTERS,
		M98095_DAI_DHF, 0);		// Update sample rate mode*/
	snd_soc_write(codec, M98095_03B_DAI3_CLKMODE,
		0x01);				// Sample Rate is 8kHz
	snd_soc_write(codec, M98095_03C_DAI3_CLKCFG_HI,
		0x10);
	snd_soc_write(codec, M98095_03D_DAI3_CLKCFG_LO,
		0x00);			//  Set Codec AnyClock
	snd_soc_write(codec, M98095_03E_DAI3_FORMAT,
		 M98095_DAI_TDM|M98095_DAI_BCI);	// Set DAI3 is Slave and TDM format		 
	snd_soc_write(codec, M98095_03F_DAI3_CLOCK,
		 0x00);				// Set DAI3 Clock 
	snd_soc_write(codec, M98095_041_DAI3_TDM,
		 0x10);				// Set TDM Right Time Slot
	snd_soc_update_bits(codec, M98095_042_DAI3_FILTERS,
		M98095_DAI_DHF, 0);	// Update sample rate mode
	snd_soc_write(codec, M98095_05A_LVL_DAI2_PLAY,0x00);//set DAI2 vol in palyback DSP
	snd_soc_write(codec, M98095_05C_LVL_DAI3_PLAY,0x00);//set DAI3 vol in palyback DSP
	snd_soc_write(codec, M98095_091_PWR_EN_OUT,0x01);   //enable left DAC	
	/*codec register config End*/

	printk("[bt_voice]	tegra_bt_voice_call_hw_params out\n");
	return 0;
}

static void tegra_bt_voice_call_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	struct tegra_max98095 *machine  =
			snd_soc_card_get_drvdata(rtd->codec->card);

	machine->codec_info[BT_SCO].rate = 0;
	machine->codec_info[BT_SCO].channels = 0;

	//snd_soc_write(codec, M98095_05A_LVL_DAI2_PLAY,0x00);
	snd_soc_write(codec, M98095_05C_LVL_DAI3_PLAY,0x00);
#if 0
	snd_soc_write(codec, M98095_05A_LVL_DAI2_PLAY,0x80);
	snd_soc_write(codec, M98095_05C_LVL_DAI3_PLAY,0x80);

	/*codec register config Begin*/
	/*snd_soc_write(codec, M98095_031_DAI2_CLKMODE,
		0x00);				
	snd_soc_write(codec, M98095_032_DAI2_CLKCFG_HI,
		0x00);
	snd_soc_write(codec, M98095_033_DAI2_CLKCFG_LO,
		0x00);		
	snd_soc_write(codec, M98095_034_DAI2_FORMAT,
		0x00);	
	snd_soc_write(codec, M98095_035_DAI2_CLOCK,
		0x00);		
	snd_soc_write(codec, M98095_037_DAI2_TDM,
		0x00);		
	snd_soc_write(codec, M98095_038_DAI2_FILTERS,
		0x00);	*/
	snd_soc_write(codec, M98095_03B_DAI3_CLKMODE,
		0x00);				
	snd_soc_write(codec, M98095_03C_DAI3_CLKCFG_HI,
		0x00);
	snd_soc_write(codec, M98095_03D_DAI3_CLKCFG_LO,
		0x00);			
	snd_soc_write(codec, M98095_03E_DAI3_FORMAT,
		 0x00);	
	snd_soc_write(codec, M98095_03F_DAI3_CLOCK,
		 0x00);				
	snd_soc_write(codec, M98095_041_DAI3_TDM,
		 0x00);				
	snd_soc_write(codec, M98095_042_DAI3_FILTERS,
		0x00);					
	/*codec register config End*/
#endif
}

static int tegra_bt_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	struct snd_soc_card *card = rtd->card;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);
	int err, srate, mclk, min_mclk, sample_size;
	printk("[bt_voice]	tegra_bt_hw_params in\n");
	printk("[bt_voice]	tegra_bt_hw_params card is %s\n",card->name);
	printk("[bt_voice]	tegra_bt_hw_params codec is %s\n",rtd->codec->name);
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_size = 16;
		break;
	default:
		return -EINVAL;
	}

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 64 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	err = snd_soc_dai_set_fmt(rtd->cpu_dai,
			SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		dev_err(rtd->codec->card->dev, "cpu_dai fmt not set\n");
		return err;
	}

	printk("[bt_voice]	tegra_bt_hw_params i2s->dam_ifc =%d \n",i2s->dam_ifc);


	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		tegra_max98095_set_dam_cif(i2s->dam_ifc, params_rate(params),
			params_channels(params), sample_size, 0, 0, 0, 0);
	printk("[bt_voice]	tegra_bt_hw_params out\n");
	return 0;
}
static int tegra_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

	return 0;
}

static struct snd_soc_ops tegra_max98095_ops = {
	.hw_params = tegra_max98095_hw_params,
	.hw_free = tegra_hw_free,
};

#if 0
static struct snd_soc_ops tegra_max98095_bt_sco_ops = {
	.hw_params = tegra_bt_sco_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_ops tegra_spdif_ops = {
	.hw_params = tegra_spdif_hw_params,
	.hw_free = tegra_hw_free,
};
#endif

static struct snd_soc_ops tegra_voice_call_ops = {
	.hw_params = tegra_voice_call_hw_params,
	.shutdown = tegra_voice_call_shutdown,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_ops tegra_bt_voice_call_ops = {
	.hw_params = tegra_bt_voice_call_hw_params,
	.shutdown = tegra_bt_voice_call_shutdown,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_ops tegra_bt_ops = {
	.hw_params = tegra_bt_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_jack tegra_max98095_hp_jack;
static struct snd_soc_jack tegra_max98095_mic_jack;

enum headset_state hp_state ;
long hp_time=0;
long mic_time=0;
	
static struct snd_soc_jack_gpio tegra_max98095_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
	.invert = 1,
};


#ifdef CONFIG_SWITCH
/* These values are copied from Android WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

static struct switch_dev tegra_max98095_headset_switch = {
	.name = "h2w",
};

static int tegra_max98095_jack_notifier(struct notifier_block *self,
			      unsigned long action, void *dev)
{
	volatile int reg_val = 0;
	struct snd_soc_jack *jack = dev;
	struct snd_soc_codec *codec = jack->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);
	enum headset_state state = BIT_NO_HEADSET;

	if (jack == &tegra_max98095_hp_jack) {
		machine->jack_status &= ~SND_JACK_HEADPHONE;
		machine->jack_status |= (action & SND_JACK_HEADPHONE);
	} else {
		machine->jack_status &= ~SND_JACK_MICROPHONE;
		machine->jack_status |= (action & SND_JACK_MICROPHONE);
	}
	/*reset the detect circuit*/
	snd_soc_write(codec, M98095_089_JACK_DET_AUTO, 0);
	snd_soc_write(codec, M98095_089_JACK_DET_AUTO, M98095_JDEN|M98095_KEYEN|M98095_PIN5EN);
	if(machine->jack_status){
		/*clear the  status register*/
		mdelay(500);
		reg_val = codec->hw_read(codec, M98095_007_JACK_AUTO_STS);
		printk("[hp]M98095_007_JACK_AUTO_STS 1 reg_val :0x%x\n",reg_val);
		/**re-enable the detect circuit*/
		snd_soc_write(codec, M98095_089_JACK_DET_AUTO, 0x0c);
		snd_soc_write(codec, M98095_089_JACK_DET_AUTO, M98095_JDEN|M98095_KEYEN|M98095_PIN5EN);
		/*read the state register*/
		mdelay(100);
		reg_val = codec->hw_read(codec, M98095_007_JACK_AUTO_STS);
		reg_val = codec->hw_read(codec, M98095_007_JACK_AUTO_STS);
		printk("[hp]M98095_007_JACK_AUTO_STS 2 reg_val :0x%x\n",reg_val);
		if((0xc0 == reg_val)||(0xa0 == reg_val)){
			state = BIT_HEADSET_NO_MIC;
			printk("HEADSET_WITHOUT_MIC\n");
		}else{
			state = BIT_HEADSET;
			printk("HEADSET_MIC\n");
		}
		snd_soc_write(codec, M98095_013_JACK_INT_EN, 0x03);	
	}else{
		state = BIT_NO_HEADSET;
		snd_soc_write(codec, M98095_013_JACK_INT_EN, 0x00);	
	}
	switch_set_state(&tegra_max98095_headset_switch, state);

	hp_time = jiffies /HZ;	
	printk("[hp]tegra_max98095_jack_notifier state :%d,hp_time :%ld\n",state,hp_time);

	return NOTIFY_OK;
}

static struct notifier_block tegra_max98095_jack_detect_nb = {
	.notifier_call = tegra_max98095_jack_notifier,
};

static irqreturn_t tegra_max98095_mic_irq(int irq, void *data)
{
	//struct snd_soc_jack *jack = dev;
	struct snd_soc_codec *codec = data;
	struct snd_soc_card *card = codec->card;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_max98095_platform_data *pdata = machine->pdata;
	int ret = 0;
	int hp_status = 0;
	int time=0;
	mic_time = jiffies /HZ;
	time= mic_time -hp_time;
	printk("[audiomic]tegra_max98095_mic_irq mic_time :%ld,time=%d\n",mic_time,time);
 
	codec->hw_read(codec, M98095_000_HOST_DATA);
	snd_soc_write(codec, M98095_089_JACK_DET_AUTO, 0);
	//snd_soc_write(codec, M98095_089_JACK_DET_AUTO, M98095_KEYEN|M98095_PIN5EN);
	snd_soc_write(codec, M98095_089_JACK_DET_AUTO, M98095_JDEN|M98095_KEYEN|M98095_PIN5EN);

	mdelay(100);
	ret = codec->hw_read(codec, M98095_007_JACK_AUTO_STS);
	printk("reg[M98095_007_JACK_AUTO_STS] = 0x%02X\n", ret);
	hp_status = gpio_get_value(pdata->gpio_hp_det);
	if ((0x03 == (ret&0x03))&&(0 == hp_status)&&(time>=2)){
		ret = 0;
		printk( "hook button press down\n");
		input_report_key(pdata->idev, KEY_MEDIA, 1);
		mdelay(50);
		input_report_key(pdata->idev, KEY_MEDIA, 0);
		printk( "input_report_key is ok\n");
		input_sync(pdata->idev);
	}
	return IRQ_HANDLED;
}

#else
static struct snd_soc_jack_pin tegra_max98095_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static struct snd_soc_jack_pin tegra_max98095_mic_jack_pins[] = {
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
};
#endif


#if 0
static int tegra_max98095_event_int_spk(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_max98095_platform_data *pdata = machine->pdata;

	if (machine->spk_reg) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			regulator_enable(machine->spk_reg);
		else
			regulator_disable(machine->spk_reg);
	}

	if (!(machine->gpio_requested & GPIO_SPKR_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_spkr_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_max98095_event_hp(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_max98095_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_HP_MUTE))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_hp_mute,
				!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_max98095_event_int_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_max98095_platform_data *pdata = machine->pdata;

	if (machine->dmic_reg) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			regulator_enable(machine->dmic_reg);
		else
			regulator_disable(machine->dmic_reg);
	}

	if (!(machine->gpio_requested & GPIO_INT_MIC_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_int_mic_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_max98095_event_ext_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_max98095_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_EXT_MIC_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_ext_mic_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}
#endif

static const struct snd_soc_dapm_widget cardhu_dapm_widgets[] = {
#if 0
	SND_SOC_DAPM_SPK("Int Spk", tegra_max98095_event_int_spk),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_max98095_event_hp),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", tegra_max98095_event_ext_mic),
	SND_SOC_DAPM_MIC("Int Mic", tegra_max98095_event_int_mic),
	SND_SOC_DAPM_LINE("Line In", NULL),
#else
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_HP("Headset Out", NULL),
	SND_SOC_DAPM_MIC("Headset In", NULL),
	SND_SOC_DAPM_SPK("Receiver", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Mainmic", NULL),
	SND_SOC_DAPM_MIC("Slavemic", NULL),
#endif
};


#if 0 
static const struct snd_soc_dapm_widget tegra_max98095_default_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", tegra_max98095_event_int_spk),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_max98095_event_hp),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static const struct snd_soc_dapm_route harmony_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1L", NULL, "Mic Bias"},
};
#endif 

static const struct snd_soc_dapm_route cardhu_audio_map[] = {
#if 0
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Line Out", NULL, "LINEOUTL"},
	{"Line Out", NULL, "LINEOUTR"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1L", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Int Mic"},
	{"IN1L", NULL, "Mic Bias"},
	{"IN1R", NULL, "Mic Bias"},
	{"IN3L", NULL, "Line In"},
	{"IN3R", NULL, "Line In"},
#else
	/* Headphone connected to HPL and HPR */
	{"Headphone", NULL, "HPL"},
	{"Headphone", NULL, "HPR"},

	/* headset Jack  - in = micin, out = HPOUT*/
	{"Headset Out", NULL, "HPL"},
	{"Headset Out", NULL, "HPR"},

        /*Speaker*/
	 {"Speaker", NULL, "SPKL"},

        /*Receiver*/
        {"Receiver", NULL, "RCV"},
		
       /*main mic*/
       {"MIC1", NULL, "Mainmic"},
       /*Slave mic*/
       {"MIC2", NULL, "Slavemic"},
	
       /*Headset In*/
	{"MIC1", NULL, "Headset In"},        
#endif
};

#if 0 
static const struct snd_soc_dapm_route seaboard_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1R", NULL, "Mic Bias"},
};
 
static const struct snd_soc_dapm_route kaen_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN2R", NULL, "Mic Bias"},
};

static const struct snd_soc_dapm_route aebl_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "LINEOUTR"},
	{"Int Spk", NULL, "LINEOUTL"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1R", NULL, "Mic Bias"},
};
#endif 

static const struct snd_kcontrol_new cardhu_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("LineOut"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("LineIn"),
};

#if 0 
static const struct snd_kcontrol_new tegra_max98095_default_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
};
#endif 

static int tegra_max98095_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_max98095_platform_data *pdata = machine->pdata;
	int ret;
	printk("[snd card]  tegra_max98095_init \n");
	#if 0
	if (gpio_is_valid(pdata->gpio_spkr_en)) {
		ret = gpio_request(pdata->gpio_spkr_en, "spkr_en");
		if (ret) {
			dev_err(card->dev, "cannot get spkr_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_SPKR_EN;

		gpio_direction_output(pdata->gpio_spkr_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_hp_mute)) {
		ret = gpio_request(pdata->gpio_hp_mute, "hp_mute");
		if (ret) {
			dev_err(card->dev, "cannot get hp_mute gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_HP_MUTE;

		gpio_direction_output(pdata->gpio_hp_mute, 0);
	}

	if (gpio_is_valid(pdata->gpio_int_mic_en)) {
		ret = gpio_request(pdata->gpio_int_mic_en, "int_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get int_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_INT_MIC_EN;

		/* Disable int mic; enable signal is active-high */
		gpio_direction_output(pdata->gpio_int_mic_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_ext_mic_en)) {
		ret = gpio_request(pdata->gpio_ext_mic_en, "ext_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get ext_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_EXT_MIC_EN;

		/* Enable ext mic; enable signal is active-low */
		gpio_direction_output(pdata->gpio_ext_mic_en, 0);
	}
#endif
#if 0	
if (machine_is_cardhu() || machine_is_ventana()) {
		ret = snd_soc_add_controls(codec, cardhu_controls,
				ARRAY_SIZE(cardhu_controls));
		if (ret < 0)
			return ret;

		snd_soc_dapm_new_controls(dapm, cardhu_dapm_widgets,
				ARRAY_SIZE(cardhu_dapm_widgets));
	}
	else {
		ret = snd_soc_add_controls(codec,
				tegra_max98095_default_controls,
				ARRAY_SIZE(tegra_max98095_default_controls));
		if (ret < 0)
			return ret;

		snd_soc_dapm_new_controls(dapm,
				tegra_max98095_default_dapm_widgets,
				ARRAY_SIZE(tegra_max98095_default_dapm_widgets));
	}
#else
	ret = snd_soc_add_controls(codec, cardhu_controls,
			ARRAY_SIZE(cardhu_controls));
	if (ret < 0)
		return ret;

	snd_soc_dapm_new_controls(dapm, cardhu_dapm_widgets,
			ARRAY_SIZE(cardhu_dapm_widgets));

#endif 



#if 0
	if (machine_is_harmony()) {
		snd_soc_dapm_add_routes(dapm, harmony_audio_map,
					ARRAY_SIZE(harmony_audio_map));
	} else if (machine_is_cardhu() || machine_is_ventana()) {
		snd_soc_dapm_add_routes(dapm, cardhu_audio_map,
					ARRAY_SIZE(cardhu_audio_map));
	} else if (machine_is_seaboard()) {
		snd_soc_dapm_add_routes(dapm, seaboard_audio_map,
					ARRAY_SIZE(seaboard_audio_map));
	} else if (machine_is_kaen()) {
		snd_soc_dapm_add_routes(dapm, kaen_audio_map,
					ARRAY_SIZE(kaen_audio_map));
	} else {
		snd_soc_dapm_add_routes(dapm, aebl_audio_map,
					ARRAY_SIZE(aebl_audio_map));
	}
#else
	snd_soc_dapm_add_routes(dapm, cardhu_audio_map,
				ARRAY_SIZE(cardhu_audio_map));

#endif 

	if (gpio_is_valid(pdata->gpio_hp_det)) {
		tegra_max98095_hp_jack_gpio.gpio = pdata->gpio_hp_det;
		snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE,
				&tegra_max98095_hp_jack);
#ifndef CONFIG_SWITCH
		snd_soc_jack_add_pins(&tegra_max98095_hp_jack,
					ARRAY_SIZE(tegra_max98095_hp_jack_pins),
					tegra_max98095_hp_jack_pins);
#else
		snd_soc_jack_notifier_register(&tegra_max98095_hp_jack,
					&tegra_max98095_jack_detect_nb);
#endif
		snd_soc_jack_add_gpios(&tegra_max98095_hp_jack,
					1,
					&tegra_max98095_hp_jack_gpio);
		machine->gpio_requested |= GPIO_HP_DET;

		ret = enable_irq_wake(TEGRA_GPIO_TO_IRQ(pdata->gpio_hp_det));
	        if (ret) {
			pr_err("Could NOT set up hp_det gpio pins for wakeup the AP.\n");
	        }
	}

	if (gpio_is_valid(pdata->gpio_mic_det)) {
		ret= request_threaded_irq(TEGRA_GPIO_TO_IRQ(pdata->gpio_mic_det), NULL, tegra_max98095_mic_irq,
					IRQF_TRIGGER_FALLING, "HOOK DETECT ", codec);
		if (ret < 0){
			printk("max98095_hook_init fail!\n");
		}

		max98095_mic_detect(codec, &tegra_max98095_mic_jack, SND_JACK_MICROPHONE,0);

		pdata->idev = kzalloc(sizeof (struct input_dev), GFP_KERNEL);
		pdata->idev = input_allocate_device();

		input_set_capability(pdata->idev, EV_KEY, KEY_MEDIA);

		ret= input_register_device(pdata->idev);
		if (ret < 0){
			printk("[audio]max98095 input_register_device fail\n");
		}

		ret = enable_irq_wake(TEGRA_GPIO_TO_IRQ(pdata->gpio_mic_det));
	        if (ret) {
			pr_err("Could NOT set up mic_det gpio pins for wakeup the AP.\n");
	        }
	}
	printk("[audio]snd_ctl_add in ");
	       /* Add call mode switch control */
	ret = snd_ctl_add(codec->card->snd_card,
			snd_ctl_new1(&tegra_call_mode_control, machine));
	if (ret < 0)
		return ret;
	printk("[audio]snd_ctl_add  ret:%d ",ret );

	snd_soc_dapm_force_enable_pin(dapm, "Mic Bias");

	/* FIXME: Calculate automatically based on DAPM routes? */
#if 0 
	if (!machine_is_harmony() && !machine_is_ventana() &&
	    !machine_is_cardhu())
		snd_soc_dapm_nc_pin(dapm, "IN1L");
	if (!machine_is_seaboard() && !machine_is_aebl() &&
	    !machine_is_cardhu())
		snd_soc_dapm_nc_pin(dapm, "IN1R");
	snd_soc_dapm_nc_pin(dapm, "IN2L");
	if (!machine_is_kaen())
		snd_soc_dapm_nc_pin(dapm, "IN2R");
	snd_soc_dapm_nc_pin(dapm, "IN3L");
	snd_soc_dapm_nc_pin(dapm, "IN3R");

	if (machine_is_aebl()) {
		snd_soc_dapm_nc_pin(dapm, "LON");
		snd_soc_dapm_nc_pin(dapm, "RON");
		snd_soc_dapm_nc_pin(dapm, "ROP");
		snd_soc_dapm_nc_pin(dapm, "LOP");
	} else {
		snd_soc_dapm_nc_pin(dapm, "LINEOUTR");
		snd_soc_dapm_nc_pin(dapm, "LINEOUTL");
	}
#else
	snd_soc_dapm_nc_pin(dapm, "IN2R");
	snd_soc_dapm_nc_pin(dapm, "IN2L");
	snd_soc_dapm_nc_pin(dapm, "IN3L");
	snd_soc_dapm_nc_pin(dapm, "IN3R");
	snd_soc_dapm_nc_pin(dapm, "LINEOUTR");
	snd_soc_dapm_nc_pin(dapm, "LINEOUTL");
#endif 

	snd_soc_dapm_sync(dapm);
	printk("[snd card]  tegra_max98095_init ok\n");
	return 0;
}

static struct snd_soc_dai_link tegra_max98095_dai[] = {
	{
		.name = "MAX98095",
		.stream_name = "max98095 PCM",
		.codec_name = "max98095.4-0010",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra30-i2s.1",
		.codec_dai_name = "max98095-hifi",
		.init = tegra_max98095_init,
		.ops = &tegra_max98095_ops,
	},
	{
		.name = "VOICE CALL",
		.stream_name = "VOICE CALL PCM",
		.codec_name = "max98095.4-0010",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "dit-hifi",
		.codec_dai_name = "max98095 Voice",
		.ops = &tegra_voice_call_ops,
	},
	{
		.name = "BT VOICE CALL",
		.stream_name = "BT VOICE CALL PCM",
		.codec_name = "max98095.4-0010",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra30-i2s.2",
		.codec_dai_name = "max98095 BT",
		.ops = &tegra_bt_voice_call_ops,
	},
	/*{
		.name = "BT SCO",
		.stream_name = "BT SCO PCM",
		.codec_name = "spdif-dit.1",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra30-i2s.3",
		.codec_dai_name = "dit-hifi",
		.ops = &tegra_bt_ops,
	},*/
};

static struct snd_soc_card snd_soc_tegra_max98095 = {
	.name = "tegra-max98095",
	.dai_link = tegra_max98095_dai,
	.num_links = ARRAY_SIZE(tegra_max98095_dai),
};

static __devinit int tegra_max98095_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_max98095;
	struct tegra_max98095 *machine;
	struct tegra_max98095_platform_data *pdata;
	int ret;
	int i;

	printk("[audio]  tegra_max98095_driver_probe in\n");
	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	machine = kzalloc(sizeof(struct tegra_max98095), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_max98095 struct\n");
		return -ENOMEM;
	}

	machine->pdata = pdata;

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev);
	if (ret){
		printk("[audio]  tegra_max98095_driver_probe0 ret:%d\n",ret);
		goto err_free_machine;
		}	

	machine->spk_reg = regulator_get(&pdev->dev, "vdd_spk_amp");
	if (IS_ERR(machine->spk_reg)) {
		dev_info(&pdev->dev, "No speaker regulator found\n");
		machine->spk_reg = 0;
	}

	machine->dmic_reg = regulator_get(&pdev->dev, "vdd_dmic");
	if (IS_ERR(machine->dmic_reg)) {
		dev_info(&pdev->dev, "No digital mic regulator found\n");
		machine->dmic_reg = 0;
	}
#if 0
	if (machine_is_cardhu()) {
		tegra_max98095_dai[0].codec_name = "max98095.4-0010",
		tegra_max98095_dai[0].cpu_dai_name = "tegra30-i2s.1";
		//tegra_max98095_dai[1].cpu_dai_name = "tegra30-spdif";
		tegra_max98095_dai[2].cpu_dai_name = "tegra30-i2s.3";
	}
#endif

#ifdef CONFIG_SWITCH
	/* Addd h2w swith class support */
	ret = switch_dev_register(&tegra_max98095_headset_switch);
	if (ret < 0){
		printk("[audio]  tegra_max98095_driver_probe1 ret:%d\n",ret);
		goto err_fini_utils;
		}	
#endif

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	for (i = 0; i < NUM_I2S_DEVICES ; i++)
	machine->codec_info[i].i2s_id = pdata->audio_port_id[i];

	ret = snd_soc_register_card(card);
	if (ret) {
		printk("[audio]  tegra_max98095_driver_probe2 ret:%d\n",ret);
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_unregister_switch;
	}
	printk("[audio]  tegra_max98095_driver_probe ok\n");
	return 0;

err_unregister_switch:
#ifdef CONFIG_SWITCH
	switch_dev_unregister(&tegra_max98095_headset_switch);
#endif
err_fini_utils:
	tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:
	kfree(machine);
	return ret;
}

static int __devexit tegra_max98095_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_max98095 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_max98095_platform_data *pdata = machine->pdata;

	if (machine->gpio_requested & GPIO_HP_DET)
		snd_soc_jack_free_gpios(&tegra_max98095_hp_jack,
					1,
					&tegra_max98095_hp_jack_gpio);
	if (machine->gpio_requested & GPIO_EXT_MIC_EN)
		gpio_free(pdata->gpio_ext_mic_en);
	if (machine->gpio_requested & GPIO_INT_MIC_EN)
		gpio_free(pdata->gpio_int_mic_en);
	if (machine->gpio_requested & GPIO_HP_MUTE)
		gpio_free(pdata->gpio_hp_mute);
	if (machine->gpio_requested & GPIO_SPKR_EN)
		gpio_free(pdata->gpio_spkr_en);
	machine->gpio_requested = 0;

	if (machine->spk_reg)
		regulator_put(machine->spk_reg);
	if (machine->dmic_reg)
		regulator_put(machine->dmic_reg);

	snd_soc_unregister_card(card);

	tegra_asoc_utils_fini(&machine->util_data);

#ifdef CONFIG_SWITCH
	switch_dev_unregister(&tegra_max98095_headset_switch);
#endif
	kfree(machine);

	return 0;
}

static struct platform_driver tegra_max98095_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = tegra_max98095_driver_probe,
	.remove = __devexit_p(tegra_max98095_driver_remove),
};

static int __init tegra_max98095_modinit(void)
{
	return platform_driver_register(&tegra_max98095_driver);
}
module_init(tegra_max98095_modinit);

static void __exit tegra_max98095_modexit(void)
{
	platform_driver_unregister(&tegra_max98095_driver);
}
module_exit(tegra_max98095_modexit);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra+MAX98095 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);

