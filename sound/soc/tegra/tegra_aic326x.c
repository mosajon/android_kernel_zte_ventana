/*
 * tegra_aic326x.c - Tegra machine ASoC driver for boards using TI 3262 codec.
 *
 * Author: Vinod G. <vinodg@nvidia.com>
 * Copyright (C) 2011 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * (c) 2010, 2011 Nvidia Graphics Pvt. Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
// #define DEBUG

#include <asm/mach-types.h>

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#include <mach/tegra_aic326x_pdata.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../codecs/tlv320aic326x/tlv320aic326x.h"

#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#include "tegra20_das.h"
#else
#include "tegra30_ahub.h"
#include "tegra30_i2s.h"
#include "tegra30_dam.h"
#endif
#include <mach/gpio.h>
#include "../../../arch/arm/mach-tegra/board-ventana.h"
#include "../../../arch/arm/mach-tegra/gpio-names.h"


#define DRV_NAME "tegra-snd-aic326x"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)

#define DAI_LINK_HIFI           0
#define DAI_LINK_SPDIF          1
#define DAI_LINK_BTSCO          2
#define DAI_LINK_VOICE_CALL     3
#define DAI_LINK_BT_VOICE_CALL  4
#define NUM_DAI_LINKS           5

#define HEAD_DET_GPIO_INDEX 0
#define CDC_IRQ_INDEX 1

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
const char *tegra_aic326x_i2s_dai_name[TEGRA30_NR_I2S_IFC] = {
    "tegra30-i2s.0",
    "tegra30-i2s.1",
    "tegra30-i2s.2",
    "tegra30-i2s.3",
    "tegra30-i2s.4",
};
#endif

struct tegra_aic326x {
    struct tegra_asoc_utils_data util_data;
    struct tegra_aic326x_platform_data *pdata;
    struct regulator *audio_reg;
    int gpio_requested;
    bool init_done;
    int is_call_mode;
    int is_device_bt;
	int fm_mode;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
    struct codec_config codec_info[NUM_I2S_DEVICES];
#endif
	struct snd_soc_card *pcard;
};

static bool is_in_call_state = false;
bool in_call_state(void)
{
	return is_in_call_state;
}
EXPORT_SYMBOL(in_call_state);

//ZTE: bt voice and call switch pin
static void audio_codec_i2s_enable_pin(bool enable)
{
        if (enable)
        {
            gpio_set_value(CODEC_I2S_EN_PIN, 1);
        }
        else
        {
            gpio_set_value(CODEC_I2S_EN_PIN, 0);
        }
}
static int tegra_aic326x_call_mode_info(struct snd_kcontrol *kcontrol,
            struct snd_ctl_elem_info *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}

static int tegra_aic326x_call_mode_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    struct tegra_aic326x *machine = snd_kcontrol_chip(kcontrol);

    ucontrol->value.integer.value[0] = machine->is_call_mode;

    return 0;
}

static int tegra_aic326x_call_mode_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    struct tegra_aic326x *machine = snd_kcontrol_chip(kcontrol);
    int is_call_mode_new = ucontrol->value.integer.value[0];
    int codec_index;
    unsigned int i;

    if (machine->is_call_mode == is_call_mode_new)
        return 0;

    if (machine->is_device_bt)
        codec_index = BT_SCO;
    else
        codec_index = HIFI_CODEC;

    if (is_call_mode_new) {

        for(i=0; i<machine->pcard->num_links; i++)
			machine->pcard->dai_link[i].ignore_suspend = 1;
		is_in_call_state = true;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
        if (machine->codec_info[codec_index].rate == 0 ||
            machine->codec_info[codec_index].channels == 0)
                return -EINVAL;

        tegra30_make_voice_call_connections(
            &machine->codec_info[codec_index],
            &machine->codec_info[BASEBAND]);
#endif
    } else {
        for(i=0; i<machine->pcard->num_links; i++)
			machine->pcard->dai_link[i].ignore_suspend = 0;
		is_in_call_state = false;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
        tegra30_break_voice_call_connections(
            &machine->codec_info[codec_index],
            &machine->codec_info[BASEBAND]);
#endif
    }

    machine->is_call_mode = is_call_mode_new;

    return 1;
}

struct snd_kcontrol_new tegra_aic326x_call_mode_control = {
    .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
    .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
    .name = "Call Mode Switch",
    .private_value = 0xffff,
    .info = tegra_aic326x_call_mode_info,
    .get = tegra_aic326x_call_mode_get,
    .put = tegra_aic326x_call_mode_put
};
static int tegra_aic326x_fm_mode_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int tegra_aic326x_fm_mode_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_aic326x *machine = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = machine->fm_mode;

	return 0;
}

static int tegra_aic326x_fm_mode_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_aic326x *machine = snd_kcontrol_chip(kcontrol);
	int fm_mode = ucontrol->value.integer.value[0];
	unsigned int i;

	if (fm_mode != machine->fm_mode) {
		if (fm_mode) {
			machine->fm_mode = 1;
			for (i = 0; i < machine->pcard->num_links; i++)
				machine->pcard->dai_link[i].ignore_suspend = 1;
		} else {
			machine->fm_mode = 0;
			for (i = 0; i < machine->pcard->num_links; i++)
				machine->pcard->dai_link[i].ignore_suspend = 0;
		}
	}
	return 0;
}

struct snd_kcontrol_new tegra_aic326x_fm_mode_control = {
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "FM Mode Switch",
	.private_value = 0xffff,
	.info = tegra_aic326x_fm_mode_info,
	.get = tegra_aic326x_fm_mode_get,
	.put = tegra_aic326x_fm_mode_put
};

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static int tegra_aic326x_set_dam_cif(int dam_ifc, int srate,
            int channels, int bit_size)
{
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

    return 0;
}
#endif

static int tegra_aic326x_get_mclk(int srate)
{
    int mclk = 0;
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
        mclk = -EINVAL;
        break;
    }

    return mclk;
}

static int tegra_aic326x_hw_params(struct snd_pcm_substream *substream,
                    struct snd_pcm_hw_params *params)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *codec_dai = rtd->codec_dai;
    struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
    struct snd_soc_codec *codec = rtd->codec;
    struct snd_soc_card *card = codec->card;
    struct tegra_aic326x *machine = snd_soc_card_get_drvdata(card);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
    struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
#endif
    int srate, mclk, sample_size, daifmt;
    int err;

    //dev_dbg(card->dev, "param: %s\n", (char *)params);
    switch (params_format(params)) {
    case SNDRV_PCM_FORMAT_S16_LE:
        sample_size = 16;
        break;
    default:
        return -EINVAL;
    }

    srate = params_rate(params);
    dev_dbg(card->dev, "rate: %d\n", srate);
    mclk = tegra_aic326x_get_mclk(srate);
    if (mclk < 0)
        return mclk;

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

    daifmt = SND_SOC_DAIFMT_I2S |	SND_SOC_DAIFMT_NB_NF |
                    SND_SOC_DAIFMT_CBS_CFS;

    err = snd_soc_dai_set_fmt(codec_dai, daifmt);
    if (err < 0) {
        dev_err(card->dev, "codec_dai fmt not set\n");
        return err;
    }

    err = snd_soc_dai_set_fmt(cpu_dai, daifmt);
    if (err < 0) {
        dev_err(card->dev, "cpu_dai fmt not set\n");
        return err;
    }
    //zuoziqiang test
    mclk = 12000000;
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
#else
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        tegra_aic326x_set_dam_cif(i2s->dam_ifc, srate,
                params_channels(params), sample_size);
#endif

    return 0;
}

static int tegra_aic326x_spdif_hw_params(struct snd_pcm_substream *substream,
                    struct snd_pcm_hw_params *params)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_card *card = rtd->card;
    struct tegra_aic326x *machine = snd_soc_card_get_drvdata(card);
    int srate, mclk, min_mclk;
    int err;

    srate = params_rate(params);

    mclk = tegra_aic326x_get_mclk(srate);
    if (mclk < 0)
        return mclk;

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
#if 0 //not used, to remove warning
static int tegra_aic326x_bt_hw_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    #ifndef CONFIG_ARCH_TEGRA_2x_SOC
    struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
    #endif
    struct snd_soc_card *card = rtd->card;
    struct tegra_aic326x *machine = snd_soc_card_get_drvdata(card);
    int err, srate, mclk, min_mclk, sample_size;

    switch (params_format(params)) {
    case SNDRV_PCM_FORMAT_S16_LE:
        sample_size = 16;
        break;
    default:
        return -EINVAL;
    }

    srate = params_rate(params);

    mclk = tegra_aic326x_get_mclk(srate);
    if (mclk < 0)
        return mclk;

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

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        tegra_aic326x_set_dam_cif(i2s->dam_ifc, params_rate(params),
                params_channels(params), sample_size);
#endif

    return 0;
}
#endif

static int tegra_aic326x_hw_free(struct snd_pcm_substream *substream)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct tegra_aic326x *machine = snd_soc_card_get_drvdata(rtd->card);

    tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

    return 0;
}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static int tegra_aic326x_startup(struct snd_pcm_substream *substream)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
    struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);

    if ((substream->stream != SNDRV_PCM_STREAM_PLAYBACK) ||
        !(i2s->is_dam_used))
        return 0;

    /*dam configuration*/
    if (!i2s->dam_ch_refcount)
        i2s->dam_ifc = tegra30_dam_allocate_controller();

    tegra30_dam_allocate_channel(i2s->dam_ifc, TEGRA30_DAM_CHIN1);
    i2s->dam_ch_refcount++;
    tegra30_dam_enable_clock(i2s->dam_ifc);
    tegra30_dam_set_gain(i2s->dam_ifc, TEGRA30_DAM_CHIN1, 0x1000);

    tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
            (i2s->dam_ifc*2), i2s->txcif);

    /*
    *make the dam tx to i2s rx connection if this is the only client
    *using i2s for playback
    */
    if (i2s->playback_ref_count == 1)
        tegra30_ahub_set_rx_cif_source(
            TEGRA30_AHUB_RXCIF_I2S0_RX0 + i2s->id,
            TEGRA30_AHUB_TXCIF_DAM0_TX0 + i2s->dam_ifc);

    /* enable the dam*/
    tegra30_dam_enable(i2s->dam_ifc, TEGRA30_DAM_ENABLE,
            TEGRA30_DAM_CHIN1);

    return 0;
}

static void tegra_aic326x_shutdown(struct snd_pcm_substream *substream)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
    struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);

    if ((substream->stream != SNDRV_PCM_STREAM_PLAYBACK) ||
        !(i2s->is_dam_used))
        return;

    /* disable the dam*/
    tegra30_dam_enable(i2s->dam_ifc, TEGRA30_DAM_DISABLE,
            TEGRA30_DAM_CHIN1);

    /* disconnect the ahub connections*/
    tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
                (i2s->dam_ifc*2));

    /* disable the dam and free the controller */
    tegra30_dam_disable_clock(i2s->dam_ifc);
    tegra30_dam_free_channel(i2s->dam_ifc, TEGRA30_DAM_CHIN1);
    i2s->dam_ch_refcount--;
    if (!i2s->dam_ch_refcount)
        tegra30_dam_free_controller(i2s->dam_ifc);
}
#endif
static int tegra_aic326x_voice_call_hw_params(
            struct snd_pcm_substream *substream,
            struct snd_pcm_hw_params *params)
    {
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *codec_dai = rtd->codec_dai;
    struct snd_soc_codec *codec = rtd->codec;
    struct snd_soc_card *card = codec->card;
    struct tegra_aic326x *machine = snd_soc_card_get_drvdata(card);
    // struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
    int dai_flag = 0, mclk;
	//int srate;
    int err;

    pr_debug(KERN_INFO "#%s Invoked..\n", __func__);
    #if 0
    srate = params_rate(params);
    mclk = tegra_aic326x_get_mclk(srate);
    if (mclk < 0)
        return mclk;
    #else
    mclk = 12000000;
    #endif
    //ZTE: modify by hn, codec is the master, begin
      dai_flag |= SND_SOC_DAIFMT_CBM_CFM;
    //ZTE: modify by hn, codec is the master, end

    /* We are supporting DSP and I2s format for now */
      dai_flag |= SND_SOC_DAIFMT_DSP_B;


    pr_debug("aic326x_voice_hw_params : dai_flag = %d \n", dai_flag);

    pr_debug("codec_dai->name = %s \n", codec_dai->name);
    err = snd_soc_dai_set_fmt(codec_dai, dai_flag);
    if (err < 0) {
        pr_err("codec_dai fmt not set\n");
        return err;
    }

    //is this needed?
    #if 0
    pr_debug("cpu_dai->name = %s \n", cpu_dai->name);
    err = snd_soc_dai_set_fmt(cpu_dai, dai_flag);
    if (err < 0) {
        pr_err("cpu_dai fmt not set\n");
        return err;
    }
    #endif

    err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
                    SND_SOC_CLOCK_IN);
    if (err < 0) {
        pr_err("cpu_dai clock not set\n");
        return err;
    }

     //is this needed? zuoziqiang ask
    #if 0
    err = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk, SND_SOC_CLOCK_IN);
    if (err < 0) {
        pr_err("cpu_dai clock not set\n");
        return err;
    }
    #endif
    //ZTE: bt voice and call switch pin
    printk("[codec] tegra_aic326x_voice_call_hw_params\n");
    //enable the level shifter and wsith from modem to codec
    audio_codec_i2s_enable_pin(1);

    machine->is_device_bt = 0;
    pr_debug("exit %s  succefully\n", __func__);
    return 0;
}
static void tegra_aic326x_voice_call_shutdown(
                    struct snd_pcm_substream *substream)
{
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct tegra_aic326x *machine  =
            snd_soc_card_get_drvdata(rtd->codec->card);

    machine->codec_info[HIFI_CODEC].rate = 0;
    machine->codec_info[HIFI_CODEC].channels = 0;
#endif
    //ZTE: bt voice and call switch pin
    printk("[codec] tegra_aic326x_voice_call_shutdown\n");
    audio_codec_i2s_enable_pin(0);
}

static int tegra_aic326x_bt_voice_call_hw_params(
            struct snd_pcm_substream *substream,
            struct snd_pcm_hw_params *params)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
    struct tegra_aic326x *machine = snd_soc_card_get_drvdata(card);
	//struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int dai_flag = 0, mclk;
	//int srate;
	int err;

	printk(KERN_INFO "#%s Invoked..\n", __func__);
#if 0
    srate = params_rate(params);
    mclk = tegra_aic326x_get_mclk(srate);
    if (mclk < 0)
        return mclk;
	#else
	mclk = 12000000;
	#endif
       printk("mclk is %d\n", mclk);
       //format?

	dai_flag |= SND_SOC_DAIFMT_CBM_CFM;

	/* We are supporting DSP and I2s format for now */
	  dai_flag |= SND_SOC_DAIFMT_DSP_A;


      printk("tegra_aic326x_bt_voice_call_hw_params : dai_flag = %d \n", dai_flag);

      printk("codec_dai->name = %s \n", codec_dai->name);
	err = snd_soc_dai_set_fmt(codec_dai, dai_flag);
    if (err < 0) {
		pr_err("codec_dai fmt not set\n");
            return err;
        }
      printk("snd_soc_dai_set_sysclk\n");
   	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err("cpu_dai clock not set\n");
		return err;
    }
	printk("set is_device_bt to 1\n");
    machine->is_device_bt = 1;
	printk("exit %s  succefully\n", __func__);
    return 0;
}

static void tegra_aic326x_bt_voice_call_shutdown(
                struct snd_pcm_substream *substream)
{
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct tegra_aic326x *machine  =
            snd_soc_card_get_drvdata(rtd->codec->card);

    machine->codec_info[BT_SCO].rate = 0;
    machine->codec_info[BT_SCO].channels = 0;
#endif
    //ZTE: bt voice and call switch pin
   printk("[codec] tegra_aic326x_bt_voice_call_shutdown\n");

}

static struct snd_soc_ops tegra_aic326x_hifi_ops = {
    .hw_params = tegra_aic326x_hw_params,
    .hw_free = tegra_aic326x_hw_free,
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
    .startup = tegra_aic326x_startup,
    .shutdown = tegra_aic326x_shutdown,
#endif
};

static struct snd_soc_ops tegra_aic326x_spdif_ops = {
    .hw_params = tegra_aic326x_spdif_hw_params,
    .hw_free = tegra_aic326x_hw_free,
};

static struct snd_soc_ops tegra_aic326x_voice_call_ops = {
    .hw_params = tegra_aic326x_voice_call_hw_params,
    .shutdown = tegra_aic326x_voice_call_shutdown,
    .hw_free = tegra_aic326x_hw_free,
};

static struct snd_soc_ops tegra_aic326x_bt_voice_call_ops = {
    .hw_params = tegra_aic326x_bt_voice_call_hw_params,
    .shutdown = tegra_aic326x_bt_voice_call_shutdown,
    .hw_free = tegra_aic326x_hw_free,
};
#if 0
static struct snd_soc_ops tegra_aic326x_bt_ops = {
    .hw_params = tegra_aic326x_bt_hw_params,
    .hw_free = tegra_aic326x_hw_free,
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
    .startup = tegra_aic326x_startup,
    .shutdown = tegra_aic326x_shutdown,
#endif
};
#endif

static int tegra_aic326x_event_int_spk(struct snd_soc_dapm_widget *w,
                    struct snd_kcontrol *k, int event)
{
    struct snd_soc_dapm_context *dapm = w->dapm;
    struct snd_soc_card *card = dapm->card;
    struct tegra_aic326x *machine = snd_soc_card_get_drvdata(card);
    struct tegra_aic326x_platform_data *pdata = machine->pdata;

    dev_dbg(dapm->dev, "%s", __func__);
    if (!(machine->gpio_requested & GPIO_SPKR_EN))
        return 0;

    gpio_set_value_cansleep(pdata->gpio_spkr_en,
                SND_SOC_DAPM_EVENT_ON(event));

    return 0;
}

static int tegra_aic326x_event_hp(struct snd_soc_dapm_widget *w,
                struct snd_kcontrol *k, int event)
{
    struct snd_soc_dapm_context *dapm = w->dapm;
    struct snd_soc_card *card = dapm->card;
    struct tegra_aic326x *machine = snd_soc_card_get_drvdata(card);
    struct tegra_aic326x_platform_data *pdata = machine->pdata;

    if (!(machine->gpio_requested & GPIO_HP_MUTE))
        return 0;

    gpio_set_value_cansleep(pdata->gpio_hp_mute,
                !SND_SOC_DAPM_EVENT_ON(event));

    return 0;
}

static const struct snd_soc_dapm_widget tegra_aic326x_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", tegra_aic326x_event_int_spk),
	SND_SOC_DAPM_HP("Earpiece", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_aic326x_event_hp),
	SND_SOC_DAPM_MIC("HP Mic", NULL),
	SND_SOC_DAPM_MIC("Main Mic", NULL),
	SND_SOC_DAPM_MIC("Aux Mic", NULL),
	SND_SOC_DAPM_LINE("Linein", NULL),
};

static const struct snd_soc_dapm_route aic326x_audio_map[] = {
	{"Int Spk", NULL, "SPKL"},
	{"Earpiece", NULL, "RECP"},
	{"Earpiece", NULL, "RECM"},
	{"Headphone Jack", NULL, "HPL"},
	{"Headphone Jack", NULL, "HPR"},
	//hp mic is mono
	{"IN1L", NULL, "Mic Bias Ext"},
	{"Mic Bias Ext", NULL, "HP Mic"},

	/* main mic */
	{"IN2L", NULL, "Mic Bias Int"},
	{"IN2R", NULL, "Mic Bias Int"},
	{"Mic Bias Int", NULL, "Main Mic"},

	//aux mic
	{"IN3L", NULL, "Mic Bias Int"},
	{"IN3R", NULL, "Mic Bias Int"},
	{"Mic Bias Int", NULL, "Aux Mic"},

	{"IN4L", NULL, "Linein"},
	{"IN4R", NULL, "Linein"},

};

static const struct snd_kcontrol_new tegra_aic326x_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Earpiece"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("HP Mic"),
	SOC_DAPM_PIN_SWITCH("Main Mic"),
	SOC_DAPM_PIN_SWITCH("Aux Mic"),
	SOC_DAPM_PIN_SWITCH("Linein"),
};

static int tegra_aic326x_init(struct snd_soc_pcm_runtime *rtd)
{
    struct snd_soc_codec *codec = rtd->codec;
    struct snd_soc_dapm_context *dapm = &codec->dapm;
    struct snd_soc_card *card = codec->card;
    struct tegra_aic326x *machine = snd_soc_card_get_drvdata(card);
    struct tegra_aic326x_platform_data *pdata = machine->pdata;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
    struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
#endif
    int ret;

    printk("%s \n", __func__);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
    if (machine->codec_info[BASEBAND].i2s_id != -1)
        i2s->is_dam_used = true;
#endif

    if (machine->init_done)
        return 0;

    machine->init_done = true;
    machine->pcard = card;
    if (machine_is_whistler()) {
        machine->audio_reg = regulator_get(NULL, "avddio_audio");
        if (IS_ERR(machine->audio_reg)) {
            dev_err(card->dev, "cannot get avddio_audio reg\n");
            ret = PTR_ERR(machine->audio_reg);
            return ret;
        }

        ret = regulator_enable(machine->audio_reg);
        if (ret) {
            dev_err(card->dev, "cannot enable avddio_audio reg\n");
            regulator_put(machine->audio_reg);
            machine->audio_reg = NULL;
            return ret;
        }
    }

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

    ret = snd_soc_add_controls(codec, tegra_aic326x_controls,
                    ARRAY_SIZE(tegra_aic326x_controls));
    if (ret < 0)
        return ret;

    snd_soc_dapm_new_controls(dapm, tegra_aic326x_dapm_widgets,
                    ARRAY_SIZE(tegra_aic326x_dapm_widgets));

	ret = snd_soc_dapm_add_routes(dapm, aic326x_audio_map,
                    ARRAY_SIZE(aic326x_audio_map));

	if(ret)
	{
		printk(KERN_INFO "%s:failed adding routes\n",__func__);
	}

    /* Add call mode switch control */
    ret = snd_ctl_add(codec->card->snd_card,
            snd_ctl_new1(&tegra_aic326x_call_mode_control,
                machine));
    if (ret < 0)
        return ret;
    	/* Add FM mode switch control */
	ret = snd_ctl_add(codec->card->snd_card,
			snd_ctl_new1(&tegra_aic326x_fm_mode_control,
				machine));
	if (ret < 0)
		return ret;


    //switch off all the PINs
    snd_soc_dapm_disable_pin(dapm, "Int Spk");
    snd_soc_dapm_disable_pin(dapm, "Earpiece");
    snd_soc_dapm_disable_pin(dapm, "Headphone Jack");
    snd_soc_dapm_disable_pin(dapm, "HP Mic");
    snd_soc_dapm_disable_pin(dapm, "Main Mic");
    snd_soc_dapm_disable_pin(dapm, "Aux Mic");
    snd_soc_dapm_disable_pin(dapm, "Linein");
    snd_soc_dapm_nc_pin(dapm, "SPKR");
    snd_soc_dapm_sync(dapm);

//to forbide fm path close when enter into suspend
	snd_soc_dapm_ignore_suspend(dapm, "Headphone Jack");
	snd_soc_dapm_ignore_suspend(dapm, "Int Spk");
	snd_soc_dapm_ignore_suspend(dapm, "Linein");

    return 0;
}

static struct snd_soc_dai_link tegra_i2c_aic326x_dai[] = {
    [DAI_LINK_HIFI] = {
        .name = "AIC3262",
        .stream_name = "AIC3262 PCM HIFI",
        .codec_name = "aic3262-codec.0-0018",   //zuoziqiang modify
        .platform_name = "tegra-pcm-audio",
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
        .cpu_dai_name = "tegra20-i2s.0",
#endif
        .codec_dai_name = "aic3262-asi1",
        .init = tegra_aic326x_init,
        .ops = &tegra_aic326x_hifi_ops,
        },
    {
        .name = "VOICE CALL",
        .stream_name = "VOICE CALL PCM",
        .codec_name = "aic3262-codec.0-0018",  //zzq modify
        .platform_name = "tegra-pcm-audio",
        .cpu_dai_name = "dit-hifi",
        .codec_dai_name = "aic3262-asi2",
        .ops = &tegra_aic326x_voice_call_ops,
      },
      {
        .name = "BT VOICE CALL",
        .stream_name = "BT VOICE CALL PCM",
        .codec_name = "aic3262-codec.0-0018",
        .platform_name = "tegra-pcm-audio",
        .cpu_dai_name = "dit-hifi",
        .codec_dai_name = "aic3262-asi3",
        .ops = &tegra_aic326x_bt_voice_call_ops,
    },
    {
         .name = "SPDIF",
         .stream_name = "SPDIF PCM",
         .codec_name = "spdif-dit.0",
         .platform_name = "tegra-pcm-audio",
         .cpu_dai_name = "tegra20-spdif",
         .codec_dai_name = "dit-hifi",
         .ops = &tegra_aic326x_spdif_ops,
    },
};

static struct snd_soc_dai_link tegra_spi_aic326x_dai[] = {
	[DAI_LINK_HIFI] = {
		.name = "AIC3262",
		.stream_name = "AIC3262 PCM HIFI",
		.codec_name = "spi2.2",
		.platform_name = "tegra-pcm-audio",
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		.cpu_dai_name = "tegra20-i2s.0",
#endif
		.codec_dai_name = "aic3262-asi1",
		.init = tegra_aic326x_init,
		.ops = &tegra_aic326x_hifi_ops,
		},
	      {
			.name = "VOICE CALL",
			.stream_name = "VOICE CALL PCM",
			.codec_name = "spi2.2",
			.platform_name = "tegra-pcm-audio",
			.cpu_dai_name = "dit-hifi",
			.codec_dai_name = "aic3262-asi2",
			.ops = &tegra_aic326x_voice_call_ops,
		},
	      {
			.name = "BT VOICE CALL",
			.stream_name = "BT VOICE CALL PCM",
			.codec_name = "spi2.2",
			.platform_name = "tegra-pcm-audio",
			.cpu_dai_name = "dit-hifi",
			.codec_dai_name = "aic3262-asi3",
			.ops = &tegra_aic326x_bt_voice_call_ops,
		},
             {
                .name = "SPDIF",
                .stream_name = "SPDIF PCM",
                .codec_name = "spdif-dit.0",
                .platform_name = "tegra-pcm-audio",
                .cpu_dai_name = "tegra20-spdif",
                .codec_dai_name = "dit-hifi",
                .ops = &tegra_aic326x_spdif_ops,
            },
};

static struct snd_soc_card snd_soc_i2c_tegra_aic326x = {
    .name = "tegra-aic326x",
	.dai_link = tegra_i2c_aic326x_dai,
	.num_links = ARRAY_SIZE(tegra_i2c_aic326x_dai),
};

static struct snd_soc_card snd_soc_spi_tegra_aic326x = {
	.name = "tegra-aic326x",
	.dai_link = tegra_spi_aic326x_dai,
	.num_links = ARRAY_SIZE(tegra_spi_aic326x_dai),
};

static __devinit int tegra_aic326x_driver_probe(struct platform_device *pdev)
{
    struct snd_soc_card *card;
    struct tegra_aic326x *machine;
    struct tegra_aic326x_platform_data *pdata;
    int ret;

    dev_dbg(&pdev->dev, "enter %s\n", __func__);

#if CONFIG_TLV320AIC326x_U970_VERSION
    if (0 == zte_get_board_id())
    {
         card = &snd_soc_i2c_tegra_aic326x;
    }
    else
    {
         card = &snd_soc_spi_tegra_aic326x;
    }
#else
    card = &snd_soc_spi_tegra_aic326x;
#endif

    pdata = pdev->dev.platform_data;
    if (!pdata) {
        dev_err(&pdev->dev, "No platform data supplied\n");
        return -EINVAL;
    }

    machine = kzalloc(sizeof(struct tegra_aic326x), GFP_KERNEL);
    if (!machine) {
        dev_err(&pdev->dev, "Can't allocate tegra_aic326x struct\n");
        return -ENOMEM;
    }

    machine->pdata = pdata;

    ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev);
	if (ret) {
		printk(" tegra_asoc_utils_init failed\n");
        goto err_free_machine;
	}
    card->dev = &pdev->dev;
    platform_set_drvdata(pdev, card);
    snd_soc_card_set_drvdata(card, machine);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
    for (i = 0; i < NUM_I2S_DEVICES ; i++)
        machine->codec_info[i].i2s_id = pdata->audio_port_id[i];

    machine->codec_info[BASEBAND].rate = pdata->baseband_param.rate;
    machine->codec_info[BASEBAND].channels = pdata->baseband_param.channels;
       //ZTE: first hardware version uses i2c bus, else other uses spi bus
       #if CONFIG_TLV320AIC326x_U970_VERSION
       if (0 == zte_get_board_id())
       {
            tegra_i2c_aic326x_dai[DAI_LINK_HIFI].cpu_dai_name =
	     tegra_aic326x_i2s_dai_name[machine->codec_info[HIFI_CODEC].i2s_id];
       }
	else
	{
	     	tegra_spi_aic326x_dai[DAI_LINK_HIFI].cpu_dai_name =
              tegra_aic326x_i2s_dai_name[machine->codec_info[HIFI_CODEC].i2s_id];
	}
	#else
       tegra_spi_aic326x_dai[DAI_LINK_HIFI].cpu_dai_name =
       tegra_aic326x_i2s_dai_name[machine->codec_info[HIFI_CODEC].i2s_id];
	#endif
#endif

    ret = snd_soc_register_card(card);
    if (ret) {
        dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
            ret);
        goto err_fini_utils;
    }

    if (!card->instantiated) {
        dev_err(&pdev->dev, "No TI AIC3262 codec\n");
        goto err_unregister_card;
    }


    dev_dbg(&pdev->dev, "eixt %s success\n", __func__);
    return 0;

err_unregister_card:
    snd_soc_unregister_card(card);
err_fini_utils:
    tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:
    kfree(machine);
    return ret;
}

static int __devexit tegra_aic326x_driver_remove(struct platform_device *pdev)
{
    struct snd_soc_card *card = platform_get_drvdata(pdev);
    struct tegra_aic326x *machine = snd_soc_card_get_drvdata(card);
    struct tegra_aic326x_platform_data *pdata = machine->pdata;

    snd_soc_unregister_card(card);

    tegra_asoc_utils_fini(&machine->util_data);

    if (machine->gpio_requested & GPIO_EXT_MIC_EN)
        gpio_free(pdata->gpio_ext_mic_en);
    if (machine->gpio_requested & GPIO_INT_MIC_EN)
        gpio_free(pdata->gpio_int_mic_en);
    if (machine->gpio_requested & GPIO_HP_MUTE)
        gpio_free(pdata->gpio_hp_mute);
    if (machine->gpio_requested & GPIO_SPKR_EN)
        gpio_free(pdata->gpio_spkr_en);

    kfree(machine);

    return 0;
}

static struct platform_driver tegra_aic326x_driver = {
    .driver = {
        .name = DRV_NAME,
        .owner = THIS_MODULE,
        .pm = &snd_soc_pm_ops,
    },
    .probe = tegra_aic326x_driver_probe,
    .remove = __devexit_p(tegra_aic326x_driver_remove),
};

static int __init tegra_aic326x_modinit(void)
{
    return platform_driver_register(&tegra_aic326x_driver);
}
module_init(tegra_aic326x_modinit);

static void __exit tegra_aic326x_modexit(void)
{
    platform_driver_unregister(&tegra_aic326x_driver);
}
module_exit(tegra_aic326x_modexit);

/* Module information */
MODULE_AUTHOR("Vinod G. <vinodg@nvidia.com>");
MODULE_DESCRIPTION("Tegra+AIC3262 machine ASoC driver");
MODULE_DESCRIPTION("Tegra ALSA SoC");
MODULE_LICENSE("GPL");

