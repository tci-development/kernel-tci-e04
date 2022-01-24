/*
 * rockchip_rt5640.c  --  RK3399 machine driver with
 * RT5640/TC358749 codecs
 *
 * Copyright (c) 2016, ROCKCHIP CORPORATION.  All rights reserved.
 * Author: Xiaotan Luo <lxt@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <sound/soc.h>

#include "rockchip_i2s.h"
#include "../codecs/rt5640.h"

#define DRV_NAME "rockchip-rt5640"

#define RT5640_STEREO_RATES SNDRV_PCM_RATE_8000_192000
#define RT5640_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S8)
static const struct snd_soc_pcm_stream rt5640_bt_params = {
	.rates = RT5640_STEREO_RATES,
	.formats = RT5640_FORMATS,
	.channels_min = 1,
	.channels_max = 2,
};


static const struct snd_soc_dapm_widget rockchip_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_MIC("BT Down", NULL),
	SND_SOC_DAPM_SPK("BT Up", NULL),
};

static const struct snd_soc_dapm_route rockchip_dapm_routes[] = {
	{"Headphones", NULL, "HPOL"},
	{"Headphones", NULL, "HPOR"},
	{"Speaker", NULL, "SPOLP"},
	{"Speaker", NULL, "SPOLN"},
	{"Speaker", NULL, "SPORP"},
	{"Speaker", NULL, "SPORN"},
	{"Headset Mic", NULL, "MICBIAS1"},
	{"IN2P", NULL, "Headset Mic"},
	{"IN2N", NULL, "Headset Mic"},
	{"Int Mic", NULL, "MICBIAS1"},
	{"IN1P", NULL, "Int Mic"},
	{"IN1N", NULL, "Int Mic"},

	// BT
	{"AIF2 Playback", NULL, "BT Down"},
	{"BT Up", NULL, "AIF2 Capture"},
};

static const struct snd_kcontrol_new rockchip_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphones"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("BT Down"),
	SOC_DAPM_PIN_SWITCH("BT Up"),
};

static int rockchip_rt5640_asoc_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	rt5640_sel_asrc_clk_src(codec_dai->codec,
				RT5640_DA_STEREO_FILTER |
				RT5640_DA_MONO_L_FILTER	|
				RT5640_DA_MONO_R_FILTER	|
				RT5640_AD_STEREO_FILTER	|
				RT5640_AD_MONO_L_FILTER	|
				RT5640_AD_MONO_R_FILTER,
				RT5640_CLK_SEL_ASRC);

	return 0;
}


static int rockchip_rt5640_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int mclk, ret;

	/* in bypass mode, the mclk has to be one of the frequencies below */
	switch (params_rate(params)) {
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
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Can't set cpu clock out %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, RT5640_SCLK_S_PLL1, params_rate(params)*512,
					SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec_dai->dev, "codec_dai sys clock not set\n");
		return ret;
	}
	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5640_PLL1_S_MCLK, mclk, params_rate(params)*512);
	if (ret < 0) {
		dev_err(codec_dai->dev, "codec_dai pll clock not set\n");
		return ret;
	}

	return 0;
}

#if 0
static int rockchip_rt5640_voice_hw_params(struct snd_pcm_substream *substream,
					   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int mclk, ret;

	/* in bypass mode, the mclk has to be one of the frequencies below */
	switch (params_rate(params)) {
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
	}

	/*Set the system clk for codec*/
#if 0
	snd_soc_dai_set_pll(codec_dai, 0, RT5640_PLL1_S_MCLK, mclk, 24576000);

	ret = snd_soc_dai_set_sysclk(codec_dai, RT5640_SCLK_S_PLL1, 24576000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Can't set codec clock in %d\n", ret);
		return ret;
	}
#else
	ret = snd_soc_dai_set_sysclk(codec_dai, RT5640_SCLK_S_PLL1, params_rate(params)*512,
					SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec_dai->dev, "codec_dai sys clock not set\n");
		return ret;
	}
	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5640_PLL1_S_MCLK, mclk, params_rate(params)*512);
	if (ret < 0) {
		dev_err(codec_dai->dev, "codec_dai pll clock not set\n");
		return ret;
	}
#endif
	return 0;
}
#endif

static struct snd_soc_ops rockchip_sound_rt5640_hifi_ops = {
	.hw_params = rockchip_rt5640_hw_params,
};
/*
static struct snd_soc_ops rockchip_sound_rt5640_voice_ops = {
	.hw_params = rockchip_rt5640_voice_hw_params,
};*/

enum {
	DAILINK_RT5640_HIFI,
	DAILINK_RT5640_VOICE,
	DAILINK_HDMI,
};

#define DAILINK_ENTITIES	(DAILINK_HDMI + 1)

static struct snd_soc_dai_link rockchip_dailinks[] = {
	[DAILINK_RT5640_HIFI] = {
		.name = "RT5640 HIFI",
		.stream_name = "RT5640 PCM",
		.codec_dai_name = "rt5640-aif1",
		.init = rockchip_rt5640_asoc_init,
		.ops = &rockchip_sound_rt5640_hifi_ops,
		/* set rt5640 as slave */
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},
	[DAILINK_RT5640_VOICE] = {
		.name = "RT5640 VOICE",
		.stream_name = "RT5640 PCM",
		.cpu_dai_name = "rt5640-aif1",
		//.codec_name = "rt5640.2-001c",
		.codec_dai_name = "rt5640-aif2",
		/* set rt5640 as slave */
		//.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		//	SND_SOC_DAIFMT_CBM_CFM,
		.params = &rt5640_bt_params,
	},
	[DAILINK_HDMI] = {
		.name = "rockchip hdmi",
		.stream_name = "rockchip hdmi",
		.codec_dai_name = "i2s-hifi",
	},
};

static struct snd_soc_card rockchip_sound_card = {
	.name = "rockchiprt5640codec",
	.owner = THIS_MODULE,
	.dai_link = rockchip_dailinks,
	.num_links =  ARRAY_SIZE(rockchip_dailinks),
	.dapm_widgets = rockchip_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rockchip_dapm_widgets),
	.dapm_routes = rockchip_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(rockchip_dapm_routes),
	.controls = rockchip_controls,
	.num_controls = ARRAY_SIZE(rockchip_controls),
};

static int rockchip_sound_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &rockchip_sound_card;
	struct device_node *cpu_node;
	int i, ret;

	dev_info(&pdev->dev, "%s\n", __func__);

	cpu_node = of_parse_phandle(pdev->dev.of_node, "rockchip,cpu", 0);
	if (!cpu_node) {
		dev_err(&pdev->dev,
			"Property 'rockchip,cpu' failed\n");
		return -EINVAL;
	}

	for (i = 0; i < DAILINK_ENTITIES; i++) {
		rockchip_dailinks[i].platform_of_node = cpu_node;
		rockchip_dailinks[i].cpu_of_node = cpu_node;

		rockchip_dailinks[i].codec_of_node =
			of_parse_phandle(pdev->dev.of_node,
					 "rockchip,codec", i);
		if (!rockchip_dailinks[i].codec_of_node) {
			dev_err(&pdev->dev,
				"Property[%d] 'rockchip,codec' failed\n", i);
			return -EINVAL;
		}
	}
    rockchip_dailinks[0].platform_of_node = rockchip_dailinks[0].cpu_of_node;

	/* for codec 2 codec DAI */
	rockchip_dailinks[1].cpu_of_node = rockchip_dailinks[0].codec_of_node;
	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "%s register card failed %d\n",
			__func__, ret);

	dev_info(&pdev->dev, "snd_soc_register_card successful\n");
	return ret;
}

static const struct of_device_id rockchip_sound_of_match[] = {
	{ .compatible = "rockchip,rockchip-rt5640-sound", },
	{},
};

static struct platform_driver rockchip_sound_driver = {
	.probe = rockchip_sound_probe,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = rockchip_sound_of_match,
#ifdef CONFIG_PM
		.pm = &snd_soc_pm_ops,
#endif
	},
};

module_platform_driver(rockchip_sound_driver);

MODULE_AUTHOR("Xiaotan Luo <lxt@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip ASoC Machine Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, rockchip_sound_of_match);
