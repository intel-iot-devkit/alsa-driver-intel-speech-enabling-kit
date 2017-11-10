/*
 * Intel S1000 Machine Driver
 *
 * Modified from:
 *   Intel Broadwell Wildcatpoint SST Audio
 *
 *  This file is provided under a dual BSD/GPLv2 license.  When using or
 *  redistributing this file, you may do so under either license.
 *
 *  GPL LICENSE SUMMARY
 *
 *  Copyright(c) 2017 Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  Contact Information:
 *  Jeeja KP <jeeja.kp@intel.com>
 *  Shreyas NC <shreyas.nc@intel.com>
 *  Intel India.
 *
 *  BSD LICENSE
 *
 *  Copyright(c) 2017 Intel Corporation.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *    * Neither the name of Intel Corporation nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>

static const struct snd_kcontrol_new s1000_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Mic"),
};

static const struct snd_soc_dapm_widget s1000_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Mic", NULL),
};

static const struct snd_soc_dapm_route s1000_audio_map[] = {
	{"Dummy Capture", NULL, "Mic"},
	{"Headphone", NULL, "Dummy playback"},
	{"Speaker", NULL, "Dummy Playback"},
};

/* Sue digital audio interface glue */
static struct snd_soc_dai_link s1000_mach_dais[] = {
	{
		.name = "S1000 Audio Port",
		.stream_name = "Audio",
		.cpu_dai_name = "System Pin",
		.platform_name = "spi0.0",
		.nonatomic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
	},
};

/* s1000 audio machine driver for SPT + RT286S */
static struct snd_soc_card s1000_mach = {
	.name = "s1000_audio_card",
	.dai_link = s1000_mach_dais,
	.num_links = ARRAY_SIZE(s1000_mach_dais),
	.controls = s1000_controls,
	.num_controls = ARRAY_SIZE(s1000_controls),
	.dapm_widgets = s1000_widgets,
	.num_dapm_widgets = ARRAY_SIZE(s1000_widgets),
	.dapm_routes = s1000_audio_map,
	.num_dapm_routes = ARRAY_SIZE(s1000_audio_map),
	.fully_routed = true,
};

static int s1000_audio_probe(struct platform_device *pdev)
{
	s1000_mach.dev = &pdev->dev;
	return devm_snd_soc_register_card(&pdev->dev, &s1000_mach);
}

static struct platform_driver s1000_audio = {
	.probe = s1000_audio_probe,
	.driver = {
		.name = "s1000_mach",
	},
};

module_platform_driver(s1000_audio)

/* Module information */
MODULE_DESCRIPTION("Intel Audio driver for Quark s1000");
MODULE_AUTHOR("Jeeja KP <jeeja.kp@intel.com>");
MODULE_AUTHOR("Shreyas NC <shreyas.nc@intel.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:s1000_mach");
