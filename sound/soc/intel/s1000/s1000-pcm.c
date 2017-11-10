/*
 *  s1000-pcm.c -ASoC S1000 Platform driver file implementing PCM functionality
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
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "s1000.h"

#define S1000_MONO 1
#define S1000_STEREO 2

#define S1000_MAX_BUF_SIZE	(1024*1024)
#define S1000_MIN_PERIOD_BYTES    32
#define S1000_MIN_PERIODS         2
#define S1000_MAX_PERIODS         32
#define S1000_HW_BUF_SIZE		4084
#define S1000_STREAM_RESP_MSECS	100
#define S1000_MIN_PERIODS_CP         4
#define S1000_MIN_PERIOD_BYTES_CP    32000

static struct snd_pcm_hardware s1000_pcm_hw = {
	.info =			(SNDRV_PCM_INFO_MMAP |
				 SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_BLOCK_TRANSFER |
				 SNDRV_PCM_INFO_MMAP_VALID |
				 SNDRV_PCM_INFO_PAUSE |
				 SNDRV_PCM_INFO_RESUME |
				 SNDRV_PCM_INFO_SYNC_START |
				 SNDRV_PCM_INFO_NO_PERIOD_WAKEUP),
	.formats =		SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S32_LE |
				SNDRV_PCM_FMTBIT_S24_LE,
	.rates =		SNDRV_PCM_RATE_48000,
	.rate_min =		48000,
	.rate_max =		48000,
	.channels_min =		1,
	.channels_max =		2,
	.buffer_bytes_max =	S1000_MAX_BUF_SIZE,
	.period_bytes_min =	4084,
	.period_bytes_max =	S1000_MAX_BUF_SIZE / 2,
	.periods_min =		2,
	.periods_max =		S1000_MAX_PERIODS,
	.fifo_size =		0,
};

static void s1000_write_data_work(struct s1000 *ctx, struct s1000_stream *stream)
{
	int ret, size;
	void *src = (void *) (stream->substream->runtime->dma_area);
	u32 offset = 0, cp_size;

	cp_size = stream->period_size;

	while (stream->started) {
		cp_size = stream->period_size;
		while (cp_size > 0) {
			if (!stream->low_thres) {
				ret = wait_event_interruptible_timeout(stream->wr_waitq,
					((stream->low_thres) || !stream->started),
					msecs_to_jiffies(S1000_STREAM_RESP_MSECS * 10));
				if (ret == 0) {
					dev_err(ctx->dev, "***response timeout\n");
					return;
				}

			}

			mutex_lock(&stream->lock);
			if (!stream->started) {
				mutex_unlock(&stream->lock);
				return;
			}

			trace_printk("In %s line=%d low=%d started=%d\n", __func__, __LINE__,
				     stream->low_thres, stream->started);

			stream->low_thres = false;

			if (cp_size > S1000_HW_BUF_SIZE)
				size = S1000_HW_BUF_SIZE;
			else
				size = cp_size;

			trace_printk("cp_size =%d size=%d hw=%d\n", cp_size,
				     size, S1000_HW_BUF_SIZE);
			if ((stream->pos + size) >  stream->buffer_size)
				size = stream->buffer_size - stream->pos;

			trace_printk("transferring total=%d bytes=%d period_pos=%d offset=%d\n",
				     stream->size,  size, stream->period_pos, offset);

			ret = s1000_write_data(ctx, stream->res_id, src + offset, size);
			mutex_unlock(&stream->lock);
			if (ret < 0) {
				dev_err(ctx->dev, "write error ret=%d\n", ret);
				return;
			}
			stream->pos += size;
			stream->pos %= stream->buffer_size;
			stream->size += size;
			stream->period_pos += size;
			trace_printk("stream position=%d\n", stream->pos);
			offset = stream->pos;
			if (stream->period_pos >= stream->period_size) {
				stream->period_pos -= stream->period_size;
				snd_pcm_period_elapsed(stream->substream);
			}
			cp_size -= size;
		}
	}
}

static void s1000_read_data_work(struct s1000 *ctx, struct s1000_stream *stream)
{
	int ret, size_rd, cp_size, sz_left, size = S1000_HW_BUF_SIZE;
	void *data;
	void *dst = (void *) (stream->substream->runtime->dma_area);
	u32 offset = 0, data_offset = 0;

	trace_printk("In %s line=%d\n", __func__, __LINE__);

	data = kzalloc(S1000_HW_BUF_SIZE, GFP_KERNEL);
	if (!data)
		return;

	while (stream->started) {
		if (!stream->high_thres || !stream->kp_detected) {
			ret = wait_event_interruptible_timeout(stream->wr_waitq,
					((stream->high_thres && stream->kp_detected)
					 || !stream->started),
				msecs_to_jiffies(S1000_STREAM_RESP_MSECS * 10));
			if (ret == 0) {
				dev_err(ctx->dev, "***response timeout\n");
				kfree(data);
				return;
			}

		}

		mutex_lock(&stream->lock);
		if (!stream->started) {
			mutex_unlock(&stream->lock);
			kfree(data);
			return;
		}

		trace_printk("In %s line=%d high=%d kp=%d\n", __func__, __LINE__,
		     stream->high_thres, stream->kp_detected);
		stream->high_thres = false;
		trace_printk("Reading size=%d total_bytes=%d\n", size,
							stream->size);
		size_rd = s1000_read_data(ctx, stream->res_id, data,
						size, &sz_left);
		mutex_unlock(&stream->lock);
		if (size_rd < 0) {
			dev_err(ctx->dev, "read error ret=%d\n", ret);
			kfree(data);
			return;
		}

		data_offset = 0;
		while (size_rd > 0) {
			if (stream->pos + size_rd > stream->buffer_size)
				cp_size = stream->buffer_size - stream->pos;
			else
				cp_size = size_rd;
			trace_printk("copy size=%d size = %d offset=%d data_offset=%d\n",
				     cp_size, size_rd, offset, data_offset);
			memcpy(dst + offset, data + data_offset, cp_size);
			stream->pos += cp_size;
			stream->pos %= stream->buffer_size;
			stream->size += cp_size;
			stream->period_pos += cp_size;
			offset = stream->pos;
			data_offset += cp_size;
			size_rd -= cp_size;
		}
		if (stream->period_pos >= stream->period_size) {
			stream->period_pos -= stream->period_size;
			snd_pcm_period_elapsed(stream->substream);
		}

		if (sz_left > size)
			stream->high_thres = true;

	}

	kfree(data);

}

static void s1000_data_work(struct work_struct *work)
{
	struct s1000_stream *stream =
			container_of(work, struct s1000_stream, data_kwork);
	struct snd_soc_dai *dai = stream->dai;
	struct s1000 *ctx = snd_soc_dai_get_drvdata(dai);
	struct snd_pcm_substream *substream = stream->substream;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		s1000_write_data_work(ctx, stream);
	else
		s1000_read_data_work(ctx, stream);
}

void s1000_stream_low_threshold(struct s1000 *ctx, u32 res_id)
{
	struct s1000_stream *stream = NULL;
	int i;

	for (i = 0 ; i < 2; i++) {
		if (ctx->streams[i].res_id == res_id) {
			stream = &ctx->streams[i];
			trace_printk("resource found\n");
			break;
		}
	}
	if (stream && stream->started) {
		stream->low_thres = true;
		wake_up(&stream->wr_waitq);
	}
}

void s1000_stream_high_threshold(struct s1000 *ctx, u32 res_id)
{
	struct s1000_stream *stream = NULL;
	int i;

	for (i = 0 ; i < 2; i++) {
		if (ctx->streams[i].res_id == res_id) {
			stream = &ctx->streams[i];
			break;
		}
	}
	if (stream) {
		stream->high_thres = true;
		if (stream->started) {
			wake_up(&stream->wr_waitq);
			schedule_work(&stream->data_kwork);
		}
		trace_printk("calling data_work...\n");
	}
}

int s1000_send_event_notify(struct s1000 *ctx)
{
	struct snd_soc_platform *platform;
	struct snd_soc_card *card;
	struct snd_kcontrol *kcontrol = NULL;

	platform = snd_soc_lookup_platform(ctx->dev);

	if (!platform)
		return -EINVAL;

	card = platform->component.card;

	kcontrol = snd_soc_card_get_kcontrol(card,
			"KP Detect Control");
	if (!kcontrol)
		return -EINVAL;

	snd_ctl_notify(card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			&kcontrol->id);

	return 0;
}

void s1000_stream_kp_detected(struct s1000 *ctx, void *kp_data)
{
	struct s1000_stream *stream = NULL;
	int i;

	for (i = 0 ; i < 2; i++) {
		if (ctx->streams[i].wov_stream) {
			stream = &ctx->streams[i];
			break;
		}
	}

	if (stream) {
		stream->kp_detected = true;
		memcpy(&stream->kp_data, kp_data, sizeof(u32));
	}
	s1000_send_event_notify(ctx);
}

static int s1000_pcm_dai_open(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct s1000 *ctx = snd_soc_dai_get_drvdata(dai);
	struct s1000_stream *stream;
	struct snd_pcm_runtime *runtime = substream->runtime;

	trace_printk("In %s line=%d %s\n", __func__, __LINE__, dai->name);

	if (!ctx->tplg_loaded)
		return -EIO;

	if (ctx->dsp_state == S1000_DSP_UPDATING)
		return -EBUSY;

	stream = &ctx->streams[substream->stream];
	stream->dai = dai;
	stream->low_thres = 0;
	stream->pos = 0;
	stream->period_pos = 0;
	stream->size = 0;
	stream->listening_mode = false;
	init_waitqueue_head(&stream->wr_waitq);
	stream->is_prepared = false;
	/* set constrain if any */
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	/* save any private data to be used later in runtime */
	stream->substream = substream;
	runtime->private_data = stream;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		runtime->hw.info &= ~SNDRV_PCM_INFO_PAUSE;

	return 0;
}

static int s1000_pcm_dai_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	int ret;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct s1000_stream *stream = runtime->private_data;

	trace_printk("In %s line=%d %s\n", __func__, __LINE__, dai->name);
	ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (ret < 0) {
		dev_err(dai->dev, "Failed to allocated pages ret=%d\n", ret);
		return ret;
	}

	memset(substream->runtime->dma_area, 0, params_buffer_bytes(params));
	stream->channels = params_channels(params);
	stream->params_rate = params_rate(params);
	stream->pcm_format_width = snd_pcm_format_width(params_format(params));
	return 0;
}

static int s1000_pcm_dai_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct s1000_stream *stream = runtime->private_data;

	if (stream->is_prepared && !(substream->runtime->status->state ==
	    SNDRV_PCM_STATE_XRUN))
		return 0;

	trace_printk("In %s line=%d %s\n", __func__, __LINE__, dai->name);
	stream->buffer_size = snd_pcm_lib_buffer_bytes(substream);
	stream->period_size = snd_pcm_lib_period_bytes(substream);
	stream->pos = 0;
	stream->period_pos = 0;
	stream->size = 0;
	trace_printk("prepare period=%d buf_size=%d\n",
			stream->period_size, stream->buffer_size);

	/* handle XRUN */
	if (substream->runtime->status->state == SNDRV_PCM_STATE_XRUN) {
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			/* if wov stream, can't reset the stream */
			stream->kp_detected = true;
			return 0;
		}
	}
	stream->is_prepared = true;
	return 0;
}

static int s1000_pcm_dai_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct s1000_stream *stream = runtime->private_data;

	trace_printk("In %s line=%d %s\n", __func__, __LINE__, dai->name);
	cancel_work_sync(&stream->data_kwork);
	stream->is_prepared = false;
	/* free stream pages */
	return snd_pcm_lib_free_pages(substream);
}

static void s1000_pcm_dai_close(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	trace_printk("In %s line=%d %s\n", __func__, __LINE__, dai->name);
}

static int s1000_pcm_dai_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct s1000_stream *stream = runtime->private_data;
	struct s1000 *ctx = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

	trace_printk("In %s line=%d cmd=%d\n", __func__, __LINE__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		mutex_lock(&stream->lock);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			stream->low_thres = 1;
			ret =  s1000_stream_control_op(ctx, S1000_CMD_START,
								stream);
		} else {
			if (stream->listening_mode ||
			    stream->streaming_mode == S1000_MODE_STREAMING) {
				stream->kp_detected = true;
				ret =  s1000_stream_control_op(ctx,
					S1000_CMD_CLOUD_START, stream);
			}
		}
		mutex_unlock(&stream->lock);

		if (ret < 0)
			return ret;

		stream->started = 1;
		schedule_work(&stream->data_kwork);
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		stream->started = 0;
		wake_up(&stream->wr_waitq);
		stream->listening_mode = true;

		mutex_lock(&stream->lock);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			ret =  s1000_stream_control_op(ctx,
							S1000_CMD_STOP,
							stream);
		} else {
			ret =  s1000_stream_control_op(ctx,
						       S1000_CMD_CLOUD_STOP,
						       stream);
			stream->kp_detected = false;
			stream->kp_data = 0;
		}
		mutex_unlock(&stream->lock);

		if (ret < 0)
			return ret;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static struct snd_soc_dai_ops s1000_pcm_dai_ops = {
	.startup = s1000_pcm_dai_open,
	.shutdown = s1000_pcm_dai_close,
	.prepare = s1000_pcm_dai_prepare,
	.hw_params = s1000_pcm_dai_hw_params,
	.hw_free = s1000_pcm_dai_hw_free,
	.trigger = s1000_pcm_dai_trigger,
};

static struct snd_soc_dai_driver s1000_platform_dai[] = {
{
	.name = "System Pin",
	.ops = &s1000_pcm_dai_ops,
	.playback = {
		.stream_name = "System Playback",
		.channels_min = S1000_MONO,
		.channels_max = S1000_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "System Capture",
		.channels_min = S1000_MONO,
		.channels_max = S1000_MONO,
		.rates = SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "System Pin1",
	.ops = &s1000_pcm_dai_ops,
	.playback = {
		.stream_name = "System Playback1",
		.channels_min = S1000_MONO,
		.channels_max = S1000_STEREO,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.stream_name = "System Capture1",
		.channels_min = S1000_MONO,
		.channels_max = S1000_STEREO,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
};

static int s1000_platform_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	dev_dbg(rtd->cpu_dai->dev, "In %s:%s\n", __func__,
					dai_link->cpu_dai_name);

	runtime = substream->runtime;
	snd_soc_set_runtime_hwparams(substream, &s1000_pcm_hw);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		runtime->hw.info &= ~SNDRV_PCM_INFO_PAUSE;
		runtime->hw.period_bytes_min = S1000_MIN_PERIOD_BYTES_CP;
		runtime->hw.periods_min = S1000_MIN_PERIODS_CP;
	}

	return 0;
}

static snd_pcm_uframes_t s1000_platform_pcm_pointer
			(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct s1000_stream *stream = runtime->private_data;

	trace_printk("In %s line=%d\n", __func__, __LINE__);
	return bytes_to_frames(runtime, stream->pos);
}

static const struct snd_pcm_ops s1000_platform_ops = {
	.open = s1000_platform_open,
	.ioctl = snd_pcm_lib_ioctl,
	.pointer = s1000_platform_pcm_pointer,
	.mmap = snd_pcm_lib_default_mmap,
	.page = snd_pcm_sgbuf_ops_page,
};

static void s1000_pcm_free(struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static int s1000_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;

	trace_printk("In %s line=%d\n", __func__, __LINE__);
	if (dai->driver->playback.channels_min ||
			dai->driver->capture.channels_min) {
		ret =  snd_pcm_lib_preallocate_pages_for_all(pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL),
			S1000_MAX_BUF_SIZE, S1000_MAX_BUF_SIZE);

		if (ret) {
			dev_err(rtd->dev, "dma buffer allocationf fail\n");
			return ret;
		}
	}
	return ret;
}

static int s1000_detect_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct s1000_stream *stream = NULL;

	struct s1000 *ctx = snd_soc_component_get_drvdata(component);
	int i;

	for (i = 0 ; i < 2; i++) {
		if (ctx->streams[i].wov_stream) {
			stream = &ctx->streams[i];
			break;
		}
	}

	if (stream)
		ucontrol->value.integer.value[0] = stream->kp_data;
	else
		ucontrol->value.integer.value[0] = 0;


	return 0;
}

static int s1000_detect_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static const char *streaming_mode_text[] = {"WOV", "Streaming"};
static const struct soc_enum streaming_mode_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(streaming_mode_text),
				streaming_mode_text);

static int streaming_mode_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct s1000 *ctx = snd_soc_component_get_drvdata(component);
	struct s1000_stream *stream = NULL;
	int i;

	for (i = 0 ; i < S1000_MAX_STREAMS; i++) {
		if (ctx->streams[i].wov_stream) {
			stream = &ctx->streams[i];
			break;
		}
	}

	if (stream)
		ucontrol->value.enumerated.item[0] = stream->streaming_mode;

	return 0;
}

static int streaming_mode_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct s1000 *ctx = snd_soc_component_get_drvdata(component);
	struct s1000_stream *stream = NULL;
	int i;

	if (ctx->dsp_state == S1000_DSP_UPDATING)
		return -EBUSY;

	for (i = 0 ; i < S1000_MAX_STREAMS; i++) {
		if (ctx->streams[i].wov_stream) {
			stream = &ctx->streams[i];
			break;
		}
	}

	if (stream) {
		if (stream->dai && stream->dai->capture_active)
			return -EBUSY;

		stream->streaming_mode = ucontrol->value.enumerated.item[0];
	}

	return 0;
}

static int s1000_dsp_tplg_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct s1000 *ctx = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = ctx->tplg_loaded;

	return 0;
}

static int s1000_dsp_tplg_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct s1000 *ctx = snd_soc_component_get_drvdata(component);
	int i;

	if (ctx->dsp_state == S1000_DSP_UPDATING)
		return -EBUSY;

	for (i = 0; i < S1000_MAX_STREAMS; i++) {
		if (ctx->streams[i].dai && ctx->streams[i].dai->active)
			return -EBUSY;
	}

	if (ucontrol->value.integer.value[0]) {
		if (s1000_stream_control_op(ctx, S1000_CMD_TPLG_START,
					    NULL) < 0)
			return -EIO;

		ctx->tplg_loaded = true;
	} else {
		if (s1000_stream_control_op(ctx, S1000_CMD_TPLG_STOP,
					    NULL) < 0)
			return -EIO;

		ctx->tplg_loaded = false;
	}

	return 0;
}

static struct snd_kcontrol_new s1000_controls[] = {
	SOC_SINGLE_EXT("KP Detect Control", 0, 0, 0, 0,
			s1000_detect_get, s1000_detect_put),
	SOC_ENUM_EXT("Capture Stream mode", streaming_mode_enum,
			streaming_mode_get, streaming_mode_set),
	SOC_SINGLE_BOOL_EXT("DSP Load Topology Control", 0,
			s1000_dsp_tplg_get, s1000_dsp_tplg_put),
};

static struct snd_soc_platform_driver s1000_platform_drv  = {
	.ops		= &s1000_platform_ops,
	.pcm_new	= s1000_pcm_new,
	.pcm_free	= s1000_pcm_free,
};

static const struct snd_soc_component_driver s1000_component = {
	.name           = "pcm",
	.controls       = s1000_controls,
	.num_controls   = ARRAY_SIZE(s1000_controls),
};

int s1000_platform_register(struct device *dev)
{
	int ret, i;
	struct s1000 *ctx = dev_get_drvdata(dev);
	struct s1000_stream *stream;

	ret = devm_snd_soc_register_platform(dev, &s1000_platform_drv);
	if (ret) {
		dev_err(dev, "soc platform registration failed %d\n", ret);
		return ret;
	}
	ret = devm_snd_soc_register_component(dev, &s1000_component,
				s1000_platform_dai,
				ARRAY_SIZE(s1000_platform_dai));
	if (ret)
		dev_err(dev, "soc component registration failed %d\n", ret);

	for (i = 0; i < 2; i++) {
		stream = &ctx->streams[i];
		INIT_WORK(&stream->data_kwork, s1000_data_work);
		mutex_init(&stream->lock);
	}

	ctx->streams[SNDRV_PCM_STREAM_PLAYBACK].id = 1;
	ctx->streams[SNDRV_PCM_STREAM_PLAYBACK].res_id = 0x1500;
	ctx->streams[SNDRV_PCM_STREAM_PLAYBACK].started = false;

	ctx->streams[SNDRV_PCM_STREAM_CAPTURE].id = 1;
	ctx->streams[SNDRV_PCM_STREAM_CAPTURE].res_id = 0x1400;
	ctx->streams[SNDRV_PCM_STREAM_CAPTURE].started = false;
	ctx->streams[SNDRV_PCM_STREAM_CAPTURE].wov_stream = true;

	return ret;

}
