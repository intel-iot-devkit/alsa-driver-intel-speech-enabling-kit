/*
 *  s1000-dsp-msg.c - Implementation of DSP protocol driver for s1000.
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
#include <linux/delay.h>
#include "s1000.h"
#include "s1000-dsp-msg.h"

#define BIT_EXT_READ4			0x80000000
#define BIT_EXT_WRITE4			0x40000000

/* Payload size */
#define S1000_RESP_PAYLOAD_SIZE_MASK 0x3FF
#define S1000_RESP_PAYLOAD_SIZE(x)   (x & S1000_RESP_PAYLOAD_SIZE_MASK)

/* Global Message - Reply */
#define S1000_GLB_REPLY_STATUS_SHIFT	24
#define S1000_GLB_REPLY_STATUS_MASK	((0x1 << S1000_GLB_REPLY_STATUS_SHIFT) - 1)
#define S1000_GLB_REPLY_STATUS(x)		((x) << S1000_GLB_REPLY_STATUS_SHIFT)

#define S1000_GLB_REPLY_TYPE_SHIFT        29
#define S1000_GLB_REPLY_TYPE_MASK         0x1F
#define S1000_GLB_REPLY_TYPE(x)           (((x) >> S1000_GLB_REPLY_TYPE_SHIFT) \
						& S1000_GLB_REPLY_TYPE_MASK)

#define S1000_MSG_TARGET_SHIFT		30
#define S1000_MSG_TARGET_MASK		0x1
#define S1000_MSG_TARGET(x)		(((x) & S1000_MSG_TARGET_MASK) \
					<< S1000_MSG_TARGET_SHIFT)

#define S1000_MSG_DIR_SHIFT		29
#define S1000_MSG_DIR_MASK		0x1
#define S1000_MSG_DIR(x)			(((x) & S1000_MSG_DIR_MASK) \
					<< S1000_MSG_DIR_SHIFT)
/* Global Notification Message */
#define S1000_GLB_NOTIFY_TYPE_SHIFT	16
#define S1000_GLB_NOTIFY_TYPE_MASK	0xFF
#define S1000_GLB_NOTIFY_TYPE(x)		(((x) >> S1000_GLB_NOTIFY_TYPE_SHIFT) \
					& S1000_GLB_NOTIFY_TYPE_MASK)

#define S1000_GLB_NOTIFY_MSG_TYPE_SHIFT	24
#define S1000_GLB_NOTIFY_MSG_TYPE_MASK	0x1F
#define S1000_GLB_NOTIFY_MSG_TYPE(x)	(((x) >> S1000_GLB_NOTIFY_MSG_TYPE_SHIFT)	\
						& S1000_GLB_NOTIFY_MSG_TYPE_MASK)

#define S1000_GLB_NOTIFY_RSP_SHIFT	29
#define S1000_GLB_NOTIFY_RSP_MASK		0x1
#define S1000_GLB_NOTIFY_RSP_TYPE(x)	(((x) >> S1000_GLB_NOTIFY_RSP_SHIFT) \
					& S1000_GLB_NOTIFY_RSP_MASK)
#define S1000_GLB_REPLY_SUCCESS 0

/* MSG_REQUEST */
#define S1000_MSG_BUSY_SHIFT		31
#define S1000_MSG_BUSY_MASK		(0x1 << S1000_MSG_BUSY_SHIFT)
#define S1000_MSG_BUSY(x)			(((x) & S1000_MSG_BUSY_MASK))

#define S1000_GLB_TYPE_SHIFT		24
#define S1000_GLB_TYPE_MASK		(0xf << S1000_GLB_TYPE_SHIFT)
#define S1000_GLB_TYPE(x)			((x) << S1000_GLB_TYPE_SHIFT)

#define S1000_MSG_FLAGS_SHIFT             25
#define S1000_MSG_FLAGS_MASK              (0x7f << S1000_MSG_FLAGS_SHIFT)
#define S1000_MSG_SET_FLAGS(x)            (((x) & S1000_MSG_FLAGS_MASK))

#define S1000_MSG_CLOCK_SELECT_SHIFT      21
#define S1000_MSG_CLOCK_SELECT_MASK       (0xf << S1000_MSG_CLOCK_SELECT_SHIFT)
#define S1000_MSG_SET_CLOCK(x)            (((x) & S1000_MSG_CLOCK_SELECT_MASK))

#define S1000_MSG_CMD_MASK                0x1ff
#define S1000_MSG_SET_CMD(x)              (((x) & S1000_MSG_CMD_MASK))

#define S1000_MSG_PL_COUNT_MASK           0x3ff
#define S1000_MSG_SET_PL_COUNT(x)         (((x) & S1000_MSG_PL_COUNT_MASK))

/* Module msg */
#define S1000_MSG_MOD_ID_MASK             0xffffff
#define S1000_MSG_SET_MOD_ID(x)           ((x) & S1000_MSG_MOD_ID_MASK)

/* Large Config message extension register */
#define S1000_DATA_OFFSET_SZ_SHIFT	0
#define S1000_DATA_OFFSET_SZ_MASK		0xFFFFF
#define S1000_DATA_OFFSET_SZ(x)		(((x) & S1000_DATA_OFFSET_SZ_MASK) \
					<< S1000_DATA_OFFSET_SZ_SHIFT)
#define S1000_DATA_OFFSET_SZ_CLEAR	~(S1000_DATA_OFFSET_SZ_MASK \
					  << S1000_DATA_OFFSET_SZ_SHIFT)

#define S1000_LARGE_PARAM_ID_SHIFT	20
#define S1000_LARGE_PARAM_ID_MASK		0xFF
#define S1000_LARGE_PARAM_ID(x)		(((x) & S1000_LARGE_PARAM_ID_MASK) \
					<< S1000_LARGE_PARAM_ID_SHIFT)

#define S1000_FINAL_BLOCK_SHIFT		28
#define S1000_FINAL_BLOCK_MASK		0x1
#define S1000_FINAL_BLOCK(x)		(((x) & S1000_FINAL_BLOCK_MASK) \
					<< S1000_FINAL_BLOCK_SHIFT)

#define S1000_INITIAL_BLOCK_SHIFT		29
#define S1000_INITIAL_BLOCK_MASK		0x1
#define S1000_INITIAL_BLOCK(x)		(((x) & S1000_INITIAL_BLOCK_MASK) \
					<< S1000_INITIAL_BLOCK_SHIFT)
#define S1000_INITIAL_BLOCK_CLEAR		~(S1000_INITIAL_BLOCK_MASK \
					  << S1000_INITIAL_BLOCK_SHIFT)

#define S1000_CLR_RSP_SHIFT		29
#define S1000_CLR_RSP_MASK		(1 << S1000_CLR_RSP_SHIFT)

/* Gateway msg */
#define S1000_IPCGATEWAY_CMD_SHIFT	0
#define S1000_IPCGATEWAY_CMD_MASK		0xFFFFFF
#define S1000_IPCGATEWAY_CMD(x)		(((x) & S1000_IPCGATEWAY_CMD_MASK) \
						<< S1000_IPCGATEWAY_CMD_SHIFT)

#define S1000_IPCGTW_DATA_OFFSET_SHIFT	0
#define S1000_IPCGTW_DATA_OFFSET_MASK	0xFFFFFFF
#define S1000_IPCGTW_DATA_OFFSET_CMD(x)	(((x) & S1000_IPCGTW_DATA_OFFSET_MASK) \
						<< S1000_IPCGTW_DATA_OFFSET_SHIFT)

#define S1000_POOL_SIZE 25

enum s1000_global_msg_type {
	S1000_GLB_MSG_ROM_CONTROL = 1,
	S1000_GLB_MSG_IPCGATEWAY = 2,
	S1000_GLB_MSG_NOTIFICATION = 27
};

enum s1000_msg_target {
	S1000_FW_GEN_MSG = 0,
	S1000_MOD_MSG = 1
};

enum s1000_msg_direction {
	S1000_MSG_REQUEST = 0,
	S1000_MSG_REPLY = 1
};

enum s1000_msg_notification_type {
	S1000_GLB_NOTIFY_PHRASE_DETECTED = 4,
	S1000_GLB_NOTIFY_RESOURCE_EVENT = 5,
	S1000_GLB_NOTIFY_LOG_BUFFER_STATUS = 6,
	S1000_GLB_NOTIFY_TIMESTAMP_CAPTURED = 7,
	S1000_GLB_NOTIFY_FW_READY = 8,
	S1000_GLB_NOTIFY_EXCEPTION_CAUGHT = 10,
	S1000_GLB_NOTIFY_MODULE_NOTIFICATION = 12,
	S1000_GLB_NOTIFY_DFU_READY = 13,
};

enum s1000_module_msg {
	S1000_MOD_CONFIG_GET = 1,
	S1000_MOD_CONFIG_SET = 2,
	S1000_MOD_LARGE_CONFIG_GET = 3,
	S1000_MOD_LARGE_CONFIG_SET = 4,
};

enum s1000_resource_type {
	S1000_RES_MODULE_INSTANCE  = 0,
	S1000_RES_PIPELINE  = 1,
	S1000_RES_GATEWAY = 2,
	S1000_RES_EDF_TASK = 3,
	S1000_RES_INVALID,
};

enum s1000_resource_event_type {
	S1000_BUDGET_VIOLATION		= 0,
	S1000_MIXER_UNDERRUN_DETECTED	= 1,
	S1000_STREAM_DATA_SEGMENT		= 2,
	S1000_PROCESS_DATA_ERROR		= 3,
	S1000_STACK_OVERFLOW		= 4,
	S1000_BUFFERING_MODE_CHANGED	= 5,
	S1000_GATEWAY_UNDERRUN_DETECTED	= 6,
	S1000_GATEWAY_OVERRUN_DETECTED	= 7,
	S1000_EDF_DOMAIN_UNSTABLE		= 8,
	S1000_WATCHDOG_EXPIRED		= 9,
	S1000_GATEWAY_HIGH_THRES		= 10,
	S1000_GATEWAY_LOW_THRES		= 11,
	S1000_GATEWAY_SYNC_STATUS		= 12,
};

/* Called with lock */
static struct s1000_gen_msg *get_msg_from_pool(struct s1000 *ctx)
{
	struct s1000_gen_msg *msg = NULL;

	if (!list_empty(&ctx->msg_pool_list)) {
		msg = list_first_entry(&ctx->msg_pool_list,
			struct s1000_gen_msg, node);
		msg->tx_size = 0;
		msg->rx_size = 0;
		msg->tx_done = 0;
		msg->err_code = 0;
		msg->raw_msg = 0;
		msg->mode_switch = 0;
		msg->recv = 0;
		memset(msg->tx_data, 0, S1000_LARGE_MSG_SIZE);
		memset(msg->rx_data, 0, S1000_LARGE_MSG_SIZE);
		list_del(&msg->node);
	}

	return msg;
}

static int s1000_register_irq_hanlder(struct s1000 *ctx, unsigned long flags)
{
	int ret;

	if (!ctx)
		return -EIO;

	if (ctx->flags == flags) {
		dev_err(ctx->dev, "flag is same\n");
		return -EIO;
	}
	ctx->flags = flags;

	if (ctx->irq < 0)
		return -EIO;

	free_irq(ctx->irq, ctx);

	ret = request_irq(ctx->irq, s1000_dsp_trigger_rising_handler,
				flags,
				"s1000-dsp", ctx);
	if (ret) {
		dev_err(ctx->dev, "request irq failed ret=%d", ret);
		return -EIO;
	}

	ret = S1000_GPIO_VAL;

	return ret;
}

/* Lock held by the caller of this function */
static struct s1000_gen_msg *s1000_find_msg(struct s1000 *ctx, u32 hdr)
{
	struct s1000_gen_msg *msg;
	u32 tx_hdr;
	int msg_no;

	msg_no = S1000_GLB_NOTIFY_MSG_TYPE(hdr);

	if (list_empty(&ctx->tx_list)) {
		dev_info(ctx->dev, "tx list empty but received 0x%x\n", hdr);
		return NULL;
	}

	msg = list_first_entry(&ctx->tx_list, struct s1000_gen_msg,
					node);
	tx_hdr = ntohl(msg->tx_data[0]);
	trace_printk("tx_hdr=%x msg_no=%x\n",
		S1000_GLB_NOTIFY_MSG_TYPE(tx_hdr), msg_no);
	if (S1000_GLB_NOTIFY_MSG_TYPE(tx_hdr) == msg_no) {
		trace_printk("found Msg*******\n");
		return msg;
	}

	dev_err(ctx->dev, "error: no matching response 0x%x\n", hdr);

	return NULL;
}

static void s1000_complete_msg(struct s1000 *ctx, struct s1000_msg *rx_msg,
		struct s1000_gen_msg *msg, u32 *payload, int size)
{
	int err;

	trace_printk("In %s size=%d\n", __func__, size);
	if (msg) {
		msg->rx_data[0] = rx_msg->primary;
		msg->rx_data[1] = rx_msg->extension;

		memcpy(&msg->rx_data[2], payload, size);
		msg->err_code = (rx_msg->primary & S1000_GLB_REPLY_STATUS_MASK);
		err = (rx_msg->primary & S1000_GLB_REPLY_STATUS_MASK);
		if (err != 0) {
			dev_err(ctx->dev, "Firmware reply err=%d\n", err);
			trace_printk("Firmware reply err=%d\n", err);
			/* FIXME map the error fw error code */
			msg->err_code = -EINVAL;
		}
	}
}

void s1000_process_notfication(struct s1000 *ctx, struct s1000_msg *msg,
				u32 *payload, size_t size)
{
	struct device *dev = ctx->dev;
	struct s1000_resource_event *event;
	struct s1000_kp_payload *kp_data;

	trace_printk("in %s lie=%d\n", __func__, __LINE__);
	if (S1000_GLB_NOTIFY_MSG_TYPE(msg->primary)) {
		switch (S1000_GLB_NOTIFY_TYPE(msg->primary)) {
		case S1000_GLB_NOTIFY_RESOURCE_EVENT:
			event = (struct s1000_resource_event *)payload;
			trace_printk("Res event:event id=%d type=%d\n",
					event->id, event->event_type);
			if (event->event_type == S1000_GATEWAY_LOW_THRES)
				s1000_stream_low_threshold(ctx, event->id);
			else if (event->event_type == S1000_GATEWAY_HIGH_THRES)
				s1000_stream_high_threshold(ctx, event->id);
			break;

		case S1000_GLB_NOTIFY_FW_READY:
			trace_printk("frimware ready: %x\n", msg->primary);
			ctx->boot_complete = true;
			wake_up(&ctx->boot_wait);
			break;

		case S1000_GLB_NOTIFY_PHRASE_DETECTED:
			kp_data = (struct s1000_kp_payload *) payload;
			trace_printk("***** Phrase Detected **********\n");

			s1000_stream_kp_detected(ctx, kp_data);
			break;

		case S1000_GLB_NOTIFY_EXCEPTION_CAUGHT:
			trace_printk("***** exception **********\n");
			break;

		case S1000_GLB_NOTIFY_MODULE_NOTIFICATION:
			dev_dbg(dev, "***** module notfication**********\n");
			trace_printk("***** module notfication**********\n");
			break;

		case S1000_GLB_NOTIFY_DFU_READY:
			trace_printk("DFU ready: %x\n", msg->primary);
			ctx->dfu_ready = true;
			wake_up(&ctx->dfu_ready_wait);
			break;

		default:
			dev_err(dev, "Unhandled error msg=%x\n",
						msg->primary);
			break;
		}
	}
}

static void s1000_process_response(struct s1000 *ctx, struct s1000_msg *msg,
					u32 *payload, size_t size)
{
	struct device *dev = ctx->dev;
	struct s1000_gen_msg *rx_msg;

	trace_printk("In %s line=%d\n", __func__, __LINE__);

	rx_msg = s1000_find_msg(ctx, msg->primary);
	if (rx_msg == NULL)
		dev_info(dev, "no matching entry found in tx_list\n");


	switch (S1000_GLB_NOTIFY_MSG_TYPE(msg->primary)) {
	case S1000_GLB_MSG_ROM_CONTROL:
		{
			switch ((msg->primary & S1000_GLB_REPLY_STATUS_MASK)) {
			case S1000_ROM_CONTROL_LOAD:
				trace_printk("Load messsage\n");
				ctx->boot_complete = true;
				wake_up(&ctx->boot_wait);
				break;

			case S1000_ROM_CONTROL_ROM_READY:
				trace_printk("Rom Ready\n");
				if (!ctx->boot_mode) {
					ctx->rom_ready = true;
					wake_up(&ctx->rom_ready_wait);
				}
				break;

			case S1000_ROM_CONTROL_MEM_WRITE:
				trace_printk("Mem write\n");
				break;

			default:
				dev_dbg(dev, "Invalid ROM control cmd\n");
				break;
			}
		}
		break;

	case S1000_MOD_LARGE_CONFIG_SET:
		if ((msg->primary & S1000_GLB_REPLY_STATUS_MASK) == 0)
			trace_printk("large message success\n");
		else
			trace_printk("LARGE CONFIG failed*********\n");
		s1000_complete_msg(ctx, msg, rx_msg, payload, size);
		break;

	case S1000_GLB_MSG_IPCGATEWAY:
		if ((msg->primary & S1000_GLB_REPLY_STATUS_MASK) == 0)
			trace_printk("gateway message success\n");
		else
			trace_printk("gateway failed*********\n");

		s1000_complete_msg(ctx, msg, rx_msg, payload, size);
		break;

	default:
		dev_dbg(dev, "unhandled msg\n");

	}

	if (rx_msg) {
		rx_msg->recv = true;
		list_del(&rx_msg->node);
		list_add_tail(&rx_msg->node, &ctx->rx_list);
		wake_up(&rx_msg->waitq);
	}

}

void s1000_process_msg(struct s1000 *ctx, u32 *tx_data, u32 *rx_data,
				size_t size, bool no_lock)
{
	struct s1000_msg rx_msg = {0};
	u32 payload_size, msg_size, tx_msg_ext;
	u32 *data;

	data = rx_data;
	msg_size = sizeof(struct s1000_msg);

	rx_msg.primary = data[0];
	rx_msg.extension = data[1];
	tx_msg_ext = ntohl(tx_data[1]);
	trace_printk("rx message primary=%x ext=%x\n",
			rx_msg.primary, rx_msg.extension);
	trace_printk("tx message primary=%x ext=%x\n", tx_data[0], tx_data[1]);

	/* check if message indicates a 4k mode switch */
	if ((rx_msg.extension & BIT_EXT_WRITE4)
		|| (tx_msg_ext & BIT_EXT_WRITE4)) {
		trace_printk("switching to 4k mode\n");
		ctx->large_msg = true;
	} else {
		ctx->large_msg = false;
	}

	/* check if NULL msg */
	if (rx_msg.primary == 0)
		goto out;

	payload_size = size - msg_size;

	if (S1000_GLB_NOTIFY_MSG_TYPE(rx_msg.primary) ==
				S1000_GLB_MSG_NOTIFICATION)
		s1000_process_notfication(ctx, &rx_msg, &data[2], payload_size);
	else
		s1000_process_response(ctx, &rx_msg, &data[2], payload_size);
out:
	if (list_empty(&ctx->tx_list) && !ctx->expect_notif) {
		if (!rx_data[0] && !tx_data[0])
			s1000_set_state(ctx, S1000_STATE_IDLE);
		else
			s1000_set_state(ctx, S1000_STATE_ACTIVE);
	}
	trace_printk("changing the state=%d expect_notif=%d tx=%d rx=%d\n",
			ctx->state, ctx->expect_notif, tx_data[0], rx_data[0]);

}

static u32 s1000_get_extension_bit(u32 hdr)
{
	switch (S1000_GLB_NOTIFY_MSG_TYPE(hdr)) {
	case S1000_GLB_MSG_IPCGATEWAY:
		if ((hdr & S1000_IPCGATEWAY_CMD_MASK) ==
				S1000_GTW_CMD_SET_DATA) {
			trace_printk("extension bit for cmd=%d, SET_DATA\n",
					(hdr & S1000_IPCGATEWAY_CMD_MASK));
			return BIT_EXT_READ4;
		} else {
			return BIT_EXT_WRITE4;
		}
		break;
	case S1000_MOD_LARGE_CONFIG_SET:
		return BIT_EXT_READ4;

	default:
		return 0;

	}

	return 0;
}

static struct s1000_gen_msg *s1000_switch_to_large_msg(struct s1000 *ctx,
								u32 *data)
{
	struct s1000_msg switch_msg = {0};
	struct s1000_gen_msg *msg;
	u32 hdr;

	trace_printk("In %s line=%d\n", __func__, __LINE__);
	msg = get_msg_from_pool(ctx);

	if (msg == NULL) {
		dev_err(ctx->dev, "%s msg not available in pool\n", __func__);
		return NULL;
	}
	hdr = data[0];
	msg->tx_size = S1000_SHORT_MSG_SIZE;
	msg->rx_size = S1000_LARGE_MSG_SIZE;
	msg->recv = false;
	msg->tx_done = false;
	msg->err_code = 0;
	msg->wait = false;
	msg->raw_msg = false;
	memcpy(&switch_msg.primary, &hdr, sizeof(hdr));
	switch_msg.extension |= s1000_get_extension_bit(hdr);
	memset(msg->tx_data, 0, S1000_SHORT_MSG_SIZE);
	memcpy(msg->tx_data, &switch_msg, sizeof(switch_msg));

	return msg;
}

static int s1000_build_msg(struct device *dev, struct s1000_msg msg,
		void *payload, int payload_size, u32 *data)
{
	u32 msg_size;

	msg_size = sizeof(struct s1000_msg)/sizeof(u32);
	memcpy(data, &msg, sizeof(msg));
	if (payload_size > 0)
		memcpy(data + msg_size, payload, payload_size);
	return 0;
}


int s1000_rom_cntrl_msg(struct s1000 *ctx, u32 val,
			u32 *param, u32 cmd, bool wait)
{
	struct s1000_msg msg = {0};
	u32 *tx_data, *rx_data;
	int ret = 0, size, pl_count;

	msg.primary = S1000_MSG_BUSY_MASK;
	msg.primary |= S1000_GLB_TYPE(S1000_GLB_MSG_ROM_CONTROL);
	msg.primary |= S1000_MSG_SET_CMD(cmd);

	msg.extension = S1000_MSG_SET_FLAGS(val);
	msg.extension |= S1000_MSG_SET_CLOCK(val);
	msg.extension |= S1000_MSG_SET_PL_COUNT(val);

	trace_printk("primary = %x ext = %x", msg.primary, msg.extension);

	pl_count = S1000_MSG_SET_PL_COUNT(val);
	size = sizeof(msg) + (pl_count * 4);
	tx_data = kzalloc(S1000_LARGE_MSG_SIZE, GFP_KERNEL);

	if (!tx_data) {
		ret = -ENOMEM;
		goto free_mem;
	}

	rx_data = kzalloc(S1000_LARGE_MSG_SIZE, GFP_KERNEL);

	if (!rx_data) {
		ret = -ENOMEM;
		goto free_mem;
	}

	ret = s1000_build_msg(ctx->dev, msg, param, pl_count * 4, tx_data);
	if (ret < 0)
		return ret;

	ret = push_request(ctx, tx_data, S1000_SHORT_MSG_SIZE, rx_data,
				S1000_SHORT_MSG_SIZE, wait, false);
	if (ret < 0)
		dev_err(ctx->dev, "%s err = %d\n", __func__, ret);

free_mem:
	kfree(tx_data);
	kfree(rx_data);
	return ret;
}

int s1000_set_ipcgateway_msg(struct s1000 *ctx, u32 param_size, u32 *param)
{
	struct s1000_msg msg = {0};
	u32 *tx_data, *rx_data;
	int ret;

	msg.primary = S1000_MSG_TARGET(S1000_FW_GEN_MSG);
	msg.primary |= S1000_MSG_DIR(S1000_MSG_REQUEST);
	msg.primary |= S1000_MSG_BUSY_MASK;
	msg.primary |= S1000_GLB_TYPE(S1000_GLB_MSG_IPCGATEWAY);
	msg.primary |= S1000_IPCGATEWAY_CMD(S1000_GTW_CMD_SET_DATA);

	msg.extension = S1000_IPCGTW_DATA_OFFSET_CMD(param_size);

	trace_printk("primary=%x ext=%x\n", msg.primary, msg.extension);

	tx_data = kzalloc(S1000_LARGE_MSG_SIZE, GFP_KERNEL);
	if (!tx_data) {
		ret = -ENOMEM;
		goto free_mem;
	}

	rx_data = kzalloc(S1000_LARGE_MSG_SIZE, GFP_KERNEL);
	if (!rx_data) {
		ret = -ENOMEM;
		goto free_mem;
	}

	ret = s1000_build_msg(ctx->dev, msg, param, param_size + sizeof(u32),
						tx_data);
	if (ret < 0)
		return ret;

	ret = push_request(ctx, tx_data, S1000_LARGE_MSG_SIZE,
				rx_data, S1000_LARGE_MSG_SIZE, true, false);
	if (ret < 0) {
		dev_err(ctx->dev, "%s err = %d\n", __func__, ret);
		goto free_mem;
	}

	ret =  rx_data[2];
free_mem:
	kfree(tx_data);
	kfree(rx_data);
	return ret;
}

int s1000_get_ipcgateway_msg(struct s1000 *ctx, u32 param_size,
				u32 *param, u32 *data, u32 *sz_left)
{
	struct s1000_msg msg = {0};
	u32 *tx_data, *rx_data;
	int ret;

	msg.primary = S1000_MSG_TARGET(S1000_FW_GEN_MSG);
	msg.primary |= S1000_MSG_DIR(S1000_MSG_REQUEST);
	msg.primary |= S1000_MSG_BUSY_MASK;
	msg.primary |= S1000_GLB_TYPE(S1000_GLB_MSG_IPCGATEWAY);
	msg.primary |= S1000_IPCGATEWAY_CMD(S1000_GTW_CMD_GET_DATA);

	msg.extension |= S1000_IPCGTW_DATA_OFFSET_CMD(param_size);

	trace_printk("primary=%x ext=%x\n", msg.primary, msg.extension);

	tx_data = kzalloc(S1000_LARGE_MSG_SIZE, GFP_KERNEL);
	if (!tx_data) {
		ret = -ENOMEM;
		goto free_mem;
	}

	rx_data = kzalloc(S1000_LARGE_MSG_SIZE, GFP_KERNEL);
	if (!rx_data) {
		ret = -ENOMEM;
		goto free_mem;
	}

	ret = s1000_build_msg(ctx->dev, msg, param, param_size, tx_data);
	if (ret < 0)
		return ret;

	ret = push_request(ctx, tx_data, S1000_SHORT_MSG_SIZE,
				rx_data, S1000_LARGE_MSG_SIZE, true, false);
	if (ret < 0) {
		dev_err(ctx->dev, "%s err = %d\n", __func__, ret);
		goto free_mem;
	}
	ret = rx_data[1] - sizeof(u32);

	*sz_left = rx_data[2];
	trace_printk("response GET cmd received=%d\n", ret);
	memcpy(data, &rx_data[3], ret);

	trace_printk("Response: 0x%x 0x%x 0x%x 0x%x\n", rx_data[0],
			rx_data[1], rx_data[2], rx_data[3]);
	trace_printk("Response: 0x%x 0x%x 0x%x 0x%x\n",
		rx_data[1020], rx_data[1021], rx_data[1022], rx_data[1023]);

free_mem:
	kfree(tx_data);
	kfree(rx_data);
	return ret;
}

int s1000_set_large_cfg_msg(struct s1000 *ctx, u32 mod_id, u32 param_id,
				u32 param_size, u32 *param)
{
	struct s1000_msg msg = {0};
	u32 *tx_data, *rx_data;
	int ret, data_size, offset, payload_sz;
	int chunk_size;
	int xfer_size = S1000_SHORT_MSG_SIZE;

	trace_printk("In %s param_size=%d\n", __func__, param_size);
	msg.primary = S1000_MSG_TARGET(S1000_MOD_MSG);
	msg.primary |= S1000_MSG_BUSY_MASK;
	msg.primary |= S1000_GLB_TYPE(S1000_MOD_LARGE_CONFIG_SET);
	msg.primary |= S1000_MSG_SET_MOD_ID(mod_id);

	msg.extension = S1000_DATA_OFFSET_SZ(param_size);
	msg.extension |= S1000_LARGE_PARAM_ID(param_id);
	msg.extension |= S1000_FINAL_BLOCK(0);
	msg.extension |= S1000_INITIAL_BLOCK(1);

	if ((sizeof(msg) + (param_size * 4)) > S1000_SHORT_MSG_SIZE)
		xfer_size = S1000_LARGE_MSG_SIZE;

	data_size = param_size;
	chunk_size = xfer_size - sizeof(msg);
	offset = 0;

	trace_printk("primary=%x ext=%x xfer_size=%d\n", msg.primary,
					msg.extension, xfer_size);
	tx_data = kzalloc(S1000_LARGE_MSG_SIZE, GFP_KERNEL);
	if (!tx_data) {
		ret = -ENOMEM;
		goto free_mem;
	}

	rx_data = kzalloc(S1000_LARGE_MSG_SIZE, GFP_KERNEL);
	if (!rx_data) {
		ret = -ENOMEM;
		goto free_mem;
	}

	trace_printk("in %s line=%d\n", __func__, __LINE__);
	while (data_size >= 0) {
		payload_sz = data_size > chunk_size
			? chunk_size : data_size;
		if (payload_sz == data_size)
			msg.extension |= S1000_FINAL_BLOCK(1);

		msg.extension |= S1000_DATA_OFFSET_SZ(param_size);
		memset(tx_data, 0, xfer_size);
		memset(rx_data, 0, xfer_size);
		ret = s1000_build_msg(ctx->dev, msg, param,
				payload_sz, tx_data);
		if (ret < 0)
			return ret;

		ret = push_request(ctx, tx_data, xfer_size, rx_data,
					S1000_LARGE_MSG_SIZE, true, false);

		if (ret < 0) {
			dev_err(ctx->dev, "%s err = %d\n", __func__, ret);
			goto free_mem;
		}

		offset = param_size - payload_sz;
		data_size -= payload_sz;

		msg.extension &= S1000_INITIAL_BLOCK_CLEAR;
		msg.extension &= S1000_DATA_OFFSET_SZ_CLEAR;

		msg.extension |= S1000_INITIAL_BLOCK(0);
		msg.extension |= S1000_DATA_OFFSET_SZ(offset);

		if (data_size == 0)
			break;
	};

free_mem:
	kfree(tx_data);
	kfree(rx_data);
	return ret;
}

#define S1000_MEGACFG_CHUNK_SIZE 4084

int s1000_set_megalarge_cfg_msg(struct s1000 *ctx, u32 mod_id, u32 param_id,
				u32 param_size, u32 *param)
{
	struct s1000_msg msg = {0};
	u32 *tx_data, *rx_data;
	int ret, data_size, offset, payload_sz;
	int chunk_size = S1000_MEGACFG_CHUNK_SIZE;
	int xfer_size = S1000_LARGE_MSG_SIZE;
	struct s1000_mega_cfg_payload *payload;

	trace_printk("In %s param_size=%d\n", __func__, param_size);
	msg.primary = S1000_MSG_TARGET(S1000_MOD_MSG);
	msg.primary |= S1000_MSG_BUSY_MASK;
	msg.primary |= S1000_GLB_TYPE(S1000_MOD_LARGE_CONFIG_SET);
	msg.primary |= S1000_MSG_SET_MOD_ID(mod_id);

	msg.extension = S1000_DATA_OFFSET_SZ(param_size);
	msg.extension |= S1000_LARGE_PARAM_ID(param_id);
	msg.extension |= S1000_FINAL_BLOCK(0);
	msg.extension |= S1000_INITIAL_BLOCK(1);

	if (xfer_size == S1000_LARGE_MSG_SIZE)
		msg.extension |= BIT_EXT_READ4;


	data_size = param_size;
	offset = 0;

	trace_printk("primary=%x ext=%x xfer_size=%d\n", msg.primary,
					msg.extension, xfer_size);

	payload = kzalloc((sizeof(payload)) + chunk_size, GFP_KERNEL);
	if (!payload)
		return -ENOMEM;

	tx_data = kzalloc(S1000_LARGE_MSG_SIZE, GFP_KERNEL);
	if (!tx_data) {
		ret = -ENOMEM;
		goto free_mem;
	}

	rx_data = kzalloc(S1000_LARGE_MSG_SIZE, GFP_KERNEL);
	if (!rx_data) {
		ret = -ENOMEM;
		goto free_mem;
	}

	/* Expects first block with offset as param size and not 0 */
	payload->offset = param_size;

	while (data_size >= 0) {
		payload_sz = data_size > chunk_size
			? chunk_size : data_size;
		if (payload_sz == data_size)
			msg.extension |= S1000_FINAL_BLOCK(1);

		msg.extension |= S1000_DATA_OFFSET_SZ(offset);
		memset(tx_data, 0, xfer_size);
		memset(rx_data, 0, xfer_size);

		memcpy(payload->data, param + (offset / 4), payload_sz);

		ret = s1000_build_msg(ctx->dev, msg, payload,
			payload_sz + sizeof(payload->offset), tx_data);
		if (ret < 0)
			goto free_mem;

		ret = push_request(ctx, tx_data, xfer_size, rx_data,
					S1000_LARGE_MSG_SIZE, true, false);

		if (ret < 0) {
			dev_err(ctx->dev, "%s err = %d\n", __func__, ret);
			goto free_mem;
		}

		/* Expects from 2nd block the actual offset */
		offset += payload_sz;
		data_size -= payload_sz;
		payload->offset = offset;

		if (data_size == 0)
			break;

		msg.extension &= S1000_INITIAL_BLOCK_CLEAR;
		msg.extension &= S1000_DATA_OFFSET_SZ_CLEAR;

		msg.extension |= S1000_INITIAL_BLOCK(0);
		msg.extension |= S1000_DATA_OFFSET_SZ(offset);
	};

free_mem:
	kfree(tx_data);
	kfree(rx_data);
	kfree(payload);
	return ret;
}
static void post_message(struct work_struct *work)
{
	struct s1000_gen_msg *msg, *switch_msg, *tx_msg;
	u32 *tx_data, *rx_data;
	struct s1000 *ctx =
			container_of(work, struct s1000, kwork);
	u32 size;
	int ret;
	bool null_xfer = false;
	bool raw_msg = false;

	mutex_lock(&ctx->thread_lock);
	trace_printk("post message msg_send=%d\n", ctx->msg_send);
	ctx->count_thread++;
	if (!ctx->msg_send) {
		mutex_unlock(&ctx->thread_lock);
		return;
	}

	if (!list_empty(&ctx->tx_list)) {
		ctx->expect_notif = false;
		tx_msg = list_first_entry(&ctx->tx_list, struct s1000_gen_msg,
					node);
		if (tx_msg->tx_done)
			null_xfer = true;
		trace_printk("tx _messge messgae mode=%p\n", tx_msg);
	} else {
		if (ctx->state == S1000_STATE_ACTIVE && !ctx->xfer_raw_mode) {
			null_xfer = true;
		} else {
			trace_printk("nothing to send gpio=%d\n", S1000_GPIO_VAL);
			if ((ctx->count_rising_irq + ctx->count_req ==
			     ctx->count_thread) && S1000_GPIO_VAL) {
				ctx->state = S1000_STATE_ACTIVE;
				ctx->expect_notif = true;
				s1000_register_irq_hanlder(ctx, IRQF_TRIGGER_FALLING);
				if (S1000_GPIO_VAL) {
					trace_printk("going to IDLE.\n");
					mutex_unlock(&ctx->thread_lock);
					return;
				}
			}
			null_xfer = true;
			if (!S1000_GPIO_VAL) {
				ctx->expect_notif = true;
				ctx->state = S1000_STATE_ACTIVE;
			}
		}
	}

	if (ctx->expect_notif == true) {
		trace_printk("when expect_notif is true GPIO is %d\n",
							S1000_GPIO_VAL);
		s1000_register_irq_hanlder(ctx, IRQF_TRIGGER_RISING);
	}

	trace_printk("Sending Data: null_transfe=%d state=%d expect_notif=%d gpio=%d\n",
		null_xfer, ctx->state, ctx->expect_notif, S1000_GPIO_VAL);
	ctx->msg_send = false;

	if (ctx->expect_notif == true && S1000_GPIO_VAL == 1) {
		trace_printk("making the expect_notif to false\n");
		ctx->expect_notif = false;
	}

	if (null_xfer) {
		if (ctx->large_msg)
			size = S1000_LARGE_MSG_SIZE;
		else
			size = S1000_SHORT_MSG_SIZE;

		tx_data = kzalloc(size, GFP_KERNEL);

		if (!tx_data) {
			mutex_unlock(&ctx->thread_lock);
			return;
		}

		rx_data = kzalloc(size, GFP_KERNEL);

		if (!rx_data) {
			kfree(tx_data);
			mutex_unlock(&ctx->thread_lock);
			return;
		}
	} else {
		s1000_set_state(ctx, S1000_STATE_ACTIVE);
		trace_printk("check if the message has to be switched\n");
		/* check for 4k mode switch */
		if (!tx_msg->mode_switch && !ctx->xfer_raw_mode &&
			 (tx_msg->tx_size >  S1000_SHORT_MSG_SIZE)) {
			if (ctx->large_msg) {
				trace_printk("already large message, just set extension bit\n");
				tx_msg->tx_data[1] |=
					s1000_get_extension_bit(tx_msg->tx_data[0]);
				msg = tx_msg;
			} else {
				trace_printk("creating switch msg\n");
				switch_msg = s1000_switch_to_large_msg(ctx, tx_msg->tx_data);
				if (!switch_msg) {
					ret = -EBUSY;
					goto err;
				}
				msg = switch_msg;
				tx_msg->mode_switch = true;
				list_add(&switch_msg->node, &ctx->tx_list);
			}
		} else {
			msg = tx_msg;
			tx_msg->mode_switch = false;
		}
		tx_data = msg->tx_data;
		rx_data = msg->rx_data;

		size = msg->tx_size;
		raw_msg = msg->raw_msg;
		trace_printk("messgae mode=%p\n", msg);
	}

	if (ctx->expect_notif == true && S1000_GPIO_VAL == 1) {
		trace_printk("making the expect_notif to false\n");
		ctx->expect_notif = false;
	}

	/*
	 * Add message to RX list after posting
	 * the message.
	 */
	ret = s1000_spi_transfer(ctx->spi, tx_data,
			rx_data, size, true);
	if (ret < 0)
		goto err;

	if (!raw_msg) {
		if (!null_xfer)
			msg->tx_done = true;
		s1000_process_msg(ctx, tx_data, rx_data, size, false);
	}

	if (!null_xfer && !msg->wait) {
		list_del(&msg->node);
		list_add_tail(&msg->node, &ctx->msg_pool_list);
	}

	if (null_xfer) {
		kfree(tx_data);
		kfree(rx_data);
	}

	mutex_unlock(&ctx->thread_lock);
	trace_printk("post messgae done......\n");
	return;

err:
	if (null_xfer) {
		kfree(tx_data);
		kfree(rx_data);
	} else {
		msg->err_code = ret;
		msg->recv = true;
		if (msg->wait) {
			wake_up(&msg->waitq);
		} else {
			list_del(&msg->node);
			list_add_tail(&msg->node, &ctx->msg_pool_list);
		}
	}
	mutex_unlock(&ctx->thread_lock);
}

int push_request(struct s1000 *ctx, u32 *data, u32 data_size,
			u32 *rx_data, u32 rx_size,
			int wait, bool raw_msg)
{
	struct s1000_gen_msg *msg;
	int ret;

	mutex_lock(&ctx->thread_lock);
	msg = get_msg_from_pool(ctx);

	if (msg == NULL) {
		mutex_unlock(&ctx->thread_lock);
		return -EBUSY;
	}

	msg->tx_size = data_size;
	msg->rx_size = rx_size;
	msg->recv = false;
	msg->tx_done = false;
	msg->err_code = 0;
	msg->wait = wait;
	msg->raw_msg = raw_msg;
	memcpy(msg->tx_data, data, data_size);

	list_add_tail(&msg->node, &ctx->tx_list);
	trace_printk("added to tx list\n");
	if (ctx->expect_notif) {
		s1000_register_irq_hanlder(ctx, IRQF_TRIGGER_RISING);
		if (schedule_work(&ctx->kwork))
			ctx->count_req++;
		trace_printk("request count count_req=%d\n", ctx->count_req);
	}
	mutex_unlock(&ctx->thread_lock);

	if (wait) {
		trace_printk("waiting .....for response\n");
		ret  = wait_event_timeout(msg->waitq, msg->recv,
				msecs_to_jiffies(S1000_BOOT_MSECS));
		mutex_lock(&ctx->thread_lock);
		if (list_empty(&ctx->rx_list))
			dev_err(ctx->dev, "In %s rx list empty\n", __func__);

		if (ret == 0) {
			ret = -ETIMEDOUT;
		} else {
			if (msg->rx_size && msg->err_code >= 0)
				memcpy(rx_data, msg->rx_data, msg->rx_size);

			ret = msg->err_code;
			dev_info(ctx->dev, "Msg received for msg 0x%x\n",
						msg->tx_data[0]);
			trace_printk("Msg received for msg 0x%x\n",
						msg->tx_data[0]);
		}

		list_del(&msg->node);
		list_add_tail(&msg->node, &ctx->msg_pool_list);
		if (!list_empty(&ctx->rx_list))
			dev_err(ctx->dev, "In rx is not list empty...\n");

		mutex_unlock(&ctx->thread_lock);

		return ret;
	}

	return 0;
}

static int init_msg_pool(struct s1000 *ctx)
{
	int i, ret;

	ctx->msg = kzalloc(sizeof(*ctx->msg) * S1000_POOL_SIZE,
					GFP_KERNEL);

	for (i = 0; i < S1000_POOL_SIZE; i++) {
		ctx->msg[i].tx_data = kzalloc(S1000_LARGE_MSG_SIZE, GFP_KERNEL);
		if (ctx->msg[i].tx_data == NULL) {
			ret = -ENOMEM;
			goto free_mem;
		}

		ctx->msg[i].rx_data = kzalloc(S1000_LARGE_MSG_SIZE, GFP_KERNEL);
		if (ctx->msg[i].rx_data == NULL) {
			ret = -ENOMEM;
			goto free_mem;
		}

		init_waitqueue_head(&ctx->msg[i].waitq);
		list_add(&ctx->msg[i].node, &ctx->msg_pool_list);
	}
	return 0;

free_mem:
	while (i > 0) {
		kfree(ctx->msg[i-1].tx_data);
		kfree(ctx->msg[i-1].rx_data);
		--i;
	}
	kfree(ctx->msg);
	return ret;
}

int s1000_msg_init(struct s1000 *ctx)
{
	int ret;

	INIT_LIST_HEAD(&ctx->tx_list);
	INIT_LIST_HEAD(&ctx->rx_list);
	INIT_LIST_HEAD(&ctx->msg_pool_list);
	ret = init_msg_pool(ctx);
	if (ret < 0)
		return ret;

	/* init the post message thread */
	INIT_WORK(&ctx->kwork, post_message);

	return 0;
}

void s1000_msg_cleanup(struct s1000 *ctx)
{
	int i = S1000_POOL_SIZE;

	while (i > 0) {
		kfree(ctx->msg[i-1].tx_data);
		kfree(ctx->msg[i-1].rx_data);
		--i;
	}
	kfree(ctx->msg);
}

irqreturn_t s1000_dsp_trigger_rising_handler(int irq, void *dev_id)
{
	struct s1000 *ctx = dev_id;

	if (ctx->irq == irq) {
		ctx->msg_send = true;
		if (schedule_work(&ctx->kwork))
			ctx->count_rising_irq++;
		else
			ctx->count_rising_miss++;
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}
