/*
 *  s1000-dsp.c - Implementation of S1000 DSP helpers.
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
#include <crypto/hash.h>
#include "s1000.h"
#include "s1000-dsp-msg.h"

int s1000_stream_control_op(struct s1000 *ctx, int cmd,
			    struct s1000_stream *stream)
{
	if (stream == NULL)
		return s1000_set_large_cfg_msg(ctx, ctx->mgm_id, cmd, 0, NULL);
	else
		return s1000_set_large_cfg_msg(ctx, ctx->mgm_id, cmd,
				sizeof(stream->id), (u32 *)&stream->id);
}

int s1000_write_data(struct s1000 *ctx, u32 res_id, void *data, size_t size)
{
	struct s1000_gtw_payload *param;
	int ret;

	param = kzalloc((sizeof(param->node_id)) + size, GFP_KERNEL);
	if (!param)
		return -ENOMEM;

	param->node_id = res_id;
	memcpy(param->data, data, size);

	ret = s1000_set_ipcgateway_msg(ctx, size, (u32 *)param);
	kfree(param);
	return ret;
}

int s1000_read_data(struct s1000 *ctx, u32 res_id, void *data,
				    size_t size, u32 *sz_left)
{
	struct s1000_gtw_payload *param;
	int ret;

	param = kzalloc((sizeof(param)) + size, GFP_KERNEL);
	if (!param)
		return -ENOMEM;
	param->node_id = res_id;
	memset(param->data, 0, size);

	ret = s1000_get_ipcgateway_msg(ctx, size, (u32 *)param, data, sz_left);
	if (ret < 0)
		return ret;

	trace_printk("size left =%d\n", *sz_left);
	kfree(param);

	return ret;
}

static int s1000_boot_dsp(struct device *dev, bool enable)
{
	int status;

	status = gpio_direction_output(S1000_HOST_RST_N, enable);
	if (status)
		dev_err(dev, "Boot dsp failed, gpio set failed%d status=%d\n",
				S1000_HOST_RST_N, status);
	trace_printk("In %s value=%d\n", __func__,
			gpio_get_value(S1000_HOST_RST_N));

	return status;
}

int s1000_firmware_upgrade(struct s1000 *ctx,
			   const void *fw, size_t size)
{
	int ret;

	/* Step1: Make sure pipeline are deleted. call toplogy stop cmd */
	ret = s1000_set_large_cfg_msg(ctx, ctx->mgm_id, S1000_CMD_TPLG_STOP,
						0, NULL);
	if (ret < 0)
		return ret;

	/* Step2: Firmware update start */
	ret = s1000_set_large_cfg_msg(ctx, ctx->mgm_id,
				      S1000_CMD_FW_UPDATE_START,
				      0, NULL);
	if (ret < 0)
		return ret;

	/* Step3: wait for DFU ready notification */
	ctx->dfu_ready = false;
	ret  = wait_event_timeout(ctx->dfu_ready_wait, ctx->dfu_ready,
					  msecs_to_jiffies(S1000_BOOT_MSECS));
	if (ret == 0) {
		dev_err(ctx->dev, "FW Update failed, DFU Ready timed-out\n");
		return -EIO;
	}

	/* Step4: Firmware update set data */
	ret = s1000_set_megalarge_cfg_msg(ctx, 0x1001,
					S1000_CMD_START,
					size, (u32 *)fw);
	if (ret < 0) {
		/* Reset DSP in case of failure to boot from flash */
		ret = s1000_boot_dsp(ctx->dev, false);
		if (ret < 0)
			return -ENOTRECOVERABLE;
		ret = s1000_boot_dsp(ctx->dev, true);
		if (ret < 0)
			return -ENOTRECOVERABLE;
	}

	/* step5: wait for firmware ready Notification */
	ctx->boot_complete = false;
	ret  = wait_event_timeout(ctx->boot_wait, ctx->boot_complete,
				msecs_to_jiffies(S1000_BOOT_MSECS));
	if (ret == 0) {
		dev_err(ctx->dev, "DSP boot failed, FW Ready timed-out\n");
		return -ENOTRECOVERABLE;
	}

	msleep(10);
	/* Step6: toplogy start cmd */
	if (ctx->tplg_loaded)
		ret = s1000_set_large_cfg_msg(ctx, ctx->mgm_id,
					      S1000_CMD_TPLG_START, 0, NULL);
	if (ret < 0)
		return -ENOTRECOVERABLE;

	return ret;
}

#define S1000_MEM_ADDR_SRAM_RETENTION_DELAY 0x304628
#define S1000_MEM_ADDR_SRAM1_RETENTION_DISABLE 0x71d14
#define S1000_MEM_ADDR_SRAM2_RETENTION_DISABLE 0x71d24
#define S1000_MEM_ADDR_WRITEBACK_L2_DISABLE 0x301000

static int s1000_set_retention_policy(struct s1000 *ctx)
{
	struct s1000_rom_mem_info rom_mem[4];
	union s1000_fw_load_flags ext = {0};

	rom_mem[0].addr = S1000_MEM_ADDR_SRAM_RETENTION_DELAY;
	rom_mem[0].value = 0x4f;
	rom_mem[1].addr = S1000_MEM_ADDR_SRAM1_RETENTION_DISABLE;
	rom_mem[1].value = 0x0;
	rom_mem[2].addr = S1000_MEM_ADDR_SRAM2_RETENTION_DISABLE;
	rom_mem[2].value = 0x0;
	rom_mem[3].addr = S1000_MEM_ADDR_WRITEBACK_L2_DISABLE;
	rom_mem[3].value = 0x20020608;
	ext.flags.count = sizeof(rom_mem)/4;

	return s1000_rom_cntrl_msg(ctx, ext.val, (u32 *)&rom_mem,
				S1000_ROM_CONTROL_MEM_WRITE, true);
}

/* For SHA256, we require an output buffer of 256 bits */
#define S1000_KEY_SIZE   32

static int s1000_set_sha256(struct device *dev, const void *fw,
					int size, u32 *data)
{
	struct crypto_shash *tfm;
	struct shash_desc *shash;
	int ret;
	u8 *key = (u8 *)fw;
	u8 *output = (u8 *)data;

	if (!size)
		return -EINVAL;

	tfm = crypto_alloc_shash("sha256", 0, 0);
	if (IS_ERR(tfm)) {
		pr_err("crypto_alloc_shash failed: err ");
		return -EINVAL;
	}

	shash = kzalloc(sizeof(*shash) + crypto_shash_descsize(tfm),
			GFP_KERNEL);
	if (!shash) {
		ret = -ENOMEM;
		goto failed;
	}

	shash->tfm = tfm;
	shash->flags = 0;

	ret = crypto_shash_digest(shash, key, size, (u8 *)output);

	kfree(shash);

failed:
	crypto_free_shash(tfm);
	return ret;

}

#define S1000_FW_HPSRAM_OFFSET 0xbe000000
#define S1000_FW_LOAD_CLOCK_SELECT  (1 << 21)

static int s1000_send_load_fw_msg(struct s1000 *ctx, const void *fw,
				  size_t size)
{
	struct s1000_fw_load_data data = {0};
	union s1000_fw_load_flags ext = {0};

	ext.flags.clock_select = S1000_CLOCK_SELECT_SPI_SLAVE;
	data.mem_addr = S1000_FW_HPSRAM_OFFSET;
	data.img_offset = 0;
	data.img_size = size;
	s1000_set_sha256(ctx->dev, fw, size, (u32 *)&data.img_sha);
	ext.flags.count = sizeof(data)/4;

	return s1000_rom_cntrl_msg(ctx, ext.val, (u32 *)&data,
				S1000_ROM_CONTROL_LOAD, false);
}

static int s1000_transfer_firmware(struct s1000 *ctx)
{
	u32 bytes_xfred, bytes_to_xfer;
	void  *tx_buff, *rx_buff;
	u32 chunk = S1000_LARGE_MSG_SIZE;
	int ret;
	struct device *dev = ctx->dev;
	const void *curr_pos = ctx->fw->data;
	size_t size = ctx->fw->size;

	ctx->xfer_raw_mode = true;
	tx_buff = kzalloc(chunk, GFP_KERNEL);
	if (tx_buff == NULL)
		return -ENOMEM;

	rx_buff = kzalloc(chunk, GFP_KERNEL);
	if (rx_buff == NULL) {
		kfree(tx_buff);
		return -ENOMEM;
	}

	dev_dbg(dev, "Start firmware transfer.\n");
	bytes_xfred = 0;
	curr_pos = ctx->fw->data;
	while (bytes_xfred < size) {
		memset(tx_buff, 0, chunk);
		memset(rx_buff, 0, chunk);
		if ((size - bytes_xfred) < chunk) {
			memcpy(tx_buff, curr_pos, size - bytes_xfred);
			bytes_xfred += (size - bytes_xfred);
			bytes_to_xfer = (size - bytes_xfred);
		} else {
			memcpy(tx_buff, curr_pos, chunk);
			bytes_xfred += chunk;
			curr_pos += chunk;
			bytes_to_xfer = chunk;
		}

		ret = s1000_spi_transfer(ctx->spi, tx_buff, rx_buff,
							chunk, true);
		if (ret < 0) {
			dev_err(dev, "firmware transfer failed ret=%d\n", ret);
			goto out;
		}
		dev_dbg(dev, "Transfered %d/%d bytes.\n", bytes_xfred, size);
	}
out:
	kfree(tx_buff);
	kfree(rx_buff);

	return ret;
}

int s1000_load_fw(struct s1000 *ctx)
{
	struct device *dev = ctx->dev;
	int ret = 0;

	if (!ctx->boot_mode) {
		if (ctx->fw == NULL) {
			ret = request_firmware(&ctx->fw, ctx->fw_name, dev);
			if (ret < 0) {
				dev_err(dev, "Request firmware failed %d\n", ret);
				return -EIO;
			}
		}

		/* step1. boot DSP core */
		ret = s1000_boot_dsp(dev, true);
		if (ret < 0)
			goto out;

		/* step2: wait for ROM Ready */
		ctx->boot_complete = false;
		ret  = wait_event_timeout(ctx->rom_ready_wait, ctx->rom_ready,
					  msecs_to_jiffies(S1000_BOOT_MSECS));
		if (ret == 0) {
			dev_err(dev, "DSP boot failed, ROM Ready timed-out\n");
			ret = -EIO;
			goto out;
		}

		trace_printk("setting raw state\n");
		/* step3: Set the retention policy using memwrite debug msg */
		ret = s1000_set_retention_policy(ctx);
		if (ret < 0)
			goto out;

		/* step4: send load firmware message */
		ctx->xfer_raw_mode = true;
		ret = s1000_send_load_fw_msg(ctx, ctx->fw->data, ctx->fw->size);
		if (ret < 0)
			goto out;
		msleep(100);

		/* step5: transfer the firmware image */
		ret = s1000_transfer_firmware(ctx);
		if (ret < 0)
			goto out;

		/* step6: wait for firmware load response */
		/* sleep until all transfers are done */
		msleep(10);
		ctx->xfer_raw_mode = false;

		/* schedule post message to send null data */
		if (schedule_work(&ctx->kwork))
			ctx->count_req++;

		ctx->boot_complete = false;
		ret  = wait_event_timeout(ctx->boot_wait, ctx->boot_complete,
					  msecs_to_jiffies(S1000_BOOT_MSECS));
		if (ret == 0) {
			dev_err(dev, "DSP boot failed, FW load timed-out %d\n",
				ctx->boot_complete);
			ret = -EIO;
			goto out;
		}
	} else {
		/* step1. boot DSP core */
		ret = s1000_boot_dsp(dev, true);
		if (ret < 0)
			return -EIO;
	}
	trace_printk("wait for firmware ready notfication\n");
	/* step7: wait for firmware ready Notification */
	ctx->boot_complete = false;
	ret  = wait_event_timeout(ctx->boot_wait, ctx->boot_complete,
				msecs_to_jiffies(S1000_BOOT_MSECS));
	if (ret == 0) {
		dev_err(dev, "DSP boot failed, FW Ready timed-out\n");
		ret = -EIO;
		goto out;
	}
	return 0;

out:
	s1000_boot_dsp(dev, false);
	if (ctx->fw)
		release_firmware(ctx->fw);
	ctx->fw = NULL;
	return ret;
}

#define S1000_FW_UPGRADE "upgrade\n"
static ssize_t store_upgrade_path(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct s1000 *ctx = dev_get_drvdata(dev);
	int i, ret;

	if (strcmp(buf, S1000_FW_UPGRADE))
		return -EINVAL;

	if (!ctx->boot_mode)
		return -EINVAL;

	if (ctx->dsp_state == S1000_DSP_UPDATING)
		return -EBUSY;

	mutex_lock(&ctx->lock);

	/* checking if any stream is active */
	for (i = 0 ; i < S1000_MAX_STREAMS; i++) {
		if (ctx->streams[i].dai && ctx->streams[i].dai->active) {
			mutex_unlock(&ctx->lock);
			return -EINVAL;
		}
	}

	/* request for new firmware image*/
	ret = request_firmware(&ctx->fw, ctx->fw_name, ctx->dev);
	if (ret < 0) {
		dev_err(ctx->dev, "Request firmware failed %d\n", ret);
		mutex_unlock(&ctx->lock);
		return ret;
	}


	ret = s1000_firmware_upgrade(ctx, ctx->fw->data, ctx->fw->size);
	if (ret < 0)
		dev_err(ctx->dev, "Firmware upgrade failed %d\n", ret);

	ctx->dsp_state = S1000_DSP_RUNNING;

	mutex_unlock(&ctx->lock);
	if (ctx->fw)
		release_firmware(ctx->fw);

	ctx->fw = NULL;

	if (ret < 0)
		return ret;
	else
		return count;
}

static DEVICE_ATTR(fw_upgrade, S_IWUSR, NULL, store_upgrade_path);

/*
 * Adds an attribute "fw_upgrade" in sysfs onto the existing spi class
 * device for the firmware upgrade.
 */
int s1000_sysfs_add_entry(struct s1000 *ctx)
{
	int ret = 0;
	struct spi_device *spi = ctx->spi;

	ret = device_create_file(&spi->dev, &dev_attr_fw_upgrade);
	if (ret != 0)
		dev_err(ctx->dev, "Add sysfs entry failed, ret = %d\n", ret);

	return ret;
}

void s1000_sysfs_remove_entry(struct s1000 *ctx)
{
	struct spi_device *spi = ctx->spi;

	device_remove_file(&spi->dev, &dev_attr_fw_upgrade);
}

int s1000_dsp_init(struct s1000 *ctx)
{
	int ret;

	ret = s1000_msg_init(ctx);
	if (ret < 0)
		return ret;

	init_waitqueue_head(&ctx->boot_wait);

	if (!ctx->boot_mode)
		init_waitqueue_head(&ctx->rom_ready_wait);
	else
		init_waitqueue_head(&ctx->dfu_ready_wait);

	ret = s1000_boot_dsp(ctx->dev, false);
	if (ret < 0)
		return ret;

	ret = request_irq(ctx->irq, s1000_dsp_trigger_rising_handler,
				IRQF_TRIGGER_RISING,
				"s1000-dsp", ctx);
	if (ret) {
		dev_err(ctx->dev, "request irq rising failed");
		return -EIO;
	}

	ctx->flags = IRQF_TRIGGER_RISING;

	ret = s1000_load_fw(ctx);
	if (ret < 0) {
		free_irq(ctx->irq, ctx);
		ctx->irq = -1;
		return ret;
	}

	ret = s1000_sysfs_add_entry(ctx);
	if (ret < 0) {
		free_irq(ctx->irq, ctx);
		ctx->irq = -1;
	}

	ctx->dsp_state = S1000_DSP_RUNNING;

	return ret;
}

void s1000_dsp_cleanup(struct s1000 *ctx)
{
	struct device *dev = ctx->dev;

	cancel_work_sync(&ctx->kwork);

	s1000_msg_cleanup(ctx);

	if (ctx->fw)
		release_firmware(ctx->fw);

	free_irq(ctx->irq, ctx);
	ctx->irq = -1;
	s1000_sysfs_remove_entry(ctx);
	s1000_boot_dsp(dev, false);
}
