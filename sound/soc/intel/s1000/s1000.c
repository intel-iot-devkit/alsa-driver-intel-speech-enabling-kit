/*
 *  s1000.c - ASoC driver for Intel quark s1000.
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
#include "s1000.h"

static int spi_clk_speed = 12600000;
static bool boot_mode = 1;

module_param(spi_clk_speed, int, 0444);
MODULE_PARM_DESC(spi_clk_speed, "SPI max CLK Speed in HZ");
module_param(boot_mode, bool, 0444);
MODULE_PARM_DESC(boot_mode, "Select Boot mode"
			    "(0=SPI_SLAVE, 1=SPI_MASTER) (default=1).");

void s1000_swap_be_le32(u32 *data, int size)
{
	u32 *x = data;

	for ( ; x - data < size; ++x)
		*x = htonl(*x);
}

void s1000_swap_le_be32(u32 *data, int size)
{
	u32 *x = data;

	for ( ; x - data < size; ++x)
		*x = ntohl(*x);
}

void s1000_set_state(struct s1000 *ctx, int state)
{
	mutex_lock(&ctx->mutex);
	ctx->state = state;
	mutex_unlock(&ctx->mutex);
}

int s1000_spi_transfer(struct spi_device *spi, void *tx_buf,
		  void *rx_buf, int size, bool swap)
{
	int ret, i;
	u32 *tx = (u32 *) tx_buf;
	u32 *rx = (u32 *) rx_buf;

	struct spi_transfer t = {
		.tx_buf = tx_buf,
		.len = size,
		.speed_hz = spi_clk_speed,
		.bits_per_word = 8,
		.rx_buf = rx_buf
	};

	trace_printk("SPi transfer ***** size=%d\n", size);
	trace_printk("Transfer:\n");
	for (i = 0; i < 64 / 4; i += 4) {
		trace_printk("0x%08x 0x%08x 0x%08x 0x%08x\n", tx[i],
				tx[i+1], tx[i+2], tx[i+3]);
	}
	if (swap)
		s1000_swap_be_le32((u32 *) tx_buf, size/4);
	ret = spi_sync_transfer(spi, &t, 1);
	if (ret != 0) {
		trace_printk("In %s failed ret=%d\n", __func__, ret);
		return -EIO;
	}

	if (swap)
		s1000_swap_le_be32((u32 *)rx_buf, size/4);

	trace_printk("Response:\n");
	for (i = 0; i < 64 / 4; i += 4) {
		trace_printk("0x%08x 0x%08x 0x%08x 0x%08x\n", rx[i],
				rx[i+1], rx[i+2], rx[i+3]);
	}

	return ret;
}

static int s1000_machine_device_register(struct s1000 *ctx, void *driver_data)
{
	struct platform_device *pdev;
	int ret;


	pdev = platform_device_alloc("s1000_mach", -1);
	if (pdev == NULL) {
		dev_err(ctx->dev, "platform device alloc failed\n");
		return -EIO;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		dev_err(ctx->dev, "failed to add machine device\n");
		platform_device_put(pdev);
		return -EIO;
	}

	ctx->i2s_dev = pdev;

	return 0;
}

static void s1000_machine_device_unregister(struct s1000 *ctx)
{
	if (ctx->i2s_dev)
		platform_device_unregister(ctx->i2s_dev);
}

static int s1000_probe(struct spi_device *spi)
{
	int status, irq;
	struct s1000 *ctx;
	char name[30] = "dsp_fw_s1000.img";


	trace_printk("s1000 probe called\n");

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (ctx == NULL)
		return -ENOMEM;

	ctx->fw_name  = kstrdup(name, GFP_KERNEL);
	if (!ctx->fw_name) {
		kfree(ctx);
		return -ENOMEM;
	}

	/*spi cfg*/
	spi->mode = SPI_MODE_3;
	spi->max_speed_hz = spi_clk_speed;
	spi->bits_per_word = 8;
	spi_setup(spi);

	status = gpio_request(S1000_HOST_IRQ, "s1000_irq");
	if (status < 0) {
		dev_err(&spi->dev, "Could not request for IRQ GPIO:%d\n",
							S1000_HOST_IRQ);
		goto out_free_ctx;
	}

	status = gpio_direction_input(S1000_HOST_IRQ);
	if (status) {
		dev_err(&spi->dev, "Cannot set input IRQ GPIO %d\n",
							S1000_HOST_IRQ);
		goto out_gpio_free;
	}
	gpio_export(S1000_HOST_IRQ, false);
	ctx->spi = spi;
	ctx->dev = &spi->dev;
	ctx->boot_mode = boot_mode;
	irq = gpio_to_irq(S1000_HOST_IRQ);
	if (irq  < 0) {
		dev_err(&spi->dev, "GPIO to IRQ mapping failure %d\n",
					gpio_to_irq(S1000_HOST_IRQ));
		status =  -EIO;
		goto out_gpio_free;
	}
	ctx->irq = irq;
	ctx->state = S1000_STATE_ACTIVE;
	ctx->large_msg = false;
	mutex_init(&ctx->mutex);
	mutex_init(&ctx->thread_lock);
	mutex_init(&ctx->lock);
	ctx->mgm_id = 0x1000;
	spi_set_drvdata(spi, ctx);

	status = s1000_dsp_init(ctx);
	if (status < 0)
		goto out_gpio_free;

	status = s1000_machine_device_register(ctx, NULL);
	if (status < 0)
		goto out_dsp_free;

	/* register platform dai and controls */
	status = s1000_platform_register(&spi->dev);
	if (status < 0)
		goto out_mach_free;

	return 0;

out_mach_free:
	s1000_machine_device_unregister(ctx);
out_dsp_free:
	s1000_dsp_cleanup(ctx);
out_gpio_free:
	gpio_free(S1000_HOST_IRQ);
out_free_ctx:
	kfree(ctx->fw_name);
	kfree(ctx);
	return status;
}

static int s1000_remove(struct spi_device *spi)
{
	struct s1000 *ctx;

	ctx = spi_get_drvdata(spi);
	if (!ctx)
		return 0;

	s1000_machine_device_unregister(ctx);
	s1000_dsp_cleanup(ctx);
	gpio_free(S1000_HOST_IRQ);
	kfree(ctx->fw_name);
	kfree(ctx);
	ctx = NULL;

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id s1000_dt_ids[] = {
	{ .compatible = "s1000", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s1000_dt_ids);
#endif

static struct spi_driver s1000_driver = {
	.driver = {
		.name = "s1000",
		.of_match_table = of_match_ptr(s1000_dt_ids),
	},
	.probe = s1000_probe,
	.remove = s1000_remove,
};

module_spi_driver(s1000_driver);

MODULE_DESCRIPTION("S1000 SPI DRIVER");
MODULE_AUTHOR("Jeeja KP <jeeja.kp@intel.com>");
MODULE_AUTHOR("Shreyas NC <shreyas.nc@intel.com>");
MODULE_LICENSE("Dual BSD/GPL");
