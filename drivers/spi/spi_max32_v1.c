/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max32_spi

#include <string.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include <wrap_max32_spi.h>

LOG_MODULE_REGISTER(spi_max32_v1, CONFIG_SPI_LOG_LEVEL);
#include "spi_context.h"

struct max32_spi_config {
	mxc_spi_regs_t *regs;
	const struct pinctrl_dev_config *pctrl;
	const struct device *clock;
	struct max32_perclk perclk;
};

/* Device run time data */
struct max32_spi_data {
	struct spi_context ctx;
};

static int spi_configure(const struct device *dev, const struct spi_config *config)
{
	int ret = 0;
	const struct max32_spi_config *cfg = dev->config;
	mxc_spi_regs_t *regs = cfg->regs;
	struct max32_spi_data *data = dev->data;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (SPI_OP_MODE_GET(config->operation) & SPI_OP_MODE_SLAVE) {
		return -ENOTSUP;
	}

	int masterMode = 1;
	int quadModeUsed = 0;
	int numSlaves = 1;
	int ssPolarity = 0;
	unsigned int spi_speed = (unsigned int)config->frequency;

	ret = Wrap_MXC_SPI_Init(regs, masterMode, quadModeUsed, numSlaves, ssPolarity, spi_speed);
	if (ret) {
		return ret;
	}

	int cpol = (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) ? 1 : 0;
	int cpha = (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) ? 1 : 0;

	if (cpol && cpha) {
		ret = MXC_SPI_SetMode(regs, SPI_MODE_3);
	} else if (cpha) {
		ret = MXC_SPI_SetMode(regs, SPI_MODE_2);
	} else if (cpol) {
		ret = MXC_SPI_SetMode(regs, SPI_MODE_1);
	} else {
		ret = MXC_SPI_SetMode(regs, SPI_MODE_0);
	}
	if (ret) {
		return ret;
	}

	ret = MXC_SPI_SetDataSize(regs, SPI_WORD_SIZE_GET(config->operation));
	if (ret) {
		return ret;
	}

#if defined(CONFIG_SPI_EXTENDED_MODES)
	switch (config->operation & SPI_LINES_MASK) {
	case SPI_LINES_QUAD:
		ret = MXC_SPI_SetWidth(regs, SPI_WIDTH_QUAD);
		break;
	case SPI_LINES_DUAL:
		ret = MXC_SPI_SetWidth(regs, SPI_WIDTH_DUAL);
		break;
	case SPI_LINES_OCTAL:
		ret = -ENOTSUP; /* MXC_SPI_SetWidth(regs, SPI_WIDTH_3WIRE); */
		break;
	case SPI_LINES_SINGLE:
	default:
		ret = MXC_SPI_SetWidth(regs, SPI_WIDTH_STANDARD);
		break;
	}

	if (ret) {
		return ret;
	}
#endif

	data->ctx.config = config;

	return ret;
}

static int spi_max32_transceive(const struct device *dev)
{
	int ret;
	const struct max32_spi_config *cfg = dev->config;
	struct max32_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	mxc_spi_req_t req;
	uint32_t len;
	uint8_t word_size, dfs_shift;

	word_size = SPI_WORD_SIZE_GET(ctx->config->operation);
	if (word_size < 9) {
		/* DFS = 1 */
		dfs_shift = 0;
	} else {
		/* DFS = 2 */
		dfs_shift = 1;
	}

	len = spi_context_max_continuous_chunk(ctx);
	req.txLen = len >> dfs_shift;
	req.txData = (uint8_t *)ctx->tx_buf;
	req.rxLen = len >> dfs_shift;
	req.rxData = ctx->rx_buf;

	req.spi = cfg->regs;
	req.ssIdx = ctx->config->slave;
	req.ssDeassert = 1;
	req.txCnt = 0;
	req.rxCnt = 0;

	ret = MXC_SPI_MasterTransaction(&req);
	if (ret) {
		ret = -EIO;
	} else {
		spi_context_update_tx(ctx, 1, len);
		spi_context_update_rx(ctx, 1, len);
	}

	return ret;
}

static int api_transceive(const struct device *dev, const struct spi_config *config,
			  const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	int ret = 0;
	const struct max32_spi_config *cfg = dev->config;
	struct max32_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	bool hw_cs_ctrl = true;

	spi_context_lock(ctx, false, NULL, NULL, config);

	ret = spi_configure(dev, config);
	if (ret != 0) {
		spi_context_release(ctx, ret);
		return -EIO;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

	/* Check if CS GPIO exists */
	if (spi_cs_is_gpio(config)) {
		hw_cs_ctrl = false;
	}
	MXC_SPI_HWSSControl(cfg->regs, hw_cs_ctrl);

	/* Assert the CS line if HW control disabled */
	if (!hw_cs_ctrl) {
		spi_context_cs_control(&data->ctx, true);
	}

	do {
		ret = spi_max32_transceive(dev);
		if (ret != 0) {
			break;
		}
	} while (spi_context_tx_on(ctx) || spi_context_rx_on(ctx));

	/* Deassert the CS line if hw control disabled */
	if (!hw_cs_ctrl) {
		spi_context_cs_control(&data->ctx, false);
	}

	spi_context_release(ctx, ret);

	return ret;
}

static int api_release(const struct device *dev, const struct spi_config *config)
{
	struct max32_spi_data *data = dev->data;

	if (!spi_context_configured(&data->ctx, config)) {
		return -EINVAL;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

/* API implementation: init */
static int spi_max32_init(const struct device *dev)
{
	int ret = 0;
	const struct max32_spi_config *const cfg = dev->config;
	mxc_spi_regs_t *regs = cfg->regs;
	struct max32_spi_data *data = dev->data;

	if (!device_is_ready(cfg->clock)) {
		return -ENODEV;
	}

	MXC_SPI_Shutdown(regs);

	/* enable clock */
	ret = clock_control_on(cfg->clock, (clock_control_subsys_t)&cfg->perclk);
	if (ret) {
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pctrl, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return ret;
}

/* SPI driver APIs structure */
static struct spi_driver_api spi_max32_api = {
	.transceive = api_transceive,
	.release = api_release,
};

/* SPI driver registration */
#define DEFINE_SPI_MAX32(_num)                                                                     \
	PINCTRL_DT_INST_DEFINE(_num);                                                              \
	static const struct max32_spi_config max32_spi_config_##_num = {                           \
		.regs = (mxc_spi_regs_t *)DT_INST_REG_ADDR(_num),                                  \
		.pctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(_num),                                     \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(_num)),                                 \
		.perclk.bus = DT_INST_CLOCKS_CELL(_num, offset),                                   \
		.perclk.bit = DT_INST_CLOCKS_CELL(_num, bit),                                      \
	};                                                                                         \
	static struct max32_spi_data max32_spi_data_##_num = {                                     \
		SPI_CONTEXT_INIT_LOCK(max32_spi_data_##_num, ctx),                                 \
		SPI_CONTEXT_INIT_SYNC(max32_spi_data_##_num, ctx),                                 \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
	DEVICE_DT_INST_DEFINE(_num, spi_max32_init, NULL, &max32_spi_data_##_num,                  \
			      &max32_spi_config_##_num, PRE_KERNEL_2, CONFIG_SPI_INIT_PRIORITY,    \
			      &spi_max32_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_SPI_MAX32)
