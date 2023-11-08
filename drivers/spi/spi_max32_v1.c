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
#ifdef CONFIG_SPI_MAX32_INTERRUPT
	void (*irq_config_func)(const struct device *dev);
#endif /* CONFIG_SPI_MAX32_INTERRUPT */
};

/* Device run time data */
struct max32_spi_data {
	struct spi_context ctx;
	mxc_spi_req_t req;
	uint8_t dummy[2];
};

#ifdef CONFIG_SPI_MAX32_INTERRUPT
static void spi_max32_callback(mxc_spi_req_t *req, int error);
#endif /* CONFIG_SPI_MAX32_INTERRUPT */

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
	data->req.txLen = len >> dfs_shift;
	data->req.txData = (uint8_t *)ctx->tx_buf;
	data->req.rxLen = len >> dfs_shift;
	data->req.rxData = ctx->rx_buf;

	data->req.rxData = ctx->rx_buf;
	data->req.rxLen = len >> dfs_shift;
	if (!data->req.rxData) {
		/* Pass a dummy buffer to HAL if receive buffer is NULL, otherwise
		 * corrupt data is read during subsequent transactions.
		 */
		data->req.rxData = data->dummy;
		data->req.rxLen = 0;
	}

	data->req.spi = cfg->regs;
	data->req.ssIdx = ctx->config->slave;
	data->req.ssDeassert = 1;
	data->req.txCnt = 0;
	data->req.rxCnt = 0;

#ifdef CONFIG_SPI_MAX32_INTERRUPT
	data->req.completeCB = (spi_complete_cb_t)spi_max32_callback;

	ret = MXC_SPI_MasterTransactionAsync(&data->req);
	if (ret) {
		ret = -EIO;
	}
#else
	ret = MXC_SPI_MasterTransaction(&data->req);
	if (ret) {
		ret = -EIO;
	} else {
		spi_context_update_tx(ctx, 1, len);
		spi_context_update_rx(ctx, 1, len);
	}
#endif

	return ret;
}

static int transceive(const struct device *dev, const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs,
		      bool async, spi_callback_t cb, void *userdata)
{
	int ret = 0;
	const struct max32_spi_config *cfg = dev->config;
	struct max32_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	bool hw_cs_ctrl = true;

#ifndef CONFIG_SPI_MAX32_INTERRUPT
	if (async) {
		return -ENOTSUP;
	}
#endif

	spi_context_lock(ctx, async, cb, userdata, config);

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
		spi_context_cs_control(ctx, true);
	}

#ifdef CONFIG_SPI_MAX32_INTERRUPT
	do {
		ret = spi_max32_transceive(dev);
		if (!ret) {
			spi_context_wait_for_completion(ctx);
		} else {
			break;
		}
	} while ((spi_context_tx_on(ctx) || spi_context_rx_on(ctx)));
#else
	do {
		ret = spi_max32_transceive(dev);
		if (ret != 0) {
			break;
		}
	} while (spi_context_tx_on(ctx) || spi_context_rx_on(ctx));

	/* Deassert the CS line if hw control disabled */
	if (!hw_cs_ctrl) {
		spi_context_cs_control(ctx, false);
	}
#endif /* CONFIG_SPI_MAX32_INTERRUPT */

	spi_context_release(ctx, ret);

	return ret;
}

static int api_transceive(const struct device *dev, const struct spi_config *config,
			  const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int api_transceive_async(const struct device *dev, const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				void *userdata)
{
	return transceive(dev, config, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

#ifdef CONFIG_SPI_MAX32_INTERRUPT
static void spi_max32_callback(mxc_spi_req_t *req, int error)
{
	struct max32_spi_data *data = CONTAINER_OF(req, struct max32_spi_data, req);
	struct spi_context *ctx = &data->ctx;
	const struct device *dev = CONTAINER_OF((void *)data, struct device, data);
	uint32_t len;

	len = spi_context_max_continuous_chunk(ctx);
	spi_context_update_tx(ctx, 1, len);
	spi_context_update_rx(ctx, 1, len);
	spi_context_complete(ctx, dev, error == E_NO_ERROR ? 0 : -EIO);
}

static void spi_max32_isr(const struct device *dev)
{
	const struct max32_spi_config *cfg = dev->config;

	MXC_SPI_AsyncHandler(cfg->regs);
}
#endif /* CONFIG_SPI_MAX32_INTERRUPT */

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

#ifdef CONFIG_SPI_MAX32_INTERRUPT
	cfg->irq_config_func(dev);
#endif

	spi_context_unlock_unconditionally(&data->ctx);

	return ret;
}

/* SPI driver APIs structure */
static struct spi_driver_api spi_max32_api = {
	.transceive = api_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = api_transceive_async,
#endif /* CONFIG_SPI_ASYNC */
	.release = api_release,
};

/* SPI driver registration */
#ifdef CONFIG_SPI_MAX32_INTERRUPT
#define SPI_MAX32_CONFIG_IRQ_FUNC(n) .irq_config_func = spi_max32_irq_config_func_##n,

#define SPI_MAX32_IRQ_CONFIG_FUNC(n)                                                               \
	static void spi_max32_irq_config_func_##n(const struct device *dev)                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), spi_max32_isr,              \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}
#else
#define SPI_MAX32_CONFIG_IRQ_FUNC(n)
#define SPI_MAX32_IRQ_CONFIG_FUNC(n)
#endif /* CONFIG_SPI_MAX32_INTERRUPT */

#define DEFINE_SPI_MAX32(_num)                                                                     \
	PINCTRL_DT_INST_DEFINE(_num);                                                              \
	SPI_MAX32_IRQ_CONFIG_FUNC(_num)                                                            \
	static const struct max32_spi_config max32_spi_config_##_num = {                           \
		.regs = (mxc_spi_regs_t *)DT_INST_REG_ADDR(_num),                                  \
		.pctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(_num),                                     \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(_num)),                                 \
		.perclk.bus = DT_INST_CLOCKS_CELL(_num, offset),                                   \
		.perclk.bit = DT_INST_CLOCKS_CELL(_num, bit),                                      \
		SPI_MAX32_CONFIG_IRQ_FUNC(_num)};                                                  \
	static struct max32_spi_data max32_spi_data_##_num = {                                     \
		SPI_CONTEXT_INIT_LOCK(max32_spi_data_##_num, ctx),                                 \
		SPI_CONTEXT_INIT_SYNC(max32_spi_data_##_num, ctx),                                 \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
	DEVICE_DT_INST_DEFINE(_num, spi_max32_init, NULL, &max32_spi_data_##_num,                  \
			      &max32_spi_config_##_num, PRE_KERNEL_2, CONFIG_SPI_INIT_PRIORITY,    \
			      &spi_max32_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_SPI_MAX32)
