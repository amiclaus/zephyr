/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max32_adc

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>
#ifdef CONFIG_ADC_MAX32_DMA
#include <zephyr/drivers/dma.h>
#endif /* CONFIG_ADC_MAX32_DMA */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_max32, CONFIG_ADC_LOG_LEVEL);

#include <wrap_max32_adc.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

/* reference voltage for the ADC */
#define MAX32_ADC_VREF_MV DT_INST_PROP(0, vref_mv)

#ifdef CONFIG_ADC_MAX32_DMA
struct max32_adc_dma_config {
	const struct device *dev;
	const uint32_t channel;
	const uint32_t slot;
};
#endif /* CONFIG_ADC_MAX32_DMA */

struct max32_adc_config {
	uint8_t channel_count;
	mxc_adc_regs_t *regs;
	const struct pinctrl_dev_config *pctrl;
	const struct device *clock;
	struct max32_perclk perclk;
	void (*irq_func)(void);
#ifdef CONFIG_ADC_MAX32_DMA
	const struct max32_adc_dma_config dma;
#endif /* CONFIG_ADC_MAX32_DMA */
};

struct max32_adc_data {
	const struct device *dev;
	struct adc_context ctx;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint32_t channels;
	uint32_t sample_channels;
	const uint8_t resolution;
};

#ifdef CONFIG_ADC_ASYNC
void adc_complete_cb(void *req, int error)
{
	(void)req;
	(void)error;
}
#endif /* CONFIG_ADC_ASYNC */

#ifdef CONFIG_ADC_MAX32_DMA
static void adc_max32_dma_callback(const struct device *dma_dev, void *user_data, uint32_t channel,
				   int status)
{
	const struct device *dev = user_data;
	struct max32_adc_data *data = dev->data;
	const struct max32_adc_config *config = dev->config;

	if (channel == config->dma.channel) {
		data->sample_channels &= ~BIT(find_lsb_set(data->sample_channels) - 1);
		data->buffer++;

		if (data->sample_channels != 0) {
			dma_reload(config->dma.dev, config->dma.channel, (uint32_t)&(config->regs->data), (uint32_t)data->buffer, sizeof(int16_t));

			/* Select a new channel and wait completing ADC reading operation */
			Wrap_MXC_ADC_ChannelSelect(config->regs, data->sample_channels);

			/* Waiting ADC done flag because DMA is faster than ADC. */
			MXC_ADC_ClearFlags(MXC_F_ADC_INTR_DONE_IF);
			config->regs->ctrl |= MXC_F_ADC_CTRL_START;
			while(MXC_ADC_GetFlags() != MXC_F_ADC_INTR_DONE_IF) {;}

			dma_start(config->dma.dev, config->dma.channel);
		} else {
			dma_stop(config->dma.dev, config->dma.channel);
			adc_context_on_sampling_done(&data->ctx, dev);
		}
	} else if (status < 0) {
		LOG_ERR("DMA sampling complete, but DMA reported error %d", status);
		dma_stop(config->dma.dev, config->dma.channel);
		adc_context_complete(&data->ctx, status);
	}
}

static int adc_max32_dma_start(const struct device *dev, void *buffer)
{
	const struct max32_adc_config *config = dev->config;
	struct max32_adc_data *data = dev->data;
	struct dma_status status;
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk_cfg;
	int ret;

	if (config->dma.channel == 0xFF) {
		LOG_ERR("DMA channel is not configured");
		return -ENOTSUP;
	}

	ret = dma_get_status(config->dma.dev, config->dma.channel, &status);
	if (ret < 0 || status.busy) {
		LOG_ERR("DMA %s", ret < 0 ? "error" : "busy");
		return ret < 0 ? ret : -EBUSY;
	}

	dma_blk_cfg.source_address = (uint32_t)&(config->regs->data);
	dma_blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	dma_blk_cfg.source_reload_en = 0;

	dma_blk_cfg.dest_address = (uint32_t)buffer;
	dma_blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	dma_blk_cfg.dest_reload_en = 0;
	dma_blk_cfg.block_size = sizeof(int16_t);

	dma_cfg.channel_direction = MEMORY_TO_MEMORY;
	dma_cfg.dma_callback = adc_max32_dma_callback;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_slot = config->dma.slot;
	dma_cfg.source_data_size = 2U;
	dma_cfg.source_burst_length = 2U;
	dma_cfg.dest_data_size = 2U;
	dma_cfg.head_block = &dma_blk_cfg;

    /* Select a new channel for ADC */
	Wrap_MXC_ADC_ChannelSelect(config->regs, data->sample_channels);

	/* Waiting ADC done flag because DMA is faster than ADC. */
	MXC_ADC_ClearFlags(MXC_F_ADC_INTR_DONE_IF);
	config->regs->ctrl |= MXC_F_ADC_CTRL_START;
	while(MXC_ADC_GetFlags() != MXC_F_ADC_INTR_DONE_IF) {;}

	ret = dma_config(config->dma.dev, config->dma.channel, &dma_cfg);
	if (ret < 0) {
		LOG_ERR("Error configuring DMA (%d)", ret);
		return ret;
	}

	ret = dma_start(config->dma.dev, config->dma.channel);
	if (ret < 0) {
		LOG_ERR("Error starting DMA (%d)", ret);
		return ret;
	}

	return ret;
}
#endif /* CONFIG_ADC_MAX32_DMA */

static void adc_max32_start_channel(const struct device *dev)
{
	struct max32_adc_data *data = dev->data;
	int ret = 0;

#if defined(CONFIG_ADC_MAX32_DMA)
	ret = adc_max32_dma_start(dev, data->buffer);
	if (ret < 0) {
		LOG_ERR("Error starting conversion (%d)", ret);
		return;
	}
#else
#if defined(CONFIG_ADC_ASYNC)
	if (data->ctx.asynchronous) {
		ret = Wrap_MXC_ADC_StartConversionAsync(&data->sample_channels, adc_complete_cb);
		if (ret < 0) {
			LOG_ERR("Error starting conversion (%d)", ret);
			return;
		}
	} else {
#endif /* CONFIG_ADC_ASYNC */
		ret = Wrap_MXC_ADC_StartConversion(&data->sample_channels);
		if (ret < 0) {
			LOG_ERR("Error starting conversion (%d)", ret);
			return;
		}
		Wrap_MXC_ADC_GetData(&data->buffer);
		data->sample_channels &= ~BIT(find_lsb_set(data->sample_channels) - 1);

		if (data->sample_channels != 0) {
			adc_max32_start_channel(dev);
		} else {
			Wrap_MXC_ADC_DisableConversion();
			adc_context_on_sampling_done(&data->ctx, dev);
		}
#if defined(CONFIG_ADC_ASYNC)
	}
#endif /* CONFIG_ADC_ASYNC */
#endif /* CONFIG_ADC_MAX32_DMA */
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct max32_adc_data *data = CONTAINER_OF(ctx, struct max32_adc_data, ctx);

	data->sample_channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	adc_max32_start_channel(data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct max32_adc_data *data = CONTAINER_OF(ctx, struct max32_adc_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int start_read(const struct device *dev, const struct adc_sequence *seq)
{
	const struct max32_adc_config *config = dev->config;
	struct max32_adc_data *data = dev->data;
	int ret = 0;

	if (seq->resolution != data->resolution) {
		LOG_ERR("Unsupported resolution (%d)", seq->resolution);
		return -ENOTSUP;
	}
	if (seq->channels == 0) {
		return -EINVAL;
	}
	if ((data->channels & seq->channels) != seq->channels) {
		return -EINVAL;
	}

	Wrap_MXC_ADC_SetCalibration(config->regs, seq->calibrate);
	ret = Wrap_MXC_ADC_AverageConfig(seq->oversampling);
	if (ret != 0) {
		return ret;
	}

	uint32_t num_of_sample_channels = POPCOUNT(seq->channels);
	uint32_t num_of_sample = 1;
	if (seq->options) {
		num_of_sample += seq->options->extra_samplings;
	}
	if (seq->buffer_size < (num_of_sample * num_of_sample_channels)) { /* Buffer size control */
		return -ENOMEM;
	}

	data->buffer = seq->buffer;
	adc_context_start_read(&data->ctx, seq);

	return adc_context_wait_for_completion(&data->ctx);
}

static int api_read(const struct device *dev, const struct adc_sequence *seq)
{
	struct max32_adc_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, false, NULL);
	error = start_read(dev, seq);
	adc_context_release(&data->ctx, error);

	return error;
}

#ifdef CONFIG_ADC_ASYNC
static int api_read_async(const struct device *dev, const struct adc_sequence *seq,
			  struct k_poll_signal *async)
{
	struct max32_adc_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, true, async);
	error = start_read(dev, seq);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif /* CONFIG_ADC_ASYNC */

static int api_channel_setup(const struct device *dev, const struct adc_channel_cfg *cfg)
{
	const struct max32_adc_config *conf = (const struct max32_adc_config *)dev->config;
	struct max32_adc_data *data = dev->data;
	wrap_mxc_adc_scale_t wrap_mxc_scale;
	uint8_t adc_reference;
	int ret = 0;

	if (cfg->channel_id >= conf->channel_count) {
		LOG_ERR("Invalid channel (%u)", cfg->channel_id);
		return -EINVAL;
	}

	if (cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Invalid channel acquisition time");
		return -EINVAL;
	}

	if (cfg->differential) {
		LOG_ERR("Differential sampling not supported");
		return -ENOTSUP;
	}

	switch (cfg->reference) {
	case ADC_REF_INTERNAL:
		adc_reference = ADI_MAX32_ADC_REF_INTERNAL;
		break;
	case ADC_REF_VDD_1_2:
		adc_reference = ADI_MAX32_ADC_REF_VDD_1_2;
		break;
	case ADC_REF_EXTERNAL0:
		adc_reference = ADI_MAX32_ADC_REF_EXT0;
		break;
	default:
		return -ENOTSUP;
	}
	ret = Wrap_MXC_ADC_ReferenceSelect((mxc_adc_regs_t *)MXC_ADC, adc_reference);
	if (ret != 0) {
		LOG_ERR("Reference is not supported.");
		return -ENOTSUP;
	}

	switch (cfg->gain) {
	case ADC_GAIN_1_6:
		wrap_mxc_scale = WRAP_MXC_ADC_SCALE_6;
		break;
	case ADC_GAIN_1_4:
		wrap_mxc_scale = WRAP_MXC_ADC_SCALE_4;
		break;
	case ADC_GAIN_1_3:
		wrap_mxc_scale = WRAP_MXC_ADC_SCALE_3;
		break;
	case ADC_GAIN_1_2:
		wrap_mxc_scale = WRAP_MXC_ADC_SCALE_2;
		break;
	case ADC_GAIN_1:
		wrap_mxc_scale = WRAP_MXC_ADC_SCALE_1;
		break;
	case ADC_GAIN_2:
		wrap_mxc_scale = WRAP_MXC_ADC_SCALE_2X;
		break;
	default:
		return -ENOTSUP;
	}
	ret = Wrap_MXC_ADC_SetExtScale(wrap_mxc_scale);
	if (ret != 0) {
		LOG_ERR("Gain value is not supported.");
		return -ENOTSUP;
	}

	data->channels |= BIT(cfg->channel_id);
	return 0;
}

static int adc_max32_init(const struct device *dev)
{
	const struct max32_adc_config *config = dev->config;
	struct max32_adc_data *data = dev->data;
	wrap_mxc_adc_req_t req = {
		.clock = 0, .clkdiv = 3, .cal = 0, .ref = 1, .trackCount = 4, .idleCount = 5};
	uint32_t ret;

	/* Enable clock */
	ret = clock_control_on(config->clock, (clock_control_subsys_t)&config->perclk);
	if (ret) {
		return ret;
	}

	ret = Wrap_MXC_ADC_Init(&req);
	if (ret) {
		return ret;
	}

	ret = pinctrl_apply_state(config->pctrl, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	config->irq_func();
	data->dev = dev;

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static void adc_max32_isr(const struct device *dev)
{
	struct max32_adc_data *const data = dev->data;
	uint32_t flags = MXC_ADC_GetFlags();
	MXC_ADC_Handler();
	MXC_ADC_ClearFlags(flags);

	if (flags & WRAP_MXC_F_ADC_CONV_DONE_IF) {
		data->sample_channels &= ~BIT(find_lsb_set(data->sample_channels) - 1);
		Wrap_MXC_ADC_GetData(&data->buffer);
			
		if (data->sample_channels != 0) {
			adc_max32_start_channel(dev);
		} else {
			Wrap_MXC_ADC_DisableConversion();
			adc_context_on_sampling_done(&data->ctx, dev);
		}
	}
}

static const struct adc_driver_api adc_max32_driver_api = {
	.channel_setup = api_channel_setup,
	.read = api_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = api_read_async,
#endif /* CONFIG_ADC_ASYNC */
	.ref_internal = MAX32_ADC_VREF_MV,
};

#ifdef CONFIG_ADC_MAX32_DMA
#define MAX32_DT_INST_DMA_CTLR(_num, name)                                                         \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(_num, dmas),                                             \
		    (DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(_num, name))), (NULL))

#define MAX32_DT_INST_DMA_CELL(_num, name, cell)                                                   \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(_num, dmas),                                             \
		    (DT_INST_DMAS_CELL_BY_NAME(_num, name, cell)), (0xff))

#define MAX32_ADC_DMA_INIT(_num)                                                                   \
	.dma = {                                                                                   \
		.dev = MAX32_DT_INST_DMA_CTLR(_num, adc),                                          \
		.channel = MAX32_DT_INST_DMA_CELL(_num, adc, channel),                             \
		.slot = MAX32_DT_INST_DMA_CELL(_num, adc, slot),                                   \
	},
#else
#define MAX32_ADC_DMA_INIT(_num)
#endif /* CONFIG_ADC_MAX32_DMA */

#define MAX32_ADC_INIT(_num)                                                                       \
	PINCTRL_DT_INST_DEFINE(_num);											\
	static void max32_adc_irq_init_##_num(void)                                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(_num), DT_INST_IRQ(_num, priority), adc_max32_isr,        \
			    DEVICE_DT_INST_GET(_num), 0);                                          \
		irq_enable(DT_INST_IRQN(_num));                                                    \
	};                                                                                         \
	static const struct max32_adc_config max32_adc_config_##_num = {                           \
		.channel_count = DT_INST_PROP(_num, channel_count),                        \
		.regs = (mxc_adc_regs_t *)DT_INST_REG_ADDR(_num),                                  \
		.pctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(_num),									\
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(_num)),                                 \
		.perclk.bus = DT_INST_CLOCKS_CELL(_num, offset),                                   \
		.perclk.bit = DT_INST_CLOCKS_CELL(_num, bit),                                      \
		.irq_func = max32_adc_irq_init_##_num,                                             \
		MAX32_ADC_DMA_INIT(_num)                                                           \
	};                                                                                         \
	static struct max32_adc_data max32_adc_data_##_num = {                                     \
		ADC_CONTEXT_INIT_TIMER(max32_adc_data_##_num, ctx),                                \
		ADC_CONTEXT_INIT_LOCK(max32_adc_data_##_num, ctx),                                 \
		ADC_CONTEXT_INIT_SYNC(max32_adc_data_##_num, ctx),                                 \
		.resolution = DT_INST_PROP(_num, resolution),									\
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(_num, &adc_max32_init, NULL, &max32_adc_data_##_num,                 \
			      &max32_adc_config_##_num, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,     \
			      &adc_max32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX32_ADC_INIT)
