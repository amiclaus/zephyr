/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/analog_switch.h>

LOG_MODULE_REGISTER(adg1712, CONFIG_ANALOG_SWITCH_LOG_LEVEL);

#define ADG1712_NUM_CHANNELS 4

struct adg1712_config {
	struct gpio_dt_spec gpios[ADG1712_NUM_CHANNELS];
};

struct adg1712_data {
	struct k_mutex lock;
};

static int adg1712_set(const struct device *dev, uint8_t channel,
		       uint8_t state)
{
	const struct adg1712_config *cfg = dev->config;
	struct adg1712_data *data = dev->data;
	int ret;

	if (channel >= ADG1712_NUM_CHANNELS || state > 1) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = gpio_pin_set_dt(&cfg->gpios[channel], state);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int adg1712_get(const struct device *dev, uint8_t channel,
		       uint8_t *state)
{
	const struct adg1712_config *cfg = dev->config;
	struct adg1712_data *data = dev->data;
	int ret;

	if (channel >= ADG1712_NUM_CHANNELS || state == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = gpio_pin_get_dt(&cfg->gpios[channel]);
	k_mutex_unlock(&data->lock);

	if (ret < 0) {
		return ret;
	}

	*state = ret ? 1 : 0;

	return 0;
}

static int adg1712_set_all(const struct device *dev, uint32_t mask)
{
	const struct adg1712_config *cfg = dev->config;
	struct adg1712_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	for (int i = 0; i < ADG1712_NUM_CHANNELS; i++) {
		ret = gpio_pin_set_dt(&cfg->gpios[i],
				      (mask & BIT(i)) ? 1 : 0);
		if (ret) {
			k_mutex_unlock(&data->lock);
			return ret;
		}
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

static int adg1712_get_all(const struct device *dev, uint32_t *mask)
{
	const struct adg1712_config *cfg = dev->config;
	struct adg1712_data *data = dev->data;
	int ret;

	if (mask == NULL) {
		return -EINVAL;
	}

	*mask = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	for (int i = 0; i < ADG1712_NUM_CHANNELS; i++) {
		ret = gpio_pin_get_dt(&cfg->gpios[i]);
		if (ret < 0) {
			k_mutex_unlock(&data->lock);
			return ret;
		}
		if (ret) {
			*mask |= BIT(i);
		}
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

static int adg1712_reset(const struct device *dev)
{
	return adg1712_set_all(dev, 0);
}

static DEVICE_API(analog_switch, adg1712_api) = {
	.set = adg1712_set,
	.get = adg1712_get,
	.set_all = adg1712_set_all,
	.get_all = adg1712_get_all,
	.reset = adg1712_reset,
	.num_channels = ADG1712_NUM_CHANNELS,
};

static int adg1712_init(const struct device *dev)
{
	const struct adg1712_config *cfg = dev->config;
	struct adg1712_data *data = dev->data;
	int ret;

	k_mutex_init(&data->lock);

	for (int i = 0; i < ADG1712_NUM_CHANNELS; i++) {
		if (!gpio_is_ready_dt(&cfg->gpios[i])) {
			LOG_ERR("GPIO %d not ready", i);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->gpios[i],
					    GPIO_OUTPUT_INACTIVE);
		if (ret) {
			LOG_ERR("Failed to configure GPIO %d: %d", i, ret);
			return ret;
		}
	}

	return 0;
}

#define ADG1712_DEFINE(inst)						\
	static struct adg1712_data adg1712_data_##inst;			\
	static const struct adg1712_config adg1712_config_##inst = {	\
		.gpios = {						\
			GPIO_DT_SPEC_INST_GET(inst, in1_gpios),		\
			GPIO_DT_SPEC_INST_GET(inst, in2_gpios),		\
			GPIO_DT_SPEC_INST_GET(inst, in3_gpios),		\
			GPIO_DT_SPEC_INST_GET(inst, in4_gpios),		\
		},							\
	};								\
	DEVICE_DT_INST_DEFINE(inst,					\
			      adg1712_init,				\
			      NULL,					\
			      &adg1712_data_##inst,			\
			      &adg1712_config_##inst,			\
			      POST_KERNEL,				\
			      CONFIG_ANALOG_SWITCH_INIT_PRIORITY,	\
			      &adg1712_api);

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_adg1712
DT_INST_FOREACH_STATUS_OKAY(ADG1712_DEFINE)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_adg2712
DT_INST_FOREACH_STATUS_OKAY(ADG1712_DEFINE)
