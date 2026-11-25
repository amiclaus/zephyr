/*
 * SPDX-FileCopyrightText: 2026 NXP
 * SPDX-FileCopyrightText: 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADI ADGM3121 DPDT MEMS switch (DC-24GHz) and ADGM3053 4-channel MEMS
 * crossover switch (DC-14GHz) as a MUX controller.
 *
 * Both devices expose four independent switches (SW1..SW4) behind the same
 * SPI register and parallel GPIO interfaces. This backend models the whole
 * device as a single control line (#mux-control-cells = 0) whose state is a
 * 4-bit switch bitmask: bit i selects SW(i+1), 1 = closed.
 *
 *   mux_control_set(dev, ctrl, mask)  -> drive all four switches at once
 *   mux_state_get(dev, ctrl, &mask)   -> read the switch bitmask back
 *   mux_control_disconnect(dev, ctrl) -> open all switches (mask = 0)
 *
 * On the ADGM3053, SW(i+1) routes source S(i+1) to drain D(i+1) when closed
 * and to the internal crossover path when open, so every source with an open
 * switch is connected to the others. No mask leaves all sources unrouted, so
 * mux_control_disconnect() returns -ENOSYS for this device.
 *
 * Two control interfaces are supported, selected by compatible:
 *   adi,adgm3121, adi,adgm3053           - SPI, single 16-bit register at
 *                                          0x20 (bits 3:0)
 *   adi,adgm3121-gpio, adi,adgm3053-gpio - parallel GPIO, one line
 *                                          (in1..in4) per switch
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mux.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(mux_adi_adgm3121, CONFIG_MUX_LOG_LEVEL);

/* Number of independent switches (SW1..SW4). */
#define ADGM3121_NUM_SWITCHES 4

/* Register address (SPI mode). */
#define ADGM3121_REG_SWITCH_DATA 0x20

/* Register bit fields. */
#define ADGM3121_SWITCH_MSK BIT_MASK(ADGM3121_NUM_SWITCHES)

/* SPI command format (16-bit frames). */
#define ADGM3121_SPI_READ     BIT(15)
#define ADGM3121_SPI_ADDR_MSK GENMASK(14, 8)

/* Timing. */
#define ADGM3121_SWITCHING_TIME_US 200
#define ADGM3121_POWER_UP_TIME_MS  45
#define ADGM3053_POWER_UP_TIME_MS  5

/* Control interfaces present in the devicetree. */
#define ADGM3121_SPI_ENABLED                                                                       \
	(DT_HAS_COMPAT_STATUS_OKAY(adi_adgm3121) || DT_HAS_COMPAT_STATUS_OKAY(adi_adgm3053))
#define ADGM3121_GPIO_ENABLED                                                                      \
	(DT_HAS_COMPAT_STATUS_OKAY(adi_adgm3121_gpio) ||                                           \
	 DT_HAS_COMPAT_STATUS_OKAY(adi_adgm3053_gpio))

enum adgm3121_mode {
	ADGM3121_MODE_SPI,
	ADGM3121_MODE_GPIO,
};

struct mux_adgm3121_config {
	enum adgm3121_mode mode;
	/* Power-up time to wait before the first switch command. */
	uint8_t power_up_time_ms;
	/* Opening all switches leaves every port unrouted. */
	bool has_open_state;
	union {
		struct spi_dt_spec spi;
		struct gpio_dt_spec gpios[ADGM3121_NUM_SWITCHES];
	};
};

struct mux_adgm3121_data {
	struct k_mutex lock;
	uint8_t switch_states;
};

#if ADGM3121_SPI_ENABLED
static int adgm3121_spi_write_reg(const struct device *dev, uint8_t reg_addr,
				  uint8_t data)
{
	const struct mux_adgm3121_config *cfg = dev->config;
	uint16_t cmd = FIELD_PREP(ADGM3121_SPI_ADDR_MSK, reg_addr) | data;
	uint8_t buf[2] = { cmd >> 8, cmd & 0xFF };
	const struct spi_buf tx_buf = { .buf = buf, .len = sizeof(buf) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	return spi_write_dt(&cfg->spi, &tx);
}

static int adgm3121_spi_read_reg(const struct device *dev, uint8_t reg_addr,
				 uint8_t *data)
{
	const struct mux_adgm3121_config *cfg = dev->config;
	uint16_t cmd = ADGM3121_SPI_READ | FIELD_PREP(ADGM3121_SPI_ADDR_MSK, reg_addr);
	uint8_t tx_data[2] = { cmd >> 8, cmd & 0xFF };
	uint8_t rx_data[2];
	const struct spi_buf tx_buf = { .buf = tx_data, .len = sizeof(tx_data) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf rx_buf = { .buf = rx_data, .len = sizeof(rx_data) };
	const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };
	int ret = spi_transceive_dt(&cfg->spi, &tx, &rx);

	if (ret < 0) {
		return ret;
	}

	*data = rx_data[1];

	return 0;
}
#endif /* ADGM3121_SPI_ENABLED */

static int adgm3121_write_mask(const struct device *dev, uint8_t mask)
{
	const struct mux_adgm3121_config *cfg = dev->config;

	switch (cfg->mode) {
#if ADGM3121_SPI_ENABLED
	case ADGM3121_MODE_SPI:
		return adgm3121_spi_write_reg(dev, ADGM3121_REG_SWITCH_DATA, mask);
#endif
#if ADGM3121_GPIO_ENABLED
	case ADGM3121_MODE_GPIO:
		for (uint8_t i = 0; i < ADGM3121_NUM_SWITCHES; i++) {
			int ret = gpio_pin_set_dt(&cfg->gpios[i], (mask >> i) & 0x1U);

			if (ret < 0) {
				return ret;
			}
		}
		return 0;
#endif
	default:
		return -ENOTSUP;
	}
}

static int mux_adgm3121_set(const struct device *dev,
			    const struct mux_control *control, uint32_t state)
{
	struct mux_adgm3121_data *data = dev->data;
	int ret;

	ARG_UNUSED(control);

	if (state > ADGM3121_SWITCH_MSK) {
		LOG_ERR("state 0x%x exceeds 4-bit switch mask", state);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = adgm3121_write_mask(dev, (uint8_t)state);
	if (ret == 0) {
		data->switch_states = (uint8_t)state;
		k_usleep(ADGM3121_SWITCHING_TIME_US);
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static int mux_adgm3121_get_state(const struct device *dev,
				  const struct mux_control *control, uint32_t *state)
{
	struct mux_adgm3121_data *data = dev->data;

	ARG_UNUSED(control);

	k_mutex_lock(&data->lock, K_FOREVER);

#if ADGM3121_SPI_ENABLED
	const struct mux_adgm3121_config *cfg = dev->config;

	if (cfg->mode == ADGM3121_MODE_SPI) {
		uint8_t reg_data;
		int ret = adgm3121_spi_read_reg(dev, ADGM3121_REG_SWITCH_DATA, &reg_data);

		if (ret < 0) {
			k_mutex_unlock(&data->lock);
			return ret;
		}

		data->switch_states = reg_data & ADGM3121_SWITCH_MSK;
	}
#endif
	/* GPIO mode: reading back output lines is not reliable across GPIO
	 * controllers, so return the cached switch bitmask (as gpio-mux does).
	 */
	*state = data->switch_states;

	k_mutex_unlock(&data->lock);

	return 0;
}

static int mux_adgm3121_disconnect(const struct device *dev,
				   const struct mux_control *control)
{
	const struct mux_adgm3121_config *cfg = dev->config;
	struct mux_adgm3121_data *data = dev->data;
	int ret;

	ARG_UNUSED(control);

	if (!cfg->has_open_state) {
		return -ENOSYS;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Open every switch: the all-open state of the switch array. */
	ret = adgm3121_write_mask(dev, 0);
	if (ret == 0) {
		data->switch_states = 0;
		k_usleep(ADGM3121_SWITCHING_TIME_US);
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static DEVICE_API(mux_control, mux_adgm3121_driver_api) = {
	.set = mux_adgm3121_set,
	.get_state = mux_adgm3121_get_state,
	.disconnect = mux_adgm3121_disconnect,
};

static int mux_adgm3121_init(const struct device *dev)
{
	const struct mux_adgm3121_config *cfg = dev->config;
	struct mux_adgm3121_data *data = dev->data;

	k_mutex_init(&data->lock);

	switch (cfg->mode) {
#if ADGM3121_SPI_ENABLED
	case ADGM3121_MODE_SPI:
		if (!spi_is_ready_dt(&cfg->spi)) {
			LOG_ERR("SPI bus not ready");
			return -ENODEV;
		}
		break;
#endif
#if ADGM3121_GPIO_ENABLED
	case ADGM3121_MODE_GPIO:
		for (uint8_t i = 0; i < ADGM3121_NUM_SWITCHES; i++) {
			const struct gpio_dt_spec *gpio = &cfg->gpios[i];
			int ret;

			if (!gpio_is_ready_dt(gpio)) {
				LOG_ERR("gpio[%u] %s not ready", i, gpio->port->name);
				return -ENODEV;
			}

			ret = gpio_pin_configure_dt(gpio, GPIO_OUTPUT_INACTIVE);
			if (ret < 0) {
				LOG_ERR("gpio_pin_configure_dt(%u) failed: %d", i, ret);
				return ret;
			}
		}
		break;
#endif
	default:
		return -ENOTSUP;
	}

	/* Wait for device power-up, then reset all switches open. */
	k_msleep(cfg->power_up_time_ms);

	return adgm3121_write_mask(dev, 0);
}

#define ADGM3121_SPI_DEFINE(inst, part, power_up_ms, open_state)                                   \
	static struct mux_adgm3121_data mux_##part##_spi_data_##inst;                              \
	static const struct mux_adgm3121_config mux_##part##_spi_cfg_##inst = {                    \
		.mode = ADGM3121_MODE_SPI,                                                         \
		.power_up_time_ms = power_up_ms,                                                   \
		.has_open_state = open_state,                                                      \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_CONTROLLER | SPI_TRANSFER_MSB |      \
							  SPI_WORD_SET(8)),                        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, mux_adgm3121_init, NULL, &mux_##part##_spi_data_##inst,        \
			      &mux_##part##_spi_cfg_##inst, POST_KERNEL,                           \
			      CONFIG_MUX_ADI_ADGM3121_INIT_PRIORITY, &mux_adgm3121_driver_api);

#define ADGM3121_GPIO_DEFINE(inst, part, power_up_ms, open_state)                                  \
	static struct mux_adgm3121_data mux_##part##_gpio_data_##inst;                             \
	static const struct mux_adgm3121_config mux_##part##_gpio_cfg_##inst = {                   \
		.mode = ADGM3121_MODE_GPIO,                                                        \
		.power_up_time_ms = power_up_ms,                                                   \
		.has_open_state = open_state,                                                      \
		.gpios = {                                                                         \
			GPIO_DT_SPEC_INST_GET(inst, in1_gpios),                                    \
			GPIO_DT_SPEC_INST_GET(inst, in2_gpios),                                    \
			GPIO_DT_SPEC_INST_GET(inst, in3_gpios),                                    \
			GPIO_DT_SPEC_INST_GET(inst, in4_gpios),                                    \
		},                                                                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, mux_adgm3121_init, NULL, &mux_##part##_gpio_data_##inst,       \
			      &mux_##part##_gpio_cfg_##inst, POST_KERNEL,                          \
			      CONFIG_MUX_ADI_ADGM3121_INIT_PRIORITY, &mux_adgm3121_driver_api);

#define DT_DRV_COMPAT adi_adgm3121
DT_INST_FOREACH_STATUS_OKAY_VARGS(ADGM3121_SPI_DEFINE, adgm3121, ADGM3121_POWER_UP_TIME_MS, true)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT adi_adgm3121_gpio
DT_INST_FOREACH_STATUS_OKAY_VARGS(ADGM3121_GPIO_DEFINE, adgm3121, ADGM3121_POWER_UP_TIME_MS, true)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT adi_adgm3053
DT_INST_FOREACH_STATUS_OKAY_VARGS(ADGM3121_SPI_DEFINE, adgm3053, ADGM3053_POWER_UP_TIME_MS, false)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT adi_adgm3053_gpio
DT_INST_FOREACH_STATUS_OKAY_VARGS(ADGM3121_GPIO_DEFINE, adgm3053, ADGM3053_POWER_UP_TIME_MS, false)
#undef DT_DRV_COMPAT
