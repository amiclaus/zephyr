/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(adgm3121);

#define ADGM3121_REG_SWITCH_DATA 0x20
#define ADGM3121_SPI_READ        BIT(7)
#define ADGM3121_SPI_ADDR_MSK    GENMASK(6, 0)

struct adgm3121_emul_data {
	uint8_t switch_reg;
};

static int adgm3121_emul_io(const struct emul *target,
			    const struct spi_config *config,
			    const struct spi_buf_set *tx_bufs,
			    const struct spi_buf_set *rx_bufs)
{
	struct adgm3121_emul_data *data = target->data;
	const uint8_t *tx;
	uint8_t cmd_hi;
	uint8_t addr;
	bool is_read;

	ARG_UNUSED(config);
	__ASSERT_NO_MSG(tx_bufs != NULL);
	__ASSERT_NO_MSG(tx_bufs->buffers != NULL);
	__ASSERT_NO_MSG(tx_bufs->buffers[0].len >= 2);

	tx = tx_bufs->buffers[0].buf;
	cmd_hi = tx[0];
	is_read = !!(cmd_hi & ADGM3121_SPI_READ);
	addr = cmd_hi & ADGM3121_SPI_ADDR_MSK;

	if (is_read) {
		if (rx_bufs == NULL || rx_bufs->buffers == NULL ||
		    rx_bufs->buffers[0].len < 2) {
			return -EINVAL;
		}

		uint8_t *rx = rx_bufs->buffers[0].buf;

		if (addr == ADGM3121_REG_SWITCH_DATA) {
			rx[1] = data->switch_reg;
		} else {
			rx[1] = 0;
		}
	} else {
		uint8_t write_data = tx[1];

		if (addr == ADGM3121_REG_SWITCH_DATA) {
			data->switch_reg = write_data;
		}
	}

	return 0;
}

static int adgm3121_emul_init(const struct emul *target,
			      const struct device *parent)
{
	struct adgm3121_emul_data *data = target->data;

	data->switch_reg = 0;

	return 0;
}

static const struct spi_emul_api adgm3121_emul_spi_api = {
	.io = adgm3121_emul_io,
};

#define ADGM3121_EMUL_DEFINE(n)                                           \
	static struct adgm3121_emul_data adgm3121_emul_data_##n;          \
	EMUL_DT_INST_DEFINE(n, adgm3121_emul_init, &adgm3121_emul_data_##n, \
			    NULL, &adgm3121_emul_spi_api, NULL)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_adgm3121
DT_INST_FOREACH_STATUS_OKAY(ADGM3121_EMUL_DEFINE)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_adgm3053
DT_INST_FOREACH_STATUS_OKAY(ADGM3121_EMUL_DEFINE)
