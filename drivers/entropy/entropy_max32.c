/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "zephyr/sys/util.h"

#define DT_DRV_COMPAT adi_max32_trng

#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>

#include <wrap_max32_trng.h>

struct max32_trng_config {
	mxc_trng_regs_t *regs;
	const struct device *clock;
	struct max32_perclk perclk;
};

static int api_get_entropy(const struct device *dev, uint8_t *buf, uint16_t len)
{
	return MXC_TRNG_Random(buf, len);
}

static int api_get_entropy_isr(const struct device *dev, uint8_t *buf, uint16_t len, uint32_t flags)
{
	int ret = 0;

	if ((flags & ENTROPY_BUSYWAIT) == 0) {
		uint32_t temp;
		int copy_len;
		int count = 0;

		while (len) {
			ret = Wrap_MXC_TRNG_RandomInt_NonBlocking(&temp);
			if (ret != 0) {
				break; /* Data not ready do not wait */
			}

			copy_len = (len > 4) ? 4 : len;
			memcpy(buf, (uint8_t *)&temp, copy_len);

			len -= copy_len;
			buf += copy_len;
			count += copy_len;
		}
		ret = count;
	} else {
		/* Allowed to busy-wait */
		ret = api_get_entropy(dev, buf, len);
		if (ret == 0) {
			ret = len; /* Data retrieved successfully. */
		}
	}

	return ret;
}

static const struct entropy_driver_api entropy_max32_api = {.get_entropy = api_get_entropy,
							    .get_entropy_isr = api_get_entropy_isr};

static int entropy_max32_init(const struct device *dev)
{
	int ret = 0;
	const struct max32_trng_config *cfg = dev->config;

	/* Enable clock */
	ret = clock_control_on(cfg->clock, (clock_control_subsys_t)&cfg->perclk);
	if (ret) {
		return ret;
	}

	return ret;
}

static const struct max32_trng_config max32_trng_cfg = {
	.regs = (mxc_trng_regs_t *)DT_INST_REG_ADDR(0),
	.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.perclk.bus = DT_INST_CLOCKS_CELL(0, offset),
	.perclk.bit = DT_INST_CLOCKS_CELL(0, bit),
};

DEVICE_DT_INST_DEFINE(0, entropy_max32_init, NULL, NULL, &max32_trng_cfg, PRE_KERNEL_1,
		      CONFIG_ENTROPY_INIT_PRIORITY, &entropy_max32_api);
