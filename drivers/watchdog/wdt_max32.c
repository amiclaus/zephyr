/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max32_watchdog

#include <zephyr/drivers/watchdog.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>
#include <soc.h>
#include <errno.h>

#define LOG_LEVEL CONFIG_WDT_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_max32);

#include <wrap_max32_wdt.h>

#define WDT_CFG(dev)  ((struct max32_wdt_config *)((dev)->config))
#define WDT_DATA(dev) ((struct max32_wdt_data *)((dev)->data))

struct max32_wdt_config {
	mxc_wdt_regs_t *regs;
	int clock_source;
	const struct device *clock;
	struct max32_perclk perclk;
};

struct max32_wdt_data {
	uint32_t timeout;
	wdt_callback_t callback;
};

static int wdt_max32_calculate_timeout(uint32_t timeout)
{
	int i;

	uint32_t number_of_tick = ((float)timeout * (float)PeripheralClock) / 1000;

	/* Find top bit index */
	for (i = 31; i >= 16; i--) {
		if (number_of_tick & (1 << i)) {
			if (number_of_tick & ~(1 << i)) {
				i += 1; /* round up if is there any more tick */
			}
			break;
		}
	}

	if (i > 31) {
		i = 31; /* max */
	} else if (i < 16) {
		i = 16; /* min */
	}

	return i;
}

static int api_disable(const struct device *dev)
{
	MXC_WDT_Disable(WDT_CFG(dev)->regs);
	return 0;
}

static int api_feed(const struct device *dev, int channel_id)
{
	ARG_UNUSED(channel_id);

	MXC_WDT_ResetTimer(WDT_CFG(dev)->regs);
	return 0;
}

static int api_setup(const struct device *dev, uint8_t options)
{
	MXC_WDT_Enable(WDT_CFG(dev)->regs);
	api_feed(dev, 0);
	return 0;
}

static int api_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	struct max32_wdt_data *data = dev->data;
	wrap_mxc_wdt_cfg_t wdt_cfg;

	if ((cfg->window.min != 0U) || (cfg->window.max == 0U)) {
		return -EINVAL;
	}

	data->timeout = cfg->window.max;
	data->callback = cfg->callback;

	int period = 31 - wdt_max32_calculate_timeout(data->timeout);

	wdt_cfg.upperResetPeriod = period;

	Wrap_MXC_WDT_SetResetPeriod(WDT_CFG(dev)->regs, &wdt_cfg);

	return 0;
}

static int wdt_max32_init(const struct device *dev)
{
	int ret = 0;
	wrap_mxc_wdt_cfg_t wdt_cfg;

	/* enable clock */
	ret = clock_control_on(WDT_CFG(dev)->clock,
			       (clock_control_subsys_t) &(WDT_CFG(dev)->perclk));
	if (ret) {
		return ret;
	}

	wdt_cfg.mode = 0; /* Todo set mode during usage*/
	wdt_cfg.upperResetPeriod = 0; /* Not used during initialization */
	wdt_cfg.lowerResetPeriod = 0; /* Not used during initialization */
	wdt_cfg.upperIntPeriod = 0;   /* Not used during initialization */
	wdt_cfg.lowerIntPeriod = 0;   /* Not used during initialization */
	Wrap_MXC_WDT_Init(WDT_CFG(dev)->regs, &wdt_cfg);

	MXC_WDT_EnableReset(WDT_CFG(dev)->regs);

	return 0;
}

static const struct wdt_driver_api max32_wdt_api = {.setup = api_setup,
						    .disable = api_disable,
						    .install_timeout = api_install_timeout,
						    .feed = api_feed};

#define MAX32_WDT_INIT(_num)                                                                       \
	static struct max32_wdt_data max32_wdt_data##_num;                                         \
	static struct max32_wdt_config max32_wdt_config##_num = {                                  \
		.regs = (mxc_wdt_regs_t *)DT_INST_REG_ADDR(_num),                                  \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(_num)),                                 \
		.clock_source = DT_INST_PROP(_num, clock_source),                                  \
		.perclk.bus = DT_INST_CLOCKS_CELL(_num, offset),                                   \
		.perclk.bit = DT_INST_CLOCKS_CELL(_num, bit),                                      \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(_num, wdt_max32_init, NULL, &max32_wdt_data##_num,                   \
			      &max32_wdt_config##_num, POST_KERNEL,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &max32_wdt_api)

#if DT_NODE_HAS_STATUS(DT_NODELABEL(wdt0), okay)
MAX32_WDT_INIT(0);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(wdt1), okay)
MAX32_WDT_INIT(1);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(wdt2), okay)
MAX32_WDT_INIT(2);
#endif
