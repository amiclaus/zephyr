/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>

#include <wrap_max32_sys.h>

#define DT_DRV_COMPAT adi_max32_gcr

/** Get prescaler value if it defined  */
#define ADI_MAX32_SYSCLK_PRESCALER DT_INST_PROP_OR(0, sysclk_prescaler, 0)

struct max32_clkctrl_config {
	mxc_gcr_regs_t *regs;
	const struct device *clock;
	int prescaler;
};

static inline int api_on(const struct device *dev, clock_control_subsys_t clkcfg)
{
	struct max32_perclk *perclk = (struct max32_perclk *)(clkcfg);

	ARG_UNUSED(dev);

	switch (perclk->bus) {
	case ADI_MAX32_CLOCK_BUS0:
		MXC_SYS_ClockEnable((mxc_sys_periph_clock_t)perclk->bit);
		break;
	case ADI_MAX32_CLOCK_BUS1:
		MXC_SYS_ClockEnable((mxc_sys_periph_clock_t)(perclk->bit + 32));
		break;
	case ADI_MAX32_CLOCK_BUS2:
		MXC_SYS_ClockEnable((mxc_sys_periph_clock_t)(perclk->bit + 64));
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static inline int api_off(const struct device *dev, clock_control_subsys_t clkcfg)
{
	struct max32_perclk *perclk = (struct max32_perclk *)(clkcfg);

	ARG_UNUSED(dev);

	switch (perclk->bus) {
	case ADI_MAX32_CLOCK_BUS0:
		MXC_SYS_ClockDisable((mxc_sys_periph_clock_t)perclk->bit);
		break;
	case ADI_MAX32_CLOCK_BUS1:
		MXC_SYS_ClockDisable((mxc_sys_periph_clock_t)(perclk->bit + 32));
		break;
	case ADI_MAX32_CLOCK_BUS2:
		MXC_SYS_ClockDisable((mxc_sys_periph_clock_t)(perclk->bit + 64));
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct clock_control_driver_api max32_clkctrl_api = {
	.on = api_on,
	.off = api_off,
};

static void setup_fixed_clocks(void)
{
	if (IS_ENABLED(ADI_MAX32_CLK_IPO_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_IPO);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_IPO);
	}

	if (IS_ENABLED(ADI_MAX32_CLK_ERFO_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_ERFO);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_ERFO);
	}

	if (IS_ENABLED(ADI_MAX32_CLK_IBRO_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_IBRO);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_IBRO);
	}

	if (IS_ENABLED(ADI_MAX32_CLK_ISO_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_ISO);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_ISO);
	}

	if (IS_ENABLED(ADI_MAX32_CLK_INRO_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_INRO);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_INRO);
	}

	if (IS_ENABLED(ADI_MAX32_CLK_ERTCO_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_ERTCO);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_ERTCO);
	}

#if DT_NODE_HAS_COMPAT(DT_NODELABEL(clk_extclk), fixed_clock)
	if (IS_ENABLED(ADI_MAX32_CLK_EXTCLK_ENABLED)) {
		MXC_SYS_ClockSourceEnable(ADI_MAX32_CLK_EXTCLK);
	} else {
		MXC_SYS_ClockSourceDisable(ADI_MAX32_CLK_EXTCLK);
	}
#endif
}

static int max32_clkctrl_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* Setup fixed clocks if enabled */
	setup_fixed_clocks();

	/* Setup device clock source */
	MXC_SYS_Clock_Select(ADI_MAX32_SYSCLK_SRC);

#if DT_NODE_HAS_PROP(DT_NODELABEL(gcr), sysclk_prescaler)
	/* Setup divider */
	Wrap_MXC_SYS_SetClockDiv(sysclk_prescaler(ADI_MAX32_SYSCLK_PRESCALER));
#endif

	return 0;
}

#define MAX32_CLOCK_CONTROL_INIT(_num)                                                             \
	static struct max32_clkctrl_config max32_clkctrl_config##_num = {                          \
		.regs = (mxc_gcr_regs_t *)DT_INST_REG_ADDR(_num),                                  \
		.prescaler = DT_PROP_OR(_num, sysclk_prescaler, 0),                                \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(_num, max32_clkctrl_init, NULL, NULL, &max32_clkctrl_config##_num,   \
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,                    \
			      &max32_clkctrl_api)

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gcr), okay)
MAX32_CLOCK_CONTROL_INIT(0);
#endif
