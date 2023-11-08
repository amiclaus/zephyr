/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max32_pwm

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/sys/util_macro.h>

#include <wrap_max32_tmr.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_max32, CONFIG_PWM_LOG_LEVEL);

/** PWM configuration. */
struct max32_pwm_config {
	mxc_tmr_regs_t *regs;
	const struct pinctrl_dev_config *pctrl;
	const struct device *clock;
	struct max32_perclk perclk;
	int clock_source;
	int prescaler;
};

/** PWM data. */
struct max32_pwm_data {
	uint32_t period_cycles;
};

static int api_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
			  uint32_t pulse_cycles, pwm_flags_t flags)
{
	int ret = 0;
	const struct max32_pwm_config *cfg = dev->config;
	mxc_tmr_regs_t *regs = cfg->regs;
	wrap_mxc_tmr_cfg_t pwm_cfg;
	int prescaler_index;

	prescaler_index = LOG2(cfg->prescaler);
	if (prescaler_index == 0) {
		pwm_cfg.pres = TMR_PRES_1; /* TMR_PRES_1 is 0 */
	} else {
		/* TMR_PRES_2 is  1<<X */
		pwm_cfg.pres = TMR_PRES_2 + (prescaler_index - 1);
	}
	pwm_cfg.mode = TMR_MODE_PWM;
	pwm_cfg.cmp_cnt = period_cycles;
	pwm_cfg.pol = 1;
	pwm_cfg.bitMode = 0;

	pwm_cfg.clock = Wrap_MXC_TMR_GetClockIndex(cfg->clock_source);
	if (pwm_cfg.clock < 0) {
		return -ENOTSUP;
	}

	MXC_TMR_Shutdown(regs);

	/* enable clock */
	ret = clock_control_on(cfg->clock, (clock_control_subsys_t)&cfg->perclk);
	if (ret) {
		return ret;
	}

	Wrap_MXC_TMR_Init(regs, &pwm_cfg);

	ret = MXC_TMR_SetPWM(regs, pulse_cycles);
	if (ret != E_NO_ERROR) {
		return ret;
	}

	MXC_TMR_Start(regs);

	return 0;
}

static int api_get_cycles_per_sec(const struct device *dev, uint32_t channel, uint64_t *cycles)
{
	const struct max32_pwm_config *cfg = dev->config;
	uint32_t clk_frequency = 0;

	switch (cfg->clock_source) {
	case ADI_MAX32_PRPH_CLK_SRC_PCLK:
		clk_frequency = PeripheralClock;
		break;

	case ADI_MAX32_PRPH_CLK_SRC_EXTCLK:
		clk_frequency = ADI_MAX32_CLK_EXTCLK_FREQ;
		break;

	case ADI_MAX32_PRPH_CLK_SRC_IBRO:
		clk_frequency = ADI_MAX32_CLK_IBRO_FREQ;
		break;

	case ADI_MAX32_PRPH_CLK_SRC_ERFO:
		clk_frequency = ADI_MAX32_CLK_ERFO_FREQ;
		break;

	case ADI_MAX32_PRPH_CLK_SRC_ERTCO:
		clk_frequency = ADI_MAX32_CLK_ERTCO_FREQ;
		break;

	case ADI_MAX32_PRPH_CLK_SRC_INRO:
		clk_frequency = ADI_MAX32_CLK_INRO_FREQ;
		break;

	case ADI_MAX32_PRPH_CLK_SRC_ISO:
		clk_frequency = ADI_MAX32_CLK_ISO_FREQ;
		break;

	case ADI_MAX32_PRPH_CLK_SRC_IBRO_DIV8:
		clk_frequency = ADI_MAX32_CLK_IBRO_FREQ / 8;
		break;

	default:
		return -ENOTSUP; /* Unsupported clock source */
	}

	*cycles = (uint64_t)(clk_frequency / cfg->prescaler);

	return 0;
}

static const struct pwm_driver_api pwm_max32_driver_api = {
	.set_cycles = api_set_cycles,
	.get_cycles_per_sec = api_get_cycles_per_sec,
};

static int pwm_max32_init(const struct device *dev)
{
	int ret = 0;
	const struct max32_pwm_config *cfg = dev->config;

	ret = pinctrl_apply_state(cfg->pctrl, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("PWM pinctrl initialization failed (%d)", ret);
		return ret;
	}

	return ret;
}

#define PWM_MAX32_DEFINE(_num)                                                                     \
	static struct max32_pwm_data max32_pwm_data_##_num;                                        \
	PINCTRL_DT_INST_DEFINE(_num);                                                              \
	static const struct max32_pwm_config max32_pwm_config_##_num = {                           \
		.regs = (mxc_tmr_regs_t *)DT_REG_ADDR(DT_INST_PARENT(_num)),                       \
		.pctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(_num),                                     \
		.clock = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(_num))),                      \
		.perclk.bus = DT_CLOCKS_CELL(DT_INST_PARENT(_num), offset),                        \
		.perclk.bit = DT_CLOCKS_CELL(DT_INST_PARENT(_num), bit),                           \
		.clock_source = DT_PROP(DT_INST_PARENT(_num), clock_source),                       \
		.prescaler = DT_PROP(DT_INST_PARENT(_num), prescaler),                             \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(_num, &pwm_max32_init, NULL, &max32_pwm_data_##_num,                 \
			      &max32_pwm_config_##_num, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,     \
			      &pwm_max32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_MAX32_DEFINE)
