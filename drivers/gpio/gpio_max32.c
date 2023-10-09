/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <pinctrl_soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>
#include <zephyr/dt-bindings/gpio/adi-max32-gpio.h>

#include <gpio.h>

#define DT_DRV_COMPAT adi_max32_gpio

LOG_MODULE_REGISTER(gpio_max32, CONFIG_GPIO_LOG_LEVEL);

#define GPIO_CFG(dev)  ((struct max32_gpio_config *)((dev)->config))
#define GPIO_DATA(dev) ((struct max32_gpio_data *)((dev)->data))

struct max32_gpio_config {
	struct gpio_driver_config common;
	mxc_gpio_regs_t *regs;
	const struct pinctrl_dev_config *pctrl;
	const struct device *clock;
	void (*irq_func)(void);
	struct max32_perclk perclk;
};

struct max32_gpio_data {
	struct gpio_driver_data common;
	sys_slist_t cb_list;
};

static int api_port_get_raw(const struct device *dev, uint32_t *value)
{
	*value = MXC_GPIO_InGet(GPIO_CFG(dev)->regs, (unsigned int)-1);
	return 0;
}

static int api_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
				   gpio_port_value_t value)
{
	MXC_GPIO_OutPut(GPIO_CFG(dev)->regs, mask, value);
	return 0;
}

static int api_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	MXC_GPIO_OutSet(GPIO_CFG(dev)->regs, pins);
	return 0;
}

static int api_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	MXC_GPIO_OutClr(GPIO_CFG(dev)->regs, pins);
	return 0;
}

static int api_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	MXC_GPIO_OutToggle(GPIO_CFG(dev)->regs, pins);
	return 0;
}

int gpio_max32_config_pinmux(const struct device *dev, int pin, int afx, int pincfg)
{
	mxc_gpio_cfg_t gpio_cfg;

	gpio_cfg.port = GPIO_CFG(dev)->regs;
	gpio_cfg.mask = BIT(pin);

	if (pincfg & BIT(MAX32_BIAS_PULL_UP_SHIFT)) {
		gpio_cfg.pad = MXC_GPIO_PAD_PULL_UP;
	} else if (pincfg & BIT(MAX32_BIAS_PULL_DOWN_SHIFT)) {
		gpio_cfg.pad = MXC_GPIO_PAD_PULL_DOWN;
	} else {
		gpio_cfg.pad = MXC_GPIO_PAD_NONE;
	}

	if (pincfg & BIT(MAX32_INPUT_ENABLE_SHIFT)) {
		gpio_cfg.func = MXC_GPIO_FUNC_IN;
	} else if (pincfg & BIT(MAX32_OUTPUT_ENABLE_SHIFT)) {
		gpio_cfg.func = MXC_GPIO_FUNC_OUT;
	} else {
		/* Add +1 to index match */
		gpio_cfg.func = (mxc_gpio_func_t)(afx + 1);
	}

	if (pincfg & BIT(MAX32_POWER_SOURCE_SHIFT)) {
		gpio_cfg.vssel = MXC_GPIO_VSSEL_VDDIOH;
	} else {
		gpio_cfg.vssel = MXC_GPIO_VSSEL_VDDIO;
	}

	switch (pincfg & MAX32_GPIO_DRV_STRENGTH_MASK) {
	case MAX32_GPIO_DRV_STRENGTH_1:
		gpio_cfg.drvstr = MXC_GPIO_DRVSTR_1;
		break;
	case MAX32_GPIO_DRV_STRENGTH_2:
		gpio_cfg.drvstr = MXC_GPIO_DRVSTR_2;
		break;
	case MAX32_GPIO_DRV_STRENGTH_3:
		gpio_cfg.drvstr = MXC_GPIO_DRVSTR_3;
		break;
	default:
		gpio_cfg.drvstr = MXC_GPIO_DRVSTR_0;
		break;
	}

	MXC_GPIO_Config(&gpio_cfg);

	if (pincfg & BIT(MAX32_OUTPUT_ENABLE_SHIFT)) {
		if (pincfg & BIT(MAX32_OUTPUT_HIGH_SHIFT)) {
			MXC_GPIO_OutSet(gpio_cfg.port, BIT(pin));
		} else {
			MXC_GPIO_OutClr(gpio_cfg.port, BIT(pin));
		}
	}

	return 0;
}

static int api_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	mxc_gpio_regs_t *gpio = GPIO_CFG(dev)->regs;
	mxc_gpio_cfg_t gpio_cfg;

	gpio_cfg.port = gpio;
	gpio_cfg.mask = BIT(pin);

	if (flags & GPIO_PULL_UP) {
		gpio_cfg.pad = MXC_GPIO_PAD_PULL_UP;
	} else if (flags & GPIO_PULL_DOWN) {
		gpio_cfg.pad = MXC_GPIO_PAD_PULL_DOWN;
	} else if (flags & MAX32_GPIO_WEAK_PULL_UP) {
		gpio_cfg.pad = MXC_GPIO_PAD_WEAK_PULL_UP;
	} else if (flags & MAX32_GPIO_WEAK_PULL_DOWN) {
		gpio_cfg.pad = MXC_GPIO_PAD_WEAK_PULL_DOWN;
	} else {
		gpio_cfg.pad = MXC_GPIO_PAD_NONE;
	}

	if (flags & GPIO_OUTPUT) {
		gpio_cfg.func = MXC_GPIO_FUNC_OUT;
	} else if (flags & GPIO_INPUT) {
		gpio_cfg.func = MXC_GPIO_FUNC_IN;
	} else {
		/* this case will not occur this function call for gpio mode in/out */
		gpio_cfg.func = MXC_GPIO_FUNC_ALT1; /* TODO: Think on it */
	}

	if (flags & MAX32_GPIO_VSEL_VDDIOH) {
		gpio_cfg.vssel = MXC_GPIO_VSSEL_VDDIOH;
	} else {
		gpio_cfg.vssel = MXC_GPIO_VSSEL_VDDIO;
	}

	switch (flags & MAX32_GPIO_DRV_STRENGTH_MASK) {
	case MAX32_GPIO_DRV_STRENGTH_1:
		gpio_cfg.drvstr = MXC_GPIO_DRVSTR_1;
		break;
	case MAX32_GPIO_DRV_STRENGTH_2:
		gpio_cfg.drvstr = MXC_GPIO_DRVSTR_2;
		break;
	case MAX32_GPIO_DRV_STRENGTH_3:
		gpio_cfg.drvstr = MXC_GPIO_DRVSTR_3;
		break;
	default:
		gpio_cfg.drvstr = MXC_GPIO_DRVSTR_0;
		break;
	}

	MXC_GPIO_Config(&gpio_cfg);

	if (flags & GPIO_OUTPUT) {
		if (flags & GPIO_OUTPUT_INIT_LOW) {
			MXC_GPIO_OutClr(gpio, BIT(pin));
		} else if (flags & GPIO_OUTPUT_INIT_HIGH) {
			MXC_GPIO_OutSet(gpio, BIT(pin));
		}
	}

	return 0;
}

static int api_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
				       enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	mxc_gpio_regs_t *gpio = GPIO_CFG(dev)->regs;
	mxc_gpio_cfg_t gpio_cfg;

	gpio_cfg.port = gpio;
	gpio_cfg.mask = BIT(pin);
	/* rest of the parameters not necessary */

	if (mode == GPIO_INT_MODE_DISABLED) {
		MXC_GPIO_DisableInt(gpio, gpio_cfg.mask);
		return 0;
	}

	switch (mode) {
	case GPIO_INT_MODE_LEVEL:
		if (trig == GPIO_INT_TRIG_LOW) {
			MXC_GPIO_IntConfig(&gpio_cfg, MXC_GPIO_INT_LOW);
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			MXC_GPIO_IntConfig(&gpio_cfg, MXC_GPIO_INT_HIGH);
		} else if (trig == GPIO_INT_TRIG_BOTH) {
			MXC_GPIO_IntConfig(&gpio_cfg, MXC_GPIO_INT_BOTH);
		} else {
			return -EINVAL;
		}
		break;
	case GPIO_INT_MODE_EDGE:
		if (trig == GPIO_INT_TRIG_LOW) {
			MXC_GPIO_IntConfig(&gpio_cfg, MXC_GPIO_INT_FALLING);
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			MXC_GPIO_IntConfig(&gpio_cfg, MXC_GPIO_INT_RISING);
		} else if (trig == GPIO_INT_TRIG_BOTH) {
			MXC_GPIO_IntConfig(&gpio_cfg, MXC_GPIO_INT_BOTH);
		} else {
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	MXC_GPIO_EnableInt(gpio, gpio_cfg.mask);

	return 0;
}

static int api_manage_callback(const struct device *dev, struct gpio_callback *callback, bool set)
{
	return gpio_manage_callback(&(GPIO_DATA(dev)->cb_list), callback, set);
}

static const struct gpio_driver_api gpio_max32_driver = {
	.pin_configure = api_pin_configure,
	.port_get_raw = api_port_get_raw,
	.port_set_masked_raw = api_port_set_masked_raw,
	.port_set_bits_raw = api_port_set_bits_raw,
	.port_clear_bits_raw = api_port_clear_bits_raw,
	.port_toggle_bits = api_port_toggle_bits,
	.pin_interrupt_configure = api_pin_interrupt_configure,
	.manage_callback = api_manage_callback,
};

static void gpio_max32_isr(const void *param)
{
	const struct device *dev = param;

	unsigned int flags = MXC_GPIO_GetFlags(GPIO_CFG(dev)->regs);
	/* clear interrupt flags */
	MXC_GPIO_ClearFlags(GPIO_CFG(dev)->regs, flags);

	gpio_fire_callbacks(&(GPIO_DATA(dev)->cb_list), dev, flags);
}

static int gpio_max32_init(const struct device *dev)
{
	const struct max32_gpio_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->clock)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* enable clock */
	ret = clock_control_on(cfg->clock, (clock_control_subsys_t) &(cfg->perclk));
	if (ret != 0) {
		LOG_ERR("cannot enable GPIO clock");
		return ret;
	}

	cfg->irq_func();

	return 0;
}

#define MAX32_GPIO_INIT(_num)                                                                      \
	static void gpio_max32_irq_init_##_num(void)                                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(_num), DT_INST_IRQ(_num, priority), gpio_max32_isr,       \
			    DEVICE_DT_INST_GET(_num), 0);                                          \
		irq_enable(DT_INST_IRQN(_num));                                                    \
	}                                                                                          \
	static struct max32_gpio_data max32_gpio_data_##_num;                                      \
	static const struct max32_gpio_config max32_gpio_config_##_num = {                         \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(_num),            \
			},                                                                         \
		.regs = (mxc_gpio_regs_t *)DT_INST_REG_ADDR(_num),                                 \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(_num)),                                 \
		.perclk.bus = DT_INST_CLOCKS_CELL(_num, offset),                                   \
		.perclk.bit = DT_INST_CLOCKS_CELL(_num, bit),                                      \
		.irq_func = &gpio_max32_irq_init_##_num,                                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(_num, gpio_max32_init, NULL, &max32_gpio_data_##_num,                \
			      &max32_gpio_config_##_num, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,  \
			      (void *)&gpio_max32_driver);

DT_INST_FOREACH_STATUS_OKAY(MAX32_GPIO_INIT)
