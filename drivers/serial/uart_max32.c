/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control/adi_max32_clock_control.h>

#include <wrap_max32_uart.h>

#define DT_DRV_COMPAT adi_max32_uart

LOG_MODULE_REGISTER(uart_max32, CONFIG_UART_LOG_LEVEL);

struct max32_uart_config {
	mxc_uart_regs_t *regs;
	int clock_source;
	const struct pinctrl_dev_config *pctrl;
	const struct device *clock;
	struct max32_perclk perclk;
	struct uart_config uart_conf;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct max32_uart_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb; /* Interrupt Callback */
	void *cb_data;                    /* Interrupt Callback Arg */
#endif
	struct uart_config conf; /* baudrate, stopbits, ... */
};

static void api_poll_out(const struct device *dev, unsigned char c)
{
	const struct max32_uart_config *cfg = dev->config;

	MXC_UART_WriteCharacter(cfg->regs, c);
}

static int api_poll_in(const struct device *dev, unsigned char *c)
{
	int val;
	const struct max32_uart_config *cfg = dev->config;

	val = MXC_UART_ReadCharacterRaw(cfg->regs);
	if (val >= 0) {
		*c = (unsigned char)val;
	} else {
		return -1;
	}

	return 0;
}

static int api_err_check(const struct device *dev)
{
	int err = 0;
	uint32_t flags;
	const struct max32_uart_config *cfg = dev->config;

	flags = MXC_UART_GetFlags(cfg->regs);

	if (flags & ADI_MAX32_UART_ERROR_FRAMING) {
		err |= UART_ERROR_FRAMING;
	}

	if (flags & ADI_MAX32_UART_ERROR_PARITY) {
		err |= UART_ERROR_PARITY;
	}

	if (flags & ADI_MAX32_UART_ERROR_OVERRUN) {
		err |= UART_ERROR_OVERRUN;
	}

	return err;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE

#define CONVERT_TO_MXC_DATABITS(x) (x + 5)

static int api_configure(const struct device *dev, const struct uart_config *uart_cfg)
{
	int err;
	const struct max32_uart_config *const cfg = dev->config;
	mxc_uart_regs_t *regs = cfg->regs;
	struct max32_uart_data *uart_priv_data = (struct max32_uart_data *)(dev->data);

	/*
	 *  Set parity
	 */
	mxc_uart_parity_t mxc_parity;

	switch (uart_cfg->parity) {
	case UART_CFG_PARITY_NONE:
		mxc_parity = ADI_MAX32_UART_CFG_PARITY_NONE;
		break;
	case UART_CFG_PARITY_ODD:
		mxc_parity = ADI_MAX32_UART_CFG_PARITY_ODD;
		break;
	case UART_CFG_PARITY_EVEN:
		mxc_parity = ADI_MAX32_UART_CFG_PARITY_EVEN;
		break;
	case UART_CFG_PARITY_MARK:
#if defined(CONFIG_SERIAL_SUPPORT_PARITY_MARK)
		mxc_parity = ADI_MAX32_UART_CFG_PARITY_MARK;
		break;
#else
		return -ENOTSUP;
#endif
	case UART_CFG_PARITY_SPACE:
#if defined(CONFIG_SERIAL_SUPPORT_PARITY_SPACE)
		mxc_parity = ADI_MAX32_UART_CFG_PARITY_SPACE;
		break;
#else
		return -ENOTSUP;
#endif
	default:
		return -EINVAL;
	}

	err = MXC_UART_SetParity(regs, mxc_parity);
	if (err < 0) {
		return -ENOTSUP;
	}
	/* incase of success keep configuration */
	uart_priv_data->conf.parity = uart_cfg->parity;

	/*
	 *  Set stop bit
	 */
	if (uart_cfg->stop_bits == UART_CFG_STOP_BITS_1) {
		err = MXC_UART_SetStopBits(regs, MXC_UART_STOP_1);
	} else if (uart_cfg->stop_bits == UART_CFG_STOP_BITS_2) {
		err = MXC_UART_SetStopBits(regs, MXC_UART_STOP_2);
	} else {
		return -ENOTSUP;
	}
	if (err < 0) {
		return -ENOTSUP;
	}
	/* incase of success keep configuration */
	uart_priv_data->conf.stop_bits = uart_cfg->stop_bits;

	/*
	 *  Set data bit
	 */
	err = MXC_UART_SetDataSize(regs, CONVERT_TO_MXC_DATABITS(uart_cfg->data_bits));
	if (err < 0) {
		return -ENOTSUP;
	}
	/* incase of success keep configuration */
	uart_priv_data->conf.data_bits = uart_cfg->data_bits;

	/*
	 *  TODO: Set flow control
	 */

	/*
	 *  Set frequency
	 */
	err = Wrap_MXC_UART_SetFrequency(regs, uart_cfg->baudrate, cfg->clock_source);
	if (err < 0) {
		return -ENOTSUP;
	}

	/* incase of success keep configuration */
	uart_priv_data->conf.baudrate = uart_cfg->baudrate;

	return 0;
}

static int api_config_get(const struct device *dev, struct uart_config *uart_cfg)
{
	struct max32_uart_data *uart_priv_data = (struct max32_uart_data *)(dev->data);

	/* copy configs from global setting */
	*uart_cfg = uart_priv_data->conf;

	return 0;
}

#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static int uart_max32_init(const struct device *dev)
{
	int ret;
	const struct max32_uart_config *const cfg = dev->config;
	mxc_uart_regs_t *regs = cfg->regs;
	struct max32_uart_data *uart_priv_data = (struct max32_uart_data *)(dev->data);

	if (!device_is_ready(cfg->clock)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	ret = MXC_UART_Shutdown(regs);
	if (ret) {
		return ret;
	}

	ret = clock_control_on(cfg->clock, (clock_control_subsys_t) &(cfg->perclk));
	if (ret != 0) {
		LOG_ERR("cannot enable UART clock");
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pctrl, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	ret = Wrap_MXC_UART_Init(regs, cfg->uart_conf.baudrate, cfg->clock_source);
	if (ret) {
		return ret;
	}

	/* store configuration at the global */
	uart_priv_data->conf = cfg->uart_conf;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/* Clear any pending UART RX/TX interrupts */
	MXC_UART_ClearFlags(regs, (ADI_MAX32_UART_INT_TX | ADI_MAX32_UART_INT_TX));
	cfg->irq_config_func(dev);
#endif

	return ret;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int api_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	unsigned int num_tx = 0;
	const struct max32_uart_config *cfg = dev->config;

	num_tx = MXC_UART_WriteTXFIFO(cfg->regs, (unsigned char *)tx_data, size);

	return (int)num_tx;
}

static int api_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	unsigned int num_rx = 0;
	const struct max32_uart_config *cfg = dev->config;

	num_rx = MXC_UART_ReadRXFIFO(cfg->regs, (unsigned char *)rx_data, size);

	return num_rx;
}

static void api_irq_tx_enable(const struct device *dev)
{
	const struct max32_uart_config *cfg = dev->config;

	MXC_UART_EnableInt(cfg->regs, ADI_MAX32_UART_INT_TX);
}

static void api_irq_tx_disable(const struct device *dev)
{
	const struct max32_uart_config *cfg = dev->config;

	MXC_UART_DisableInt(cfg->regs, ADI_MAX32_UART_INT_TX);
}

static int api_irq_tx_ready(const struct device *dev)
{
	const struct max32_uart_config *cfg = dev->config;

	return (MXC_UART_GetFlags(cfg->regs) & ADI_MAX32_UART_INT_TX);
}

static void api_irq_rx_enable(const struct device *dev)
{
	const struct max32_uart_config *cfg = dev->config;

	MXC_UART_EnableInt(cfg->regs, ADI_MAX32_UART_INT_RX);
}

static void api_irq_rx_disable(const struct device *dev)
{
	const struct max32_uart_config *cfg = dev->config;

	MXC_UART_DisableInt(cfg->regs, ADI_MAX32_UART_INT_RX);
}

static int api_irq_tx_complete(const struct device *dev)
{
	const struct max32_uart_config *cfg = dev->config;

	if (MXC_UART_GetActive(cfg->regs) == E_BUSY) {
		return 0;
	} else {
		return 1; /* tranmission completed */
	}
}

static int api_irq_rx_ready(const struct device *dev)
{
	const struct max32_uart_config *cfg = dev->config;

	return (MXC_UART_GetFlags(cfg->regs) & ADI_MAX32_UART_INT_RX);
}

static void api_irq_err_enable(const struct device *dev)
{
	const struct max32_uart_config *cfg = dev->config;

	MXC_UART_EnableInt(cfg->regs, ADI_MAX32_UART_ERROR_INTERRUPTS);
}

static void api_irq_err_disable(const struct device *dev)
{
	const struct max32_uart_config *cfg = dev->config;

	MXC_UART_DisableInt(cfg->regs, ADI_MAX32_UART_ERROR_INTERRUPTS);
}

static int api_irq_is_pending(const struct device *dev)
{
	const struct max32_uart_config *cfg = dev->config;

	return (MXC_UART_GetFlags(cfg->regs) & (ADI_MAX32_UART_INT_RX | ADI_MAX32_UART_INT_TX));
}

static int api_irq_update(const struct device *dev)
{
	return 1;
}

static void api_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				 void *cb_data)
{
	struct max32_uart_data *const dev_data = (struct max32_uart_data *)(dev->data);

	dev_data->cb = cb;
	dev_data->cb_data = cb_data;
}

static void uart_max32_isr(const struct device *dev)
{
	const struct max32_uart_config *const cfg = dev->config;
	struct max32_uart_data *dev_data = (struct max32_uart_data *)(dev->data);
	unsigned int int_status;

	int_status = MXC_UART_GetFlags(cfg->regs);

	if (dev_data->cb) {
		dev_data->cb(dev, dev_data->cb_data);
	}

	/* Clear RX/TX interrupts flag after cb called */
	MXC_UART_ClearFlags(cfg->regs, int_status);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API

static int api_callback_set(const struct device *dev, uart_callback_t callback, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(callback);
	ARG_UNUSED(user_data);
	return 0;
}

static int api_tx(const struct device *dev, const uint8_t *buf, size_t len, int32_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	ARG_UNUSED(timeout);
	return 0;
}

static int api_tx_abort(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int api_rx_enable(const struct device *dev, uint8_t *buf, size_t len, int32_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	ARG_UNUSED(timeout);
	return 0;
}

static int api_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	return 0;
}

static int api_rx_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

#endif

static const struct uart_driver_api uart_max32_driver_api = {
	.poll_in = api_poll_in,
	.poll_out = api_poll_out,
	.err_check = api_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = api_configure,
	.config_get = api_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = api_fifo_fill,
	.fifo_read = api_fifo_read,
	.irq_tx_enable = api_irq_tx_enable,
	.irq_tx_disable = api_irq_tx_disable,
	.irq_tx_ready = api_irq_tx_ready,
	.irq_rx_enable = api_irq_rx_enable,
	.irq_rx_disable = api_irq_rx_disable,
	.irq_tx_complete = api_irq_tx_complete,
	.irq_rx_ready = api_irq_rx_ready,
	.irq_err_enable = api_irq_err_enable,
	.irq_err_disable = api_irq_err_disable,
	.irq_is_pending = api_irq_is_pending,
	.irq_update = api_irq_update,
	.irq_callback_set = api_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = api_callback_set,
	.tx = api_tx,
	.tx_abort = api_tx_abort,
	.rx_enable = api_rx_enable,
	.rx_buf_rsp = api_rx_buf_rsp,
	.rx_disable = api_rx_disable,
#endif /* CONFIG_UART_ASYNC_API */
};

#define MAX32_UART_INIT(_num)                                                                      \
	PINCTRL_DT_INST_DEFINE(_num);                                                              \
	IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN,                                                   \
		   (static void uart_max32_irq_init_##_num(const struct device *dev) {             \
			   IF_ENABLED(                                                             \
				   CONFIG_UART_INTERRUPT_DRIVEN,                                   \
				   (IRQ_CONNECT(DT_INST_IRQN(_num), DT_INST_IRQ(_num, priority),   \
						uart_max32_isr, DEVICE_DT_INST_GET(_num), 0);      \
				    irq_enable(DT_INST_IRQN(_num))));                              \
		   }));                                                                            \
	static const struct max32_uart_config max32_uart_config_##_num = {                         \
		.regs = (mxc_uart_regs_t *)DT_INST_REG_ADDR(_num),                                 \
		.pctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(_num),                                     \
		.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(_num)),                                 \
		.perclk.bus = DT_INST_CLOCKS_CELL(_num, offset),                                   \
		.perclk.bit = DT_INST_CLOCKS_CELL(_num, bit),                                      \
		.clock_source = DT_INST_PROP_OR(_num, clock_source, 0),                            \
		.uart_conf.baudrate = DT_INST_PROP(_num, current_speed),                           \
		.uart_conf.parity = DT_INST_ENUM_IDX_OR(_num, parity, UART_CFG_PARITY_NONE),       \
		.uart_conf.data_bits = DT_INST_ENUM_IDX(_num, data_bits),                          \
		IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN,                                           \
			   (.irq_config_func = uart_max32_irq_init_##_num, ))};                    \
	static struct max32_uart_data max32_uart_data##_num = {                                    \
		IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN, (.cb = NULL, ))};                         \
	DEVICE_DT_INST_DEFINE(_num, uart_max32_init, NULL, &max32_uart_data##_num,                 \
			      &max32_uart_config_##_num, PRE_KERNEL_1,                             \
			      CONFIG_SERIAL_INIT_PRIORITY, (void *)&uart_max32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX32_UART_INIT)
