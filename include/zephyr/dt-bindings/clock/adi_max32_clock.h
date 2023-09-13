/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ADI_MAX32_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ADI_MAX32_CLOCK_H_

/** Peripheral clock register */
#define ADI_MAX32_CLOCK_BUS0 0
#define ADI_MAX32_CLOCK_BUS1 1
#define ADI_MAX32_CLOCK_BUS2 2

/** Clock source for UART interface */
#define ADI_MAX32_UART_CLK_PCLK   0
#define ADI_MAX32_UART_CLK_EXTCLK 1
#define ADI_MAX32_UART_CLK_IBRO   2
#define ADI_MAX32_UART_CLK_ERFO   3
#define ADI_MAX32_UART_CLK_ERTCO  4
#define ADI_MAX32_UART_CLK_INRO   5

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ADI_MAX32_CLOCK_H_ */
