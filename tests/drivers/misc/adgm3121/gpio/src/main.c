/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/ztest.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/misc/adgm3121/adgm3121.h>

#define ADGM3121_NODE DT_NODELABEL(adgm3121_gpio)

static const struct device *dev = DEVICE_DT_GET(ADGM3121_NODE);
static const struct gpio_dt_spec gpios[] = {
	GPIO_DT_SPEC_GET(ADGM3121_NODE, in1_gpios),
	GPIO_DT_SPEC_GET(ADGM3121_NODE, in2_gpios),
	GPIO_DT_SPEC_GET(ADGM3121_NODE, in3_gpios),
	GPIO_DT_SPEC_GET(ADGM3121_NODE, in4_gpios),
};

ZTEST(adgm3121_gpio, test_device_ready)
{
	zassert_true(device_is_ready(dev), "ADGM3121 device not ready");
}

ZTEST(adgm3121_gpio, test_gpios_configured_as_output)
{
	gpio_flags_t flags;

	for (int i = 0; i < 4; i++) {
		int ret = gpio_emul_flags_get(gpios[i].port, gpios[i].pin,
					      &flags);
		zassert_ok(ret, "Failed to get GPIO %d flags: %d", i, ret);
		zassert_true(flags & GPIO_OUTPUT,
			     "GPIO %d should be configured as output", i);
	}
}

ZTEST(adgm3121_gpio, test_set_switch_drives_gpio)
{
	int ret;

	ret = adgm3121_set_switch_state(dev, ADGM3121_SW1, ADGM3121_ENABLE);
	zassert_ok(ret, "Failed to enable SW1: %d", ret);
	zassert_equal(gpio_emul_output_get(gpios[0].port, gpios[0].pin), 1,
		      "GPIO 0 should be high when SW1 enabled");

	ret = adgm3121_set_switch_state(dev, ADGM3121_SW1, ADGM3121_DISABLE);
	zassert_ok(ret, "Failed to disable SW1: %d", ret);
	zassert_equal(gpio_emul_output_get(gpios[0].port, gpios[0].pin), 0,
		      "GPIO 0 should be low when SW1 disabled");
}

ZTEST(adgm3121_gpio, test_set_switches_drives_all_gpios)
{
	int ret;

	ret = adgm3121_set_switches(dev, 0x0A);
	zassert_ok(ret, "Failed to set switches: %d", ret);

	zassert_equal(gpio_emul_output_get(gpios[0].port, gpios[0].pin), 0,
		      "GPIO 0 should be low (SW1 off)");
	zassert_equal(gpio_emul_output_get(gpios[1].port, gpios[1].pin), 1,
		      "GPIO 1 should be high (SW2 on)");
	zassert_equal(gpio_emul_output_get(gpios[2].port, gpios[2].pin), 0,
		      "GPIO 2 should be low (SW3 off)");
	zassert_equal(gpio_emul_output_get(gpios[3].port, gpios[3].pin), 1,
		      "GPIO 3 should be high (SW4 on)");
}

ZTEST(adgm3121_gpio, test_get_switch_reads_gpio)
{
	enum adgm3121_state state;
	int ret;

	ret = adgm3121_set_switch_state(dev, ADGM3121_SW3, ADGM3121_ENABLE);
	zassert_ok(ret);

	ret = adgm3121_get_switch_state(dev, ADGM3121_SW3, &state);
	zassert_ok(ret, "Failed to get SW3 state: %d", ret);
	zassert_equal(state, ADGM3121_ENABLE, "SW3 should be enabled");
}

ZTEST(adgm3121_gpio, test_get_switches_bitmask)
{
	uint8_t mask;
	int ret;

	ret = adgm3121_set_switches(dev, 0x05);
	zassert_ok(ret);

	ret = adgm3121_get_switches(dev, &mask);
	zassert_ok(ret, "Failed to get switches: %d", ret);
	zassert_equal(mask, 0x05, "Switch mask should be 0x05, got 0x%02x",
		      mask);
}

ZTEST(adgm3121_gpio, test_reset_clears_all_gpios)
{
	int ret;

	ret = adgm3121_set_switches(dev, 0x0F);
	zassert_ok(ret);

	ret = adgm3121_reset_switches(dev);
	zassert_ok(ret, "Failed to reset switches: %d", ret);

	for (int i = 0; i < 4; i++) {
		zassert_equal(gpio_emul_output_get(gpios[i].port,
						   gpios[i].pin), 0,
			      "GPIO %d should be low after reset", i);
	}
}

ZTEST(adgm3121_gpio, test_all_switches_individually)
{
	int ret;

	ret = adgm3121_reset_switches(dev);
	zassert_ok(ret);

	for (int i = 0; i < 4; i++) {
		ret = adgm3121_set_switch_state(dev, i, ADGM3121_ENABLE);
		zassert_ok(ret, "Failed to enable SW%d: %d", i + 1, ret);
		zassert_equal(gpio_emul_output_get(gpios[i].port,
						   gpios[i].pin), 1,
			      "GPIO %d should be high", i);
	}
}

ZTEST(adgm3121_gpio, test_check_internal_error_unsupported)
{
	uint8_t error_status;
	int ret;

	ret = adgm3121_check_internal_error(dev, &error_status);
	zassert_equal(ret, -ENOTSUP,
		      "check_internal_error should return -ENOTSUP in GPIO mode");
}

ZTEST(adgm3121_gpio, test_invalid_switch)
{
	int ret;

	ret = adgm3121_set_switch_state(dev, 4, ADGM3121_ENABLE);
	zassert_equal(ret, -EINVAL, "Expected -EINVAL for invalid switch");
}

static void *adgm3121_gpio_setup(void)
{
	zassert_true(device_is_ready(dev), "ADGM3121 device not ready");
	return NULL;
}

static void adgm3121_gpio_before(void *fixture)
{
	ARG_UNUSED(fixture);
	adgm3121_reset_switches(dev);
}

ZTEST_SUITE(adgm3121_gpio, NULL, adgm3121_gpio_setup, adgm3121_gpio_before,
	    NULL, NULL);
