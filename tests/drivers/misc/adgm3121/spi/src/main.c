/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/ztest.h>
#include <zephyr/drivers/misc/adgm3121/adgm3121.h>

#define ADGM3121_NODE DT_NODELABEL(adgm3121)

static const struct device *dev = DEVICE_DT_GET(ADGM3121_NODE);

ZTEST(adgm3121_spi, test_device_ready)
{
	zassert_true(device_is_ready(dev), "ADGM3121 device not ready");
}

ZTEST(adgm3121_spi, test_set_get_switch_state)
{
	enum adgm3121_state state;
	int ret;

	ret = adgm3121_set_switch_state(dev, ADGM3121_SW1, ADGM3121_ENABLE);
	zassert_ok(ret, "Failed to enable SW1: %d", ret);

	ret = adgm3121_get_switch_state(dev, ADGM3121_SW1, &state);
	zassert_ok(ret, "Failed to get SW1 state: %d", ret);
	zassert_equal(state, ADGM3121_ENABLE, "SW1 should be enabled");

	ret = adgm3121_set_switch_state(dev, ADGM3121_SW1, ADGM3121_DISABLE);
	zassert_ok(ret, "Failed to disable SW1: %d", ret);

	ret = adgm3121_get_switch_state(dev, ADGM3121_SW1, &state);
	zassert_ok(ret, "Failed to get SW1 state: %d", ret);
	zassert_equal(state, ADGM3121_DISABLE, "SW1 should be disabled");
}

ZTEST(adgm3121_spi, test_set_get_switches_bitmask)
{
	uint8_t mask;
	int ret;

	ret = adgm3121_set_switches(dev, 0x0A);
	zassert_ok(ret, "Failed to set switches: %d", ret);

	ret = adgm3121_get_switches(dev, &mask);
	zassert_ok(ret, "Failed to get switches: %d", ret);
	zassert_equal(mask, 0x0A, "Switch mask should be 0x0A, got 0x%02x", mask);

	ret = adgm3121_set_switches(dev, 0x05);
	zassert_ok(ret, "Failed to set switches: %d", ret);

	ret = adgm3121_get_switches(dev, &mask);
	zassert_ok(ret, "Failed to get switches: %d", ret);
	zassert_equal(mask, 0x05, "Switch mask should be 0x05, got 0x%02x", mask);
}

ZTEST(adgm3121_spi, test_set_switches_mask_truncation)
{
	uint8_t mask;
	int ret;

	ret = adgm3121_set_switches(dev, 0xFF);
	zassert_ok(ret, "Failed to set switches: %d", ret);

	ret = adgm3121_get_switches(dev, &mask);
	zassert_ok(ret, "Failed to get switches: %d", ret);
	zassert_equal(mask, 0x0F,
		      "Upper bits should be masked, got 0x%02x", mask);
}

ZTEST(adgm3121_spi, test_reset_switches)
{
	uint8_t mask;
	int ret;

	ret = adgm3121_set_switches(dev, 0x0F);
	zassert_ok(ret, "Failed to set switches: %d", ret);

	ret = adgm3121_reset_switches(dev);
	zassert_ok(ret, "Failed to reset switches: %d", ret);

	ret = adgm3121_get_switches(dev, &mask);
	zassert_ok(ret, "Failed to get switches: %d", ret);
	zassert_equal(mask, 0x00, "All switches should be off, got 0x%02x",
		      mask);
}

ZTEST(adgm3121_spi, test_all_switches_individually)
{
	enum adgm3121_state state;
	int ret;

	ret = adgm3121_reset_switches(dev);
	zassert_ok(ret);

	for (int i = ADGM3121_SW1; i <= ADGM3121_SW4; i++) {
		ret = adgm3121_set_switch_state(dev, i, ADGM3121_ENABLE);
		zassert_ok(ret, "Failed to enable SW%d: %d", i + 1, ret);

		ret = adgm3121_get_switch_state(dev, i, &state);
		zassert_ok(ret, "Failed to get SW%d state: %d", i + 1, ret);
		zassert_equal(state, ADGM3121_ENABLE,
			      "SW%d should be enabled", i + 1);
	}

	uint8_t mask;

	ret = adgm3121_get_switches(dev, &mask);
	zassert_ok(ret);
	zassert_equal(mask, 0x0F, "All switches should be on, got 0x%02x",
		      mask);
}

ZTEST(adgm3121_spi, test_check_internal_error)
{
	uint8_t error_status;
	int ret;

	ret = adgm3121_check_internal_error(dev, &error_status);
	zassert_ok(ret, "Failed to check internal error: %d", ret);
	zassert_equal(error_status, 0, "Expected no errors, got 0x%02x",
		      error_status);
}

ZTEST(adgm3121_spi, test_invalid_switch)
{
	int ret;

	ret = adgm3121_set_switch_state(dev, 4, ADGM3121_ENABLE);
	zassert_equal(ret, -EINVAL, "Expected -EINVAL for invalid switch");
}

ZTEST(adgm3121_spi, test_null_params)
{
	int ret;

	ret = adgm3121_get_switch_state(dev, ADGM3121_SW1, NULL);
	zassert_equal(ret, -EINVAL, "Expected -EINVAL for NULL state");

	ret = adgm3121_get_switches(dev, NULL);
	zassert_equal(ret, -EINVAL, "Expected -EINVAL for NULL mask");

	ret = adgm3121_check_internal_error(dev, NULL);
	zassert_equal(ret, -EINVAL, "Expected -EINVAL for NULL error_status");
}

static void *adgm3121_spi_setup(void)
{
	zassert_true(device_is_ready(dev), "ADGM3121 device not ready");
	return NULL;
}

static void adgm3121_spi_before(void *fixture)
{
	ARG_UNUSED(fixture);
	adgm3121_reset_switches(dev);
}

ZTEST_SUITE(adgm3121_spi, NULL, adgm3121_spi_setup, adgm3121_spi_before,
	    NULL, NULL);
