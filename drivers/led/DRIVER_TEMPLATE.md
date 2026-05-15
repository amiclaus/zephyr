# Zephyr RTOS LED Driver Template

Reference driver: `zephyr/drivers/led/lp50xx.c` (I2C-based LED driver).

This template covers every file needed to add a new Zephyr LED subsystem
driver for an Analog Devices I2C-based LED driver IC.  Replace `<devname>`
with the lowercase part number (e.g., `ltc3208`), `<DEVNAME>` with its
uppercase form (e.g., `LTC3208`), and adjust channel counts and register
maps to match your specific part.

---

## 1. Purpose & Zephyr Subsystem Mapping

Zephyr LED drivers live under the **LED subsystem** (`drivers/led/`).
The LED API (`include/zephyr/drivers/led.h`) provides a uniform interface
for controlling LEDs without exposing hardware-specific details.

### LED API Functions

The `led_driver_api` struct defines the following callbacks:

| Zephyr API function         | Purpose                                         |
|-----------------------------|-------------------------------------------------|
| `led_on()`                  | Turn an LED fully on                            |
| `led_off()`                 | Turn an LED fully off                           |
| `led_set_brightness()`      | Set brightness as a percentage (0--100)         |
| `led_blink()`               | Set up hardware blinking (delay_on/delay_off)   |
| `led_set_color()`           | Set color for multi-color LEDs (RGB/RGBW)       |
| `led_write_channels()`      | Bulk-write raw brightness to multiple channels  |
| `led_get_info()`            | Retrieve LED metadata (label, color, index)     |

Most simple LED current-sink drivers only need to implement `on`, `off`,
and `set_brightness`.  The remaining callbacks can be left `NULL` or
implemented as needed.

### No-OS to Zephyr Mapping

| no-OS concept                 | Zephyr equivalent                                 |
|-------------------------------|---------------------------------------------------|
| `<devname>_init()`            | `<devname>_init()` called via `DEVICE_DT_INST_DEFINE` |
| `<devname>_set_current()`     | `led_api->set_brightness()`                       |
| `<devname>_set_channel_enable()` | `led_api->on()` / `led_api->off()`             |
| `<devname>_reset()`           | Called internally during `init()`                 |
| `<devname>_init_param`        | Devicetree properties + `struct <devname>_config` |
| `<devname>_dev`               | `struct <devname>_data` (mutable runtime state)   |
| `no_os_i2c_desc`              | `struct i2c_dt_spec` from DT                      |
| `no_os_gpio_desc`             | `struct gpio_dt_spec` from DT                     |
| IIO subsystem                 | Not applicable (Zephyr uses its own LED API)      |

### Brightness Mapping

The Zephyr LED API expresses brightness as a percentage (0--100).  The
driver must scale this to the hardware's native resolution.  For example,
an 8-bit current DAC (0--255) maps brightness 100% to 255 and 50% to 127:

```c
hw_value = (brightness * <DEVNAME>_MAX_CURRENT_LEVEL) / 100;
```

---

## 2. File Checklist

```
zephyr/drivers/led/
    led_<devname>.c              # Driver source (single file)
    Kconfig.<devname>            # Kconfig fragment (sourced by parent Kconfig)

zephyr/drivers/led/CMakeLists.txt   # Add zephyr_library_sources_ifdef() line
zephyr/drivers/led/Kconfig          # Add source "Kconfig.<devname>" line

zephyr/dts/bindings/led/
    adi,<devname>.yaml           # Devicetree binding

zephyr/tests/drivers/led/<devname>/
    testcase.yaml
    prj.conf
    boards/native_sim.overlay    # (optional, for emulated testing)
    src/main.c                   # Test source
```

---

## 3. Devicetree Binding (`adi,<devname>.yaml`)

Path: `zephyr/dts/bindings/led/adi,<devname>.yaml`

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Analog Devices <DEVNAME> multi-channel LED driver with I2C interface
  and programmable current sinks.

  Example devicetree node:

    &i2c0 {
        <devname>@1b {
            compatible = "adi,<devname>";
            reg = <0x1b>;
            en-gpios = <&gpio0 14 GPIO_ACTIVE_HIGH>;
            num-leds = <4>;
        };
    };

compatible: "adi,<devname>"

include: [i2c-device.yaml]

properties:
  en-gpios:
    type: phandle-array
    description: |
      GPIO connected to the enable pin (e.g., ENRGBS on LTC3208).
      If present the driver will assert this pin high during init
      to enable the device.

  num-leds:
    type: int
    default: 4
    description: |
      Number of LED channels supported by this device instance.
      Defaults to 4.  Adjust per variant (e.g., 8, 12, 16).
    enum:
      - 4
      - 8
      - 12
      - 16
```

### Example devicetree node

```dts
&i2c0 {
    led_driver: <devname>@1b {
        compatible = "adi,<devname>";
        reg = <0x1b>;
        en-gpios = <&gpio0 14 GPIO_ACTIVE_HIGH>;
        num-leds = <8>;
        status = "okay";
    };
};
```

---

## 4. Kconfig (`Kconfig.<devname>`)

Path: `zephyr/drivers/led/Kconfig.<devname>`

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config LED_<DEVNAME>
	bool "<DEVNAME> LED driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	select I2C
	help
	  Enable the driver for the Analog Devices <DEVNAME>
	  multi-channel I2C LED driver with programmable current sinks.
```

Then add to the parent `zephyr/drivers/led/Kconfig`:

```kconfig
source "drivers/led/Kconfig.<devname>"
```

---

## 5. CMakeLists.txt

Add the following line to `zephyr/drivers/led/CMakeLists.txt`:

```cmake
zephyr_library_sources_ifdef(CONFIG_LED_<DEVNAME> led_<devname>.c)
```

---

## 6. Driver Header (optional, usually not needed)

Most Zephyr LED drivers keep everything in a single `.c` file since the
config and data structs are `static`.  A separate header is only needed
when the driver exposes chip-specific types to other modules (e.g., a
shared multi-function driver).

If needed, place it at `zephyr/drivers/led/led_<devname>.h`:

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_LED_<DEVNAME>_H_
#define ZEPHYR_DRIVERS_LED_<DEVNAME>_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

/* ---- Register Addresses ------------------------------------------- */

#define <DEVNAME>_REG_LED_CTRL       0x01
#define <DEVNAME>_REG_BRIGHTNESS     0x02
#define <DEVNAME>_REG_CONFIG         0x03

/* ---- Field Masks -------------------------------------------------- */

#define <DEVNAME>_CURRENT_MASK       GENMASK(7, 0)
#define <DEVNAME>_BRIGHT_HIGH_MASK   GENMASK(7, 4)
#define <DEVNAME>_BRIGHT_LOW_MASK    GENMASK(3, 0)

/* ---- Bit Defines -------------------------------------------------- */

#define <DEVNAME>_ENABLE_BIT         BIT(0)
#define <DEVNAME>_SHUTDOWN_BIT       BIT(7)

/* ---- Constants ---------------------------------------------------- */

#define <DEVNAME>_NUM_CHANNELS       4
#define <DEVNAME>_MAX_CURRENT_LEVEL  255

/* ---- Per-instance config (from devicetree, const, in ROM) --------- */

struct <devname>_config {
	/** I2C bus specification from devicetree. */
	struct i2c_dt_spec i2c;
	/** Optional enable GPIO. */
	struct gpio_dt_spec en_gpio;
	/** Number of LED channels. */
	uint8_t num_leds;
};

/* ---- Per-instance mutable data (in RAM) --------------------------- */

struct <devname>_data {
	/** Per-channel brightness cache (hardware may be write-only). */
	uint8_t brightness[<DEVNAME>_NUM_CHANNELS];
	/** Per-channel on/off state. */
	bool ch_enabled[<DEVNAME>_NUM_CHANNELS];
};

#endif /* ZEPHYR_DRIVERS_LED_<DEVNAME>_H_ */
```

---

## 7. Driver Source (`led_<devname>.c`)

Path: `zephyr/drivers/led/led_<devname>.c`

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>
/* NOTE: Replace hyphens in the compatible string with underscores.
 * E.g., compatible = "adi,ltc3208" => DT_DRV_COMPAT = adi_ltc3208
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(<devname>, CONFIG_LED_LOG_LEVEL);

/* ---- Register Addresses ------------------------------------------- */

#define <DEVNAME>_REG_LED_CTRL       0x01
#define <DEVNAME>_REG_BRIGHTNESS     0x02
#define <DEVNAME>_REG_CONFIG         0x03

/* ---- Field Masks -------------------------------------------------- */

#define <DEVNAME>_CURRENT_MASK       GENMASK(7, 0)
#define <DEVNAME>_BRIGHT_HIGH_MASK   GENMASK(7, 4)
#define <DEVNAME>_BRIGHT_LOW_MASK    GENMASK(3, 0)

/* ---- Bit Defines -------------------------------------------------- */

#define <DEVNAME>_ENABLE_BIT         BIT(0)
#define <DEVNAME>_SHUTDOWN_BIT       BIT(7)

/* ---- Constants ---------------------------------------------------- */

/** Maximum number of LED channels supported by the driver. */
#define <DEVNAME>_MAX_CHANNELS       16

/** Maximum current DAC level (e.g., 255 for 8-bit, 15 for 4-bit). */
#define <DEVNAME>_MAX_CURRENT_LEVEL  255

/* ---- Per-instance config (from devicetree, const, in ROM) --------- */

struct <devname>_config {
	/** I2C bus and address from devicetree. */
	struct i2c_dt_spec i2c;
	/** Optional enable GPIO (e.g., ENRGBS). */
	struct gpio_dt_spec en_gpio;
	/** Number of LED channels from devicetree. */
	uint8_t num_leds;
};

/* ---- Per-instance mutable data (in RAM) --------------------------- */

struct <devname>_data {
	/** Per-channel cached brightness (0--<DEVNAME>_MAX_CURRENT_LEVEL).
	 *  Many LED driver ICs are write-only, so we cache the state.
	 */
	uint8_t brightness[<DEVNAME>_MAX_CHANNELS];
	/** Per-channel on/off state. */
	bool ch_enabled[<DEVNAME>_MAX_CHANNELS];
};

/* ---- Low-level I2C register access -------------------------------- */

/**
 * @brief Write an 8-bit register via I2C.
 *
 * @param dev      Zephyr device pointer.
 * @param reg_addr Register address.
 * @param value    Data byte to write.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_reg_write(const struct device *dev, uint8_t reg_addr,
			       uint8_t value)
{
	const struct <devname>_config *cfg = dev->config;

	return i2c_reg_write_byte_dt(&cfg->i2c, reg_addr, value);
}

/**
 * @brief Read an 8-bit register via I2C (if supported by hardware).
 *
 * Many LED drivers are write-only.  Remove this function if the part
 * does not support register read-back.
 *
 * @param dev      Zephyr device pointer.
 * @param reg_addr Register address.
 * @param value    Pointer to store the read value.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_reg_read(const struct device *dev, uint8_t reg_addr,
			      uint8_t *value)
{
	const struct <devname>_config *cfg = dev->config;

	return i2c_reg_read_byte_dt(&cfg->i2c, reg_addr, value);
}

/**
 * @brief Write raw current-sink level to a specific LED channel.
 *
 * Converts the channel number to the appropriate register address
 * and writes the current level.  Adjust the register mapping per
 * your part's datasheet.
 *
 * @param dev     Zephyr device pointer.
 * @param led     LED channel number (0-based).
 * @param level   Raw current level (0 to <DEVNAME>_MAX_CURRENT_LEVEL).
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_set_current(const struct device *dev, uint32_t led,
				 uint8_t level)
{
	uint8_t reg;
	uint8_t data;

	/*
	 * Map channel number to register address.
	 * Adjust per your part's register layout.
	 * Example: channel 0 => reg 0x01, channel 1 => reg 0x02, etc.
	 */
	reg = <DEVNAME>_REG_LED_CTRL + (uint8_t)led;
	data = FIELD_PREP(<DEVNAME>_CURRENT_MASK, level);

	return <devname>_reg_write(dev, reg, data);
}

/* ---- LED API: led_on ---------------------------------------------- */

/**
 * @brief Turn on an LED at its previously cached brightness.
 *
 * If no brightness was previously set, turns on at full brightness.
 *
 * @param dev Zephyr device pointer.
 * @param led LED channel number (0-based).
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_led_on(const struct device *dev, uint32_t led)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint8_t level;
	int ret;

	if (led >= cfg->num_leds) {
		LOG_ERR("Invalid LED channel %u (max %u)", led,
			cfg->num_leds - 1);
		return -EINVAL;
	}

	/* Restore cached brightness, or use maximum if never set. */
	level = data->brightness[led];
	if (level == 0) {
		level = <DEVNAME>_MAX_CURRENT_LEVEL;
	}

	ret = <devname>_set_current(dev, led, level);
	if (ret) {
		LOG_ERR("Failed to turn on LED %u: %d", led, ret);
		return ret;
	}

	data->ch_enabled[led] = true;

	return 0;
}

/* ---- LED API: led_off --------------------------------------------- */

/**
 * @brief Turn off an LED by setting its current to zero.
 *
 * The previously configured brightness is retained in the cache
 * so that led_on() can restore it.
 *
 * @param dev Zephyr device pointer.
 * @param led LED channel number (0-based).
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_led_off(const struct device *dev, uint32_t led)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	if (led >= cfg->num_leds) {
		LOG_ERR("Invalid LED channel %u (max %u)", led,
			cfg->num_leds - 1);
		return -EINVAL;
	}

	ret = <devname>_set_current(dev, led, 0);
	if (ret) {
		LOG_ERR("Failed to turn off LED %u: %d", led, ret);
		return ret;
	}

	data->ch_enabled[led] = false;

	return 0;
}

/* ---- LED API: led_set_brightness ---------------------------------- */

/**
 * @brief Set LED brightness as a percentage (0--100).
 *
 * Scales the percentage to the hardware's native current DAC
 * resolution and writes the value.  The raw level is cached so
 * that led_on() can restore it after led_off().
 *
 * @param dev        Zephyr device pointer.
 * @param led        LED channel number (0-based).
 * @param brightness Brightness percentage (0 = off, 100 = full).
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_led_set_brightness(const struct device *dev,
					uint32_t led, uint8_t brightness)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint8_t level;
	int ret;

	if (led >= cfg->num_leds) {
		LOG_ERR("Invalid LED channel %u (max %u)", led,
			cfg->num_leds - 1);
		return -EINVAL;
	}

	if (brightness > 100) {
		LOG_ERR("Brightness %u exceeds 100%%", brightness);
		return -EINVAL;
	}

	/* Scale percentage to hardware range. */
	level = (uint8_t)(((uint32_t)brightness * <DEVNAME>_MAX_CURRENT_LEVEL)
			  / 100U);

	ret = <devname>_set_current(dev, led, level);
	if (ret) {
		LOG_ERR("Failed to set brightness for LED %u: %d", led, ret);
		return ret;
	}

	data->brightness[led] = level;
	data->ch_enabled[led] = (brightness > 0);

	return 0;
}

/* ---- LED API: led_blink (optional) -------------------------------- */

/**
 * @brief Configure hardware blinking for an LED channel.
 *
 * If the part has hardware blink support (dedicated blink registers),
 * implement this callback.  Otherwise, remove it or return -ENOTSUP
 * to let the application handle software blinking.
 *
 * @param dev       Zephyr device pointer.
 * @param led       LED channel number (0-based).
 * @param delay_on  On-time in milliseconds.
 * @param delay_off Off-time in milliseconds.
 * @return 0 on success, -ENOTSUP if not supported, negative errno
 *         on failure.
 */
static int <devname>_led_blink(const struct device *dev, uint32_t led,
			       uint32_t delay_on, uint32_t delay_off)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(led);
	ARG_UNUSED(delay_on);
	ARG_UNUSED(delay_off);

	/*
	 * Most simple LED current-sink drivers do not have hardware
	 * blink support.  Return -ENOTSUP so the application knows
	 * to use software-based blinking (e.g., a timer or thread).
	 *
	 * If the part supports hardware blink registers, program
	 * them here and return 0 on success.
	 */
	return -ENOTSUP;
}

/* ---- LED API: led_write_channels (optional) ----------------------- */

/**
 * @brief Bulk-write brightness values to a contiguous range of channels.
 *
 * This is useful for updating many channels atomically.  For parts
 * that support auto-increment I2C writes, this can be implemented as
 * a single multi-byte I2C transaction for efficiency.
 *
 * @param dev        Zephyr device pointer.
 * @param start_channel First channel number in the range.
 * @param num_channels  Number of channels to write.
 * @param buf        Array of brightness values (one byte per channel,
 *                   raw hardware units, NOT percentages).
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_led_write_channels(const struct device *dev,
					uint32_t start_channel,
					uint32_t num_channels,
					const uint8_t *buf)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	if (start_channel + num_channels > cfg->num_leds) {
		LOG_ERR("Channel range %u..%u exceeds num_leds %u",
			start_channel, start_channel + num_channels - 1,
			cfg->num_leds);
		return -EINVAL;
	}

	/*
	 * Option A: Individual register writes (always works).
	 * Option B: For parts with auto-increment support, use a
	 *           single i2c_burst_write_dt() for efficiency.
	 */
	for (uint32_t i = 0; i < num_channels; i++) {
		ret = <devname>_set_current(dev, start_channel + i, buf[i]);
		if (ret) {
			LOG_ERR("Failed to write channel %u: %d",
				start_channel + i, ret);
			return ret;
		}
		data->brightness[start_channel + i] = buf[i];
		data->ch_enabled[start_channel + i] = (buf[i] > 0);
	}

	/*
	 * For auto-increment I2C, replace the loop above with:
	 *
	 * uint8_t reg = <DEVNAME>_REG_LED_CTRL + (uint8_t)start_channel;
	 *
	 * ret = i2c_burst_write_dt(&cfg->i2c, reg, buf, num_channels);
	 * if (ret) {
	 *     LOG_ERR("Bulk write failed: %d", ret);
	 *     return ret;
	 * }
	 *
	 * for (uint32_t i = 0; i < num_channels; i++) {
	 *     data->brightness[start_channel + i] = buf[i];
	 *     data->ch_enabled[start_channel + i] = (buf[i] > 0);
	 * }
	 */

	return 0;
}

/* ---- LED API table ------------------------------------------------ */

static DEVICE_API(led, <devname>_led_api) = {
	.on              = <devname>_led_on,
	.off             = <devname>_led_off,
	.set_brightness  = <devname>_led_set_brightness,
	.blink           = <devname>_led_blink,
	.write_channels  = <devname>_led_write_channels,
	/*
	 * .set_color       = NULL,
	 * .get_info        = NULL,
	 *
	 * Implement set_color for RGB/multi-color LED drivers.
	 * Implement get_info if the driver needs to expose LED
	 * metadata (label, color mapping, index).
	 */
};

/* ---- Hardware reset ----------------------------------------------- */

/**
 * @brief Reset all LED channels to their default (off) state.
 *
 * Writes zero current to all channels and clears the cached state.
 *
 * @param dev Zephyr device pointer.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_reset(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	for (uint8_t i = 0; i < cfg->num_leds; i++) {
		ret = <devname>_set_current(dev, i, 0);
		if (ret) {
			LOG_ERR("Failed to reset channel %u: %d", i, ret);
			return ret;
		}
		data->brightness[i] = 0;
		data->ch_enabled[i] = false;
	}

	return 0;
}

/* ---- Initialization ----------------------------------------------- */

/**
 * @brief Initialise a <DEVNAME> instance.
 *
 * Verifies I2C bus readiness, configures the enable GPIO (if present),
 * and resets all LED channels to the off state.
 *
 * @param dev Zephyr device pointer.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_init(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	int ret;

	/* Verify the I2C bus is ready. */
	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Configure the enable GPIO if present. */
	if (cfg->en_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->en_gpio)) {
			LOG_ERR("Enable GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->en_gpio,
					    GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure enable GPIO: %d", ret);
			return ret;
		}

		/* Allow the device to power up after enable assertion. */
		k_usleep(100);
	}

	/* Reset all channels to off. */
	ret = <devname>_reset(dev);
	if (ret) {
		LOG_ERR("Failed to reset device: %d", ret);
		return ret;
	}

	LOG_INF("<DEVNAME> initialised on %s (addr 0x%02x, %u LEDs)",
		cfg->i2c.bus->name, cfg->i2c.addr, cfg->num_leds);

	return 0;
}

/* ---- Device instantiation macros ---------------------------------- */

/*
 * The DT_INST_FOREACH_STATUS_OKAY block creates one config + data
 * pair for every enabled devicetree node with a matching compatible
 * string.  No dynamic memory allocation is used.
 */

#define <DEVNAME>_INIT(inst)                                                   \
	static struct <devname>_data <devname>_data_##inst;                    \
                                                                               \
	static const struct <devname>_config <devname>_config_##inst = {       \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                            \
		.en_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}),     \
		.num_leds = DT_INST_PROP(inst, num_leds),                     \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst,                                            \
			      <devname>_init,                                  \
			      NULL,   /* pm_action_cb (NULL if no PM) */       \
			      &<devname>_data_##inst,                          \
			      &<devname>_config_##inst,                        \
			      POST_KERNEL,                                     \
			      CONFIG_LED_INIT_PRIORITY,                        \
			      &<devname>_led_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_INIT)
```

### Key points

- **`DEVICE_API(led, ...)`** creates a `struct led_driver_api` with
  the compile-time type tag required by the device model.
- **`DEVICE_DT_INST_DEFINE()`** registers the device with the LED
  subsystem.  Unlike sensors, there is no `LED_DEVICE_DT_INST_DEFINE`
  wrapper, so use the generic `DEVICE_DT_INST_DEFINE()` directly.
- **`DT_INST_FOREACH_STATUS_OKAY()`** expands the instantiation macro
  once for every `status = "okay"` node with matching compatible.
- **`LOG_MODULE_REGISTER()`** registers the driver's log module; use
  `CONFIG_LED_LOG_LEVEL` as the default level.
- **`i2c_reg_write_byte_dt()`** is the simplest I2C write helper.
  For multi-byte auto-increment writes, use `i2c_burst_write_dt()`.
- **Brightness caching** is essential because many LED driver ICs
  are write-only (no register read-back).
- **`GPIO_DT_SPEC_INST_GET_OR()`** provides a safe fallback when the
  enable GPIO is not specified in the devicetree.

---

## 8. Power Management (optional)

If the LED driver supports a hardware shutdown or low-power mode,
implement power management via the `pm_action_cb`:

```c
#include <zephyr/pm/device.h>

/**
 * @brief PM action callback for suspend/resume.
 *
 * @param dev    Zephyr device pointer.
 * @param action PM action (SUSPEND or RESUME).
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_pm_action(const struct device *dev,
			       enum pm_device_action action)
{
	const struct <devname>_config *cfg = dev->config;
	int ret;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		/*
		 * Enter low-power mode.  Options:
		 * - Set the shutdown register bit.
		 * - De-assert the enable GPIO.
		 * - Write zero to all channels.
		 */
		if (cfg->en_gpio.port != NULL) {
			ret = gpio_pin_set_dt(&cfg->en_gpio, 0);
			if (ret) {
				return ret;
			}
		}
		LOG_DBG("<DEVNAME> suspended");
		break;

	case PM_DEVICE_ACTION_RESUME:
		/* Wake from low-power mode. */
		if (cfg->en_gpio.port != NULL) {
			ret = gpio_pin_set_dt(&cfg->en_gpio, 1);
			if (ret) {
				return ret;
			}
			k_usleep(100);
		}
		LOG_DBG("<DEVNAME> resumed");
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}
```

Then update the `DEVICE_DT_INST_DEFINE()` call in the instantiation
macro:

```c
	PM_DEVICE_DT_INST_DEFINE(inst, <devname>_pm_action);        \
                                                                     \
	DEVICE_DT_INST_DEFINE(inst,                                  \
			      <devname>_init,                        \
			      PM_DEVICE_DT_INST_GET(inst),           \
			      &<devname>_data_##inst,                \
			      &<devname>_config_##inst,              \
			      POST_KERNEL,                           \
			      CONFIG_LED_INIT_PRIORITY,              \
			      &<devname>_led_api);
```

---

## 9. Test Skeleton

Path: `zephyr/tests/drivers/led/<devname>/`

### 9.1 `testcase.yaml`

```yaml
tests:
  drivers.led.<devname>:
    tags:
      - drivers
      - led
    depends_on: i2c
    harness: ztest
    platform_allow:
      - native_sim
```

### 9.2 `prj.conf`

```ini
CONFIG_ZTEST=y
CONFIG_I2C=y
CONFIG_LED=y
CONFIG_LED_<DEVNAME>=y
CONFIG_LOG=y
```

### 9.3 `boards/native_sim.overlay`

```dts
/ {
	test_i2c: i2c@0 {
		compatible = "vnd,i2c";
		reg = <0x0 0x1000>;
		#address-cells = <1>;
		#size-cells = <0>;
		status = "okay";
		clock-frequency = <400000>;

		test_led: <devname>@1b {
			compatible = "adi,<devname>";
			reg = <0x1b>;
			num-leds = <4>;
			status = "okay";
		};
	};
};
```

### 9.4 `src/main.c`

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/ztest.h>

#define LED_NODE DT_NODELABEL(test_led)

static const struct device *get_led_device(void)
{
	const struct device *dev = DEVICE_DT_GET(LED_NODE);

	zassert_true(device_is_ready(dev), "LED device not ready");
	return dev;
}

ZTEST(led_<devname>, test_led_on_off)
{
	const struct device *dev = get_led_device();
	int ret;

	/* Turn on LED 0 */
	ret = led_on(dev, 0);
	zassert_ok(ret, "led_on failed: %d", ret);

	/* Turn off LED 0 */
	ret = led_off(dev, 0);
	zassert_ok(ret, "led_off failed: %d", ret);
}

ZTEST(led_<devname>, test_led_set_brightness)
{
	const struct device *dev = get_led_device();
	int ret;

	/* Set to 50% brightness */
	ret = led_set_brightness(dev, 0, 50);
	zassert_ok(ret, "set_brightness(50) failed: %d", ret);

	/* Set to full brightness */
	ret = led_set_brightness(dev, 0, 100);
	zassert_ok(ret, "set_brightness(100) failed: %d", ret);

	/* Turn off via brightness */
	ret = led_set_brightness(dev, 0, 0);
	zassert_ok(ret, "set_brightness(0) failed: %d", ret);
}

ZTEST(led_<devname>, test_led_invalid_channel)
{
	const struct device *dev = get_led_device();
	int ret;

	/* Channel 99 is out of range. */
	ret = led_on(dev, 99);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for invalid channel, got %d", ret);

	ret = led_off(dev, 99);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for invalid channel, got %d", ret);

	ret = led_set_brightness(dev, 99, 50);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for invalid channel, got %d", ret);
}

ZTEST(led_<devname>, test_led_invalid_brightness)
{
	const struct device *dev = get_led_device();
	int ret;

	/* Brightness > 100 should fail. */
	ret = led_set_brightness(dev, 0, 150);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for brightness > 100, got %d", ret);
}

ZTEST(led_<devname>, test_led_blink_unsupported)
{
	const struct device *dev = get_led_device();
	int ret;

	/*
	 * If hardware blink is not implemented, the driver should
	 * return -ENOTSUP.  Adjust this test if your part supports it.
	 */
	ret = led_blink(dev, 0, 500, 500);
	zassert_equal(ret, -ENOTSUP,
		      "Expected -ENOTSUP for blink, got %d", ret);
}

ZTEST(led_<devname>, test_led_on_restores_brightness)
{
	const struct device *dev = get_led_device();
	int ret;

	/* Set brightness, turn off, then turn back on. */
	ret = led_set_brightness(dev, 0, 75);
	zassert_ok(ret, "set_brightness(75) failed: %d", ret);

	ret = led_off(dev, 0);
	zassert_ok(ret, "led_off failed: %d", ret);

	/* led_on should restore the cached brightness of 75%. */
	ret = led_on(dev, 0);
	zassert_ok(ret, "led_on (restore) failed: %d", ret);
}

ZTEST_SUITE(led_<devname>, NULL, NULL, NULL, NULL, NULL);
```

---

## 10. Key Conventions

1. **`DEVICE_API(led, ...)`** -- Always use the `DEVICE_API` macro to
   declare the API struct.  This enables compile-time type checking
   against `struct led_driver_api`.

2. **`i2c_dt_spec`** -- Never manually fill I2C bus parameters.  Use
   `I2C_DT_SPEC_INST_GET()` to extract the bus and address from the
   devicetree.

3. **`gpio_dt_spec`** -- Use `GPIO_DT_SPEC_INST_GET_OR()` for optional
   GPIOs (enable, reset, shutdown).  The `_OR` variant provides a
   fallback so the driver compiles even when the property is absent
   in the DTS.

4. **`LOG_MODULE_REGISTER()`** -- Register the log module at the top
   of the `.c` file using the parent subsystem's log level
   (`CONFIG_LED_LOG_LEVEL`).

5. **No dynamic allocation** -- All config and data structs are
   statically instantiated via `DT_INST_FOREACH_STATUS_OKAY()`.
   Never call `k_malloc()` in an LED driver.

6. **Bus readiness checks** -- Always verify `i2c_is_ready_dt()`
   and `gpio_is_ready_dt()` in `init()` before performing any
   transfers.

7. **Init priority** -- Use `CONFIG_LED_INIT_PRIORITY` (default 90)
   so the LED driver initialises after the I2C controller (priority 50).

8. **`DT_DRV_COMPAT`** -- Must match the compatible string in the
   binding with dots and hyphens replaced by underscores
   (`"adi,ltc3208"` becomes `adi_ltc3208`).

9. **Brightness as percentage** -- The Zephyr LED API uses 0--100
   (uint8_t) for brightness.  The driver must scale to the hardware's
   native range internally.

10. **Write-only caching** -- Many LED driver ICs do not support
    register read-back.  Cache the current level and on/off state
    in the data struct so that `led_on()` can restore the previous
    brightness after `led_off()`.

11. **Error codes** -- Return standard negative `errno` values
    (`-EINVAL`, `-ENOTSUP`, `-ENODEV`, `-EIO`).  Never return
    positive error codes.

12. **Coding style** -- Follow the Zephyr coding style:
    - Tabs for indentation (not spaces).
    - Opening brace on the same line for functions.
    - K&R braces for `if`/`for`/`while`.
    - Line length limit: 100 columns.
    - Zephyr uses `clang-format` with the project `.clang-format`.

13. **SPDX headers** -- Every file must have a single-line
    `/* SPDX-License-Identifier: Apache-2.0 */` near the top.
    Zephyr uses Apache-2.0 (not the BSD-3-Clause used by no-OS).

14. **PM (power management)** -- Pass `NULL` as the `pm_action_cb`
    argument to `DEVICE_DT_INST_DEFINE()` unless the part supports
    hardware power states.  If it does, implement a
    `<devname>_pm_action()` callback handling
    `PM_DEVICE_ACTION_SUSPEND` and `PM_DEVICE_ACTION_RESUME`.

15. **Single-file drivers** -- Unlike the sensor subsystem, LED
    drivers typically do not need a separate header file or trigger
    file.  Keep everything in `led_<devname>.c` unless the driver
    is shared across multiple compilation units.

---

## 11. Commit Message Format

Zephyr uses a prefix-based commit message convention.  The subsystem
prefix for LED drivers is `drivers: led:`.

### Adding a new driver

```
drivers: led: add support for <DEVNAME>

Add a Zephyr LED subsystem driver for the Analog Devices <DEVNAME>,
a multi-channel I2C LED driver with programmable current sinks.

The driver implements led_on, led_off, and led_set_brightness
callbacks.  Brightness is scaled from the Zephyr 0-100% API
range to the hardware's native <N>-bit current DAC resolution.

Signed-off-by: Your Name <your.name@analog.com>
```

### Adding the devicetree binding

```
dts: bindings: add binding for adi,<devname>

Add a devicetree binding for the Analog Devices <DEVNAME>
I2C LED driver. The binding defines optional en-gpios and
num-leds properties.

Signed-off-by: Your Name <your.name@analog.com>
```

### Adding tests

```
tests: drivers: led: add tests for <DEVNAME>

Add a ztest-based test suite for the <DEVNAME> LED driver
covering on/off, brightness control, invalid channel handling,
and brightness restoration after off/on cycles.

Signed-off-by: Your Name <your.name@analog.com>
```

**Note:** In a Zephyr contribution, the binding, driver, and tests are
typically submitted as separate commits in a single PR, or as a single
commit if the changeset is small.  Check the Zephyr contribution
guidelines for current preferences.
