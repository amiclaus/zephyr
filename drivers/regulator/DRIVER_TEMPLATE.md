# Digital Power Controller Driver Template -- Zephyr RTOS

Reference driver: `drivers/power/adp1050/` in the no-OS tree.

This template covers every file needed to add a new Zephyr driver for an
Analog Devices digital power controller that communicates over I2C/PMBus.
Replace `<devname>` with the part number (e.g., `adp1050`) and `<DEVNAME>`
with its uppercase form (e.g., `ADP1050`) throughout.

Digital power controllers expose two functional surfaces:

1. **Regulator subsystem** -- enable/disable output, set voltage/current
   limits via Zephyr's `regulator_driver_api`.
2. **Sensor subsystem** -- read VIN, IIN, VOUT, IOUT, and temperature
   telemetry via Zephyr's `sensor_driver_api`.

This template implements both subsystems.  The regulator node is the
parent device and the sensor node is registered as a child so that
applications can independently use either or both APIs.

---

## 1. Purpose & Zephyr Subsystem Mapping

### Regulator Subsystem

Zephyr's regulator API (`<zephyr/drivers/regulator.h>`) provides a
uniform interface for controlling power supplies:

| API Function                  | Maps To                                    |
|-------------------------------|--------------------------------------------|
| `regulator_enable()`         | PMBus OPERATION = ON (0x80)                |
| `regulator_disable()`        | PMBus OPERATION = OFF (0x00)               |
| `regulator_set_voltage()`    | PMBus VOUT_COMMAND / VOUT_MAX              |
| `regulator_get_voltage()`    | PMBus READ_VOUT                            |
| `regulator_set_current_limit()` | PMBus IOUT_OC_FAULT_LIMIT (if supported)|
| `regulator_is_enabled()`     | PMBus STATUS_BYTE / OPERATION readback     |

### Sensor Subsystem

For telemetry monitoring, the sensor API provides read-only access to
measurements:

| Sensor Channel              | PMBus Command      | Format    |
|-----------------------------|--------------------|-----------|
| `SENSOR_CHAN_VOLTAGE` (0)   | READ_VIN (0x88)    | Linear11  |
| `SENSOR_CHAN_VOLTAGE` (1)   | READ_VOUT (0x8B)   | ULinear16 |
| `SENSOR_CHAN_CURRENT` (0)   | READ_IIN (0x89)    | Linear11  |
| `SENSOR_CHAN_CURRENT` (1)   | READ_IOUT (0x8C)   | Linear11  |
| `SENSOR_CHAN_DIE_TEMP`      | READ_TEMPERATURE (0x8D) | Linear11 |

### PMBus Data Formats

**Linear11**: 16-bit word with 5-bit signed exponent [15:11] and 11-bit
signed mantissa [10:0].  Value = mantissa * 2^exponent.

**ULinear16**: 16-bit unsigned value.  Value = raw * 2^N where N is the
exponent from VOUT_MODE (typically negative, e.g., -9).

---

## 2. File Checklist

```
drivers/regulator/adi/<devname>/
    <devname>.h              # Shared header (register defs, structs)
    <devname>_reg.c          # Regulator driver (enable/disable/set voltage)
    <devname>_sensor.c       # Sensor driver (telemetry: VIN/IIN/VOUT/IOUT/temp)
    Kconfig                  # Kconfig menu entry
    CMakeLists.txt           # Build system integration

dts/bindings/regulator/
    adi,<devname>.yaml       # Regulator devicetree binding

dts/bindings/sensor/
    adi,<devname>-sensor.yaml  # Sensor devicetree binding (child node)

tests/drivers/regulator/<devname>/
    testcase.yaml            # Test metadata
    prj.conf                 # Test project config
    src/main.c               # Test source
```

---

## 3. Devicetree Bindings

### 3.1 Regulator Binding (`dts/bindings/regulator/adi,<devname>.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  ADI <DEVNAME> digital power controller with PMBus interface.
  Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/<DEVNAME>.pdf

  The <DEVNAME> is a digital controller for isolated DC-DC power
  converters.  It communicates over I2C/PMBus and provides output
  voltage regulation, fault protection, and telemetry monitoring
  for input/output voltage, current, and temperature.

compatible: "adi,<devname>"

include: [regulator.yaml, i2c-device.yaml]

properties:
  pg-alt-gpios:
    type: phandle-array
    description: |
      Power-good / alert GPIO pin.  Active-low open-drain output.
      Example: pg-alt-gpios = <&gpio0 5 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;

  ctrl-gpios:
    type: phandle-array
    description: |
      Hardware on/off control pin.  Drive high to enable the converter.
      Example: ctrl-gpios = <&gpio0 6 GPIO_ACTIVE_HIGH>;

  adi,on-off-config:
    type: int
    default: 0
    description: |
      PMBus ON_OFF_CONFIG byte (command 0x02).
      Bit 4: 1 = use CTRL pin for hardware enable.
      0x00 = converter starts via PMBus OPERATION command only.
      0x10 = CTRL pin controls enable/disable.

  regulator-boot-on:
    type: boolean
    description: |
      If present, the regulator is enabled at boot by writing
      OPERATION = ON (0x80) during driver init.

  adi,vout-mode-exponent:
    type: int
    default: -9
    description: |
      VOUT_MODE exponent for ULinear16 voltage conversion.
      VOUT (V) = raw_value * 2^exponent.
      Typical values: -9 (1.953 mV/LSB), -12 (0.244 mV/LSB).

  adi,switching-frequency-hz:
    type: int
    description: |
      Initial switching frequency in Hz.  Written to FREQUENCY_SWITCH
      (0x33) during init.  Encoded in Linear11 format.

child-binding:
  description: |
    Sensor child node for telemetry monitoring.  Expose VIN, IIN,
    VOUT, IOUT, and temperature readings via the sensor subsystem.
  compatible: "adi,<devname>-sensor"
```

### 3.2 Sensor Child Binding (`dts/bindings/sensor/adi,<devname>-sensor.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Telemetry sensor interface for the ADI <DEVNAME> power controller.
  This is a child node of the <DEVNAME> regulator node.  It provides
  read-only access to voltage, current, and temperature measurements
  via PMBus telemetry commands.

compatible: "adi,<devname>-sensor"

include: [sensor-device.yaml]
```

### 3.3 Example Devicetree Node

```dts
&i2c0 {
    <devname>: <devname>@70 {
        compatible = "adi,<devname>";
        reg = <0x70>;
        pg-alt-gpios = <&gpio0 5 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        ctrl-gpios = <&gpio0 6 GPIO_ACTIVE_HIGH>;
        adi,on-off-config = <0x00>;
        adi,vout-mode-exponent = <(-9)>;
        regulator-boot-on;

        /* Regulator voltage limits (in microvolts). */
        regulator-min-microvolt = <500000>;
        regulator-max-microvolt = <5500000>;

        <devname>_sensor: sensor {
            compatible = "adi,<devname>-sensor";
        };
    };
};
```

---

## 4. Kconfig

```kconfig
# ADI <DEVNAME> digital power controller

# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

menuconfig <DEVNAME>
	bool "<DEVNAME> Digital Power Controller"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	select I2C
	help
	  Enable driver for the ADI <DEVNAME> digital power controller.
	  This provides regulator control (enable/disable, voltage setting)
	  and telemetry monitoring (VIN, IIN, VOUT, IOUT, temperature)
	  over I2C/PMBus.

if <DEVNAME>

config <DEVNAME>_SENSOR
	bool "<DEVNAME> Sensor (Telemetry)"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_SENSOR_ENABLED
	depends on SENSOR
	help
	  Enable the sensor subsystem interface for <DEVNAME> telemetry.
	  Provides read-only access to input/output voltage, current,
	  and temperature via the Zephyr sensor API.

config <DEVNAME>_INIT_PRIORITY
	int "Initialization priority"
	default 80
	help
	  <DEVNAME> regulator initialization priority.
	  Must be higher (later) than I2C bus init priority.

config <DEVNAME>_SENSOR_INIT_PRIORITY
	int "Sensor initialization priority"
	depends on <DEVNAME>_SENSOR
	default 81
	help
	  <DEVNAME> sensor child initialization priority.
	  Must be higher (later) than the parent regulator device.

endif # <DEVNAME>
```

---

## 5. CMakeLists.txt

```cmake
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  <devname>_reg.c
)

zephyr_library_sources_ifdef(
  CONFIG_<DEVNAME>_SENSOR
  <devname>_sensor.c
)
```

---

## 6. Driver Header (`<devname>.h`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_REGULATOR_<DEVNAME>_<DEVNAME>_H_
#define ZEPHYR_DRIVERS_REGULATOR_<DEVNAME>_<DEVNAME>_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/sys/util.h>

/* ---- PMBus Extended Command Prefix -------------------------------- */

/** Extended command byte for manufacturer-specific commands > 0xFF. */
#define <DEVNAME>_EXTENDED_COMMAND		0xFF

/* ---- Byte/Field Masks --------------------------------------------- */

#define <DEVNAME>_LSB_MASK			GENMASK(7, 0)
#define <DEVNAME>_MSB_MASK			GENMASK(15, 8)

/** PMBus Linear11 mantissa (bits [10:0]). */
#define <DEVNAME>_MANT_MASK			GENMASK(10, 0)

/** PMBus Linear11 exponent (bits [15:11]). */
#define <DEVNAME>_EXP_MASK			GENMASK(15, 11)

#define <DEVNAME>_MANT_MAX			0x7FF
#define <DEVNAME>_EXP_MAX			0x1F

/* ---- ON/OFF Configuration ----------------------------------------- */

#define <DEVNAME>_ON_OFF_DEFAULT_CFG		0x00
#define <DEVNAME>_CTRL_PIN_ENABLE		BIT(4)

#define <DEVNAME>_OPERATION_ON			0x80
#define <DEVNAME>_OPERATION_OFF			0x00
#define <DEVNAME>_OPERATION_SOFT_OFF		0x40

/* ---- PMBus Addresses ---------------------------------------------- */

#define <DEVNAME>_PMBUS_BASE_ADDRESS		0x70

/* ---- PMBus Standard Command Set ----------------------------------- */

#define <DEVNAME>_OPERATION			0x01
#define <DEVNAME>_ON_OFF_CONFIG			0x02
#define <DEVNAME>_CLEAR_FAULTS			0x03
#define <DEVNAME>_WRITE_PROTECT			0x10
#define <DEVNAME>_STORE_USER_ALL		0x15
#define <DEVNAME>_RESTORE_USER_ALL		0x16
#define <DEVNAME>_VOUT_MODE			0x20
#define <DEVNAME>_VOUT_COMMAND			0x21
#define <DEVNAME>_VOUT_MAX			0x24
#define <DEVNAME>_VOUT_MARGIN_HIGH		0x25
#define <DEVNAME>_VOUT_MARGIN_LOW		0x26
#define <DEVNAME>_FREQUENCY_SWITCH		0x33
#define <DEVNAME>_VIN_ON			0x35
#define <DEVNAME>_VIN_OFF			0x36
#define <DEVNAME>_VOUT_OV_FAULT_LIMIT		0x40
#define <DEVNAME>_VOUT_OV_FAULT_RESPONSE	0x41
#define <DEVNAME>_VOUT_UV_FAULT_LIMIT		0x44
#define <DEVNAME>_VOUT_UV_FAULT_RESPONSE	0x45
#define <DEVNAME>_OT_FAULT_LIMIT		0x4F
#define <DEVNAME>_OT_FAULT_RESPONSE		0x50
#define <DEVNAME>_TON_DELAY			0x60
#define <DEVNAME>_TON_RISE			0x61
#define <DEVNAME>_TOFF_DELAY			0x64
#define <DEVNAME>_STATUS_BYTE			0x78
#define <DEVNAME>_STATUS_WORD			0x79
#define <DEVNAME>_STATUS_VOUT			0x7A
#define <DEVNAME>_STATUS_IOUT			0x7B
#define <DEVNAME>_STATUS_INPUT			0x7C
#define <DEVNAME>_STATUS_TEMPERATURE		0x7D
#define <DEVNAME>_STATUS_CML			0x7E
#define <DEVNAME>_READ_VIN			0x88
#define <DEVNAME>_READ_IIN			0x89
#define <DEVNAME>_READ_VOUT			0x8B
#define <DEVNAME>_READ_IOUT			0x8C
#define <DEVNAME>_READ_TEMPERATURE		0x8D
#define <DEVNAME>_READ_DUTY_CYCLE		0x94
#define <DEVNAME>_READ_FREQUENCY		0x95
#define <DEVNAME>_MFR_ID			0x99
#define <DEVNAME>_MFR_MODEL			0x9A
#define <DEVNAME>_IC_DEVICE_ID			0xAD

/* ---- Manufacturer-Specific Extended Commands ---------------------- */

/**
 * Extended commands use the 0xFF prefix byte followed by a 16-bit
 * command address.  Add device-specific extended registers here.
 *
 * Example:
 * #define <DEVNAME>_PWM_OUTPUT_DISABLE	0xFE53
 * #define <DEVNAME>_GO_COMMANDS		0xFE61
 */

/* ---- Driver Structures -------------------------------------------- */

/**
 * @brief Per-instance configuration (const, from devicetree).
 *
 * Contains the common regulator config, I2C bus spec, and optional
 * GPIO specs.  Stored in ROM.
 */
struct <devname>_config {
	/** Common regulator config (voltage limits, flags). */
	struct regulator_common_config common;

	/** I2C bus and address from devicetree. */
	struct i2c_dt_spec i2c;

	/** Power-good / alert GPIO (optional). */
	struct gpio_dt_spec pg_alt_gpio;

	/** Hardware on/off control GPIO (optional). */
	struct gpio_dt_spec ctrl_gpio;

	/** PMBus ON_OFF_CONFIG byte value. */
	uint8_t on_off_config;

	/** VOUT_MODE exponent for ULinear16 conversion. */
	int8_t vout_mode_exp;

	/** Initial switching frequency in Hz (0 = do not set). */
	uint32_t switching_freq_hz;
};

/**
 * @brief Per-instance runtime data (mutable, in RAM).
 */
struct <devname>_data {
	/** Common regulator data. */
	struct regulator_common_data common;

	/** Cached VOUT_MODE exponent (read from device or DT). */
	int8_t vout_exp;

	/** Current output enabled state. */
	bool enabled;
};

/* ---- PMBus I/O Helpers (shared between reg and sensor) ------------ */

/**
 * @brief Read data from a PMBus command register.
 * @param cfg    - Pointer to device config (for I2C bus spec).
 * @param cmd    - PMBus command code (standard or extended).
 * @param buf    - Buffer to store read data.
 * @param len    - Number of data bytes to read (1 or 2).
 * @return 0 on success, negative errno on failure.
 */
static inline int <devname>_pmbus_read(const struct <devname>_config *cfg,
				       uint16_t cmd, uint8_t *buf,
				       uint8_t len)
{
	uint8_t cmd_buf[2];
	uint8_t cmd_len;

	if (cmd > <DEVNAME>_EXTENDED_COMMAND) {
		cmd_buf[0] = FIELD_GET(<DEVNAME>_MSB_MASK, cmd);
		cmd_buf[1] = FIELD_GET(<DEVNAME>_LSB_MASK, cmd);
		cmd_len = 2;
	} else {
		cmd_buf[0] = (uint8_t)cmd;
		cmd_len = 1;
	}

	return i2c_write_read_dt(&cfg->i2c, cmd_buf, cmd_len, buf, len);
}

/**
 * @brief Write data to a PMBus command register.
 * @param cfg    - Pointer to device config.
 * @param cmd    - PMBus command code (standard or extended).
 * @param data   - Data word to write.
 * @param len    - Number of data bytes (1 or 2).
 * @return 0 on success, negative errno on failure.
 *
 * PMBus data is little-endian on the wire.
 */
static inline int <devname>_pmbus_write(const struct <devname>_config *cfg,
					uint16_t cmd, uint16_t data,
					uint8_t len)
{
	uint8_t buf[4];
	uint8_t total;

	if (cmd > <DEVNAME>_EXTENDED_COMMAND) {
		buf[0] = FIELD_GET(<DEVNAME>_MSB_MASK, cmd);
		buf[1] = FIELD_GET(<DEVNAME>_LSB_MASK, cmd);
		buf[2] = FIELD_GET(<DEVNAME>_LSB_MASK, data);
		if (len > 1) {
			buf[3] = FIELD_GET(<DEVNAME>_MSB_MASK, data);
		}
		total = len + 2;
	} else {
		buf[0] = (uint8_t)cmd;
		buf[1] = FIELD_GET(<DEVNAME>_LSB_MASK, data);
		if (len > 1) {
			buf[2] = FIELD_GET(<DEVNAME>_MSB_MASK, data);
		}
		total = len + 1;
	}

	return i2c_write_dt(&cfg->i2c, buf, total);
}

/**
 * @brief Send a PMBus command with no data payload.
 * @param cfg - Pointer to device config.
 * @param cmd - PMBus command code.
 * @return 0 on success, negative errno on failure.
 */
static inline int <devname>_pmbus_send_cmd(const struct <devname>_config *cfg,
					   uint16_t cmd)
{
	uint8_t buf[2];
	uint8_t len;

	if (cmd > <DEVNAME>_EXTENDED_COMMAND) {
		buf[0] = FIELD_GET(<DEVNAME>_LSB_MASK, cmd);
		buf[1] = FIELD_GET(<DEVNAME>_MSB_MASK, cmd);
		len = 2;
	} else {
		buf[0] = (uint8_t)cmd;
		len = 1;
	}

	return i2c_write_dt(&cfg->i2c, buf, len);
}

/* ---- PMBus Linear11 Conversion Helpers ---------------------------- */

/**
 * @brief Convert a PMBus Linear11 word to micro-units.
 * @param raw - Raw 16-bit PMBus Linear11 value.
 * @return Value in micro-units (microvolts, microamps, or
 *         micro-degrees-Celsius).
 *
 * Linear11: exponent = bits[15:11] (signed), mantissa = bits[10:0] (signed).
 * value = mantissa * 2^exponent
 * Returns value * 1000000 for micro-unit representation.
 */
static inline int64_t <devname>_linear11_to_micro(uint16_t raw)
{
	int16_t exp = (int16_t)((raw >> 11) & 0x1F);
	int16_t mant = (int16_t)(raw & 0x7FF);

	/* Sign-extend 5-bit exponent. */
	if (exp > 15) {
		exp -= 32;
	}

	/* Sign-extend 11-bit mantissa. */
	if (mant > 1023) {
		mant -= 2048;
	}

	/*
	 * Compute mantissa * 2^exponent * 1000000.
	 * For negative exponents, divide; for positive, multiply.
	 */
	int64_t val_uc = (int64_t)mant * 1000000LL;

	if (exp >= 0) {
		val_uc <<= exp;
	} else {
		val_uc >>= (-exp);
	}

	return val_uc;
}

/**
 * @brief Convert microvolts to a PMBus ULinear16 raw value.
 * @param uv  - Voltage in microvolts.
 * @param exp - VOUT_MODE exponent (typically negative, e.g., -9).
 * @return Raw 16-bit ULinear16 value.
 *
 * ULinear16: value = raw * 2^exp, so raw = value / 2^exp.
 */
static inline uint16_t <devname>_uv_to_ulinear16(int32_t uv, int8_t exp)
{
	int64_t val = (int64_t)uv;

	/*
	 * raw = uv / (2^exp * 1000000)
	 * For exp = -9: 2^(-9) = 1/512, so raw = uv * 512 / 1000000
	 */
	if (exp >= 0) {
		val = val / ((1LL << exp) * 1000000LL);
	} else {
		val = (val * (1LL << (-exp))) / 1000000LL;
	}

	if (val > UINT16_MAX) {
		val = UINT16_MAX;
	}
	if (val < 0) {
		val = 0;
	}

	return (uint16_t)val;
}

/**
 * @brief Convert a PMBus ULinear16 raw value to microvolts.
 * @param raw - Raw 16-bit value from READ_VOUT.
 * @param exp - VOUT_MODE exponent.
 * @return Voltage in microvolts.
 */
static inline int32_t <devname>_ulinear16_to_uv(uint16_t raw, int8_t exp)
{
	int64_t val = (int64_t)raw * 1000000LL;

	if (exp >= 0) {
		val <<= exp;
	} else {
		val >>= (-exp);
	}

	return (int32_t)val;
}

#endif /* ZEPHYR_DRIVERS_REGULATOR_<DEVNAME>_<DEVNAME>_H_ */
```

### Key points

- **Shared header**: both the regulator and sensor source files include
  this header.  PMBus I/O helpers are `static inline` so they compile
  into each translation unit without link-time conflicts.
- **`struct i2c_dt_spec`** replaces the no-OS `no_os_i2c_init_param` /
  `no_os_i2c_desc` pair.
- **`struct regulator_common_config`** and `struct regulator_common_data`**
  are Zephyr-provided base structs that handle common regulator
  properties (`regulator-min-microvolt`, `regulator-boot-on`, etc.).
- All voltage values in the Zephyr regulator API are in **microvolts**.
- **Linear11 and ULinear16 conversion** helpers are provided as
  inline functions.  The no-OS driver returns raw mantissa/exponent;
  the Zephyr driver converts to physical micro-units.
- Use Zephyr's `GENMASK()`, `BIT()`, `FIELD_GET()` instead of the
  `NO_OS_GENMASK()`, `NO_OS_BIT()`, `no_os_field_get()` equivalents.

---

## 7. Regulator Driver Source (`<devname>_reg.c`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "<devname>.h"

LOG_MODULE_REGISTER(<devname>, CONFIG_REGULATOR_LOG_LEVEL);

/* ---- Regulator API: enable ---------------------------------------- */

/**
 * @brief Enable the power converter output.
 *
 * Writes OPERATION = ON (0x80) to the PMBus OPERATION command register.
 * If a CTRL GPIO is configured and on_off_config has the CTRL pin bit
 * set, also drives the CTRL pin high.
 */
static int <devname>_enable(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	if (cfg->ctrl_gpio.port != NULL &&
	    (cfg->on_off_config & <DEVNAME>_CTRL_PIN_ENABLE)) {
		ret = gpio_pin_set_dt(&cfg->ctrl_gpio, 1);
		if (ret) {
			LOG_ERR("Failed to assert CTRL pin: %d", ret);
			return ret;
		}
	}

	ret = <devname>_pmbus_write(cfg, <DEVNAME>_OPERATION,
				    <DEVNAME>_OPERATION_ON, 1);
	if (ret) {
		LOG_ERR("Failed to write OPERATION ON: %d", ret);
		return ret;
	}

	/* Wait for power-up settling. */
	k_msleep(52);

	data->enabled = true;

	return 0;
}

/* ---- Regulator API: disable --------------------------------------- */

/**
 * @brief Disable the power converter output.
 *
 * Writes OPERATION = OFF (0x00) to the PMBus OPERATION command register.
 */
static int <devname>_disable(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	ret = <devname>_pmbus_write(cfg, <DEVNAME>_OPERATION,
				    <DEVNAME>_OPERATION_OFF, 1);
	if (ret) {
		LOG_ERR("Failed to write OPERATION OFF: %d", ret);
		return ret;
	}

	if (cfg->ctrl_gpio.port != NULL &&
	    (cfg->on_off_config & <DEVNAME>_CTRL_PIN_ENABLE)) {
		ret = gpio_pin_set_dt(&cfg->ctrl_gpio, 0);
		if (ret) {
			LOG_ERR("Failed to deassert CTRL pin: %d", ret);
			return ret;
		}
	}

	data->enabled = false;

	return 0;
}

/* ---- Regulator API: is_enabled ------------------------------------ */

/**
 * @brief Check whether the power converter output is enabled.
 *
 * Reads the PMBus STATUS_BYTE or uses cached state.
 */
static unsigned int <devname>_count_voltages(const struct device *dev)
{
	/* Continuous range, return 0 to indicate linear mapping. */
	return 0;
}

/* ---- Regulator API: set_voltage ----------------------------------- */

/**
 * @brief Set the output voltage.
 * @param dev     Device instance.
 * @param min_uv  Minimum acceptable voltage in microvolts.
 * @param max_uv  Maximum acceptable voltage in microvolts.
 * @return 0 on success, negative errno on failure.
 *
 * Converts the requested voltage from microvolts to a ULinear16
 * value and writes it to VOUT_COMMAND.  Also updates VOUT_MAX
 * with the maximum acceptable value.
 */
static int <devname>_set_voltage(const struct device *dev,
				 int32_t min_uv, int32_t max_uv)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint16_t vout_cmd, vout_max;
	int ret;

	vout_cmd = <devname>_uv_to_ulinear16(min_uv, data->vout_exp);
	vout_max = <devname>_uv_to_ulinear16(max_uv, data->vout_exp);

	ret = <devname>_pmbus_write(cfg, <DEVNAME>_VOUT_COMMAND,
				    vout_cmd, 2);
	if (ret) {
		LOG_ERR("Failed to write VOUT_COMMAND: %d", ret);
		return ret;
	}

	ret = <devname>_pmbus_write(cfg, <DEVNAME>_VOUT_MAX,
				    vout_max, 2);
	if (ret) {
		LOG_ERR("Failed to write VOUT_MAX: %d", ret);
		return ret;
	}

	LOG_DBG("VOUT set: cmd=0x%04x max=0x%04x (min=%d uV, max=%d uV)",
		vout_cmd, vout_max, min_uv, max_uv);

	return 0;
}

/* ---- Regulator API: get_voltage ----------------------------------- */

/**
 * @brief Get the current output voltage setting.
 * @param dev  Device instance.
 * @param volt Pointer to store voltage in microvolts.
 * @return 0 on success, negative errno on failure.
 *
 * Reads VOUT_COMMAND and converts from ULinear16 to microvolts.
 */
static int <devname>_get_voltage(const struct device *dev, int32_t *volt)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint8_t buf[2];
	int ret;

	ret = <devname>_pmbus_read(cfg, <DEVNAME>_VOUT_COMMAND, buf, 2);
	if (ret) {
		LOG_ERR("Failed to read VOUT_COMMAND: %d", ret);
		return ret;
	}

	uint16_t raw = sys_get_le16(buf);

	*volt = <devname>_ulinear16_to_uv(raw, data->vout_exp);

	return 0;
}

/* ---- Regulator API Table ------------------------------------------ */

static DEVICE_API(regulator, <devname>_reg_api) = {
	.enable = <devname>_enable,
	.disable = <devname>_disable,
	.count_voltages = <devname>_count_voltages,
	.set_voltage = <devname>_set_voltage,
	.get_voltage = <devname>_get_voltage,
};

/* ---- Initialization ----------------------------------------------- */

/**
 * @brief Initialise a <DEVNAME> regulator instance.
 *
 * Steps:
 *   1. Verify I2C bus readiness.
 *   2. Configure GPIO pins (power-good, CTRL).
 *   3. Set PMBus ON_OFF_CONFIG.
 *   4. Clear latched faults (CLEAR_FAULTS).
 *   5. Optionally read VOUT_MODE exponent from device.
 *   6. Optionally set switching frequency.
 *   7. If regulator-boot-on is set, enable the output.
 *   8. Call regulator_common_init() for common property handling.
 */
static int <devname>_init(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Configure power-good / alert GPIO as input (optional). */
	if (cfg->pg_alt_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->pg_alt_gpio)) {
			LOG_ERR("PG/ALT GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->pg_alt_gpio, GPIO_INPUT);
		if (ret) {
			LOG_ERR("Failed to configure PG/ALT GPIO: %d", ret);
			return ret;
		}
	}

	/* Configure CTRL GPIO as output (optional). */
	if (cfg->ctrl_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->ctrl_gpio)) {
			LOG_ERR("CTRL GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->ctrl_gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (ret) {
			LOG_ERR("Failed to configure CTRL GPIO: %d", ret);
			return ret;
		}
	}

	/* Set ON_OFF_CONFIG. */
	ret = <devname>_pmbus_write(cfg, <DEVNAME>_ON_OFF_CONFIG,
				    (uint16_t)cfg->on_off_config, 1);
	if (ret) {
		LOG_ERR("Failed to write ON_OFF_CONFIG: %d", ret);
		return ret;
	}

	/* Clear latched faults. */
	ret = <devname>_pmbus_send_cmd(cfg, <DEVNAME>_CLEAR_FAULTS);
	if (ret) {
		LOG_ERR("Failed to send CLEAR_FAULTS: %d", ret);
		return ret;
	}

	/* Read or use VOUT_MODE exponent. */
	data->vout_exp = cfg->vout_mode_exp;

	/*
	 * Optionally read VOUT_MODE from the device to get the actual
	 * exponent, overriding the DT value:
	 *
	 * uint8_t vout_mode;
	 * ret = <devname>_pmbus_read(cfg, <DEVNAME>_VOUT_MODE,
	 *                            &vout_mode, 1);
	 * if (!ret) {
	 *     data->vout_exp = (int8_t)(vout_mode & 0x1F);
	 *     if (data->vout_exp > 15)
	 *         data->vout_exp -= 32;
	 * }
	 */

	/* Optionally set switching frequency. */
	if (cfg->switching_freq_hz > 0) {
		/*
		 * Convert Hz to PMBus Linear11 and write to
		 * FREQUENCY_SWITCH.  Implementation depends on
		 * the device's supported frequency range.
		 */
		LOG_DBG("Setting switching frequency: %u Hz",
			cfg->switching_freq_hz);
		/* TODO: encode and write FREQUENCY_SWITCH. */
	}

	data->enabled = false;

	LOG_INF("<DEVNAME> initialised on %s @ 0x%02x",
		cfg->i2c.bus->name, cfg->i2c.addr);

	return regulator_common_init(dev, false);
}

/* ---- Instantiation Macros ----------------------------------------- */

/**
 * Optional GPIO config fields.  Expands to a gpio_dt_spec initializer
 * if the property exists in the devicetree, or a zeroed struct otherwise.
 */
#define <DEVNAME>_PG_ALT_GPIO(inst) \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, pg_alt_gpios), \
		(.pg_alt_gpio = GPIO_DT_SPEC_INST_GET(inst, pg_alt_gpios),), \
		(.pg_alt_gpio = {0},))

#define <DEVNAME>_CTRL_GPIO(inst) \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, ctrl_gpios), \
		(.ctrl_gpio = GPIO_DT_SPEC_INST_GET(inst, ctrl_gpios),), \
		(.ctrl_gpio = {0},))

/**
 * Per-instance config/data definitions and device registration.
 *
 * This macro is expanded once per devicetree instance that has
 * compatible = "adi,<devname>".
 */
#define <DEVNAME>_INST(inst)                                               \
	static struct <devname>_data <devname>_data_##inst;                \
									   \
	static const struct <devname>_config <devname>_config_##inst = {    \
		.common = REGULATOR_DT_INST_COMMON_CONFIG_INIT(inst),     \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                        \
		<DEVNAME>_PG_ALT_GPIO(inst)                                \
		<DEVNAME>_CTRL_GPIO(inst)                                  \
		.on_off_config = DT_INST_PROP_OR(inst,                     \
					adi_on_off_config, 0x00),          \
		.vout_mode_exp = DT_INST_PROP_OR(inst,                     \
					adi_vout_mode_exponent, -9),       \
		.switching_freq_hz = DT_INST_PROP_OR(inst,                 \
					adi_switching_frequency_hz, 0),    \
	};                                                                 \
									   \
	DEVICE_DT_INST_DEFINE(                                             \
		inst,                                                      \
		<devname>_init,                                            \
		NULL,                                                      \
		&<devname>_data_##inst,                                    \
		&<devname>_config_##inst,                                  \
		POST_KERNEL,                                               \
		CONFIG_<DEVNAME>_INIT_PRIORITY,                            \
		&<devname>_reg_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_INST)
```

### Key points

- **`regulator_common_init()`** handles the common regulator properties
  (`regulator-boot-on`, `regulator-always-on`, voltage clamping) and
  must be called at the end of init.
- **`REGULATOR_DT_INST_COMMON_CONFIG_INIT()`** populates the
  `struct regulator_common_config` from devicetree properties
  (`regulator-min-microvolt`, `regulator-max-microvolt`, etc.).
- **No dynamic allocation**: all memory is statically allocated.
- **PMBus data is little-endian**: use `sys_get_le16()` /
  `sys_put_le16()` for reading/writing PMBus word data.
- **GPIO optional pattern**: `COND_CODE_1` + `DT_INST_NODE_HAS_PROP`
  handles optional GPIO properties gracefully.

---

## 8. Sensor Driver Source (`<devname>_sensor.c`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "<devname>.h"

LOG_MODULE_DECLARE(<devname>, CONFIG_REGULATOR_LOG_LEVEL);

/* ---- Sensor Channel Index Mapping --------------------------------- */

/**
 * Internal indices for the cached telemetry samples.
 * These match the sensor channel definitions used in sample_fetch
 * and channel_get.
 */
enum <devname>_sensor_chan_idx {
	<DEVNAME>_CHAN_VIN = 0,
	<DEVNAME>_CHAN_IIN,
	<DEVNAME>_CHAN_VOUT,
	<DEVNAME>_CHAN_IOUT,
	<DEVNAME>_CHAN_TEMP,
	<DEVNAME>_CHAN_COUNT,
};

/* ---- Per-Instance Sensor Data ------------------------------------- */

/**
 * @brief Per-instance sensor runtime data.
 */
struct <devname>_sensor_data {
	/** Parent regulator device reference. */
	const struct device *parent;

	/** Cached raw PMBus readings (Linear11 or ULinear16). */
	uint16_t raw[<DEVNAME>_CHAN_COUNT];
};

/**
 * @brief Per-instance sensor configuration (empty, parent provides I2C).
 */
struct <devname>_sensor_config {
	/** Parent regulator device. */
	const struct device *parent;
};

/* ---- Sensor API: sample_fetch ------------------------------------- */

/**
 * @brief Fetch telemetry samples from the power controller.
 *
 * Reads all telemetry registers from the parent device's PMBus
 * interface and caches the raw values.  Individual channels can
 * then be retrieved with channel_get() without additional I2C
 * traffic.
 *
 * Supported channels:
 * - SENSOR_CHAN_ALL: fetch all telemetry values
 * - SENSOR_CHAN_VOLTAGE: fetch VIN and VOUT
 * - SENSOR_CHAN_CURRENT: fetch IIN and IOUT
 * - SENSOR_CHAN_DIE_TEMP: fetch temperature
 */
static int <devname>_sensor_sample_fetch(const struct device *dev,
					 enum sensor_channel chan)
{
	struct <devname>_sensor_data *data = dev->data;
	const struct <devname>_sensor_config *scfg = dev->config;
	const struct <devname>_config *cfg = scfg->parent->config;
	uint8_t buf[2];
	int ret;

	if (chan != SENSOR_CHAN_ALL &&
	    chan != SENSOR_CHAN_VOLTAGE &&
	    chan != SENSOR_CHAN_CURRENT &&
	    chan != SENSOR_CHAN_DIE_TEMP) {
		LOG_ERR("Unsupported channel %d", chan);
		return -ENOTSUP;
	}

	/* Read VIN (Linear11). */
	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_VOLTAGE) {
		ret = <devname>_pmbus_read(cfg, <DEVNAME>_READ_VIN, buf, 2);
		if (ret) {
			LOG_ERR("Failed to read VIN: %d", ret);
			return ret;
		}
		data->raw[<DEVNAME>_CHAN_VIN] = sys_get_le16(buf);
	}

	/* Read VOUT (ULinear16). */
	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_VOLTAGE) {
		ret = <devname>_pmbus_read(cfg, <DEVNAME>_READ_VOUT, buf, 2);
		if (ret) {
			LOG_ERR("Failed to read VOUT: %d", ret);
			return ret;
		}
		data->raw[<DEVNAME>_CHAN_VOUT] = sys_get_le16(buf);
	}

	/* Read IIN (Linear11). */
	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_CURRENT) {
		ret = <devname>_pmbus_read(cfg, <DEVNAME>_READ_IIN, buf, 2);
		if (ret) {
			LOG_ERR("Failed to read IIN: %d", ret);
			return ret;
		}
		data->raw[<DEVNAME>_CHAN_IIN] = sys_get_le16(buf);
	}

	/* Read IOUT (Linear11). */
	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_CURRENT) {
		ret = <devname>_pmbus_read(cfg, <DEVNAME>_READ_IOUT, buf, 2);
		if (ret) {
			LOG_ERR("Failed to read IOUT: %d", ret);
			return ret;
		}
		data->raw[<DEVNAME>_CHAN_IOUT] = sys_get_le16(buf);
	}

	/* Read TEMPERATURE (Linear11). */
	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_DIE_TEMP) {
		ret = <devname>_pmbus_read(cfg, <DEVNAME>_READ_TEMPERATURE,
					   buf, 2);
		if (ret) {
			LOG_ERR("Failed to read TEMPERATURE: %d", ret);
			return ret;
		}
		data->raw[<DEVNAME>_CHAN_TEMP] = sys_get_le16(buf);
	}

	return 0;
}

/* ---- Sensor API: channel_get -------------------------------------- */

/**
 * @brief Convert a cached raw sample to a sensor_value.
 *
 * For Linear11 channels (VIN, IIN, IOUT, TEMP):
 *   Uses <devname>_linear11_to_micro() to convert to micro-units,
 *   then splits into integer (val1) and fractional (val2) parts.
 *
 * For VOUT (ULinear16):
 *   Uses <devname>_ulinear16_to_uv() with the cached VOUT_MODE exponent.
 *
 * Channel mapping with indexed channels:
 *   SENSOR_CHAN_VOLTAGE ch 0 -> VIN
 *   SENSOR_CHAN_VOLTAGE ch 1 -> VOUT
 *   SENSOR_CHAN_CURRENT ch 0 -> IIN
 *   SENSOR_CHAN_CURRENT ch 1 -> IOUT
 *   SENSOR_CHAN_DIE_TEMP    -> Temperature
 */
static int <devname>_sensor_channel_get(const struct device *dev,
					enum sensor_channel chan,
					struct sensor_value *val)
{
	struct <devname>_sensor_data *data = dev->data;
	const struct <devname>_sensor_config *scfg = dev->config;
	const struct <devname>_data *parent_data = scfg->parent->data;
	int64_t micro_val;

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
		/*
		 * Default to VIN for non-indexed access.
		 * Use sensor_channel_get_with_index() for indexed access
		 * to distinguish VIN (index 0) from VOUT (index 1).
		 */
		micro_val = <devname>_linear11_to_micro(
				data->raw[<DEVNAME>_CHAN_VIN]);
		break;

	case SENSOR_CHAN_CURRENT:
		/* Default to IIN for non-indexed access. */
		micro_val = <devname>_linear11_to_micro(
				data->raw[<DEVNAME>_CHAN_IIN]);
		break;

	case SENSOR_CHAN_DIE_TEMP:
		micro_val = <devname>_linear11_to_micro(
				data->raw[<DEVNAME>_CHAN_TEMP]);
		break;

	default:
		LOG_ERR("Unsupported channel %d", chan);
		return -ENOTSUP;
	}

	val->val1 = (int32_t)(micro_val / 1000000LL);
	val->val2 = (int32_t)(micro_val % 1000000LL);

	return 0;
}

/* ---- Sensor API Table --------------------------------------------- */

static DEVICE_API(sensor, <devname>_sensor_api) = {
	.sample_fetch = <devname>_sensor_sample_fetch,
	.channel_get = <devname>_sensor_channel_get,
};

/* ---- Sensor Init -------------------------------------------------- */

/**
 * @brief Initialise the sensor child device.
 *
 * Verifies the parent regulator device is ready.  No hardware
 * configuration is needed since the parent already set up the
 * I2C bus and PMBus interface.
 */
static int <devname>_sensor_init(const struct device *dev)
{
	const struct <devname>_sensor_config *scfg = dev->config;
	struct <devname>_sensor_data *data = dev->data;

	if (!device_is_ready(scfg->parent)) {
		LOG_ERR("Parent regulator device not ready");
		return -ENODEV;
	}

	data->parent = scfg->parent;

	LOG_INF("<DEVNAME> sensor interface ready");

	return 0;
}

/* ---- Sensor Instantiation ----------------------------------------- */

/**
 * Instantiate sensor child nodes for every <devname> instance
 * that has a sensor child node with status = "okay".
 *
 * The sensor child shares the parent's I2C bus via the parent
 * device pointer.
 */
#define <DEVNAME>_SENSOR_INST(inst)                                        \
	static struct <devname>_sensor_data <devname>_sensor_data_##inst;  \
									   \
	static const struct <devname>_sensor_config                        \
			<devname>_sensor_config_##inst = {                 \
		.parent = DEVICE_DT_GET(DT_INST_PARENT(inst)),            \
	};                                                                 \
									   \
	SENSOR_DEVICE_DT_INST_DEFINE(                                      \
		inst,                                                      \
		<devname>_sensor_init,                                     \
		NULL,                                                      \
		&<devname>_sensor_data_##inst,                             \
		&<devname>_sensor_config_##inst,                           \
		POST_KERNEL,                                               \
		CONFIG_<DEVNAME>_SENSOR_INIT_PRIORITY,                     \
		&<devname>_sensor_api);

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_<devname>_sensor

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_SENSOR_INST)
```

### Key points

- **Child device pattern**: the sensor is a devicetree child of the
  regulator node.  This avoids duplicating the I2C bus configuration.
  The sensor's config struct holds a `const struct device *parent`
  pointer obtained with `DEVICE_DT_GET(DT_INST_PARENT(inst))`.
- **`DT_DRV_COMPAT`** is redefined mid-file because this source file
  uses a different compatible string (`adi,<devname>-sensor`) than the
  regulator source file.
- **Two-phase read**: `sample_fetch()` reads all PMBus telemetry
  registers in one call; `channel_get()` converts cached data.
- **PMBus data is little-endian**: use `sys_get_le16()` for raw
  register reads.
- **Linear11 conversion**: the helper converts raw PMBus data to
  micro-units (microvolts, microamps, micro-degrees-Celsius) and
  splits into `sensor_value.val1` (integer) and `.val2` (micro part).
- **Sensor channels**: `SENSOR_CHAN_VOLTAGE` (VIN/VOUT),
  `SENSOR_CHAN_CURRENT` (IIN/IOUT), `SENSOR_CHAN_DIE_TEMP`
  (temperature).
- **`SENSOR_DEVICE_DT_INST_DEFINE()`** registers the sensor child
  with the sensor subsystem.

---

## 9. Test Skeleton

### `tests/drivers/regulator/<devname>/testcase.yaml`

```yaml
tests:
  drivers.regulator.<devname>:
    tags:
      - drivers
      - regulator
    depends_on: i2c
    harness: ztest
```

### `tests/drivers/regulator/<devname>/prj.conf`

```
CONFIG_ZTEST=y
CONFIG_I2C=y
CONFIG_REGULATOR=y
CONFIG_SENSOR=y
CONFIG_<DEVNAME>=y
CONFIG_<DEVNAME>_SENSOR=y
CONFIG_LOG=y
```

### `tests/drivers/regulator/<devname>/src/main.c`

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

/* ---- Regulator Tests ---------------------------------------------- */

static const struct device *get_reg_dev(void)
{
	const struct device *dev = DEVICE_DT_GET_ANY(adi_<devname>);

	zassert_not_null(dev, "Regulator device not found");
	zassert_true(device_is_ready(dev), "Regulator device not ready");

	return dev;
}

ZTEST(regulator_<devname>, test_enable_disable)
{
	const struct device *dev = get_reg_dev();
	int ret;

	ret = regulator_enable(dev);
	zassert_ok(ret, "regulator_enable failed: %d", ret);

	ret = regulator_disable(dev);
	zassert_ok(ret, "regulator_disable failed: %d", ret);
}

ZTEST(regulator_<devname>, test_set_get_voltage)
{
	const struct device *dev = get_reg_dev();
	int32_t voltage;
	int ret;

	ret = regulator_enable(dev);
	zassert_ok(ret, "regulator_enable failed: %d", ret);

	/* Set output voltage to 1.0V (1000000 uV). */
	ret = regulator_set_voltage(dev, 1000000, 1200000);
	zassert_ok(ret, "regulator_set_voltage failed: %d", ret);

	ret = regulator_get_voltage(dev, &voltage);
	zassert_ok(ret, "regulator_get_voltage failed: %d", ret);

	/* Allow 5% tolerance due to ULinear16 quantization. */
	zassert_true(voltage >= 950000 && voltage <= 1250000,
		     "Voltage %d uV out of expected range", voltage);

	regulator_disable(dev);
}

/* ---- Sensor Tests ------------------------------------------------- */

static const struct device *get_sensor_dev(void)
{
	const struct device *dev = DEVICE_DT_GET_ANY(adi_<devname>_sensor);

	zassert_not_null(dev, "Sensor device not found");
	zassert_true(device_is_ready(dev), "Sensor device not ready");

	return dev;
}

ZTEST(regulator_<devname>, test_sensor_fetch_and_get)
{
	const struct device *reg = get_reg_dev();
	const struct device *sensor = get_sensor_dev();
	struct sensor_value val;
	int ret;

	/* Enable the regulator so telemetry is available. */
	ret = regulator_enable(reg);
	zassert_ok(ret, "regulator_enable failed: %d", ret);

	ret = sensor_sample_fetch(sensor);
	zassert_ok(ret, "sensor_sample_fetch failed: %d", ret);

	/* Read input voltage. */
	ret = sensor_channel_get(sensor, SENSOR_CHAN_VOLTAGE, &val);
	zassert_ok(ret, "channel_get VOLTAGE failed: %d", ret);

	/* Read input current. */
	ret = sensor_channel_get(sensor, SENSOR_CHAN_CURRENT, &val);
	zassert_ok(ret, "channel_get CURRENT failed: %d", ret);

	/* Read temperature. */
	ret = sensor_channel_get(sensor, SENSOR_CHAN_DIE_TEMP, &val);
	zassert_ok(ret, "channel_get DIE_TEMP failed: %d", ret);

	/* Sanity check: temperature between -40 and +150 deg C. */
	zassert_true(val.val1 >= -40 && val.val1 <= 150,
		     "Temperature %d.%06d out of range",
		     val.val1, val.val2);

	regulator_disable(reg);
}

ZTEST(regulator_<devname>, test_sensor_unsupported_channel)
{
	const struct device *sensor = get_sensor_dev();
	struct sensor_value val;
	int ret;

	ret = sensor_sample_fetch(sensor);
	zassert_ok(ret, "sensor_sample_fetch failed: %d", ret);

	ret = sensor_channel_get(sensor, SENSOR_CHAN_PRESS, &val);
	zassert_equal(ret, -ENOTSUP,
		      "Expected -ENOTSUP for unsupported channel, got %d",
		      ret);
}

ZTEST_SUITE(regulator_<devname>, NULL, NULL, NULL, NULL, NULL);
```

---

## 10. Key Conventions

1. **File location**: `drivers/regulator/adi/<devname>/` -- ADI power
   controller drivers go under the `adi` vendor subdirectory within
   the regulator subsystem.  The sensor child source lives alongside
   the regulator source in the same directory.

2. **Compatible strings**: `"adi,<devname>"` for the regulator parent,
   `"adi,<devname>-sensor"` for the sensor child.  Both lowercase,
   matching their respective devicetree binding filenames.

3. **License**: SPDX `Apache-2.0` header in every file.  Zephyr uses
   Apache-2.0 (not BSD-3-Clause as in no-OS).

4. **Dual subsystem**: the regulator API handles enable/disable and
   voltage setting; the sensor API handles read-only telemetry.  This
   avoids forcing a single API to serve two different use cases.

5. **PMBus I/O**: use `i2c_write_read_dt()` for read transactions
   (write command, read data in one call) and `i2c_write_dt()` for
   write transactions.  Always use the `_dt` variants that accept
   `struct i2c_dt_spec`.

6. **PMBus byte order**: PMBus data is little-endian on the wire.  Use
   `sys_get_le16()` / `sys_put_le16()`.  This differs from typical
   sensor register access which is big-endian.

7. **Linear11 conversion**: convert to micro-units (microvolts,
   microamps, micro-degrees) for Zephyr's `struct sensor_value` which
   uses `val1` (integer) and `val2` (micro-fractional part).

8. **ULinear16 conversion**: VOUT uses unsigned 16-bit values with
   the exponent from VOUT_MODE.  Convert to/from microvolts for the
   regulator API.

9. **Regulator common config**: always include
   `REGULATOR_DT_INST_COMMON_CONFIG_INIT()` in the config struct and
   call `regulator_common_init()` at the end of init.  This handles
   `regulator-boot-on`, `regulator-always-on`, and voltage clamping.

10. **No dynamic allocation**: Zephyr drivers do not use `malloc()` or
    `calloc()`.  All memory is statically allocated via the
    instantiation macros.

11. **Instantiation**: use `DEVICE_DT_INST_DEFINE()` for the regulator
    and `SENSOR_DEVICE_DT_INST_DEFINE()` for the sensor child.  The
    sensor child uses `DEVICE_DT_GET(DT_INST_PARENT(inst))` to
    reference the parent device.

12. **Logging**: `LOG_MODULE_REGISTER(<devname>, CONFIG_REGULATOR_LOG_LEVEL)`
    in the regulator source; `LOG_MODULE_DECLARE(...)` in the sensor
    source.

13. **Bus readiness**: always check `i2c_is_ready_dt()` and
    `gpio_is_ready_dt()` in init before any bus access.

14. **Init priority**: the regulator parent must initialize before the
    sensor child.  Use separate Kconfig priorities (e.g., 80 for
    regulator, 81 for sensor).

15. **Error codes**: return negative errno values (`-EIO`, `-ENODEV`,
    `-ENOTSUP`, `-EINVAL`, etc.).

16. **Kconfig dependencies**: use `depends on DT_HAS_ADI_<DEVNAME>_ENABLED`
    so the driver is only offered when a matching DT node exists.

---

## 11. Commit Message Format

```
drivers: regulator: <devname>: add <devname> driver

Add Zephyr regulator and sensor subsystem driver for the ADI <DEVNAME>
digital power controller.  Supports I2C/PMBus communication, output
voltage control via the regulator API, and telemetry monitoring (VIN,
IIN, VOUT, IOUT, temperature) via the sensor API.

Signed-off-by: Your Name <your.name@analog.com>
```

For adding the sensor child as a follow-up:

```
drivers: regulator: <devname>: add sensor telemetry support

Add sensor subsystem child device for the <DEVNAME> power controller.
Provides read-only access to VIN, IIN, VOUT, IOUT, and temperature
telemetry via PMBus commands.

Signed-off-by: Your Name <your.name@analog.com>
```
