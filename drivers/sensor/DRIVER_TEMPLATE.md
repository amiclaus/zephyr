# Consolidated Zephyr Sensor Driver Template

This template covers every file needed to add a new Analog Devices sensor
driver to Zephyr RTOS under the sensor subsystem. It consolidates patterns
from accelerometer, temperature, energy meter, and other sensor-type templates
into a single reference.

Replace `<devname>` with the part number (e.g., `adxl362`, `adt7420`,
`ade7753`) and `<DEVNAME>` with its uppercase form throughout.

Hardware patterns (register maps, SPI framing, calibration constants) come
from the companion no-OS template in each driver category directory. This
template focuses exclusively on the Zephyr integration layer: devicetree
bindings, Kconfig, the sensor API implementation, triggers, and tests.

---

## 1. Purpose & Subsystem Mapping

All Analog Devices sensor drivers live under `drivers/sensor/` in the Zephyr
tree. The sensor API provides a uniform two-phase interface (fetch + get) for
applications to read sensor data without knowing hardware details.

### Sensor Value Representation

Zephyr represents all sensor values with `struct sensor_value`:

```c
struct sensor_value {
    int32_t val1;  /* integer part */
    int32_t val2;  /* fractional part in micro-units (1/1,000,000) */
};
```

Examples:
- 9.806650 m/s^2 -> `{ .val1 = 9, .val2 = 806650 }`
- 25.0625 deg C  -> `{ .val1 = 25, .val2 = 62500 }`
- 49.987 Hz      -> `{ .val1 = 49, .val2 = 987000 }`

### Channel Mapping Table

The following table maps sensor device types to the appropriate Zephyr
sensor channels. Use standard channels where available; define custom
channels starting at `SENSOR_CHAN_PRIV_START` for device-specific
quantities.

| Sensor Type           | Standard Channels                                        | Custom Channels (`SENSOR_CHAN_PRIV_START + N`)      |
|-----------------------|----------------------------------------------------------|-----------------------------------------------------|
| **accel**             | `SENSOR_CHAN_ACCEL_X`, `_Y`, `_Z`, `_XYZ`               | --                                                  |
| **gyro**              | `SENSOR_CHAN_GYRO_X`, `_Y`, `_Z`, `_XYZ`                | --                                                  |
| **temperature**       | `SENSOR_CHAN_AMBIENT_TEMP`, `SENSOR_CHAN_DIE_TEMP`        | --                                                  |
| **ecg / afe**         | `SENSOR_CHAN_VOLTAGE` (if applicable)                     | `SENSOR_CHAN_PRIV_START` (raw ADC, biopotential)     |
| **filter / amplifier**| --                                                       | `SENSOR_CHAN_PRIV_START` (filtered output)           |
| **meter**             | `SENSOR_CHAN_VOLTAGE`, `SENSOR_CHAN_CURRENT`, `SENSOR_CHAN_POWER` | `+0` active energy, `+1` apparent energy, `+2` line frequency |
| **resolver / position**| `SENSOR_CHAN_ROTATION`                                   | --                                                  |
| **cdc**               | --                                                       | `SENSOR_CHAN_PRIV_START` (capacitance)               |
| **impedance-analyzer**| --                                                       | `SENSOR_CHAN_PRIV_START` (impedance)                 |
| **potentiometer**     | --                                                       | `SENSOR_CHAN_PRIV_START` (resistance)                |
| **photo-electronic**  | `SENSOR_CHAN_LIGHT`, `SENSOR_CHAN_IR`                     | --                                                  |

### Defining Custom Channels

When standard channels do not cover your sensor's measurement quantities,
define custom channel enums in the driver header:

```c
#define <DEVNAME>_CHAN_ACTIVE_ENERGY \
    ((enum sensor_channel)(SENSOR_CHAN_PRIV_START + 0))
#define <DEVNAME>_CHAN_APPARENT_ENERGY \
    ((enum sensor_channel)(SENSOR_CHAN_PRIV_START + 1))
#define <DEVNAME>_CHAN_LINE_FREQUENCY \
    ((enum sensor_channel)(SENSOR_CHAN_PRIV_START + 2))
```

### Units by Channel Type

| Channel Type       | Unit                  | `val1`       | `val2`              |
|--------------------|-----------------------|--------------|---------------------|
| Acceleration       | m/s^2                 | integer part | micro m/s^2         |
| Gyroscope          | rad/s                 | integer part | micro rad/s         |
| Temperature        | degrees Celsius       | integer part | micro-degrees       |
| Voltage            | Volts                 | integer part | micro-Volts         |
| Current            | Amps                  | integer part | micro-Amps          |
| Power              | Watts                 | integer part | micro-Watts         |
| Rotation           | degrees               | integer part | micro-degrees       |
| Light              | lux                   | integer part | micro-lux           |

---

## 2. File Checklist

All ADI sensor drivers use vendor-based directory organization:
`drivers/sensor/adi/<devname>/`.

```
drivers/sensor/adi/<devname>/
    <devname>.h              # Driver header (config/data structs, register map, prototypes)
    <devname>.c              # Driver source (init, sample_fetch, channel_get)
    <devname>_trigger.c      # Trigger/interrupt support (optional)
    Kconfig                  # Build-time configuration options
    CMakeLists.txt           # Build rules

dts/bindings/sensor/
    adi,<devname>.yaml       # Devicetree binding

tests/drivers/sensor/<devname>/
    testcase.yaml            # Test metadata
    prj.conf                 # Test project config
    src/main.c               # Test source
```

### Naming Conventions

| Item              | Pattern                           | Example                          |
|-------------------|-----------------------------------|----------------------------------|
| Driver directory  | `drivers/sensor/adi/<devname>/`   | `drivers/sensor/adi/adxl362/`    |
| Source files      | `<devname>.c`, `<devname>.h`      | `adxl362.c`, `adxl362.h`        |
| Trigger file      | `<devname>_trigger.c`             | `adxl362_trigger.c`             |
| Kconfig symbol    | `ADI_<DEVNAME>`                   | `ADI_ADXL362`                   |
| Trigger Kconfig   | `<DEVNAME>_TRIGGER`               | `ADXL362_TRIGGER`               |
| DT compatible     | `"adi,<devname>"`                 | `"adi,adxl362"`                 |
| DT binding file   | `adi,<devname>.yaml`              | `adi,adxl362.yaml`              |
| Config struct     | `<devname>_config`                | `adxl362_config`                |
| Data struct       | `<devname>_data`                  | `adxl362_data`                  |
| API struct        | `<devname>_driver_api`            | `adxl362_driver_api`            |
| Init function     | `<devname>_init`                  | `adxl362_init`                  |
| Register defines  | `<DEVNAME>_REG_<NAME>`            | `ADXL362_REG_POWER_CTL`         |
| Field masks       | `<DEVNAME>_<FIELD>_MSK`           | `ADXL362_RANGE_MSK`             |
| Custom channels   | `<DEVNAME>_CHAN_<NAME>`           | `ADE7753_CHAN_ACTIVE_ENERGY`     |

---

## 3. Devicetree Binding (`dts/bindings/sensor/adi,<devname>.yaml`)

### SPI Sensor Example (accelerometer / meter)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Analog Devices <DEVNAME> sensor.
  Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/<DEVNAME>.pdf

compatible: "adi,<devname>"

include: [sensor-device.yaml, spi-device.yaml]

properties:
  int1-gpios:
    type: phandle-array
    description: |
      INT1 interrupt pin. This signal is active high as produced by the
      sensor. The property value should ensure the flags properly describe
      the signal that is presented to the driver.

  int2-gpios:
    type: phandle-array
    description: |
      INT2 interrupt pin (optional). Active-high by default from the sensor.

  reset-gpios:
    type: phandle-array
    description: |
      GPIO connected to the hardware RESET pin.
      If omitted, the driver performs a software reset at initialisation.

  odr:
    type: int
    default: 3
    description: |
      Output data rate selection (device-specific enum).
```

### I2C Sensor Example (temperature)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  ADI <DEVNAME> temperature sensor.
  Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/<DEVNAME>.pdf

compatible: "adi,<devname>"

include: [sensor-device.yaml, i2c-device.yaml]

properties:
  int-gpios:
    type: phandle-array
    description: |
      INT pin connection. This pin is active-low by default.
      Provide a GPIO specifier to enable trigger/interrupt support.
      Example: int-gpios = <&gpio0 2 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;

  resolution:
    type: int
    default: 13
    description: |
      ADC resolution in bits. Supported values depend on the part.
    enum:
      - 13
      - 16
```

### Dual-Bus Support

For drivers supporting both SPI and I2C, use one of these approaches:

1. **Separate bindings**: `adi,<devname>-spi.yaml` and `adi,<devname>-i2c.yaml`
   each including the respective bus yaml.
2. **Common binding**: a shared `adi,<devname>-common.yaml` with bus-specific
   child bindings that include both the common file and the bus yaml.

### Devicetree Usage Examples

**SPI:**

```dts
&spi1 {
    status = "okay";

    <devname>@0 {
        compatible = "adi,<devname>";
        reg = <0>;
        spi-max-frequency = <8000000>;
        int1-gpios = <&gpio0 14 GPIO_ACTIVE_HIGH>;
        odr = <3>;
    };
};
```

**I2C:**

```dts
&i2c0 {
    <devname>@48 {
        compatible = "adi,<devname>";
        reg = <0x48>;
        int-gpios = <&gpio0 2 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        resolution = <16>;
    };
};
```

---

## 4. Kconfig

```kconfig
# ADI <DEVNAME> sensor driver

# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

menuconfig ADI_<DEVNAME>
	bool "<DEVNAME> Sensor Driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	depends on SPI || I2C
	select SPI if $(dt_compat_on_bus,$(DT_COMPAT_ADI_<DEVNAME>),spi)
	select I2C if $(dt_compat_on_bus,$(DT_COMPAT_ADI_<DEVNAME>),i2c)
	help
	  Enable driver for the Analog Devices <DEVNAME> sensor.

if ADI_<DEVNAME>

choice <DEVNAME>_TRIGGER_MODE
	prompt "Trigger mode"
	default <DEVNAME>_TRIGGER_NONE
	help
	  Specify the type of triggering to be used by the driver.

config <DEVNAME>_TRIGGER_NONE
	bool "No trigger"

config <DEVNAME>_TRIGGER_GLOBAL_THREAD
	bool "Use global thread"
	depends on GPIO
	select <DEVNAME>_TRIGGER

config <DEVNAME>_TRIGGER_OWN_THREAD
	bool "Use own thread"
	depends on GPIO
	select <DEVNAME>_TRIGGER

endchoice

config <DEVNAME>_TRIGGER
	bool

if <DEVNAME>_TRIGGER

config <DEVNAME>_THREAD_PRIORITY
	int "Thread priority"
	depends on <DEVNAME>_TRIGGER_OWN_THREAD
	default 10
	help
	  Priority of thread used by the driver to handle interrupts.

config <DEVNAME>_THREAD_STACK_SIZE
	int "Thread stack size"
	depends on <DEVNAME>_TRIGGER_OWN_THREAD
	default 1024
	help
	  Stack size of thread used by the driver to handle interrupts.

endif # <DEVNAME>_TRIGGER

endif # ADI_<DEVNAME>
```

### Trigger Mode Summary

| Config                              | Description                                    |
|-------------------------------------|------------------------------------------------|
| `<DEVNAME>_TRIGGER_NONE`           | Polling only, no interrupt support              |
| `<DEVNAME>_TRIGGER_GLOBAL_THREAD`  | Interrupt deferred to system workqueue thread   |
| `<DEVNAME>_TRIGGER_OWN_THREAD`     | Interrupt deferred to a dedicated thread        |

For SPI-only drivers, replace `depends on SPI || I2C` with `select SPI`
and remove the I2C bus selection lines.

---

## 5. CMakeLists.txt

```cmake
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  <devname>.c
)

zephyr_library_sources_ifdef(
  CONFIG_<DEVNAME>_TRIGGER
  <devname>_trigger.c
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

#ifndef ZEPHYR_DRIVERS_SENSOR_<DEVNAME>_<DEVNAME>_H_
#define ZEPHYR_DRIVERS_SENSOR_<DEVNAME>_<DEVNAME>_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>     /* Include for SPI devices */
#include <zephyr/drivers/i2c.h>     /* Include for I2C devices */
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/* ---- Register Addresses ------------------------------------------- */

#define <DEVNAME>_REG_STATUS         0x00
#define <DEVNAME>_REG_CONFIG         0x01
#define <DEVNAME>_REG_DATA           0x02
#define <DEVNAME>_REG_ID             0x0B
/* ... add device-specific registers ... */

/* ---- Field Masks -------------------------------------------------- */

#define <DEVNAME>_CONFIG_MODE_MSK    GENMASK(1, 0)
#define <DEVNAME>_STATUS_DRDY        BIT(0)
/* ... add device-specific field masks ... */

/* ---- Custom Sensor Channels (if needed) --------------------------- */
/*
 * Standard channels are defined in <zephyr/drivers/sensor_types.h>.
 * Define custom channels for device-specific quantities:
 */
/* #define <DEVNAME>_CHAN_CUSTOM_0 \
 *     ((enum sensor_channel)(SENSOR_CHAN_PRIV_START + 0))
 */

/* ---- Bus Abstraction (for dual SPI/I2C support) ------------------- */
/*
 * For single-bus drivers, use struct spi_dt_spec or struct i2c_dt_spec
 * directly.  For dual-bus drivers, use a union and function pointers:
 */

typedef int (*<devname>_bus_access_fn)(const struct device *dev,
                                      uint8_t cmd, uint8_t reg_addr,
                                      uint8_t *data, size_t length);

struct <devname>_bus_ops {
    <devname>_bus_access_fn read;
    <devname>_bus_access_fn write;
};

union <devname>_bus {
    struct spi_dt_spec spi;
    struct i2c_dt_spec i2c;
};

/* ---- Driver Structures -------------------------------------------- */

/**
 * @brief Per-instance configuration (const, from devicetree, stored in ROM).
 */
struct <devname>_config {
    /** Bus spec (SPI or I2C) populated from devicetree. */
    union <devname>_bus bus;
    /** Bus-specific read/write operations. */
    const struct <devname>_bus_ops *bus_ops;

    /*
     * For a single-bus driver, replace the above with:
     *   struct spi_dt_spec spi;   -- SPI only
     *   struct i2c_dt_spec i2c;   -- I2C only
     */

#ifdef CONFIG_<DEVNAME>_TRIGGER
    /** INT1 GPIO spec from devicetree. */
    struct gpio_dt_spec int1_gpio;
#endif

    /** Device-specific DT properties. */
    uint8_t odr;
    uint8_t range;
};

/**
 * @brief Per-instance runtime data (mutable, stored in RAM).
 */
struct <devname>_data {
    /*
     * Cached sample data. Structure depends on sensor type:
     *
     * Multi-axis (accel/gyro):
     *   int16_t acc_x, acc_y, acc_z;
     *   uint8_t selected_range;
     *
     * Single-value (temperature):
     *   int16_t sample;
     *
     * Multi-channel (meter):
     *   uint32_t vrms_raw;
     *   uint32_t irms_raw;
     *   int32_t active_energy_raw;
     */
    int16_t sample;

#ifdef CONFIG_<DEVNAME>_TRIGGER
    /** Device back-reference for trigger work. */
    const struct device *dev;
    /** GPIO callback structure. */
    struct gpio_callback gpio_cb;
    /** Application trigger handler and trigger reference. */
    sensor_trigger_handler_t drdy_handler;
    const struct sensor_trigger *drdy_trigger;

#if defined(CONFIG_<DEVNAME>_TRIGGER_OWN_THREAD)
    K_KERNEL_STACK_MEMBER(thread_stack,
                          CONFIG_<DEVNAME>_THREAD_STACK_SIZE);
    struct k_thread thread;
    struct k_sem sem;
#elif defined(CONFIG_<DEVNAME>_TRIGGER_GLOBAL_THREAD)
    struct k_work work;
#endif
#endif /* CONFIG_<DEVNAME>_TRIGGER */
};

/* ---- Function Prototypes ------------------------------------------ */

int <devname>_reg_read(const struct device *dev, uint8_t reg_addr,
                       uint8_t *buf, size_t len);
int <devname>_reg_write(const struct device *dev, uint8_t reg_addr,
                        uint8_t val);
int <devname>_reg_write_mask(const struct device *dev, uint8_t reg_addr,
                             uint8_t mask, uint8_t val);

#ifdef CONFIG_<DEVNAME>_TRIGGER
int <devname>_trigger_init(const struct device *dev);
int <devname>_trigger_set(const struct device *dev,
                          const struct sensor_trigger *trig,
                          sensor_trigger_handler_t handler);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_<DEVNAME>_<DEVNAME>_H_ */
```

### Key Points

- `struct spi_dt_spec` / `struct i2c_dt_spec` holds bus reference and
  address, obtained from devicetree via `SPI_DT_SPEC_INST_GET()` or
  `I2C_DT_SPEC_INST_GET()`.
- `struct gpio_dt_spec` holds the GPIO controller, pin, and flags for
  interrupt pins.
- The config struct is `const` (stored in flash); the data struct is
  mutable (stored in RAM).
- For dual-bus support, use the `union` + `bus_ops` pattern to avoid
  runtime if/else in register access paths.

---

## 7. Driver Source (`<devname>.c`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(<devname>, CONFIG_SENSOR_LOG_LEVEL);

#include <zephyr/sys/byteorder.h>
#include <zephyr/init.h>
#include "<devname>.h"
```

### Register Access Helpers

```c
/* ---- SPI bus implementation --------------------------------------- */

static int <devname>_spi_reg_read(const struct device *dev, uint8_t cmd,
                                  uint8_t reg_addr, uint8_t *data,
                                  size_t length)
{
    const struct <devname>_config *cfg = dev->config;
    /* ... SPI transceive using spi_transceive_dt(&cfg->bus.spi, ...) ... */
}

static int <devname>_spi_reg_write(const struct device *dev, uint8_t cmd,
                                   uint8_t reg_addr, uint8_t *data,
                                   size_t length)
{
    const struct <devname>_config *cfg = dev->config;
    /* ... SPI write using spi_write_dt(&cfg->bus.spi, ...) ... */
}

static const struct <devname>_bus_ops <devname>_spi_ops = {
    .read  = <devname>_spi_reg_read,
    .write = <devname>_spi_reg_write,
};

/* ---- I2C bus implementation --------------------------------------- */

static int <devname>_i2c_reg_read(const struct device *dev, uint8_t cmd,
                                  uint8_t reg_addr, uint8_t *data,
                                  size_t length)
{
    const struct <devname>_config *cfg = dev->config;
    ARG_UNUSED(cmd);
    return i2c_burst_read_dt(&cfg->bus.i2c, reg_addr, data, length);
}

static int <devname>_i2c_reg_write(const struct device *dev, uint8_t cmd,
                                   uint8_t reg_addr, uint8_t *data,
                                   size_t length)
{
    const struct <devname>_config *cfg = dev->config;
    ARG_UNUSED(cmd);
    return i2c_burst_write_dt(&cfg->bus.i2c, reg_addr, data, length);
}

static const struct <devname>_bus_ops <devname>_i2c_ops = {
    .read  = <devname>_i2c_reg_read,
    .write = <devname>_i2c_reg_write,
};
```

### sample_fetch Patterns

The `sample_fetch` function reads hardware and caches raw data. The
implementation varies by sensor type:

**Pattern A: Multi-Axis Sensor (accelerometer, gyroscope)**

```c
static int <devname>_sample_fetch(const struct device *dev,
                                  enum sensor_channel chan)
{
    struct <devname>_data *data = dev->data;
    uint8_t buf[6]; /* 3 axes x 2 bytes */
    int ret;

    if (chan != SENSOR_CHAN_ALL &&
        chan != SENSOR_CHAN_ACCEL_X &&
        chan != SENSOR_CHAN_ACCEL_Y &&
        chan != SENSOR_CHAN_ACCEL_Z &&
        chan != SENSOR_CHAN_ACCEL_XYZ) {
        return -ENOTSUP;
    }

    ret = <devname>_reg_read(dev, <DEVNAME>_REG_XDATA_L, buf, sizeof(buf));
    if (ret) {
        LOG_ERR("Failed to read acceleration data: %d", ret);
        return ret;
    }

    data->acc_x = (int16_t)sys_get_le16(&buf[0]);
    data->acc_y = (int16_t)sys_get_le16(&buf[2]);
    data->acc_z = (int16_t)sys_get_le16(&buf[4]);

    return 0;
}
```

**Pattern B: Single-Value Sensor (temperature)**

```c
static int <devname>_sample_fetch(const struct device *dev,
                                  enum sensor_channel chan)
{
    struct <devname>_data *data = dev->data;
    const struct <devname>_config *cfg = dev->config;
    uint8_t buf[2];
    int ret;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_TEMP) {
        return -ENOTSUP;
    }

    ret = i2c_burst_read_dt(&cfg->i2c, <DEVNAME>_REG_TEMP_MSB, buf, 2);
    if (ret) {
        LOG_ERR("Failed to read temperature register: %d", ret);
        return ret;
    }

    data->sample = (int16_t)sys_get_be16(buf);

    return 0;
}
```

**Pattern C: Multi-Channel Sensor (energy meter)**

```c
static int <devname>_sample_fetch(const struct device *dev,
                                  enum sensor_channel chan)
{
    struct <devname>_data *data = dev->data;
    uint32_t val;
    int ret;

    if (chan == SENSOR_CHAN_ALL) {
        /* Read all measurement registers. */
        ret = <devname>_reg_read(dev, <DEVNAME>_REG_VRMS, &val);
        if (ret < 0) return ret;
        data->vrms_raw = val;

        ret = <devname>_reg_read(dev, <DEVNAME>_REG_IRMS, &val);
        if (ret < 0) return ret;
        data->irms_raw = val;

        /* ... read additional channels ... */
        return 0;
    }

    /* Single-channel fetch. */
    switch ((int)chan) {
    case SENSOR_CHAN_VOLTAGE:
        ret = <devname>_reg_read(dev, <DEVNAME>_REG_VRMS, &val);
        if (ret < 0) return ret;
        data->vrms_raw = val;
        return 0;
    case SENSOR_CHAN_CURRENT:
        ret = <devname>_reg_read(dev, <DEVNAME>_REG_IRMS, &val);
        if (ret < 0) return ret;
        data->irms_raw = val;
        return 0;
    default:
        break;
    }

    /* Handle custom channels via SENSOR_CHAN_PRIV_START comparisons. */
    LOG_ERR("Unsupported channel %d", chan);
    return -ENOTSUP;
}
```

### channel_get Patterns

The `channel_get` function converts cached raw data to `struct sensor_value`.

**Pattern A: Multi-Axis (accelerometer)**

```c
static void <devname>_accel_convert(struct sensor_value *val,
                                    int16_t raw, uint8_t range)
{
    int64_t micro;
    /* scale_table[] holds micro m/s^2 per LSB for each range. */
    micro = (int64_t)raw * <devname>_scale_table[range];
    val->val1 = (int32_t)(micro / 1000000);
    val->val2 = (int32_t)(micro % 1000000);
}

static int <devname>_channel_get(const struct device *dev,
                                 enum sensor_channel chan,
                                 struct sensor_value *val)
{
    struct <devname>_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_ACCEL_X:
        <devname>_accel_convert(val, data->acc_x, data->selected_range);
        break;
    case SENSOR_CHAN_ACCEL_Y:
        <devname>_accel_convert(val, data->acc_y, data->selected_range);
        break;
    case SENSOR_CHAN_ACCEL_Z:
        <devname>_accel_convert(val, data->acc_z, data->selected_range);
        break;
    case SENSOR_CHAN_ACCEL_XYZ:
        /* val must point to an array of 3 sensor_value structs. */
        <devname>_accel_convert(&val[0], data->acc_x, data->selected_range);
        <devname>_accel_convert(&val[1], data->acc_y, data->selected_range);
        <devname>_accel_convert(&val[2], data->acc_z, data->selected_range);
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}
```

**Pattern B: Single-Value (temperature)**

```c
static int <devname>_channel_get(const struct device *dev,
                                 enum sensor_channel chan,
                                 struct sensor_value *val)
{
    struct <devname>_data *data = dev->data;
    int32_t raw;
    int64_t temp_uc; /* temperature in micro-degrees C */

    if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
        return -ENOTSUP;
    }

    /* 13-bit mode: data is in bits [15:3], 1 LSB = 0.0625 deg C. */
    raw = data->sample >> 3;
    temp_uc = (int64_t)raw * 62500; /* 0.0625 * 1000000 */

    val->val1 = (int32_t)(temp_uc / 1000000);
    val->val2 = (int32_t)(temp_uc % 1000000);

    return 0;
}
```

**Pattern C: Multi-Channel (meter)**

```c
static int <devname>_channel_get(const struct device *dev,
                                 enum sensor_channel chan,
                                 struct sensor_value *val)
{
    struct <devname>_data *data = dev->data;

    switch ((int)chan) {
    case SENSOR_CHAN_VOLTAGE:
        val->val1 = (int32_t)data->vrms_raw;
        val->val2 = 0;
        return 0;
    case SENSOR_CHAN_CURRENT:
        val->val1 = (int32_t)data->irms_raw;
        val->val2 = 0;
        return 0;
    case SENSOR_CHAN_POWER:
        val->val1 = data->active_energy_raw;
        val->val2 = 0;
        return 0;
    default:
        break;
    }

    /* Handle custom channels. */
    if (chan == <DEVNAME>_CHAN_LINE_FREQUENCY) {
        uint32_t freq_cc = 448280; /* Device-specific constant */
        uint32_t divisor = data->period_raw + 1;
        uint64_t freq_uhz;

        freq_uhz = ((uint64_t)freq_cc * 1000000ULL) / divisor;
        val->val1 = (int32_t)(freq_uhz / 1000000ULL);
        val->val2 = (int32_t)(freq_uhz % 1000000ULL);
        return 0;
    }

    return -ENOTSUP;
}
```

### Driver API Table and Instantiation

```c
/* ---- Sensor API Table --------------------------------------------- */

static DEVICE_API(sensor, <devname>_driver_api) = {
    .attr_set     = <devname>_attr_set,     /* optional */
    .sample_fetch = <devname>_sample_fetch,
    .channel_get  = <devname>_channel_get,
#ifdef CONFIG_<DEVNAME>_TRIGGER
    .trigger_set  = <devname>_trigger_set,
#endif
};

/* ---- Initialization ----------------------------------------------- */

static int <devname>_init(const struct device *dev)
{
    const struct <devname>_config *cfg = dev->config;
    int ret;

    /* 1. Verify bus readiness. */
    if (IS_ENABLED(CONFIG_SPI) && cfg->bus_ops == &<devname>_spi_ops) {
        if (!spi_is_ready_dt(&cfg->bus.spi)) {
            LOG_ERR("SPI bus not ready");
            return -ENODEV;
        }
    } else if (IS_ENABLED(CONFIG_I2C) &&
               cfg->bus_ops == &<devname>_i2c_ops) {
        if (!i2c_is_ready_dt(&cfg->bus.i2c)) {
            LOG_ERR("I2C bus not ready");
            return -ENODEV;
        }
    }

    /* 2. Reset and verify device ID. */
    /* ... device-specific reset and ID check ... */

    /* 3. Apply configuration from devicetree. */
    /* ... ODR, range, filters, etc. ... */

#ifdef CONFIG_<DEVNAME>_TRIGGER
    /* 4. Trigger / interrupt init. */
    ret = <devname>_trigger_init(dev);
    if (ret) {
        LOG_ERR("Trigger init failed: %d", ret);
        return ret;
    }
#endif

    LOG_INF("<DEVNAME> initialized");
    return 0;
}

/* ---- Device Instantiation Macros ---------------------------------- */

#define <DEVNAME>_SPI_CFG(inst)                                         \
    .bus = {                                                            \
        .spi = SPI_DT_SPEC_INST_GET(                                   \
            inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),              \
    },                                                                  \
    .bus_ops = &<devname>_spi_ops,

#define <DEVNAME>_I2C_CFG(inst)                                         \
    .bus = {                                                            \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                             \
    },                                                                  \
    .bus_ops = &<devname>_i2c_ops,

#define <DEVNAME>_BUS_CFG(inst)                                         \
    COND_CODE_1(DT_INST_ON_BUS(inst, spi),                             \
        (<DEVNAME>_SPI_CFG(inst)),                                      \
        (<DEVNAME>_I2C_CFG(inst)))

#define <DEVNAME>_TRIGGER_CFG(inst)                                     \
    .int1_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int1_gpios, {0}),

#define <DEVNAME>_DEFINE(inst)                                          \
    static struct <devname>_data <devname>_data_##inst;                 \
                                                                        \
    static const struct <devname>_config <devname>_config_##inst = {    \
        <DEVNAME>_BUS_CFG(inst)                                         \
        IF_ENABLED(CONFIG_<DEVNAME>_TRIGGER,                            \
                   (<DEVNAME>_TRIGGER_CFG(inst)))                       \
        .odr   = DT_INST_PROP_OR(inst, odr, 3),                        \
        .range = DT_INST_PROP_OR(inst, range, 0),                      \
    };                                                                  \
                                                                        \
    SENSOR_DEVICE_DT_INST_DEFINE(                                       \
        inst,                                                           \
        <devname>_init,                                                 \
        NULL,                                                           \
        &<devname>_data_##inst,                                         \
        &<devname>_config_##inst,                                       \
        POST_KERNEL,                                                    \
        CONFIG_SENSOR_INIT_PRIORITY,                                    \
        &<devname>_driver_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_DEFINE)
```

---

## 8. Trigger/Interrupt Support (`<devname>_trigger.c`)

This section shows the DRDY (data-ready) trigger pattern with support
for both own-thread and global-thread modes.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(<devname>, CONFIG_SENSOR_LOG_LEVEL);

#include "<devname>.h"

/* ---- Trigger handler dispatch ------------------------------------- */

/**
 * @brief Invoke the application's trigger handler.
 *
 * Called from either the global workqueue thread or the driver's own
 * thread, depending on the Kconfig choice. Reads the status register
 * to determine the interrupt source.
 */
static void <devname>_handle_interrupt(const struct device *dev)
{
    struct <devname>_data *data = dev->data;
    uint8_t status;
    int ret;

    ret = <devname>_reg_read(dev, <DEVNAME>_REG_STATUS, &status, 1);
    if (ret) {
        LOG_ERR("Failed to read status: %d", ret);
        return;
    }

    if ((status & <DEVNAME>_STATUS_DRDY) && data->drdy_handler) {
        data->drdy_handler(dev, data->drdy_trigger);
    }
}

/* ---- Own thread path ---------------------------------------------- */

#if defined(CONFIG_<DEVNAME>_TRIGGER_OWN_THREAD)

/**
 * @brief Dedicated thread entry point.
 *
 * Waits on the semaphore (signalled by the GPIO ISR) and then invokes
 * the trigger handler.
 */
static void <devname>_thread_main(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct <devname>_data *data = p1;

    while (true) {
        k_sem_take(&data->sem, K_FOREVER);
        <devname>_handle_interrupt(data->dev);
    }
}

#elif defined(CONFIG_<DEVNAME>_TRIGGER_GLOBAL_THREAD)

/* ---- Global thread path ------------------------------------------- */

/**
 * @brief Work handler submitted to the system workqueue.
 */
static void <devname>_work_handler(struct k_work *work)
{
    struct <devname>_data *data =
        CONTAINER_OF(work, struct <devname>_data, work);

    <devname>_handle_interrupt(data->dev);
}

#endif /* TRIGGER_OWN_THREAD / GLOBAL_THREAD */

/* ---- GPIO ISR ----------------------------------------------------- */

/**
 * @brief GPIO interrupt callback (runs in ISR context).
 *
 * Defers actual processing to a thread context by either submitting
 * work to the system workqueue or signalling the dedicated thread's
 * semaphore. Never perform bus I/O in ISR context.
 */
static void <devname>_gpio_callback(const struct device *port,
                                    struct gpio_callback *cb,
                                    gpio_port_pins_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    struct <devname>_data *data =
        CONTAINER_OF(cb, struct <devname>_data, gpio_cb);

#if defined(CONFIG_<DEVNAME>_TRIGGER_OWN_THREAD)
    k_sem_give(&data->sem);
#elif defined(CONFIG_<DEVNAME>_TRIGGER_GLOBAL_THREAD)
    k_work_submit(&data->work);
#endif
}

/* ---- Public API: trigger_set -------------------------------------- */

/**
 * @brief Register an application trigger handler.
 *
 * @param dev     Device instance.
 * @param trig    Trigger descriptor (type + channel).
 * @param handler Application callback, or NULL to disable.
 * @return 0 on success, negative errno on failure.
 */
int <devname>_trigger_set(const struct device *dev,
                          const struct sensor_trigger *trig,
                          sensor_trigger_handler_t handler)
{
    struct <devname>_data *data = dev->data;
    const struct <devname>_config *cfg = dev->config;

    if (trig->type != SENSOR_TRIG_DATA_READY) {
        LOG_ERR("Unsupported trigger type %d", trig->type);
        return -ENOTSUP;
    }

    /* Disable interrupt while updating handler to avoid races. */
    gpio_pin_interrupt_configure_dt(&cfg->int1_gpio, GPIO_INT_DISABLE);

    data->drdy_handler = handler;
    data->drdy_trigger = trig;

    if (handler != NULL) {
        gpio_pin_interrupt_configure_dt(&cfg->int1_gpio,
                                        GPIO_INT_EDGE_TO_ACTIVE);
    }

    return 0;
}

/* ---- Trigger initialisation --------------------------------------- */

/**
 * @brief Set up GPIO interrupt and thread/workqueue resources.
 *
 * Called from <devname>_init() when trigger support is enabled.
 */
int <devname>_trigger_init(const struct device *dev)
{
    struct <devname>_data *data = dev->data;
    const struct <devname>_config *cfg = dev->config;
    int ret;

    if (!gpio_is_ready_dt(&cfg->int1_gpio)) {
        LOG_ERR("INT GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&cfg->int1_gpio, GPIO_INPUT);
    if (ret) {
        LOG_ERR("Failed to configure INT pin: %d", ret);
        return ret;
    }

    gpio_init_callback(&data->gpio_cb, <devname>_gpio_callback,
                       BIT(cfg->int1_gpio.pin));

    ret = gpio_add_callback(cfg->int1_gpio.port, &data->gpio_cb);
    if (ret) {
        LOG_ERR("Failed to add GPIO callback: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&cfg->int1_gpio,
                                          GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
        LOG_ERR("Failed to configure GPIO interrupt: %d", ret);
        return ret;
    }

    data->dev = dev;

#if defined(CONFIG_<DEVNAME>_TRIGGER_OWN_THREAD)
    k_sem_init(&data->sem, 0, K_SEM_MAX_LIMIT);

    k_thread_create(&data->thread, data->thread_stack,
                    CONFIG_<DEVNAME>_THREAD_STACK_SIZE,
                    <devname>_thread_main, data, NULL, NULL,
                    K_PRIO_COOP(CONFIG_<DEVNAME>_THREAD_PRIORITY),
                    0, K_NO_WAIT);

    k_thread_name_set(&data->thread, "<devname>_trigger");
#elif defined(CONFIG_<DEVNAME>_TRIGGER_GLOBAL_THREAD)
    k_work_init(&data->work, <devname>_work_handler);
#endif

    LOG_DBG("Trigger initialized");
    return 0;
}
```

### Key Points

- **ISR context safety**: the GPIO callback runs in interrupt context and
  must not call blocking APIs or perform bus I/O. It defers work to a
  thread.
- **Two deferral strategies**:
  - `GLOBAL_THREAD`: submits a `k_work` item to the system workqueue.
    Simpler, lower RAM cost, but shares the workqueue with other
    subsystems.
  - `OWN_THREAD`: signals a semaphore consumed by a dedicated thread.
    Provides deterministic latency at the cost of an extra stack.
- **Trigger types**: use `SENSOR_TRIG_DATA_READY` for DRDY signals,
  `SENSOR_TRIG_THRESHOLD` for over/under-threshold alerts (temperature,
  activity detection), and `SENSOR_TRIG_FIFO_WATERMARK` for FIFO events.
- **`gpio_pin_interrupt_configure_dt()`** is used to arm/disarm the
  interrupt atomically when the handler is set or cleared.

---

## 9. Test Skeleton

### `tests/drivers/sensor/<devname>/testcase.yaml`

```yaml
tests:
  drivers.sensor.<devname>:
    tags:
      - drivers
      - sensor
    depends_on: spi
    harness: ztest
```

### `tests/drivers/sensor/<devname>/prj.conf`

```
CONFIG_ZTEST=y
CONFIG_SPI=y
CONFIG_SENSOR=y
CONFIG_ADI_<DEVNAME>=y
CONFIG_LOG=y
```

### `tests/drivers/sensor/<devname>/src/main.c`

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/ztest.h>

static const struct device *get_dev(void)
{
    const struct device *dev = DEVICE_DT_GET_ANY(adi_<devname>);

    zassert_not_null(dev, "Device not found");
    zassert_true(device_is_ready(dev), "Device not ready");

    return dev;
}

ZTEST(sensor_<devname>, test_fetch_and_get)
{
    const struct device *dev = get_dev();
    struct sensor_value val;
    int ret;

    ret = sensor_sample_fetch(dev);
    zassert_ok(ret, "sample_fetch failed: %d", ret);

    /*
     * Adjust channel and expected range for your sensor type:
     *   SENSOR_CHAN_ACCEL_X     -- accelerometer
     *   SENSOR_CHAN_AMBIENT_TEMP -- temperature
     *   SENSOR_CHAN_VOLTAGE     -- meter
     */
    ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &val);
    zassert_ok(ret, "channel_get failed: %d", ret);

    /* Sanity check: value within expected physical range. */
    zassert_true(val.val1 >= -40 && val.val1 <= 150,
                 "Value %d.%06d out of range", val.val1, val.val2);
}

ZTEST(sensor_<devname>, test_unsupported_channel)
{
    const struct device *dev = get_dev();
    struct sensor_value val;
    int ret;

    ret = sensor_sample_fetch(dev);
    zassert_ok(ret, "sample_fetch failed: %d", ret);

    /* Request a channel the driver does not support. */
    ret = sensor_channel_get(dev, SENSOR_CHAN_PRESS, &val);
    zassert_equal(ret, -ENOTSUP,
                  "Expected -ENOTSUP for unsupported channel, got %d", ret);
}

ZTEST_SUITE(sensor_<devname>, NULL, NULL, NULL, NULL, NULL);
```

### Using a Sensor Emulator

For hardware-independent testing, Zephyr supports sensor emulation.
Create an emulator backend that feeds known values to the driver's
register read functions, allowing tests to run on `native_sim` without
real hardware.

---

## 10. Key Conventions

1. **File location**: `drivers/sensor/adi/<devname>/` -- ADI drivers go
   under the `adi` vendor subdirectory.

2. **Compatible string**: `"adi,<devname>"` -- lowercase, matches the
   devicetree binding filename `adi,<devname>.yaml`.

3. **License**: SPDX `Apache-2.0` header in every file. Zephyr uses
   Apache-2.0 (not BSD-3-Clause as in no-OS).

4. **Two-phase read**: `sample_fetch()` performs the bus transaction and
   caches raw data; `channel_get()` converts and returns the value.
   This separation allows fetching all channels in one shot and
   retrieving individual channels without re-reading hardware.

5. **`struct sensor_value`**: all values use `val1` (integer part) and
   `val2` (fractional part in micro-units). The unit depends on the
   channel type (m/s^2, deg C, V, A, W, rad/s, lux, etc.).

6. **Devicetree-driven config**: all instance-specific parameters
   (bus, address, GPIO pins, ODR, range) come from the devicetree,
   not from C structs. Use `SPI_DT_SPEC_INST_GET()`,
   `I2C_DT_SPEC_INST_GET()`, `GPIO_DT_SPEC_INST_GET()`.

7. **Instantiation**: use `SENSOR_DEVICE_DT_INST_DEFINE()` (not the
   bare `DEVICE_DT_INST_DEFINE()`). This macro adds sensor-specific
   metadata needed by the sensor subsystem.

8. **API struct**: use `DEVICE_API(sensor, <devname>_api)` to define the
   driver API table with compile-time type safety.

9. **DT_DRV_COMPAT**: define `DT_DRV_COMPAT adi_<devname>` (underscores,
   not commas) at the top of every `.c` file that uses `DT_INST_*` macros.

10. **Logging**: `LOG_MODULE_REGISTER(<devname>, CONFIG_SENSOR_LOG_LEVEL)`
    in the main .c file; `LOG_MODULE_DECLARE(...)` in supplementary files
    (e.g., trigger.c). Use `LOG_ERR`, `LOG_WRN`, `LOG_INF`, `LOG_DBG` --
    never `printk` in drivers.

11. **Bus readiness**: always check `spi_is_ready_dt()` /
    `i2c_is_ready_dt()` and `gpio_is_ready_dt()` in `init()`.

12. **Register I/O**: use the `_dt` variants that accept bus spec structs:
    `spi_transceive_dt()`, `spi_write_dt()`, `i2c_burst_read_dt()`,
    `i2c_reg_read_byte_dt()`, etc. Avoid raw `spi_transfer()` /
    `i2c_transfer()` unless necessary.

13. **Byte order**: sensor registers are typically big-endian on the wire.
    Use `sys_get_be16()` / `sys_put_be16()` for conversion. Some devices
    (e.g., ADXL362) are little-endian -- use `sys_get_le16()` in that case.

14. **Bit manipulation**: use `BIT()`, `GENMASK()`, `FIELD_PREP()`,
    `FIELD_GET()` from `<zephyr/sys/util.h>` (not `NO_OS_BIT` etc.).

15. **Error codes**: return negative errno values (`-EIO`, `-ENODEV`,
    `-ENOTSUP`, `-EINVAL`, etc.).

16. **No dynamic allocation**: Zephyr drivers do not use `malloc()` or
    `k_malloc()`. All memory is statically allocated via the instantiation
    macros (`DT_INST_FOREACH_STATUS_OKAY()`).

17. **Kconfig dependencies**: use `depends on DT_HAS_ADI_<DEVNAME>_ENABLED`
    so the driver is only offered when a matching DT node exists.

18. **Dual-bus support**: for devices supporting both SPI and I2C, use the
    `union` + `bus_ops` function pointer pattern. Select the correct bus
    at build time with `DT_INST_ON_BUS(inst, spi)`.

19. **Variable-width registers**: meter ICs and some other devices have
    registers of different widths (8, 12, 16, 24 bits). Implement a width
    lookup function and use it in SPI read/write helpers to determine the
    correct transaction length.

20. **Custom channels**: document the mapping between custom channel enums
    and their physical quantities in the driver header. Applications must
    include the driver header to access `SENSOR_CHAN_PRIV_START`-based
    channel definitions.

---

## 11. Commit Message Format

Use the prefix `drivers: sensor: adi: <devname>:` for all commits.

### Adding a new driver

```
drivers: sensor: adi: <devname>: add <devname> driver

Add Zephyr sensor subsystem driver for the ADI <DEVNAME>.
Supports [SPI/I2C] communication, [describe key features],
and optional interrupt-triggered data-ready via GPIO.

Signed-off-by: Your Name <your.name@analog.com>
```

### Adding trigger support

```
drivers: sensor: adi: <devname>: add trigger support

Add interrupt/trigger support for the <DEVNAME> sensor.
Supports global thread and own thread trigger modes via
the [INT1/IRQ] pin.

Signed-off-by: Your Name <your.name@analog.com>
```

### Adding tests

```
tests: drivers: sensor: <devname>: add basic test suite

Add ztest-based test suite for the <DEVNAME> sensor driver.
Covers sample_fetch/channel_get and unsupported channel
error handling.

Signed-off-by: Your Name <your.name@analog.com>
```
