# Misc Device Zephyr Driver Template (Consolidated)

This template covers Zephyr RTOS drivers for devices that do not map to any
standard Zephyr subsystem (sensor, ADC, DAC, GPIO, etc.). These devices are
placed under `drivers/misc/` with custom, device-specific APIs. The template
provides patterns for GPIO-only, SPI-based, and I2C-based devices in a single
reference.

Replace `<devname>` with the lowercase part number (e.g., `adg2404`),
`<DEVNAME>` with its uppercase form (e.g., `ADG2404`), and `<vendor>` with
the vendor prefix (e.g., `adi`) throughout.

---

## 1. Purpose & Subsystem Mapping

Zephyr has no dedicated subsystem for the device types listed below. Each is
implemented as a custom driver under `drivers/misc/` with a vendor-specific
API header and `DEVICE_DT_INST_DEFINE()` for instantiation.

### Device Type Reference

| Device Type | Description | Bus Interface | Example Parts | Typical API Operations |
|---|---|---|---|---|
| mux | Analog multiplexers -- GPIO-controlled channel select | GPIO only | ADG2404, ADG1606 | `select_channel()`, `enable()`, `get_channel()` |
| switch | Analog switches -- GPIO on/off per channel | GPIO only | ADG1712, ADG2712 | `set_switch_state()`, `get_switch_state()`, `set_all()` |
| mcs | Machine control signals -- GPIO-based fault/enable | GPIO only | AD74115, LTC2672 | `set_output()`, `get_fault()`, `enable()` |
| io-link | IO-Link protocol transceivers | SPI | MAX14819, LT3669 | `write_isdu()`, `read_isdu()`, `set_mode()` |
| gmsl | GMSL serializer/deserializer | I2C | MAX96717, MAX96714 | `configure_link()`, `set_i2c_translate()`, `get_lock_status()` |
| rf-transceiver | Packet radio transceivers | SPI | ADF7023, ADF7242 | `transmit()`, `receive()`, `set_channel_freq()`, `set_fw_state()` |

### When to Use `drivers/misc/`

Use the misc category when **all** of the following are true:

- No existing Zephyr subsystem API covers the device functionality.
- The device does not naturally extend an existing subsystem (e.g., a
  temperature sensor should use the sensor subsystem, not misc).
- The device requires a custom API that is specific to its function
  rather than a generic interface.

If multiple devices of the same type accumulate under misc and share a
common API pattern, consider proposing a new Zephyr subsystem upstream.

---

## 2. File Checklist

```
drivers/misc/<vendor>_<devname>/
    <devname>.h               # Public API header (custom API, config/data structs)
    <devname>.c               # Driver implementation
    Kconfig                   # Driver configuration
    CMakeLists.txt            # Build integration

dts/bindings/misc/
    <vendor>,<devname>.yaml   # Devicetree binding

tests/drivers/misc/<devname>/
    testcase.yaml             # Test metadata
    prj.conf                  # Test Kconfig
    CMakeLists.txt            # Test build file
    boards/
        native_sim.overlay    # DT overlay for native_sim
    src/
        main.c                # Test source

samples/<devname>/
    sample.yaml               # Sample metadata
    prj.conf                  # Sample project config
    CMakeLists.txt            # Sample build file
    boards/
        <board>.overlay       # Board-specific overlay
    src/
        main.c                # Sample application
```

Key difference from subsystem drivers: the API header (`<devname>.h`) lives
alongside the driver source under `drivers/misc/<vendor>_<devname>/` rather
than in a global `include/zephyr/drivers/` directory, since there is no
shared subsystem API. Applications add the driver directory to their include
path via CMake.

---

## 3. Devicetree Binding (`dts/bindings/misc/<vendor>,<devname>.yaml`)

### 3.1 GPIO-controlled device (mux / switch / mcs)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  <DEVNAME> analog multiplexer/switch with GPIO-based channel selection.

  Example:

    <devname> {
        compatible = "<vendor>,<devname>";
        a0-gpios = <&gpio0 2 GPIO_ACTIVE_HIGH>;
        a1-gpios = <&gpio0 3 GPIO_ACTIVE_HIGH>;
        en-gpios = <&gpio0 4 GPIO_ACTIVE_HIGH>;
    };

compatible: "<vendor>,<devname>"

include: base.yaml

properties:
  a0-gpios:
    type: phandle-array
    required: true
    description: |
      GPIO specifier for address line A0 (LSB of channel address).

  a1-gpios:
    type: phandle-array
    required: true
    description: |
      GPIO specifier for address line A1 (bit 1 of channel address).

  en-gpios:
    type: phandle-array
    required: true
    description: |
      GPIO specifier for the enable pin. Check the datasheet for polarity
      and set GPIO_ACTIVE_HIGH or GPIO_ACTIVE_LOW accordingly.

  # For switch devices, use per-channel GPIOs instead of address lines:
  # in1-gpios:
  #   type: phandle-array
  #   required: true
  #   description: GPIO connected to switch channel 1 control input.
  # in2-gpios:
  #   ...

  # Optional initial state (switch devices):
  # default-state:
  #   type: array
  #   default: [0, 0, 0, 0]
  #   description: Initial state for each switch [SW1..SW4].
```

### 3.2 SPI-based device (io-link / rf-transceiver)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  <DEVNAME> SPI-based IO-Link / RF transceiver device.

  Example:

    &spi1 {
        <devname>: <devname>@0 {
            compatible = "<vendor>,<devname>";
            reg = <0>;
            spi-max-frequency = <1000000>;
            irq-gpios = <&gpio0 5 GPIO_ACTIVE_HIGH>;
            reset-gpios = <&gpio0 6 GPIO_ACTIVE_LOW>;
        };
    };

compatible: "<vendor>,<devname>"

include: [spi-device.yaml]

properties:
  irq-gpios:
    type: phandle-array
    required: true
    description: |
      GPIO connected to the device interrupt output pin.

  reset-gpios:
    type: phandle-array
    description: |
      GPIO connected to the hardware reset pin. Optional; if not
      provided, software reset via SPI command is used.

  # Add device-specific properties (frequency, data rate, etc.)
  # as needed for the particular device.
```

### 3.3 I2C-based device (gmsl)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  <DEVNAME> GMSL serializer/deserializer with I2C control interface.

  Example:

    &i2c0 {
        <devname>: <devname>@40 {
            compatible = "<vendor>,<devname>";
            reg = <0x40>;
            lock-gpios = <&gpio0 7 GPIO_ACTIVE_HIGH>;
            reset-gpios = <&gpio0 8 GPIO_ACTIVE_LOW>;
        };
    };

compatible: "<vendor>,<devname>"

include: [i2c-device.yaml]

properties:
  lock-gpios:
    type: phandle-array
    description: |
      GPIO connected to the GMSL link lock indicator output.
      Optional; used to detect link establishment.

  reset-gpios:
    type: phandle-array
    description: |
      GPIO connected to the hardware reset pin. Optional.

  # Add device-specific properties (link rate, I2C address
  # translation tables, etc.) as needed.
```

---

## 4. Kconfig (`drivers/misc/<vendor>_<devname>/Kconfig`)

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config <VENDOR>_<DEVNAME>
	bool "<DEVNAME> <device-type> driver"
	default y
	depends on DT_HAS_<VENDOR>_<DEVNAME>_ENABLED
	# For GPIO-only devices:
	depends on GPIO
	# For SPI devices, replace the GPIO line with:
	#   select SPI
	#   select GPIO
	# For I2C devices:
	#   select I2C
	help
	  Enable the driver for the <DEVNAME>. This device communicates
	  via <GPIO/SPI/I2C> and provides <brief description>.
```

Also add entries to the parent misc Kconfig and CMakeLists.txt:

**`drivers/misc/Kconfig`** (add line):
```kconfig
source "drivers/misc/<vendor>_<devname>/Kconfig"
```

**`drivers/misc/CMakeLists.txt`** (add line):
```cmake
add_subdirectory_ifdef(CONFIG_<VENDOR>_<DEVNAME> <vendor>_<devname>)
```

---

## 5. CMakeLists.txt (`drivers/misc/<vendor>_<devname>/CMakeLists.txt`)

```cmake
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(<devname>.c)

zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
```

The `zephyr_library_include_directories()` line ensures that the custom
API header (`<devname>.h`) can be found by both the driver itself and by
applications that add this directory to their include path.

---

## 6. Driver Header (`<devname>.h`)

The custom API header defines the device-specific API using `__subsystem`
and `__syscall` annotations. This allows the API to participate in Zephyr's
syscall mechanism when user-mode threads are enabled.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MISC_<DEVNAME>_H_
#define ZEPHYR_DRIVERS_MISC_<DEVNAME>_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * Custom API Definition
 * ================================================================
 *
 * The __subsystem annotation marks this as a driver API that can
 * be validated at compile time with DEVICE_API(). The __syscall
 * annotation on each function allows Zephyr's syscall generator
 * to create user-mode wrappers when CONFIG_USERSPACE is enabled.
 *
 * Adapt the API callbacks below to match the device type.
 */

/**
 * @brief API callback types for <device-type> operations.
 *
 * Each callback corresponds to a public API function. Drivers
 * implement these callbacks and register them via DEVICE_API().
 */

/* --- Example: mux/switch API callbacks --- */

typedef int (*<devname>_select_channel_t)(const struct device *dev,
					  uint8_t channel);
typedef int (*<devname>_enable_t)(const struct device *dev, bool enable);
typedef int (*<devname>_get_channel_t)(const struct device *dev);

/* --- Example: SPI-based device API callbacks --- */
/* typedef int (*<devname>_read_reg_t)(const struct device *dev,
 *                                     uint16_t addr, uint8_t *val);
 * typedef int (*<devname>_write_reg_t)(const struct device *dev,
 *                                      uint16_t addr, uint8_t val);
 * typedef int (*<devname>_set_mode_t)(const struct device *dev,
 *                                     uint8_t mode);
 */

/**
 * @brief <DEVNAME> driver API structure.
 *
 * Drivers populate this struct with their implementation callbacks.
 * Applications access the driver exclusively through the public
 * __syscall functions below, never through this struct directly.
 */
__subsystem struct <devname>_api {
	<devname>_select_channel_t select_channel;
	<devname>_enable_t enable;
	<devname>_get_channel_t get_channel;
};

/* ================================================================
 * Public API Functions (__syscall)
 * ================================================================ */

/**
 * @brief Select a channel (mux) or perform the primary device operation.
 *
 * @param dev     Pointer to the device instance.
 * @param channel Channel number to select.
 *
 * @retval 0       Success.
 * @retval -EINVAL Invalid channel or NULL device.
 * @retval -EIO    Hardware operation failed.
 */
__syscall int <devname>_select_channel(const struct device *dev,
				       uint8_t channel);

static inline int z_impl_<devname>_select_channel(const struct device *dev,
						   uint8_t channel)
{
	const struct <devname>_api *api =
		(const struct <devname>_api *)dev->api;

	if (api->select_channel == NULL) {
		return -ENOSYS;
	}

	return api->select_channel(dev, channel);
}

/**
 * @brief Enable or disable the device.
 *
 * @param dev    Pointer to the device instance.
 * @param enable true to enable, false to disable.
 *
 * @retval 0       Success.
 * @retval -EIO    Hardware operation failed.
 */
__syscall int <devname>_enable(const struct device *dev, bool enable);

static inline int z_impl_<devname>_enable(const struct device *dev,
					   bool enable)
{
	const struct <devname>_api *api =
		(const struct <devname>_api *)dev->api;

	if (api->enable == NULL) {
		return -ENOSYS;
	}

	return api->enable(dev, enable);
}

/**
 * @brief Get the current channel or state.
 *
 * @param dev Pointer to the device instance.
 *
 * @return The current channel/state, or a negative error code.
 */
__syscall int <devname>_get_channel(const struct device *dev);

static inline int z_impl_<devname>_get_channel(const struct device *dev)
{
	const struct <devname>_api *api =
		(const struct <devname>_api *)dev->api;

	if (api->get_channel == NULL) {
		return -ENOSYS;
	}

	return api->get_channel(dev);
}

/* ================================================================
 * Device Structures
 * ================================================================ */

/**
 * @brief Immutable per-instance configuration (from devicetree).
 *
 * Stored in ROM. One instance per DT node.
 */
struct <devname>_config {
	/* --- GPIO-only device fields (mux/switch/mcs) --- */
	struct gpio_dt_spec addr[2];  /* Address line GPIOs */
	struct gpio_dt_spec en;       /* Enable GPIO */

	/* --- SPI device fields (io-link/rf-transceiver) --- */
	/* struct spi_dt_spec spi;             */
	/* struct gpio_dt_spec irq_gpio;       */
	/* struct gpio_dt_spec reset_gpio;     */

	/* --- I2C device fields (gmsl) --- */
	/* struct i2c_dt_spec i2c;             */
	/* struct gpio_dt_spec lock_gpio;      */
	/* struct gpio_dt_spec reset_gpio;     */
};

/**
 * @brief Mutable per-instance runtime data.
 *
 * Stored in RAM. One instance per DT node.
 */
struct <devname>_data {
	/** Cached state (channel, switch states, etc.). */
	uint8_t current_channel;

	/* --- SPI device additional fields --- */
	/* struct k_mutex bus_lock;       */
	/* struct k_sem isr_sem;          */
	/* struct gpio_callback irq_cb;   */

	/* --- I2C device additional fields --- */
	/* struct k_mutex bus_lock;       */
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_MISC_<DEVNAME>_H_ */
```

### Key Points

- **`__subsystem`** on the API struct enables compile-time validation with
  `DEVICE_API()` in the instantiation macro.
- **`__syscall`** on each public function lets Zephyr's build system
  generate user-mode wrappers when `CONFIG_USERSPACE` is enabled.
- **`z_impl_`** prefixed static inline functions provide the kernel-mode
  implementation that dispatches through the API function pointers.
- Applications include the header as `#include "<devname>.h"` and call
  the `__syscall` functions. They never call the `z_impl_` variants or
  access the API struct directly.

---

## 7. Driver Source (`<devname>.c`)

### 7.1 GPIO-only device (mux / switch)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT <vendor>_<devname>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "<devname>.h"

LOG_MODULE_REGISTER(<devname>, CONFIG_GPIO_LOG_LEVEL);

#define <DEVNAME>_NUM_ADDR_LINES 2

/* ---------- API implementations ------------------------------------ */

static int <devname>_select_channel_impl(const struct device *dev,
					  uint8_t channel)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	if (channel == 0) {
		/* Channel 0 = disable (high-impedance). */
		ret = gpio_pin_set_dt(&cfg->en, 0);
		if (ret == 0) {
			data->current_channel = 0;
		}
		return ret;
	}

	/* Enable the device first. */
	ret = gpio_pin_set_dt(&cfg->en, 1);
	if (ret) {
		return ret;
	}

	/* Encode (channel - 1) into address line GPIOs. */
	for (int i = 0; i < <DEVNAME>_NUM_ADDR_LINES; i++) {
		ret = gpio_pin_set_dt(&cfg->addr[i],
				      ((channel - 1) >> i) & 0x01);
		if (ret < 0) {
			LOG_ERR("Failed to set A%d: %d", i, ret);
			return ret;
		}
	}

	data->current_channel = channel;
	LOG_DBG("Selected channel %u", channel);

	return 0;
}

static int <devname>_enable_impl(const struct device *dev, bool enable)
{
	const struct <devname>_config *cfg = dev->config;

	return gpio_pin_set_dt(&cfg->en, enable ? 1 : 0);
}

static int <devname>_get_channel_impl(const struct device *dev)
{
	const struct <devname>_data *data = dev->data;

	return (int)data->current_channel;
}

/* ---------- API struct --------------------------------------------- */

static DEVICE_API(misc, <devname>_api_impl) = {
	.select_channel = <devname>_select_channel_impl,
	.enable = <devname>_enable_impl,
	.get_channel = <devname>_get_channel_impl,
};

/* ---------- Initialization ----------------------------------------- */

static int <devname>_init(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	for (int i = 0; i < <DEVNAME>_NUM_ADDR_LINES; i++) {
		if (!gpio_is_ready_dt(&cfg->addr[i])) {
			LOG_ERR("Address line A%d GPIO not ready", i);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->addr[i],
					    GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure A%d: %d", i, ret);
			return ret;
		}
	}

	if (!gpio_is_ready_dt(&cfg->en)) {
		LOG_ERR("Enable GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->en, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure enable GPIO: %d", ret);
		return ret;
	}

	data->current_channel = 0;

	LOG_INF("<DEVNAME> initialized (disabled)");

	return 0;
}

/* ---------- Instantiation ------------------------------------------ */

#define <DEVNAME>_DEFINE(inst)                                                 \
	static struct <devname>_data <devname>_data_##inst;                    \
                                                                               \
	static const struct <devname>_config <devname>_config_##inst = {       \
		.addr = {                                                      \
			GPIO_DT_SPEC_INST_GET(inst, a0_gpios),                 \
			GPIO_DT_SPEC_INST_GET(inst, a1_gpios),                 \
		},                                                             \
		.en = GPIO_DT_SPEC_INST_GET(inst, en_gpios),                   \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst,                                            \
			      <devname>_init,                                  \
			      NULL,                                            \
			      &<devname>_data_##inst,                          \
			      &<devname>_config_##inst,                        \
			      POST_KERNEL,                                     \
			      CONFIG_GPIO_INIT_PRIORITY,                       \
			      DEVICE_API_GET(misc));

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_DEFINE)
```

### 7.2 SPI-based device (io-link / rf-transceiver)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT <vendor>_<devname>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "<devname>.h"

LOG_MODULE_REGISTER(<devname>, CONFIG_<DEVNAME>_LOG_LEVEL);

/* ---------- SPI helpers -------------------------------------------- */

/**
 * @brief Read a register over SPI.
 */
static int <devname>_read_reg(const struct device *dev,
			       uint16_t addr, uint8_t *val)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[3] = { <DEVNAME>_SPI_RD_CMD, addr >> 8, addr & 0xFF };
	uint8_t rx_buf[4] = { 0 };

	const struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	int ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);

	if (ret == 0) {
		*val = rx_buf[3];
	}

	return ret;
}

/**
 * @brief Write a register over SPI.
 */
static int <devname>_write_reg(const struct device *dev,
				uint16_t addr, uint8_t val)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[4] = { <DEVNAME>_SPI_WR_CMD, addr >> 8,
			       addr & 0xFF, val };

	const struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	return spi_write_dt(&cfg->spi, &tx_set);
}

/* ---------- IRQ handler -------------------------------------------- */

static void <devname>_irq_handler(const struct device *gpio_dev,
				   struct gpio_callback *cb,
				   uint32_t pins)
{
	struct <devname>_data *data =
		CONTAINER_OF(cb, struct <devname>_data, irq_cb);

	k_sem_give(&data->isr_sem);
}

/* ---------- Initialization ----------------------------------------- */

static int <devname>_init(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	/* Verify SPI bus readiness. */
	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Initialise synchronisation primitives. */
	k_mutex_init(&data->bus_lock);
	k_sem_init(&data->isr_sem, 0, 1);

	/* Configure IRQ GPIO. */
	if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
		LOG_ERR("IRQ GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	gpio_init_callback(&data->irq_cb, <devname>_irq_handler,
			   BIT(cfg->irq_gpio.pin));

	ret = gpio_add_callback(cfg->irq_gpio.port, &data->irq_cb);
	if (ret) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		return ret;
	}

	/* Configure optional reset GPIO. */
	if (cfg->reset_gpio.port) {
		if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->reset_gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (ret) {
			return ret;
		}

		/* Perform hardware reset. */
		gpio_pin_set_dt(&cfg->reset_gpio, 1);
		k_busy_wait(100);
		gpio_pin_set_dt(&cfg->reset_gpio, 0);
		k_msleep(10);
	}

	/* Device-specific initialization goes here. */

	LOG_INF("<DEVNAME> initialized");

	return 0;
}

/* ---------- Instantiation ------------------------------------------ */

#define <DEVNAME>_DEFINE(inst)                                                 \
	static struct <devname>_data <devname>_data_##inst;                    \
                                                                               \
	static const struct <devname>_config <devname>_config_##inst = {       \
		.spi = SPI_DT_SPEC_INST_GET(                                   \
			inst,                                                  \
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |                \
			SPI_WORD_SET(8),                                       \
			0),                                                    \
		.irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),           \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(                        \
			inst, reset_gpios, {0}),                               \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst,                                            \
			      <devname>_init,                                  \
			      NULL,                                            \
			      &<devname>_data_##inst,                          \
			      &<devname>_config_##inst,                        \
			      POST_KERNEL,                                     \
			      CONFIG_<DEVNAME>_INIT_PRIORITY,                  \
			      DEVICE_API_GET(misc));

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_DEFINE)
```

### 7.3 I2C-based device (gmsl)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT <vendor>_<devname>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "<devname>.h"

LOG_MODULE_REGISTER(<devname>, CONFIG_<DEVNAME>_LOG_LEVEL);

/* ---------- I2C helpers -------------------------------------------- */

/**
 * @brief Read a register over I2C.
 */
static int <devname>_read_reg(const struct device *dev,
			       uint16_t addr, uint8_t *val)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t addr_buf[2] = { addr >> 8, addr & 0xFF };

	return i2c_write_read_dt(&cfg->i2c, addr_buf, sizeof(addr_buf),
				 val, 1);
}

/**
 * @brief Write a register over I2C.
 */
static int <devname>_write_reg(const struct device *dev,
				uint16_t addr, uint8_t val)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t buf[3] = { addr >> 8, addr & 0xFF, val };

	return i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
}

/* ---------- Initialization ----------------------------------------- */

static int <devname>_init(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	/* Verify I2C bus readiness. */
	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	k_mutex_init(&data->bus_lock);

	/* Configure optional reset GPIO. */
	if (cfg->reset_gpio.port) {
		if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->reset_gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (ret) {
			return ret;
		}

		/* Perform hardware reset. */
		gpio_pin_set_dt(&cfg->reset_gpio, 1);
		k_busy_wait(100);
		gpio_pin_set_dt(&cfg->reset_gpio, 0);
		k_msleep(10);
	}

	/* Configure optional lock indicator GPIO. */
	if (cfg->lock_gpio.port) {
		if (!gpio_is_ready_dt(&cfg->lock_gpio)) {
			LOG_ERR("Lock GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->lock_gpio, GPIO_INPUT);
		if (ret) {
			return ret;
		}
	}

	/* Device-specific initialization goes here. */

	LOG_INF("<DEVNAME> initialized");

	return 0;
}

/* ---------- Instantiation ------------------------------------------ */

#define <DEVNAME>_DEFINE(inst)                                                 \
	static struct <devname>_data <devname>_data_##inst;                    \
                                                                               \
	static const struct <devname>_config <devname>_config_##inst = {       \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                            \
		.lock_gpio = GPIO_DT_SPEC_INST_GET_OR(                         \
			inst, lock_gpios, {0}),                                \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(                        \
			inst, reset_gpios, {0}),                               \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst,                                            \
			      <devname>_init,                                  \
			      NULL,                                            \
			      &<devname>_data_##inst,                          \
			      &<devname>_config_##inst,                        \
			      POST_KERNEL,                                     \
			      CONFIG_<DEVNAME>_INIT_PRIORITY,                  \
			      DEVICE_API_GET(misc));

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_DEFINE)
```

---

## 8. Trigger / Interrupt Support

Interrupt and trigger support depends on the device type:

| Device Type | Interrupt Support | Typical Pattern |
|---|---|---|
| mux | None -- purely GPIO output driven | N/A |
| switch | None -- purely GPIO output driven | N/A |
| mcs | Optional -- fault/alert input GPIO | GPIO interrupt callback |
| io-link | Required -- IRQ pin for data events | GPIO ISR + k_sem + worker thread |
| gmsl | Optional -- lock status change | GPIO interrupt or polling |
| rf-transceiver | Required -- TX/RX complete, state change | GPIO ISR + k_sem + RX thread |

### Pattern for interrupt-driven devices (SPI/I2C)

```c
/**
 * @brief GPIO interrupt handler for the IRQ pin.
 */
static void <devname>_irq_handler(const struct device *gpio_dev,
				   struct gpio_callback *cb,
				   uint32_t pins)
{
	struct <devname>_data *data =
		CONTAINER_OF(cb, struct <devname>_data, irq_cb);

	/* Signal the worker thread. Do NOT do SPI/I2C from ISR context. */
	k_sem_give(&data->isr_sem);
}

/**
 * @brief Worker thread that processes interrupt events.
 */
static void <devname>_worker_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct <devname>_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sem_take(&data->isr_sem, K_FOREVER);

		/* Read status register over SPI/I2C to determine
		 * the interrupt source, then act accordingly. */
	}
}
```

### Pattern for optional fault/alert GPIO (mcs)

```c
/**
 * @brief Optional: read the fault status from a sense/alert GPIO.
 */
int <devname>_get_fault(const struct device *dev, bool *fault)
{
	const struct <devname>_config *cfg = dev->config;
	int val;

	if (!cfg->fault_gpio.port) {
		return -ENOTSUP;
	}

	val = gpio_pin_get_dt(&cfg->fault_gpio);
	if (val < 0) {
		return val;
	}

	*fault = (bool)val;

	return 0;
}
```

For GPIO-only devices (mux, switch) that are purely output-driven, no
trigger or interrupt support is needed.

---

## 9. Test Skeleton

### 9.1 `tests/drivers/misc/<devname>/testcase.yaml`

```yaml
tests:
  drivers.misc.<devname>:
    tags:
      - drivers
      - misc
    depends_on: gpio     # Adjust: gpio, spi, or i2c
    harness: ztest
    platform_allow:
      - native_sim
    integration_platforms:
      - native_sim
```

### 9.2 `tests/drivers/misc/<devname>/prj.conf`

```ini
CONFIG_ZTEST=y
CONFIG_GPIO=y
# For SPI devices: CONFIG_SPI=y
# For I2C devices: CONFIG_I2C=y
CONFIG_<VENDOR>_<DEVNAME>=y
CONFIG_LOG=y
```

### 9.3 `tests/drivers/misc/<devname>/CMakeLists.txt`

```cmake
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(<devname>_test)

target_sources(app PRIVATE src/main.c)

# Include the driver's custom API header.
target_include_directories(app PRIVATE
    ${ZEPHYR_BASE}/drivers/misc/<vendor>_<devname>
)
```

### 9.4 `tests/drivers/misc/<devname>/boards/native_sim.overlay`

```dts
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/ {
	test_gpio: gpio@0 {
		compatible = "vnd,gpio";
		reg = <0x0 0x1000>;
		gpio-controller;
		#gpio-cells = <2>;
		status = "okay";
	};

	test_<devname>: <devname> {
		compatible = "<vendor>,<devname>";
		/* GPIO-only device: */
		a0-gpios = <&test_gpio 0 GPIO_ACTIVE_HIGH>;
		a1-gpios = <&test_gpio 1 GPIO_ACTIVE_HIGH>;
		en-gpios = <&test_gpio 2 GPIO_ACTIVE_HIGH>;
		status = "okay";

		/* SPI device: place under &spi0 with reg, etc. */
		/* I2C device: place under &i2c0 with reg, etc. */
	};
};
```

### 9.5 `tests/drivers/misc/<devname>/src/main.c`

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/ztest.h>

#include "<devname>.h"

#define TEST_NODE DT_NODELABEL(test_<devname>)

static const struct device *dev;

static void *test_setup(void)
{
	dev = DEVICE_DT_GET(TEST_NODE);
	zassert_not_null(dev, "Device not found");
	zassert_true(device_is_ready(dev), "Device not ready");

	return NULL;
}

static void test_before(void *fixture)
{
	ARG_UNUSED(fixture);
	/* Reset to known state before each test. */
	int ret = <devname>_select_channel(dev, 0);

	zassert_ok(ret, "Failed to reset: %d", ret);
}

/* ---------- Test cases --------------------------------------------- */

ZTEST(misc_<devname>, test_device_ready)
{
	zassert_true(device_is_ready(dev), "Device should be ready");
}

ZTEST(misc_<devname>, test_initial_state)
{
	int ch = <devname>_get_channel(dev);

	zassert_equal(ch, 0, "Expected channel 0 after reset, got %d", ch);
}

ZTEST(misc_<devname>, test_select_channel)
{
	int ret = <devname>_select_channel(dev, 1);

	zassert_ok(ret, "Failed to select channel 1: %d", ret);
	zassert_equal(<devname>_get_channel(dev), 1,
		      "Channel mismatch after selecting 1");
}

ZTEST(misc_<devname>, test_select_all_channels)
{
	/* Iterate over all valid channels. Adjust max for device. */
	for (uint8_t ch = 1; ch <= 4; ch++) {
		int ret = <devname>_select_channel(dev, ch);

		zassert_ok(ret, "Failed to select ch %u: %d", ch, ret);
		zassert_equal(<devname>_get_channel(dev), ch,
			      "Mismatch for ch %u", ch);
	}
}

ZTEST(misc_<devname>, test_disable)
{
	int ret;

	ret = <devname>_select_channel(dev, 2);
	zassert_ok(ret);

	ret = <devname>_enable(dev, false);
	zassert_ok(ret, "Failed to disable: %d", ret);

	ret = <devname>_enable(dev, true);
	zassert_ok(ret, "Failed to re-enable: %d", ret);
}

ZTEST(misc_<devname>, test_invalid_channel)
{
	/* Device-specific: adjust the out-of-range value. */
	int ret = <devname>_select_channel(dev, 255);

	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for invalid channel, got %d", ret);
}

/* ---------- Suite registration ------------------------------------- */

ZTEST_SUITE(misc_<devname>, NULL, test_setup, test_before, NULL, NULL);
```

---

## 10. Key Conventions

1. **Custom API with `__subsystem` + `__syscall`** -- misc drivers define
   their own API struct annotated with `__subsystem`. Public functions use
   `__syscall` so that user-mode wrappers are generated when
   `CONFIG_USERSPACE` is enabled. Each `__syscall` function has a
   corresponding `z_impl_` static inline that dispatches through the API
   function pointer table.

2. **`DEVICE_DT_INST_DEFINE()` with `DEVICE_API()`** -- the instantiation
   macro passes `DEVICE_API_GET(misc)` (or the custom API instance
   pointer) as the last argument. This replaces the `NULL` API pointer
   used by older drivers that exposed functions directly without an API
   struct.

3. **Custom API naming** -- name the API struct `<devname>_api` and the
   callback types `<devname>_<operation>_t`. The public functions are
   named `<devname>_<operation>()`. This keeps the API discoverable and
   consistent with the Zephyr pattern used by subsystem drivers.

4. **When to use misc vs. a new subsystem** -- use `drivers/misc/` when
   only one or two devices share the same API pattern. If three or more
   device families converge on the same API (e.g., multiple analog mux
   vendors), consider proposing a formal Zephyr subsystem with a shared
   API header under `include/zephyr/drivers/`.

5. **Devicetree-driven** -- all hardware configuration (GPIO ports/pins,
   SPI/I2C bus specs, device-specific properties) comes from the
   devicetree. No hardcoded pin assignments in the driver source.

6. **GPIO-only vs. bus-based** -- GPIO-only devices (mux, switch, mcs)
   include `base.yaml` in the binding and use `GPIO_DT_SPEC_INST_GET()`.
   SPI devices include `spi-device.yaml` and use `SPI_DT_SPEC_INST_GET()`.
   I2C devices include `i2c-device.yaml` and use `I2C_DT_SPEC_INST_GET()`.

7. **`DT_DRV_COMPAT`** -- define `DT_DRV_COMPAT <vendor>_<devname>`
   (underscores, not commas or hyphens) at the top of every `.c` file that
   uses `DT_INST_*` macros. Must match the compatible string with dots and
   hyphens replaced by underscores.

8. **Init level** -- `POST_KERNEL` for all misc drivers. GPIO-only devices
   use `CONFIG_GPIO_INIT_PRIORITY` (typically 40-50). SPI/I2C devices use
   a custom `CONFIG_<DEVNAME>_INIT_PRIORITY` (default 80) to ensure the
   bus controller is initialized first.

9. **Thread safety** -- SPI and I2C devices must use `struct k_mutex` to
   serialize bus transactions. GPIO-only devices do not require a mutex
   unless accessed from multiple threads. If a device has an interrupt
   handler, use `struct k_sem` to signal from ISR context to a worker
   thread -- never perform bus transactions from ISR context.

10. **No dynamic allocation** -- all driver memory is statically allocated
    via `DT_INST_FOREACH_STATUS_OKAY()`. No `malloc`, `calloc`, `k_malloc`,
    or `k_free` in drivers.

11. **Config is const** -- `struct <devname>_config` is `static const`
    (ROM). Only `struct <devname>_data` is mutable (RAM).

12. **SPDX license** -- use `SPDX-License-Identifier: Apache-2.0` (Zephyr
    convention) in all files, not the BSD-3-Clause text block used in no-OS.

13. **Error codes** -- return standard Zephyr/POSIX negative error codes:
    `-EINVAL` for bad parameters, `-ENODEV` for hardware not ready,
    `-EIO` for bus failures, `-ENOSYS` for unimplemented API callbacks,
    `-ETIMEDOUT` for timeout conditions.

14. **Logging** -- use `LOG_MODULE_REGISTER(<devname>, CONFIG_<X>_LOG_LEVEL)`
    where `<X>` is `GPIO` for GPIO-only devices or a custom Kconfig symbol
    for bus-based devices. Use `LOG_ERR`, `LOG_WRN`, `LOG_INF`, `LOG_DBG` --
    never `printk` in drivers.

15. **Coding style** -- Zephyr coding style: tabs for indentation, opening
    brace on same line for functions, K&R braces, 100-column line limit.

---

## 11. Commit Message Format

All misc device drivers use the `drivers: misc:` prefix.

### Adding a new driver

```
drivers: misc: <devname>: add Zephyr driver for <DEVNAME>

Add a custom Zephyr driver for the <DEVNAME> <device-type>. The driver
communicates via <GPIO/SPI/I2C> and implements a custom API using
__subsystem and __syscall for <brief functionality description>.
Uses DEVICE_DT_INST_DEFINE() with DEVICE_API() for devicetree-driven
instantiation.

Signed-off-by: Your Name <your.name@analog.com>
```

### Adding the devicetree binding

```
dts: bindings: add binding for <vendor>,<devname>

Add a devicetree binding for the <DEVNAME> <device-type> with
<GPIO/SPI/I2C> interface properties.

Signed-off-by: Your Name <your.name@analog.com>
```

### Adding tests

```
tests: drivers: misc: add tests for <DEVNAME>

Add a ztest-based test suite for the <DEVNAME> <device-type> driver
covering <brief list of what is tested>.

Signed-off-by: Your Name <your.name@analog.com>
```

### Adding a sample application

```
samples: <devname>: add sample application

Add a sample application demonstrating <device operation> on the
<DEVNAME> <device-type>.

Signed-off-by: Your Name <your.name@analog.com>
```
