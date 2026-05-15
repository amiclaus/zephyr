# Zephyr GPIO Driver Template (Consolidated)

This template covers every file needed to add a new GPIO-class driver
to the Zephyr RTOS for an Analog Devices part that maps to the GPIO
subsystem. It consolidates patterns for both **I2C GPIO expanders**
(e.g., ADP5589) and **SPI digital I/O devices** (e.g., MAX14906).

Replace `<devname>` with the part number in lowercase (e.g., `adp5589`),
`<DEVNAME>` with uppercase (e.g., `ADP5589`), and `<devnum>` with the
numeric portion (e.g., `5589`) throughout.

---

## 1. Purpose & Subsystem Mapping

This driver maps to the Zephyr **GPIO subsystem** defined in
`include/zephyr/drivers/gpio.h`. The subsystem provides a unified API
that applications use identically for on-chip GPIOs, off-chip I2C GPIO
expanders, and SPI-attached digital I/O devices. The driver registers
itself as a GPIO controller (provider) through the `gpio_driver_api`.

### Device categories covered

| Category | Bus | Examples | Typical pin count |
|----------|-----|----------|-------------------|
| I2C GPIO expander | I2C | ADP5589, ADP5585 | 8-19 |
| SPI digital I/O | SPI | MAX14906, MAX22915 | 4-8 |

### Zephyr GPIO API mapping

| Zephyr GPIO API Function         | What it does                                    |
|----------------------------------|-------------------------------------------------|
| `gpio_pin_configure()`           | Configure a pin as input, output, or disconnected |
| `gpio_pin_set()`                 | Set the output level of a pin                   |
| `gpio_pin_get()`                 | Read the current level of a pin                 |
| `gpio_pin_toggle()`              | Toggle the output level of a pin                |
| `gpio_port_get_raw()`            | Read all pins of a port simultaneously          |
| `gpio_port_set_masked_raw()`     | Set multiple output pins at once                |
| `gpio_port_set_bits_raw()`       | Set selected output pins high                   |
| `gpio_port_clear_bits_raw()`     | Set selected output pins low                    |
| `gpio_port_toggle_bits()`        | Toggle selected output pins                     |
| `gpio_pin_interrupt_configure()` | Configure edge/level interrupt on a pin         |
| `gpio_manage_callback()`         | Register/unregister an interrupt callback       |

The driver implements the `gpio_driver_api` struct (via the
`DEVICE_API(gpio, ...)` macro) which contains pointers to the driver's
pin configuration, port I/O, and interrupt management implementations.

All configuration comes from **devicetree** at compile time. There is no
dynamic allocation -- the config struct is `const` and populated from DT
macros, while the data struct is static and holds mutable runtime state.

**GPIO controller vs. GPIO user:** These drivers are GPIO *controllers*
(providers). They register a `gpio_driver_api` implementation.
Application code then uses the standard `gpio_pin_*` API functions
which the GPIO subsystem dispatches to the driver.

---

## 2. File Checklist

```
zephyr/
    drivers/gpio/
        gpio_<devname>.c          # Driver implementation
        Kconfig.<devname>         # Kconfig fragment
        CMakeLists.txt            # (append to existing)
        Kconfig                   # (append to existing)

    dts/bindings/gpio/
        adi,<devname>.yaml        # Devicetree binding

    tests/drivers/gpio/<devname>/
        testcase.yaml             # Test metadata
        prj.conf                  # Test project config
        boards/native_sim.overlay # DT overlay for test
        src/main.c                # Test source
```

---

## 3. Devicetree Binding (`dts/bindings/gpio/adi,<devname>.yaml`)

### 3.1 I2C bus variant (e.g., ADP5589 I/O expander)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Analog Devices <DEVNAME> configurable I/O expander with up to
  <n>-pin GPIO. Communicates via I2C.

  Example devicetree node:

    &i2c0 {
        status = "okay";

        <devname>: <devname>@34 {
            compatible = "adi,<devname>";
            reg = <0x34>;
            gpio-controller;
            #gpio-cells = <2>;
            ngpios = <19>;

            /* Optional: interrupt output from the device */
            /* int-gpios = <&gpiob 5 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>; */
        };
    };

compatible: "adi,<devname>"

include: [i2c-device.yaml, gpio-controller.yaml]

properties:
  "#gpio-cells":
    const: 2

  ngpios:
    type: int
    required: true
    description: |
      Number of GPIO pins exposed by this device.
      For <DEVNAME> this is typically <n>.

  int-gpios:
    type: phandle-array
    description: |
      GPIO connected to the INT output pin of the device.
      This is optional; when omitted, interrupt-driven
      notification is not available.
```

### 3.2 SPI bus variant (e.g., MAX14906 digital I/O)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Analog Devices <DEVNAME> <channels>-channel digital I/O
  with SPI interface.

  Example devicetree node:

    &spi1 {
        status = "okay";
        cs-gpios = <&gpioa 4 GPIO_ACTIVE_LOW>;

        <devname>0: <devname>@0 {
            compatible = "adi,<devname>";
            reg = <0>;
            spi-max-frequency = <10000000>;
            gpio-controller;
            #gpio-cells = <2>;
            ngpios = <8>;

            /* Optional fault / control pins */
            fault-gpios  = <&gpiob 0 GPIO_ACTIVE_LOW>;
            enable-gpios = <&gpiob 1 GPIO_ACTIVE_HIGH>;
            reset-gpios  = <&gpiob 2 GPIO_ACTIVE_LOW>;
        };
    };

compatible: "adi,<devname>"

include: [spi-device.yaml, gpio-controller.yaml, base.yaml]

properties:
  "#gpio-cells":
    const: 2

  ngpios:
    type: int
    default: 4
    description: |
      Number of I/O channels exposed by this device instance.
      Typical values: 4 or 8 depending on the part number.

  fault-gpios:
    type: phandle-array
    description: |
      GPIO connected to the device FAULT output pin (active-low).
      Used for interrupt-driven fault notification. Optional;
      when omitted, faults are detected by polling status registers.

  enable-gpios:
    type: phandle-array
    description: |
      GPIO connected to the device EN (enable) pin. Optional.
      When present, the driver asserts this pin during initialization.

  reset-gpios:
    type: phandle-array
    description: |
      GPIO connected to the device RESET pin (active-low). Optional.
      When present, the driver pulses this pin during initialization.
```

### Why `gpio-controller.yaml`?

Including `gpio-controller.yaml` marks this node as a GPIO provider in
devicetree. This lets other nodes reference its pins with the phandle
syntax (e.g., `gpios = <&<devname> 3 GPIO_ACTIVE_HIGH>`). The
`#gpio-cells = <2>` means each reference takes two cells: pin number
and flags.

The `ngpios` property tells the GPIO subsystem how many pins the
controller provides. It is used by `GPIO_PORT_PIN_MASK_FROM_DT_INST()`
to generate the valid pin bitmask.

---

## 4. Kconfig (`drivers/gpio/Kconfig.<devname>`)

### I2C device

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config GPIO_<DEVNAME>
	bool "<DEVNAME> I/O expander GPIO driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	select I2C
	help
	  Enable support for the Analog Devices <DEVNAME>
	  configurable I/O expander with up to <n> GPIO pins,
	  accessed over I2C.

config GPIO_<DEVNAME>_INIT_PRIORITY
	int "<DEVNAME> init priority"
	default 75
	depends on GPIO_<DEVNAME>
	help
	  Device driver initialization priority for the <DEVNAME>.
	  Must be higher than the I2C bus driver priority (so the
	  bus is ready first).

config GPIO_<DEVNAME>_INTERRUPT
	bool "<DEVNAME> interrupt support"
	default y
	depends on GPIO_<DEVNAME>
	help
	  Enable interrupt support for the <DEVNAME> I/O expander.
	  When enabled, the driver configures the INT output pin
	  and dispatches GPIO callbacks on pin state changes.
```

### SPI device

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config GPIO_<DEVNAME>
	bool "<DEVNAME> digital I/O driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	select SPI if $(dt_compat_on_bus,$(DT_COMPAT_ADI_<DEVNAME>),spi)
	select GPIO_ENABLE_DISABLE_INTERRUPT
	help
	  Enable support for the Analog Devices <DEVNAME> configurable
	  digital I/O device.

	  The driver exposes each channel as a GPIO pin through the
	  standard Zephyr GPIO subsystem API.

config GPIO_<DEVNAME>_INIT_PRIORITY
	int "<DEVNAME> init priority"
	default 75
	depends on GPIO_<DEVNAME>
	help
	  Device driver initialization priority for the <DEVNAME>.
	  Must be higher than the SPI bus driver priority.
```

Then add to the parent `drivers/gpio/Kconfig`:

```kconfig
source "drivers/gpio/Kconfig.<devname>"
```

---

## 5. CMakeLists.txt (Build System Integration)

Append to the existing `drivers/gpio/CMakeLists.txt`:

```cmake
zephyr_library_sources_ifdef(CONFIG_GPIO_<DEVNAME> gpio_<devname>.c)
```

---

## 6. Driver Header (Optional)

If the driver is self-contained, no separate header is needed. However,
if register definitions must be shared with an emulator or test, create
`include/zephyr/drivers/gpio/<devname>.h`:

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GPIO_<DEVNAME>_H_
#define ZEPHYR_INCLUDE_DRIVERS_GPIO_<DEVNAME>_H_

/* ---- Config struct (compile-time, from devicetree) --------------- */

struct <devname>_config {
	/* Must be first -- GPIO subsystem casts through this */
	struct gpio_driver_config common;

	/* === I2C variant === */
	struct i2c_dt_spec i2c;

	/* === SPI variant === */
	/* struct spi_dt_spec spi; */

	/* Optional interrupt / control GPIOs */
	struct gpio_dt_spec int_gpio;
};

/* ---- Data struct (mutable runtime state) ------------------------- */

struct <devname>_data {
	/* Must be first -- GPIO subsystem casts through this */
	struct gpio_driver_data common;

	const struct device *dev;
	struct k_mutex lock;

	/* Cached register state */
	uint8_t dir[BANK_COUNT];       /* Direction: 1=output */
	uint8_t output[BANK_COUNT];    /* Output data latch */

	/* Interrupt support */
	struct gpio_callback int_cb;
	sys_slist_t callbacks;         /* Application callbacks */
	struct k_work int_work;
};

/* ---- API (gpio_driver_api) --------------------------------------- */

/*
 * The API struct is declared via the DEVICE_API(gpio, ...) macro.
 * It implements all gpio_driver_api callbacks:
 *   .pin_configure
 *   .port_get_raw
 *   .port_set_masked_raw
 *   .port_set_bits_raw
 *   .port_clear_bits_raw
 *   .port_toggle_bits
 *   .pin_interrupt_configure  (optional)
 *   .manage_callback          (optional)
 */

/* ---- Register Map (adapt to specific part) ----------------------- */

/* I2C expander example */
#define <DEVNAME>_REG_ID		0x00
#define <DEVNAME>_REG_GPI_STATUS_A	0x16
#define <DEVNAME>_REG_GPO_DATA_A	0x2A
#define <DEVNAME>_REG_GPIO_DIR_A	0x30

/* SPI digital I/O example */
/* #define <DEVNAME>_REG_SET_OUTPUT	0x00 */
/* #define <DEVNAME>_REG_SET_INPUT	0x01 */
/* #define <DEVNAME>_REG_CONFIG		0x02 */
/* #define <DEVNAME>_REG_FAULT		0x04 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_GPIO_<DEVNAME>_H_ */
```

---

## 7. Driver Source (`drivers/gpio/gpio_<devname>.c`)

This is the core of the driver. It follows the Zephyr GPIO subsystem
contract and Linux kernel coding style. The template below shows the
full `gpio_driver_api` implementation with both I2C and SPI register
access patterns annotated side by side.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file gpio_<devname>.c
 * @brief GPIO driver for the Analog Devices <DEVNAME>.
 *
 * This driver implements the Zephyr GPIO subsystem API for the
 * <DEVNAME> device, which is accessed over [I2C / SPI].
 */

#define DT_DRV_COMPAT adi_<devname>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

/* === Include the bus header for your variant === */
#include <zephyr/drivers/i2c.h>   /* I2C variant */
/* #include <zephyr/drivers/spi.h> */  /* SPI variant */

LOG_MODULE_REGISTER(gpio_<devname>, CONFIG_GPIO_LOG_LEVEL);

/* ---------------- Register Map ------------------------------------ */
/*
 * Adapt from the device datasheet or no-OS driver header.
 * I2C expanders typically use single-byte register addresses.
 * SPI devices typically pack address + R/W into a command byte.
 */

/* I2C expander registers (example: multi-bank, 8 bits per bank) */
#define <DEVNAME>_REG_ID		0x00
#define <DEVNAME>_REG_INT_STATUS	0x01
#define <DEVNAME>_REG_GPI_INT_STAT_A	0x13
#define <DEVNAME>_REG_GPI_STATUS_A	0x16
#define <DEVNAME>_REG_GPI_INT_EN_A	0x24
#define <DEVNAME>_REG_GPO_DATA_A	0x2A
#define <DEVNAME>_REG_GPO_OUT_MODE_A	0x2D
#define <DEVNAME>_REG_GPIO_DIR_A	0x30
#define <DEVNAME>_REG_RPULL_CFG_A	0x19
#define <DEVNAME>_REG_GENERAL_CFG	0x4D
#define <DEVNAME>_REG_INT_EN		0x4E

/* SPI digital I/O registers (example: single-byte address, 8-bit data) */
/* #define <DEVNAME>_REG_SET_OUTPUT	0x00 */
/* #define <DEVNAME>_REG_SET_INPUT	0x01 */
/* #define <DEVNAME>_REG_CONFIG		0x02 */
/* #define <DEVNAME>_REG_INTERRUPT	0x03 */
/* #define <DEVNAME>_REG_FAULT		0x04 */
/* #define <DEVNAME>_REG_FAULT_MASK	0x05 */
/* #define <DEVNAME>_REG_STATUS		0x06 */

/* Field masks */
#define <DEVNAME>_MAN_ID_MASK		GENMASK(7, 4)
#define <DEVNAME>_DEVICE_ID		0x10
#define <DEVNAME>_INT_CFG_MASK		BIT(1)
#define <DEVNAME>_GPI_IEN_MASK		BIT(1)
#define <DEVNAME>_OSC_EN_MASK		BIT(7)

/* GPIO pin banking (I2C multi-bank devices) */
#define <DEVNAME>_BANK_COUNT		3

/* SPI command framing */
/* #define <DEVNAME>_ADDR_MASK		GENMASK(5, 1) */
/* #define <DEVNAME>_RW_MASK		BIT(0) */
/* #define <DEVNAME>_SPI_FRAME_SIZE	2 */

/* ---------------- Config & Data Structs --------------------------- */

/**
 * Compile-time configuration from devicetree.
 * This struct is const and stored in ROM.
 *
 * gpio_driver_config is the common base that holds the port_pin_mask
 * (populated from the "ngpios" DT property).
 */
struct <devname>_config {
	struct gpio_driver_config common;   /* Must be first field */

	/* === I2C variant === */
	struct i2c_dt_spec i2c;

	/* === SPI variant === */
	/* struct spi_dt_spec spi; */

	/* Optional interrupt / control GPIOs */
	struct gpio_dt_spec int_gpio;       /* INT output (I2C) */
	/* struct gpio_dt_spec fault_gpio; */  /* FAULT output (SPI) */
	/* struct gpio_dt_spec enable_gpio; */ /* EN pin (SPI) */
	/* struct gpio_dt_spec reset_gpio; */  /* RESET pin (SPI) */
};

/**
 * Mutable runtime data.
 * This struct is allocated statically per instance.
 *
 * gpio_driver_data is the common base that holds the
 * invert mask applied by the GPIO subsystem.
 */
struct <devname>_data {
	struct gpio_driver_data common;   /* Must be first field */

	const struct device *dev;
	struct k_mutex lock;

	/* === I2C variant: multi-bank cached state === */
	uint8_t dir[<DEVNAME>_BANK_COUNT];	/* Direction: 1=output */
	uint8_t output[<DEVNAME>_BANK_COUNT];	/* Output data latch */
	uint8_t pull_cfg[<DEVNAME>_BANK_COUNT];	/* Pull resistor config */

	/* === SPI variant: single-register cached state === */
	/* uint8_t output_state; */
	/* uint8_t direction; */
	/* uint8_t config_cache; */
	/* uint8_t buf[<DEVNAME>_SPI_FRAME_SIZE + 1]; */

	/* Interrupt support */
#ifdef CONFIG_GPIO_<DEVNAME>_INTERRUPT
	struct gpio_callback int_cb;
	sys_slist_t callbacks;		/* Application callbacks */
	uint8_t int_en[<DEVNAME>_BANK_COUNT];	/* Interrupt enable mask */
	struct k_work int_work;
#endif
};

/* ---------------- Bus Access Helpers ------------------------------ */

/*
 * Choose ONE of the following sections based on bus type.
 * Both are shown here for reference.
 */

/* ===== I2C register access ===== */

/**
 * @brief Read a single register over I2C.
 */
static int <devname>_reg_read(const struct device *dev, uint8_t addr,
			      uint8_t *val)
{
	const struct <devname>_config *cfg = dev->config;

	return i2c_reg_read_byte_dt(&cfg->i2c, addr, val);
}

/**
 * @brief Write a single register over I2C.
 */
static int <devname>_reg_write(const struct device *dev, uint8_t addr,
			       uint8_t val)
{
	const struct <devname>_config *cfg = dev->config;

	return i2c_reg_write_byte_dt(&cfg->i2c, addr, val);
}

/**
 * @brief Update specific bits in a register (read-modify-write) over I2C.
 */
static int <devname>_reg_update(const struct device *dev, uint8_t addr,
				uint8_t mask, uint8_t val)
{
	const struct <devname>_config *cfg = dev->config;

	return i2c_reg_update_byte_dt(&cfg->i2c, addr, mask, val);
}

/**
 * @brief Read a block of consecutive registers over I2C.
 *
 * Used to read all GPIO status banks in a single burst.
 */
static int <devname>_reg_burst_read(const struct device *dev,
				    uint8_t start_addr,
				    uint8_t *buf, uint8_t len)
{
	const struct <devname>_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->i2c, start_addr, buf, len);
}

/* ===== SPI register access ===== */

#if 0 /* Enable for SPI variant */
/**
 * @brief Read a register over SPI.
 *
 * SPI frame: [ADDR(7:1) | RW(0)] [DATA(7:0)]
 * The address and R/W bit are packed into the first byte.
 */
static int <devname>_reg_read(const struct device *dev, uint8_t addr,
			      uint8_t *val)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	data->buf[0] = FIELD_PREP(<DEVNAME>_ADDR_MASK, addr) |
		       <DEVNAME>_RW_MASK;
	data->buf[1] = 0x00;

	const struct spi_buf tx = {
		.buf = data->buf,
		.len = <DEVNAME>_SPI_FRAME_SIZE,
	};
	const struct spi_buf rx = {
		.buf = data->buf,
		.len = <DEVNAME>_SPI_FRAME_SIZE,
	};
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI read failed (reg 0x%02X): %d", addr, ret);
		return ret;
	}

	*val = data->buf[1];
	return 0;
}

/**
 * @brief Write a register over SPI.
 */
static int <devname>_reg_write(const struct device *dev, uint8_t addr,
			       uint8_t val)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;

	data->buf[0] = FIELD_PREP(<DEVNAME>_ADDR_MASK, addr);
	data->buf[1] = val;

	const struct spi_buf tx = {
		.buf = data->buf,
		.len = <DEVNAME>_SPI_FRAME_SIZE,
	};
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	return spi_write_dt(&cfg->spi, &tx_set);
}

/**
 * @brief Update specific bits in a register (read-modify-write) over SPI.
 */
static int <devname>_reg_update(const struct device *dev, uint8_t addr,
				uint8_t mask, uint8_t val)
{
	uint8_t reg_val;
	int ret;

	ret = <devname>_reg_read(dev, addr, &reg_val);
	if (ret < 0) {
		return ret;
	}

	reg_val = (reg_val & ~mask) | (val & mask);

	return <devname>_reg_write(dev, addr, reg_val);
}
#endif /* SPI variant */

/* ---------------- GPIO Subsystem Callbacks ------------------------ */

/**
 * @brief Configure a single GPIO pin.
 *
 * Called by the application via gpio_pin_configure(). Sets up the pin
 * direction, output drive mode, and pull resistor configuration.
 *
 * @param dev   Device instance.
 * @param pin   Pin number (0 to ngpios-1).
 * @param flags GPIO configuration flags (GPIO_INPUT, GPIO_OUTPUT, etc.).
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_pin_configure(const struct device *dev,
				   gpio_pin_t pin,
				   gpio_flags_t flags)
{
	struct <devname>_data *data = dev->data;
	uint8_t bank = pin / 8;
	uint8_t bit = BIT(pin % 8);
	int ret;

	if (bank >= <DEVNAME>_BANK_COUNT) {
		return -EINVAL;
	}

	/* Unsupported flag combinations */
	if ((flags & GPIO_SINGLE_ENDED) != 0 &&
	    (flags & GPIO_LINE_OPEN_DRAIN) == 0) {
		/* Open-source not supported by this hardware */
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Configure direction */
	if (flags & GPIO_OUTPUT) {
		data->dir[bank] |= bit;

		/* Set initial output value if specified */
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			data->output[bank] |= bit;
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			data->output[bank] &= ~bit;
		}

		/* Write output data first, then set direction to output */
		ret = <devname>_reg_write(dev,
					  <DEVNAME>_REG_GPO_DATA_A + bank,
					  data->output[bank]);
		if (ret < 0) {
			goto unlock;
		}

		ret = <devname>_reg_write(dev,
					  <DEVNAME>_REG_GPIO_DIR_A + bank,
					  data->dir[bank]);
		if (ret < 0) {
			goto unlock;
		}

		/* Configure open-drain if requested */
		if (flags & GPIO_OPEN_DRAIN) {
			ret = <devname>_reg_update(
				dev,
				<DEVNAME>_REG_GPO_OUT_MODE_A + bank,
				bit, 0);  /* 0 = open-drain */
		} else {
			ret = <devname>_reg_update(
				dev,
				<DEVNAME>_REG_GPO_OUT_MODE_A + bank,
				bit, bit);  /* 1 = push-pull */
		}
		if (ret < 0) {
			goto unlock;
		}
	} else if (flags & GPIO_INPUT) {
		data->dir[bank] &= ~bit;

		ret = <devname>_reg_write(dev,
					  <DEVNAME>_REG_GPIO_DIR_A + bank,
					  data->dir[bank]);
		if (ret < 0) {
			goto unlock;
		}
	} else {
		/* GPIO_DISCONNECTED -- set as input, disable pull */
		data->dir[bank] &= ~bit;

		ret = <devname>_reg_write(dev,
					  <DEVNAME>_REG_GPIO_DIR_A + bank,
					  data->dir[bank]);
		if (ret < 0) {
			goto unlock;
		}
	}

	/* Configure pull-up / pull-down resistors */
	if (flags & GPIO_PULL_UP) {
		ret = <devname>_reg_update(dev,
					   <DEVNAME>_REG_RPULL_CFG_A + bank,
					   bit, bit);
	} else if (flags & GPIO_PULL_DOWN) {
		ret = <devname>_reg_update(dev,
					   <DEVNAME>_REG_RPULL_CFG_A + bank,
					   bit, 0);
	} else {
		/* Disable pull resistor */
		ret = <devname>_reg_update(dev,
					   <DEVNAME>_REG_RPULL_CFG_A + bank,
					   bit, 0);
	}

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

/*
 * SPI variant pin_configure is simpler -- single register for direction,
 * no pull-up support, channel mask instead of bank/bit:
 *
 * static int <devname>_pin_configure(const struct device *dev,
 *                                    gpio_pin_t pin, gpio_flags_t flags)
 * {
 *     struct <devname>_data *data = dev->data;
 *
 *     if (pin >= cfg->ngpios)
 *         return -EINVAL;
 *     if (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN))
 *         return -ENOTSUP;
 *
 *     k_mutex_lock(&data->lock, K_FOREVER);
 *     if (flags & GPIO_OUTPUT) {
 *         <devname>_reg_update(dev, CONFIG_REG, CH_MASK(pin), CH_MASK(pin));
 *         data->direction |= BIT(pin);
 *         if (flags & GPIO_OUTPUT_INIT_HIGH)
 *             data->output_state |= BIT(pin);
 *         else if (flags & GPIO_OUTPUT_INIT_LOW)
 *             data->output_state &= ~BIT(pin);
 *         <devname>_reg_write(dev, SET_OUTPUT_REG, data->output_state);
 *     } else {
 *         <devname>_reg_update(dev, CONFIG_REG, CH_MASK(pin), 0);
 *         data->direction &= ~BIT(pin);
 *     }
 *     k_mutex_unlock(&data->lock);
 *     return ret;
 * }
 */

/**
 * @brief Read the raw input state of all pins in the port.
 *
 * Called by the GPIO subsystem for gpio_port_get_raw().
 * Returns a bitmask where bit N corresponds to pin N.
 *
 * @param dev   Device instance.
 * @param value Pointer to store the port value.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_port_get_raw(const struct device *dev,
				  gpio_port_value_t *value)
{
	struct <devname>_data *data = dev->data;
	uint8_t regs[<DEVNAME>_BANK_COUNT];
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	/* I2C: burst-read all status banks */
	ret = <devname>_reg_burst_read(dev, <DEVNAME>_REG_GPI_STATUS_A,
				       regs, <DEVNAME>_BANK_COUNT);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	/*
	 * Combine the three 8-bit banks into a single 32-bit value.
	 * Bank A = bits [7:0], Bank B = bits [15:8], Bank C = bits [23:16].
	 */
	*value = (uint32_t)regs[0] |
		 ((uint32_t)regs[1] << 8) |
		 ((uint32_t)regs[2] << 16);

	k_mutex_unlock(&data->lock);
	return 0;
}

/*
 * SPI variant port_get_raw reads a single register and merges with
 * cached output state:
 *
 * static int <devname>_port_get_raw(const struct device *dev,
 *                                   gpio_port_value_t *value)
 * {
 *     struct <devname>_data *data = dev->data;
 *     uint8_t input_val;
 *
 *     k_mutex_lock(&data->lock, K_FOREVER);
 *     ret = <devname>_reg_read(dev, SET_INPUT_REG, &input_val);
 *     *value = (input_val & ~data->direction) |
 *              (data->output_state & data->direction);
 *     k_mutex_unlock(&data->lock);
 *     return ret;
 * }
 */

/**
 * @brief Set output pins according to a mask.
 *
 * Called by the GPIO subsystem for gpio_port_set_masked_raw().
 * Only pins whose corresponding bit in @p mask is set are modified.
 *
 * @param dev   Device instance.
 * @param mask  Bitmask of pins to modify.
 * @param value Desired output levels for the masked pins.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_port_set_masked_raw(const struct device *dev,
					 gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	struct <devname>_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	/* I2C: iterate over register banks */
	for (uint8_t bank = 0; bank < <DEVNAME>_BANK_COUNT; bank++) {
		uint8_t bank_mask = (mask >> (bank * 8)) & 0xFF;
		uint8_t bank_val = (value >> (bank * 8)) & 0xFF;

		if (bank_mask == 0) {
			continue;
		}

		data->output[bank] = (data->output[bank] & ~bank_mask) |
				     (bank_val & bank_mask);

		ret = <devname>_reg_write(dev,
					  <DEVNAME>_REG_GPO_DATA_A + bank,
					  data->output[bank]);
		if (ret < 0) {
			break;
		}
	}

	/*
	 * SPI variant: single register write:
	 *   data->output_state = (data->output_state & ~mask) |
	 *                        (value & mask);
	 *   ret = <devname>_reg_write(dev, SET_OUTPUT_REG,
	 *                             data->output_state);
	 */

	k_mutex_unlock(&data->lock);
	return ret;
}

/**
 * @brief Set output pins high.
 *
 * Called by the GPIO subsystem for gpio_port_set_bits_raw().
 *
 * @param dev  Device instance.
 * @param pins Bitmask of pins to set high.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_port_set_bits_raw(const struct device *dev,
				       gpio_port_pins_t pins)
{
	return <devname>_port_set_masked_raw(dev, pins, pins);
}

/**
 * @brief Set output pins low.
 *
 * Called by the GPIO subsystem for gpio_port_clear_bits_raw().
 *
 * @param dev  Device instance.
 * @param pins Bitmask of pins to set low.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_port_clear_bits_raw(const struct device *dev,
					 gpio_port_pins_t pins)
{
	return <devname>_port_set_masked_raw(dev, pins, 0);
}

/**
 * @brief Toggle output pins.
 *
 * Called by the GPIO subsystem for gpio_port_toggle_bits().
 *
 * @param dev  Device instance.
 * @param pins Bitmask of pins to toggle.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_port_toggle_bits(const struct device *dev,
				      gpio_port_pins_t pins)
{
	struct <devname>_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	/* I2C: iterate over register banks */
	for (uint8_t bank = 0; bank < <DEVNAME>_BANK_COUNT; bank++) {
		uint8_t bank_pins = (pins >> (bank * 8)) & 0xFF;

		if (bank_pins == 0) {
			continue;
		}

		data->output[bank] ^= bank_pins;

		ret = <devname>_reg_write(dev,
					  <DEVNAME>_REG_GPO_DATA_A + bank,
					  data->output[bank]);
		if (ret < 0) {
			break;
		}
	}

	/*
	 * SPI variant:
	 *   gpio_port_value_t toggled = data->output_state ^ pins;
	 *   return <devname>_port_set_masked_raw(dev, pins, toggled);
	 */

	k_mutex_unlock(&data->lock);
	return ret;
}

/* ---------------- Interrupt Support ------------------------------- */

#ifdef CONFIG_GPIO_<DEVNAME>_INTERRUPT

/**
 * @brief Configure interrupt for a single pin.
 *
 * Called by the GPIO subsystem for gpio_pin_interrupt_configure().
 * Enables or disables the interrupt for the specified pin and
 * configures the interrupt level/edge in the device registers.
 *
 * @param dev  Device instance.
 * @param pin  Pin number.
 * @param mode Interrupt mode (edge, level, or disabled).
 * @param trig Interrupt trigger (rising, falling, or both).
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_pin_interrupt_configure(const struct device *dev,
					     gpio_pin_t pin,
					     enum gpio_int_mode mode,
					     enum gpio_int_trig trig)
{
	struct <devname>_data *data = dev->data;
	uint8_t bank = pin / 8;
	uint8_t bit = BIT(pin % 8);
	int ret;

	if (bank >= <DEVNAME>_BANK_COUNT) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (mode == GPIO_INT_MODE_DISABLED) {
		data->int_en[bank] &= ~bit;
	} else {
		/*
		 * Edge vs level and rising vs falling configuration
		 * depends on the specific device registers -- adapt
		 * for your part's interrupt configuration registers.
		 */
		if (mode == GPIO_INT_MODE_LEVEL) {
			/*
			 * Level-triggered interrupts may or may not be
			 * supported. Return -ENOTSUP if not available.
			 */
		}
		data->int_en[bank] |= bit;
	}

	ret = <devname>_reg_write(dev,
				  <DEVNAME>_REG_GPI_INT_EN_A + bank,
				  data->int_en[bank]);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	/* Enable the global interrupt if any per-pin interrupt is active */
	uint32_t all_ints = (uint32_t)data->int_en[0] |
			    ((uint32_t)data->int_en[1] << 8) |
			    ((uint32_t)data->int_en[2] << 16);

	if (all_ints) {
		ret = <devname>_reg_update(dev, <DEVNAME>_REG_INT_EN,
					   <DEVNAME>_GPI_IEN_MASK,
					   <DEVNAME>_GPI_IEN_MASK);
	} else {
		ret = <devname>_reg_update(dev, <DEVNAME>_REG_INT_EN,
					   <DEVNAME>_GPI_IEN_MASK, 0);
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

/*
 * SPI variant pin_interrupt_configure uses a fault mask register:
 *
 * static int <devname>_pin_interrupt_configure(...)
 * {
 *     if (cfg->fault_gpio.port == NULL)
 *         return (mode == GPIO_INT_MODE_DISABLED) ? 0 : -ENOTSUP;
 *
 *     k_mutex_lock(&data->lock, K_FOREVER);
 *     if (mode == GPIO_INT_MODE_DISABLED)
 *         <devname>_reg_update(dev, FAULT_MASK_REG, CH_MASK(pin), 0);
 *     else
 *         <devname>_reg_update(dev, FAULT_MASK_REG, CH_MASK(pin),
 *                              CH_MASK(pin));
 *     k_mutex_unlock(&data->lock);
 *     return ret;
 * }
 */

/**
 * @brief Manage (add/remove) GPIO callbacks.
 *
 * Called by the GPIO subsystem for gpio_manage_callback().
 */
static int <devname>_manage_callback(const struct device *dev,
				     struct gpio_callback *callback,
				     bool set)
{
	struct <devname>_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

#endif /* CONFIG_GPIO_<DEVNAME>_INTERRUPT */

/* ---------------- Initialization ---------------------------------- */

/**
 * @brief Verify the device identity by reading the ID register.
 */
static int <devname>_verify_id(const struct device *dev)
{
	uint8_t id;
	int ret;

	ret = <devname>_reg_read(dev, <DEVNAME>_REG_ID, &id);
	if (ret < 0) {
		return ret;
	}

	if (FIELD_GET(<DEVNAME>_MAN_ID_MASK, id) != <DEVNAME>_DEVICE_ID) {
		LOG_ERR("Unexpected device ID: 0x%02X (expected 0x%02X)",
			FIELD_GET(<DEVNAME>_MAN_ID_MASK, id),
			<DEVNAME>_DEVICE_ID);
		return -ENODEV;
	}

	LOG_DBG("Device ID verified: 0x%02X", id);
	return 0;
}

/**
 * @brief Device initialization function.
 *
 * Called automatically at boot for each DT instance.
 * Verifies bus readiness, checks the device ID, and configures
 * all pins as inputs.
 */
static int <devname>_init(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	data->dev = dev;

	/* === I2C variant: verify bus ready === */
	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* === SPI variant: verify bus ready === */
	/* if (!spi_is_ready_dt(&cfg->spi)) {
	 *     LOG_ERR("SPI bus not ready");
	 *     return -ENODEV;
	 * }
	 */

	k_mutex_init(&data->lock);

	/* Verify device identity */
	ret = <devname>_verify_id(dev);
	if (ret < 0) {
		return ret;
	}

	/* Initialize all pins as inputs */
	for (uint8_t bank = 0; bank < <DEVNAME>_BANK_COUNT; bank++) {
		data->dir[bank] = 0x00;
		data->output[bank] = 0x00;

		ret = <devname>_reg_write(dev,
					  <DEVNAME>_REG_GPIO_DIR_A + bank,
					  0x00);
		if (ret < 0) {
			LOG_ERR("Failed to init direction bank %u: %d",
				bank, ret);
			return ret;
		}
	}

#ifdef CONFIG_GPIO_<DEVNAME>_INTERRUPT
	/* Initialize interrupt support (see Section 8 for details) */
	if (cfg->int_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->int_gpio)) {
			LOG_ERR("INT GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Failed to configure INT GPIO: %d", ret);
			return ret;
		}

		gpio_init_callback(&data->int_cb, <devname>_int_handler,
				   BIT(cfg->int_gpio.pin));
		ret = gpio_add_callback(cfg->int_gpio.port, &data->int_cb);
		if (ret < 0) {
			LOG_ERR("Failed to add INT callback: %d", ret);
			return ret;
		}

		k_work_init(&data->int_work, <devname>_int_work_handler);

		ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
						      GPIO_INT_EDGE_TO_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure INT interrupt: %d", ret);
			return ret;
		}
	}
#endif

	LOG_INF("<DEVNAME> initialized");
	return 0;
}

/* ---------------- API Struct -------------------------------------- */

/*
 * Use the DEVICE_API macro. This creates a static const struct that
 * the Zephyr GPIO subsystem uses to dispatch calls.
 *
 * The gpio_driver_api contains:
 *   - pin_configure     (required)
 *   - port_get_raw      (required for input)
 *   - port_set_masked_raw (required for output)
 *   - port_set_bits_raw (required for output)
 *   - port_clear_bits_raw (required for output)
 *   - port_toggle_bits  (required for output)
 *   - pin_interrupt_configure (optional, for interrupt support)
 *   - manage_callback   (optional, for interrupt support)
 */
static DEVICE_API(gpio, <devname>_api) = {
	.pin_configure = <devname>_pin_configure,
	.port_get_raw = <devname>_port_get_raw,
	.port_set_masked_raw = <devname>_port_set_masked_raw,
	.port_set_bits_raw = <devname>_port_set_bits_raw,
	.port_clear_bits_raw = <devname>_port_clear_bits_raw,
	.port_toggle_bits = <devname>_port_toggle_bits,
#ifdef CONFIG_GPIO_<DEVNAME>_INTERRUPT
	.pin_interrupt_configure = <devname>_pin_interrupt_configure,
	.manage_callback = <devname>_manage_callback,
#endif
};

/* ---------------- Instance Macros --------------------------------- */

/*
 * These macros are expanded once per DT instance with status "okay".
 * They create the config, data, and DEVICE_DT_INST_DEFINE entries.
 */

/* === I2C variant === */
#define <DEVNAME>_INT_GPIO_INIT(n)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, int_gpios),		\
		(.int_gpio = GPIO_DT_SPEC_INST_GET(n, int_gpios),),	\
		(.int_gpio = { 0 },))

#define <DEVNAME>_INIT(n)						\
	static struct <devname>_data <devname>_data_##n;		\
									\
	static const struct <devname>_config <devname>_config_##n = {	\
		.common = {						\
			.port_pin_mask =				\
				GPIO_PORT_PIN_MASK_FROM_DT_INST(n),	\
		},							\
		.i2c = I2C_DT_SPEC_INST_GET(n),				\
		<DEVNAME>_INT_GPIO_INIT(n)				\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, <devname>_init, NULL,			\
			      &<devname>_data_##n,			\
			      &<devname>_config_##n,			\
			      POST_KERNEL,				\
			      CONFIG_GPIO_<DEVNAME>_INIT_PRIORITY,	\
			      &<devname>_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_INIT)

/*
 * === SPI variant (replace the I2C block above) ===
 *
 * #define <DEVNAME>_FAULT_GPIO_INIT(n)				\
 *     COND_CODE_1(DT_INST_NODE_HAS_PROP(n, fault_gpios),		\
 *         (.fault_gpio = GPIO_DT_SPEC_INST_GET(n, fault_gpios),),	\
 *         (.fault_gpio = { 0 },))
 *
 * #define <DEVNAME>_ENABLE_GPIO_INIT(n)				\
 *     COND_CODE_1(DT_INST_NODE_HAS_PROP(n, enable_gpios),		\
 *         (.enable_gpio = GPIO_DT_SPEC_INST_GET(n, enable_gpios),),\
 *         (.enable_gpio = { 0 },))
 *
 * #define <DEVNAME>_RESET_GPIO_INIT(n)				\
 *     COND_CODE_1(DT_INST_NODE_HAS_PROP(n, reset_gpios),		\
 *         (.reset_gpio = GPIO_DT_SPEC_INST_GET(n, reset_gpios),),	\
 *         (.reset_gpio = { 0 },))
 *
 * #define <DEVNAME>_INIT(n)						\
 *     static struct <devname>_data <devname>_data_##n;		\
 *     static const struct <devname>_config <devname>_config_##n = {\
 *         .common = {						\
 *             .port_pin_mask =					\
 *                 GPIO_PORT_PIN_MASK_FROM_DT_INST(n),		\
 *         },							\
 *         .spi = SPI_DT_SPEC_INST_GET(n,				\
 *             SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),		\
 *         .ngpios = DT_INST_PROP(n, ngpios),			\
 *         <DEVNAME>_FAULT_GPIO_INIT(n)				\
 *         <DEVNAME>_ENABLE_GPIO_INIT(n)				\
 *         <DEVNAME>_RESET_GPIO_INIT(n)				\
 *     };								\
 *     DEVICE_DT_INST_DEFINE(n, <devname>_init, NULL,			\
 *                           &<devname>_data_##n,			\
 *                           &<devname>_config_##n,			\
 *                           POST_KERNEL,				\
 *                           CONFIG_GPIO_<DEVNAME>_INIT_PRIORITY,	\
 *                           &<devname>_api);
 *
 * DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_INIT)
 */
```

### Key elements explained

| Element | Purpose |
|---------|---------|
| `DT_DRV_COMPAT` | Must match the compatible string with commas replaced by underscores |
| `struct gpio_driver_config common` | Must be first field in config; holds `port_pin_mask` from DT |
| `struct gpio_driver_data common` | Must be first field in data; holds invert mask from `GPIO_ACTIVE_LOW` |
| `DEVICE_API(gpio, ...)` | Typed API struct -- replaces the old untyped `struct gpio_driver_api` literal |
| `I2C_DT_SPEC_INST_GET()` | Pulls I2C bus and address from DT |
| `SPI_DT_SPEC_INST_GET()` | Pulls SPI bus, CS, and frequency from DT |
| `GPIO_DT_SPEC_INST_GET()` | Pulls GPIO port/pin/flags for optional pins from DT |
| `GPIO_PORT_PIN_MASK_FROM_DT_INST()` | Generates pin mask from `ngpios` DT property |
| `DEVICE_DT_INST_DEFINE()` | Registers the device with Zephyr's device model |
| `DT_INST_FOREACH_STATUS_OKAY()` | Instantiates one driver per DT node with `status = "okay"` |
| `gpio_fire_callbacks()` | Utility from `gpio_utils.h` to dispatch application callbacks |
| `gpio_manage_callback()` | Utility from `gpio_utils.h` to add/remove callbacks from the list |
| `LOG_MODULE_REGISTER()` | Creates a logging module; level controlled by `CONFIG_GPIO_LOG_LEVEL` |
| `k_mutex` | Serializes bus transactions for thread safety |

---

## 8. Trigger/Interrupt Support

GPIO expanders and digital I/O devices typically expose a single
interrupt output (INT or FAULT pin) on the host side that fires when
any enabled GPIO input changes state or a fault condition occurs.

### 8.1 I2C expander interrupt flow

Because the I2C bus cannot be accessed from ISR context, the interrupt
handler must defer the actual register reads to a work queue:

```
1. Device INT pin fires (edge detected on host GPIO)
        |
        v
2. <devname>_int_handler() [ISR context]
   -> k_work_submit(&data->int_work)
        |
        v
3. <devname>_int_work_handler() [work queue thread]
   -> i2c_burst_read() interrupt status registers
   -> gpio_fire_callbacks() with the triggered pin mask
        |
        v
4. Application callback(s) execute [work queue thread]
```

```c
/**
 * @brief Work handler for processing GPIO interrupts.
 *
 * Deferred to a work queue because I2C transactions cannot be
 * performed in ISR context. Reads the interrupt status registers
 * to determine which pins triggered, then fires application callbacks.
 */
static void <devname>_int_work_handler(struct k_work *work)
{
	struct <devname>_data *data =
		CONTAINER_OF(work, struct <devname>_data, int_work);
	const struct device *dev = data->dev;
	uint8_t int_stat[<DEVNAME>_BANK_COUNT];
	gpio_port_pins_t fired_pins;
	int ret;

	/* Read and clear interrupt status */
	ret = <devname>_reg_burst_read(dev,
				       <DEVNAME>_REG_GPI_INT_STAT_A,
				       int_stat, <DEVNAME>_BANK_COUNT);
	if (ret < 0) {
		LOG_ERR("Failed to read interrupt status: %d", ret);
		return;
	}

	fired_pins = (uint32_t)int_stat[0] |
		     ((uint32_t)int_stat[1] << 8) |
		     ((uint32_t)int_stat[2] << 16);

	if (fired_pins) {
		gpio_fire_callbacks(&data->callbacks, dev, fired_pins);
	}
}

/**
 * @brief ISR for the INT pin from the device.
 *
 * Schedules the work handler to perform bus reads and fire callbacks.
 */
static void <devname>_int_handler(const struct device *port,
				  struct gpio_callback *cb,
				  gpio_port_pins_t pins)
{
	struct <devname>_data *data =
		CONTAINER_OF(cb, struct <devname>_data, int_cb);

	k_work_submit(&data->int_work);
}
```

### 8.2 SPI digital I/O fault interrupt flow

SPI devices with a FAULT pin follow a similar deferred pattern. If the
SPI controller supports ISR-context transfers, the fault handler can
read registers directly. Otherwise, defer to a work queue:

```c
/* Deferred variant for SPI fault handling */
struct <devname>_data {
	/* ... existing fields ... */
	struct k_work fault_work;
};

static void <devname>_fault_work_handler(struct k_work *work)
{
	struct <devname>_data *data =
		CONTAINER_OF(work, struct <devname>_data, fault_work);
	uint8_t fault_status;

	<devname>_read_faults(data->dev, &fault_status);

	if (fault_status) {
		gpio_fire_callbacks(&data->callbacks,
				    data->dev,
				    fault_status &
				    GENMASK(<DEVNAME>_MAX_CHANNELS - 1, 0));
	}
}

static void <devname>_fault_handler(const struct device *port,
				    struct gpio_callback *cb,
				    gpio_port_pins_t pins)
{
	struct <devname>_data *data =
		CONTAINER_OF(cb, struct <devname>_data, fault_cb);

	k_work_submit(&data->fault_work);
}
```

### 8.3 Interrupt initialization (in `<devname>_init()`)

```c
if (cfg->int_gpio.port != NULL) {
	if (!gpio_is_ready_dt(&cfg->int_gpio)) {
		LOG_ERR("INT GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&data->int_cb, <devname>_int_handler,
			   BIT(cfg->int_gpio.pin));
	ret = gpio_add_callback(cfg->int_gpio.port, &data->int_cb);
	if (ret < 0) {
		return ret;
	}

	k_work_init(&data->int_work, <devname>_int_work_handler);

	ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		return ret;
	}
}
```

---

## 9. Test Skeleton

### 9.1 Test Metadata (`tests/drivers/gpio/<devname>/testcase.yaml`)

```yaml
tests:
  drivers.gpio.<devname>:
    tags:
      - drivers
      - gpio
    depends_on: gpio
    platform_allow: native_sim
    integration_platforms:
      - native_sim
```

### 9.2 Test Project Config (`tests/drivers/gpio/<devname>/prj.conf`)

For I2C devices:

```
CONFIG_ZTEST=y
CONFIG_GPIO=y
CONFIG_GPIO_<DEVNAME>=y
CONFIG_I2C=y
CONFIG_LOG=y
```

For SPI devices:

```
CONFIG_ZTEST=y
CONFIG_GPIO=y
CONFIG_GPIO_<DEVNAME>=y
CONFIG_SPI=y
CONFIG_LOG=y
```

### 9.3 DT Overlay (`tests/drivers/gpio/<devname>/boards/native_sim.overlay`)

For I2C devices:

```dts
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

&i2c0 {
	status = "okay";

	<devname>_test: <devname>@34 {
		compatible = "adi,<devname>";
		reg = <0x34>;
		gpio-controller;
		#gpio-cells = <2>;
		ngpios = <19>;
	};
};
```

For SPI devices:

```dts
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

&spi0 {
	status = "okay";

	<devname>_test: <devname>@0 {
		compatible = "adi,<devname>";
		reg = <0>;
		spi-max-frequency = <1000000>;
		gpio-controller;
		#gpio-cells = <2>;
		ngpios = <4>;
	};
};
```

### 9.4 Test Source (`tests/drivers/gpio/<devname>/src/main.c`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>

#define GPIO_NODE DT_NODELABEL(<devname>_test)
static const struct device *gpio_dev = DEVICE_DT_GET(GPIO_NODE);

#define TEST_PIN_OUT  0
#define TEST_PIN_IN   1

ZTEST(gpio_<devname>, test_device_ready)
{
	zassert_true(device_is_ready(gpio_dev),
		     "GPIO device not ready");
}

ZTEST(gpio_<devname>, test_pin_configure_output)
{
	int ret;

	ret = gpio_pin_configure(gpio_dev, TEST_PIN_OUT,
				 GPIO_OUTPUT_ACTIVE);
	zassert_ok(ret, "gpio_pin_configure output failed: %d", ret);
}

ZTEST(gpio_<devname>, test_pin_configure_input)
{
	int ret;

	ret = gpio_pin_configure(gpio_dev, TEST_PIN_IN, GPIO_INPUT);
	zassert_ok(ret, "gpio_pin_configure input failed: %d", ret);
}

ZTEST(gpio_<devname>, test_pin_set_get)
{
	int ret;
	int val;

	ret = gpio_pin_configure(gpio_dev, TEST_PIN_OUT,
				 GPIO_OUTPUT_LOW);
	zassert_ok(ret, "gpio_pin_configure failed: %d", ret);

	ret = gpio_pin_set(gpio_dev, TEST_PIN_OUT, 1);
	zassert_ok(ret, "gpio_pin_set high failed: %d", ret);

	ret = gpio_pin_configure(gpio_dev, TEST_PIN_OUT, GPIO_INPUT);
	zassert_ok(ret, "reconfigure as input failed: %d", ret);

	val = gpio_pin_get(gpio_dev, TEST_PIN_OUT);
	zassert_true(val >= 0, "gpio_pin_get failed: %d", val);
}

ZTEST(gpio_<devname>, test_pin_toggle)
{
	int ret;

	ret = gpio_pin_configure(gpio_dev, TEST_PIN_OUT,
				 GPIO_OUTPUT_INACTIVE);
	zassert_ok(ret, "gpio_pin_configure failed: %d", ret);

	ret = gpio_pin_toggle(gpio_dev, TEST_PIN_OUT);
	zassert_ok(ret, "gpio_pin_toggle failed: %d", ret);
}

ZTEST(gpio_<devname>, test_invalid_pin)
{
	int ret;

	ret = gpio_pin_configure(gpio_dev, 255, GPIO_OUTPUT);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for invalid pin, got %d", ret);
}

ZTEST(gpio_<devname>, test_port_operations)
{
	int ret;
	gpio_port_value_t port_val;

	/* Configure pins as outputs */
	for (int i = 0; i < 8; i++) {
		ret = gpio_pin_configure(gpio_dev, i, GPIO_OUTPUT);
		zassert_ok(ret, "configure pin %d failed: %d", i, ret);
	}

	/* Set pins 0, 2, 4, 6 high */
	ret = gpio_port_set_masked_raw(gpio_dev, 0xFF, 0x55);
	zassert_ok(ret, "port_set_masked_raw failed: %d", ret);

	/* Read back port value */
	ret = gpio_port_get_raw(gpio_dev, &port_val);
	zassert_ok(ret, "port_get_raw failed: %d", ret);

	/* Set bits and clear bits */
	ret = gpio_port_set_bits_raw(gpio_dev, BIT(0));
	zassert_ok(ret, "port_set_bits_raw failed: %d", ret);

	ret = gpio_port_clear_bits_raw(gpio_dev, BIT(0));
	zassert_ok(ret, "port_clear_bits_raw failed: %d", ret);
}

ZTEST_SUITE(gpio_<devname>, NULL, NULL, NULL, NULL, NULL);
```

---

## 10. Key Conventions

1. **GPIO base structs** -- the config struct must embed
   `struct gpio_driver_config` as its first member (named `common`),
   and the data struct must embed `struct gpio_driver_data` as its
   first member (named `common`). The GPIO subsystem casts through
   these base types for pin mask validation and invert handling.

2. **`port_pin_mask` validation** -- the `port_pin_mask` in
   `gpio_driver_config` is generated from the `ngpios` DT property
   via `GPIO_PORT_PIN_MASK_FROM_DT_INST()`. The GPIO subsystem uses
   this mask to reject operations on pins that do not exist.

3. **`gpio_fire_callbacks()`** -- always use this utility from
   `<zephyr/drivers/gpio/gpio_utils.h>` to dispatch registered
   application callbacks. It walks the `sys_slist_t` of callbacks and
   invokes each one whose pin mask overlaps with the triggered pins.

4. **`gpio_manage_callback()`** -- use this utility to add/remove
   callbacks from the driver's `sys_slist_t`. It is the standard
   implementation for the `manage_callback` API entry.

5. **`gpio_utils.h`** -- always include
   `<zephyr/drivers/gpio/gpio_utils.h>`. It provides
   `gpio_fire_callbacks()` and `gpio_manage_callback()` which are
   not part of the public API header.

6. **No dynamic allocation** -- all per-instance state is declared
   statically via the `DEVICE_DT_INST_DEFINE()` instantiation macros.
   Never call `k_malloc()` or `k_calloc()` in a driver.

7. **Config vs Data** -- `config` is compile-time-constant (from DT)
   and stored in flash/ROM. `data` is mutable runtime state stored in
   RAM. Access them via `dev->config` and `dev->data`.

8. **Devicetree macros** -- use `DT_INST_*` macros (which rely on
   `DT_DRV_COMPAT`) rather than hardcoding node paths.

9. **Bus readiness** -- check `i2c_is_ready_dt()` or
   `spi_is_ready_dt()` in `init()` before any bus access.

10. **Thread safety** -- protect register access and cached state with
    `k_mutex`. GPIO drivers are commonly accessed from multiple threads
    (different subsystems controlling different pins). I2C interrupt
    handlers must defer to a `k_work` item since I2C transactions
    cannot run in ISR context.

11. **Error codes** -- return negative `errno` values (`-EINVAL`,
    `-ENOTSUP`, `-ENODEV`, etc.). Never return positive error codes.

12. **Logging** -- use `LOG_MODULE_REGISTER(gpio_<devname>, CONFIG_GPIO_LOG_LEVEL)`.
    Use `LOG_ERR` for errors, `LOG_WRN` for warnings, `LOG_INF` for
    informational messages, `LOG_DBG` for debug. Never use `printk()`.

13. **Bit manipulation** -- use `BIT()`, `GENMASK()`, `FIELD_PREP()`,
    `FIELD_GET()` from `<zephyr/sys/util.h>`.

14. **SPDX headers** -- every file needs a copyright line and
    `SPDX-License-Identifier: Apache-2.0`.

15. **DEVICE_API macro** -- always use `DEVICE_API(gpio, <devname>_api)`
    for the API struct declaration, not a raw `struct gpio_driver_api`.

16. **Init priority** -- use a custom `CONFIG_GPIO_<DEVNAME>_INIT_PRIORITY`
    that defaults to 75 (after bus drivers at ~60). The GPIO expander
    must initialize after the bus it sits on.

17. **Register caching** -- cache direction, output data, and interrupt
    enable state in the data struct to avoid unnecessary bus reads
    during read-modify-write operations.

18. **`gpio-controller.yaml` include** -- the DT binding must include
    `gpio-controller.yaml` so that other DT nodes can reference the
    device pins with `gpios = <&label PIN FLAGS>`.

19. **Coding style** -- Linux kernel style. Tabs for indentation (not
    spaces). 80-column soft limit, 100-column hard limit. Opening brace
    on a new line for function definitions.

---

## 11. Commit Message Format

Zephyr follows a strict commit message format. Each commit must have a
subsystem prefix, a short subject, and an informative body.

### Adding a new driver (typically 3-4 commits):

```
# Commit 1: Devicetree binding
dts: bindings: add binding for Analog Devices <DEVNAME>

Add devicetree binding for the Analog Devices <DEVNAME>
configurable [I/O expander / digital I/O] with <n>-pin GPIO
over [I2C / SPI] interface.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 2: Driver implementation
drivers: gpio: add Analog Devices <DEVNAME> driver

Add GPIO driver for the Analog Devices <DEVNAME>. The driver
implements the Zephyr GPIO subsystem API with support for
input/output configuration, [pull resistors / fault detection],
[open-drain output / current limiting], and interrupt-driven
pin change notification.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 3: Tests
tests: drivers: gpio: add tests for <DEVNAME>

Add unit tests for the <DEVNAME> GPIO driver covering pin
configuration, port read/write, toggle, and error handling.

Signed-off-by: Your Name <your.name@analog.com>
```

### Key commit message rules:

- Subject line: max 72 characters, no trailing period
- Prefix: matches the path (e.g., `drivers: gpio:`, `dts: bindings:`)
- Body: wrapped at 75 characters, explains the "why"
- Must include `Signed-off-by:` (DCO requirement)
- Use imperative mood ("add", not "added" or "adds")
