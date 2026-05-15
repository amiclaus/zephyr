# Zephyr Clock Control / Frequency Driver Template

Reference: `drivers/frequency/DRIVER_TEMPLATE.md` (no-OS hardware patterns)

This template covers every file needed to add a new clock generator or
clock distribution driver to the Zephyr RTOS for an Analog Devices part
(e.g., AD9508, AD9528, ADF4368). Replace `<devname>` with the part
number in lowercase (e.g., `ad9508`), `<DEVNAME>` with uppercase
(e.g., `AD9508`), and `<devnum>` with the numeric portion (e.g., `9508`)
throughout.

---

## 1. Purpose & Subsystem Mapping

This driver maps to the Zephyr **clock_control subsystem** defined in
`include/zephyr/drivers/clock_control.h`. The subsystem provides a
unified API for clock providers -- devices that generate, distribute, or
gate clock signals to other peripherals.

| Zephyr clock_control API Function | What it does                             |
|-----------------------------------|------------------------------------------|
| `clock_control_on()`             | Enable (ungate) a clock output           |
| `clock_control_off()`            | Disable (gate) a clock output            |
| `clock_control_get_rate()`       | Query the current output frequency (Hz)  |
| `clock_control_set_rate()`       | Set a new output frequency               |
| `clock_control_get_status()`     | Check if a clock output is running       |

The driver implements the `clock_control_driver_api` struct (via the
`DEVICE_API(clock_control, ...)` macro) which contains pointers to the
driver's `on`, `off`, `get_rate`, and `set_rate` implementations.

Clock generators act as **clock providers** in the devicetree. Consumer
devices reference the clock provider node via a `clocks` phandle, and
the `#clock-cells` property determines how many cells are needed to
identify a specific output (typically 1 -- the output channel index).

All configuration comes from **devicetree** at compile time. There is no
dynamic allocation -- the config struct is `const` and populated from DT
macros, while the data struct is static and holds mutable runtime state.

---

## 2. File Checklist

```
zephyr/
    drivers/clock_control/
        clock_control_<devname>.c     # Driver implementation
        Kconfig.<devname>             # Kconfig fragment
        CMakeLists.txt                # (append to existing)
        Kconfig                       # (append to existing)

    include/zephyr/drivers/clock_control/
        <devname>.h                   # Driver header (optional, for register
                                      # defines shared with tests/emulators)

    dts/bindings/clock/
        adi,<devname>.yaml            # Devicetree binding

    tests/drivers/clock_control/<devname>/
        testcase.yaml                 # Test metadata
        prj.conf                      # Test project config
        boards/native_sim.overlay     # DT overlay for test
        src/main.c                    # Test source
```

---

## 3. Devicetree Binding (`dts/bindings/clock/adi,<devname>.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Analog Devices <DEVNAME> clock generator / distributor.

  The <DEVNAME> is a clock distribution IC with <N> outputs and
  configurable dividers. It communicates over SPI.

  Example devicetree node:

    &spi1 {
        status = "okay";
        cs-gpios = <&gpioa 4 GPIO_ACTIVE_LOW>;

        clk0: <devname>@0 {
            compatible = "adi,<devname>";
            reg = <0>;
            spi-max-frequency = <10000000>;
            #clock-cells = <1>;

            clock-output-names = "out0", "out1", "out2", "out3";

            /* Reference clock input (parent clock) */
            clocks = <&ref_clk>;

            /* Optional: lock-detect interrupt */
            /* lock-detect-gpios = <&gpiob 3 GPIO_ACTIVE_HIGH>; */
        };
    };

    /* Consumer example: */
    &adc0 {
        clocks = <&clk0 1>;   /* Use output 1 from <devname> */
    };

compatible: "adi,<devname>"

include: [spi-device.yaml, base.yaml]

properties:
  "#clock-cells":
    const: 1
    description: |
      The clock cell specifies the output channel index (0-based).

  clock-output-names:
    type: string-array
    description: |
      Human-readable names for each clock output. The number of
      entries determines the number of outputs the driver manages.

  clocks:
    type: phandle-array
    description: |
      Reference to the parent (input) clock source. Used to
      determine the input frequency for divider calculations.

  lock-detect-gpios:
    type: phandle-array
    description: |
      GPIO connected to the lock-detect (LD) output pin.
      This is optional; when omitted the driver does not monitor
      PLL lock status via interrupt.
```

For I2C devices, replace `spi-device.yaml` with `i2c-device.yaml` and
adjust the example node accordingly.

---

## 4. Kconfig (`drivers/clock_control/Kconfig.<devname>`)

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config CLOCK_CONTROL_<DEVNAME>
	bool "<DEVNAME> clock generator driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	select SPI
	help
	  Enable support for the Analog Devices <DEVNAME>
	  clock generator / distribution IC with <N> outputs.
```

Then add to the parent `drivers/clock_control/Kconfig`:

```kconfig
source "drivers/clock_control/Kconfig.<devname>"
```

For I2C devices, replace `select SPI` with `select I2C`.

---

## 5. CMakeLists.txt (Build System Integration)

Append to the existing `drivers/clock_control/CMakeLists.txt`:

```cmake
zephyr_library_sources_ifdef(CONFIG_CLOCK_CONTROL_<DEVNAME> clock_control_<devname>.c)
```

---

## 6. Driver Header (Optional)

If the driver is self-contained, no separate header is needed. However,
if register definitions or data types must be shared with an emulator or
test, create `include/zephyr/drivers/clock_control/<devname>.h`:

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_<DEVNAME>_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_<DEVNAME>_H_

/* ---------------- Register Map ------------------------------------ */
/* Adapt from the no-OS driver or datasheet.
 * Example based on AD9508 register layout.
 */

#define <DEVNAME>_REG_SPI_CONFIG	0x000
#define <DEVNAME>_REG_PART_ID_LOW	0x00C
#define <DEVNAME>_REG_PART_ID_HIGH	0x00D
#define <DEVNAME>_REG_OUT_DIV_LOW(ch)	(0x01B + (ch) * 0x08)
#define <DEVNAME>_REG_OUT_DIV_HIGH(ch)	(0x01C + (ch) * 0x08)
#define <DEVNAME>_REG_OUT_PHASE_LOW(ch)	(0x01D + (ch) * 0x08)
#define <DEVNAME>_REG_OUT_PHASE_HIGH(ch) (0x01E + (ch) * 0x08)

#define <DEVNAME>_PART_ID_VALUE		0x005

/* Maximum number of clock outputs. */
#define <DEVNAME>_MAX_OUTPUTS		4

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_<DEVNAME>_H_ */
```

---

## 7. Driver Source (`drivers/clock_control/clock_control_<devname>.c`)

This is the core of the driver. It follows the Zephyr clock_control
subsystem contract and Linux kernel coding style.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(<devname>, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

/* ---------------- Register Map ------------------------------------ */
/*
 * If a shared header was created (Section 6), include it here instead.
 * Otherwise, define registers locally.
 * Example based on AD9508 (clock distribution, SPI, 3-byte frames).
 */

#define <DEVNAME>_REG_SPI_CONFIG	0x000
#define <DEVNAME>_REG_PART_ID_LOW	0x00C
#define <DEVNAME>_REG_PART_ID_HIGH	0x00D
#define <DEVNAME>_REG_OUT_DIV_LOW(ch)	(0x01B + (ch) * 0x08)
#define <DEVNAME>_REG_OUT_DIV_HIGH(ch)	(0x01C + (ch) * 0x08)
#define <DEVNAME>_REG_OUT_ENABLE	0x03D

#define <DEVNAME>_PART_ID_VALUE		0x005
#define <DEVNAME>_SOFT_RESET		0x24

#define <DEVNAME>_MAX_OUTPUTS		4

/* ---------------- Config & Data Structs --------------------------- */

/**
 * Compile-time configuration from devicetree.
 * This struct is const and stored in ROM.
 */
struct <devname>_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec lock_detect_gpio; /* Optional lock-detect GPIO */
	uint32_t num_outputs;
	uint32_t input_freq_hz;               /* From parent clock, if known */
};

/**
 * Mutable runtime data.
 * This struct is allocated statically per instance.
 */
struct <devname>_data {
	const struct device *dev;
	uint32_t output_freq_hz[<DEVNAME>_MAX_OUTPUTS];
	uint32_t divider[<DEVNAME>_MAX_OUTPUTS];
	bool output_enabled[<DEVNAME>_MAX_OUTPUTS];
	struct k_mutex lock;
};

/* ---------------- SPI Helpers ------------------------------------- */

/**
 * @brief Read a register over SPI.
 *
 * AD9508-style: 3-byte frame. Byte 0 MSB=1 for read, bits[14:8] of
 * addr in byte 0 bits[6:0], bits[7:0] of addr in byte 1, data in byte 2.
 */
static int <devname>_reg_read(const struct device *dev, uint16_t addr,
			      uint8_t *val)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[3] = {
		0x80 | (uint8_t)(addr >> 8),
		(uint8_t)(addr & 0xFF),
		0x00
	};
	uint8_t rx_buf[3];
	int ret;

	const struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	const struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("SPI read failed: %d", ret);
		return ret;
	}

	*val = rx_buf[2];

	return 0;
}

/**
 * @brief Write a register over SPI.
 *
 * AD9508-style: 3-byte frame. Byte 0 MSB=0 for write.
 */
static int <devname>_reg_write(const struct device *dev, uint16_t addr,
			       uint8_t val)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[3] = {
		(uint8_t)(addr >> 8),
		(uint8_t)(addr & 0xFF),
		val
	};

	const struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	return spi_write_dt(&cfg->spi, &tx_set);
}

/**
 * @brief Update specific bits in a register (read-modify-write).
 */
static int <devname>_reg_update(const struct device *dev, uint16_t addr,
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

/* ---------------- Clock Control Subsystem Callbacks --------------- */

/**
 * @brief Enable (ungate) a clock output.
 *
 * @param dev      Clock controller device.
 * @param sys      Subsystem identifier -- cast to output channel index.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_clock_on(const struct device *dev,
			      clock_control_subsys_t sys)
{
	struct <devname>_data *data = dev->data;
	uint32_t output = (uint32_t)(uintptr_t)sys;
	int ret;

	if (output >= <DEVNAME>_MAX_OUTPUTS) {
		LOG_ERR("Invalid output channel: %u", output);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/*
	 * Enable the output by setting the appropriate bit in the
	 * output enable register. Adapt to match the device's
	 * register layout.
	 */
	ret = <devname>_reg_update(dev, <DEVNAME>_REG_OUT_ENABLE,
				   BIT(output), BIT(output));
	if (ret == 0) {
		data->output_enabled[output] = true;
		LOG_DBG("Output %u enabled", output);
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

/**
 * @brief Disable (gate) a clock output.
 */
static int <devname>_clock_off(const struct device *dev,
			       clock_control_subsys_t sys)
{
	struct <devname>_data *data = dev->data;
	uint32_t output = (uint32_t)(uintptr_t)sys;
	int ret;

	if (output >= <DEVNAME>_MAX_OUTPUTS) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = <devname>_reg_update(dev, <DEVNAME>_REG_OUT_ENABLE,
				   BIT(output), 0);
	if (ret == 0) {
		data->output_enabled[output] = false;
		LOG_DBG("Output %u disabled", output);
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

/**
 * @brief Get the current rate of a clock output.
 *
 * @param dev      Clock controller device.
 * @param sys      Output channel index.
 * @param rate     Pointer to store the frequency in Hz.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_clock_get_rate(const struct device *dev,
				    clock_control_subsys_t sys,
				    uint32_t *rate)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint32_t output = (uint32_t)(uintptr_t)sys;

	if (output >= <DEVNAME>_MAX_OUTPUTS) {
		return -EINVAL;
	}

	if (rate == NULL) {
		return -EINVAL;
	}

	/*
	 * For a clock distributor: output_freq = input_freq / divider.
	 * For a PLL-based clock generator: output_freq is the VCO
	 * frequency divided by the output divider.
	 * Adapt the calculation to match the device architecture.
	 */
	if (data->divider[output] == 0) {
		*rate = cfg->input_freq_hz;  /* Bypass / divide-by-1 */
	} else {
		*rate = cfg->input_freq_hz / data->divider[output];
	}

	return 0;
}

/**
 * @brief Set the rate of a clock output.
 *
 * @param dev      Clock controller device.
 * @param sys      Output channel index.
 * @param rate     Pointer to the desired frequency in Hz. On clock
 *                 distribution devices this computes the nearest
 *                 achievable divider. On return, the actual frequency
 *                 is written back.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_clock_set_rate(const struct device *dev,
				    clock_control_subsys_t sys,
				    clock_control_subsys_rate_t rate)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint32_t output = (uint32_t)(uintptr_t)sys;
	uint32_t *freq = (uint32_t *)rate;
	uint32_t divider;
	int ret;

	if (output >= <DEVNAME>_MAX_OUTPUTS || freq == NULL) {
		return -EINVAL;
	}

	if (*freq == 0 || cfg->input_freq_hz == 0) {
		return -EINVAL;
	}

	/*
	 * Compute divider. For a simple clock distributor:
	 *   divider = input_freq / desired_freq
	 * Clamp to the device's supported range.
	 */
	divider = cfg->input_freq_hz / *freq;
	if (divider == 0) {
		divider = 1;  /* Cannot exceed input frequency */
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Write divider low byte */
	ret = <devname>_reg_write(dev, <DEVNAME>_REG_OUT_DIV_LOW(output),
				  (uint8_t)(divider & 0xFF));
	if (ret < 0) {
		goto unlock;
	}

	/* Write divider high byte */
	ret = <devname>_reg_write(dev, <DEVNAME>_REG_OUT_DIV_HIGH(output),
				  (uint8_t)((divider >> 8) & 0x03));
	if (ret < 0) {
		goto unlock;
	}

	data->divider[output] = divider;
	data->output_freq_hz[output] = cfg->input_freq_hz / divider;

	/* Write back the actual achieved frequency */
	*freq = data->output_freq_hz[output];

	LOG_DBG("Output %u: divider=%u, freq=%u Hz", output, divider,
		data->output_freq_hz[output]);

unlock:
	k_mutex_unlock(&data->lock);

	return ret;
}

/**
 * @brief Get the status of a clock output.
 *
 * @return CLOCK_CONTROL_STATUS_ON or CLOCK_CONTROL_STATUS_OFF.
 */
static enum clock_control_status <devname>_clock_get_status(
		const struct device *dev, clock_control_subsys_t sys)
{
	struct <devname>_data *data = dev->data;
	uint32_t output = (uint32_t)(uintptr_t)sys;

	if (output >= <DEVNAME>_MAX_OUTPUTS) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	return data->output_enabled[output]
		? CLOCK_CONTROL_STATUS_ON
		: CLOCK_CONTROL_STATUS_OFF;
}

/* ---------------- Initialization ---------------------------------- */

/**
 * @brief Verify the device identity by reading the product ID registers.
 */
static int <devname>_verify_id(const struct device *dev)
{
	uint8_t id_low, id_high;
	uint16_t id;
	int ret;

	ret = <devname>_reg_read(dev, <DEVNAME>_REG_PART_ID_LOW, &id_low);
	if (ret < 0) {
		return ret;
	}

	ret = <devname>_reg_read(dev, <DEVNAME>_REG_PART_ID_HIGH, &id_high);
	if (ret < 0) {
		return ret;
	}

	id = ((uint16_t)id_high << 8) | id_low;

	if (id != <DEVNAME>_PART_ID_VALUE) {
		LOG_ERR("Unexpected product ID: 0x%04X (expected 0x%04X)",
			id, <DEVNAME>_PART_ID_VALUE);
		return -ENODEV;
	}

	LOG_DBG("Product ID verified: 0x%04X", id);

	return 0;
}

/**
 * @brief Device initialization function.
 *
 * Called automatically at boot for each DT instance.
 */
static int <devname>_init(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	data->dev = dev;
	k_mutex_init(&data->lock);

	/* Verify SPI bus is ready */
	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Optional: configure lock-detect GPIO */
	if (cfg->lock_detect_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->lock_detect_gpio)) {
			LOG_ERR("Lock-detect GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->lock_detect_gpio,
					    GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Failed to configure lock-detect GPIO: %d",
				ret);
			return ret;
		}
	}

	/* Software reset */
	ret = <devname>_reg_write(dev, <DEVNAME>_REG_SPI_CONFIG,
				  <DEVNAME>_SOFT_RESET);
	if (ret < 0) {
		LOG_ERR("Software reset failed: %d", ret);
		return ret;
	}

	k_msleep(250);  /* Wait for reset to complete */

	/* Verify device identity */
	ret = <devname>_verify_id(dev);
	if (ret < 0) {
		return ret;
	}

	/*
	 * Apply default configuration:
	 * - Set all output dividers to 1 (passthrough)
	 * - Enable all configured outputs
	 * Adapt to the device datasheet power-up sequence.
	 */
	for (uint32_t i = 0; i < cfg->num_outputs; i++) {
		data->divider[i] = 1;
		data->output_freq_hz[i] = cfg->input_freq_hz;
		data->output_enabled[i] = true;
	}

	LOG_INF("<DEVNAME> initialized on %s (%u outputs, input=%u Hz)",
		cfg->spi.bus->name, cfg->num_outputs,
		cfg->input_freq_hz);

	return 0;
}

/* ---------------- API Struct -------------------------------------- */

/*
 * Use the DEVICE_API macro. This creates a static const struct that
 * the Zephyr clock_control subsystem uses to dispatch calls.
 */
static DEVICE_API(clock_control, <devname>_api) = {
	.on       = <devname>_clock_on,
	.off      = <devname>_clock_off,
	.get_rate = <devname>_clock_get_rate,
	.set_rate = <devname>_clock_set_rate,
	.get_status = <devname>_clock_get_status,
};

/* ---------------- Instance Macros --------------------------------- */

/*
 * These macros are expanded once per DT instance with status "okay".
 * They create the config, data, and DEVICE_DT_INST_DEFINE entries.
 */

#define <DEVNAME>_LOCK_DETECT_GPIO_INIT(n)				\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, lock_detect_gpios),	\
		(.lock_detect_gpio =					\
			GPIO_DT_SPEC_INST_GET(n, lock_detect_gpios),),	\
		(.lock_detect_gpio = { 0 },))

#define <DEVNAME>_INIT(n)						\
	static struct <devname>_data <devname>_data_##n;		\
									\
	static const struct <devname>_config <devname>_config_##n = {	\
		.spi = SPI_DT_SPEC_INST_GET(n,				\
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),	\
		<DEVNAME>_LOCK_DETECT_GPIO_INIT(n)			\
		.num_outputs = DT_INST_PROP_LEN_OR(n,			\
				clock_output_names, 1),			\
		.input_freq_hz = DT_INST_PROP_BY_IDX_OR(n,		\
				clock_frequency, 0, 0),			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, <devname>_init, NULL,			\
			      &<devname>_data_##n,			\
			      &<devname>_config_##n,			\
			      POST_KERNEL,				\
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,	\
			      &<devname>_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_INIT)
```

### Key elements explained

| Element | Purpose |
|---------|---------|
| `DT_DRV_COMPAT` | Must match the compatible string with commas replaced by underscores |
| `struct <devname>_config` | Compile-time config from DT (stored in ROM) |
| `struct <devname>_data` | Mutable runtime state (one per instance) |
| `DEVICE_API(clock_control, ...)` | Typed API struct -- replaces the old untyped `struct clock_control_driver_api` literal |
| `SPI_DT_SPEC_INST_GET()` | Pulls SPI bus, CS, and frequency from DT |
| `GPIO_DT_SPEC_INST_GET()` | Pulls GPIO port/pin/flags from DT |
| `DEVICE_DT_INST_DEFINE()` | Registers the device with Zephyr's device model |
| `DT_INST_FOREACH_STATUS_OKAY()` | Instantiates one driver per DT node with `status = "okay"` |
| `LOG_MODULE_REGISTER()` | Creates a logging module; level controlled by `CONFIG_CLOCK_CONTROL_LOG_LEVEL` |
| `clock_control_subsys_t` | Opaque pointer cast to/from the output channel index |
| `#clock-cells = <1>` | Tells devicetree that consumers need one cell (the output index) |

---

## 8. Trigger / Interrupt Support (Optional)

If the device has a lock-detect (LD) output, implement a GPIO callback
to notify consumers when the PLL has achieved lock. This is useful for
clock generators with internal PLLs.

```c
/* Add to data struct: */
struct <devname>_data {
	/* ... existing fields ... */
	struct gpio_callback ld_cb;
	struct k_sem lock_sem;
	bool pll_locked;
};

/* Lock-detect interrupt handler: */
static void <devname>_lock_detect_handler(const struct device *port,
					  struct gpio_callback *cb,
					  gpio_port_pins_t pins)
{
	struct <devname>_data *data =
		CONTAINER_OF(cb, struct <devname>_data, ld_cb);

	data->pll_locked = true;
	k_sem_give(&data->lock_sem);
}

/* In init(), set up the interrupt: */
static int <devname>_init_lock_detect(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	if (cfg->lock_detect_gpio.port == NULL) {
		return 0;  /* No lock-detect GPIO configured */
	}

	k_sem_init(&data->lock_sem, 0, 1);

	gpio_init_callback(&data->ld_cb, <devname>_lock_detect_handler,
			   BIT(cfg->lock_detect_gpio.pin));

	ret = gpio_add_callback(cfg->lock_detect_gpio.port, &data->ld_cb);
	if (ret < 0) {
		return ret;
	}

	return gpio_pin_interrupt_configure_dt(&cfg->lock_detect_gpio,
					       GPIO_INT_EDGE_TO_ACTIVE);
}

/* Wait for PLL lock after frequency change: */
static int <devname>_wait_for_lock(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;

	if (cfg->lock_detect_gpio.port == NULL) {
		/* Polling fallback -- wait a fixed time */
		k_msleep(10);
		return 0;
	}

	data->pll_locked = false;

	return k_sem_take(&data->lock_sem, K_MSEC(1000));
}
```

---

## 9. Test Skeleton

### 9.1 Test Metadata (`tests/drivers/clock_control/<devname>/testcase.yaml`)

```yaml
tests:
  drivers.clock_control.<devname>:
    tags:
      - drivers
      - clock_control
    depends_on: spi
    platform_allow: native_sim
    integration_platforms:
      - native_sim
```

### 9.2 Test Project Config (`tests/drivers/clock_control/<devname>/prj.conf`)

```
CONFIG_ZTEST=y
CONFIG_CLOCK_CONTROL=y
CONFIG_CLOCK_CONTROL_<DEVNAME>=y
CONFIG_SPI=y
CONFIG_LOG=y
```

### 9.3 DT Overlay (`tests/drivers/clock_control/<devname>/boards/native_sim.overlay`)

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
		#clock-cells = <1>;
		clock-output-names = "out0", "out1";
	};
};
```

### 9.4 Test Source (`tests/drivers/clock_control/<devname>/src/main.c`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/device.h>

#define CLK_NODE DT_NODELABEL(<devname>_test)
static const struct device *clk_dev = DEVICE_DT_GET(CLK_NODE);

/* Output channel 0 */
#define CLK_OUTPUT_0 ((clock_control_subsys_t)(uintptr_t)0)
#define CLK_OUTPUT_1 ((clock_control_subsys_t)(uintptr_t)1)

ZTEST(clk_<devname>, test_device_ready)
{
	zassert_true(device_is_ready(clk_dev),
		     "Clock device not ready");
}

ZTEST(clk_<devname>, test_clock_on_off)
{
	int ret;

	ret = clock_control_on(clk_dev, CLK_OUTPUT_0);
	zassert_ok(ret, "clock_control_on failed: %d", ret);

	zassert_equal(clock_control_get_status(clk_dev, CLK_OUTPUT_0),
		      CLOCK_CONTROL_STATUS_ON,
		      "Expected clock ON after enable");

	ret = clock_control_off(clk_dev, CLK_OUTPUT_0);
	zassert_ok(ret, "clock_control_off failed: %d", ret);

	zassert_equal(clock_control_get_status(clk_dev, CLK_OUTPUT_0),
		      CLOCK_CONTROL_STATUS_OFF,
		      "Expected clock OFF after disable");
}

ZTEST(clk_<devname>, test_get_rate)
{
	uint32_t rate = 0;
	int ret;

	ret = clock_control_get_rate(clk_dev, CLK_OUTPUT_0, &rate);
	zassert_ok(ret, "clock_control_get_rate failed: %d", ret);
	zassert_true(rate > 0, "Rate should be non-zero");
}

ZTEST(clk_<devname>, test_set_rate)
{
	uint32_t rate = 25000000;  /* Request 25 MHz */
	int ret;

	ret = clock_control_set_rate(clk_dev, CLK_OUTPUT_0,
				     (clock_control_subsys_rate_t)&rate);
	zassert_ok(ret, "clock_control_set_rate failed: %d", ret);
	zassert_true(rate > 0, "Actual rate should be non-zero");
}

ZTEST(clk_<devname>, test_invalid_output)
{
	int ret;
	clock_control_subsys_t bad_output =
		(clock_control_subsys_t)(uintptr_t)255;

	ret = clock_control_on(clk_dev, bad_output);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for invalid output, got %d", ret);
}

ZTEST_SUITE(clk_<devname>, NULL, NULL, NULL, NULL, NULL);
```

### 9.5 SPI Emulator Pattern (Optional)

For proper hardware-in-the-loop testing on `native_sim`, create an SPI
emulator that responds to the device's register protocol:

```c
/*
 * tests/drivers/clock_control/<devname>/src/<devname>_emul.c
 *
 * Minimal SPI emulator for <DEVNAME> tests.
 */

#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/spi_emul.h>

struct <devname>_emul_data {
	uint8_t regs[256];
};

static int <devname>_emul_io(const struct emul *target,
			     const struct spi_config *config,
			     const struct spi_buf_set *tx_bufs,
			     const struct spi_buf_set *rx_bufs)
{
	struct <devname>_emul_data *data = target->data;
	const uint8_t *tx = tx_bufs->buffers[0].buf;
	uint8_t *rx = rx_bufs ? rx_bufs->buffers[0].buf : NULL;
	uint8_t addr = (uint8_t)(((tx[0] & 0x7F) << 8) | tx[1]);
	bool is_read = (tx[0] & 0x80) != 0;

	if (is_read && rx) {
		rx[2] = data->regs[addr & 0xFF];
	} else {
		data->regs[addr & 0xFF] = tx[2];
	}

	return 0;
}

static const struct spi_emul_api <devname>_emul_api = {
	.io = <devname>_emul_io,
};

static int <devname>_emul_init(const struct emul *target,
			       const struct device *parent)
{
	struct <devname>_emul_data *data = target->data;

	/* Pre-populate the product ID register */
	data->regs[0x0C] = <DEVNAME>_PART_ID_VALUE & 0xFF;
	data->regs[0x0D] = (<DEVNAME>_PART_ID_VALUE >> 8) & 0xFF;

	return 0;
}

#define <DEVNAME>_EMUL_INIT(n)						\
	static struct <devname>_emul_data <devname>_emul_data_##n;	\
	EMUL_DT_INST_DEFINE(n, <devname>_emul_init,			\
			    &<devname>_emul_data_##n,			\
			    NULL, &<devname>_emul_api, NULL);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_EMUL_INIT)
```

---

## 10. Key Conventions

1. **Coding style** -- Linux kernel style. Tabs for indentation (not
   spaces). 80-column soft limit, 100-column hard limit. Opening braces
   on the same line for functions (`static int foo(void) {` is wrong,
   use a newline before `{`).

2. **No dynamic allocation** -- all per-instance state is declared
   statically via the `DEVICE_DT_INST_DEFINE()` instantiation macros.
   Never call `k_malloc()` or `k_calloc()` in a driver.

3. **Config vs Data** -- `config` is compile-time-constant (from DT)
   and stored in flash/ROM. `data` is mutable runtime state stored in
   RAM. Access them via `dev->config` and `dev->data`.

4. **Devicetree macros** -- use `DT_INST_*` macros (which rely on
   `DT_DRV_COMPAT`) rather than hardcoding node paths.

5. **SPI/I2C specs** -- always use `spi_dt_spec` / `i2c_dt_spec`
   obtained from `SPI_DT_SPEC_INST_GET()` / `I2C_DT_SPEC_INST_GET()`.
   Never store raw bus pointers.

6. **Bus readiness** -- check `spi_is_ready_dt()` or
   `i2c_is_ready_dt()` in `init()` before any bus access.

7. **Error codes** -- return negative `errno` values (`-EINVAL`,
   `-ENOTSUP`, `-ENODEV`, `-ENOMEM`, etc.). Never return positive
   error codes.

8. **Logging** -- use
   `LOG_MODULE_REGISTER(<devname>, CONFIG_CLOCK_CONTROL_LOG_LEVEL)`.
   Use `LOG_ERR` for errors, `LOG_WRN` for warnings, `LOG_INF` for
   informational messages, `LOG_DBG` for debug messages. Never use
   `printk()` in a driver.

9. **Byte order** -- use `sys_get_be16()` / `sys_put_be16()` (or le
   variants) from `<zephyr/sys/byteorder.h>` instead of manual
   shifting.

10. **Bit manipulation** -- use `BIT()`, `GENMASK()`, `FIELD_PREP()`,
    `FIELD_GET()` from `<zephyr/sys/util.h>`. These are the Zephyr
    equivalents of the no-OS `NO_OS_BIT()`, `NO_OS_GENMASK()`,
    `no_os_field_prep()`, and `no_os_field_get()` macros.

11. **SPDX headers** -- every file needs a copyright line and
    `SPDX-License-Identifier: Apache-2.0`. No lengthy BSD-3-Clause
    blocks -- Zephyr uses the SPDX short form.

12. **DEVICE_API macro** -- always use
    `DEVICE_API(clock_control, <devname>_api)` for the API struct
    declaration, not a raw `struct clock_control_driver_api` assignment.

13. **Init priority** -- use `CONFIG_CLOCK_CONTROL_INIT_PRIORITY`
    (default 60) so that clock providers initialize before their
    consumers. If the clock device sits behind another bus controller,
    ensure the bus initializes at an earlier priority.

14. **Thread safety** -- clock outputs may be enabled/disabled or
    reconfigured from multiple threads. Protect shared state with
    `k_mutex`. The driver template uses a mutex around all register
    read-modify-write sequences.

15. **Clock provider pattern** -- in devicetree, clock generators use
    `#clock-cells = <1>` so consumers can reference them as
    `clocks = <&clk_node output_index>`. The `clock_control_subsys_t`
    parameter in API callbacks carries the output index cast from the
    consumer's phandle cell.

---

## 11. Commit Message Format

Zephyr follows a strict commit message format. Each commit must have a
subsystem prefix, a short subject, and an informative body.

### Adding a new driver (typically 3-4 commits):

```
# Commit 1: Devicetree binding
dts: bindings: add binding for Analog Devices <DEVNAME>

Add devicetree binding for the Analog Devices <DEVNAME> clock
generator / distribution IC with <N> outputs and SPI interface.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 2: Driver implementation
drivers: clock_control: add Analog Devices <DEVNAME> driver

Add support for the Analog Devices <DEVNAME> clock generator /
distribution IC. The driver implements the Zephyr clock_control
subsystem API with support for per-output enable/disable, rate
query, and divider configuration over SPI.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 3: Tests
tests: drivers: clock_control: add tests for <DEVNAME>

Add unit tests for the <DEVNAME> clock control driver covering
output enable/disable, rate get/set, and error handling on
invalid outputs.

Signed-off-by: Your Name <your.name@analog.com>
```

### Key commit message rules:

- Subject line: max 72 characters, no trailing period
- Prefix: matches the path (e.g., `drivers: clock_control:`, `dts: bindings:`)
- Body: wrapped at 75 characters, explains the "why"
- Must include `Signed-off-by:` (DCO requirement)
- Use imperative mood ("add", not "added" or "adds")
