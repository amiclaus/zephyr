# Zephyr ADC Driver Template

Reference: `drivers/adc/DRIVER_TEMPLATE.md` (no-OS hardware patterns)

This template covers every file needed to add a new ADC driver to the
Zephyr RTOS for an Analog Devices part. Replace `<devname>` with the part
number in lowercase (e.g., `ad4052`), `<DEVNAME>` with uppercase
(e.g., `AD4052`), and `<devnum>` with the numeric portion (e.g., `4052`)
throughout.

---

## 1. Purpose & Zephyr Subsystem Mapping

This driver maps to the Zephyr **ADC subsystem** defined in
`include/zephyr/drivers/adc.h`. The subsystem provides a unified API for
analog-to-digital converters with the following key entry points:

| Zephyr ADC API Function  | What it does                              |
|--------------------------|-------------------------------------------|
| `adc_channel_setup()`    | Configure a single ADC channel            |
| `adc_read()`             | Perform a synchronous (polled) conversion |
| `adc_read_async()`       | Perform an asynchronous conversion        |

The driver implements the `adc_driver_api` struct (via the
`DEVICE_API(adc, ...)` macro) which contains pointers to the driver's
`channel_setup` and `read` implementations.

All configuration comes from **devicetree** at compile time. There is no
dynamic allocation -- the config struct is `const` and populated from DT
macros, while the data struct is static and holds mutable runtime state.

---

## 2. File Checklist

```
zephyr/
    drivers/adc/
        adc_<devname>.c           # Driver implementation
        Kconfig.<devname>         # Kconfig fragment
        CMakeLists.txt            # (append to existing)
        Kconfig                   # (append to existing)

    include/zephyr/drivers/adc/
        <devname>.h               # Driver header (optional, for register
                                  # defines shared with tests/emulators)

    dts/bindings/adc/
        adi,<devname>.yaml        # Devicetree binding

    tests/drivers/adc/<devname>/
        testcase.yaml             # Test metadata
        prj.conf                  # Test project config
        boards/native_sim.overlay # DT overlay for test
        src/main.c                # Test source
```

---

## 3. Devicetree Binding (`dts/bindings/adc/adi,<devname>.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Analog Devices <DEVNAME> <resolution>-bit <channels>-channel ADC.

  Example devicetree node:

    &spi1 {
        status = "okay";
        cs-gpios = <&gpioa 4 GPIO_ACTIVE_LOW>;

        adc0: <devname>@0 {
            compatible = "adi,<devname>";
            reg = <0>;
            spi-max-frequency = <20000000>;
            #io-channel-cells = <1>;

            /* Optional: data-ready interrupt */
            /* drdy-gpios = <&gpiob 0 GPIO_ACTIVE_LOW>; */
        };
    };

compatible: "adi,<devname>"

include: [spi-device.yaml, base.yaml]

properties:
  "#io-channel-cells":
    const: 1

  drdy-gpios:
    type: phandle-array
    description: |
      GPIO connected to the DRDY / interrupt output pin.
      This is optional; when omitted the driver uses polling.

  vref-mv:
    type: int
    default: 2500
    description: |
      Reference voltage in millivolts. Used to compute the scale
      attribute. Defaults to the internal 2.5 V reference.
```

For I2C devices, replace `spi-device.yaml` with `i2c-device.yaml` and
adjust the example node accordingly.

---

## 4. Kconfig (`drivers/adc/Kconfig.<devname>`)

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config ADC_<DEVNAME>
	bool "<DEVNAME> ADC driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	select SPI
	help
	  Enable support for the Analog Devices <DEVNAME>
	  <resolution>-bit, <channels>-channel ADC.
```

Then add to the parent `drivers/adc/Kconfig`:

```kconfig
source "drivers/adc/Kconfig.<devname>"
```

For I2C devices, replace `select SPI` with `select I2C`.
For parts supporting both buses, use:

```kconfig
	select SPI if $(dt_compat_on_bus,$(DT_COMPAT_ADI_<DEVNAME>),spi)
	select I2C if $(dt_compat_on_bus,$(DT_COMPAT_ADI_<DEVNAME>),i2c)
```

---

## 5. CMakeLists.txt (Build System Integration)

Append to the existing `drivers/adc/CMakeLists.txt`:

```cmake
zephyr_library_sources_ifdef(CONFIG_ADC_<DEVNAME> adc_<devname>.c)
```

---

## 6. Driver Header (Optional)

If the driver is self-contained, no separate header is needed. However,
if register definitions or data types must be shared with an emulator or
test, create `include/zephyr/drivers/adc/<devname>.h`:

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ADC_<DEVNAME>_H_
#define ZEPHYR_INCLUDE_DRIVERS_ADC_<DEVNAME>_H_

/* ---------------- Register Map ------------------------------------ */
/* Adapt from the no-OS template or datasheet. */

#define <DEVNAME>_REG_STATUS		0x00
#define <DEVNAME>_REG_CONFIG		0x01
#define <DEVNAME>_REG_DATA		0x02

#define <DEVNAME>_CFG_FIELD_A_MSK	BIT(7)
#define <DEVNAME>_CFG_FIELD_B_MSK	GENMASK(6, 4)
#define <DEVNAME>_CFG_FIELD_C_MSK	GENMASK(3, 0)

#define <DEVNAME>_PRODUCT_ID		0xXX

/* Max number of ADC channels this device supports. */
#define <DEVNAME>_MAX_CHANNELS		8

/* ADC resolution in bits. */
#define <DEVNAME>_RESOLUTION		16

#endif /* ZEPHYR_INCLUDE_DRIVERS_ADC_<DEVNAME>_H_ */
```

---

## 7. Driver Source (`drivers/adc/adc_<devname>.c`)

This is the core of the driver. It follows the Zephyr ADC subsystem
contract and Linux kernel coding style.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(<devname>, CONFIG_ADC_LOG_LEVEL);

/* ---------------- Register Map ------------------------------------ */
/*
 * If a shared header was created (Section 6), include it here instead.
 * Otherwise, define registers locally.
 */

#define <DEVNAME>_REG_STATUS		0x00
#define <DEVNAME>_REG_CONFIG		0x01
#define <DEVNAME>_REG_DATA		0x02

#define <DEVNAME>_CFG_FIELD_A_MSK	BIT(7)
#define <DEVNAME>_CFG_FIELD_B_MSK	GENMASK(6, 4)
#define <DEVNAME>_CFG_FIELD_C_MSK	GENMASK(3, 0)

#define <DEVNAME>_PRODUCT_ID		0xXX

#define <DEVNAME>_MAX_CHANNELS		8
#define <DEVNAME>_RESOLUTION		16

/* ---------------- Config & Data Structs --------------------------- */

/**
 * Compile-time configuration from devicetree.
 * This struct is const and stored in ROM.
 */
struct <devname>_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec drdy_gpio;  /* Optional data-ready GPIO */
	uint32_t vref_mv;
};

/**
 * Mutable runtime data.
 * This struct is allocated statically per instance.
 */
struct <devname>_data {
	struct adc_context ctx;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *buffer_ptr;
	uint16_t channel_cfg;    /* Bitmask of configured channels */
	struct k_sem acquire_sem;
};

/* ---------------- SPI Helpers ------------------------------------- */

/**
 * @brief Read a register over SPI.
 *
 * Adapt the transaction format to match the device datasheet.
 * The example below uses a 1-byte address + 2-byte data protocol.
 */
static int <devname>_reg_read(const struct device *dev, uint8_t addr,
			      uint16_t *val)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[3] = { addr | 0x80, 0x00, 0x00 };
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

	*val = sys_get_be16(&rx_buf[1]);

	return 0;
}

/**
 * @brief Write a register over SPI.
 */
static int <devname>_reg_write(const struct device *dev, uint8_t addr,
			       uint16_t val)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[3];

	tx_buf[0] = addr & 0x7F;
	sys_put_be16(val, &tx_buf[1]);

	const struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	return spi_write_dt(&cfg->spi, &tx_set);
}

/**
 * @brief Update specific bits in a register (read-modify-write).
 */
static int <devname>_reg_update(const struct device *dev, uint8_t addr,
				uint16_t mask, uint16_t val)
{
	uint16_t reg_val;
	int ret;

	ret = <devname>_reg_read(dev, addr, &reg_val);
	if (ret < 0) {
		return ret;
	}

	reg_val = (reg_val & ~mask) | FIELD_PREP(mask, val);

	return <devname>_reg_write(dev, addr, reg_val);
}

/* ---------------- ADC Subsystem Callbacks ------------------------- */

/**
 * @brief Configure a single ADC channel.
 *
 * Called by the application via adc_channel_setup(). Validates the
 * channel configuration and stores it for later use during read.
 */
static int <devname>_channel_setup(const struct device *dev,
				   const struct adc_channel_cfg *chan_cfg)
{
	struct <devname>_data *data = dev->data;

	if (chan_cfg->channel_id >= <DEVNAME>_MAX_CHANNELS) {
		LOG_ERR("Invalid channel ID: %u", chan_cfg->channel_id);
		return -EINVAL;
	}

	if (chan_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Unsupported gain: %d", chan_cfg->gain);
		return -ENOTSUP;
	}

	if (chan_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Unsupported reference: %d", chan_cfg->reference);
		return -ENOTSUP;
	}

	/*
	 * Validate acquisition time. ADC_ACQ_TIME_DEFAULT means use
	 * the device default; reject anything else unless the device
	 * supports configurable acquisition time.
	 */
	if (chan_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Unsupported acquisition time");
		return -ENOTSUP;
	}

	/*
	 * Handle differential vs single-ended.
	 * For differential, chan_cfg->differential should be true and
	 * chan_cfg->input_positive / input_negative are set.
	 */
	if (chan_cfg->differential) {
		/* Program the device for differential input mode */
	}

	data->channel_cfg |= BIT(chan_cfg->channel_id);

	return 0;
}

/**
 * @brief Read a single sample from the specified channel.
 */
static int <devname>_read_sample(const struct device *dev, uint8_t channel,
				 uint16_t *result)
{
	int ret;

	/*
	 * Select the channel (if the device has a mux), trigger
	 * conversion, and read back the result.
	 */
	ret = <devname>_reg_update(dev, <DEVNAME>_REG_CONFIG,
				   <DEVNAME>_CFG_FIELD_C_MSK, channel);
	if (ret < 0) {
		return ret;
	}

	/* Wait for conversion to complete. Adapt timing to datasheet. */
	k_usleep(10);

	return <devname>_reg_read(dev, <DEVNAME>_REG_DATA, result);
}

/**
 * @brief Start an ADC read sequence.
 *
 * Called by the application via adc_read(). Iterates over all
 * channels in the channel mask and stores results in the buffer.
 */
static int <devname>_read(const struct device *dev,
			  const struct adc_sequence *sequence)
{
	struct <devname>_data *data = dev->data;
	uint32_t channels = sequence->channels;
	uint16_t *buf = (uint16_t *)sequence->buffer;
	uint8_t channel;
	int ret;

	if (sequence->resolution != <DEVNAME>_RESOLUTION) {
		LOG_ERR("Unsupported resolution: %u", sequence->resolution);
		return -EINVAL;
	}

	if (!channels) {
		LOG_ERR("No channels selected");
		return -EINVAL;
	}

	/* Verify all requested channels have been configured */
	if ((channels & data->channel_cfg) != channels) {
		LOG_ERR("Unconfigured channel(s) requested");
		return -EINVAL;
	}

	/* Check buffer size is sufficient */
	size_t needed = sizeof(uint16_t) * POPCOUNT(channels);

	if (sequence->buffer_size < needed) {
		LOG_ERR("Buffer too small: need %zu, have %u",
			needed, sequence->buffer_size);
		return -ENOMEM;
	}

	while (channels) {
		channel = find_lsb_set(channels) - 1;

		ret = <devname>_read_sample(dev, channel, buf);
		if (ret < 0) {
			return ret;
		}

		buf++;
		channels &= ~BIT(channel);
	}

	return 0;
}

/* ---------------- Initialization ---------------------------------- */

/**
 * @brief Verify the device identity by reading the product ID register.
 */
static int <devname>_verify_id(const struct device *dev)
{
	uint16_t id;
	int ret;

	ret = <devname>_reg_read(dev, <DEVNAME>_REG_STATUS, &id);
	if (ret < 0) {
		return ret;
	}

	if (id != <DEVNAME>_PRODUCT_ID) {
		LOG_ERR("Unexpected product ID: 0x%04X (expected 0x%04X)",
			id, <DEVNAME>_PRODUCT_ID);
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

	/* Verify SPI bus is ready */
	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Optional: configure data-ready GPIO */
	if (cfg->drdy_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->drdy_gpio)) {
			LOG_ERR("DRDY GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->drdy_gpio,
					    GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Failed to configure DRDY GPIO: %d", ret);
			return ret;
		}
	}

	/* Verify device identity */
	ret = <devname>_verify_id(dev);
	if (ret < 0) {
		return ret;
	}

	/* Apply default configuration */
	/* TODO: Write initial register settings as required by the
	 * datasheet power-up sequence.
	 */

	LOG_INF("<DEVNAME> initialized on %s", cfg->spi.bus->name);

	return 0;
}

/* ---------------- API Struct -------------------------------------- */

/*
 * Use the DEVICE_API macro. This creates a static const struct that
 * the Zephyr ADC subsystem uses to dispatch calls.
 */
static DEVICE_API(adc, <devname>_api) = {
	.channel_setup = <devname>_channel_setup,
	.read = <devname>_read,
	.ref_internal = <DEVNAME>_VREF_MV,   /* Or use DT property */
};

/*
 * Note: For async support, also set:
 *   #ifdef CONFIG_ADC_ASYNC
 *       .read_async = <devname>_read_async,
 *   #endif
 */

/* ---------------- Instance Macros --------------------------------- */

/*
 * These macros are expanded once per DT instance with status "okay".
 * They create the config, data, and DEVICE_DT_INST_DEFINE entries.
 */

#define <DEVNAME>_DRDY_GPIO_INIT(n)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, drdy_gpios),		\
		(.drdy_gpio = GPIO_DT_SPEC_INST_GET(n, drdy_gpios),),	\
		(.drdy_gpio = { 0 },))

#define <DEVNAME>_INIT(n)						\
	static struct <devname>_data <devname>_data_##n;		\
									\
	static const struct <devname>_config <devname>_config_##n = {	\
		.spi = SPI_DT_SPEC_INST_GET(n,				\
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),	\
		<DEVNAME>_DRDY_GPIO_INIT(n)				\
		.vref_mv = DT_INST_PROP(n, vref_mv),			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, <devname>_init, NULL,			\
			      &<devname>_data_##n,			\
			      &<devname>_config_##n,			\
			      POST_KERNEL,				\
			      CONFIG_ADC_INIT_PRIORITY,			\
			      &<devname>_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_INIT)
```

### Key elements explained

| Element | Purpose |
|---------|---------|
| `DT_DRV_COMPAT` | Must match the compatible string with commas replaced by underscores |
| `struct <devname>_config` | Compile-time config from DT (stored in ROM) |
| `struct <devname>_data` | Mutable runtime state (one per instance) |
| `DEVICE_API(adc, ...)` | Typed API struct -- replaces the old untyped `struct adc_driver_api` literal |
| `SPI_DT_SPEC_INST_GET()` | Pulls SPI bus, CS, and frequency from DT |
| `GPIO_DT_SPEC_INST_GET()` | Pulls GPIO port/pin/flags from DT |
| `DEVICE_DT_INST_DEFINE()` | Registers the device with Zephyr's device model |
| `DT_INST_FOREACH_STATUS_OKAY()` | Instantiates one driver per DT node with `status = "okay"` |
| `LOG_MODULE_REGISTER()` | Creates a logging module; level controlled by `CONFIG_ADC_LOG_LEVEL` |

---

## 8. Trigger / Interrupt Support (Optional)

If the device has a data-ready (DRDY) interrupt output, implement a
GPIO callback to signal conversion completion instead of polling.

```c
/* Add to data struct: */
struct <devname>_data {
	/* ... existing fields ... */
	struct gpio_callback drdy_cb;
	struct k_sem drdy_sem;
};

/* Interrupt handler: */
static void <devname>_drdy_handler(const struct device *port,
				   struct gpio_callback *cb,
				   gpio_port_pins_t pins)
{
	struct <devname>_data *data =
		CONTAINER_OF(cb, struct <devname>_data, drdy_cb);

	k_sem_give(&data->drdy_sem);
}

/* In init(), set up the interrupt: */
static int <devname>_init_drdy(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	if (cfg->drdy_gpio.port == NULL) {
		return 0;  /* No DRDY GPIO configured */
	}

	k_sem_init(&data->drdy_sem, 0, 1);

	gpio_init_callback(&data->drdy_cb, <devname>_drdy_handler,
			   BIT(cfg->drdy_gpio.pin));

	ret = gpio_add_callback(cfg->drdy_gpio.port, &data->drdy_cb);
	if (ret < 0) {
		return ret;
	}

	return gpio_pin_interrupt_configure_dt(&cfg->drdy_gpio,
					       GPIO_INT_EDGE_TO_ACTIVE);
}

/* In read, wait for interrupt instead of polling: */
static int <devname>_wait_drdy(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;

	if (cfg->drdy_gpio.port == NULL) {
		/* Polling fallback */
		k_usleep(10);
		return 0;
	}

	return k_sem_take(&data->drdy_sem, K_MSEC(1000));
}
```

---

## 9. Test Skeleton

### 9.1 Test Metadata (`tests/drivers/adc/<devname>/testcase.yaml`)

```yaml
tests:
  drivers.adc.<devname>:
    tags:
      - drivers
      - adc
    depends_on: spi
    platform_allow: native_sim
    integration_platforms:
      - native_sim
```

### 9.2 Test Project Config (`tests/drivers/adc/<devname>/prj.conf`)

```
CONFIG_ZTEST=y
CONFIG_ADC=y
CONFIG_ADC_<DEVNAME>=y
CONFIG_SPI=y
CONFIG_LOG=y
```

### 9.3 DT Overlay (`tests/drivers/adc/<devname>/boards/native_sim.overlay`)

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
		#io-channel-cells = <1>;
		vref-mv = <2500>;
	};
};
```

### 9.4 Test Source (`tests/drivers/adc/<devname>/src/main.c`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>

#define ADC_NODE DT_NODELABEL(<devname>_test)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

#define ADC_CHANNEL_ID   0
#define ADC_RESOLUTION   16

static struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN_1,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_CHANNEL_ID,
};

ZTEST(adc_<devname>, test_device_ready)
{
	zassert_true(device_is_ready(adc_dev),
		     "ADC device not ready");
}

ZTEST(adc_<devname>, test_channel_setup)
{
	int ret;

	ret = adc_channel_setup(adc_dev, &channel_cfg);
	zassert_ok(ret, "adc_channel_setup failed: %d", ret);
}

ZTEST(adc_<devname>, test_read)
{
	int ret;
	int16_t buffer;

	ret = adc_channel_setup(adc_dev, &channel_cfg);
	zassert_ok(ret, "adc_channel_setup failed: %d", ret);

	struct adc_sequence seq = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = &buffer,
		.buffer_size = sizeof(buffer),
		.resolution = ADC_RESOLUTION,
	};

	ret = adc_read(adc_dev, &seq);
	zassert_ok(ret, "adc_read failed: %d", ret);
}

ZTEST(adc_<devname>, test_invalid_channel)
{
	struct adc_channel_cfg bad_cfg = channel_cfg;
	int ret;

	bad_cfg.channel_id = 255;  /* Invalid */

	ret = adc_channel_setup(adc_dev, &bad_cfg);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for invalid channel, got %d", ret);
}

ZTEST(adc_<devname>, test_unsupported_resolution)
{
	int ret;
	int16_t buffer;

	ret = adc_channel_setup(adc_dev, &channel_cfg);
	zassert_ok(ret, "adc_channel_setup failed: %d", ret);

	struct adc_sequence seq = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = &buffer,
		.buffer_size = sizeof(buffer),
		.resolution = 8,  /* Unsupported */
	};

	ret = adc_read(adc_dev, &seq);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for bad resolution, got %d", ret);
}

ZTEST_SUITE(adc_<devname>, NULL, NULL, NULL, NULL, NULL);
```

### 9.5 SPI Emulator Pattern (Optional)

For proper hardware-in-the-loop testing on `native_sim`, create an SPI
emulator that responds to the device's register protocol:

```c
/*
 * tests/drivers/adc/<devname>/src/<devname>_emul.c
 *
 * Minimal SPI emulator for <DEVNAME> tests.
 */

#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/spi_emul.h>

struct <devname>_emul_data {
	uint16_t regs[256];
};

static int <devname>_emul_io(const struct emul *target,
			     const struct spi_config *config,
			     const struct spi_buf_set *tx_bufs,
			     const struct spi_buf_set *rx_bufs)
{
	struct <devname>_emul_data *data = target->data;
	const uint8_t *tx = tx_bufs->buffers[0].buf;
	uint8_t *rx = rx_bufs ? rx_bufs->buffers[0].buf : NULL;
	uint8_t addr = tx[0] & 0x7F;
	bool is_read = (tx[0] & 0x80) != 0;

	if (is_read && rx) {
		sys_put_be16(data->regs[addr], &rx[1]);
	} else {
		data->regs[addr] = sys_get_be16(&tx[1]);
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
	data->regs[0x00] = <DEVNAME>_PRODUCT_ID;

	return 0;
}

#define <DEVNAME>_EMUL_INIT(n)						\
	static struct <devname>_emul_data <devname>_emul_data_##n;	\
	EMUL_DT_INST_DEFINE(n, <devname>_emul_init, &<devname>_emul_data_##n, \
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

8. **Logging** -- use `LOG_MODULE_REGISTER(<devname>, CONFIG_ADC_LOG_LEVEL)`.
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

12. **DEVICE_API macro** -- always use `DEVICE_API(adc, <devname>_api)`
    for the API struct declaration, not a raw `struct adc_driver_api`
    assignment.

13. **Init priority** -- use `CONFIG_ADC_INIT_PRIORITY` (default 80)
    unless the driver depends on another subsystem that initializes
    later.

14. **Thread safety** -- if the driver can be called from multiple
    threads, protect shared state with `k_mutex` or `k_sem`. The ADC
    context helpers (`adc_context_*`) handle this for drivers that use
    them.

---

## 11. Commit Message Format

Zephyr follows a strict commit message format. Each commit must have a
subsystem prefix, a short subject, and an informative body.

### Adding a new driver (typically 3-4 commits):

```
# Commit 1: Devicetree binding
dts: bindings: add binding for Analog Devices <DEVNAME>

Add devicetree binding for the Analog Devices <DEVNAME>
<resolution>-bit, <channels>-channel ADC with SPI interface.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 2: Driver implementation
drivers: adc: add Analog Devices <DEVNAME> driver

Add support for the Analog Devices <DEVNAME> <resolution>-bit,
<channels>-channel successive approximation ADC. The driver
implements the Zephyr ADC subsystem API with support for
single-ended and differential input modes.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 3: Tests
tests: drivers: adc: add tests for <DEVNAME>

Add unit tests for the <DEVNAME> ADC driver covering channel
setup, basic read, and error handling.

Signed-off-by: Your Name <your.name@analog.com>
```

### Key commit message rules:

- Subject line: max 72 characters, no trailing period
- Prefix: matches the path (e.g., `drivers: adc:`, `dts: bindings:`)
- Body: wrapped at 75 characters, explains the "why"
- Must include `Signed-off-by:` (DCO requirement)
- Use imperative mood ("add", not "added" or "adds")
