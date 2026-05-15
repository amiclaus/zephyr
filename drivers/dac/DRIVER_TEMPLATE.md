# Zephyr RTOS DAC Driver Template

Reference driver: `zephyr/drivers/dac/dac_ad56xx.c`

This template covers every file needed to add a new DAC driver to the
Zephyr RTOS for an Analog Devices part.  Replace `<devname>` with the
lowercase part number (e.g., `ad56xx`), `<DEVNAME>` with its uppercase
form (e.g., `AD56XX`), and `<DevName>` with the mixed-case form used in
Kconfig (e.g., `AD56XX`).

---

## 1. Purpose & Zephyr Subsystem Mapping

The Zephyr DAC subsystem (`include/zephyr/drivers/dac.h`) defines a
small, channel-oriented API:

| Zephyr API function      | Purpose                                        |
|--------------------------|-------------------------------------------------|
| `dac_channel_setup()`    | Configure resolution and channel ID             |
| `dac_write_value()`      | Write a digital code to a DAC channel           |

A Zephyr DAC driver implements these two callbacks (plus `init`) and
registers them with `DEVICE_API(dac, ...)`.  The subsystem also exposes
a `struct dac_channel_cfg` that carries the channel ID and resolution
requested by the application.

**No-OS to Zephyr mapping:**

| no-OS concept              | Zephyr equivalent                          |
|----------------------------|--------------------------------------------|
| `<devname>_init()`         | `<devname>_init()` called via `DEVICE_DT_INST_DEFINE` |
| `<devname>_write_update()` | `dac_api->write_value()`                   |
| `<devname>_init_param`     | Devicetree properties + `struct <devname>_config` |
| `<devname>_dev`            | `struct <devname>_data` (mutable runtime state) |
| SPI/I2C descriptors        | `spi_dt_spec` / `i2c_dt_spec` from DT      |
| GPIO descriptors           | `gpio_dt_spec` from DT                     |
| IIO subsystem              | Not applicable (Zephyr uses its own sensor/DAC API) |

---

## 2. File Checklist

```
zephyr/drivers/dac/
    dac_<devname>.c              # Driver source
    Kconfig.<devname>            # Kconfig fragment (sourced by parent Kconfig)

zephyr/drivers/dac/CMakeLists.txt   # Add zephyr_library_sources_ifdef() line
zephyr/drivers/dac/Kconfig          # Add source "Kconfig.<devname>" line

zephyr/dts/bindings/dac/
    adi,<devname>.yaml           # Devicetree binding

zephyr/tests/drivers/dac/<devname>/
    testcase.yaml
    prj.conf
    boards/<board>.overlay       # (optional, for on-target testing)
    src/main.c                   # Test source
```

---

## 3. Devicetree Binding (`adi,<devname>.yaml`)

Path: `zephyr/dts/bindings/dac/adi,<devname>.yaml`

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Analog Devices <DEVNAME> <resolution>-bit, <channels>-channel
  voltage-output DAC with <SPI/I2C> interface.

  Example devicetree node (SPI):

    &spi1 {
        dac0: <devname>@0 {
            compatible = "adi,<devname>";
            reg = <0>;
            spi-max-frequency = <10000000>;
            /* Optional GPIO properties */
            reset-gpios = <&gpio0 4 GPIO_ACTIVE_LOW>;
            ldac-gpios  = <&gpio0 5 GPIO_ACTIVE_LOW>;
            #io-channel-cells = <1>;
        };
    };

compatible: "adi,<devname>"

include: [spi-device.yaml]
# For I2C parts use: include: [i2c-device.yaml]

properties:
  "#io-channel-cells":
    const: 1

  reset-gpios:
    type: phandle-array
    description: |
      GPIO connected to the RESET pin (active low).
      If present the driver will de-assert reset during init.

  ldac-gpios:
    type: phandle-array
    description: |
      GPIO connected to the LDAC pin (active low).
      If present the driver will hold LDAC high so that
      outputs update only on explicit software update commands.
```

**Notes:**
- For parts that also support I2C, create a second binding
  `adi,<devname>-i2c.yaml` that includes `i2c-device.yaml` instead, or
  use a single binding with a bus selector.
- `#io-channel-cells` is conventionally `1` (the channel number).

---

## 4. Kconfig (`Kconfig.<devname>`)

Path: `zephyr/drivers/dac/Kconfig.<devname>`

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config DAC_<DEVNAME>
	bool "<DEVNAME> DAC driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	select SPI
	# For I2C parts use: select I2C
	help
	  Enable the driver for the Analog Devices <DEVNAME>
	  <resolution>-bit, <channels>-channel voltage output DAC.
```

Then add to the parent `zephyr/drivers/dac/Kconfig`:

```kconfig
source "drivers/dac/Kconfig.<devname>"
```

---

## 5. CMakeLists.txt

Add the following line to `zephyr/drivers/dac/CMakeLists.txt`:

```cmake
zephyr_library_sources_ifdef(CONFIG_DAC_<DEVNAME> dac_<devname>.c)
```

---

## 6. Driver Header (`dac_<devname>.h`) -- optional

Most Zephyr DAC drivers keep everything in the `.c` file (config and
data structs are `static`).  A separate header is only needed when
other drivers or subsystems must reference chip-specific types.

If needed, place it at `zephyr/drivers/dac/dac_<devname>.h`:

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_DAC_DAC_<DEVNAME>_H_
#define ZEPHYR_DRIVERS_DAC_DAC_<DEVNAME>_H_

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
/* For I2C parts: #include <zephyr/drivers/i2c.h> */
#include <zephyr/drivers/gpio.h>

/* ---- Register / command definitions ------------------------------- */

#define <DEVNAME>_CMD_NOP               0x0
#define <DEVNAME>_CMD_WRITE_INPUT_REG   0x1
#define <DEVNAME>_CMD_UPDATE_DAC_REG    0x2
#define <DEVNAME>_CMD_WRITE_AND_UPDATE  0x3
#define <DEVNAME>_CMD_POWER_UPDOWN      0x4
#define <DEVNAME>_CMD_SW_RESET          0x6

/* ---- Per-instance config (from devicetree, const) ----------------- */

struct <devname>_config {
	/** SPI bus specification from devicetree. */
	struct spi_dt_spec spi;
	/* For I2C parts: struct i2c_dt_spec i2c; */
	/** Optional reset GPIO. */
	struct gpio_dt_spec reset_gpio;
	/** Optional LDAC GPIO. */
	struct gpio_dt_spec ldac_gpio;
	/** Number of channels this variant supports. */
	uint8_t num_channels;
	/** DAC resolution in bits. */
	uint8_t resolution;
};

/* ---- Per-instance mutable data ------------------------------------ */

struct <devname>_data {
	/** Bitmask of channels that have been configured via channel_setup. */
	uint8_t configured_channels;
};

#endif /* ZEPHYR_DRIVERS_DAC_DAC_<DEVNAME>_H_ */
```

---

## 7. Driver Source (`dac_<devname>.c`)

Path: `zephyr/drivers/dac/dac_<devname>.c`

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>
/* NOTE: Replace hyphens in the compatible string with underscores.
 * E.g., compatible = "adi,ad5686" => DT_DRV_COMPAT = adi_ad5686
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/spi.h>
/* For I2C parts: #include <zephyr/drivers/i2c.h> */
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(<devname>, CONFIG_DAC_LOG_LEVEL);

/* ---- Register / command definitions ------------------------------- */

#define <DEVNAME>_CMD_NOP               0x0
#define <DEVNAME>_CMD_WRITE_INPUT_REG   0x1
#define <DEVNAME>_CMD_UPDATE_DAC_REG    0x2
#define <DEVNAME>_CMD_WRITE_AND_UPDATE  0x3
#define <DEVNAME>_CMD_POWER_UPDOWN      0x4
#define <DEVNAME>_CMD_LDAC_MASK         0x5
#define <DEVNAME>_CMD_SW_RESET          0x6
#define <DEVNAME>_CMD_INT_REF           0x7

#define <DEVNAME>_MAX_RESOLUTION        16

/* ---- Per-instance config (from devicetree, const) ----------------- */

struct <devname>_config {
	struct spi_dt_spec spi;
	/* For I2C parts: struct i2c_dt_spec i2c; */
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec ldac_gpio;
	uint8_t num_channels;
	uint8_t resolution;
};

/* ---- Per-instance mutable data ------------------------------------ */

struct <devname>_data {
	uint8_t configured_channels;
};

/* ---- Low-level SPI transfer --------------------------------------- */

/**
 * @brief Write a 3-byte command frame to the DAC via SPI.
 *
 * Packet format (24 bits):
 *   [CMD(4) | ADDR(4) | DATA(16)]
 *
 * @param dev    Zephyr device pointer.
 * @param cmd    Command nibble.
 * @param addr   Channel address nibble.
 * @param data   16-bit data word.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_write_cmd(const struct device *dev, uint8_t cmd,
			       uint8_t addr, uint16_t data)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[3];

	tx_buf[0] = ((cmd & 0x0F) << 4) | (addr & 0x0F);
	tx_buf[1] = (uint8_t)(data >> 8);
	tx_buf[2] = (uint8_t)(data & 0xFF);

	const struct spi_buf buf = {
		.buf = tx_buf,
		.len = sizeof(tx_buf),
	};
	const struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1,
	};

	return spi_write_dt(&cfg->spi, &tx);
}

/* For I2C parts, replace with:
 *
 * static int <devname>_write_cmd(const struct device *dev, uint8_t cmd,
 *                                uint8_t addr, uint16_t data)
 * {
 *     const struct <devname>_config *cfg = dev->config;
 *     uint8_t tx_buf[3];
 *
 *     tx_buf[0] = ((cmd & 0x0F) << 4) | (addr & 0x0F);
 *     tx_buf[1] = (uint8_t)(data >> 8);
 *     tx_buf[2] = (uint8_t)(data & 0xFF);
 *
 *     return i2c_write_dt(&cfg->i2c, tx_buf, sizeof(tx_buf));
 * }
 */

/* ---- DAC API: channel_setup -------------------------------------- */

/**
 * @brief Configure a DAC channel (Zephyr DAC API callback).
 *
 * Validates the channel ID and requested resolution against the
 * hardware capabilities.  No actual register write is needed for
 * most external DAC chips -- the resolution is fixed in hardware.
 *
 * @param dev         Zephyr device pointer.
 * @param channel_cfg Channel configuration (channel ID + resolution).
 * @return 0 on success, -ENOTSUP / -EINVAL on error.
 */
static int <devname>_channel_setup(const struct device *dev,
				   const struct dac_channel_cfg *channel_cfg)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;

	if (channel_cfg->channel_id >= cfg->num_channels) {
		LOG_ERR("Invalid channel %u (max %u)",
			channel_cfg->channel_id, cfg->num_channels - 1);
		return -EINVAL;
	}

	if (channel_cfg->resolution != cfg->resolution) {
		LOG_ERR("Unsupported resolution %u (device supports %u)",
			channel_cfg->resolution, cfg->resolution);
		return -ENOTSUP;
	}

	data->configured_channels |= BIT(channel_cfg->channel_id);

	LOG_DBG("Channel %u configured, resolution %u bits",
		channel_cfg->channel_id, channel_cfg->resolution);

	return 0;
}

/* ---- DAC API: write_value ---------------------------------------- */

/**
 * @brief Write a value to a DAC channel (Zephyr DAC API callback).
 *
 * The value is left-aligned (MSB-aligned) in the data field when
 * the part resolution is less than 16 bits.
 *
 * @param dev     Zephyr device pointer.
 * @param channel Channel number (0-based).
 * @param value   Digital code to output.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_write_value(const struct device *dev, uint8_t channel,
				 uint32_t value)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;

	if (channel >= cfg->num_channels) {
		LOG_ERR("Invalid channel %u", channel);
		return -EINVAL;
	}

	if (!(data->configured_channels & BIT(channel))) {
		LOG_ERR("Channel %u not configured", channel);
		return -EINVAL;
	}

	if (value >= (1U << cfg->resolution)) {
		LOG_ERR("Value %u exceeds %u-bit range", value,
			cfg->resolution);
		return -EINVAL;
	}

	/*
	 * Left-align the value in the 16-bit data field.
	 * Example: 12-bit DAC => shift left by 4.
	 */
	uint8_t shift = <DEVNAME>_MAX_RESOLUTION - cfg->resolution;
	uint16_t aligned_value = (uint16_t)(value << shift);

	/*
	 * Channel address encoding is device-specific.
	 * Common pattern: channel 0 => addr 0x1, ch 1 => 0x2, etc.
	 * Adjust per your datasheet.
	 */
	uint8_t addr = BIT(channel);

	return <devname>_write_cmd(dev, <DEVNAME>_CMD_WRITE_AND_UPDATE,
				   addr, aligned_value);
}

/* ---- DAC API table ------------------------------------------------ */

static DEVICE_API(dac, <devname>_api) = {
	.channel_setup = <devname>_channel_setup,
	.write_value   = <devname>_write_value,
};

/* ---- Hardware init ------------------------------------------------ */

/**
 * @brief Initialise the DAC hardware (called once at boot by the
 *        device model).
 *
 * Checks bus readiness, configures optional GPIOs (reset, LDAC),
 * and performs a software reset.
 *
 * @param dev Zephyr device pointer.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_init(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	int ret;

	/* Verify the SPI bus is ready. */
	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}
	/* For I2C parts:
	 * if (!i2c_is_ready_dt(&cfg->i2c)) {
	 *     LOG_ERR("I2C bus not ready");
	 *     return -ENODEV;
	 * }
	 */

	/* Configure optional reset GPIO. */
	if (cfg->reset_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->reset_gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure reset GPIO: %d", ret);
			return ret;
		}

		/* Pulse reset: assert, wait, de-assert. */
		gpio_pin_set_dt(&cfg->reset_gpio, 1);
		k_usleep(100);
		gpio_pin_set_dt(&cfg->reset_gpio, 0);
		k_usleep(100);
	}

	/* Configure optional LDAC GPIO. */
	if (cfg->ldac_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->ldac_gpio)) {
			LOG_ERR("LDAC GPIO not ready");
			return -ENODEV;
		}

		/* Hold LDAC high so outputs update only via software. */
		ret = gpio_pin_configure_dt(&cfg->ldac_gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure LDAC GPIO: %d", ret);
			return ret;
		}
	}

	/* Issue a software reset. */
	ret = <devname>_write_cmd(dev, <DEVNAME>_CMD_SW_RESET, 0, 0);
	if (ret < 0) {
		LOG_ERR("Software reset failed: %d", ret);
		return ret;
	}

	LOG_INF("<DEVNAME> initialised");

	return 0;
}

/* ---- Device instantiation macros ---------------------------------- */

/*
 * The DT_INST_FOREACH_STATUS_OKAY block creates one config + data
 * pair for every enabled devicetree node with a matching compatible
 * string.  No dynamic memory allocation is used.
 */

#define <DEVNAME>_INIT(inst)                                                \
	static struct <devname>_data <devname>_data_##inst;                 \
                                                                            \
	static const struct <devname>_config <devname>_config_##inst = {    \
		.spi = SPI_DT_SPEC_INST_GET(                                \
			inst,                                                \
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB,                  \
			0),                                                  \
		/* For I2C parts replace .spi with:                          \
		 * .i2c = I2C_DT_SPEC_INST_GET(inst),                       \
		 */                                                          \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios,   \
						       {0}),                 \
		.ldac_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, ldac_gpios,     \
						      {0}),                  \
		.num_channels = 4,  /* Adjust per variant */                 \
		.resolution = 16,   /* Adjust per variant */                 \
	};                                                                   \
                                                                            \
	DEVICE_DT_INST_DEFINE(inst,                                          \
			      <devname>_init,                                \
			      NULL,   /* pm_action_cb (NULL if no PM) */     \
			      &<devname>_data_##inst,                        \
			      &<devname>_config_##inst,                      \
			      POST_KERNEL,                                   \
			      CONFIG_DAC_INIT_PRIORITY,                      \
			      &<devname>_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_INIT)
```

**Multi-variant support:**

When a single driver covers multiple compatible strings (e.g.,
`ad5624`, `ad5644`, `ad5664`), add an `enum` for the variant and
store the variant ID in `struct <devname>_config`:

```c
enum <devname>_type {
	<DEVNAME>_TYPE_12BIT_3CH,
	<DEVNAME>_TYPE_14BIT_3CH,
	<DEVNAME>_TYPE_16BIT_3CH,
};

struct <devname>_type_info {
	uint8_t resolution;
	uint8_t num_channels;
};

static const struct <devname>_type_info <devname>_type_table[] = {
	[<DEVNAME>_TYPE_12BIT_3CH] = { .resolution = 12, .num_channels = 3 },
	[<DEVNAME>_TYPE_14BIT_3CH] = { .resolution = 14, .num_channels = 3 },
	[<DEVNAME>_TYPE_16BIT_3CH] = { .resolution = 16, .num_channels = 3 },
};
```

Then use `DT_INST_ENUM_IDX()` or additional compatibles with
separate `DT_DRV_COMPAT` blocks.

---

## 8. Trigger / Interrupt Support

Most DAC parts do not generate interrupts.  If your part has an
ALERT or FAULT output (e.g., over-temperature, open-circuit
detection), wire it as follows:

```c
/* In the config struct: */
struct gpio_dt_spec alert_gpio;

/* In init: */
if (cfg->alert_gpio.port != NULL) {
	if (!gpio_is_ready_dt(&cfg->alert_gpio)) {
		LOG_ERR("Alert GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->alert_gpio, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&data->alert_cb, <devname>_alert_handler,
			   BIT(cfg->alert_gpio.pin));

	ret = gpio_add_callback(cfg->alert_gpio.port, &data->alert_cb);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->alert_gpio,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		return ret;
	}
}

/* Callback handler: */
static void <devname>_alert_handler(const struct device *gpio_dev,
				    struct gpio_callback *cb,
				    gpio_port_pins_t pins)
{
	struct <devname>_data *data =
		CONTAINER_OF(cb, struct <devname>_data, alert_cb);

	LOG_WRN("<DEVNAME> alert triggered");
	/* Schedule work item or set flag for deferred processing. */
}
```

Add to the binding:

```yaml
  alert-gpios:
    type: phandle-array
    description: GPIO connected to the ALERT/FAULT pin (active low).
```

For parts with an LDAC trigger (hardware-triggered simultaneous
update), the LDAC GPIO configured in Section 3 is toggled low
momentarily:

```c
static int <devname>_trigger_ldac(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;

	if (cfg->ldac_gpio.port == NULL) {
		return -ENOTSUP;
	}

	gpio_pin_set_dt(&cfg->ldac_gpio, 1);
	k_usleep(1);
	gpio_pin_set_dt(&cfg->ldac_gpio, 0);

	return 0;
}
```

---

## 9. Test Skeleton

Path: `zephyr/tests/drivers/dac/<devname>/`

### 9.1 `testcase.yaml`

```yaml
tests:
  drivers.dac.<devname>:
    tags:
      - drivers
      - dac
    depends_on: spi
    # For I2C parts: depends_on: i2c
    harness: ztest
    platform_allow:
      - native_sim
```

### 9.2 `prj.conf`

```ini
CONFIG_ZTEST=y
CONFIG_DAC=y
CONFIG_DAC_<DEVNAME>=y
CONFIG_SPI=y
# For I2C parts: CONFIG_I2C=y
CONFIG_LOG=y
```

### 9.3 `boards/native_sim.overlay`

```dts
/ {
	/* Fake SPI controller for native_sim testing. */
	test_spi: spi@0 {
		compatible = "vnd,spi";
		reg = <0x0 0x1000>;
		#address-cells = <1>;
		#size-cells = <0>;
		status = "okay";
		clock-frequency = <1000000>;

		test_dac: <devname>@0 {
			compatible = "adi,<devname>";
			reg = <0>;
			spi-max-frequency = <10000000>;
			#io-channel-cells = <1>;
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
#include <zephyr/drivers/dac.h>
#include <zephyr/ztest.h>

#define DAC_NODE DT_NODELABEL(test_dac)

static const struct device *get_dac_device(void)
{
	const struct device *dev = DEVICE_DT_GET(DAC_NODE);

	zassert_true(device_is_ready(dev), "DAC device not ready");
	return dev;
}

ZTEST(dac_<devname>, test_channel_setup)
{
	const struct device *dev = get_dac_device();
	struct dac_channel_cfg ch_cfg = {
		.channel_id  = 0,
		.resolution  = 16, /* Adjust per variant */
	};
	int ret;

	ret = dac_channel_setup(dev, &ch_cfg);
	zassert_equal(ret, 0, "channel_setup failed: %d", ret);
}

ZTEST(dac_<devname>, test_channel_setup_invalid_channel)
{
	const struct device *dev = get_dac_device();
	struct dac_channel_cfg ch_cfg = {
		.channel_id  = 99, /* Out of range */
		.resolution  = 16,
	};
	int ret;

	ret = dac_channel_setup(dev, &ch_cfg);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for invalid channel, got %d", ret);
}

ZTEST(dac_<devname>, test_channel_setup_invalid_resolution)
{
	const struct device *dev = get_dac_device();
	struct dac_channel_cfg ch_cfg = {
		.channel_id  = 0,
		.resolution  = 8, /* Unsupported */
	};
	int ret;

	ret = dac_channel_setup(dev, &ch_cfg);
	zassert_equal(ret, -ENOTSUP,
		      "Expected -ENOTSUP for bad resolution, got %d", ret);
}

ZTEST(dac_<devname>, test_write_value)
{
	const struct device *dev = get_dac_device();
	struct dac_channel_cfg ch_cfg = {
		.channel_id  = 0,
		.resolution  = 16,
	};
	int ret;

	ret = dac_channel_setup(dev, &ch_cfg);
	zassert_equal(ret, 0, "channel_setup failed: %d", ret);

	/* Write mid-scale */
	ret = dac_write_value(dev, 0, 0x8000);
	zassert_equal(ret, 0, "write_value failed: %d", ret);
}

ZTEST(dac_<devname>, test_write_unconfigured_channel)
{
	const struct device *dev = get_dac_device();
	int ret;

	/*
	 * Attempt to write to channel 1 without calling
	 * channel_setup first.
	 */
	ret = dac_write_value(dev, 1, 0x1000);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for unconfigured channel, got %d",
		      ret);
}

ZTEST_SUITE(dac_<devname>, NULL, NULL, NULL, NULL, NULL);
```

---

## 10. Key Conventions

1. **`DEVICE_API(dac, ...)`** -- Always use the `DEVICE_API` macro to
   declare the API struct.  This enables compile-time type checking
   against `struct dac_driver_api`.

2. **`spi_dt_spec` / `i2c_dt_spec`** -- Never manually fill bus
   parameters.  Use `SPI_DT_SPEC_INST_GET()` or
   `I2C_DT_SPEC_INST_GET()` to extract them from the devicetree.

3. **`gpio_dt_spec`** -- Use `GPIO_DT_SPEC_INST_GET_OR()` for
   optional GPIOs (reset, LDAC, gain).  The `_OR` variant provides a
   fallback so the driver compiles even when the property is absent
   in the DTS.

4. **`LOG_MODULE_REGISTER()`** -- Register the log module at the top
   of the `.c` file using the parent subsystem's log level
   (`CONFIG_DAC_LOG_LEVEL`).

5. **No dynamic allocation** -- All config and data structs are
   statically instantiated via `DT_INST_FOREACH_STATUS_OKAY()`.
   Never call `k_malloc()` in a DAC driver.

6. **Bus readiness checks** -- Always verify
   `spi_is_ready_dt()` / `i2c_is_ready_dt()` in `init()` before
   performing any transfers.

7. **Init priority** -- Use `CONFIG_DAC_INIT_PRIORITY` (default 80)
   so the DAC initialises after the SPI/I2C controller (priority 70).

8. **`DT_DRV_COMPAT`** -- Must match the compatible string in the
   binding with dots and hyphens replaced by underscores
   (`"adi,ad5686"` becomes `adi_ad5686`).

9. **Error codes** -- Return standard negative `errno` values
   (`-EINVAL`, `-ENOTSUP`, `-ENODEV`, `-EIO`).  Never return
   positive error codes.

10. **Coding style** -- Follow the Zephyr coding style:
    - Tabs for indentation (not spaces).
    - Opening brace on the same line for functions.
    - K&R braces for `if`/`for`/`while`.
    - Line length limit: 100 columns.
    - Zephyr uses `clang-format` with the project `.clang-format`.

11. **SPDX headers** -- Every file must have a single-line
    `/* SPDX-License-Identifier: Apache-2.0 */` near the top.
    Zephyr uses Apache-2.0 (not the BSD-3-Clause used by no-OS).

12. **PM (power management)** -- Pass `NULL` as the `pm_action_cb`
    argument to `DEVICE_DT_INST_DEFINE()` unless the part supports
    hardware power states.  If it does, implement
    `<devname>_pm_action()` handling `PM_DEVICE_ACTION_SUSPEND` and
    `PM_DEVICE_ACTION_RESUME`.

---

## 11. Commit Message Format

Zephyr uses a prefix-based commit message convention.  The subsystem
prefix for DAC drivers is `drivers: dac:`.

### Adding a new driver

```
drivers: dac: add support for <DEVNAME>

Add a Zephyr DAC driver for the Analog Devices <DEVNAME>,
a <resolution>-bit, <channels>-channel voltage output DAC
with <SPI/I2C> interface.

The driver implements the standard DAC API callbacks
(channel_setup, write_value) and supports optional reset
and LDAC GPIOs from devicetree.

Signed-off-by: Your Name <your.name@analog.com>
```

### Adding the devicetree binding

```
dts: bindings: add binding for adi,<devname>

Add a devicetree binding for the Analog Devices <DEVNAME>
<SPI/I2C> DAC. The binding defines optional reset-gpios
and ldac-gpios properties.

Signed-off-by: Your Name <your.name@analog.com>
```

### Adding tests

```
tests: drivers: dac: add tests for <DEVNAME>

Add a ztest-based test suite for the <DEVNAME> DAC driver
covering channel_setup validation, write_value, and error
paths.

Signed-off-by: Your Name <your.name@analog.com>
```

**Note:** In a Zephyr contribution, the binding, driver, and tests are
typically submitted as separate commits in a single PR, or as a single
commit if the changeset is small.  Check the Zephyr contribution
guidelines for current preferences.
