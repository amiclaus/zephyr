# Zephyr RTOS ADC-DAC (Multi-Function) Driver Template

Reference no-OS driver: `drivers/adc-dac/ad5592r/`

This template covers every file needed to port a combined ADC/DAC driver
(like the AD5592R/AD5593R) to Zephyr RTOS. These devices feature
configurable pins that can individually operate as ADC inputs, DAC
outputs, or GPIOs, and typically support both SPI and I2C interfaces.

The recommended Zephyr architecture uses the **MFD (Multi-Function
Device)** pattern: a parent MFD driver owns the bus communication and
register access, while child drivers register with the ADC, DAC, and
GPIO subsystems independently. This avoids the need to wedge a
multi-subsystem device into a single API and matches the Linux kernel
IIO approach used by the no-OS driver.

Replace `<devname>` with the lowercase base part number (e.g.,
`ad5592r`), `<DEVNAME>` with uppercase (e.g., `AD5592R`),
`<devname_spi>` with the SPI variant (e.g., `ad5592r`), and
`<devname_i2c>` with the I2C variant (e.g., `ad5593r`) throughout.

---

## 1. Naming Conventions

| Item | Pattern | Example |
|---|---|---|
| MFD parent dir | `drivers/mfd/<devname>/` | `drivers/mfd/ad5592r/` |
| MFD source | `<devname>.c` | `ad5592r.c` |
| MFD header | `<devname>.h` | `ad5592r.h` |
| ADC child dir | `drivers/adc/` | `drivers/adc/` |
| ADC child source | `adc_<devname>.c` | `adc_ad5592r.c` |
| DAC child dir | `drivers/dac/` | `drivers/dac/` |
| DAC child source | `dac_<devname>.c` | `dac_ad5592r.c` |
| GPIO child dir | `drivers/gpio/` | `drivers/gpio/` |
| GPIO child source | `gpio_<devname>.c` | `gpio_ad5592r.c` |
| DT compatible (MFD, SPI) | `"adi,<devname_spi>"` | `"adi,ad5592r"` |
| DT compatible (MFD, I2C) | `"adi,<devname_i2c>"` | `"adi,ad5593r"` |
| DT compatible (ADC child) | `"adi,<devname>-adc"` | `"adi,ad5592r-adc"` |
| DT compatible (DAC child) | `"adi,<devname>-dac"` | `"adi,ad5592r-dac"` |
| DT compatible (GPIO child) | `"adi,<devname>-gpio"` | `"adi,ad5592r-gpio"` |
| Kconfig (MFD) | `MFD_<DEVNAME>` | `MFD_AD5592R` |
| Kconfig (ADC) | `ADC_<DEVNAME>` | `ADC_AD5592R` |
| Kconfig (DAC) | `DAC_<DEVNAME>` | `DAC_AD5592R` |
| Kconfig (GPIO) | `GPIO_<DEVNAME>` | `GPIO_AD5592R` |
| Config struct (MFD) | `<devname>_config` | `ad5592r_config` |
| Data struct (MFD) | `<devname>_data` | `ad5592r_data` |
| Register defines | `<DEVNAME>_REG_<NAME>` | `AD5592R_REG_CTRL` |
| Field masks | `<DEVNAME>_<FIELD>_MSK` | `AD5592R_CTRL_ADC_RANGE_MSK` |
| Channel modes | `<DEVNAME>_CH_MODE_<MODE>` | `AD5592R_CH_MODE_ADC` |

---

## 2. File Checklist

```
zephyr/
    drivers/mfd/<devname>/
        <devname>.c              # MFD parent driver (bus access, reset, config)
        <devname>.h              # Shared header (register map, structs, helpers)
        CMakeLists.txt           # MFD build rules
        Kconfig                  # MFD Kconfig

    drivers/adc/
        adc_<devname>.c          # ADC child driver
        Kconfig.<devname>        # ADC Kconfig fragment
        CMakeLists.txt           # (append line)

    drivers/dac/
        dac_<devname>.c          # DAC child driver
        Kconfig.<devname>        # DAC Kconfig fragment
        CMakeLists.txt           # (append line)

    drivers/gpio/
        gpio_<devname>.c         # GPIO child driver (optional)
        Kconfig.<devname>        # GPIO Kconfig fragment
        CMakeLists.txt           # (append line)

    dts/bindings/
        mfd/adi,<devname>.yaml            # MFD parent binding (SPI variant)
        mfd/adi,<devname_i2c>.yaml        # MFD parent binding (I2C variant)
        adc/adi,<devname>-adc.yaml        # ADC child binding
        dac/adi,<devname>-dac.yaml        # DAC child binding
        gpio/adi,<devname>-gpio.yaml      # GPIO child binding

    tests/drivers/mfd/<devname>/
        testcase.yaml
        prj.conf
        boards/native_sim.overlay
        src/main.c
```

---

## 3. Devicetree Bindings

### 3.1 MFD Parent Binding -- SPI (`dts/bindings/mfd/adi,<devname>.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Analog Devices <DEVNAME> <resolution>-bit, <channels>-channel
  configurable ADC/DAC/GPIO with SPI interface.

  This is the parent MFD (multi-function device) node. Child nodes
  select which subsystem each function maps to (ADC, DAC, GPIO).

  Example devicetree node:

    &spi1 {
        status = "okay";
        cs-gpios = <&gpioa 4 GPIO_ACTIVE_LOW>;

        <devname>: <devname>@0 {
            compatible = "adi,<devname_spi>";
            reg = <0>;
            spi-max-frequency = <10000000>;
            reset-gpios = <&gpiob 0 GPIO_ACTIVE_LOW>;

            adi,channel-modes = /bits/ 8
                <0x01 0x01 0x02 0x02 0x04 0x05 0x00 0x00>;
            adi,int-ref;

            <devname>_adc: adc {
                compatible = "adi,<devname>-adc";
                #io-channel-cells = <1>;
            };

            <devname>_dac: dac {
                compatible = "adi,<devname>-dac";
                #io-channel-cells = <1>;
            };

            <devname>_gpio: gpio {
                compatible = "adi,<devname>-gpio";
                gpio-controller;
                #gpio-cells = <2>;
                ngpios = <8>;
            };
        };
    };

compatible: "adi,<devname_spi>"

include: [spi-device.yaml, base.yaml]

properties:
  reset-gpios:
    type: phandle-array
    description: |
      GPIO connected to the hardware RESET pin (active low).
      If present the driver pulses reset during initialization.

  adi,channel-modes:
    type: uint8-array
    required: true
    description: |
      Per-channel mode configuration. One byte per channel, up to 8
      channels. Each byte encodes the pin function:
        0 = unused
        1 = ADC input
        2 = DAC output
        3 = DAC and ADC (bidirectional)
        4 = GPI (general-purpose input)
        5 = GPO (general-purpose output)

  adi,int-ref:
    type: boolean
    description: |
      Enable the internal voltage reference. When omitted, the
      external reference is used instead.

  adi,ext-ref-mv:
    type: int
    default: 2500
    description: |
      External reference voltage in millivolts. Only used when
      adi,int-ref is not present. Defaults to 2500 mV.

  adi,adc-range:
    type: int
    default: 0
    enum: [0, 1]
    description: |
      ADC input range selection.
        0 = 0 to VREF (default)
        1 = 0 to 2*VREF

  adi,dac-range:
    type: int
    default: 0
    enum: [0, 1]
    description: |
      DAC output range selection.
        0 = 0 to VREF (default)
        1 = 0 to 2*VREF
```

### 3.2 MFD Parent Binding -- I2C (`dts/bindings/mfd/adi,<devname_i2c>.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Analog Devices <DEVNAME_I2C> <resolution>-bit, <channels>-channel
  configurable ADC/DAC/GPIO with I2C interface.

  This is the I2C variant of the <DEVNAME> MFD parent. See the SPI
  variant binding for full property descriptions and child node examples.

  Example devicetree node:

    &i2c0 {
        status = "okay";

        <devname_i2c>: <devname_i2c>@10 {
            compatible = "adi,<devname_i2c>";
            reg = <0x10>;

            adi,channel-modes = /bits/ 8
                <0x01 0x01 0x02 0x02 0x00 0x00 0x00 0x00>;
            adi,int-ref;

            <devname>_adc: adc {
                compatible = "adi,<devname>-adc";
                #io-channel-cells = <1>;
            };

            <devname>_dac: dac {
                compatible = "adi,<devname>-dac";
                #io-channel-cells = <1>;
            };
        };
    };

compatible: "adi,<devname_i2c>"

include: [i2c-device.yaml, base.yaml]

properties:
  reset-gpios:
    type: phandle-array
    description: |
      GPIO connected to the hardware RESET pin (active low).

  adi,channel-modes:
    type: uint8-array
    required: true
    description: |
      Per-channel mode configuration (same encoding as the SPI variant).

  adi,int-ref:
    type: boolean
    description: Enable the internal voltage reference.

  adi,ext-ref-mv:
    type: int
    default: 2500
    description: External reference voltage in millivolts.

  adi,adc-range:
    type: int
    default: 0
    enum: [0, 1]
    description: ADC input range (0 = Vref, 1 = 2*Vref).

  adi,dac-range:
    type: int
    default: 0
    enum: [0, 1]
    description: DAC output range (0 = Vref, 1 = 2*Vref).
```

### 3.3 ADC Child Binding (`dts/bindings/adc/adi,<devname>-adc.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  ADC function of the Analog Devices <DEVNAME> configurable
  ADC/DAC/GPIO. Must be a child node of an adi,<devname_spi>
  or adi,<devname_i2c> MFD parent.

compatible: "adi,<devname>-adc"

include: [base.yaml]

properties:
  "#io-channel-cells":
    const: 1
```

### 3.4 DAC Child Binding (`dts/bindings/dac/adi,<devname>-dac.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  DAC function of the Analog Devices <DEVNAME> configurable
  ADC/DAC/GPIO. Must be a child node of an adi,<devname_spi>
  or adi,<devname_i2c> MFD parent.

compatible: "adi,<devname>-dac"

include: [base.yaml]

properties:
  "#io-channel-cells":
    const: 1
```

### 3.5 GPIO Child Binding (`dts/bindings/gpio/adi,<devname>-gpio.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  GPIO function of the Analog Devices <DEVNAME> configurable
  ADC/DAC/GPIO. Must be a child node of an adi,<devname_spi>
  or adi,<devname_i2c> MFD parent.

compatible: "adi,<devname>-gpio"

include: [gpio-controller.yaml, base.yaml]

properties:
  "#gpio-cells":
    const: 2

  ngpios:
    type: int
    default: 8
    description: Number of GPIO-capable pins on the device.
```

---

## 4. Kconfig

### 4.1 MFD Parent (`drivers/mfd/<devname>/Kconfig`)

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config MFD_<DEVNAME>
	bool "<DEVNAME> MFD driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME_SPI>_ENABLED || \
		   DT_HAS_ADI_<DEVNAME_I2C>_ENABLED
	select SPI if $(dt_compat_on_bus,$(DT_COMPAT_ADI_<DEVNAME_SPI>),spi)
	select I2C if $(dt_compat_on_bus,$(DT_COMPAT_ADI_<DEVNAME_I2C>),i2c)
	help
	  Enable the MFD (multi-function device) parent driver for the
	  Analog Devices <DEVNAME_SPI> (SPI) / <DEVNAME_I2C> (I2C)
	  <resolution>-bit, <channels>-channel configurable ADC/DAC/GPIO.

	  This driver handles bus communication, device reset, channel
	  mode configuration, and reference management. Child drivers
	  (ADC, DAC, GPIO) register with their respective subsystems.
```

### 4.2 ADC Child (`drivers/adc/Kconfig.<devname>`)

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config ADC_<DEVNAME>
	bool "<DEVNAME> ADC child driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ADC_ENABLED
	depends on MFD_<DEVNAME>
	help
	  Enable the ADC child driver for the <DEVNAME> MFD device.
	  Exposes channels configured as ADC inputs through the
	  Zephyr ADC subsystem API.
```

Append to `drivers/adc/Kconfig`:

```kconfig
source "drivers/adc/Kconfig.<devname>"
```

### 4.3 DAC Child (`drivers/dac/Kconfig.<devname>`)

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config DAC_<DEVNAME>
	bool "<DEVNAME> DAC child driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_DAC_ENABLED
	depends on MFD_<DEVNAME>
	help
	  Enable the DAC child driver for the <DEVNAME> MFD device.
	  Exposes channels configured as DAC outputs through the
	  Zephyr DAC subsystem API.
```

Append to `drivers/dac/Kconfig`:

```kconfig
source "drivers/dac/Kconfig.<devname>"
```

### 4.4 GPIO Child (`drivers/gpio/Kconfig.<devname>`)

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config GPIO_<DEVNAME>
	bool "<DEVNAME> GPIO child driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_GPIO_ENABLED
	depends on MFD_<DEVNAME>
	help
	  Enable the GPIO child driver for the <DEVNAME> MFD device.
	  Exposes channels configured as GPI or GPO through the
	  Zephyr GPIO subsystem API.
```

Append to `drivers/gpio/Kconfig`:

```kconfig
source "drivers/gpio/Kconfig.<devname>"
```

---

## 5. CMakeLists.txt (Build System Integration)

### 5.1 MFD Parent (`drivers/mfd/<devname>/CMakeLists.txt`)

```cmake
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_MFD_<DEVNAME> <devname>.c)
```

Also add to the parent `drivers/mfd/CMakeLists.txt`:

```cmake
add_subdirectory_ifdef(CONFIG_MFD_<DEVNAME> <devname>)
```

### 5.2 ADC Child (append to `drivers/adc/CMakeLists.txt`)

```cmake
zephyr_library_sources_ifdef(CONFIG_ADC_<DEVNAME> adc_<devname>.c)
```

### 5.3 DAC Child (append to `drivers/dac/CMakeLists.txt`)

```cmake
zephyr_library_sources_ifdef(CONFIG_DAC_<DEVNAME> dac_<devname>.c)
```

### 5.4 GPIO Child (append to `drivers/gpio/CMakeLists.txt`)

```cmake
zephyr_library_sources_ifdef(CONFIG_GPIO_<DEVNAME> gpio_<devname>.c)
```

---

## 6. MFD Parent Driver

### 6.1 Header (`drivers/mfd/<devname>/<devname>.h`)

This header is shared across the MFD parent and all child drivers.
It defines the register map, channel mode constants, config/data
structs, and bus-access helpers.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MFD_<DEVNAME>_H_
#define ZEPHYR_DRIVERS_MFD_<DEVNAME>_H_

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/* ---------------- Channel Mode Constants ------------------------------ */

#define <DEVNAME>_CH_MODE_UNUSED	0
#define <DEVNAME>_CH_MODE_ADC		1
#define <DEVNAME>_CH_MODE_DAC		2
#define <DEVNAME>_CH_MODE_DAC_AND_ADC	3
#define <DEVNAME>_CH_MODE_GPI		4
#define <DEVNAME>_CH_MODE_GPO		5

#define <DEVNAME>_NUM_CHANNELS		8
#define <DEVNAME>_RESOLUTION		12

/* ---------------- Register Map ---------------------------------------- */

#define <DEVNAME>_REG_NOOP		0x0
#define <DEVNAME>_REG_DAC_READBACK	0x1
#define <DEVNAME>_REG_ADC_SEQ		0x2
#define <DEVNAME>_REG_CTRL		0x3
#define <DEVNAME>_REG_ADC_EN		0x4
#define <DEVNAME>_REG_DAC_EN		0x5
#define <DEVNAME>_REG_PULLDOWN		0x6
#define <DEVNAME>_REG_LDAC		0x7
#define <DEVNAME>_REG_GPIO_OUT_EN	0x8
#define <DEVNAME>_REG_GPIO_SET		0x9
#define <DEVNAME>_REG_GPIO_IN_EN	0xA
#define <DEVNAME>_REG_PD		0xB
#define <DEVNAME>_REG_OPEN_DRAIN	0xC
#define <DEVNAME>_REG_TRISTATE		0xD
#define <DEVNAME>_REG_RESET		0xF

/* Control register fields */
#define <DEVNAME>_CTRL_ADC_RANGE_MSK	BIT(5)
#define <DEVNAME>_CTRL_DAC_RANGE_MSK	BIT(4)
#define <DEVNAME>_CTRL_ADC_BUFF_EN_MSK	BIT(8)
#define <DEVNAME>_CTRL_WRITE_ALL_MSK	BIT(11)

/* Power-down register fields */
#define <DEVNAME>_PD_EN_REF_MSK		BIT(9)

/* ADC sequence register fields */
#define <DEVNAME>_ADC_SEQ_TEMP_MSK	BIT(8)
#define <DEVNAME>_ADC_SEQ_REP_MSK	BIT(9)

/* DAC write / ADC readback data field */
#define <DEVNAME>_DATA_MSK		GENMASK(11, 0)
#define <DEVNAME>_CHAN_ADDR_MSK		GENMASK(14, 12)

/* Reset key */
#define <DEVNAME>_RESET_KEY		0xDAC

/* ---------------- Range Enumeration ----------------------------------- */

enum <devname>_range {
	<DEVNAME>_RANGE_VREF = 0,      /* 0 to VREF */
	<DEVNAME>_RANGE_2VREF = 1,     /* 0 to 2*VREF */
};

/* ---------------- Bus Type Enumeration -------------------------------- */

enum <devname>_bus_type {
	<DEVNAME>_BUS_SPI,
	<DEVNAME>_BUS_I2C,
};

/* ---------------- Config & Data Structs ------------------------------- */

/**
 * @brief Per-instance compile-time config (from devicetree, const).
 *
 * Supports both SPI and I2C bus types. Only one bus spec is active
 * per instance, determined by the compatible string.
 */
struct <devname>_config {
	/** Bus type: SPI or I2C. */
	enum <devname>_bus_type bus_type;
	/** SPI bus spec (valid only when bus_type == SPI). */
	struct spi_dt_spec spi;
	/** I2C bus spec (valid only when bus_type == I2C). */
	struct i2c_dt_spec i2c;
	/** Optional hardware reset GPIO. */
	struct gpio_dt_spec reset_gpio;
	/** Per-channel mode configuration from devicetree. */
	uint8_t channel_modes[<DEVNAME>_NUM_CHANNELS];
	/** Number of channels configured in the devicetree. */
	uint8_t num_channels;
	/** Use internal reference if true. */
	bool int_ref;
	/** External reference voltage in millivolts. */
	uint32_t ext_ref_mv;
	/** ADC range selection. */
	enum <devname>_range adc_range;
	/** DAC range selection. */
	enum <devname>_range dac_range;
};

/**
 * @brief Per-instance mutable runtime data.
 *
 * Holds cached state and a mutex for serialising bus access
 * across child drivers (ADC, DAC, GPIO).
 */
struct <devname>_data {
	/** Mutex to serialise bus transactions across children. */
	struct k_mutex lock;
	/** Cached DAC output values per channel. */
	uint16_t cached_dac[<DEVNAME>_NUM_CHANNELS];
	/** Active reference voltage in millivolts. */
	uint32_t vref_mv;
};

/* ---------------- Bus Access Helpers ---------------------------------- */

/**
 * @brief Write a register value to the device.
 *
 * The caller must hold data->lock. Frame format is device-specific;
 * adapt to match the datasheet.
 *
 * @param dev  MFD parent device.
 * @param reg  Register address (4-bit for AD5592R-family).
 * @param val  16-bit value to write.
 * @return 0 on success, negative errno on error.
 */
int <devname>_reg_write(const struct device *dev, uint8_t reg,
			uint16_t val);

/**
 * @brief Read a register value from the device.
 *
 * @param dev  MFD parent device.
 * @param reg  Register address.
 * @param val  Pointer to store the read value.
 * @return 0 on success, negative errno on error.
 */
int <devname>_reg_read(const struct device *dev, uint8_t reg,
		       uint16_t *val);

/**
 * @brief Write a DAC channel with a given code.
 *
 * @param dev   MFD parent device.
 * @param chan  Channel number (0-7).
 * @param code  12-bit DAC code.
 * @return 0 on success, negative errno on error.
 */
int <devname>_write_dac(const struct device *dev, uint8_t chan,
			uint16_t code);

/**
 * @brief Read an ADC channel.
 *
 * Issues an ADC conversion on the specified channel and returns
 * the result.
 *
 * @param dev     MFD parent device.
 * @param chan    Channel number (0-7).
 * @param result  Pointer to store the 12-bit ADC result.
 * @return 0 on success, negative errno on error.
 */
int <devname>_read_adc(const struct device *dev, uint8_t chan,
		       uint16_t *result);

/**
 * @brief Read the GPIO input state.
 *
 * @param dev   MFD parent device.
 * @param val   Pointer to store the GPIO pin states (bitfield).
 * @return 0 on success, negative errno on error.
 */
int <devname>_gpio_read(const struct device *dev, uint8_t *val);

/**
 * @brief Set GPIO output pins.
 *
 * @param dev   MFD parent device.
 * @param mask  Bitmask of pins to modify.
 * @param val   Pin values to set.
 * @return 0 on success, negative errno on error.
 */
int <devname>_gpio_write(const struct device *dev, uint8_t mask,
			 uint8_t val);

/**
 * @brief Lock the device mutex.
 *
 * Child drivers must call this before any bus access.
 */
static inline void <devname>_lock(const struct device *dev)
{
	struct <devname>_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
}

/**
 * @brief Unlock the device mutex.
 */
static inline void <devname>_unlock(const struct device *dev)
{
	struct <devname>_data *data = dev->data;

	k_mutex_unlock(&data->lock);
}

#endif /* ZEPHYR_DRIVERS_MFD_<DEVNAME>_H_ */
```

### 6.2 Source (`drivers/mfd/<devname>/<devname>.c`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The MFD parent handles dual-bus (SPI + I2C) support by using two
 * DT_DRV_COMPAT blocks. Each compatible string gets its own set of
 * instance macros. The actual driver logic is shared.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "<devname>.h"

LOG_MODULE_REGISTER(<devname>, CONFIG_MFD_LOG_LEVEL);

/* ---------- SPI bus access ------------------------------------------- */

static int <devname>_spi_write(const struct device *dev, uint8_t reg,
			       uint16_t val)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[2];
	uint16_t frame;

	/*
	 * SPI frame format (16 bits):
	 *   [15]    = 0 (write)
	 *   [14:11] = register address
	 *   [10:0]  = data (11 bits for config regs)
	 *
	 * Adapt to match the actual device framing.
	 */
	frame = ((uint16_t)(reg & 0x0F) << 11) | (val & 0x07FF);
	sys_put_be16(frame, tx_buf);

	const struct spi_buf buf = {
		.buf = tx_buf,
		.len = sizeof(tx_buf),
	};
	const struct spi_buf_set tx_set = {
		.buffers = &buf,
		.count = 1,
	};

	return spi_write_dt(&cfg->spi, &tx_set);
}

static int <devname>_spi_read(const struct device *dev, uint8_t reg,
			      uint16_t *val)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[2];
	uint8_t rx_buf[2];
	uint16_t frame;
	int ret;

	/*
	 * SPI read sequence:
	 *   1. Send readback command with register address
	 *   2. Send NOP to clock out the result
	 *
	 * Step 1: issue readback request.
	 */
	frame = ((uint16_t)(<DEVNAME>_REG_DAC_READBACK & 0x0F) << 11) |
		((uint16_t)(reg & 0x0F) << 2);
	sys_put_be16(frame, tx_buf);

	const struct spi_buf tx = {
		.buf = tx_buf,
		.len = sizeof(tx_buf),
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx,
		.count = 1,
	};
	const struct spi_buf rx = {
		.buf = rx_buf,
		.len = sizeof(rx_buf),
	};
	const struct spi_buf_set rx_set = {
		.buffers = &rx,
		.count = 1,
	};

	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret < 0) {
		return ret;
	}

	/* Step 2: NOP read to get the actual data. */
	frame = ((uint16_t)<DEVNAME>_REG_NOOP << 11);
	sys_put_be16(frame, tx_buf);

	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret < 0) {
		return ret;
	}

	*val = sys_get_be16(rx_buf);

	return 0;
}

static int <devname>_spi_write_dac(const struct device *dev, uint8_t chan,
				   uint16_t code)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[2];
	uint16_t frame;

	/*
	 * DAC write frame (16 bits):
	 *   [15]    = 1 (DAC write indicator)
	 *   [14:12] = channel address
	 *   [11:0]  = DAC data
	 */
	frame = BIT(15) | ((uint16_t)(chan & 0x07) << 12) |
		(code & <DEVNAME>_DATA_MSK);
	sys_put_be16(frame, tx_buf);

	const struct spi_buf buf = {
		.buf = tx_buf,
		.len = sizeof(tx_buf),
	};
	const struct spi_buf_set tx_set = {
		.buffers = &buf,
		.count = 1,
	};

	return spi_write_dt(&cfg->spi, &tx_set);
}

static int <devname>_spi_read_adc(const struct device *dev, uint8_t chan,
				  uint16_t *result)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[2];
	uint8_t rx_buf[2];
	uint16_t frame;
	int ret;

	/*
	 * ADC read sequence:
	 *   1. Set the ADC sequence register to select the channel.
	 *   2. Send NOP to trigger conversion and read result.
	 *   3. Send another NOP to read the converted data.
	 */

	/* Step 1: select channel for conversion. */
	frame = ((uint16_t)<DEVNAME>_REG_ADC_SEQ << 11) | BIT(chan);
	sys_put_be16(frame, tx_buf);

	const struct spi_buf tx = {
		.buf = tx_buf,
		.len = sizeof(tx_buf),
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx,
		.count = 1,
	};
	const struct spi_buf rx = {
		.buf = rx_buf,
		.len = sizeof(rx_buf),
	};
	const struct spi_buf_set rx_set = {
		.buffers = &rx,
		.count = 1,
	};

	ret = spi_write_dt(&cfg->spi, &tx_set);
	if (ret < 0) {
		return ret;
	}

	/* Step 2: NOP to trigger conversion. */
	frame = ((uint16_t)<DEVNAME>_REG_NOOP << 11);
	sys_put_be16(frame, tx_buf);

	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret < 0) {
		return ret;
	}

	/* Step 3: NOP to read the result. */
	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret < 0) {
		return ret;
	}

	*result = sys_get_be16(rx_buf) & <DEVNAME>_DATA_MSK;

	return 0;
}

/* ---------- I2C bus access ------------------------------------------- */

static int <devname>_i2c_write(const struct device *dev, uint8_t reg,
			       uint16_t val)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[2];
	uint16_t frame;

	/*
	 * I2C write format:
	 *   Byte 0: (reg << 4) | MSB nibble of val
	 *   Byte 1: LSB byte of val
	 *
	 * Adapt the framing to match the I2C variant datasheet.
	 */
	frame = ((uint16_t)(reg & 0x0F) << 11) | (val & 0x07FF);
	sys_put_be16(frame, tx_buf);

	return i2c_write_dt(&cfg->i2c, tx_buf, sizeof(tx_buf));
}

static int <devname>_i2c_read(const struct device *dev, uint8_t reg,
			      uint16_t *val)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[2];
	uint8_t rx_buf[2];
	uint16_t frame;
	int ret;

	/* Write readback command, then read 2 bytes. */
	frame = ((uint16_t)(<DEVNAME>_REG_DAC_READBACK & 0x0F) << 11) |
		((uint16_t)(reg & 0x0F) << 2);
	sys_put_be16(frame, tx_buf);

	ret = i2c_write_dt(&cfg->i2c, tx_buf, sizeof(tx_buf));
	if (ret < 0) {
		return ret;
	}

	ret = i2c_read_dt(&cfg->i2c, rx_buf, sizeof(rx_buf));
	if (ret < 0) {
		return ret;
	}

	*val = sys_get_be16(rx_buf);

	return 0;
}

static int <devname>_i2c_write_dac(const struct device *dev, uint8_t chan,
				   uint16_t code)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[3];

	/*
	 * I2C DAC write: 3 bytes.
	 *   Byte 0: (1 << 7) | (chan << 4) | MSB nibble
	 *   Byte 1-2: data
	 *
	 * Adapt framing to the actual I2C variant.
	 */
	uint16_t frame = BIT(15) | ((uint16_t)(chan & 0x07) << 12) |
			 (code & <DEVNAME>_DATA_MSK);

	tx_buf[0] = (uint8_t)(frame >> 8);
	tx_buf[1] = (uint8_t)(frame & 0xFF);

	return i2c_write_dt(&cfg->i2c, tx_buf, 2);
}

static int <devname>_i2c_read_adc(const struct device *dev, uint8_t chan,
				  uint16_t *result)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[2];
	uint8_t rx_buf[2];
	uint16_t frame;
	int ret;

	/* Select ADC channel. */
	frame = ((uint16_t)<DEVNAME>_REG_ADC_SEQ << 11) | BIT(chan);
	sys_put_be16(frame, tx_buf);

	ret = i2c_write_dt(&cfg->i2c, tx_buf, sizeof(tx_buf));
	if (ret < 0) {
		return ret;
	}

	/* Read the conversion result. */
	ret = i2c_read_dt(&cfg->i2c, rx_buf, sizeof(rx_buf));
	if (ret < 0) {
		return ret;
	}

	*result = sys_get_be16(rx_buf) & <DEVNAME>_DATA_MSK;

	return 0;
}

/* ---------- Dispatch to active bus ----------------------------------- */

int <devname>_reg_write(const struct device *dev, uint8_t reg,
			uint16_t val)
{
	const struct <devname>_config *cfg = dev->config;

	if (cfg->bus_type == <DEVNAME>_BUS_SPI) {
		return <devname>_spi_write(dev, reg, val);
	}

	return <devname>_i2c_write(dev, reg, val);
}

int <devname>_reg_read(const struct device *dev, uint8_t reg,
		       uint16_t *val)
{
	const struct <devname>_config *cfg = dev->config;

	if (cfg->bus_type == <DEVNAME>_BUS_SPI) {
		return <devname>_spi_read(dev, reg, val);
	}

	return <devname>_i2c_read(dev, reg, val);
}

int <devname>_write_dac(const struct device *dev, uint8_t chan,
			uint16_t code)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	if (cfg->bus_type == <DEVNAME>_BUS_SPI) {
		ret = <devname>_spi_write_dac(dev, chan, code);
	} else {
		ret = <devname>_i2c_write_dac(dev, chan, code);
	}

	if (ret == 0) {
		data->cached_dac[chan] = code;
	}

	return ret;
}

int <devname>_read_adc(const struct device *dev, uint8_t chan,
		       uint16_t *result)
{
	const struct <devname>_config *cfg = dev->config;

	if (cfg->bus_type == <DEVNAME>_BUS_SPI) {
		return <devname>_spi_read_adc(dev, chan, result);
	}

	return <devname>_i2c_read_adc(dev, chan, result);
}

int <devname>_gpio_read(const struct device *dev, uint8_t *val)
{
	uint16_t reg_val;
	int ret;

	ret = <devname>_reg_read(dev, <DEVNAME>_REG_GPIO_IN_EN, &reg_val);
	if (ret < 0) {
		return ret;
	}

	/*
	 * GPIO readback depends on the device: some return GPIO state
	 * in the read-back of GPIO_IN_EN, others need a dedicated
	 * GPIO read command. Adapt to the datasheet.
	 */
	*val = (uint8_t)(reg_val & 0xFF);

	return 0;
}

int <devname>_gpio_write(const struct device *dev, uint8_t mask,
			 uint8_t val)
{
	uint16_t reg_val;
	int ret;

	ret = <devname>_reg_read(dev, <DEVNAME>_REG_GPIO_SET, &reg_val);
	if (ret < 0) {
		return ret;
	}

	reg_val = (reg_val & ~(uint16_t)mask) | (uint16_t)(val & mask);

	return <devname>_reg_write(dev, <DEVNAME>_REG_GPIO_SET, reg_val);
}

/* ---------- Channel mode programming -------------------------------- */

/**
 * @brief Program channel modes into the device registers.
 *
 * Scans the channel_modes array and writes the ADC enable, DAC enable,
 * GPIO output enable, and GPIO input enable registers accordingly.
 */
static int <devname>_set_channel_modes(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	uint16_t adc_en = 0, dac_en = 0;
	uint16_t gpio_out_en = 0, gpio_in_en = 0;
	int ret;

	for (uint8_t i = 0; i < cfg->num_channels; i++) {
		switch (cfg->channel_modes[i]) {
		case <DEVNAME>_CH_MODE_ADC:
			adc_en |= BIT(i);
			break;
		case <DEVNAME>_CH_MODE_DAC:
			dac_en |= BIT(i);
			break;
		case <DEVNAME>_CH_MODE_DAC_AND_ADC:
			adc_en |= BIT(i);
			dac_en |= BIT(i);
			break;
		case <DEVNAME>_CH_MODE_GPI:
			gpio_in_en |= BIT(i);
			break;
		case <DEVNAME>_CH_MODE_GPO:
			gpio_out_en |= BIT(i);
			break;
		case <DEVNAME>_CH_MODE_UNUSED:
		default:
			break;
		}
	}

	ret = <devname>_reg_write(dev, <DEVNAME>_REG_ADC_EN, adc_en);
	if (ret < 0) {
		return ret;
	}

	ret = <devname>_reg_write(dev, <DEVNAME>_REG_DAC_EN, dac_en);
	if (ret < 0) {
		return ret;
	}

	ret = <devname>_reg_write(dev, <DEVNAME>_REG_GPIO_OUT_EN,
				   gpio_out_en);
	if (ret < 0) {
		return ret;
	}

	ret = <devname>_reg_write(dev, <DEVNAME>_REG_GPIO_IN_EN,
				   gpio_in_en);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/* ---------- Initialization ------------------------------------------- */

static int <devname>_init(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint16_t ctrl_val = 0;
	int ret;

	k_mutex_init(&data->lock);

	/* Verify bus readiness */
	if (cfg->bus_type == <DEVNAME>_BUS_SPI) {
		if (!spi_is_ready_dt(&cfg->spi)) {
			LOG_ERR("SPI bus not ready");
			return -ENODEV;
		}
	} else {
		if (!i2c_is_ready_dt(&cfg->i2c)) {
			LOG_ERR("I2C bus not ready");
			return -ENODEV;
		}
	}

	/* Optional hardware reset */
	if (cfg->reset_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->reset_gpio,
					    GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure reset GPIO: %d", ret);
			return ret;
		}

		/* Pulse reset: assert, wait, de-assert */
		k_usleep(100);
		gpio_pin_set_dt(&cfg->reset_gpio, 0);
		k_usleep(250);
	}

	/* Software reset */
	ret = <devname>_reg_write(dev, <DEVNAME>_REG_RESET,
				   <DEVNAME>_RESET_KEY);
	if (ret < 0) {
		LOG_ERR("Software reset failed: %d", ret);
		return ret;
	}

	k_usleep(250);

	/* Configure ADC / DAC ranges */
	if (cfg->adc_range == <DEVNAME>_RANGE_2VREF) {
		ctrl_val |= <DEVNAME>_CTRL_ADC_RANGE_MSK;
	}

	if (cfg->dac_range == <DEVNAME>_RANGE_2VREF) {
		ctrl_val |= <DEVNAME>_CTRL_DAC_RANGE_MSK;
	}

	ret = <devname>_reg_write(dev, <DEVNAME>_REG_CTRL, ctrl_val);
	if (ret < 0) {
		LOG_ERR("Failed to set CTRL register: %d", ret);
		return ret;
	}

	/* Configure reference */
	if (cfg->int_ref) {
		ret = <devname>_reg_write(dev, <DEVNAME>_REG_PD,
					  <DEVNAME>_PD_EN_REF_MSK);
		if (ret < 0) {
			LOG_ERR("Failed to enable internal ref: %d", ret);
			return ret;
		}
		/* Internal VREF is typically 2500 mV */
		data->vref_mv = 2500;
	} else {
		data->vref_mv = cfg->ext_ref_mv;
	}

	/* Program channel modes */
	ret = <devname>_set_channel_modes(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set channel modes: %d", ret);
		return ret;
	}

	LOG_INF("<DEVNAME> initialized (%s bus, vref=%u mV)",
		cfg->bus_type == <DEVNAME>_BUS_SPI ? "SPI" : "I2C",
		data->vref_mv);

	return 0;
}

/* ---------- SPI instance macros -------------------------------------- */

/*
 * Two separate DT_DRV_COMPAT blocks handle SPI and I2C variants.
 * Undefine and redefine DT_DRV_COMPAT between blocks.
 */

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_<devname_spi>

#define <DEVNAME>_SPI_CHANNEL_MODES(n)					\
	{ DT_INST_PROP_BY_IDX(n, adi_channel_modes, 0),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 1),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 2),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 3),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 4),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 5),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 6),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 7) }

#define <DEVNAME>_SPI_INIT(n)						\
	static struct <devname>_data <devname>_spi_data_##n;		\
									\
	static const struct <devname>_config				\
		<devname>_spi_config_##n = {				\
		.bus_type = <DEVNAME>_BUS_SPI,				\
		.spi = SPI_DT_SPEC_INST_GET(				\
			n, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),	\
		.i2c = { 0 },						\
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(			\
			n, reset_gpios, {0}),				\
		.channel_modes = <DEVNAME>_SPI_CHANNEL_MODES(n),	\
		.num_channels = DT_INST_PROP_LEN(n,			\
			adi_channel_modes),				\
		.int_ref = DT_INST_PROP(n, adi_int_ref),		\
		.ext_ref_mv = DT_INST_PROP(n, adi_ext_ref_mv),		\
		.adc_range = DT_INST_PROP(n, adi_adc_range),		\
		.dac_range = DT_INST_PROP(n, adi_dac_range),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, <devname>_init, NULL,			\
			      &<devname>_spi_data_##n,			\
			      &<devname>_spi_config_##n,		\
			      POST_KERNEL,				\
			      CONFIG_MFD_INIT_PRIORITY,			\
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_SPI_INIT)

/* ---------- I2C instance macros -------------------------------------- */

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_<devname_i2c>

#define <DEVNAME>_I2C_CHANNEL_MODES(n)					\
	{ DT_INST_PROP_BY_IDX(n, adi_channel_modes, 0),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 1),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 2),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 3),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 4),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 5),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 6),		\
	  DT_INST_PROP_BY_IDX(n, adi_channel_modes, 7) }

#define <DEVNAME>_I2C_INIT(n)						\
	static struct <devname>_data <devname>_i2c_data_##n;		\
									\
	static const struct <devname>_config				\
		<devname>_i2c_config_##n = {				\
		.bus_type = <DEVNAME>_BUS_I2C,				\
		.spi = { 0 },						\
		.i2c = I2C_DT_SPEC_INST_GET(n),			\
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(			\
			n, reset_gpios, {0}),				\
		.channel_modes = <DEVNAME>_I2C_CHANNEL_MODES(n),	\
		.num_channels = DT_INST_PROP_LEN(n,			\
			adi_channel_modes),				\
		.int_ref = DT_INST_PROP(n, adi_int_ref),		\
		.ext_ref_mv = DT_INST_PROP(n, adi_ext_ref_mv),		\
		.adc_range = DT_INST_PROP(n, adi_adc_range),		\
		.dac_range = DT_INST_PROP(n, adi_dac_range),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, <devname>_init, NULL,			\
			      &<devname>_i2c_data_##n,			\
			      &<devname>_i2c_config_##n,		\
			      POST_KERNEL,				\
			      CONFIG_MFD_INIT_PRIORITY,			\
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_I2C_INIT)
```

---

## 7. ADC Child Driver (`drivers/adc/adc_<devname>.c`)

The ADC child driver registers with the Zephyr ADC subsystem. It
obtains the parent MFD device via `DEVICE_DT_GET(DT_INST_PARENT(n))`
and delegates all bus operations to the parent's helper functions.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>_adc

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

/* Include the MFD parent header for register defs and helpers */
#include "<devname>.h"

LOG_MODULE_REGISTER(<devname>_adc, CONFIG_ADC_LOG_LEVEL);

/* ---------- Config & Data -------------------------------------------- */

struct <devname>_adc_config {
	/** Parent MFD device pointer. */
	const struct device *mfd_dev;
};

struct <devname>_adc_data {
	/** Bitmask of channels configured via adc_channel_setup(). */
	uint16_t configured_channels;
};

/* ---------- ADC API: channel_setup ----------------------------------- */

static int <devname>_adc_channel_setup(const struct device *dev,
				       const struct adc_channel_cfg *cfg)
{
	const struct <devname>_adc_config *adc_cfg = dev->config;
	const struct <devname>_config *mfd_cfg = adc_cfg->mfd_dev->config;
	struct <devname>_adc_data *data = dev->data;

	if (cfg->channel_id >= <DEVNAME>_NUM_CHANNELS) {
		LOG_ERR("Invalid channel ID: %u", cfg->channel_id);
		return -EINVAL;
	}

	/* Verify this channel is configured as an ADC in the MFD parent */
	uint8_t mode = mfd_cfg->channel_modes[cfg->channel_id];

	if (mode != <DEVNAME>_CH_MODE_ADC &&
	    mode != <DEVNAME>_CH_MODE_DAC_AND_ADC) {
		LOG_ERR("Channel %u not configured as ADC (mode=%u)",
			cfg->channel_id, mode);
		return -EINVAL;
	}

	if (cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Unsupported gain: %d", cfg->gain);
		return -ENOTSUP;
	}

	if (cfg->reference != ADC_REF_INTERNAL &&
	    cfg->reference != ADC_REF_EXTERNAL0) {
		LOG_ERR("Unsupported reference: %d", cfg->reference);
		return -ENOTSUP;
	}

	if (cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Unsupported acquisition time");
		return -ENOTSUP;
	}

	data->configured_channels |= BIT(cfg->channel_id);

	LOG_DBG("Channel %u configured for ADC", cfg->channel_id);

	return 0;
}

/* ---------- ADC API: read -------------------------------------------- */

static int <devname>_adc_read(const struct device *dev,
			      const struct adc_sequence *sequence)
{
	const struct <devname>_adc_config *adc_cfg = dev->config;
	struct <devname>_adc_data *data = dev->data;
	const struct device *mfd = adc_cfg->mfd_dev;
	uint32_t channels = sequence->channels;
	uint16_t *buf = (uint16_t *)sequence->buffer;
	uint8_t channel;
	int ret;

	if (sequence->resolution != <DEVNAME>_RESOLUTION) {
		LOG_ERR("Unsupported resolution: %u (expected %u)",
			sequence->resolution, <DEVNAME>_RESOLUTION);
		return -EINVAL;
	}

	if (!channels) {
		LOG_ERR("No channels selected");
		return -EINVAL;
	}

	/* Verify all requested channels are configured */
	if ((channels & data->configured_channels) != channels) {
		LOG_ERR("Unconfigured channel(s) requested");
		return -EINVAL;
	}

	/* Verify buffer is large enough */
	size_t needed = sizeof(uint16_t) * POPCOUNT(channels);

	if (sequence->buffer_size < needed) {
		LOG_ERR("Buffer too small: need %zu, have %u",
			needed, sequence->buffer_size);
		return -ENOMEM;
	}

	/* Lock the parent device and perform conversions */
	<devname>_lock(mfd);

	while (channels) {
		channel = find_lsb_set(channels) - 1;

		ret = <devname>_read_adc(mfd, channel, buf);
		if (ret < 0) {
			<devname>_unlock(mfd);
			LOG_ERR("ADC read ch%u failed: %d", channel, ret);
			return ret;
		}

		buf++;
		channels &= ~BIT(channel);
	}

	<devname>_unlock(mfd);

	return 0;
}

/* ---------- ADC API table -------------------------------------------- */

static DEVICE_API(adc, <devname>_adc_api) = {
	.channel_setup = <devname>_adc_channel_setup,
	.read = <devname>_adc_read,
	.ref_internal = 2500,   /* Adjust to match device internal ref */
};

/* ---------- Init ----------------------------------------------------- */

static int <devname>_adc_init(const struct device *dev)
{
	const struct <devname>_adc_config *cfg = dev->config;

	if (!device_is_ready(cfg->mfd_dev)) {
		LOG_ERR("Parent MFD device not ready");
		return -ENODEV;
	}

	LOG_INF("<DEVNAME> ADC child initialized");

	return 0;
}

/* ---------- Instance macros ------------------------------------------ */

#define <DEVNAME>_ADC_INIT(n)						\
	static struct <devname>_adc_data <devname>_adc_data_##n;	\
									\
	static const struct <devname>_adc_config			\
		<devname>_adc_config_##n = {				\
		.mfd_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, <devname>_adc_init, NULL,		\
			      &<devname>_adc_data_##n,			\
			      &<devname>_adc_config_##n,		\
			      POST_KERNEL,				\
			      CONFIG_ADC_INIT_PRIORITY,			\
			      &<devname>_adc_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_ADC_INIT)
```

---

## 8. DAC Child Driver (`drivers/dac/dac_<devname>.c`)

The DAC child driver follows the same parent-reference pattern as
the ADC child.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>_dac

#include <zephyr/device.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "<devname>.h"

LOG_MODULE_REGISTER(<devname>_dac, CONFIG_DAC_LOG_LEVEL);

/* ---------- Config & Data -------------------------------------------- */

struct <devname>_dac_config {
	const struct device *mfd_dev;
};

struct <devname>_dac_data {
	uint8_t configured_channels;
};

/* ---------- DAC API: channel_setup ----------------------------------- */

static int <devname>_dac_channel_setup(const struct device *dev,
				       const struct dac_channel_cfg *cfg)
{
	const struct <devname>_dac_config *dac_cfg = dev->config;
	const struct <devname>_config *mfd_cfg = dac_cfg->mfd_dev->config;
	struct <devname>_dac_data *data = dev->data;

	if (cfg->channel_id >= <DEVNAME>_NUM_CHANNELS) {
		LOG_ERR("Invalid channel ID: %u", cfg->channel_id);
		return -EINVAL;
	}

	/* Verify this channel is configured as a DAC in the MFD parent */
	uint8_t mode = mfd_cfg->channel_modes[cfg->channel_id];

	if (mode != <DEVNAME>_CH_MODE_DAC &&
	    mode != <DEVNAME>_CH_MODE_DAC_AND_ADC) {
		LOG_ERR("Channel %u not configured as DAC (mode=%u)",
			cfg->channel_id, mode);
		return -EINVAL;
	}

	if (cfg->resolution != <DEVNAME>_RESOLUTION) {
		LOG_ERR("Unsupported resolution %u (expected %u)",
			cfg->resolution, <DEVNAME>_RESOLUTION);
		return -ENOTSUP;
	}

	data->configured_channels |= BIT(cfg->channel_id);

	LOG_DBG("Channel %u configured for DAC", cfg->channel_id);

	return 0;
}

/* ---------- DAC API: write_value ------------------------------------- */

static int <devname>_dac_write_value(const struct device *dev,
				     uint8_t channel, uint32_t value)
{
	const struct <devname>_dac_config *dac_cfg = dev->config;
	struct <devname>_dac_data *data = dev->data;
	const struct device *mfd = dac_cfg->mfd_dev;
	int ret;

	if (channel >= <DEVNAME>_NUM_CHANNELS) {
		LOG_ERR("Invalid channel %u", channel);
		return -EINVAL;
	}

	if (!(data->configured_channels & BIT(channel))) {
		LOG_ERR("Channel %u not configured", channel);
		return -EINVAL;
	}

	if (value >= (1U << <DEVNAME>_RESOLUTION)) {
		LOG_ERR("Value %u exceeds %u-bit range",
			value, <DEVNAME>_RESOLUTION);
		return -EINVAL;
	}

	<devname>_lock(mfd);
	ret = <devname>_write_dac(mfd, channel, (uint16_t)value);
	<devname>_unlock(mfd);

	if (ret < 0) {
		LOG_ERR("DAC write ch%u failed: %d", channel, ret);
	}

	return ret;
}

/* ---------- DAC API table -------------------------------------------- */

static DEVICE_API(dac, <devname>_dac_api) = {
	.channel_setup = <devname>_dac_channel_setup,
	.write_value = <devname>_dac_write_value,
};

/* ---------- Init ----------------------------------------------------- */

static int <devname>_dac_init(const struct device *dev)
{
	const struct <devname>_dac_config *cfg = dev->config;

	if (!device_is_ready(cfg->mfd_dev)) {
		LOG_ERR("Parent MFD device not ready");
		return -ENODEV;
	}

	LOG_INF("<DEVNAME> DAC child initialized");

	return 0;
}

/* ---------- Instance macros ------------------------------------------ */

#define <DEVNAME>_DAC_INIT(n)						\
	static struct <devname>_dac_data <devname>_dac_data_##n;	\
									\
	static const struct <devname>_dac_config			\
		<devname>_dac_config_##n = {				\
		.mfd_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, <devname>_dac_init, NULL,		\
			      &<devname>_dac_data_##n,			\
			      &<devname>_dac_config_##n,		\
			      POST_KERNEL,				\
			      CONFIG_DAC_INIT_PRIORITY,			\
			      &<devname>_dac_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_DAC_INIT)
```

---

## 9. GPIO Child Driver (`drivers/gpio/gpio_<devname>.c`)

The GPIO child driver implements the Zephyr GPIO subsystem API for
channels configured as GPI or GPO.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>_gpio

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "<devname>.h"

LOG_MODULE_REGISTER(<devname>_gpio, CONFIG_GPIO_LOG_LEVEL);

/* ---------- Config & Data -------------------------------------------- */

struct <devname>_gpio_config {
	struct gpio_driver_config common;
	const struct device *mfd_dev;
};

struct <devname>_gpio_data {
	struct gpio_driver_data common;
	/** Bitmask of pins configured as output. */
	uint8_t output_pins;
	/** Cached output pin values. */
	uint8_t output_state;
};

/* ---------- GPIO API: pin_configure ---------------------------------- */

static int <devname>_gpio_pin_configure(const struct device *dev,
					gpio_pin_t pin,
					gpio_flags_t flags)
{
	const struct <devname>_gpio_config *gpio_cfg = dev->config;
	const struct <devname>_config *mfd_cfg =
		gpio_cfg->mfd_dev->config;
	struct <devname>_gpio_data *data = dev->data;
	uint8_t mode;

	if (pin >= <DEVNAME>_NUM_CHANNELS) {
		return -EINVAL;
	}

	mode = mfd_cfg->channel_modes[pin];

	if (flags & GPIO_OUTPUT) {
		if (mode != <DEVNAME>_CH_MODE_GPO) {
			LOG_ERR("Pin %u not configured as GPO", pin);
			return -ENOTSUP;
		}
		data->output_pins |= BIT(pin);

		/* Set initial output value */
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			data->output_state |= BIT(pin);
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			data->output_state &= ~BIT(pin);
		}

		<devname>_lock(gpio_cfg->mfd_dev);
		int ret = <devname>_gpio_write(gpio_cfg->mfd_dev,
					       BIT(pin),
					       data->output_state);
		<devname>_unlock(gpio_cfg->mfd_dev);

		return ret;
	}

	if (flags & GPIO_INPUT) {
		if (mode != <DEVNAME>_CH_MODE_GPI) {
			LOG_ERR("Pin %u not configured as GPI", pin);
			return -ENOTSUP;
		}
		data->output_pins &= ~BIT(pin);
		return 0;
	}

	/* Disconnect */
	data->output_pins &= ~BIT(pin);

	return 0;
}

/* ---------- GPIO API: port_get_raw ----------------------------------- */

static int <devname>_gpio_port_get_raw(const struct device *dev,
				       gpio_port_value_t *value)
{
	const struct <devname>_gpio_config *gpio_cfg = dev->config;
	uint8_t val;
	int ret;

	<devname>_lock(gpio_cfg->mfd_dev);
	ret = <devname>_gpio_read(gpio_cfg->mfd_dev, &val);
	<devname>_unlock(gpio_cfg->mfd_dev);

	if (ret < 0) {
		return ret;
	}

	*value = (gpio_port_value_t)val;

	return 0;
}

/* ---------- GPIO API: port_set_masked_raw ---------------------------- */

static int <devname>_gpio_port_set_masked_raw(const struct device *dev,
					      gpio_port_pins_t mask,
					      gpio_port_value_t value)
{
	const struct <devname>_gpio_config *gpio_cfg = dev->config;
	struct <devname>_gpio_data *data = dev->data;
	int ret;

	/* Only affect output pins */
	mask &= data->output_pins;

	data->output_state = (data->output_state & ~(uint8_t)mask) |
			     ((uint8_t)value & (uint8_t)mask);

	<devname>_lock(gpio_cfg->mfd_dev);
	ret = <devname>_gpio_write(gpio_cfg->mfd_dev,
				   (uint8_t)mask, data->output_state);
	<devname>_unlock(gpio_cfg->mfd_dev);

	return ret;
}

/* ---------- GPIO API: port_set_bits_raw ------------------------------ */

static int <devname>_gpio_port_set_bits_raw(const struct device *dev,
					    gpio_port_pins_t pins)
{
	return <devname>_gpio_port_set_masked_raw(dev, pins, pins);
}

/* ---------- GPIO API: port_clear_bits_raw ---------------------------- */

static int <devname>_gpio_port_clear_bits_raw(const struct device *dev,
					      gpio_port_pins_t pins)
{
	return <devname>_gpio_port_set_masked_raw(dev, pins, 0);
}

/* ---------- GPIO API: port_toggle_bits ------------------------------- */

static int <devname>_gpio_port_toggle_bits(const struct device *dev,
					   gpio_port_pins_t pins)
{
	struct <devname>_gpio_data *data = dev->data;

	return <devname>_gpio_port_set_masked_raw(
		dev, pins, ~data->output_state);
}

/* ---------- GPIO API table ------------------------------------------- */

static DEVICE_API(gpio, <devname>_gpio_api) = {
	.pin_configure = <devname>_gpio_pin_configure,
	.port_get_raw = <devname>_gpio_port_get_raw,
	.port_set_masked_raw = <devname>_gpio_port_set_masked_raw,
	.port_set_bits_raw = <devname>_gpio_port_set_bits_raw,
	.port_clear_bits_raw = <devname>_gpio_port_clear_bits_raw,
	.port_toggle_bits = <devname>_gpio_port_toggle_bits,
};

/* ---------- Init ----------------------------------------------------- */

static int <devname>_gpio_init(const struct device *dev)
{
	const struct <devname>_gpio_config *cfg = dev->config;

	if (!device_is_ready(cfg->mfd_dev)) {
		LOG_ERR("Parent MFD device not ready");
		return -ENODEV;
	}

	LOG_INF("<DEVNAME> GPIO child initialized");

	return 0;
}

/* ---------- Instance macros ------------------------------------------ */

#define <DEVNAME>_GPIO_INIT(n)						\
	static struct <devname>_gpio_data <devname>_gpio_data_##n;	\
									\
	static const struct <devname>_gpio_config			\
		<devname>_gpio_config_##n = {				\
		.common = {						\
			.port_pin_mask =				\
				GPIO_PORT_PIN_MASK_FROM_DT_INST(n),	\
		},							\
		.mfd_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, <devname>_gpio_init, NULL,		\
			      &<devname>_gpio_data_##n,			\
			      &<devname>_gpio_config_##n,		\
			      POST_KERNEL,				\
			      CONFIG_GPIO_INIT_PRIORITY,		\
			      &<devname>_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_GPIO_INIT)
```

---

## 10. Test Skeleton

### 10.1 `tests/drivers/mfd/<devname>/testcase.yaml`

```yaml
tests:
  drivers.mfd.<devname>:
    tags:
      - drivers
      - mfd
      - adc
      - dac
      - gpio
    depends_on: spi
    platform_allow: native_sim
    integration_platforms:
      - native_sim
```

### 10.2 `tests/drivers/mfd/<devname>/prj.conf`

```ini
CONFIG_ZTEST=y
CONFIG_MFD=y
CONFIG_MFD_<DEVNAME>=y
CONFIG_ADC=y
CONFIG_ADC_<DEVNAME>=y
CONFIG_DAC=y
CONFIG_DAC_<DEVNAME>=y
CONFIG_GPIO=y
CONFIG_GPIO_<DEVNAME>=y
CONFIG_SPI=y
CONFIG_LOG=y
```

### 10.3 `tests/drivers/mfd/<devname>/boards/native_sim.overlay`

```dts
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/ {
	test_spi: spi@0 {
		compatible = "vnd,spi";
		reg = <0x0 0x1000>;
		#address-cells = <1>;
		#size-cells = <0>;
		status = "okay";
		clock-frequency = <1000000>;

		test_<devname>: <devname>@0 {
			compatible = "adi,<devname_spi>";
			reg = <0>;
			spi-max-frequency = <10000000>;
			status = "okay";

			adi,channel-modes = /bits/ 8
				<0x01 0x01 0x02 0x02 0x04 0x05 0x00 0x00>;
			adi,int-ref;

			test_<devname>_adc: adc {
				compatible = "adi,<devname>-adc";
				#io-channel-cells = <1>;
				status = "okay";
			};

			test_<devname>_dac: dac {
				compatible = "adi,<devname>-dac";
				#io-channel-cells = <1>;
				status = "okay";
			};

			test_<devname>_gpio: gpio {
				compatible = "adi,<devname>-gpio";
				gpio-controller;
				#gpio-cells = <2>;
				ngpios = <8>;
				status = "okay";
			};
		};
	};
};
```

### 10.4 `tests/drivers/mfd/<devname>/src/main.c`

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/gpio.h>

/* -- Device handles --------------------------------------------------- */

#define MFD_NODE  DT_NODELABEL(test_<devname>)
#define ADC_NODE  DT_NODELABEL(test_<devname>_adc)
#define DAC_NODE  DT_NODELABEL(test_<devname>_dac)
#define GPIO_NODE DT_NODELABEL(test_<devname>_gpio)

static const struct device *mfd_dev = DEVICE_DT_GET(MFD_NODE);
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static const struct device *dac_dev = DEVICE_DT_GET(DAC_NODE);
static const struct device *gpio_dev = DEVICE_DT_GET(GPIO_NODE);

/* -- MFD parent tests ------------------------------------------------- */

ZTEST(<devname>_suite, test_mfd_device_ready)
{
	zassert_true(device_is_ready(mfd_dev),
		     "MFD parent device not ready");
}

/* -- ADC child tests -------------------------------------------------- */

ZTEST(<devname>_suite, test_adc_device_ready)
{
	zassert_true(device_is_ready(adc_dev),
		     "ADC child device not ready");
}

ZTEST(<devname>_suite, test_adc_channel_setup)
{
	struct adc_channel_cfg cfg = {
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = 0,  /* ch0 = ADC in the overlay */
	};

	int ret = adc_channel_setup(adc_dev, &cfg);

	zassert_ok(ret, "ADC channel_setup failed: %d", ret);
}

ZTEST(<devname>_suite, test_adc_channel_setup_invalid)
{
	struct adc_channel_cfg cfg = {
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = 2,  /* ch2 = DAC, not ADC */
	};

	int ret = adc_channel_setup(adc_dev, &cfg);

	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for DAC channel used as ADC, got %d",
		      ret);
}

ZTEST(<devname>_suite, test_adc_read)
{
	struct adc_channel_cfg cfg = {
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = 0,
	};

	int ret = adc_channel_setup(adc_dev, &cfg);

	zassert_ok(ret, "ADC channel_setup failed: %d", ret);

	uint16_t buffer;
	struct adc_sequence seq = {
		.channels = BIT(0),
		.buffer = &buffer,
		.buffer_size = sizeof(buffer),
		.resolution = 12,
	};

	ret = adc_read(adc_dev, &seq);
	zassert_ok(ret, "ADC read failed: %d", ret);
}

/* -- DAC child tests -------------------------------------------------- */

ZTEST(<devname>_suite, test_dac_device_ready)
{
	zassert_true(device_is_ready(dac_dev),
		     "DAC child device not ready");
}

ZTEST(<devname>_suite, test_dac_channel_setup)
{
	struct dac_channel_cfg cfg = {
		.channel_id = 2,  /* ch2 = DAC in the overlay */
		.resolution = 12,
	};

	int ret = dac_channel_setup(dac_dev, &cfg);

	zassert_ok(ret, "DAC channel_setup failed: %d", ret);
}

ZTEST(<devname>_suite, test_dac_channel_setup_invalid)
{
	struct dac_channel_cfg cfg = {
		.channel_id = 0,  /* ch0 = ADC, not DAC */
		.resolution = 12,
	};

	int ret = dac_channel_setup(dac_dev, &cfg);

	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for ADC channel used as DAC, got %d",
		      ret);
}

ZTEST(<devname>_suite, test_dac_write_value)
{
	struct dac_channel_cfg cfg = {
		.channel_id = 2,
		.resolution = 12,
	};

	int ret = dac_channel_setup(dac_dev, &cfg);

	zassert_ok(ret, "DAC channel_setup failed: %d", ret);

	/* Write mid-scale */
	ret = dac_write_value(dac_dev, 2, 2048);
	zassert_ok(ret, "DAC write_value failed: %d", ret);
}

ZTEST(<devname>_suite, test_dac_write_out_of_range)
{
	struct dac_channel_cfg cfg = {
		.channel_id = 3,
		.resolution = 12,
	};

	int ret = dac_channel_setup(dac_dev, &cfg);

	zassert_ok(ret, "DAC channel_setup failed: %d", ret);

	/* Value exceeds 12-bit range */
	ret = dac_write_value(dac_dev, 3, 5000);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for out-of-range value, got %d", ret);
}

/* -- GPIO child tests ------------------------------------------------- */

ZTEST(<devname>_suite, test_gpio_device_ready)
{
	zassert_true(device_is_ready(gpio_dev),
		     "GPIO child device not ready");
}

ZTEST(<devname>_suite, test_gpio_output_configure)
{
	/* ch5 = GPO in the overlay */
	int ret = gpio_pin_configure(gpio_dev, 5, GPIO_OUTPUT_LOW);

	zassert_ok(ret, "GPIO output configure failed: %d", ret);
}

ZTEST(<devname>_suite, test_gpio_input_configure)
{
	/* ch4 = GPI in the overlay */
	int ret = gpio_pin_configure(gpio_dev, 4, GPIO_INPUT);

	zassert_ok(ret, "GPIO input configure failed: %d", ret);
}

ZTEST(<devname>_suite, test_gpio_set_and_clear)
{
	int ret;

	ret = gpio_pin_configure(gpio_dev, 5, GPIO_OUTPUT_LOW);
	zassert_ok(ret, "GPIO configure failed: %d", ret);

	ret = gpio_pin_set(gpio_dev, 5, 1);
	zassert_ok(ret, "GPIO set failed: %d", ret);

	ret = gpio_pin_set(gpio_dev, 5, 0);
	zassert_ok(ret, "GPIO clear failed: %d", ret);
}

ZTEST(<devname>_suite, test_gpio_wrong_mode)
{
	/* ch0 = ADC, should not be configurable as GPIO output */
	int ret = gpio_pin_configure(gpio_dev, 0, GPIO_OUTPUT_LOW);

	zassert_equal(ret, -ENOTSUP,
		      "Expected -ENOTSUP for ADC pin as GPIO, got %d", ret);
}

ZTEST_SUITE(<devname>_suite, NULL, NULL, NULL, NULL, NULL);
```

---

## 11. Key Conventions

### Architecture

1. **MFD parent + child pattern** -- the parent MFD driver owns the
   bus (SPI or I2C), performs device reset, configures channel modes,
   and exposes register-level helpers. Child drivers obtain a pointer
   to the parent via `DEVICE_DT_GET(DT_INST_PARENT(n))` and call the
   parent's helper functions for all bus access. The parent exposes
   **no subsystem API** (its API pointer is `NULL`).

2. **Dual-bus support** -- two `DT_DRV_COMPAT` blocks in the MFD
   source handle SPI and I2C variants. Undefine and redefine
   `DT_DRV_COMPAT` between blocks. Each compatible has its own
   binding YAML and Kconfig `DT_HAS_*_ENABLED` dependency.

3. **Mutex serialisation** -- a `k_mutex` in the parent data struct
   serialises bus transactions across all children. Each child must
   call `<devname>_lock()` / `<devname>_unlock()` around any sequence
   of parent helper calls that must be atomic.

4. **Init priority ordering** -- the MFD parent uses
   `CONFIG_MFD_INIT_PRIORITY` (typically 80). ADC, DAC, and GPIO
   children use their respective subsystem priorities (also 80 by
   default), which is fine because Zephyr resolves parent-before-child
   ordering automatically for devicetree child nodes. If needed, bump
   children to a higher numeric priority value.

### Devicetree

5. **Channel mode array** -- per-channel function is specified as a
   `uint8-array` property (`adi,channel-modes`) on the MFD parent
   node. This is more flexible than fixed child-node channel lists
   and matches the no-OS `channel_modes[]` array pattern.

6. **Child node compatibles** -- each child function has a dedicated
   compatible string (e.g., `adi,ad5592r-adc`) and binding YAML.
   The child node is a direct child of the MFD parent node in the
   devicetree, enabling `DT_INST_PARENT()` to resolve the parent.

7. **No dynamic allocation** -- all config and data structs are
   statically allocated via `DT_INST_FOREACH_STATUS_OKAY()`. No
   `k_malloc()` or `k_calloc()` calls in any driver.

### Coding Style

8. **Linux kernel style** -- tabs for indentation, 100-column hard
   limit. K&R braces. Opening brace on same line for control
   structures, new line for function definitions.

9. **SPDX headers** -- every file uses the short-form
   `SPDX-License-Identifier: Apache-2.0`. No lengthy license blocks.

10. **Logging** -- use `LOG_MODULE_REGISTER()` with the appropriate
    subsystem log level (`CONFIG_MFD_LOG_LEVEL`, `CONFIG_ADC_LOG_LEVEL`,
    etc.). Use `LOG_ERR`, `LOG_WRN`, `LOG_INF`, `LOG_DBG`. Never use
    `printk()`.

11. **Error codes** -- return negative `errno` values throughout.
    Never return positive error codes.

12. **Bit manipulation** -- use Zephyr's `BIT()`, `GENMASK()`,
    `FIELD_PREP()`, `FIELD_GET()` from `<zephyr/sys/util.h>`.

13. **Byte order** -- use `sys_get_be16()` / `sys_put_be16()` from
    `<zephyr/sys/byteorder.h>` for bus frame construction.

14. **`DEVICE_API()` macro** -- each child uses the typed
    `DEVICE_API(adc, ...)`, `DEVICE_API(dac, ...)`, or
    `DEVICE_API(gpio, ...)` macro for compile-time API type checking.

### no-OS to Zephyr Mapping

| no-OS Concept | Zephyr Equivalent |
|---|---|
| `<devname>-base.h` / `.c` | `drivers/mfd/<devname>/<devname>.h` / `.c` |
| `<devname_spi>.c` / `<devname_i2c>.c` | Dual `DT_DRV_COMPAT` blocks in MFD source |
| `<devname>_rw_ops` function pointers | Bus dispatch via `<devname>_config.bus_type` |
| `<devname>_init_param` | Devicetree properties + `<devname>_config` |
| `<devname>_dev` | `<devname>_data` + `const struct device *` |
| `<devname>_init()` / `_remove()` | `<devname>_init()` (no remove in Zephyr) |
| `iio_<devname>` channels | ADC / DAC / GPIO subsystem child drivers |
| `channel_modes[]` | `adi,channel-modes` DT property |
| `no_os_spi_init()` | `SPI_DT_SPEC_INST_GET()` + `spi_is_ready_dt()` |
| `no_os_i2c_init()` | `I2C_DT_SPEC_INST_GET()` + `i2c_is_ready_dt()` |
| `no_os_calloc()` / `no_os_free()` | Static allocation via DT macros |
| `NO_OS_BIT()` | `BIT()` |
| `NO_OS_GENMASK()` | `GENMASK()` |
| `no_os_field_prep()` / `no_os_field_get()` | `FIELD_PREP()` / `FIELD_GET()` |
| `pr_info()` | `LOG_INF()` |
| BSD-3-Clause license | Apache-2.0 license |

### Commit Message Format

```
# Commit 1: Devicetree bindings
dts: bindings: add bindings for Analog Devices <DEVNAME> MFD

Add devicetree bindings for the <DEVNAME_SPI> (SPI) and <DEVNAME_I2C>
(I2C) configurable ADC/DAC/GPIO MFD device, plus child bindings for
the ADC, DAC, and GPIO sub-functions.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 2: MFD parent driver
drivers: mfd: add Analog Devices <DEVNAME> parent driver

Add the MFD parent driver for the <DEVNAME_SPI>/<DEVNAME_I2C>
<resolution>-bit, <channels>-channel configurable ADC/DAC/GPIO.
The driver handles dual-bus (SPI + I2C) communication, device
reset, channel mode configuration, and reference management.
Child drivers (ADC, DAC, GPIO) use the parent's helper functions
for all bus access.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 3: ADC child driver
drivers: adc: add <DEVNAME> ADC child driver

Add an ADC child driver for the <DEVNAME> MFD device. The driver
exposes channels configured as ADC inputs through the Zephyr ADC
subsystem API, delegating bus access to the MFD parent.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 4: DAC child driver
drivers: dac: add <DEVNAME> DAC child driver

Add a DAC child driver for the <DEVNAME> MFD device. The driver
exposes channels configured as DAC outputs through the Zephyr DAC
subsystem API.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 5: GPIO child driver
drivers: gpio: add <DEVNAME> GPIO child driver

Add a GPIO child driver for the <DEVNAME> MFD device. The driver
exposes channels configured as GPI or GPO through the Zephyr GPIO
subsystem API.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 6: Tests
tests: drivers: mfd: add tests for <DEVNAME>

Add a ztest-based test suite for the <DEVNAME> MFD device covering
MFD parent initialization, ADC channel setup and read, DAC channel
setup and write, GPIO pin configuration and I/O, and error paths
for cross-function channel misuse.

Signed-off-by: Your Name <your.name@analog.com>
```
