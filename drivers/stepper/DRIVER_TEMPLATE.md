# Zephyr Motor Controller Driver Template

Reference driver: `drivers/stepper/adi/` (Zephyr stepper subsystem pattern),
study the TMC stepper drivers in `drivers/stepper/` for motion controller
specifics.

This template covers every file needed to add a new ADI stepper motor
controller driver to the Zephyr RTOS tree. Replace `<devname>` with the
part number in lowercase (e.g., `tmc5240`), `<DEVNAME>` with uppercase
(e.g., `TMC5240`), and `<devnum>` with the numeric portion (e.g., `5240`)
throughout.

---

## 1. Purpose & Zephyr Subsystem Mapping

This driver maps to the Zephyr **stepper subsystem** defined in
`include/zephyr/drivers/stepper.h`. The subsystem provides a unified API
for stepper motor controllers with the following key entry points:

| Zephyr Stepper API Function       | What it does                                  |
|-----------------------------------|-----------------------------------------------|
| `stepper_enable()`                | Enable or disable the motor driver            |
| `stepper_move_by()`               | Move by a relative number of microsteps       |
| `stepper_move_to()`               | Move to an absolute microstep position        |
| `stepper_set_max_velocity()`      | Set the maximum velocity in microsteps/second  |
| `stepper_set_micro_step_res()`    | Set the microstepping resolution              |
| `stepper_get_micro_step_res()`    | Get the current microstepping resolution      |
| `stepper_set_reference_position()`| Preset the current position counter           |
| `stepper_get_actual_position()`   | Read the current actual position              |
| `stepper_is_moving()`             | Check if the motor is currently in motion     |

The driver implements the `stepper_driver_api` struct (via the
`DEVICE_API(stepper, ...)` macro) which contains pointers to the driver's
implementations of the above functions.

Key differences from the no-OS driver model:

| Aspect | no-OS | Zephyr |
|---|---|---|
| Init pattern | `<devname>_init()` allocates descriptor | `<devname>_init()` called by device model at boot |
| Motion control | `<devname>_set_target_pos()` | `stepper_move_to()` + async event callback |
| Velocity mode | `<devname>_set_target_vel()` | `stepper_set_max_velocity()` + `stepper_move_by()` |
| Position readout | `<devname>_get_current_pos()` | `stepper_get_actual_position()` |
| Microstepping | `init_param.microsteps_res` field | `stepper_set_micro_step_res()` API + devicetree |
| Bus access | `no_os_spi_write_and_read()` | Zephyr SPI API (`spi_transceive_dt()`) |
| Config | Init param struct | Devicetree + Kconfig |
| Motion profile | Stored in device descriptor | Stored in mutable data struct, applied at init |
| Completion events | Polling `get_current_pos` | Async callback via `stepper_event_callback_t` |

All configuration comes from **devicetree** at compile time. There is no
dynamic allocation -- the config struct is `const` and populated from DT
macros, while the data struct is static and holds mutable runtime state.

---

## 2. File Checklist

```
zephyr/
    drivers/stepper/adi/<devname>/
        <devname>.h              # Driver private header
        <devname>.c              # Driver implementation
        <devname>_spi.c          # SPI transport layer
        <devname>_spi.h          # SPI transport header
        Kconfig                  # Kconfig symbol definitions
        CMakeLists.txt           # Build system integration

    dts/bindings/stepper/
        adi,<devname>.yaml       # Devicetree binding

    tests/drivers/stepper/<devname>/
        testcase.yaml            # Test metadata
        prj.conf                 # Test project config
        boards/native_sim.overlay # Board-specific DT overlay
        src/main.c               # Test source
```

---

## 3. Devicetree Binding (`dts/bindings/stepper/adi,<devname>.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Analog Devices <DEVNAME> stepper motor controller/driver with SPI interface.
  Integrates a motion controller with configurable multi-phase
  acceleration/deceleration ramps, position and velocity operating modes,
  and up to 256x microstepping.
  Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/<DEVNAME>.pdf

compatible: "adi,<devname>"

include: [stepper-controller.yaml, spi-device.yaml]

properties:
  clock-frequency:
    type: int
    default: 16000000
    description: |
      Internal clock frequency in Hz. Used for velocity and acceleration
      unit conversions. Typically 12 MHz or 16 MHz depending on the
      crystal/oscillator connected to the CLK pin.

  en-gpios:
    type: phandle-array
    description: |
      Enable GPIO pin. Active-low by default on most evaluation boards.
      Controls the DRV_ENN hardware enable input.

  diag0-gpios:
    type: phandle-array
    description: |
      DIAG0 interrupt GPIO. Active-high by default.
      Used for position-reached, stall detection, and error events.

  diag1-gpios:
    type: phandle-array
    description: |
      DIAG1 interrupt GPIO (optional). Can be configured as a secondary
      diagnostic output.

  micro-step-res:
    type: int
    default: 256
    description: |
      Microstepping resolution in microsteps per full step.
      Valid values: 1, 2, 4, 8, 16, 32, 64, 128, 256

  current-run:
    type: int
    default: 16
    description: |
      Motor run current setting (0-31). Determines the peak current
      during active motion. The actual current depends on the global
      scaler and current range settings.

  current-hold:
    type: int
    default: 8
    description: |
      Motor hold (standstill) current setting (0-31). Lower values
      reduce power consumption and heat when stationary.

  ihold-delay:
    type: int
    default: 6
    description: |
      Delay before switching from run current to hold current.
      Range 0-15, units of 2^18 clock cycles.

  irun-delay:
    type: int
    default: 0
    description: |
      Delay before switching from hold current to run current.
      Range 0-15.

  global-scaler:
    type: int
    default: 128
    description: |
      Global current scaler (0-255). Scales the motor current
      for all operating modes. 0 = maximum current, 128 = 50%.

  toff:
    type: int
    default: 3
    description: |
      Chopper off-time setting (0-15). Controls the slow decay
      duration. 0 disables the driver.

  tbl:
    type: int
    default: 2
    description: |
      Chopper blank time setting (0-3). Controls the comparator
      blanking time after a switching event.

  slope-control:
    type: int
    default: 0
    description: |
      Slope control setting for voltage ramping (0-3). Controls
      the slew rate of the driver outputs.

  current-range:
    type: int
    default: 0
    description: |
      Current range selection (0-3). Selects the full-scale current
      range of the driver stage.

  vstart:
    type: int
    default: 0
    description: |
      Start velocity for motion ramp. Motor begins moving at this
      speed. In velocity units (microsteps per time unit).

  a1:
    type: int
    default: 0
    description: |
      First acceleration phase value. Used in S-curve ramp profiles.
      0 = skip first acceleration phase (use trapezoidal profile).

  v1:
    type: int
    default: 0
    description: |
      First velocity threshold. Transition point between first
      and second acceleration phases in S-curve profiles.

  a2:
    type: int
    default: 0
    description: |
      Second acceleration phase value. 0 = skip second phase.

  v2:
    type: int
    default: 0
    description: |
      Second velocity threshold. Transition point between second
      acceleration phase and maximum acceleration phase.

  amax:
    type: int
    default: 1000
    description: |
      Maximum acceleration value. Determines the steepest part of
      the velocity ramp.

  vmax:
    type: int
    default: 50000
    description: |
      Maximum velocity value. The motor will not exceed this speed
      during ramp-controlled motion.

  dmax:
    type: int
    default: 1000
    description: |
      Maximum deceleration value. Controls the deceleration rate
      when approaching the target position.

  d1:
    type: int
    default: 10
    description: |
      First deceleration phase value. Must be non-zero.

  d2:
    type: int
    default: 10
    description: |
      Second deceleration phase value. Must be non-zero.

  vstop:
    type: int
    default: 10
    description: |
      Stop velocity. Motor is considered stopped below this speed.
      Must be >= 1 and >= VSTART for correct operation.
```

A sample devicetree node using this binding:

```dts
&spi1 {
    status = "okay";
    cs-gpios = <&gpio0 16 GPIO_ACTIVE_LOW>;

    <devname>: <devname>@0 {
        compatible = "adi,<devname>";
        reg = <0>;
        spi-max-frequency = <4000000>;
        en-gpios = <&gpio0 12 GPIO_ACTIVE_LOW>;
        diag0-gpios = <&gpio0 14 GPIO_ACTIVE_HIGH>;
        clock-frequency = <16000000>;
        micro-step-res = <256>;
        current-run = <16>;
        current-hold = <8>;
        ihold-delay = <6>;
        global-scaler = <128>;
        toff = <3>;
        tbl = <2>;
        amax = <1000>;
        vmax = <50000>;
        dmax = <1000>;
        d1 = <10>;
        vstop = <10>;
    };
};
```

---

## 4. Kconfig (`drivers/stepper/adi/<devname>/Kconfig`)

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config <DEVNAME>
	bool "<DEVNAME> Stepper Motor Controller"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	select SPI
	help
	  Enable driver for the Analog Devices <DEVNAME> stepper motor
	  controller/driver. Supports configurable motion profiles with
	  multi-phase acceleration/deceleration ramps, position and velocity
	  operating modes, and up to 256x microstepping.

if <DEVNAME>

config <DEVNAME>_RAMPSTAT_POLL_INTERVAL_MS
	int "Ramp status polling interval (ms)"
	default 10
	help
	  Interval in milliseconds between polling the ramp status
	  register to detect position-reached events when GPIO-based
	  DIAG0 interrupt is not available. Set to 0 to disable polling.

endif # <DEVNAME>
```

---

## 5. CMakeLists.txt (`drivers/stepper/adi/<devname>/CMakeLists.txt`)

```cmake
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(<devname>.c)
zephyr_library_sources(<devname>_spi.c)
```

---

## 6. Driver Header (`drivers/stepper/adi/<devname>/<devname>.h`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_STEPPER_ADI_<DEVNAME>_H_
#define ZEPHYR_DRIVERS_STEPPER_ADI_<DEVNAME>_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

/* ---------------- Register Map ----------------------------------------- */

#define <DEVNAME>_REG_GCONF		0x00
#define <DEVNAME>_REG_GSTAT		0x01
#define <DEVNAME>_REG_IOIN		0x04
#define <DEVNAME>_REG_DRV_CONF		0x0A
#define <DEVNAME>_REG_GLOBAL_SCALER	0x0B
#define <DEVNAME>_REG_IHOLD_IRUN	0x10
#define <DEVNAME>_REG_RAMPMODE		0x20
#define <DEVNAME>_REG_XACTUAL		0x21
#define <DEVNAME>_REG_VACTUAL		0x22
#define <DEVNAME>_REG_VSTART		0x23
#define <DEVNAME>_REG_A1		0x24
#define <DEVNAME>_REG_V1		0x25
#define <DEVNAME>_REG_AMAX		0x26
#define <DEVNAME>_REG_VMAX		0x27
#define <DEVNAME>_REG_DMAX		0x28
#define <DEVNAME>_REG_D1		0x2A
#define <DEVNAME>_REG_VSTOP		0x2B
#define <DEVNAME>_REG_XTARGET		0x2D
#define <DEVNAME>_REG_A2		0x2E
#define <DEVNAME>_REG_V2		0x2F
#define <DEVNAME>_REG_D2		0x30
#define <DEVNAME>_REG_AACTUAL		0x31
#define <DEVNAME>_REG_RAMP_STAT	0x35
#define <DEVNAME>_REG_CHOPCONF		0x6C
#define <DEVNAME>_REG_DRV_STATUS	0x6F

/* SPI protocol constants */
#define <DEVNAME>_SPI_FRAME_LEN		5
#define <DEVNAME>_SPI_WRITE_BIT		BIT(7)

/* ---------------- Field Masks ------------------------------------------ */

/** GCONF register fields */
#define <DEVNAME>_SHAFT_MSK		BIT(4)

/** IHOLD_IRUN register fields */
#define <DEVNAME>_IHOLD_MSK		GENMASK(4, 0)
#define <DEVNAME>_IRUN_MSK		GENMASK(12, 8)
#define <DEVNAME>_IHOLDDELAY_MSK	GENMASK(19, 16)
#define <DEVNAME>_IRUNDELAY_MSK		GENMASK(27, 24)

/** DRV_CONF register fields */
#define <DEVNAME>_SLOPE_CONTROL_MSK	GENMASK(5, 4)
#define <DEVNAME>_CURRENT_RANGE_MSK	GENMASK(1, 0)

/** CHOPCONF register fields */
#define <DEVNAME>_TOFF_MSK		GENMASK(3, 0)
#define <DEVNAME>_TBL_MSK		GENMASK(16, 15)
#define <DEVNAME>_MRES_MSK		GENMASK(27, 24)

/** Ramp status register fields */
#define <DEVNAME>_RAMP_STAT_POS_REACHED	BIT(9)
#define <DEVNAME>_RAMP_STAT_VEL_REACHED	BIT(8)

/** Motion parameter masks */
#define <DEVNAME>_VMAX_MSK		GENMASK(22, 0)
#define <DEVNAME>_AMAX_MSK		GENMASK(17, 0)
#define <DEVNAME>_DMAX_MSK		GENMASK(17, 0)
#define <DEVNAME>_VSTART_MSK		GENMASK(17, 0)
#define <DEVNAME>_VSTOP_MSK		GENMASK(17, 0)
#define <DEVNAME>_A1_MSK		GENMASK(17, 0)
#define <DEVNAME>_V1_MSK		GENMASK(19, 0)
#define <DEVNAME>_A2_MSK		GENMASK(17, 0)
#define <DEVNAME>_V2_MSK		GENMASK(19, 0)
#define <DEVNAME>_D1_MSK		GENMASK(17, 0)
#define <DEVNAME>_D2_MSK		GENMASK(17, 0)
#define <DEVNAME>_RAMPMODE_MSK		GENMASK(1, 0)

/* Ramp mode values */
#define <DEVNAME>_RAMP_POSITION		0
#define <DEVNAME>_RAMP_VELPOS		1
#define <DEVNAME>_RAMP_VELNEG		2
#define <DEVNAME>_RAMP_HOLD		3

/* Default motion profile values (non-zero minimums) */
#define <DEVNAME>_DEFAULT_D1		10
#define <DEVNAME>_DEFAULT_D2		10
#define <DEVNAME>_DEFAULT_VSTOP		10

/* Microstepping resolution encoding: register value -> microstep count */
#define <DEVNAME>_MRES_256		0
#define <DEVNAME>_MRES_128		1
#define <DEVNAME>_MRES_64		2
#define <DEVNAME>_MRES_32		3
#define <DEVNAME>_MRES_16		4
#define <DEVNAME>_MRES_8		5
#define <DEVNAME>_MRES_4		6
#define <DEVNAME>_MRES_2		7
#define <DEVNAME>_MRES_FULLSTEP		8

/* Product / version ID expected value */
#define <DEVNAME>_VERSION_ID		0xXX

/* ---------------- Driver Structures ------------------------------------ */

/**
 * @brief Devicetree-derived configuration (const, per-instance).
 *
 * Populated at compile time from devicetree properties and stored in ROM.
 * Contains SPI bus specification, GPIO pins, and all motor configuration
 * parameters from the devicetree.
 */
struct <devname>_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec en_gpio;
	struct gpio_dt_spec diag0_gpio;
	uint32_t clock_frequency;
	uint32_t vstart;
	uint32_t a1;
	uint32_t v1;
	uint32_t a2;
	uint32_t v2;
	uint32_t amax;
	uint32_t vmax;
	uint32_t dmax;
	uint32_t d1;
	uint32_t d2;
	uint32_t vstop;
	uint16_t micro_step_res;
	uint8_t current_run;
	uint8_t current_hold;
	uint8_t ihold_delay;
	uint8_t irun_delay;
	uint8_t global_scaler;
	uint8_t toff;
	uint8_t tbl;
	uint8_t slope_control;
	uint8_t current_range;
};

/**
 * @brief Mutable runtime data (per-instance).
 *
 * Holds the current motion state, event callback, and synchronisation
 * primitives for position-reached event handling.
 */
struct <devname>_data {
	const struct device *dev;

	/** Current microstepping resolution (register encoding). */
	uint8_t mres;
	/** Cached maximum velocity for motion commands. */
	uint32_t max_velocity;

	/** Stepper event callback registered by the application. */
	stepper_event_callback_t callback;
	/** User data passed to the event callback. */
	void *event_cb_user_data;

	/** Work item for polling ramp status. */
	struct k_work_delayable rampstat_work;

	/** Mutex protecting concurrent register access. */
	struct k_mutex lock;
};

/* SPI transport function prototypes (defined in <devname>_spi.c) */
int <devname>_spi_read(const struct device *dev, uint8_t reg_addr,
		       uint32_t *reg_data);
int <devname>_spi_write(const struct device *dev, uint8_t reg_addr,
			uint32_t reg_data);
int <devname>_spi_update(const struct device *dev, uint8_t reg_addr,
			 uint32_t mask, uint32_t val);

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_<DEVNAME>_H_ */
```

---

## 7. SPI Transport (`drivers/stepper/adi/<devname>/<devname>_spi.c`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(<devname>, CONFIG_STEPPER_LOG_LEVEL);

#include "<devname>.h"

/**
 * @brief Read a 32-bit register over SPI.
 *
 * Motor controller SPI protocol uses a 5-byte frame:
 *   TX: [addr & 0x7F] [0x00] [0x00] [0x00] [0x00]
 *   RX: [status]      [data MSB ... data LSB]
 *
 * A read requires two SPI transfers: the first sends the read request,
 * the second clocks out the actual data.
 *
 * @param dev       Zephyr device pointer.
 * @param reg_addr  Register address (7-bit).
 * @param reg_data  Pointer to store the 32-bit read value.
 * @return 0 on success, negative errno on failure.
 */
int <devname>_spi_read(const struct device *dev, uint8_t reg_addr,
		       uint32_t *reg_data)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[<DEVNAME>_SPI_FRAME_LEN] = { 0 };
	uint8_t rx_buf[<DEVNAME>_SPI_FRAME_LEN] = { 0 };
	int ret;

	const struct spi_buf tx = {
		.buf = tx_buf, .len = sizeof(tx_buf),
	};
	const struct spi_buf rx = {
		.buf = rx_buf, .len = sizeof(rx_buf),
	};
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	/* First transfer: send read request */
	tx_buf[0] = reg_addr & 0x7F;

	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret) {
		LOG_ERR("SPI read request failed: %d", ret);
		return ret;
	}

	/* Second transfer: clock out the data */
	memset(tx_buf, 0, sizeof(tx_buf));
	tx_buf[0] = reg_addr & 0x7F;
	memset(rx_buf, 0, sizeof(rx_buf));

	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret) {
		LOG_ERR("SPI read data failed: %d", ret);
		return ret;
	}

	*reg_data = ((uint32_t)rx_buf[1] << 24) |
		    ((uint32_t)rx_buf[2] << 16) |
		    ((uint32_t)rx_buf[3] << 8)  |
		    (uint32_t)rx_buf[4];

	return 0;
}

/**
 * @brief Write a 32-bit register over SPI.
 *
 * Motor controller SPI protocol uses a 5-byte frame:
 *   TX: [addr | 0x80 (write)] [data MSB ... data LSB]
 *
 * @param dev       Zephyr device pointer.
 * @param reg_addr  Register address (7-bit).
 * @param reg_data  32-bit value to write.
 * @return 0 on success, negative errno on failure.
 */
int <devname>_spi_write(const struct device *dev, uint8_t reg_addr,
			uint32_t reg_data)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t tx_buf[<DEVNAME>_SPI_FRAME_LEN];

	tx_buf[0] = reg_addr | <DEVNAME>_SPI_WRITE_BIT;
	tx_buf[1] = (uint8_t)(reg_data >> 24);
	tx_buf[2] = (uint8_t)(reg_data >> 16);
	tx_buf[3] = (uint8_t)(reg_data >> 8);
	tx_buf[4] = (uint8_t)(reg_data & 0xFF);

	const struct spi_buf tx = {
		.buf = tx_buf, .len = sizeof(tx_buf),
	};
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	return spi_write_dt(&cfg->spi, &tx_set);
}

/**
 * @brief Read-modify-write a register field.
 *
 * @param dev       Zephyr device pointer.
 * @param reg_addr  Register address.
 * @param mask      Bitmask of the field(s) to update.
 * @param val       New field value (pre-shifted to the correct position).
 * @return 0 on success, negative errno on failure.
 */
int <devname>_spi_update(const struct device *dev, uint8_t reg_addr,
			 uint32_t mask, uint32_t val)
{
	uint32_t reg_val;
	int ret;

	ret = <devname>_spi_read(dev, reg_addr, &reg_val);
	if (ret) {
		return ret;
	}

	reg_val = (reg_val & ~mask) | (val & mask);

	return <devname>_spi_write(dev, reg_addr, reg_val);
}
```

---

## 8. Driver Source (`drivers/stepper/adi/<devname>/<devname>.c`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(<devname>, CONFIG_STEPPER_LOG_LEVEL);

#include "<devname>.h"

/* ---------- Microstepping resolution helpers --------------------------- */

/**
 * @brief Convert a microstep count (1, 2, 4, ..., 256) to the MRES
 *        register encoding.
 *
 * @param res Microstepping resolution (power of 2).
 * @return MRES register value, or -EINVAL for unsupported values.
 */
static int <devname>_mres_from_ustep(enum stepper_micro_step_resolution res)
{
	switch (res) {
	case STEPPER_MICRO_STEP_1:
		return <DEVNAME>_MRES_FULLSTEP;
	case STEPPER_MICRO_STEP_2:
		return <DEVNAME>_MRES_2;
	case STEPPER_MICRO_STEP_4:
		return <DEVNAME>_MRES_4;
	case STEPPER_MICRO_STEP_8:
		return <DEVNAME>_MRES_8;
	case STEPPER_MICRO_STEP_16:
		return <DEVNAME>_MRES_16;
	case STEPPER_MICRO_STEP_32:
		return <DEVNAME>_MRES_32;
	case STEPPER_MICRO_STEP_64:
		return <DEVNAME>_MRES_64;
	case STEPPER_MICRO_STEP_128:
		return <DEVNAME>_MRES_128;
	case STEPPER_MICRO_STEP_256:
		return <DEVNAME>_MRES_256;
	default:
		return -EINVAL;
	}
}

/**
 * @brief Convert a devicetree micro-step-res integer to the
 *        stepper_micro_step_resolution enum.
 *
 * @param dt_val Microstep count from devicetree (1, 2, ..., 256).
 * @return Corresponding enum value, or STEPPER_MICRO_STEP_256 as default.
 */
static enum stepper_micro_step_resolution <devname>_dt_to_ustep(uint16_t dt_val)
{
	switch (dt_val) {
	case 1:   return STEPPER_MICRO_STEP_1;
	case 2:   return STEPPER_MICRO_STEP_2;
	case 4:   return STEPPER_MICRO_STEP_4;
	case 8:   return STEPPER_MICRO_STEP_8;
	case 16:  return STEPPER_MICRO_STEP_16;
	case 32:  return STEPPER_MICRO_STEP_32;
	case 64:  return STEPPER_MICRO_STEP_64;
	case 128: return STEPPER_MICRO_STEP_128;
	case 256: return STEPPER_MICRO_STEP_256;
	default:  return STEPPER_MICRO_STEP_256;
	}
}

/* ---------- Motion profile --------------------------------------------- */

/**
 * @brief Apply the full motion profile from the devicetree configuration
 *        to the device registers.
 *
 * Configures all multi-phase acceleration/deceleration parameters
 * (VSTART, A1, V1, A2, V2, AMAX, VMAX, DMAX, D1, D2, VSTOP).
 * Applies defaults for D1, D2, and VSTOP if they are zero.
 *
 * @param dev Zephyr device pointer.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_set_motion_profile(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	uint32_t temp;
	int ret;

	ret = <devname>_spi_write(dev, <DEVNAME>_REG_AMAX,
				  cfg->amax & <DEVNAME>_AMAX_MSK);
	if (ret) {
		return ret;
	}

	ret = <devname>_spi_write(dev, <DEVNAME>_REG_VMAX,
				  cfg->vmax & <DEVNAME>_VMAX_MSK);
	if (ret) {
		return ret;
	}

	ret = <devname>_spi_write(dev, <DEVNAME>_REG_DMAX,
				  cfg->dmax & <DEVNAME>_DMAX_MSK);
	if (ret) {
		return ret;
	}

	ret = <devname>_spi_write(dev, <DEVNAME>_REG_VSTART,
				  cfg->vstart & <DEVNAME>_VSTART_MSK);
	if (ret) {
		return ret;
	}

	ret = <devname>_spi_write(dev, <DEVNAME>_REG_V1,
				  cfg->v1 & <DEVNAME>_V1_MSK);
	if (ret) {
		return ret;
	}

	ret = <devname>_spi_write(dev, <DEVNAME>_REG_A1,
				  cfg->a1 & <DEVNAME>_A1_MSK);
	if (ret) {
		return ret;
	}

	ret = <devname>_spi_write(dev, <DEVNAME>_REG_V2,
				  cfg->v2 & <DEVNAME>_V2_MSK);
	if (ret) {
		return ret;
	}

	ret = <devname>_spi_write(dev, <DEVNAME>_REG_A2,
				  cfg->a2 & <DEVNAME>_A2_MSK);
	if (ret) {
		return ret;
	}

	/* Apply defaults for D1, D2, VSTOP if zero */
	temp = cfg->d1 ? cfg->d1 : <DEVNAME>_DEFAULT_D1;
	ret = <devname>_spi_write(dev, <DEVNAME>_REG_D1,
				  temp & <DEVNAME>_D1_MSK);
	if (ret) {
		return ret;
	}

	temp = cfg->d2 ? cfg->d2 : <DEVNAME>_DEFAULT_D2;
	ret = <devname>_spi_write(dev, <DEVNAME>_REG_D2,
				  temp & <DEVNAME>_D2_MSK);
	if (ret) {
		return ret;
	}

	temp = cfg->vstop ? cfg->vstop : <DEVNAME>_DEFAULT_VSTOP;
	if (temp < cfg->vstart) {
		temp = cfg->vstart;
	}

	return <devname>_spi_write(dev, <DEVNAME>_REG_VSTOP,
				   temp & <DEVNAME>_VSTOP_MSK);
}

/* ---------- Ramp status polling ---------------------------------------- */

/**
 * @brief Delayed work handler to poll the ramp status register.
 *
 * Checks if the motor has reached its target position or velocity.
 * If an event is detected and a callback is registered, the callback
 * is invoked. Otherwise the work is rescheduled for continued polling.
 */
static void <devname>_rampstat_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct <devname>_data *data =
		CONTAINER_OF(dwork, struct <devname>_data, rampstat_work);
	const struct device *dev = data->dev;
	uint32_t ramp_stat;
	int ret;

	ret = <devname>_spi_read(dev, <DEVNAME>_REG_RAMP_STAT, &ramp_stat);
	if (ret) {
		LOG_ERR("Failed to read RAMP_STAT: %d", ret);
		return;
	}

	if (ramp_stat & <DEVNAME>_RAMP_STAT_POS_REACHED) {
		if (data->callback) {
			data->callback(dev,
				       STEPPER_EVENT_STEPS_COMPLETED,
				       data->event_cb_user_data);
		}
		return; /* Stop polling -- target reached */
	}

	/* Reschedule polling */
	k_work_reschedule(dwork,
			  K_MSEC(CONFIG_<DEVNAME>_RAMPSTAT_POLL_INTERVAL_MS));
}

/* ---------- Stepper API: enable ---------------------------------------- */

/**
 * @brief Enable or disable the motor driver outputs.
 *
 * Controls the EN GPIO pin if available. When disabling, also stops
 * the motor by setting VMAX to 0.
 *
 * @param dev    Zephyr device pointer.
 * @param enable true to enable, false to disable.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_stepper_enable(const struct device *dev, bool enable)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (cfg->en_gpio.port != NULL) {
		ret = gpio_pin_set_dt(&cfg->en_gpio, enable ? 1 : 0);
		if (ret) {
			LOG_ERR("Failed to set EN GPIO: %d", ret);
			goto unlock;
		}
	}

	if (!enable) {
		/* Stop the motor when disabling */
		ret = <devname>_spi_write(dev, <DEVNAME>_REG_VMAX, 0);
		if (ret) {
			LOG_ERR("Failed to stop motor: %d", ret);
			goto unlock;
		}
	}

	ret = 0;

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

/* ---------- Stepper API: move_by --------------------------------------- */

/**
 * @brief Move the motor by a relative number of microsteps.
 *
 * Calculates the new absolute target from the current position and
 * the relative displacement, applies the motion profile, and writes
 * the target position.
 *
 * @param dev         Zephyr device pointer.
 * @param micro_steps Relative displacement in microsteps (signed).
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_stepper_move_by(const struct device *dev,
				     int32_t micro_steps)
{
	struct <devname>_data *data = dev->data;
	uint32_t xactual;
	int32_t current_pos;
	int32_t target_pos;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Read current actual position */
	ret = <devname>_spi_read(dev, <DEVNAME>_REG_XACTUAL, &xactual);
	if (ret) {
		LOG_ERR("Failed to read XACTUAL: %d", ret);
		goto unlock;
	}

	current_pos = (int32_t)xactual;
	target_pos = current_pos + micro_steps;

	/* Switch to position mode */
	ret = <devname>_spi_write(dev, <DEVNAME>_REG_RAMPMODE,
				  <DEVNAME>_RAMP_POSITION);
	if (ret) {
		goto unlock;
	}

	/* Apply motion profile */
	ret = <devname>_set_motion_profile(dev);
	if (ret) {
		goto unlock;
	}

	/* Write target position */
	ret = <devname>_spi_write(dev, <DEVNAME>_REG_XTARGET,
				  (uint32_t)target_pos);
	if (ret) {
		goto unlock;
	}

	/* Start polling for position reached event */
	if (data->callback) {
		k_work_reschedule(&data->rampstat_work,
				  K_MSEC(CONFIG_<DEVNAME>_RAMPSTAT_POLL_INTERVAL_MS));
	}

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

/* ---------- Stepper API: move_to --------------------------------------- */

/**
 * @brief Move the motor to an absolute microstep position.
 *
 * Applies the motion profile and writes the target position register.
 * Triggers position-reached event polling if a callback is registered.
 *
 * @param dev      Zephyr device pointer.
 * @param position Absolute target position in microsteps.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_stepper_move_to(const struct device *dev,
				     int32_t position)
{
	struct <devname>_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Switch to position mode */
	ret = <devname>_spi_write(dev, <DEVNAME>_REG_RAMPMODE,
				  <DEVNAME>_RAMP_POSITION);
	if (ret) {
		goto unlock;
	}

	/* Apply motion profile */
	ret = <devname>_set_motion_profile(dev);
	if (ret) {
		goto unlock;
	}

	/* Write target position */
	ret = <devname>_spi_write(dev, <DEVNAME>_REG_XTARGET,
				  (uint32_t)position);
	if (ret) {
		goto unlock;
	}

	/* Start polling for position reached event */
	if (data->callback) {
		k_work_reschedule(&data->rampstat_work,
				  K_MSEC(CONFIG_<DEVNAME>_RAMPSTAT_POLL_INTERVAL_MS));
	}

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

/* ---------- Stepper API: set_max_velocity ------------------------------ */

/**
 * @brief Set the maximum velocity for subsequent motion commands.
 *
 * Updates the VMAX register and caches the value in the data struct.
 *
 * @param dev         Zephyr device pointer.
 * @param max_velocity Maximum velocity in microsteps per second.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_stepper_set_max_velocity(const struct device *dev,
					      uint32_t max_velocity)
{
	struct <devname>_data *data = dev->data;
	int ret;

	if (max_velocity > <DEVNAME>_VMAX_MSK) {
		LOG_ERR("Velocity %u exceeds maximum %u",
			max_velocity, <DEVNAME>_VMAX_MSK);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = <devname>_spi_write(dev, <DEVNAME>_REG_VMAX, max_velocity);
	if (ret == 0) {
		data->max_velocity = max_velocity;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

/* ---------- Stepper API: set_micro_step_res ---------------------------- */

/**
 * @brief Set the microstepping resolution.
 *
 * Updates the MRES field in the CHOPCONF register.
 *
 * @param dev Zephyr device pointer.
 * @param res Microstepping resolution enum value.
 * @return 0 on success, -EINVAL for unsupported resolutions.
 */
static int <devname>_stepper_set_micro_step_res(const struct device *dev,
						enum stepper_micro_step_resolution res)
{
	struct <devname>_data *data = dev->data;
	int mres;
	int ret;

	mres = <devname>_mres_from_ustep(res);
	if (mres < 0) {
		LOG_ERR("Unsupported microstepping resolution");
		return mres;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = <devname>_spi_update(dev, <DEVNAME>_REG_CHOPCONF,
				   <DEVNAME>_MRES_MSK,
				   FIELD_PREP(<DEVNAME>_MRES_MSK, mres));
	if (ret == 0) {
		data->mres = (uint8_t)mres;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

/* ---------- Stepper API: get_micro_step_res ---------------------------- */

/**
 * @brief Get the current microstepping resolution.
 *
 * Returns the cached microstepping resolution from the data struct.
 *
 * @param dev Zephyr device pointer.
 * @param res Pointer to store the microstepping resolution.
 * @return 0 on success, -EINVAL if the cached value is invalid.
 */
static int <devname>_stepper_get_micro_step_res(const struct device *dev,
						enum stepper_micro_step_resolution *res)
{
	const struct <devname>_data *data = dev->data;

	switch (data->mres) {
	case <DEVNAME>_MRES_FULLSTEP:
		*res = STEPPER_MICRO_STEP_1;
		break;
	case <DEVNAME>_MRES_2:
		*res = STEPPER_MICRO_STEP_2;
		break;
	case <DEVNAME>_MRES_4:
		*res = STEPPER_MICRO_STEP_4;
		break;
	case <DEVNAME>_MRES_8:
		*res = STEPPER_MICRO_STEP_8;
		break;
	case <DEVNAME>_MRES_16:
		*res = STEPPER_MICRO_STEP_16;
		break;
	case <DEVNAME>_MRES_32:
		*res = STEPPER_MICRO_STEP_32;
		break;
	case <DEVNAME>_MRES_64:
		*res = STEPPER_MICRO_STEP_64;
		break;
	case <DEVNAME>_MRES_128:
		*res = STEPPER_MICRO_STEP_128;
		break;
	case <DEVNAME>_MRES_256:
		*res = STEPPER_MICRO_STEP_256;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* ---------- Stepper API: set_reference_position ------------------------ */

/**
 * @brief Preset the current position counter.
 *
 * Switches to hold mode and writes the XACTUAL register with the
 * specified position value. Used for homing or position calibration.
 *
 * @param dev      Zephyr device pointer.
 * @param position New reference position in microsteps.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_stepper_set_reference_position(const struct device *dev,
						    int32_t position)
{
	struct <devname>_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Switch to hold mode to allow position preset */
	ret = <devname>_spi_write(dev, <DEVNAME>_REG_RAMPMODE,
				  <DEVNAME>_RAMP_HOLD);
	if (ret) {
		goto unlock;
	}

	ret = <devname>_spi_write(dev, <DEVNAME>_REG_XACTUAL,
				  (uint32_t)position);

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

/* ---------- Stepper API: get_actual_position --------------------------- */

/**
 * @brief Read the current actual position from the XACTUAL register.
 *
 * @param dev      Zephyr device pointer.
 * @param position Pointer to store the current position.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_stepper_get_actual_position(const struct device *dev,
						 int32_t *position)
{
	struct <devname>_data *data = dev->data;
	uint32_t reg_val;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = <devname>_spi_read(dev, <DEVNAME>_REG_XACTUAL, &reg_val);
	if (ret == 0) {
		*position = (int32_t)reg_val;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

/* ---------- Stepper API: is_moving ------------------------------------- */

/**
 * @brief Check if the motor is currently in motion.
 *
 * Reads the VACTUAL register. The motor is considered moving if
 * the velocity is non-zero.
 *
 * @param dev       Zephyr device pointer.
 * @param is_moving Pointer to store the result (true = moving).
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_stepper_is_moving(const struct device *dev,
				       bool *is_moving)
{
	struct <devname>_data *data = dev->data;
	uint32_t vactual;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = <devname>_spi_read(dev, <DEVNAME>_REG_VACTUAL, &vactual);
	if (ret == 0) {
		*is_moving = (vactual != 0);
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

/* ---------- Stepper API: set_event_callback ---------------------------- */

/**
 * @brief Register a callback for stepper motor events.
 *
 * The callback is invoked when the motor completes a move
 * (STEPPER_EVENT_STEPS_COMPLETED). If set to NULL, event
 * notifications are disabled and ramp status polling is stopped.
 *
 * @param dev       Zephyr device pointer.
 * @param callback  Event callback function, or NULL to disable.
 * @param user_data User data passed to the callback.
 * @return 0 on success.
 */
static int <devname>_stepper_set_event_callback(const struct device *dev,
						stepper_event_callback_t callback,
						void *user_data)
{
	struct <devname>_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	data->callback = callback;
	data->event_cb_user_data = user_data;

	if (callback == NULL) {
		/* Cancel any pending ramp status polling */
		k_work_cancel_delayable(&data->rampstat_work);
	}

	k_mutex_unlock(&data->lock);
	return 0;
}

/* ---------- Device initialisation -------------------------------------- */

/**
 * @brief Initialise the <DEVNAME> stepper motor controller.
 *
 * Called automatically by the Zephyr device model at boot (via
 * DEVICE_DT_INST_DEFINE). Verifies SPI bus readiness, configures the
 * EN GPIO, sets up the driver stage (DRV_CONF, GLOBAL_SCALER, IHOLD_IRUN),
 * the chopper (CHOPCONF), the microstepping resolution, and the full
 * motion profile from devicetree parameters.
 *
 * @param dev Zephyr device pointer.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_init(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	enum stepper_micro_step_resolution ustep_res;
	int mres;
	int ret;

	data->dev = dev;
	k_mutex_init(&data->lock);
	k_work_init_delayable(&data->rampstat_work,
			      <devname>_rampstat_work_handler);

	/* Verify SPI bus is ready */
	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Configure EN GPIO if available */
	if (cfg->en_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->en_gpio)) {
			LOG_ERR("EN GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->en_gpio,
					    GPIO_OUTPUT_ACTIVE);
		if (ret) {
			LOG_ERR("Failed to configure EN GPIO: %d", ret);
			return ret;
		}
	}

	/* Configure DIAG0 GPIO if available */
	if (cfg->diag0_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->diag0_gpio)) {
			LOG_ERR("DIAG0 GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->diag0_gpio, GPIO_INPUT);
		if (ret) {
			LOG_ERR("Failed to configure DIAG0 GPIO: %d", ret);
			return ret;
		}
	}

	/* Configure driver settings: DRV_CONF (slope control + current range) */
	ret = <devname>_spi_write(dev, <DEVNAME>_REG_DRV_CONF,
				  FIELD_PREP(<DEVNAME>_SLOPE_CONTROL_MSK,
					     cfg->slope_control) |
				  FIELD_PREP(<DEVNAME>_CURRENT_RANGE_MSK,
					     cfg->current_range));
	if (ret) {
		LOG_ERR("Failed to write DRV_CONF: %d", ret);
		return ret;
	}

	/* Configure motor current: GLOBAL_SCALER */
	ret = <devname>_spi_write(dev, <DEVNAME>_REG_GLOBAL_SCALER,
				  cfg->global_scaler);
	if (ret) {
		LOG_ERR("Failed to write GLOBAL_SCALER: %d", ret);
		return ret;
	}

	/* Configure motor current: IHOLD_IRUN */
	ret = <devname>_spi_write(dev, <DEVNAME>_REG_IHOLD_IRUN,
				  FIELD_PREP(<DEVNAME>_IRUNDELAY_MSK,
					     cfg->irun_delay) |
				  FIELD_PREP(<DEVNAME>_IHOLDDELAY_MSK,
					     cfg->ihold_delay) |
				  FIELD_PREP(<DEVNAME>_IRUN_MSK,
					     cfg->current_run) |
				  FIELD_PREP(<DEVNAME>_IHOLD_MSK,
					     cfg->current_hold));
	if (ret) {
		LOG_ERR("Failed to write IHOLD_IRUN: %d", ret);
		return ret;
	}

	/* Set microstepping resolution and chopper settings */
	ustep_res = <devname>_dt_to_ustep(cfg->micro_step_res);
	mres = <devname>_mres_from_ustep(ustep_res);
	if (mres < 0) {
		LOG_ERR("Invalid microstepping resolution: %d",
			cfg->micro_step_res);
		return mres;
	}

	data->mres = (uint8_t)mres;

	ret = <devname>_spi_update(dev, <DEVNAME>_REG_CHOPCONF,
				   <DEVNAME>_TOFF_MSK | <DEVNAME>_TBL_MSK |
				   <DEVNAME>_MRES_MSK,
				   FIELD_PREP(<DEVNAME>_TOFF_MSK,
					      cfg->toff) |
				   FIELD_PREP(<DEVNAME>_TBL_MSK,
					      cfg->tbl) |
				   FIELD_PREP(<DEVNAME>_MRES_MSK, mres));
	if (ret) {
		LOG_ERR("Failed to write CHOPCONF: %d", ret);
		return ret;
	}

	/* Set initial ramp mode to position */
	ret = <devname>_spi_write(dev, <DEVNAME>_REG_RAMPMODE,
				  <DEVNAME>_RAMP_POSITION);
	if (ret) {
		LOG_ERR("Failed to set ramp mode: %d", ret);
		return ret;
	}

	/* Apply motion profile from devicetree */
	ret = <devname>_set_motion_profile(dev);
	if (ret) {
		LOG_ERR("Failed to apply motion profile: %d", ret);
		return ret;
	}

	data->max_velocity = cfg->vmax;

	LOG_INF("<DEVNAME> initialised on %s", cfg->spi.bus->name);

	return 0;
}

/* ---------- Stepper driver API structure ------------------------------- */

static DEVICE_API(stepper, <devname>_stepper_api) = {
	.enable              = <devname>_stepper_enable,
	.move_by             = <devname>_stepper_move_by,
	.move_to             = <devname>_stepper_move_to,
	.set_max_velocity    = <devname>_stepper_set_max_velocity,
	.set_micro_step_res  = <devname>_stepper_set_micro_step_res,
	.get_micro_step_res  = <devname>_stepper_get_micro_step_res,
	.set_reference_position = <devname>_stepper_set_reference_position,
	.get_actual_position = <devname>_stepper_get_actual_position,
	.is_moving           = <devname>_stepper_is_moving,
	.set_event_callback  = <devname>_stepper_set_event_callback,
};

/* ---------- Device instantiation macros -------------------------------- */

/**
 * Per-instance config and data allocation.
 *
 * DEVICE_DT_INST_DEFINE generates:
 *   - A static <devname>_config struct initialised from devicetree
 *   - A static <devname>_data struct (zero-initialised)
 *   - The struct device entry in the device table
 *   - Calls <devname>_init() at the POST_KERNEL init level
 */
#define <DEVNAME>_DEFINE(inst)						\
	static struct <devname>_data <devname>_data_##inst;		\
									\
	static const struct <devname>_config <devname>_config_##inst = {\
		.spi = SPI_DT_SPEC_INST_GET(inst,			\
					    SPI_WORD_SET(8) |		\
					    SPI_TRANSFER_MSB |		\
					    SPI_MODE_CPOL |		\
					    SPI_MODE_CPHA,		\
					    0),				\
		.en_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,		\
						    en_gpios,		\
						    { 0 }),		\
		.diag0_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,		\
						       diag0_gpios,	\
						       { 0 }),		\
		.clock_frequency = DT_INST_PROP(inst, clock_frequency),	\
		.micro_step_res = DT_INST_PROP(inst, micro_step_res),	\
		.current_run = DT_INST_PROP(inst, current_run),		\
		.current_hold = DT_INST_PROP(inst, current_hold),	\
		.ihold_delay = DT_INST_PROP(inst, ihold_delay),	\
		.irun_delay = DT_INST_PROP(inst, irun_delay),		\
		.global_scaler = DT_INST_PROP(inst, global_scaler),	\
		.toff = DT_INST_PROP(inst, toff),			\
		.tbl = DT_INST_PROP(inst, tbl),				\
		.slope_control = DT_INST_PROP(inst, slope_control),	\
		.current_range = DT_INST_PROP(inst, current_range),	\
		.vstart = DT_INST_PROP(inst, vstart),			\
		.a1 = DT_INST_PROP(inst, a1),				\
		.v1 = DT_INST_PROP(inst, v1),				\
		.a2 = DT_INST_PROP(inst, a2),				\
		.v2 = DT_INST_PROP(inst, v2),				\
		.amax = DT_INST_PROP(inst, amax),			\
		.vmax = DT_INST_PROP(inst, vmax),			\
		.dmax = DT_INST_PROP(inst, dmax),			\
		.d1 = DT_INST_PROP(inst, d1),				\
		.d2 = DT_INST_PROP(inst, d2),				\
		.vstop = DT_INST_PROP(inst, vstop),			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			      <devname>_init,				\
			      NULL, /* PM not supported */		\
			      &<devname>_data_##inst,			\
			      &<devname>_config_##inst,			\
			      POST_KERNEL,				\
			      CONFIG_STEPPER_INIT_PRIORITY,		\
			      &<devname>_stepper_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_DEFINE)
```

---

## 9. Test Skeleton (`tests/drivers/stepper/<devname>/`)

### 9.1 `testcase.yaml`

```yaml
tests:
  drivers.stepper.<devname>:
    tags:
      - drivers
      - stepper
      - motor
    depends_on: spi
    platform_allow:
      - native_sim
    harness: ztest
```

### 9.2 `prj.conf`

```conf
CONFIG_ZTEST=y
CONFIG_STEPPER=y
CONFIG_SPI=y
CONFIG_<DEVNAME>=y
CONFIG_LOG=y
CONFIG_STEPPER_LOG_LEVEL_DBG=y
```

### 9.3 `boards/native_sim.overlay`

```dts
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/ {
    fake_spi: spi {
        compatible = "zephyr,spi-emul-controller";
        #address-cells = <1>;
        #size-cells = <0>;

        <devname>: <devname>@0 {
            compatible = "adi,<devname>";
            reg = <0>;
            spi-max-frequency = <4000000>;
            clock-frequency = <16000000>;
            micro-step-res = <256>;
            current-run = <16>;
            current-hold = <8>;
            amax = <1000>;
            vmax = <50000>;
            dmax = <1000>;
            d1 = <10>;
            vstop = <10>;
        };
    };
};
```

### 9.4 `src/main.c`

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/stepper.h>

static const struct device *get_dev(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(<devname>));

	zassert_not_null(dev, "Device not found");
	zassert_true(device_is_ready(dev), "Device not ready");

	return dev;
}

ZTEST(test_<devname>, test_enable_disable)
{
	const struct device *dev = get_dev();
	int ret;

	ret = stepper_enable(dev, true);
	zassert_ok(ret, "stepper_enable(true) failed: %d", ret);

	ret = stepper_enable(dev, false);
	zassert_ok(ret, "stepper_enable(false) failed: %d", ret);
}

ZTEST(test_<devname>, test_set_max_velocity)
{
	const struct device *dev = get_dev();
	int ret;

	ret = stepper_set_max_velocity(dev, 10000);
	zassert_ok(ret, "set_max_velocity failed: %d", ret);
}

ZTEST(test_<devname>, test_set_micro_step_res)
{
	const struct device *dev = get_dev();
	enum stepper_micro_step_resolution res;
	int ret;

	ret = stepper_set_micro_step_res(dev, STEPPER_MICRO_STEP_128);
	zassert_ok(ret, "set_micro_step_res failed: %d", ret);

	ret = stepper_get_micro_step_res(dev, &res);
	zassert_ok(ret, "get_micro_step_res failed: %d", ret);
	zassert_equal(res, STEPPER_MICRO_STEP_128,
		      "Expected MICRO_STEP_128, got %d", res);
}

ZTEST(test_<devname>, test_set_reference_position)
{
	const struct device *dev = get_dev();
	int32_t pos;
	int ret;

	ret = stepper_set_reference_position(dev, 1000);
	zassert_ok(ret, "set_reference_position failed: %d", ret);

	ret = stepper_get_actual_position(dev, &pos);
	zassert_ok(ret, "get_actual_position failed: %d", ret);

	TC_PRINT("Position after preset: %d\n", pos);
}

ZTEST(test_<devname>, test_move_to)
{
	const struct device *dev = get_dev();
	int ret;

	ret = stepper_enable(dev, true);
	zassert_ok(ret, "stepper_enable failed: %d", ret);

	ret = stepper_move_to(dev, 51200);
	zassert_ok(ret, "move_to failed: %d", ret);

	TC_PRINT("Move to 51200 microsteps initiated\n");
}

ZTEST(test_<devname>, test_move_by)
{
	const struct device *dev = get_dev();
	int ret;

	ret = stepper_enable(dev, true);
	zassert_ok(ret, "stepper_enable failed: %d", ret);

	ret = stepper_move_by(dev, 5000);
	zassert_ok(ret, "move_by +5000 failed: %d", ret);

	ret = stepper_move_by(dev, -5000);
	zassert_ok(ret, "move_by -5000 failed: %d", ret);

	TC_PRINT("Relative move commands issued\n");
}

ZTEST(test_<devname>, test_is_moving)
{
	const struct device *dev = get_dev();
	bool moving;
	int ret;

	ret = stepper_is_moving(dev, &moving);
	zassert_ok(ret, "is_moving failed: %d", ret);

	TC_PRINT("Motor is %smoving\n", moving ? "" : "not ");
}

ZTEST_SUITE(test_<devname>, NULL, NULL, NULL, NULL, NULL);
```

---

## 10. Conventions

### Zephyr-specific conventions to follow:

1. **SPDX headers** -- use `/* SPDX-License-Identifier: Apache-2.0 */` (not
   BSD-3-Clause as in no-OS). Zephyr uses Apache 2.0 throughout.

2. **DT_DRV_COMPAT** -- define `DT_DRV_COMPAT adi_<devname>` at the very top
   of each source file, before any includes. The compatible string uses commas
   replaced by underscores.

3. **Logging** -- use `LOG_MODULE_REGISTER(<devname>, CONFIG_STEPPER_LOG_LEVEL)`
   in the main source file and `LOG_MODULE_DECLARE(...)` in supplementary
   files (e.g. SPI transport). Never use `printk` in driver code.

4. **Config vs Data** -- devicetree-derived constants go in the `config`
   struct (const, ROM). Mutable runtime state goes in the `data` struct
   (RAM). Access via `dev->config` and `dev->data`.

5. **SPI access** -- always use `spi_transceive_dt()` / `spi_write_dt()` with
   the `spi_dt_spec` from the config struct. Motor controllers use 5-byte SPI
   frames (1 address + 4 data) with bit 7 = write flag. Separate the SPI
   transport into its own source file (`<devname>_spi.c`) for clarity.

6. **GPIO access** -- always use `gpio_dt_spec` and `_dt()` suffixed
   functions. Check readiness with `gpio_is_ready_dt()`. Use
   `GPIO_DT_SPEC_INST_GET_OR()` for optional GPIOs.

7. **Stepper subsystem** -- implement the `stepper_driver_api` struct using
   `DEVICE_API(stepper, ...)`. The core entry points are:
   - `enable` -- enable/disable driver outputs
   - `move_by` -- relative position move
   - `move_to` -- absolute position move
   - `set_max_velocity` -- configure velocity limit
   - `set_micro_step_res` / `get_micro_step_res` -- microstepping control
   - `set_reference_position` -- preset position counter
   - `get_actual_position` -- read current position
   - `is_moving` -- check motion status
   - `set_event_callback` -- register completion event handler

8. **Event callbacks** -- use `stepper_event_callback_t` for asynchronous
   motion completion events. Poll the RAMP_STAT register via a
   `k_work_delayable` when GPIO-based interrupts are not available.

9. **Thread safety** -- protect all register access with a `k_mutex`.
   Motor controller operations (read-modify-write, multi-register motion
   profile writes) must be atomic with respect to concurrent API calls.

10. **No dynamic allocation** -- Zephyr drivers must not call `malloc` /
    `k_malloc`. All memory is statically allocated by the instantiation
    macros.

11. **Instantiation** -- use `DEVICE_DT_INST_DEFINE()` and
    `DT_INST_FOREACH_STATUS_OKAY()` for multi-instance support. Each
    DT node with `status = "okay"` gets its own config/data pair.

12. **Motion profile** -- store all ramp parameters (VSTART, A1, V1, A2, V2,
    AMAX, VMAX, DMAX, D1, D2, VSTOP) in the devicetree binding. Apply the
    complete profile atomically in a helper function before each move.

13. **Init level** -- stepper drivers use `POST_KERNEL` init level with
    `CONFIG_STEPPER_INIT_PRIORITY`. The SPI bus must be ready before the
    stepper driver initialises.

14. **Error handling** -- return negative errno values (`-ENODEV`,
    `-ENOTSUP`, `-EIO`, `-EINVAL`, etc.). Log errors with `LOG_ERR` and
    debug info with `LOG_DBG`.

15. **Naming** -- source files use lowercase with underscores. The
    compatible string is `"adi,<devname>"`. Kconfig symbols are
    uppercase: `CONFIG_<DEVNAME>`.

16. **Byte order** -- motor controller SPI frames are big-endian (MSB first).
    Use explicit byte shifting for the 5-byte frame protocol rather than
    `sys_get_be32()`, since the frame includes a status/address byte.

17. **Devicetree binding includes** -- use `stepper-controller.yaml` and
    `spi-device.yaml` as base includes for the binding. This ensures
    compatibility with the stepper subsystem infrastructure.

---

## 11. Commit Format

Follow the Zephyr commit message format:

```
drivers: stepper: <devname>: add <devname> stepper motor controller driver

Add support for the Analog Devices <DEVNAME> stepper motor controller/driver
with SPI interface. The driver provides position and velocity motion control
through the Zephyr stepper API, with configurable multi-phase
acceleration/deceleration ramps and up to 256x microstepping. Includes
ramp status polling for position-reached event callbacks.

Signed-off-by: Your Name <your.name@analog.com>
```

For adding only the devicetree binding:

```
dts: bindings: add binding for adi,<devname>

Add devicetree binding for the Analog Devices <DEVNAME> stepper motor
controller. Supports SPI bus configuration, EN/DIAG0 GPIO pins,
microstepping resolution, motor current settings, chopper configuration,
and full S-curve motion profile parameters.

Signed-off-by: Your Name <your.name@analog.com>
```
