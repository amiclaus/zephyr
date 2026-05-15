# RTC Driver Template -- Zephyr RTOS

Reference driver: `drivers/rtc/rtc_pcf8563.c` in the Zephyr tree.

This template covers every file needed to add a new Zephyr RTC
subsystem driver for an Analog Devices real-time clock IC (e.g.,
MAX31343).  Replace `<devname>` with the part number (e.g., `max31343`)
and `<DEVNAME>` with its uppercase form (e.g., `MAX31343`) throughout.

---

## 1. Purpose & Zephyr Subsystem Mapping

Zephyr RTC drivers live under the **RTC subsystem**
(`drivers/rtc/`).  The RTC API provides a uniform interface for
applications to set/get calendar time and configure alarms without
knowing hardware-specific register details.

### RTC API Functions

| Zephyr RTC API Function         | What it does                                   |
|---------------------------------|------------------------------------------------|
| `rtc_set_time()`               | Set the calendar date and time                 |
| `rtc_get_time()`               | Read the current date and time                 |
| `rtc_alarm_get_supported_fields()` | Query which alarm time fields the HW supports |
| `rtc_alarm_set_time()`         | Set the alarm match time and field mask         |
| `rtc_alarm_get_time()`         | Read back the current alarm configuration       |
| `rtc_alarm_is_pending()`       | Check whether an alarm has fired                |
| `rtc_alarm_set_callback()`     | Register a callback invoked when alarm fires    |
| `rtc_update_set_callback()`    | Register a callback invoked every second         |

The driver implements the `rtc_driver_api` struct (via the
`DEVICE_API(rtc, ...)` macro) which contains pointers to the driver's
implementations of the above functions.

### Time Representation

Zephyr represents RTC time with `struct rtc_time`, which mirrors the
POSIX `struct tm` layout:

```c
struct rtc_time {
    int tm_sec;    /* seconds [0-59]           */
    int tm_min;    /* minutes [0-59]           */
    int tm_hour;   /* hours   [0-23]           */
    int tm_mday;   /* day of month [1-31]      */
    int tm_mon;    /* month [0-11] (Jan = 0)   */
    int tm_year;   /* years since 1900         */
    int tm_wday;   /* day of week [0-6]        */
    int tm_yday;   /* day of year [0-365]      */
    int tm_isdst;  /* daylight saving flag     */
    int tm_nsec;   /* nanoseconds [0-999999999]*/
};
```

**Important encoding differences from no-OS:**
- `tm_mon` is 0-based (January = 0), not 1-based like no-OS `mon`.
- `tm_year` is years since 1900, not an absolute year.
  For example, 2025 is represented as `tm_year = 125`.
- `tm_mday` is 1-based (same as no-OS `day`).

### Alarm Field Masks

Alarm match fields are specified with a bitmask:

| Mask                          | Meaning                   |
|-------------------------------|---------------------------|
| `RTC_ALARM_TIME_MASK_SECOND`  | Match seconds             |
| `RTC_ALARM_TIME_MASK_MINUTE`  | Match minutes             |
| `RTC_ALARM_TIME_MASK_HOUR`    | Match hours               |
| `RTC_ALARM_TIME_MASK_MONTHDAY`| Match day of month        |
| `RTC_ALARM_TIME_MASK_MONTH`   | Match month               |
| `RTC_ALARM_TIME_MASK_YEAR`    | Match year                |
| `RTC_ALARM_TIME_MASK_WEEKDAY` | Match day of week         |

### BCD Encoding

Most RTC ICs store time registers in BCD (Binary-Coded Decimal) format.
Zephyr does not provide global BCD helpers, so each driver defines its
own static inline conversion functions:

```c
static inline uint8_t bin2bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

static inline uint8_t bcd2bin(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}
```

---

## 2. File Checklist

```
drivers/rtc/
    rtc_<devname>.c              # Driver implementation (single file)
    Kconfig.<devname>            # Kconfig fragment
    CMakeLists.txt               # (append to existing)
    Kconfig                      # (append to existing)

dts/bindings/rtc/
    adi,<devname>.yaml           # Devicetree binding

tests/drivers/rtc/<devname>/
    testcase.yaml                # Test metadata
    prj.conf                     # Test project config
    src/main.c                   # Test source
```

Note: unlike the sensor subsystem, RTC drivers are typically
implemented as a single `.c` file (no separate header). Register
defines, structs, and BCD helpers are all placed within the `.c` file.
If the driver is large or shared with emulators/tests, extract a
header to `drivers/rtc/<devname>.h`.

---

## 3. Devicetree Binding (`dts/bindings/rtc/adi,<devname>.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  ADI <DEVNAME> real-time clock with I2C interface.
  Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/<DEVNAME>.pdf

compatible: "adi,<devname>"

include: [rtc-device.yaml, i2c-device.yaml]

properties:
  int-gpios:
    type: phandle-array
    description: |
      Interrupt output pin.  Active-low open-drain on most RTCs.
      Required for alarm callback support.
      Example: int-gpios = <&gpio0 5 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;

  trickle-resistor-ohms:
    type: int
    description: |
      Trickle charger series resistance in ohms for the backup
      supercapacitor or rechargeable battery.  Set to 0 to disable.
      Typical values: 3000, 6000, 11000.
    default: 0
    enum:
      - 0
      - 3000
      - 6000
      - 11000

  backup-battery:
    type: boolean
    description: |
      Enable automatic backup battery switchover.  When set, the
      device will continue timekeeping from the backup supply when
      VCC drops below the power-fail threshold.
```

### Example devicetree node

```dts
&i2c0 {
    <devname>: <devname>@68 {
        compatible = "adi,<devname>";
        reg = <0x68>;
        status = "okay";
        int-gpios = <&gpio0 5 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        backup-battery;
        trickle-resistor-ohms = <3000>;
    };
};
```

---

## 4. Kconfig

### `Kconfig.<devname>`

```kconfig
# ADI <DEVNAME> Real-Time Clock

# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config RTC_<DEVNAME>
	bool "<DEVNAME> Real-Time Clock"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	select I2C
	help
	  Enable driver for the ADI <DEVNAME> real-time clock.
	  Supports time get/set, alarms, and optional trickle charger
	  configuration via I2C.
```

### Append to `drivers/rtc/Kconfig`

```kconfig
source "drivers/rtc/Kconfig.<devname>"
```

### Append to `drivers/rtc/CMakeLists.txt`

```cmake
zephyr_library_sources_ifdef(CONFIG_RTC_<DEVNAME> rtc_<devname>.c)
```

---

## 5. CMakeLists.txt

No separate `CMakeLists.txt` is needed for the driver itself since
RTC drivers are built from the top-level `drivers/rtc/CMakeLists.txt`.
The only change is appending the `zephyr_library_sources_ifdef` line
shown in section 4.

If the driver were complex enough to live in its own subdirectory
(uncommon for RTCs), you would create:

```cmake
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  rtc_<devname>.c
)
```

---

## 6. Driver Source (`rtc_<devname>.c`)

This is the core of the template.  RTC drivers are typically a single
self-contained `.c` file.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(rtc_<devname>, CONFIG_RTC_LOG_LEVEL);

/* ---- Register Addresses: Time/Date -------------------------------- */

#define <DEVNAME>_REG_SECONDS        0x06
#define <DEVNAME>_REG_MINUTES        0x07
#define <DEVNAME>_REG_HOURS          0x08
#define <DEVNAME>_REG_DAY            0x09  /* day of week */
#define <DEVNAME>_REG_DATE           0x0A  /* day of month */
#define <DEVNAME>_REG_MONTH          0x0B
#define <DEVNAME>_REG_YEAR           0x0C

/* ---- Register Addresses: Alarm 1 ---------------------------------- */

#define <DEVNAME>_REG_ALM1_SEC       0x0D
#define <DEVNAME>_REG_ALM1_MIN       0x0E
#define <DEVNAME>_REG_ALM1_HRS       0x0F
#define <DEVNAME>_REG_ALM1_DAY_DATE  0x10
#define <DEVNAME>_REG_ALM1_MON       0x11
#define <DEVNAME>_REG_ALM1_YEAR      0x12

/* ---- Register Addresses: Alarm 2 ---------------------------------- */

#define <DEVNAME>_REG_ALM2_MIN       0x13
#define <DEVNAME>_REG_ALM2_HRS       0x14
#define <DEVNAME>_REG_ALM2_DAY_DATE  0x15

/* ---- Register Addresses: Control/Status --------------------------- */

#define <DEVNAME>_REG_STATUS         0x00
#define <DEVNAME>_REG_INT_EN         0x01
#define <DEVNAME>_REG_RTC_RESET      0x02
#define <DEVNAME>_REG_CFG1           0x03
#define <DEVNAME>_REG_CFG2           0x04
#define <DEVNAME>_REG_TIMER_CFG      0x05

/* ---- Register Addresses: Power / Trickle Charger ------------------ */

#define <DEVNAME>_REG_PWR_MGMT       0x18
#define <DEVNAME>_REG_TRICKLE        0x19

/* ---- Status Register Bits ----------------------------------------- */

#define <DEVNAME>_STATUS_A1F         BIT(0)
#define <DEVNAME>_STATUS_A2F         BIT(1)
#define <DEVNAME>_STATUS_TIF         BIT(2)
#define <DEVNAME>_STATUS_OSF         BIT(6)

/* ---- Interrupt Enable Bits ---------------------------------------- */

#define <DEVNAME>_INT_A1IE           BIT(0)
#define <DEVNAME>_INT_A2IE           BIT(1)

/* ---- Config Bits -------------------------------------------------- */

#define <DEVNAME>_CFG1_ENOSC         BIT(1)
#define <DEVNAME>_SWRST              BIT(0)

/* ---- Power Management Bits ---------------------------------------- */

#define <DEVNAME>_PWR_D_MAN_SEL      BIT(2)
#define <DEVNAME>_PWR_D_VBACK        BIT(3)

/* ---- Trickle Charger Bits ----------------------------------------- */

#define <DEVNAME>_TRICKLE_D_MASK     GENMASK(3, 0)
#define <DEVNAME>_TRICKLE_TCHE_MASK  GENMASK(5, 4)

/* ---- BCD Masks for Time Registers --------------------------------- */

#define <DEVNAME>_SEC_MASK           GENMASK(6, 0)
#define <DEVNAME>_MIN_MASK           GENMASK(6, 0)
#define <DEVNAME>_HR_MASK            GENMASK(5, 0)
#define <DEVNAME>_DAY_MASK           GENMASK(2, 0)
#define <DEVNAME>_DATE_MASK          GENMASK(5, 0)
#define <DEVNAME>_MON_MASK           GENMASK(4, 0)
#define <DEVNAME>_CENTURY_BIT        BIT(7)

/* Number of alarms supported by the hardware. */
#define <DEVNAME>_NUM_ALARMS         1

/* Alarm 1 supports sec, min, hr, day-of-month, month, year. */
#define <DEVNAME>_ALARM_SUPPORTED_FIELDS \
	(RTC_ALARM_TIME_MASK_SECOND   | \
	 RTC_ALARM_TIME_MASK_MINUTE   | \
	 RTC_ALARM_TIME_MASK_HOUR     | \
	 RTC_ALARM_TIME_MASK_MONTHDAY | \
	 RTC_ALARM_TIME_MASK_MONTH    | \
	 RTC_ALARM_TIME_MASK_YEAR)

/* ---- BCD Helpers -------------------------------------------------- */

static inline uint8_t bin2bcd(uint8_t val)
{
	return ((val / 10) << 4) | (val % 10);
}

static inline uint8_t bcd2bin(uint8_t val)
{
	return ((val >> 4) * 10) + (val & 0x0F);
}

/* ---- Driver Structures -------------------------------------------- */

/**
 * @brief Per-instance configuration (const, from devicetree).
 */
struct <devname>_config {
	/** I2C bus and address from devicetree. */
	struct i2c_dt_spec i2c;

	/** Interrupt output GPIO (optional). */
	struct gpio_dt_spec int_gpio;

	/** Trickle charger resistance in ohms (0 = disabled). */
	uint32_t trickle_ohms;

	/** Enable backup battery switchover. */
	bool backup_battery;
};

/**
 * @brief Per-instance runtime data (mutable, in RAM).
 */
struct <devname>_data {
	/** Mutex protecting I2C transactions. */
	struct k_mutex lock;

	/** GPIO callback structure for alarm interrupt. */
	struct gpio_callback gpio_cb;

	/** Device back-reference for ISR context. */
	const struct device *dev;

	/** Application alarm callback and user data. */
	rtc_alarm_callback alarm_cb;
	void *alarm_cb_user_data;

	/** Application update callback and user data. */
	rtc_update_callback update_cb;
	void *update_cb_user_data;

	/** Work item for deferred interrupt processing. */
	struct k_work alarm_work;
};

/* ---- Register Access Helpers -------------------------------------- */

/**
 * @brief Read a single register over I2C.
 */
static int <devname>_reg_read(const struct device *dev, uint8_t reg,
			      uint8_t *val)
{
	const struct <devname>_config *cfg = dev->config;

	return i2c_reg_read_byte_dt(&cfg->i2c, reg, val);
}

/**
 * @brief Write a single register over I2C.
 */
static int <devname>_reg_write(const struct device *dev, uint8_t reg,
			       uint8_t val)
{
	const struct <devname>_config *cfg = dev->config;

	return i2c_reg_write_byte_dt(&cfg->i2c, reg, val);
}

/**
 * @brief Update specific bits in a register (read-modify-write).
 */
static int <devname>_reg_update(const struct device *dev, uint8_t reg,
				uint8_t mask, uint8_t val)
{
	const struct <devname>_config *cfg = dev->config;

	return i2c_reg_update_byte_dt(&cfg->i2c, reg, mask, val);
}

/* ---- RTC API: set_time -------------------------------------------- */

/**
 * @brief Set the calendar date and time.
 *
 * Converts the struct rtc_time fields from binary to BCD and writes
 * them to the time registers.  The century bit in the month register
 * is set for years >= 2100 (i.e., tm_year >= 200).
 *
 * @param dev  Device instance.
 * @param timeptr  Pointer to the time to set.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_set_time(const struct device *dev,
			      const struct rtc_time *timeptr)
{
	struct <devname>_data *data = dev->data;
	uint8_t regs[7];
	int year_offset;
	uint8_t century;
	int ret;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	/*
	 * tm_year is years since 1900.  The RTC stores a 2-digit BCD
	 * year relative to 2000 or 2100.
	 *   - tm_year 100..199 -> year 2000..2099 (century = 0)
	 *   - tm_year 200..299 -> year 2100..2199 (century = 1)
	 */
	if (timeptr->tm_year < 100 || timeptr->tm_year > 299) {
		LOG_ERR("Year %d out of range (2000-2199)", timeptr->tm_year + 1900);
		return -EINVAL;
	}

	century = (timeptr->tm_year >= 200) ? 1 : 0;
	year_offset = timeptr->tm_year - (century ? 200 : 100);

	regs[0] = bin2bcd(timeptr->tm_sec);
	regs[1] = bin2bcd(timeptr->tm_min);
	regs[2] = bin2bcd(timeptr->tm_hour);
	regs[3] = bin2bcd(timeptr->tm_wday + 1);    /* HW uses 1-7 */
	regs[4] = bin2bcd(timeptr->tm_mday);
	regs[5] = bin2bcd(timeptr->tm_mon + 1);      /* tm_mon is 0-based */
	if (century) {
		regs[5] |= <DEVNAME>_CENTURY_BIT;
	}
	regs[6] = bin2bcd(year_offset);

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = i2c_burst_write_dt(&((const struct <devname>_config *)dev->config)->i2c,
				 <DEVNAME>_REG_SECONDS, regs, sizeof(regs));

	k_mutex_unlock(&data->lock);

	if (ret) {
		LOG_ERR("Failed to set time: %d", ret);
	}

	return ret;
}

/* ---- RTC API: get_time -------------------------------------------- */

/**
 * @brief Read the current date and time.
 *
 * Reads 7 consecutive registers starting at SECONDS in a single I2C
 * burst.  BCD values are converted to binary and stored in the
 * caller's struct rtc_time.
 *
 * @param dev  Device instance.
 * @param timeptr  Pointer to store the current time.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_get_time(const struct device *dev,
			      struct rtc_time *timeptr)
{
	struct <devname>_data *data = dev->data;
	const struct <devname>_config *cfg = dev->config;
	uint8_t regs[7];
	uint8_t century;
	int ret;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = i2c_burst_read_dt(&cfg->i2c, <DEVNAME>_REG_SECONDS,
				regs, sizeof(regs));

	k_mutex_unlock(&data->lock);

	if (ret) {
		LOG_ERR("Failed to read time: %d", ret);
		return ret;
	}

	timeptr->tm_sec  = bcd2bin(regs[0] & <DEVNAME>_SEC_MASK);
	timeptr->tm_min  = bcd2bin(regs[1] & <DEVNAME>_MIN_MASK);
	timeptr->tm_hour = bcd2bin(regs[2] & <DEVNAME>_HR_MASK);
	timeptr->tm_wday = bcd2bin(regs[3] & <DEVNAME>_DAY_MASK) - 1; /* 0-based */
	timeptr->tm_mday = bcd2bin(regs[4] & <DEVNAME>_DATE_MASK);

	century = regs[5] & <DEVNAME>_CENTURY_BIT;
	timeptr->tm_mon  = bcd2bin(regs[5] & <DEVNAME>_MON_MASK) - 1; /* 0-based */

	timeptr->tm_year = bcd2bin(regs[6]) + (century ? 200 : 100);

	/* Not tracked by hardware -- set to -1 (unknown). */
	timeptr->tm_yday  = -1;
	timeptr->tm_isdst = -1;
	timeptr->tm_nsec  = 0;

	return 0;
}

/* ---- RTC API: alarm_get_supported_fields -------------------------- */

/**
 * @brief Query which alarm time fields the hardware supports.
 *
 * @param dev  Device instance.
 * @param id   Alarm index (must be 0 for single-alarm devices).
 * @param mask Pointer to store the supported field bitmask.
 * @return 0 on success, -EINVAL if alarm id is out of range.
 */
static int <devname>_alarm_get_supported_fields(const struct device *dev,
						uint16_t id,
						uint16_t *mask)
{
	ARG_UNUSED(dev);

	if (id >= <DEVNAME>_NUM_ALARMS) {
		return -EINVAL;
	}

	*mask = <DEVNAME>_ALARM_SUPPORTED_FIELDS;

	return 0;
}

/* ---- RTC API: alarm_set_time -------------------------------------- */

/**
 * @brief Set the alarm match time and field mask.
 *
 * Only fields enabled in @p mask are written to the alarm registers.
 * Disabled fields are set to 0x00 (match-all on most RTCs).
 *
 * After programming the alarm time, the alarm interrupt enable bit
 * is set and any pending alarm flag is cleared.
 *
 * @param dev      Device instance.
 * @param id       Alarm index.
 * @param mask     Bitmask of RTC_ALARM_TIME_MASK_* fields to match.
 * @param timeptr  Pointer to the alarm time values.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_alarm_set_time(const struct device *dev,
				    uint16_t id, uint16_t mask,
				    const struct rtc_time *timeptr)
{
	struct <devname>_data *data = dev->data;
	uint8_t regs[6] = {0};
	int year_offset;
	uint8_t century;
	int ret;

	if (id >= <DEVNAME>_NUM_ALARMS) {
		return -EINVAL;
	}

	if (mask & ~<DEVNAME>_ALARM_SUPPORTED_FIELDS) {
		LOG_ERR("Unsupported alarm fields: 0x%04x", mask);
		return -EINVAL;
	}

	if (timeptr == NULL && mask != 0) {
		return -EINVAL;
	}

	if (mask & RTC_ALARM_TIME_MASK_SECOND) {
		regs[0] = bin2bcd(timeptr->tm_sec);
	}

	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		regs[1] = bin2bcd(timeptr->tm_min);
	}

	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		regs[2] = bin2bcd(timeptr->tm_hour);
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		regs[3] = bin2bcd(timeptr->tm_mday);
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTH) {
		regs[4] = bin2bcd(timeptr->tm_mon + 1);

		if ((mask & RTC_ALARM_TIME_MASK_YEAR) && timeptr->tm_year >= 200) {
			regs[4] |= <DEVNAME>_CENTURY_BIT;
		}
	}

	if (mask & RTC_ALARM_TIME_MASK_YEAR) {
		century = (timeptr->tm_year >= 200) ? 1 : 0;
		year_offset = timeptr->tm_year - (century ? 200 : 100);
		regs[5] = bin2bcd(year_offset);
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Write alarm 1 registers (SEC through YEAR). */
	ret = i2c_burst_write_dt(
		&((const struct <devname>_config *)dev->config)->i2c,
		<DEVNAME>_REG_ALM1_SEC, regs, sizeof(regs));
	if (ret) {
		LOG_ERR("Failed to write alarm registers: %d", ret);
		goto unlock;
	}

	/* Clear any pending alarm flag. */
	ret = <devname>_reg_update(dev, <DEVNAME>_REG_STATUS,
				   <DEVNAME>_STATUS_A1F, 0);
	if (ret) {
		goto unlock;
	}

	/* Enable or disable alarm interrupt based on mask. */
	if (mask != 0) {
		ret = <devname>_reg_update(dev, <DEVNAME>_REG_INT_EN,
					   <DEVNAME>_INT_A1IE,
					   <DEVNAME>_INT_A1IE);
	} else {
		ret = <devname>_reg_update(dev, <DEVNAME>_REG_INT_EN,
					   <DEVNAME>_INT_A1IE, 0);
	}

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

/* ---- RTC API: alarm_get_time -------------------------------------- */

/**
 * @brief Read back the current alarm configuration.
 *
 * @param dev      Device instance.
 * @param id       Alarm index.
 * @param mask     Pointer to store the active field mask.
 * @param timeptr  Pointer to store the alarm time values.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_alarm_get_time(const struct device *dev,
				    uint16_t id, uint16_t *mask,
				    struct rtc_time *timeptr)
{
	struct <devname>_data *data = dev->data;
	const struct <devname>_config *cfg = dev->config;
	uint8_t regs[6];
	uint8_t int_en;
	uint8_t century;
	int ret;

	if (id >= <DEVNAME>_NUM_ALARMS) {
		return -EINVAL;
	}

	if (timeptr == NULL || mask == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = i2c_burst_read_dt(&cfg->i2c, <DEVNAME>_REG_ALM1_SEC,
				regs, sizeof(regs));
	if (ret) {
		LOG_ERR("Failed to read alarm registers: %d", ret);
		goto unlock;
	}

	ret = <devname>_reg_read(dev, <DEVNAME>_REG_INT_EN, &int_en);
	if (ret) {
		goto unlock;
	}

	k_mutex_unlock(&data->lock);

	/* If alarm interrupt is disabled, report empty mask. */
	if (!(int_en & <DEVNAME>_INT_A1IE)) {
		*mask = 0;
		memset(timeptr, 0, sizeof(*timeptr));
		return 0;
	}

	/* Decode the alarm registers. */
	timeptr->tm_sec  = bcd2bin(regs[0] & <DEVNAME>_SEC_MASK);
	timeptr->tm_min  = bcd2bin(regs[1] & <DEVNAME>_MIN_MASK);
	timeptr->tm_hour = bcd2bin(regs[2] & <DEVNAME>_HR_MASK);
	timeptr->tm_mday = bcd2bin(regs[3] & <DEVNAME>_DATE_MASK);

	century = regs[4] & <DEVNAME>_CENTURY_BIT;
	timeptr->tm_mon  = bcd2bin(regs[4] & <DEVNAME>_MON_MASK) - 1;

	timeptr->tm_year = bcd2bin(regs[5]) + (century ? 200 : 100);

	timeptr->tm_wday  = -1;
	timeptr->tm_yday  = -1;
	timeptr->tm_isdst = -1;
	timeptr->tm_nsec  = 0;

	*mask = <DEVNAME>_ALARM_SUPPORTED_FIELDS;

	return 0;

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

/* ---- RTC API: alarm_is_pending ------------------------------------ */

/**
 * @brief Check whether the alarm has fired.
 *
 * Reads the status register and checks the A1F bit.  If pending,
 * the flag is cleared.
 *
 * @param dev  Device instance.
 * @param id   Alarm index.
 * @return 1 if alarm is pending, 0 if not, negative errno on error.
 */
static int <devname>_alarm_is_pending(const struct device *dev,
				      uint16_t id)
{
	struct <devname>_data *data = dev->data;
	uint8_t status;
	int ret;

	if (id >= <DEVNAME>_NUM_ALARMS) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = <devname>_reg_read(dev, <DEVNAME>_REG_STATUS, &status);
	if (ret) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	if (status & <DEVNAME>_STATUS_A1F) {
		/* Clear the alarm flag. */
		ret = <devname>_reg_update(dev, <DEVNAME>_REG_STATUS,
					   <DEVNAME>_STATUS_A1F, 0);
		k_mutex_unlock(&data->lock);
		if (ret) {
			return ret;
		}
		return 1;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

/* ---- Interrupt Handling ------------------------------------------- */

/**
 * @brief Work handler for alarm interrupt (runs in system workqueue).
 *
 * Reads the status register to determine the interrupt source and
 * invokes the registered callback.
 */
static void <devname>_alarm_work_handler(struct k_work *work)
{
	struct <devname>_data *data =
		CONTAINER_OF(work, struct <devname>_data, alarm_work);
	const struct device *dev = data->dev;
	uint8_t status;
	int ret;

	ret = <devname>_reg_read(dev, <DEVNAME>_REG_STATUS, &status);
	if (ret) {
		LOG_ERR("Failed to read status in ISR: %d", ret);
		return;
	}

	if ((status & <DEVNAME>_STATUS_A1F) && data->alarm_cb) {
		/* Clear the alarm flag. */
		<devname>_reg_update(dev, <DEVNAME>_REG_STATUS,
				     <DEVNAME>_STATUS_A1F, 0);
		data->alarm_cb(dev, 0, data->alarm_cb_user_data);
	}

	/* Re-read for update callback (1-second tick). */
	if (data->update_cb) {
		data->update_cb(dev, data->update_cb_user_data);
	}
}

/**
 * @brief GPIO interrupt callback (ISR context).
 *
 * Defers processing to the system workqueue since I2C operations
 * cannot run in interrupt context.
 */
static void <devname>_gpio_callback(const struct device *port,
				    struct gpio_callback *cb,
				    gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct <devname>_data *data =
		CONTAINER_OF(cb, struct <devname>_data, gpio_cb);

	k_work_submit(&data->alarm_work);
}

/* ---- RTC API: alarm_set_callback ---------------------------------- */

/**
 * @brief Register a callback invoked when the alarm fires.
 *
 * @param dev       Device instance.
 * @param id        Alarm index.
 * @param callback  Application callback, or NULL to disable.
 * @param user_data Opaque pointer passed to the callback.
 * @return 0 on success, -EINVAL if alarm id is out of range,
 *         -ENOTSUP if the int-gpios property is not configured.
 */
static int <devname>_alarm_set_callback(const struct device *dev,
					uint16_t id,
					rtc_alarm_callback callback,
					void *user_data)
{
	struct <devname>_data *data = dev->data;
	const struct <devname>_config *cfg = dev->config;

	if (id >= <DEVNAME>_NUM_ALARMS) {
		return -EINVAL;
	}

	if (cfg->int_gpio.port == NULL) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	data->alarm_cb = callback;
	data->alarm_cb_user_data = user_data;

	if (callback != NULL) {
		gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
						GPIO_INT_EDGE_TO_ACTIVE);
	} else {
		gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
						GPIO_INT_DISABLE);
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

/* ---- RTC API: update_set_callback --------------------------------- */

/**
 * @brief Register a callback invoked every second (1 Hz update).
 *
 * Requires the interrupt pin to be connected and configured in DT.
 * The RTC must be configured to assert the interrupt pin on each
 * second tick (device-specific configuration).
 *
 * @param dev       Device instance.
 * @param callback  Application callback, or NULL to disable.
 * @param user_data Opaque pointer passed to the callback.
 * @return 0 on success, -ENOTSUP if int-gpios is not configured.
 */
static int <devname>_update_set_callback(const struct device *dev,
					 rtc_update_callback callback,
					 void *user_data)
{
	struct <devname>_data *data = dev->data;
	const struct <devname>_config *cfg = dev->config;

	if (cfg->int_gpio.port == NULL) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	data->update_cb = callback;
	data->update_cb_user_data = user_data;

	/*
	 * TODO: Enable/disable the 1 Hz square-wave output or the
	 * timer interrupt as the update source.  This is device-specific.
	 * Example for MAX31343: configure SQW output to 1 Hz via CFG2.
	 */

	k_mutex_unlock(&data->lock);

	return 0;
}

/* ---- Trickle Charger Configuration -------------------------------- */

/**
 * @brief Configure the trickle charger from devicetree properties.
 *
 * Maps the trickle-resistor-ohms DT property to the appropriate
 * register field value.  The exact encoding is device-specific.
 *
 * @param dev  Device instance.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_trickle_setup(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t trickle_val;

	if (cfg->trickle_ohms == 0) {
		/* Trickle charger disabled. */
		return <devname>_reg_write(dev, <DEVNAME>_REG_TRICKLE, 0x00);
	}

	/*
	 * Map resistance to register value.  Adjust for your device.
	 * Example encoding:
	 *   3000 ohm  -> 0x25  (diode + 3k)
	 *   6000 ohm  -> 0x26  (diode + 6k)
	 *   11000 ohm -> 0x27  (diode + 11k)
	 */
	switch (cfg->trickle_ohms) {
	case 3000:
		trickle_val = 0x25;
		break;
	case 6000:
		trickle_val = 0x26;
		break;
	case 11000:
		trickle_val = 0x27;
		break;
	default:
		LOG_ERR("Unsupported trickle resistance: %u", cfg->trickle_ohms);
		return -EINVAL;
	}

	return <devname>_reg_write(dev, <DEVNAME>_REG_TRICKLE, trickle_val);
}

/* ---- Driver API Table --------------------------------------------- */

static DEVICE_API(rtc, <devname>_api) = {
	.set_time                 = <devname>_set_time,
	.get_time                 = <devname>_get_time,
	.alarm_get_supported_fields = <devname>_alarm_get_supported_fields,
	.alarm_set_time           = <devname>_alarm_set_time,
	.alarm_get_time           = <devname>_alarm_get_time,
	.alarm_is_pending         = <devname>_alarm_is_pending,
	.alarm_set_callback       = <devname>_alarm_set_callback,
	.update_set_callback      = <devname>_update_set_callback,
};

/* ---- Interrupt Setup ---------------------------------------------- */

/**
 * @brief Set up the GPIO interrupt for alarm/update callbacks.
 *
 * Called during init when int-gpios is present in the devicetree.
 */
static int <devname>_interrupt_init(const struct device *dev)
{
	struct <devname>_data *data = dev->data;
	const struct <devname>_config *cfg = dev->config;
	int ret;

	if (cfg->int_gpio.port == NULL) {
		LOG_DBG("No interrupt GPIO configured");
		return 0;
	}

	if (!gpio_is_ready_dt(&cfg->int_gpio)) {
		LOG_ERR("INT GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (ret) {
		LOG_ERR("Failed to configure INT pin: %d", ret);
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, <devname>_gpio_callback,
			   BIT(cfg->int_gpio.pin));

	ret = gpio_add_callback(cfg->int_gpio.port, &data->gpio_cb);
	if (ret) {
		LOG_ERR("Failed to add GPIO callback: %d", ret);
		return ret;
	}

	data->dev = dev;
	k_work_init(&data->alarm_work, <devname>_alarm_work_handler);

	return 0;
}

/* ---- Initialization ----------------------------------------------- */

/**
 * @brief Initialise a <DEVNAME> instance.
 *
 * Performs:
 *   1. Verify I2C bus is ready.
 *   2. Initialise the mutex.
 *   3. Deassert software reset.
 *   4. Enable the oscillator.
 *   5. Disable all interrupt sources.
 *   6. Configure backup battery switchover (if enabled in DT).
 *   7. Configure trickle charger (if configured in DT).
 *   8. Set up GPIO interrupt (if int-gpios is present).
 *
 * @param dev Device instance.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_init(const struct device *dev)
{
	struct <devname>_data *data = dev->data;
	const struct <devname>_config *cfg = dev->config;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	k_mutex_init(&data->lock);

	/* Deassert software reset. */
	ret = <devname>_reg_update(dev, <DEVNAME>_REG_RTC_RESET,
				   <DEVNAME>_SWRST, 0);
	if (ret) {
		LOG_ERR("Failed to deassert reset: %d", ret);
		return ret;
	}

	/* Enable oscillator. */
	ret = <devname>_reg_update(dev, <DEVNAME>_REG_CFG1,
				   <DEVNAME>_CFG1_ENOSC,
				   <DEVNAME>_CFG1_ENOSC);
	if (ret) {
		LOG_ERR("Failed to enable oscillator: %d", ret);
		return ret;
	}

	/* Disable all interrupts. */
	ret = <devname>_reg_write(dev, <DEVNAME>_REG_INT_EN, 0);
	if (ret) {
		LOG_ERR("Failed to disable interrupts: %d", ret);
		return ret;
	}

	/* Clear any pending status flags. */
	ret = <devname>_reg_write(dev, <DEVNAME>_REG_STATUS, 0);
	if (ret) {
		LOG_ERR("Failed to clear status: %d", ret);
		return ret;
	}

	/* Configure backup battery switchover. */
	if (cfg->backup_battery) {
		ret = <devname>_reg_update(dev, <DEVNAME>_REG_PWR_MGMT,
					   <DEVNAME>_PWR_D_VBACK,
					   <DEVNAME>_PWR_D_VBACK);
		if (ret) {
			LOG_ERR("Failed to enable backup battery: %d", ret);
			return ret;
		}
	}

	/* Configure trickle charger. */
	ret = <devname>_trickle_setup(dev);
	if (ret) {
		LOG_ERR("Failed to configure trickle charger: %d", ret);
		return ret;
	}

	/* Set up interrupt GPIO. */
	ret = <devname>_interrupt_init(dev);
	if (ret) {
		return ret;
	}

	LOG_INF("<DEVNAME> initialised on %s @ 0x%02x",
		cfg->i2c.bus->name, cfg->i2c.addr);

	return 0;
}

/* ---- Instantiation Macros ----------------------------------------- */

/**
 * Interrupt GPIO config -- present only when int-gpios is specified
 * in the devicetree node.
 */
#define <DEVNAME>_INT_GPIO_INIT(inst)                                         \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, int_gpios),                  \
		     (.int_gpio = GPIO_DT_SPEC_INST_GET(inst, int_gpios),),   \
		     (.int_gpio = {0},))

/**
 * Per-instance config/data definitions and device registration.
 *
 * This macro is expanded once per devicetree instance that has
 * compatible = "adi,<devname>" and status = "okay".
 */
#define <DEVNAME>_INST(inst)                                                  \
	static struct <devname>_data <devname>_data_##inst;                   \
                                                                              \
	static const struct <devname>_config <devname>_config_##inst = {       \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                            \
		<DEVNAME>_INT_GPIO_INIT(inst)                                  \
		.trickle_ohms = DT_INST_PROP(inst, trickle_resistor_ohms),    \
		.backup_battery = DT_INST_PROP(inst, backup_battery),         \
	};                                                                     \
                                                                              \
	DEVICE_DT_INST_DEFINE(                                                 \
		inst,                                                          \
		<devname>_init,                                                \
		NULL,                                                          \
		&<devname>_data_##inst,                                        \
		&<devname>_config_##inst,                                      \
		POST_KERNEL,                                                   \
		CONFIG_RTC_INIT_PRIORITY,                                      \
		&<devname>_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_INST)
```

### Key points

- **`DEVICE_DT_INST_DEFINE()`** is used instead of
  `SENSOR_DEVICE_DT_INST_DEFINE()`.  The RTC subsystem does not have
  a sensor-style wrapper macro; use the generic device macro directly.
- **`DEVICE_API(rtc, ...)`** creates a `struct rtc_driver_api` with the
  compile-time type tag required by the device model.
- **`DT_INST_FOREACH_STATUS_OKAY()`** expands the instantiation macro
  once for every `status = "okay"` node with matching compatible.
- **BCD helpers** are defined locally (not provided by Zephyr globally).
- **Mutex** protects all I2C register access since RTC get/set/alarm
  functions may be called from different threads.
- **`COND_CODE_1()`** conditionally includes the GPIO spec only when
  `int-gpios` is present, allowing the driver to work with or without
  an interrupt pin connected.
- **`i2c_burst_read_dt()` / `i2c_burst_write_dt()`** are used for
  multi-register time reads/writes (7 consecutive registers) for
  atomicity and efficiency.
- **`i2c_reg_update_byte_dt()`** is the Zephyr equivalent of the no-OS
  `update_bits()` pattern (read-modify-write).

---

## 7. Application Example

### `samples/drivers/rtc/<devname>/prj.conf`

```
CONFIG_I2C=y
CONFIG_RTC=y
CONFIG_RTC_<DEVNAME>=y
CONFIG_LOG=y
CONFIG_RTC_LOG_LEVEL_INF=y
```

### `samples/drivers/rtc/<devname>/src/main.c`

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Get the RTC device from devicetree. */
static const struct device *rtc_dev = DEVICE_DT_GET(DT_NODELABEL(<devname>));

/* ---- Alarm callback ----------------------------------------------- */

static void alarm_handler(const struct device *dev, uint16_t id,
			   void *user_data)
{
	ARG_UNUSED(user_data);

	LOG_INF("Alarm %u fired!", id);
}

/* ---- Main --------------------------------------------------------- */

int main(void)
{
	struct rtc_time now;
	struct rtc_time alarm_time;
	int ret;

	if (!device_is_ready(rtc_dev)) {
		LOG_ERR("RTC device not ready");
		return -ENODEV;
	}

	LOG_INF("<DEVNAME> RTC sample application");

	/* Set the current time: 2025-06-15 12:30:00 (Sunday). */
	now = (struct rtc_time){
		.tm_sec  = 0,
		.tm_min  = 30,
		.tm_hour = 12,
		.tm_mday = 15,
		.tm_mon  = 5,    /* June (0-based) */
		.tm_year = 125,  /* 2025 - 1900 */
		.tm_wday = 0,    /* Sunday */
	};

	ret = rtc_set_time(rtc_dev, &now);
	if (ret) {
		LOG_ERR("Failed to set time: %d", ret);
		return ret;
	}

	/* Set an alarm for 12:31:00. */
	alarm_time = (struct rtc_time){
		.tm_sec  = 0,
		.tm_min  = 31,
		.tm_hour = 12,
	};

	ret = rtc_alarm_set_time(rtc_dev, 0,
				 RTC_ALARM_TIME_MASK_SECOND |
				 RTC_ALARM_TIME_MASK_MINUTE |
				 RTC_ALARM_TIME_MASK_HOUR,
				 &alarm_time);
	if (ret) {
		LOG_ERR("Failed to set alarm: %d", ret);
		return ret;
	}

	/* Register the alarm callback. */
	ret = rtc_alarm_set_callback(rtc_dev, 0, alarm_handler, NULL);
	if (ret && ret != -ENOTSUP) {
		LOG_ERR("Failed to set alarm callback: %d", ret);
		return ret;
	}

	/* Poll and print time every second. */
	while (1) {
		ret = rtc_get_time(rtc_dev, &now);
		if (ret) {
			LOG_ERR("Failed to get time: %d", ret);
		} else {
			LOG_INF("Time: %02d:%02d:%02d  Date: %04d-%02d-%02d",
				now.tm_hour, now.tm_min, now.tm_sec,
				now.tm_year + 1900,
				now.tm_mon + 1,
				now.tm_mday);
		}

		/* Check alarm status (polled mode). */
		ret = rtc_alarm_is_pending(rtc_dev, 0);
		if (ret > 0) {
			LOG_INF("Alarm pending (polled)!");
		}

		k_sleep(K_SECONDS(1));
	}

	return 0;
}
```

---

## 8. Test Skeleton

### `tests/drivers/rtc/<devname>/testcase.yaml`

```yaml
tests:
  drivers.rtc.<devname>:
    tags:
      - drivers
      - rtc
    depends_on: i2c
    harness: ztest
```

### `tests/drivers/rtc/<devname>/prj.conf`

```
CONFIG_ZTEST=y
CONFIG_I2C=y
CONFIG_RTC=y
CONFIG_RTC_<DEVNAME>=y
CONFIG_LOG=y
```

### `tests/drivers/rtc/<devname>/src/main.c`

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/ztest.h>

static const struct device *get_dev(void)
{
	const struct device *dev = DEVICE_DT_GET_ANY(adi_<devname>);

	zassert_not_null(dev, "Device not found");
	zassert_true(device_is_ready(dev), "Device not ready");

	return dev;
}

ZTEST(rtc_<devname>, test_set_get_time)
{
	const struct device *dev = get_dev();
	struct rtc_time set_time = {
		.tm_sec  = 30,
		.tm_min  = 45,
		.tm_hour = 14,
		.tm_mday = 20,
		.tm_mon  = 2,   /* March */
		.tm_year = 125, /* 2025 */
		.tm_wday = 4,   /* Thursday */
	};
	struct rtc_time get_time;
	int ret;

	ret = rtc_set_time(dev, &set_time);
	zassert_ok(ret, "set_time failed: %d", ret);

	/* Brief delay for registers to update. */
	k_msleep(10);

	ret = rtc_get_time(dev, &get_time);
	zassert_ok(ret, "get_time failed: %d", ret);

	zassert_equal(get_time.tm_hour, 14, "Hour mismatch: %d", get_time.tm_hour);
	zassert_equal(get_time.tm_min, 45, "Minute mismatch: %d", get_time.tm_min);
	/* Seconds may have advanced by 1 depending on timing. */
	zassert_true(get_time.tm_sec >= 30 && get_time.tm_sec <= 31,
		     "Second out of range: %d", get_time.tm_sec);
	zassert_equal(get_time.tm_mday, 20, "Day mismatch: %d", get_time.tm_mday);
	zassert_equal(get_time.tm_mon, 2, "Month mismatch: %d", get_time.tm_mon);
	zassert_equal(get_time.tm_year, 125, "Year mismatch: %d", get_time.tm_year);
}

ZTEST(rtc_<devname>, test_alarm_set_get)
{
	const struct device *dev = get_dev();
	struct rtc_time alarm_set = {
		.tm_sec  = 0,
		.tm_min  = 15,
		.tm_hour = 8,
	};
	struct rtc_time alarm_get;
	uint16_t mask_set = RTC_ALARM_TIME_MASK_SECOND |
			    RTC_ALARM_TIME_MASK_MINUTE |
			    RTC_ALARM_TIME_MASK_HOUR;
	uint16_t mask_get;
	int ret;

	ret = rtc_alarm_set_time(dev, 0, mask_set, &alarm_set);
	zassert_ok(ret, "alarm_set_time failed: %d", ret);

	ret = rtc_alarm_get_time(dev, 0, &mask_get, &alarm_get);
	zassert_ok(ret, "alarm_get_time failed: %d", ret);

	zassert_true(mask_get != 0, "Alarm mask should be non-zero");
	zassert_equal(alarm_get.tm_sec, 0, "Alarm sec mismatch");
	zassert_equal(alarm_get.tm_min, 15, "Alarm min mismatch");
	zassert_equal(alarm_get.tm_hour, 8, "Alarm hour mismatch");
}

ZTEST(rtc_<devname>, test_alarm_supported_fields)
{
	const struct device *dev = get_dev();
	uint16_t mask;
	int ret;

	ret = rtc_alarm_get_supported_fields(dev, 0, &mask);
	zassert_ok(ret, "alarm_get_supported_fields failed: %d", ret);

	/* At minimum, seconds, minutes, and hours should be supported. */
	zassert_true(mask & RTC_ALARM_TIME_MASK_SECOND,
		     "Second not supported");
	zassert_true(mask & RTC_ALARM_TIME_MASK_MINUTE,
		     "Minute not supported");
	zassert_true(mask & RTC_ALARM_TIME_MASK_HOUR,
		     "Hour not supported");
}

ZTEST(rtc_<devname>, test_alarm_invalid_id)
{
	const struct device *dev = get_dev();
	uint16_t mask;
	int ret;

	ret = rtc_alarm_get_supported_fields(dev, 99, &mask);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for invalid alarm id, got %d", ret);
}

ZTEST(rtc_<devname>, test_alarm_is_pending)
{
	const struct device *dev = get_dev();
	int ret;

	/* Set time to a known value. */
	struct rtc_time now = {
		.tm_sec  = 58,
		.tm_min  = 0,
		.tm_hour = 0,
		.tm_mday = 1,
		.tm_mon  = 0,
		.tm_year = 125,
		.tm_wday = 0,
	};

	ret = rtc_set_time(dev, &now);
	zassert_ok(ret, "set_time failed: %d", ret);

	/* Set alarm for 00:01:00. */
	struct rtc_time alarm = {
		.tm_sec  = 0,
		.tm_min  = 1,
		.tm_hour = 0,
	};

	ret = rtc_alarm_set_time(dev, 0,
				 RTC_ALARM_TIME_MASK_SECOND |
				 RTC_ALARM_TIME_MASK_MINUTE |
				 RTC_ALARM_TIME_MASK_HOUR,
				 &alarm);
	zassert_ok(ret, "alarm_set_time failed: %d", ret);

	/* Initially should not be pending. */
	ret = rtc_alarm_is_pending(dev, 0);
	zassert_true(ret >= 0, "alarm_is_pending error: %d", ret);

	/* Wait for alarm (up to 5 seconds). */
	for (int i = 0; i < 5; i++) {
		k_sleep(K_SECONDS(1));
		ret = rtc_alarm_is_pending(dev, 0);
		if (ret > 0) {
			break;
		}
	}

	zassert_equal(ret, 1, "Alarm did not fire within timeout");
}

ZTEST_SUITE(rtc_<devname>, NULL, NULL, NULL, NULL, NULL);
```

---

## 9. no-OS vs. Zephyr Comparison

This section maps the key differences between the no-OS RTC driver
approach and the Zephyr RTC subsystem approach.

| Aspect | no-OS | Zephyr |
|--------|-------|--------|
| **Subsystem** | None (standalone driver) | `drivers/rtc/` subsystem with `rtc_driver_api` |
| **Device struct** | `<devname>_desc` (heap-allocated) | Split into `<devname>_config` (const/flash) + `<devname>_data` (mutable/RAM), statically allocated |
| **Init pattern** | `<devname>_init(**desc, *init_param)` allocates with `no_os_calloc()` | `<devname>_init(dev)` -- no dynamic allocation; config from DT macros |
| **Cleanup** | `<devname>_remove(desc)` frees memory | No explicit remove; device lifecycle managed by kernel |
| **I2C access** | `no_os_i2c_write()` / `no_os_i2c_read()` with stop-bit parameter | `i2c_reg_read_byte_dt()` / `i2c_reg_write_byte_dt()` / `i2c_burst_read_dt()` / `i2c_reg_update_byte_dt()` |
| **I2C address** | `#define` constant, passed in `init_param` | From devicetree via `I2C_DT_SPEC_INST_GET()` |
| **BCD helpers** | `no_os_bin2bcd()` / `no_os_bcd2bin()` from `no_os_util.h` | Local `bin2bcd()` / `bcd2bin()` static inline functions |
| **Time struct** | `<devname>_time_stamp` -- month 1-based, year absolute | `struct rtc_time` -- month 0-based, year since 1900 |
| **Alarm API** | Direct register writes via `reg_write()` | `rtc_alarm_set_time()` with field mask bitmask |
| **Interrupts** | Not abstracted; device-specific | `rtc_alarm_set_callback()` / `rtc_update_set_callback()` |
| **Thread safety** | Not provided; caller's responsibility | Mutex in data struct protects all register access |
| **Bit macros** | `NO_OS_BIT()` / `NO_OS_GENMASK()` | `BIT()` / `GENMASK()` |
| **Error codes** | Negative errno or `no_os_error.h` constants | Negative errno (`-EIO`, `-EINVAL`, `-ENOTSUP`) |
| **Logging** | `pr_info()` / `printf` | `LOG_MODULE_REGISTER()` / `LOG_INF()` / `LOG_ERR()` |
| **License** | BSD-3-Clause (ADI header block) | Apache-2.0 (SPDX one-liner) |
| **Registration** | None (caller manages descriptor pointer) | `DEVICE_DT_INST_DEFINE()` with `DT_INST_FOREACH_STATUS_OKAY()` |
| **Configuration** | C structs (`init_param`) filled at runtime | Devicetree properties resolved at compile time |
| **File naming** | `<devname>.c` / `<devname>.h` | `rtc_<devname>.c` (single file, no header) |

---

## 10. Key Conventions

1. **File location**: `drivers/rtc/rtc_<devname>.c` -- RTC drivers are
   placed directly in `drivers/rtc/` (not in a vendor subdirectory).
   The file is prefixed with `rtc_` to follow the subsystem convention.

2. **Compatible string**: `"adi,<devname>"` -- lowercase, matches the
   devicetree binding filename `adi,<devname>.yaml`.

3. **License**: SPDX `Apache-2.0` header in every file.  Zephyr uses
   Apache-2.0 (not BSD-3-Clause as in no-OS).

4. **Time encoding**: `struct rtc_time` follows POSIX `struct tm`
   conventions.  Month is 0-based, year is offset from 1900.  BCD
   conversion to/from hardware registers is the driver's responsibility.

5. **Alarm API**: three-function pattern -- `alarm_set_time()` programs
   the alarm, `alarm_get_time()` reads it back, `alarm_is_pending()`
   checks and clears the flag.  The field mask controls which time
   fields participate in the match.

6. **Callbacks**: `alarm_set_callback()` and `update_set_callback()`
   require a GPIO interrupt pin.  The callback is invoked from the
   system workqueue (not ISR context), so it may perform I2C operations.

7. **Devicetree-driven config**: all instance-specific parameters
   (I2C bus, address, GPIO pins, trickle charger, battery) come from
   the devicetree, not from C structs.  Use `I2C_DT_SPEC_INST_GET()`,
   `GPIO_DT_SPEC_INST_GET()`, `DT_INST_PROP()`.

8. **Instantiation**: use `DEVICE_DT_INST_DEFINE()` (not
   `SENSOR_DEVICE_DT_INST_DEFINE()`).  The RTC subsystem does not have
   its own device define wrapper.

9. **API struct**: use `DEVICE_API(rtc, <devname>_api)` to define the
   driver API table with compile-time type safety.

10. **Logging**: `LOG_MODULE_REGISTER(rtc_<devname>, CONFIG_RTC_LOG_LEVEL)`
    in the `.c` file.

11. **Bus readiness**: always check `i2c_is_ready_dt()` and
    `gpio_is_ready_dt()` in `init()`.

12. **Register I/O**: use `i2c_reg_read_byte_dt()`,
    `i2c_reg_write_byte_dt()`, `i2c_burst_read_dt()`,
    `i2c_burst_write_dt()`, and `i2c_reg_update_byte_dt()` -- the `_dt`
    variants that accept `struct i2c_dt_spec`.

13. **Thread safety**: use a `struct k_mutex` in the data struct to
    protect all register access.  RTC operations may be called from
    multiple threads (e.g., alarm thread, application thread).

14. **No dynamic allocation**: Zephyr drivers do not use `malloc()`.
    All memory is statically allocated via the instantiation macros.

15. **Kconfig symbol**: use `RTC_<DEVNAME>` as the Kconfig symbol name,
    with `depends on DT_HAS_ADI_<DEVNAME>_ENABLED` so the driver is
    only offered when a matching DT node exists.

---

## 11. Commit Message Format

```
drivers: rtc: <devname>: add <devname> driver

Add Zephyr RTC subsystem driver for the ADI <DEVNAME> real-time clock.
Supports I2C communication, BCD time encoding, calendar set/get,
alarm configuration with callback support, trickle charger setup,
and backup battery switchover via devicetree properties.

Signed-off-by: Your Name <your.name@analog.com>
```

For adding alarm callback support as a follow-up:

```
drivers: rtc: <devname>: add alarm callback support

Add GPIO interrupt handling for alarm callbacks on the <DEVNAME> RTC.
The interrupt is deferred to the system workqueue for safe I2C access.
Requires the int-gpios property to be set in the devicetree node.

Signed-off-by: Your Name <your.name@analog.com>
```
