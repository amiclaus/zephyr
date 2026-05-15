# EEPROM Driver Template -- Zephyr RTOS

Reference driver: `drivers/eeprom/eeprom_at2x.c` in the Zephyr tree.

This template covers every file needed to add a new Zephyr EEPROM
subsystem driver for an Analog Devices I2C EEPROM.  Replace `<devname>`
with the part number in lowercase (e.g., `at24c256`), `<DEVNAME>` with
its uppercase form (e.g., `AT24C256`), and adjust memory/page sizes to
match the specific part's datasheet throughout.

---

## 1. Purpose & Zephyr Subsystem Mapping

Zephyr EEPROM drivers live under the **EEPROM subsystem**
(`drivers/eeprom/`).  The subsystem provides a uniform API defined in
`include/zephyr/drivers/eeprom.h` for applications to read, write, and
query EEPROM storage without knowing hardware-specific details.

### EEPROM API Functions

| Zephyr EEPROM API Function | What it does                                   |
|----------------------------|-------------------------------------------------|
| `eeprom_read()`            | Read bytes starting at a given offset            |
| `eeprom_write()`           | Write bytes starting at a given offset           |
| `eeprom_get_size()`        | Return total EEPROM capacity in bytes            |

The driver implements the `eeprom_driver_api` struct (via the
`DEVICE_API(eeprom, ...)` macro) which contains pointers to the
driver's `read`, `write`, and `size` implementations.

### Key Characteristics

- **Page-based writes** -- I2C EEPROMs have a page buffer; writes that
  cross a page boundary wrap within the page.  The driver must split
  multi-page writes into page-aligned chunks.
- **Write cycle time** -- after each page write, the EEPROM enters an
  internal write cycle (typically 5 ms) during which it does not ACK
  I2C transactions.  The driver must either poll for ACK or insert a
  fixed delay.
- **No channels or triggers** -- unlike sensor or ADC drivers, EEPROM
  drivers have no concept of channels, triggers, or interrupts.
- **All configuration from devicetree** -- memory size, page size, I2C
  address, and write cycle timing come from devicetree properties.

---

## 2. File Checklist

```
zephyr/
    drivers/eeprom/
        eeprom_<devname>.c        # Driver implementation
        Kconfig.<devname>         # Kconfig fragment
        CMakeLists.txt            # (append to existing)
        Kconfig                   # (append to existing)

    dts/bindings/eeprom/
        adi,<devname>.yaml        # Devicetree binding

    tests/drivers/eeprom/<devname>/
        testcase.yaml             # Test metadata
        prj.conf                  # Test project config
        boards/native_sim.overlay # DT overlay for test
        src/main.c                # Test source
```

---

## 3. Devicetree Binding (`dts/bindings/eeprom/adi,<devname>.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Analog Devices <DEVNAME> I2C EEPROM.
  Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/<DEVNAME>.pdf

  Example devicetree node:

    &i2c0 {
        status = "okay";

        eeprom0: <devname>@50 {
            compatible = "adi,<devname>";
            reg = <0x50>;
            size = <32768>;
            pagesize = <64>;
            timeout = <5>;
        };
    };

compatible: "adi,<devname>"

include: [i2c-device.yaml]

properties:
  size:
    type: int
    required: true
    description: |
      Total EEPROM storage capacity in bytes.
      Example: 32768 for a 256-Kbit EEPROM.

  pagesize:
    type: int
    required: true
    description: |
      Page size in bytes.  Writes that cross a page boundary wrap
      within the page, so the driver must split writes at page
      boundaries.  Typical values: 32, 64, 128, 256.

  address-width:
    type: int
    default: 16
    description: |
      Width of the internal memory address in bits.
      Use 8 for EEPROMs with up to 256 bytes (e.g., 24C02).
      Use 16 for EEPROMs with more than 256 bytes (e.g., 24C256).
    enum:
      - 8
      - 16

  timeout:
    type: int
    default: 5
    description: |
      Write cycle time in milliseconds.  After each page write, the
      driver waits this long for the internal write cycle to complete.
      Typical value: 5 ms (per datasheet).

  wp-gpios:
    type: phandle-array
    description: |
      Write-protect pin GPIO connection.  When provided, the driver
      can de-assert WP before writes and re-assert it afterwards.
      Example: wp-gpios = <&gpio0 3 GPIO_ACTIVE_LOW>;
```

### Example devicetree node

```dts
&i2c0 {
    eeprom0: <devname>@50 {
        compatible = "adi,<devname>";
        reg = <0x50>;
        size = <32768>;       /* 256 Kbit = 32768 bytes */
        pagesize = <64>;
        address-width = <16>;
        timeout = <5>;
        wp-gpios = <&gpio0 3 GPIO_ACTIVE_LOW>;
    };
};
```

---

## 4. Kconfig (`drivers/eeprom/Kconfig.<devname>`)

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config EEPROM_<DEVNAME>
	bool "<DEVNAME> I2C EEPROM driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	select I2C
	help
	  Enable driver for the Analog Devices <DEVNAME> I2C EEPROM.
	  Supports page-based read/write and write-protect GPIO control.
```

Then add to the parent `drivers/eeprom/Kconfig`:

```kconfig
source "drivers/eeprom/Kconfig.<devname>"
```

---

## 5. CMakeLists.txt (Build System Integration)

Append to the existing `drivers/eeprom/CMakeLists.txt`:

```cmake
zephyr_library_sources_ifdef(CONFIG_EEPROM_<DEVNAME> eeprom_<devname>.c)
```

---

## 6. Driver Header (Optional)

EEPROM drivers are typically self-contained in a single `.c` file with
no separate header, since the EEPROM subsystem API is the public
interface and there are no register maps to share.  All defines
(page size, address width, etc.) come from devicetree.

If a separate header is needed for testing or emulation purposes:

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_EEPROM_<DEVNAME>_H_
#define ZEPHYR_DRIVERS_EEPROM_<DEVNAME>_H_

/* Maximum I2C transfer size (address bytes + page data). */
#define <DEVNAME>_MAX_XFER_SIZE		(2 + 256)

#endif /* ZEPHYR_DRIVERS_EEPROM_<DEVNAME>_H_ */
```

---

## 7. Driver Source (`drivers/eeprom/eeprom_<devname>.c`)

This is the core of the driver.  It follows the Zephyr EEPROM subsystem
contract: implement `read`, `write`, and `size` callbacks.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <string.h>

LOG_MODULE_REGISTER(<devname>, CONFIG_EEPROM_LOG_LEVEL);

/* ---------------- Config & Data Structs --------------------------- */

/**
 * Compile-time configuration from devicetree.
 * This struct is const and stored in ROM.
 */
struct <devname>_config {
	/** I2C bus and device address from devicetree. */
	struct i2c_dt_spec i2c;

	/** Total EEPROM size in bytes. */
	uint32_t size;

	/** Page size in bytes. */
	uint16_t pagesize;

	/** Internal address width: 8 or 16 bits. */
	uint8_t addr_width;

	/** Write cycle time in milliseconds. */
	uint8_t timeout;

	/** Write-protect GPIO (optional). */
	struct gpio_dt_spec wp_gpio;

	/** Whether a WP GPIO is configured. */
	bool has_wp;
};

/**
 * Mutable runtime data.
 * This struct is allocated statically per instance.
 */
struct <devname>_data {
	/** Mutex to serialise concurrent access. */
	struct k_mutex lock;
};

/* ---------------- Write-Protect Helpers --------------------------- */

/**
 * @brief De-assert the write-protect pin (allow writes).
 */
static int <devname>_wp_enable(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;

	if (!cfg->has_wp) {
		return 0;
	}

	return gpio_pin_set_dt(&cfg->wp_gpio, 0);
}

/**
 * @brief Assert the write-protect pin (block writes).
 */
static int <devname>_wp_disable(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;

	if (!cfg->has_wp) {
		return 0;
	}

	return gpio_pin_set_dt(&cfg->wp_gpio, 1);
}

/* ---------------- I2C Helpers ------------------------------------- */

/**
 * @brief Prepare the memory address bytes.
 *
 * @param cfg    - Device configuration.
 * @param offset - Memory offset within the EEPROM.
 * @param buf    - Buffer to write address bytes into (at least 2 bytes).
 *
 * @return Number of address bytes (1 for 8-bit, 2 for 16-bit).
 */
static int <devname>_addr_prepare(const struct <devname>_config *cfg,
				  uint32_t offset, uint8_t *buf)
{
	if (cfg->addr_width == 16) {
		sys_put_be16((uint16_t)offset, buf);
		return 2;
	}

	buf[0] = (uint8_t)offset;
	return 1;
}

/* ---------------- EEPROM API: read -------------------------------- */

/**
 * @brief Read data from the EEPROM.
 *
 * Sends the memory address (1 or 2 bytes, big-endian for 16-bit) as a
 * write, then reads the requested number of bytes using a repeated
 * start condition.  Sequential reads wrap automatically at the end
 * of the address space.
 *
 * @param dev    - Device instance.
 * @param offset - Starting memory address.
 * @param buf    - Buffer to store read data.
 * @param len    - Number of bytes to read.
 *
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_read(const struct device *dev, off_t offset,
			  void *buf, size_t len)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint8_t addr_buf[2];
	int addr_len;
	int ret;

	if (!len) {
		return 0;
	}

	if ((offset + len) > cfg->size) {
		LOG_ERR("Read out of bounds: offset=%ld len=%zu size=%u",
			(long)offset, len, cfg->size);
		return -EINVAL;
	}

	addr_len = <devname>_addr_prepare(cfg, offset, addr_buf);

	k_mutex_lock(&data->lock, K_FOREVER);

	/*
	 * Use i2c_write_read_dt() which sends the address bytes with a
	 * repeated start, then reads the data.  This is the standard
	 * pattern for I2C EEPROM reads.
	 */
	ret = i2c_write_read_dt(&cfg->i2c, addr_buf, addr_len, buf, len);
	if (ret) {
		LOG_ERR("I2C read failed: %d", ret);
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

/* ---------------- EEPROM API: write ------------------------------- */

/**
 * @brief Write a single page-aligned chunk to the EEPROM.
 *
 * The caller must ensure that the write does not cross a page boundary.
 * After the write, the function waits for the internal write cycle to
 * complete.
 *
 * @param dev    - Device instance.
 * @param offset - Starting memory address (must be page-aligned or
 *                 within a single page).
 * @param buf    - Data to write.
 * @param len    - Number of bytes to write (must not cross page boundary).
 *
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_write_page(const struct device *dev, off_t offset,
				const void *buf, size_t len)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t wr_buf[2 + 256]; /* max: 2 addr bytes + largest page */
	int addr_len;
	int ret;

	addr_len = <devname>_addr_prepare(cfg, offset, wr_buf);
	memcpy(&wr_buf[addr_len], buf, len);

	ret = i2c_write_dt(&cfg->i2c, wr_buf, addr_len + len);
	if (ret) {
		LOG_ERR("I2C page write failed at offset %ld: %d",
			(long)offset, ret);
		return ret;
	}

	/* Wait for the internal write cycle to complete. */
	k_msleep(cfg->timeout);

	return 0;
}

/**
 * @brief Write data to the EEPROM (handles page boundaries).
 *
 * Splits the write into page-aligned chunks.  Each chunk is written
 * with <devname>_write_page(), which includes the write cycle delay.
 *
 * If a write-protect GPIO is configured, WP is de-asserted before the
 * write sequence and re-asserted afterwards.
 *
 * @param dev    - Device instance.
 * @param offset - Starting memory address.
 * @param buf    - Data buffer to write.
 * @param len    - Number of bytes to write.
 *
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_write(const struct device *dev, off_t offset,
			   const void *buf, size_t len)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	const uint8_t *src = (const uint8_t *)buf;
	size_t written = 0;
	size_t chunk;
	size_t page_offset;
	int ret;

	if (!len) {
		return 0;
	}

	if ((offset + len) > cfg->size) {
		LOG_ERR("Write out of bounds: offset=%ld len=%zu size=%u",
			(long)offset, len, cfg->size);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = <devname>_wp_enable(dev);
	if (ret) {
		LOG_ERR("Failed to de-assert WP: %d", ret);
		goto unlock;
	}

	while (written < len) {
		/*
		 * Calculate how many bytes fit in the current page.
		 * The first write may be partial if the starting
		 * address is not page-aligned.
		 */
		page_offset = (offset + written) % cfg->pagesize;
		chunk = cfg->pagesize - page_offset;
		if (chunk > (len - written)) {
			chunk = len - written;
		}

		ret = <devname>_write_page(dev, offset + written,
					   &src[written], chunk);
		if (ret) {
			goto wp_disable;
		}

		written += chunk;
	}

wp_disable:
	<devname>_wp_disable(dev);
unlock:
	k_mutex_unlock(&data->lock);

	return ret;
}

/* ---------------- EEPROM API: size -------------------------------- */

/**
 * @brief Return the total EEPROM capacity in bytes.
 *
 * @param dev - Device instance.
 *
 * @return EEPROM size in bytes.
 */
static size_t <devname>_size(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;

	return (size_t)cfg->size;
}

/* ---------------- API Struct -------------------------------------- */

/*
 * Use the DEVICE_API macro.  This creates a static const struct that
 * the Zephyr EEPROM subsystem uses to dispatch calls.
 */
static DEVICE_API(eeprom, <devname>_api) = {
	.read = <devname>_read,
	.write = <devname>_write,
	.size = <devname>_size,
};

/* ---------------- Initialization ---------------------------------- */

/**
 * @brief Device initialization function.
 *
 * Called automatically at boot for each DT instance.  Verifies bus
 * readiness, initialises the mutex, and optionally configures the
 * write-protect GPIO.
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

	k_mutex_init(&data->lock);

	/* Configure write-protect GPIO if present. */
	if (cfg->has_wp) {
		if (!gpio_is_ready_dt(&cfg->wp_gpio)) {
			LOG_ERR("WP GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->wp_gpio,
					    GPIO_OUTPUT_ACTIVE);
		if (ret) {
			LOG_ERR("Failed to configure WP GPIO: %d", ret);
			return ret;
		}
	}

	/*
	 * Optional: perform a dummy read to verify the device is present
	 * on the bus.  A 1-byte read at offset 0 is sufficient.
	 */
	uint8_t dummy;

	ret = <devname>_read(dev, 0, &dummy, 1);
	if (ret) {
		LOG_ERR("Device not responding on I2C bus: %d", ret);
		return ret;
	}

	LOG_INF("<DEVNAME> EEPROM initialized: %u bytes, %u-byte pages",
		cfg->size, cfg->pagesize);

	return 0;
}

/* ---------------- Instance Macros --------------------------------- */

/*
 * Write-protect GPIO configuration, present only when the wp-gpios
 * property exists in the devicetree node.
 */
#define <DEVNAME>_WP_GPIO_INIT(inst)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, wp_gpios),		\
		(.wp_gpio = GPIO_DT_SPEC_INST_GET(inst, wp_gpios),	\
		 .has_wp = true,),					\
		(.has_wp = false,))

/*
 * Per-instance config/data definitions and device registration.
 *
 * This macro is expanded once per devicetree instance that has
 * compatible = "adi,<devname>" with status = "okay".
 */
#define <DEVNAME>_INST(inst)						\
	static struct <devname>_data <devname>_data_##inst;		\
									\
	static const struct <devname>_config <devname>_config_##inst = {	\
		.i2c = I2C_DT_SPEC_INST_GET(inst),			\
		.size = DT_INST_PROP(inst, size),			\
		.pagesize = DT_INST_PROP(inst, pagesize),		\
		.addr_width = DT_INST_PROP(inst, address_width),	\
		.timeout = DT_INST_PROP(inst, timeout),			\
		<DEVNAME>_WP_GPIO_INIT(inst)				\
	};								\
									\
	DEVICE_DT_INST_DEFINE(						\
		inst,							\
		<devname>_init,						\
		NULL,							\
		&<devname>_data_##inst,					\
		&<devname>_config_##inst,				\
		POST_KERNEL,						\
		CONFIG_EEPROM_INIT_PRIORITY,				\
		&<devname>_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_INST)
```

### Key elements explained

| Element | Purpose |
|---------|---------|
| `DT_DRV_COMPAT` | Must match the compatible string with commas replaced by underscores (`adi,<devname>` becomes `adi_<devname>`) |
| `struct <devname>_config` | Compile-time config from DT (stored in ROM): I2C spec, size, pagesize, address width, timeout, WP GPIO |
| `struct <devname>_data` | Mutable runtime state: mutex for thread safety |
| `DEVICE_API(eeprom, ...)` | Typed API struct for the EEPROM subsystem |
| `I2C_DT_SPEC_INST_GET()` | Pulls I2C bus reference and device address from DT |
| `GPIO_DT_SPEC_INST_GET()` | Pulls GPIO port/pin/flags from DT for write-protect |
| `DEVICE_DT_INST_DEFINE()` | Registers the device with Zephyr's device model |
| `DT_INST_FOREACH_STATUS_OKAY()` | Instantiates one driver per DT node with `status = "okay"` |
| `LOG_MODULE_REGISTER()` | Creates a logging module; level controlled by `CONFIG_EEPROM_LOG_LEVEL` |
| `i2c_write_read_dt()` | Combined write+read with repeated start, standard I2C EEPROM read pattern |
| `i2c_write_dt()` | Single write transaction for page writes (address + data) |
| `k_mutex` | Protects concurrent read/write access from multiple threads |

### Comparison with no-OS EEPROM driver pattern

| Aspect | no-OS | Zephyr |
|--------|-------|--------|
| API interface | `no_os_eeprom_platform_ops` (`init`, `read`, `write`, `remove`) | `eeprom_driver_api` (`read`, `write`, `size`) |
| Memory management | Dynamic: `no_os_calloc()` / `no_os_free()` | Static: all memory declared at compile time via macros |
| Configuration | C structs (`init_param` with `extra` field) | Devicetree properties parsed at build time |
| I2C access | `no_os_i2c_write()` / `no_os_i2c_read()` with stop-bit param | `i2c_write_read_dt()` / `i2c_write_dt()` with `i2c_dt_spec` |
| Address handling | `ADDR_HIGH()` / `ADDR_LOW()` macros | `sys_put_be16()` from `<zephyr/sys/byteorder.h>` |
| Page write delay | `no_os_mdelay()` | `k_msleep()` |
| Thread safety | Not built in (caller's responsibility) | `k_mutex` in driver |
| Cleanup | Explicit `_remove()` function | No remove -- lifecycle managed by device model |
| License | BSD-3-Clause | Apache-2.0 |

---

## 8. Trigger / Interrupt Support

EEPROM devices do **not** have interrupt or trigger capabilities.  There
is no data-ready signal, no threshold alert, and no asynchronous
notification mechanism.  This section is intentionally left minimal.

If a specific EEPROM part includes an interrupt output (e.g., for
write-completion signalling), it can be implemented as a simple GPIO
callback that signals a semaphore, replacing the fixed `k_msleep()`
delay in `<devname>_write_page()`:

```c
/* Optional: ACK-polling or interrupt-based write completion.
 *
 * Some I2C EEPROM drivers use ACK polling instead of a fixed delay.
 * The device NACKs during its internal write cycle and ACKs when
 * done.  This approach is faster for writes shorter than the worst
 * case write cycle time.
 */
static int <devname>_wait_write_complete(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	uint8_t dummy;
	int ret;
	int attempts = 0;

	/*
	 * Poll with 1-byte reads until the device ACKs.
	 * Timeout after the maximum write cycle time.
	 */
	while (attempts < cfg->timeout) {
		ret = i2c_write_read_dt(&cfg->i2c, NULL, 0, &dummy, 0);
		if (ret == 0) {
			return 0;
		}

		k_msleep(1);
		attempts++;
	}

	LOG_ERR("Write cycle timeout after %d ms", cfg->timeout);
	return -ETIMEDOUT;
}
```

---

## 9. Test Skeleton

### 9.1 Test Metadata (`tests/drivers/eeprom/<devname>/testcase.yaml`)

```yaml
tests:
  drivers.eeprom.<devname>:
    tags:
      - drivers
      - eeprom
    depends_on: i2c
    platform_allow: native_sim
    integration_platforms:
      - native_sim
```

### 9.2 Test Project Config (`tests/drivers/eeprom/<devname>/prj.conf`)

```
CONFIG_ZTEST=y
CONFIG_I2C=y
CONFIG_EEPROM=y
CONFIG_EEPROM_<DEVNAME>=y
CONFIG_LOG=y
```

### 9.3 DT Overlay (`tests/drivers/eeprom/<devname>/boards/native_sim.overlay`)

```dts
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

&i2c0 {
    status = "okay";

    <devname>_test: <devname>@50 {
        compatible = "adi,<devname>";
        reg = <0x50>;
        size = <32768>;
        pagesize = <64>;
        address-width = <16>;
        timeout = <5>;
    };
};
```

### 9.4 Test Source (`tests/drivers/eeprom/<devname>/src/main.c`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/device.h>

#include <string.h>

#define EEPROM_NODE DT_NODELABEL(<devname>_test)
static const struct device *eeprom_dev = DEVICE_DT_GET(EEPROM_NODE);

#define TEST_OFFSET   0x0000
#define TEST_SIZE     64

ZTEST(eeprom_<devname>, test_device_ready)
{
	zassert_true(device_is_ready(eeprom_dev),
		     "EEPROM device not ready");
}

ZTEST(eeprom_<devname>, test_size)
{
	size_t size;

	size = eeprom_get_size(eeprom_dev);
	zassert_true(size > 0, "EEPROM size should be > 0, got %zu", size);
}

ZTEST(eeprom_<devname>, test_write_read)
{
	uint8_t wr_buf[TEST_SIZE];
	uint8_t rd_buf[TEST_SIZE];
	int ret;

	/* Fill write buffer with test pattern. */
	for (int i = 0; i < TEST_SIZE; i++) {
		wr_buf[i] = (uint8_t)(i & 0xFF);
	}

	ret = eeprom_write(eeprom_dev, TEST_OFFSET, wr_buf, TEST_SIZE);
	zassert_ok(ret, "eeprom_write failed: %d", ret);

	memset(rd_buf, 0, sizeof(rd_buf));
	ret = eeprom_read(eeprom_dev, TEST_OFFSET, rd_buf, TEST_SIZE);
	zassert_ok(ret, "eeprom_read failed: %d", ret);

	zassert_mem_equal(wr_buf, rd_buf, TEST_SIZE,
			  "Read data does not match written data");
}

ZTEST(eeprom_<devname>, test_write_read_page_crossing)
{
	uint8_t wr_buf[TEST_SIZE];
	uint8_t rd_buf[TEST_SIZE];
	off_t offset;
	int ret;

	/*
	 * Start at a page-unaligned offset to exercise the page
	 * boundary splitting logic.
	 */
	offset = 48;  /* Intentionally not page-aligned */

	for (int i = 0; i < TEST_SIZE; i++) {
		wr_buf[i] = (uint8_t)((i + 0xA5) & 0xFF);
	}

	ret = eeprom_write(eeprom_dev, offset, wr_buf, TEST_SIZE);
	zassert_ok(ret, "eeprom_write (page crossing) failed: %d", ret);

	memset(rd_buf, 0, sizeof(rd_buf));
	ret = eeprom_read(eeprom_dev, offset, rd_buf, TEST_SIZE);
	zassert_ok(ret, "eeprom_read (page crossing) failed: %d", ret);

	zassert_mem_equal(wr_buf, rd_buf, TEST_SIZE,
			  "Page-crossing read data mismatch");
}

ZTEST(eeprom_<devname>, test_read_out_of_bounds)
{
	uint8_t buf[16];
	size_t size;
	int ret;

	size = eeprom_get_size(eeprom_dev);

	ret = eeprom_read(eeprom_dev, size, buf, sizeof(buf));
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for out-of-bounds read, got %d",
		      ret);
}

ZTEST(eeprom_<devname>, test_write_out_of_bounds)
{
	uint8_t buf[16] = { 0 };
	size_t size;
	int ret;

	size = eeprom_get_size(eeprom_dev);

	ret = eeprom_write(eeprom_dev, size, buf, sizeof(buf));
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for out-of-bounds write, got %d",
		      ret);
}

ZTEST(eeprom_<devname>, test_zero_length_operations)
{
	uint8_t buf[1];
	int ret;

	ret = eeprom_read(eeprom_dev, 0, buf, 0);
	zassert_ok(ret, "Zero-length read should succeed: %d", ret);

	ret = eeprom_write(eeprom_dev, 0, buf, 0);
	zassert_ok(ret, "Zero-length write should succeed: %d", ret);
}

ZTEST_SUITE(eeprom_<devname>, NULL, NULL, NULL, NULL, NULL);
```

---

## 10. Key Conventions

1. **File location** -- `drivers/eeprom/eeprom_<devname>.c`.  EEPROM
   drivers are flat files directly in `drivers/eeprom/`, not in vendor
   subdirectories (unlike sensors which go under `drivers/sensor/adi/`).

2. **Compatible string** -- `"adi,<devname>"` in lowercase, matching
   the binding filename `adi,<devname>.yaml`.

3. **License** -- SPDX `Apache-2.0` header in every file.  Zephyr uses
   Apache-2.0 (not BSD-3-Clause as in no-OS).

4. **EEPROM API contract** -- implement three callbacks:
   - `read(dev, offset, buf, len)` -- read `len` bytes starting at
     `offset`.
   - `write(dev, offset, buf, len)` -- write `len` bytes starting at
     `offset`, handling page boundaries internally.
   - `size(dev)` -- return total EEPROM capacity in bytes.

5. **Page-boundary writes** -- the driver must split writes at page
   boundaries.  A write that crosses a page boundary will wrap within
   the page on the hardware, corrupting data.

6. **Write cycle delay** -- after each page write, wait for the
   internal write cycle to complete.  Use `k_msleep(cfg->timeout)` or
   implement ACK polling.

7. **No `DEVICE_DT_INST_DEFINE()` wrapper** -- unlike sensor drivers
   (which use `SENSOR_DEVICE_DT_INST_DEFINE()`), EEPROM drivers use
   the base `DEVICE_DT_INST_DEFINE()` macro directly.

8. **Devicetree-driven config** -- all instance-specific parameters
   (I2C bus, address, memory size, page size, address width, write
   cycle time, WP GPIO) come from the devicetree.  Use
   `I2C_DT_SPEC_INST_GET()`, `GPIO_DT_SPEC_INST_GET()`,
   `DT_INST_PROP()`.

9. **API struct** -- use `DEVICE_API(eeprom, <devname>_api)` to define
   the driver API table with compile-time type safety.

10. **Logging** -- `LOG_MODULE_REGISTER(<devname>, CONFIG_EEPROM_LOG_LEVEL)`
    in the .c file.  Use `LOG_ERR` for errors, `LOG_INF` for
    informational messages, `LOG_DBG` for debug.  Never use `printk()`.

11. **Bus readiness** -- always check `i2c_is_ready_dt()` and
    `gpio_is_ready_dt()` in `init()` before any bus or GPIO access.

12. **Thread safety** -- protect shared state with `k_mutex`.  EEPROM
    read and write operations may be called from multiple threads.

13. **No dynamic allocation** -- all per-instance state is declared
    statically via the instantiation macros.  Never call `k_malloc()`
    or `k_calloc()` in a driver.

14. **Byte order** -- EEPROM memory addresses are big-endian on the
    wire.  Use `sys_put_be16()` from `<zephyr/sys/byteorder.h>`.

15. **Error codes** -- return negative `errno` values (`-EINVAL`,
    `-ENODEV`, `-EIO`, etc.).

16. **Init priority** -- use `CONFIG_EEPROM_INIT_PRIORITY` so the
    EEPROM initialises after the I2C bus is ready.

17. **Kconfig dependencies** -- use
    `depends on DT_HAS_ADI_<DEVNAME>_ENABLED` so the driver is only
    offered when a matching DT node exists.  Use `select I2C` to
    ensure the I2C subsystem is enabled.

---

## 11. Commit Message Format

Zephyr follows a strict commit message format.  Each commit must have a
subsystem prefix, a short subject, and an informative body.

### Adding a new driver (typically 3 commits):

```
# Commit 1: Devicetree binding
dts: bindings: add binding for Analog Devices <DEVNAME> EEPROM

Add devicetree binding for the Analog Devices <DEVNAME> I2C EEPROM
with <memory_size> of non-volatile storage.  Supports configurable
page size, address width, write cycle timeout, and optional
write-protect GPIO.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 2: Driver implementation
drivers: eeprom: add Analog Devices <DEVNAME> driver

Add Zephyr EEPROM subsystem driver for the Analog Devices <DEVNAME>
I2C EEPROM.  Supports page-based read/write with automatic page
boundary handling, configurable write cycle delay, and optional
write-protect GPIO control.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 3: Tests
tests: drivers: eeprom: add tests for <DEVNAME>

Add unit tests for the <DEVNAME> EEPROM driver covering read, write,
page-boundary crossing, bounds checking, and zero-length operations.

Signed-off-by: Your Name <your.name@analog.com>
```

### Key commit message rules:

- Subject line: max 72 characters, no trailing period
- Prefix: matches the path (e.g., `drivers: eeprom:`, `dts: bindings:`)
- Body: wrapped at 75 characters, explains the "why"
- Must include `Signed-off-by:` (DCO requirement)
- Use imperative mood ("add", not "added" or "adds")
