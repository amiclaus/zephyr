# Zephyr Display Driver Template

Reference: `drivers/display/DRIVER_TEMPLATE.md` (no-OS hardware patterns)

This template covers every file needed to add a new display controller
driver to the Zephyr RTOS for an Analog Devices part. Replace `<devname>`
with the part number in lowercase (e.g., `nhd_c12832a1z`), `<DEVNAME>`
with uppercase (e.g., `NHD_C12832A1Z`), and `<devcompat>` with the
devicetree compatible string portion (e.g., `nhd-c12832a1z`) throughout.

---

## 1. Purpose & Zephyr Subsystem Mapping

This driver maps to the Zephyr **display subsystem** defined in
`include/zephyr/drivers/display.h`. The subsystem provides a unified API
for display controllers with the following key entry points:

| Zephyr Display API Function          | What it does                                   |
|--------------------------------------|------------------------------------------------|
| `display_write()`                    | Write pixel data to a rectangular region       |
| `display_read()`                     | Read pixel data from a rectangular region      |
| `display_blanking_on()`              | Turn the display off (blank)                   |
| `display_blanking_off()`             | Turn the display on (unblank)                  |
| `display_set_brightness()`           | Set the display brightness (0-100)             |
| `display_set_contrast()`             | Set the display contrast (0-100)               |
| `display_set_pixel_format()`         | Set the active pixel format                    |
| `display_set_orientation()`          | Set the display orientation (rotation)         |
| `display_get_capabilities()`         | Query display resolution, pixel formats, etc.  |
| `display_get_framebuffer()`          | Get pointer to framebuffer (if applicable)     |

The driver implements the `display_driver_api` struct (via the
`DEVICE_API(display, ...)` macro) which contains pointers to the driver's
callback implementations.

All configuration comes from **devicetree** at compile time. There is no
dynamic allocation -- the config struct is `const` and populated from DT
macros, while the data struct is static and holds mutable runtime state
(framebuffer, blanking state, etc.).

**Note:** Display controllers are output-only peripherals. Unlike sensor
drivers, they do not produce measurement data, so IIO / sensor subsystem
integration is not applicable.

---

## 2. File Checklist

```
zephyr/
    drivers/display/
        <devname>.c               # Driver implementation
        Kconfig.<devname>         # Kconfig fragment
        CMakeLists.txt            # (append to existing)
        Kconfig                   # (append to existing)

    dts/bindings/display/
        adi,<devcompat>.yaml      # Devicetree binding

    tests/drivers/display/<devname>/
        testcase.yaml             # Test metadata
        prj.conf                  # Test project config
        boards/native_sim.overlay # DT overlay for test
        src/main.c                # Test source
```

---

## 3. Devicetree Binding (`dts/bindings/display/adi,<devcompat>.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Analog Devices <DEVNAME> monochrome graphic LCD display module.

  The <DEVNAME> is a <width>x<height> pixel monochrome display with a
  <controller> display controller, communicating via SPI with a separate
  Data/Command (DC) GPIO pin.

  Example devicetree node:

    &spi1 {
        status = "okay";
        cs-gpios = <&gpioa 4 GPIO_ACTIVE_LOW>;

        display0: <devcompat>@0 {
            compatible = "adi,<devcompat>";
            reg = <0>;
            spi-max-frequency = <10000000>;

            dc-gpios = <&gpiob 1 GPIO_ACTIVE_HIGH>;
            reset-gpios = <&gpiob 0 GPIO_ACTIVE_LOW>;

            width = <128>;
            height = <32>;
        };
    };

compatible: "adi,<devcompat>"

include: spi-device.yaml

properties:
  dc-gpios:
    type: phandle-array
    required: true
    description: |
      GPIO connected to the Data/Command (DC or A0) pin.
      When high the controller interprets SPI bytes as display RAM data;
      when low it interprets them as commands.

  reset-gpios:
    type: phandle-array
    description: |
      GPIO connected to the hardware reset pin. Optional; when omitted
      the driver skips the hardware reset sequence.

  width:
    type: int
    required: true
    description: |
      Display width in pixels (number of columns).

  height:
    type: int
    required: true
    description: |
      Display height in pixels. For page-addressed monochrome displays,
      height must be a multiple of 8.
```

---

## 4. Kconfig (`drivers/display/Kconfig.<devname>`)

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config DISPLAY_<DEVNAME>
	bool "<DEVNAME> display driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	select SPI
	help
	  Enable support for the Analog Devices <DEVNAME>
	  <width>x<height> monochrome graphic LCD display module
	  with <controller> controller and SPI interface.
```

Then add to the parent `drivers/display/Kconfig`:

```kconfig
source "drivers/display/Kconfig.<devname>"
```

---

## 5. CMakeLists.txt (Build System Integration)

Append to the existing `drivers/display/CMakeLists.txt`:

```cmake
zephyr_library_sources_ifdef(CONFIG_DISPLAY_<DEVNAME> <devname>.c)
```

---

## 6. Driver Header (Optional)

Display drivers are typically self-contained (all register defines and
command constants live in the `.c` file). However, if command definitions
must be shared with a test emulator, create
`include/zephyr/drivers/display/<devname>.h`:

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DISPLAY_<DEVNAME>_H_
#define ZEPHYR_INCLUDE_DRIVERS_DISPLAY_<DEVNAME>_H_

/* ---------------- Display Commands -------------------------------- */

/** Display on/off commands. */
#define <DEVNAME>_CMD_DISP_ON		0xAFU
#define <DEVNAME>_CMD_DISP_OFF		0xAEU

/** ADC select commands. */
#define <DEVNAME>_CMD_ADC_NORMAL	0xA0U
#define <DEVNAME>_CMD_ADC_REVERSE	0xA1U

/** COM output scan direction. */
#define <DEVNAME>_CMD_COM_NORMAL	0xC0U
#define <DEVNAME>_CMD_COM_REVERSE	0xC8U

/** LCD bias set. */
#define <DEVNAME>_CMD_LCD_BIAS		0xA2U

/** Power control. */
#define <DEVNAME>_CMD_PWR_CTRL		0x2FU

/** Resistor ratio. */
#define <DEVNAME>_CMD_RES_RATIO		0x21U

/** Electronic volume (contrast) commands. */
#define <DEVNAME>_CMD_ELECTRIC_VOL	0x81U
#define <DEVNAME>_CMD_ELECTRIC_VAL	0x20U

/** Page address base. */
#define <DEVNAME>_CMD_PAGE_ADDR		0xB0U

/** Display start line offset. */
#define <DEVNAME>_CMD_START_LINE	0x40U

/** Set column address upper nibble. */
#define <DEVNAME>_CMD_COL_UPPER		0x10U

/** Set column address lower nibble. */
#define <DEVNAME>_CMD_COL_LOWER		0x00U

/* ---------------- Display Constants ------------------------------- */

/** Bits per pixel for monochrome display. */
#define <DEVNAME>_BPP			1

#endif /* ZEPHYR_INCLUDE_DRIVERS_DISPLAY_<DEVNAME>_H_ */
```

---

## 7. Driver Source (`drivers/display/<devname>.c`)

This is the core of the driver. It follows the Zephyr display subsystem
contract and Linux kernel coding style.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(<devname>, CONFIG_DISPLAY_LOG_LEVEL);

/* ---------------- Display Commands -------------------------------- */
/*
 * If a shared header was created (Section 6), include it here instead.
 * Otherwise, define commands locally.
 */

#define <DEVNAME>_CMD_DISP_ON		0xAFU
#define <DEVNAME>_CMD_DISP_OFF		0xAEU
#define <DEVNAME>_CMD_ADC_NORMAL	0xA0U
#define <DEVNAME>_CMD_ADC_REVERSE	0xA1U
#define <DEVNAME>_CMD_COM_NORMAL	0xC0U
#define <DEVNAME>_CMD_COM_REVERSE	0xC8U
#define <DEVNAME>_CMD_LCD_BIAS		0xA2U
#define <DEVNAME>_CMD_PWR_CTRL		0x2FU
#define <DEVNAME>_CMD_RES_RATIO		0x21U
#define <DEVNAME>_CMD_ELECTRIC_VOL	0x81U
#define <DEVNAME>_CMD_ELECTRIC_VAL	0x20U
#define <DEVNAME>_CMD_PAGE_ADDR		0xB0U
#define <DEVNAME>_CMD_START_LINE	0x40U
#define <DEVNAME>_CMD_COL_UPPER		0x10U
#define <DEVNAME>_CMD_COL_LOWER		0x00U

/** Bits per pixel for monochrome display. */
#define <DEVNAME>_BPP			1

/* ---------------- Config & Data Structs --------------------------- */

/**
 * Compile-time configuration from devicetree.
 * This struct is const and stored in ROM.
 */
struct <devname>_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec dc_gpio;
	struct gpio_dt_spec reset_gpio;
	uint16_t width;
	uint16_t height;
};

/**
 * Mutable runtime data.
 * This struct is allocated statically per instance.
 */
struct <devname>_data {
	/** Blanking state: true = display off, false = display on. */
	bool blanking;
	/** Current contrast value (0-255). */
	uint8_t contrast;
	/*
	 * Framebuffer: pages x columns.
	 * For a 128x32 monochrome display: 4 pages x 128 columns = 512 bytes.
	 * Allocate the maximum supported size; actual usage depends on
	 * the width/height from DT.
	 *
	 * Adjust <DEVNAME>_MAX_WIDTH and <DEVNAME>_MAX_HEIGHT to the
	 * maximum dimensions your driver family supports.
	 */
#define <DEVNAME>_MAX_WIDTH		128
#define <DEVNAME>_MAX_HEIGHT		64
#define <DEVNAME>_MAX_PAGES		(<DEVNAME>_MAX_HEIGHT / 8)
	uint8_t framebuffer[<DEVNAME>_MAX_PAGES][<DEVNAME>_MAX_WIDTH];
};

/* ---------------- SPI Helpers ------------------------------------- */

/**
 * @brief Write a command byte to the display controller.
 *
 * Sets the DC pin low (command mode), then sends the byte over SPI.
 */
static int <devname>_write_cmd(const struct device *dev, uint8_t cmd)
{
	const struct <devname>_config *cfg = dev->config;
	int ret;

	ret = gpio_pin_set_dt(&cfg->dc_gpio, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set DC low: %d", ret);
		return ret;
	}

	const struct spi_buf tx = { .buf = &cmd, .len = 1 };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	ret = spi_write_dt(&cfg->spi, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI command write failed: %d", ret);
	}

	return ret;
}

/**
 * @brief Write a data buffer to the display controller.
 *
 * Sets the DC pin high (data mode), then sends the buffer over SPI.
 */
static int <devname>_write_data(const struct device *dev, const uint8_t *data,
				size_t len)
{
	const struct <devname>_config *cfg = dev->config;
	int ret;

	ret = gpio_pin_set_dt(&cfg->dc_gpio, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set DC high: %d", ret);
		return ret;
	}

	const struct spi_buf tx = { .buf = (void *)data, .len = len };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	ret = spi_write_dt(&cfg->spi, &tx_set);
	if (ret < 0) {
		LOG_ERR("SPI data write failed: %d", ret);
	}

	return ret;
}

/**
 * @brief Set the page and column address for the next write.
 */
static int <devname>_set_cursor(const struct device *dev, uint8_t page,
				uint8_t col)
{
	int ret;

	ret = <devname>_write_cmd(dev, <DEVNAME>_CMD_PAGE_ADDR | page);
	if (ret < 0) {
		return ret;
	}

	ret = <devname>_write_cmd(dev, <DEVNAME>_CMD_COL_UPPER | (col >> 4));
	if (ret < 0) {
		return ret;
	}

	return <devname>_write_cmd(dev, <DEVNAME>_CMD_COL_LOWER | (col & 0x0F));
}

/**
 * @brief Flush the full framebuffer to the display over SPI.
 */
static int <devname>_flush_fb(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint8_t num_pages = cfg->height / 8;
	int ret;

	for (uint8_t page = 0; page < num_pages; page++) {
		ret = <devname>_set_cursor(dev, page, 0);
		if (ret < 0) {
			return ret;
		}

		ret = <devname>_write_data(dev, data->framebuffer[page],
					   cfg->width);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

/* ---------------- Display Subsystem Callbacks --------------------- */

/**
 * @brief Turn blanking on (display off).
 *
 * Sends the display-off command. The framebuffer contents are preserved.
 */
static int <devname>_blanking_on(const struct device *dev)
{
	struct <devname>_data *data = dev->data;
	int ret;

	ret = <devname>_write_cmd(dev, <DEVNAME>_CMD_DISP_OFF);
	if (ret < 0) {
		return ret;
	}

	data->blanking = true;

	return 0;
}

/**
 * @brief Turn blanking off (display on).
 *
 * Sends the display-on command. The previously written framebuffer
 * contents become visible.
 */
static int <devname>_blanking_off(const struct device *dev)
{
	struct <devname>_data *data = dev->data;
	int ret;

	ret = <devname>_write_cmd(dev, <DEVNAME>_CMD_DISP_ON);
	if (ret < 0) {
		return ret;
	}

	data->blanking = false;

	return 0;
}

/**
 * @brief Write pixel data to a rectangular region of the display.
 *
 * @param dev        Device handle.
 * @param x          X coordinate (column) of the top-left corner.
 * @param y          Y coordinate (row) of the top-left corner.
 * @param desc       Buffer descriptor with width, height, and pitch.
 * @param buf        Pointer to the pixel data to write.
 *
 * For monochrome page-addressed displays, the y coordinate and height
 * must be aligned to 8-pixel page boundaries. Each byte in the input
 * buffer maps to 8 vertical pixels (one page row).
 *
 * The driver copies the data into its internal framebuffer and then
 * flushes the affected pages to the display.
 */
static int <devname>_write(const struct device *dev, const uint16_t x,
			   const uint16_t y,
			   const struct display_buffer_descriptor *desc,
			   const void *buf)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	const uint8_t *src = (const uint8_t *)buf;
	uint8_t start_page;
	uint8_t num_pages;
	int ret;

	/* Validate coordinates */
	if (x >= cfg->width || y >= cfg->height) {
		LOG_ERR("Write position (%u, %u) out of bounds", x, y);
		return -EINVAL;
	}

	if ((x + desc->width) > cfg->width ||
	    (y + desc->height) > cfg->height) {
		LOG_ERR("Write region exceeds display dimensions");
		return -EINVAL;
	}

	/* For page-addressed displays, y must be page-aligned */
	if ((y % 8) != 0 || (desc->height % 8) != 0) {
		LOG_ERR("Y and height must be aligned to 8-pixel pages");
		return -EINVAL;
	}

	start_page = y / 8;
	num_pages = desc->height / 8;

	/* Copy pixel data into framebuffer */
	for (uint8_t page = 0; page < num_pages; page++) {
		for (uint16_t col = 0; col < desc->width; col++) {
			data->framebuffer[start_page + page][x + col] =
				src[page * desc->pitch + col];
		}
	}

	/* Flush the affected pages to the display */
	for (uint8_t page = start_page; page < start_page + num_pages; page++) {
		ret = <devname>_set_cursor(dev, page, x);
		if (ret < 0) {
			return ret;
		}

		ret = <devname>_write_data(dev,
					   &data->framebuffer[page][x],
					   desc->width);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

/**
 * @brief Read pixel data from a rectangular region of the display.
 *
 * Reads from the driver's internal framebuffer (not from the display
 * controller's RAM, since most SPI display controllers do not support
 * read-back). The returned data reflects the last write() call.
 */
static int <devname>_read(const struct device *dev, const uint16_t x,
			  const uint16_t y,
			  const struct display_buffer_descriptor *desc,
			  void *buf)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint8_t *dst = (uint8_t *)buf;
	uint8_t start_page;
	uint8_t num_pages;

	if (x >= cfg->width || y >= cfg->height) {
		return -EINVAL;
	}

	if ((x + desc->width) > cfg->width ||
	    (y + desc->height) > cfg->height) {
		return -EINVAL;
	}

	if ((y % 8) != 0 || (desc->height % 8) != 0) {
		return -EINVAL;
	}

	start_page = y / 8;
	num_pages = desc->height / 8;

	for (uint8_t page = 0; page < num_pages; page++) {
		for (uint16_t col = 0; col < desc->width; col++) {
			dst[page * desc->pitch + col] =
				data->framebuffer[start_page + page][x + col];
		}
	}

	return 0;
}

/**
 * @brief Get the display capabilities.
 *
 * Reports the display resolution, supported pixel formats, current
 * pixel format, and screen orientation.
 */
static void <devname>_get_capabilities(const struct device *dev,
				       struct display_capabilities *caps)
{
	const struct <devname>_config *cfg = dev->config;

	memset(caps, 0, sizeof(*caps));

	caps->x_resolution = cfg->width;
	caps->y_resolution = cfg->height;
	caps->supported_pixel_formats = PIXEL_FORMAT_MONO01 |
					PIXEL_FORMAT_MONO10;
	caps->current_pixel_format = PIXEL_FORMAT_MONO10;
	caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
	caps->screen_info = SCREEN_INFO_MONO_VTILED;
}

/**
 * @brief Set the pixel format.
 *
 * Monochrome displays support PIXEL_FORMAT_MONO01 and PIXEL_FORMAT_MONO10.
 */
static int <devname>_set_pixel_format(const struct device *dev,
				      enum display_pixel_format format)
{
	if (format != PIXEL_FORMAT_MONO01 && format != PIXEL_FORMAT_MONO10) {
		LOG_ERR("Unsupported pixel format: %d", format);
		return -ENOTSUP;
	}

	/*
	 * For controllers that support inversion via command (e.g., 0xA6/0xA7),
	 * send the appropriate command here to switch between MONO01/MONO10.
	 */

	LOG_DBG("Pixel format set to %d", format);

	return 0;
}

/**
 * @brief Set the display brightness.
 *
 * Not all display controllers support brightness control. Return
 * -ENOTSUP if the hardware does not have this capability.
 */
static int <devname>_set_brightness(const struct device *dev,
				    const uint8_t brightness)
{
	LOG_WRN("Brightness control not supported");
	return -ENOTSUP;
}

/**
 * @brief Set the display contrast.
 *
 * For controllers that support electronic volume / contrast adjustment,
 * send the contrast command sequence. The input is 0-255.
 */
static int <devname>_set_contrast(const struct device *dev,
				  const uint8_t contrast)
{
	struct <devname>_data *data = dev->data;
	int ret;

	ret = <devname>_write_cmd(dev, <DEVNAME>_CMD_ELECTRIC_VOL);
	if (ret < 0) {
		return ret;
	}

	/* Scale 0-255 input to the device's contrast range (e.g., 0-63) */
	uint8_t hw_contrast = contrast >> 2;

	ret = <devname>_write_cmd(dev, hw_contrast);
	if (ret < 0) {
		return ret;
	}

	data->contrast = contrast;

	return 0;
}

/**
 * @brief Set the display orientation.
 *
 * Supports normal and 180-degree rotation via ADC and COM scan direction
 * commands. 90/270-degree rotations are not supported in hardware.
 */
static int <devname>_set_orientation(const struct device *dev,
				     const enum display_orientation orientation)
{
	int ret;

	switch (orientation) {
	case DISPLAY_ORIENTATION_NORMAL:
		ret = <devname>_write_cmd(dev, <DEVNAME>_CMD_ADC_NORMAL);
		if (ret < 0) {
			return ret;
		}
		return <devname>_write_cmd(dev, <DEVNAME>_CMD_COM_NORMAL);

	case DISPLAY_ORIENTATION_ROTATED_180:
		ret = <devname>_write_cmd(dev, <DEVNAME>_CMD_ADC_REVERSE);
		if (ret < 0) {
			return ret;
		}
		return <devname>_write_cmd(dev, <DEVNAME>_CMD_COM_REVERSE);

	default:
		LOG_ERR("Unsupported orientation: %d", orientation);
		return -ENOTSUP;
	}
}

/**
 * @brief Get the framebuffer pointer.
 *
 * Returns NULL because this driver does not expose a memory-mapped
 * framebuffer. Pixel data must be written via display_write().
 */
static void *<devname>_get_framebuffer(const struct device *dev)
{
	return NULL;
}

/* ---------------- Initialization ---------------------------------- */

/**
 * @brief Send the controller initialization command sequence.
 */
static int <devname>_hw_init(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	int ret;

	/* Hardware reset sequence (if reset GPIO is available) */
	if (cfg->reset_gpio.port != NULL) {
		ret = gpio_pin_set_dt(&cfg->reset_gpio, 1);
		if (ret < 0) {
			LOG_ERR("Failed to assert reset: %d", ret);
			return ret;
		}

		k_usleep(3);

		ret = gpio_pin_set_dt(&cfg->reset_gpio, 0);
		if (ret < 0) {
			LOG_ERR("Failed to deassert reset: %d", ret);
			return ret;
		}

		/* Wait for the controller to come out of reset */
		k_msleep(1);
	}

	/* Controller initialization command sequence */
	static const uint8_t init_cmds[] = {
		<DEVNAME>_CMD_DISP_OFF,
		<DEVNAME>_CMD_ADC_NORMAL,
		<DEVNAME>_CMD_COM_REVERSE,
		<DEVNAME>_CMD_LCD_BIAS,
		<DEVNAME>_CMD_PWR_CTRL,
		<DEVNAME>_CMD_RES_RATIO,
		<DEVNAME>_CMD_ELECTRIC_VOL,
		<DEVNAME>_CMD_ELECTRIC_VAL,
		<DEVNAME>_CMD_START_LINE,
	};

	for (size_t i = 0; i < ARRAY_SIZE(init_cmds); i++) {
		ret = <devname>_write_cmd(dev, init_cmds[i]);
		if (ret < 0) {
			LOG_ERR("Init command 0x%02X failed: %d",
				init_cmds[i], ret);
			return ret;
		}
	}

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

	/* Verify SPI bus is ready */
	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Configure DC GPIO */
	if (!gpio_is_ready_dt(&cfg->dc_gpio)) {
		LOG_ERR("DC GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->dc_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure DC GPIO: %d", ret);
		return ret;
	}

	/* Configure reset GPIO (optional) */
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
	}

	/* Send controller initialization sequence */
	ret = <devname>_hw_init(dev);
	if (ret < 0) {
		return ret;
	}

	/* Clear framebuffer */
	memset(data->framebuffer, 0, sizeof(data->framebuffer));

	/* Flush cleared framebuffer to display */
	ret = <devname>_flush_fb(dev);
	if (ret < 0) {
		return ret;
	}

	/* Turn display on */
	ret = <devname>_blanking_off(dev);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("<DEVNAME> (%ux%u) initialized on %s",
		cfg->width, cfg->height, cfg->spi.bus->name);

	return 0;
}

/* ---------------- API Struct -------------------------------------- */

/*
 * Use the DEVICE_API macro. This creates a static const struct that
 * the Zephyr display subsystem uses to dispatch calls.
 */
static DEVICE_API(display, <devname>_api) = {
	.blanking_on = <devname>_blanking_on,
	.blanking_off = <devname>_blanking_off,
	.write = <devname>_write,
	.read = <devname>_read,
	.get_framebuffer = <devname>_get_framebuffer,
	.set_brightness = <devname>_set_brightness,
	.set_contrast = <devname>_set_contrast,
	.get_capabilities = <devname>_get_capabilities,
	.set_pixel_format = <devname>_set_pixel_format,
	.set_orientation = <devname>_set_orientation,
};

/* ---------------- Instance Macros --------------------------------- */

/*
 * These macros are expanded once per DT instance with status "okay".
 * They create the config, data, and DEVICE_DT_INST_DEFINE entries.
 */

#define <DEVNAME>_RESET_GPIO_INIT(n)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, reset_gpios),		\
		(.reset_gpio = GPIO_DT_SPEC_INST_GET(n, reset_gpios),),\
		(.reset_gpio = { 0 },))

#define <DEVNAME>_INIT(n)						\
	static struct <devname>_data <devname>_data_##n;		\
									\
	static const struct <devname>_config <devname>_config_##n = {	\
		.spi = SPI_DT_SPEC_INST_GET(n,				\
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),	\
		.dc_gpio = GPIO_DT_SPEC_INST_GET(n, dc_gpios),		\
		<DEVNAME>_RESET_GPIO_INIT(n)				\
		.width = DT_INST_PROP(n, width),			\
		.height = DT_INST_PROP(n, height),			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, <devname>_init, NULL,			\
			      &<devname>_data_##n,			\
			      &<devname>_config_##n,			\
			      POST_KERNEL,				\
			      CONFIG_DISPLAY_INIT_PRIORITY,		\
			      &<devname>_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_INIT)
```

### Key elements explained

| Element | Purpose |
|---------|---------|
| `DT_DRV_COMPAT` | Must match the compatible string with commas replaced by underscores |
| `struct <devname>_config` | Compile-time config from DT: SPI bus, DC GPIO, reset GPIO, dimensions |
| `struct <devname>_data` | Mutable runtime state: framebuffer, blanking flag, contrast |
| `DEVICE_API(display, ...)` | Typed API struct -- replaces untyped `struct display_driver_api` literal |
| `SPI_DT_SPEC_INST_GET()` | Pulls SPI bus, CS, and frequency from DT |
| `GPIO_DT_SPEC_INST_GET()` | Pulls GPIO port/pin/flags from DT |
| `DEVICE_DT_INST_DEFINE()` | Registers the device with Zephyr's device model |
| `DT_INST_FOREACH_STATUS_OKAY()` | Instantiates one driver per DT node with `status = "okay"` |
| `LOG_MODULE_REGISTER()` | Creates a logging module; level controlled by `CONFIG_DISPLAY_LOG_LEVEL` |
| `dc_gpio` | Data/Command GPIO -- the key pattern for display controllers (DC low = command, DC high = data) |
| `framebuffer[]` | Local framebuffer for page-addressed monochrome displays; enables `read()` without hardware read-back |

---

## 8. Display Subsystem Callback Details

This section explains the display subsystem callbacks in detail and
how they map to the hardware.

### DC Pin Pattern

Display controllers distinguish between command bytes and data bytes
using the DC (Data/Command) GPIO pin. This is the fundamental
difference from register-based device drivers:

- **DC low** -- the byte is interpreted as a controller command
  (`<devname>_write_cmd`).
- **DC high** -- the byte is interpreted as display RAM data
  (`<devname>_write_data`).

Every SPI transfer must be preceded by setting the DC pin to the
correct state.

### Page-Addressed Framebuffer

Monochrome display controllers organize their display RAM as pages.
Each page is 8 pixels tall and spans the full display width. A single
byte written in data mode maps to 8 vertical pixels at the current
page and column position.

For a 128x32 display:
- 4 pages (32 / 8 = 4), each 128 columns wide
- Total framebuffer size: 4 x 128 = 512 bytes

The `SCREEN_INFO_MONO_VTILED` flag in `get_capabilities()` tells
the display subsystem about this vertical tiling layout.

### Callback Mapping

| Callback | Hardware Action |
|----------|----------------|
| `blanking_on` | Send display-off command (0xAE) |
| `blanking_off` | Send display-on command (0xAF) |
| `write` | Copy data to framebuffer, flush affected pages via SPI |
| `read` | Copy data from local framebuffer (no SPI read-back) |
| `get_capabilities` | Return resolution, pixel format, tiling info |
| `set_pixel_format` | Switch MONO01/MONO10 via display inversion command |
| `set_contrast` | Send electronic volume command pair |
| `set_brightness` | Return -ENOTSUP (hardware limitation) |
| `set_orientation` | Switch ADC/COM scan direction commands |
| `get_framebuffer` | Return NULL (no memory-mapped framebuffer) |

---

## 9. Test Skeleton

### 9.1 Test Metadata (`tests/drivers/display/<devname>/testcase.yaml`)

```yaml
tests:
  drivers.display.<devname>:
    tags:
      - drivers
      - display
    depends_on: spi gpio
    platform_allow: native_sim
    integration_platforms:
      - native_sim
```

### 9.2 Test Project Config (`tests/drivers/display/<devname>/prj.conf`)

```
CONFIG_ZTEST=y
CONFIG_DISPLAY=y
CONFIG_DISPLAY_<DEVNAME>=y
CONFIG_SPI=y
CONFIG_GPIO=y
CONFIG_LOG=y
```

### 9.3 DT Overlay (`tests/drivers/display/<devname>/boards/native_sim.overlay`)

```dts
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

&spi0 {
	status = "okay";

	<devname>_test: <devcompat>@0 {
		compatible = "adi,<devcompat>";
		reg = <0>;
		spi-max-frequency = <1000000>;

		dc-gpios = <&gpio0 0 GPIO_ACTIVE_HIGH>;
		reset-gpios = <&gpio0 1 GPIO_ACTIVE_LOW>;

		width = <128>;
		height = <32>;
	};
};
```

### 9.4 Test Source (`tests/drivers/display/<devname>/src/main.c`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/display.h>
#include <zephyr/device.h>
#include <string.h>

#define DISPLAY_NODE DT_NODELABEL(<devname>_test)
static const struct device *display_dev = DEVICE_DT_GET(DISPLAY_NODE);

ZTEST(display_<devname>, test_device_ready)
{
	zassert_true(device_is_ready(display_dev),
		     "Display device not ready");
}

ZTEST(display_<devname>, test_get_capabilities)
{
	struct display_capabilities caps;

	display_get_capabilities(display_dev, &caps);

	zassert_equal(caps.x_resolution, 128,
		      "Unexpected x_resolution: %u", caps.x_resolution);
	zassert_equal(caps.y_resolution, 32,
		      "Unexpected y_resolution: %u", caps.y_resolution);
	zassert_true(caps.supported_pixel_formats & PIXEL_FORMAT_MONO10,
		     "MONO10 not supported");
	zassert_equal(caps.screen_info & SCREEN_INFO_MONO_VTILED,
		      SCREEN_INFO_MONO_VTILED,
		      "Expected MONO_VTILED screen info");
}

ZTEST(display_<devname>, test_blanking)
{
	int ret;

	ret = display_blanking_on(display_dev);
	zassert_ok(ret, "blanking_on failed: %d", ret);

	ret = display_blanking_off(display_dev);
	zassert_ok(ret, "blanking_off failed: %d", ret);
}

ZTEST(display_<devname>, test_write_and_read)
{
	int ret;
	uint8_t write_buf[128];  /* One page, full width */
	uint8_t read_buf[128];

	/* Fill with a test pattern */
	memset(write_buf, 0xAA, sizeof(write_buf));

	struct display_buffer_descriptor desc = {
		.buf_size = sizeof(write_buf),
		.width = 128,
		.height = 8,
		.pitch = 128,
	};

	/* Write to page 0 (y=0, height=8) */
	ret = display_write(display_dev, 0, 0, &desc, write_buf);
	zassert_ok(ret, "display_write failed: %d", ret);

	/* Read back */
	memset(read_buf, 0, sizeof(read_buf));
	ret = display_read(display_dev, 0, 0, &desc, read_buf);
	zassert_ok(ret, "display_read failed: %d", ret);

	zassert_mem_equal(write_buf, read_buf, sizeof(write_buf),
			  "Read-back data mismatch");
}

ZTEST(display_<devname>, test_write_out_of_bounds)
{
	int ret;
	uint8_t buf[8];
	struct display_buffer_descriptor desc = {
		.buf_size = sizeof(buf),
		.width = 8,
		.height = 8,
		.pitch = 8,
	};

	/* Write beyond the display width */
	ret = display_write(display_dev, 200, 0, &desc, buf);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for out-of-bounds write, got %d", ret);
}

ZTEST(display_<devname>, test_set_pixel_format)
{
	int ret;

	ret = display_set_pixel_format(display_dev, PIXEL_FORMAT_MONO10);
	zassert_ok(ret, "set_pixel_format MONO10 failed: %d", ret);

	ret = display_set_pixel_format(display_dev, PIXEL_FORMAT_MONO01);
	zassert_ok(ret, "set_pixel_format MONO01 failed: %d", ret);

	/* Unsupported format */
	ret = display_set_pixel_format(display_dev, PIXEL_FORMAT_RGB_888);
	zassert_equal(ret, -ENOTSUP,
		      "Expected -ENOTSUP for RGB_888, got %d", ret);
}

ZTEST(display_<devname>, test_set_contrast)
{
	int ret;

	ret = display_set_contrast(display_dev, 128);
	zassert_ok(ret, "set_contrast failed: %d", ret);
}

ZTEST(display_<devname>, test_set_orientation)
{
	int ret;

	ret = display_set_orientation(display_dev,
				      DISPLAY_ORIENTATION_NORMAL);
	zassert_ok(ret, "set_orientation NORMAL failed: %d", ret);

	ret = display_set_orientation(display_dev,
				      DISPLAY_ORIENTATION_ROTATED_180);
	zassert_ok(ret, "set_orientation ROTATED_180 failed: %d", ret);

	/* 90-degree rotation not supported */
	ret = display_set_orientation(display_dev,
				      DISPLAY_ORIENTATION_ROTATED_90);
	zassert_equal(ret, -ENOTSUP,
		      "Expected -ENOTSUP for 90-degree rotation, got %d",
		      ret);
}

ZTEST_SUITE(display_<devname>, NULL, NULL, NULL, NULL, NULL);
```

### 9.5 SPI Emulator Pattern (Optional)

For proper hardware-in-the-loop testing on `native_sim`, create an SPI
emulator. Display controllers are simpler to emulate than ADCs because
commands are write-only -- the emulator just needs to accept and
silently consume the SPI traffic.

```c
/*
 * tests/drivers/display/<devname>/src/<devname>_emul.c
 *
 * Minimal SPI emulator for <DEVNAME> display tests.
 */

#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/spi_emul.h>

struct <devname>_emul_data {
	/* Display emulators typically need no state for basic tests */
	bool blanking;
};

static int <devname>_emul_io(const struct emul *target,
			     const struct spi_config *config,
			     const struct spi_buf_set *tx_bufs,
			     const struct spi_buf_set *rx_bufs)
{
	/*
	 * For display drivers, the emulator just accepts all
	 * SPI writes. No read-back is needed since the driver
	 * uses a local framebuffer.
	 */
	return 0;
}

static const struct spi_emul_api <devname>_emul_api = {
	.io = <devname>_emul_io,
};

static int <devname>_emul_init(const struct emul *target,
			       const struct device *parent)
{
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

5. **SPI specs** -- always use `spi_dt_spec` obtained from
   `SPI_DT_SPEC_INST_GET()`. Never store raw bus pointers.

6. **Bus readiness** -- check `spi_is_ready_dt()` in `init()` before
   any bus access.

7. **Error codes** -- return negative `errno` values (`-EINVAL`,
   `-ENOTSUP`, `-ENODEV`, `-ENOMEM`, etc.). Never return positive
   error codes.

8. **Logging** -- use `LOG_MODULE_REGISTER(<devname>, CONFIG_DISPLAY_LOG_LEVEL)`.
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

12. **DEVICE_API macro** -- always use `DEVICE_API(display, <devname>_api)`
    for the API struct declaration, not a raw `struct display_driver_api`
    assignment.

13. **Init priority** -- use `CONFIG_DISPLAY_INIT_PRIORITY` (default 80)
    unless the driver depends on another subsystem that initializes
    later.

14. **DC pin pattern** -- display drivers toggle the Data/Command GPIO
    before every SPI transfer; commands set DC low, pixel data sets
    DC high. This is the fundamental difference from register-based
    device drivers and must be handled correctly in every SPI helper.

15. **No IIO / sensor mapping** -- display controllers are output-only
    peripherals. They do not produce measurement data, so neither the
    Zephyr sensor subsystem nor the IIO subsystem applies.

16. **Framebuffer ownership** -- the driver owns a local framebuffer
    because most SPI display controllers do not support read-back.
    The `read()` callback returns data from this local copy, not from
    the hardware.

---

## 11. Commit Message Format

Zephyr follows a strict commit message format. Each commit must have a
subsystem prefix, a short subject, and an informative body.

### Adding a new driver (typically 3-4 commits):

```
# Commit 1: Devicetree binding
dts: bindings: add binding for Analog Devices <DEVNAME>

Add devicetree binding for the Analog Devices <DEVNAME>
<width>x<height> monochrome graphic LCD display module with
<controller> controller and SPI interface.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 2: Driver implementation
drivers: display: add Analog Devices <DEVNAME> driver

Add support for the Analog Devices <DEVNAME> <width>x<height>
monochrome graphic LCD display module. The driver implements the
Zephyr display subsystem API with support for page-addressed
framebuffer writes, blanking control, contrast adjustment, and
180-degree rotation.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 3: Tests
tests: drivers: display: add tests for <DEVNAME>

Add unit tests for the <DEVNAME> display driver covering device
readiness, capabilities query, blanking, write/read, pixel format,
contrast, and orientation.

Signed-off-by: Your Name <your.name@analog.com>
```

### Key commit message rules:

- Subject line: max 72 characters, no trailing period
- Prefix: matches the path (e.g., `drivers: display:`, `dts: bindings:`)
- Body: wrapped at 75 characters, explains the "why"
- Must include `Signed-off-by:` (DCO requirement)
- Use imperative mood ("add", not "added" or "adds")
