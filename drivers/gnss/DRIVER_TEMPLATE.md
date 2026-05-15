# GNSS/GPS Zephyr Driver Template

Reference: Zephyr GNSS subsystem (`drivers/gnss/` in the Zephyr tree)

This template covers every file needed to add a new GNSS/GPS receiver driver
to the Zephyr RTOS GNSS subsystem. GNSS devices communicate over UART using
NMEA 0183 sentences and optional vendor binary protocols. The Zephyr GNSS
subsystem provides a unified API for navigation data (position, velocity,
time), satellite information, and fix control.

Replace `<devname>` with the part number in lowercase (e.g., `max_m10s`),
`<DEVNAME>` with uppercase (e.g., `MAX_M10S`), and `<vendorname>` with the
vendor prefix (e.g., `ublox`) throughout.

---

## 1. Purpose & Zephyr Subsystem Mapping

This driver maps to the Zephyr **GNSS subsystem** defined in
`include/zephyr/drivers/gnss.h`. The subsystem provides a unified API for
GNSS receivers with the following key entry points:

| Zephyr GNSS API Function           | What it does                                    |
|-------------------------------------|-------------------------------------------------|
| `gnss_set_fix_rate()`              | Set the position fix rate in milliseconds        |
| `gnss_get_fix_rate()`              | Get the current fix rate                         |
| `gnss_set_navigation_mode()`       | Set navigation mode (zero dynamics, low, etc.)   |
| `gnss_get_navigation_mode()`       | Get the current navigation mode                  |
| `gnss_set_enabled_systems()`       | Enable/disable GNSS constellations               |
| `gnss_get_enabled_systems()`       | Query enabled constellations                     |
| `gnss_get_supported_systems()`     | Query which constellations the hardware supports |

The driver implements the `gnss_driver_api` struct (via the
`DEVICE_API(gnss, ...)` macro) which contains function pointers for the
operations above.

Navigation data and satellite information are delivered asynchronously via
**callbacks** registered by the application:

| Callback Type                      | Purpose                                         |
|------------------------------------|-------------------------------------------------|
| `GNSS_DATA_CALLBACK_DEFINE()`      | Receive `gnss_data` (PVT fix, satellite count)  |
| `GNSS_SATELLITES_CALLBACK_DEFINE()`| Receive per-satellite info (SV, SNR, elevation)  |

The GNSS subsystem includes built-in NMEA 0183 parsing helpers in
`gnss_nmea0183.h` and a match callback system for dispatching individual
NMEA sentence types.

All configuration comes from **devicetree** at compile time. There is no
dynamic allocation -- the config struct is `const` and populated from DT
macros, while the data struct is static and holds mutable runtime state.

---

## 2. File Checklist

```
zephyr/
    drivers/gnss/
        gnss_<devname>.c              # Driver implementation
        Kconfig.<devname>             # Kconfig fragment
        CMakeLists.txt                # (append to existing)
        Kconfig                       # (append to existing)

    dts/bindings/gnss/
        adi,<devname>.yaml            # Devicetree binding

    tests/drivers/gnss/<devname>/
        testcase.yaml                 # Test metadata
        prj.conf                      # Test project config
        boards/native_sim.overlay     # DT overlay for test
        src/main.c                    # Test source

    samples/drivers/gnss/<devname>/   # Optional sample application
        CMakeLists.txt
        prj.conf
        boards/<board>.overlay
        src/main.c
```

---

## 3. Devicetree Binding (`dts/bindings/gnss/adi,<devname>.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  Analog Devices <DEVNAME> multi-constellation GNSS receiver module.
  Communicates over UART using NMEA 0183 and an optional vendor
  binary protocol. Supports GPS, GLONASS, Galileo, and BeiDou.

  Example devicetree node:

    &uart2 {
        status = "okay";
        current-speed = <9600>;

        gnss0: <devname> {
            compatible = "adi,<devname>";
            current-speed = <9600>;

            /* Optional: PPS output */
            /* pps-gpios = <&gpiob 5 GPIO_ACTIVE_HIGH>; */

            /* Optional: hardware reset */
            /* reset-gpios = <&gpioa 8 GPIO_ACTIVE_LOW>; */
        };
    };

compatible: "adi,<devname>"

include: [uart-device.yaml, base.yaml]

properties:
  current-speed:
    type: int
    default: 9600
    description: |
      UART baud rate for communication with the GNSS receiver.
      Common values: 9600 (default), 38400, 115200.

  pps-gpios:
    type: phandle-array
    description: |
      GPIO connected to the Pulse-Per-Second (PPS) output of the
      GNSS receiver. This signal is aligned to the GNSS time grid
      and asserts once per second when a valid fix is available.

  reset-gpios:
    type: phandle-array
    description: |
      GPIO connected to the hardware reset pin of the GNSS receiver.
      Active-low. When asserted, the receiver performs a full reset.

  pps-frequency:
    type: int
    default: 1
    description: |
      PPS output frequency in Hz. Default is 1 Hz (once per second).
      Some receivers support higher rates (e.g., 10 Hz).

  pps-pulse-length-ms:
    type: int
    default: 100
    description: |
      PPS pulse duration in milliseconds. Default is 100 ms.
```

For vendor-specific bindings (e.g., u-blox), use the appropriate vendor
prefix (e.g., `ublox,<devname>`) and adjust the compatible string.

---

## 4. Kconfig (`drivers/gnss/Kconfig.<devname>`)

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

config GNSS_<DEVNAME>
	bool "<DEVNAME> GNSS receiver driver"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	depends on GNSS
	select UART_INTERRUPT_DRIVEN
	select GNSS_NMEA0183
	select GNSS_NMEA0183_MATCH
	help
	  Enable support for the Analog Devices <DEVNAME>
	  multi-constellation GNSS receiver module. The driver
	  communicates over UART using NMEA 0183 sentences and
	  an optional vendor binary protocol.

config GNSS_<DEVNAME>_SATELLITES
	bool "Report satellite information from <DEVNAME>"
	depends on GNSS_<DEVNAME>
	depends on GNSS_SATELLITES
	default y
	help
	  Enable reporting of per-satellite information (PRN,
	  SNR, elevation, azimuth) via the GNSS satellites
	  callback.

config GNSS_<DEVNAME>_VENDOR_BINARY
	bool "Enable vendor binary protocol for <DEVNAME>"
	depends on GNSS_<DEVNAME>
	default n
	help
	  Enable the vendor-specific binary protocol in addition
	  to NMEA 0183. This provides nanosecond time precision
	  and additional configuration capabilities.
```

Then add to the parent `drivers/gnss/Kconfig`:

```kconfig
source "drivers/gnss/Kconfig.<devname>"
```

---

## 5. CMakeLists.txt (Build System Integration)

Append to the existing `drivers/gnss/CMakeLists.txt`:

```cmake
zephyr_library_sources_ifdef(CONFIG_GNSS_<DEVNAME> gnss_<devname>.c)
```

---

## 6. Driver Header (Optional)

If register definitions, protocol constants, or data types must be shared
with tests or emulators, create a header in the driver directory. For most
GNSS drivers, all definitions are kept in the `.c` file since the GNSS
subsystem provides the public API.

If needed, create `drivers/gnss/gnss_<devname>.h`:

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GNSS_<DEVNAME>_H_
#define ZEPHYR_DRIVERS_GNSS_<DEVNAME>_H_

/* ---------------- NMEA Constants ---------------------------------- */

/** Maximum NMEA sentence length (including CR+LF). */
#define <DEVNAME>_NMEA_MAX_LEN			82

/** NMEA sentence start delimiter. */
#define <DEVNAME>_NMEA_START			'$'

/** NMEA sentence end (CR+LF). */
#define <DEVNAME>_NMEA_CR			'\r'
#define <DEVNAME>_NMEA_LF			'\n'

/* ---------------- Vendor Binary Protocol Constants --------------- */

/** Maximum binary packet size (header + payload + checksum). */
#define <DEVNAME>_MAX_PACKET_SIZE		1024

/** Binary protocol header size. */
#define <DEVNAME>_HEADER_SIZE			6

/** Binary protocol checksum size. */
#define <DEVNAME>_CHECKSUM_SIZE			2

/* Sync bytes */
#define <DEVNAME>_SYNC_1			0xB5
#define <DEVNAME>_SYNC_2			0x62

/* Message class IDs */
#define <DEVNAME>_CLASS_NAV			0x01
#define <DEVNAME>_CLASS_CFG			0x06
#define <DEVNAME>_CLASS_ACK			0x05

/* NAV message IDs */
#define <DEVNAME>_NAV_PVT			0x07
#define <DEVNAME>_NAV_SAT			0x35
#define <DEVNAME>_NAV_TIMEUTC			0x21

/* ACK/NACK message IDs */
#define <DEVNAME>_ACK_ACK			0x01
#define <DEVNAME>_ACK_NACK			0x00

/* Configuration keys */
#define <DEVNAME>_CFG_UART1_BAUDRATE		0x40520001
#define <DEVNAME>_CFG_RATE_MEAS		0x30210001
#define <DEVNAME>_CFG_NAVSPG_DYNMODEL		0x20110021
#define <DEVNAME>_CFG_TP_FREQ_TP1		0x40050024
#define <DEVNAME>_CFG_TP_TP1_ENA		0x10050007

/* GNSS system enable keys */
#define <DEVNAME>_CFG_SIGNAL_GPS_ENA		0x10310001
#define <DEVNAME>_CFG_SIGNAL_GLO_ENA		0x10310004
#define <DEVNAME>_CFG_SIGNAL_GAL_ENA		0x10310007
#define <DEVNAME>_CFG_SIGNAL_BDS_ENA		0x1031000D

/* Navigation model values */
#define <DEVNAME>_DYNMODEL_PORTABLE		0
#define <DEVNAME>_DYNMODEL_STATIONARY		2
#define <DEVNAME>_DYNMODEL_PEDESTRIAN		3
#define <DEVNAME>_DYNMODEL_AUTOMOTIVE		4
#define <DEVNAME>_DYNMODEL_SEA			5
#define <DEVNAME>_DYNMODEL_AIRBORNE_1G		6

/* Configuration layers */
#define <DEVNAME>_CFG_LAYER_RAM			0x01
#define <DEVNAME>_CFG_LAYER_BBR			0x02
#define <DEVNAME>_CFG_LAYER_FLASH		0x04

#endif /* ZEPHYR_DRIVERS_GNSS_<DEVNAME>_H_ */
```

---

## 7. Driver Source (`drivers/gnss/gnss_<devname>.c`)

This is the core of the driver. It follows the Zephyr GNSS subsystem
contract, uses UART interrupt-driven I/O for receiving NMEA sentences,
and dispatches parsed navigation data via the GNSS callback system.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gnss/gnss_publish.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/net/buf.h>

#include <string.h>
#include <stdlib.h>

/* NMEA 0183 parsing support */
#include <zephyr/drivers/gnss/gnss_nmea0183.h>

LOG_MODULE_REGISTER(gnss_<devname>, CONFIG_GNSS_LOG_LEVEL);

/* ------------------------------------------------------------------ */
/* Protocol Constants                                                 */
/* ------------------------------------------------------------------ */

#define <DEVNAME>_NMEA_MAX_LEN			82
#define <DEVNAME>_UART_RX_BUF_SIZE		256
#define <DEVNAME>_UART_TX_BUF_SIZE		128

#define <DEVNAME>_NMEA_START			'$'
#define <DEVNAME>_NMEA_END_CR			'\r'
#define <DEVNAME>_NMEA_END_LF			'\n'

/* Vendor binary protocol constants (if applicable) */
#define <DEVNAME>_SYNC_1			0xB5
#define <DEVNAME>_SYNC_2			0x62
#define <DEVNAME>_HEADER_SIZE			6
#define <DEVNAME>_CHECKSUM_SIZE			2
#define <DEVNAME>_MAX_PACKET_SIZE		256

/* Message classes */
#define <DEVNAME>_CLASS_NAV			0x01
#define <DEVNAME>_CLASS_CFG			0x06
#define <DEVNAME>_CLASS_ACK			0x05

/* NAV message IDs */
#define <DEVNAME>_NAV_PVT			0x07

/* Dynamic model values for navigation mode mapping */
#define <DEVNAME>_DYNMODEL_PORTABLE		0
#define <DEVNAME>_DYNMODEL_STATIONARY		2
#define <DEVNAME>_DYNMODEL_PEDESTRIAN		3
#define <DEVNAME>_DYNMODEL_AUTOMOTIVE		4
#define <DEVNAME>_DYNMODEL_SEA			5
#define <DEVNAME>_DYNMODEL_AIRBORNE_1G		6

/* Configuration keys */
#define <DEVNAME>_CFG_RATE_MEAS		0x30210001
#define <DEVNAME>_CFG_NAVSPG_DYNMODEL		0x20110021
#define <DEVNAME>_CFG_SIGNAL_GPS_ENA		0x10310001
#define <DEVNAME>_CFG_SIGNAL_GLO_ENA		0x10310004
#define <DEVNAME>_CFG_SIGNAL_GAL_ENA		0x10310007
#define <DEVNAME>_CFG_SIGNAL_BDS_ENA		0x1031000D
#define <DEVNAME>_CFG_LAYER_RAM		0x01

/* Reset delay after hardware reset (ms). */
#define <DEVNAME>_RESET_DELAY_MS		1000

/* Work queue stack size for NMEA processing */
#define <DEVNAME>_WORK_STACK_SIZE		2048

/* ------------------------------------------------------------------ */
/* Config & Data Structs                                              */
/* ------------------------------------------------------------------ */

/**
 * Compile-time configuration from devicetree.
 * This struct is const and stored in ROM.
 */
struct <devname>_config {
	const struct device *uart_dev;
	uint32_t uart_baudrate;
	struct gpio_dt_spec pps_gpio;
	struct gpio_dt_spec reset_gpio;
	uint32_t pps_frequency;
	uint32_t pps_pulse_length_ms;
};

/**
 * NMEA receive state machine states.
 */
enum <devname>_rx_state {
	<DEVNAME>_RX_IDLE,
	<DEVNAME>_RX_NMEA,
	<DEVNAME>_RX_BINARY_SYNC2,
	<DEVNAME>_RX_BINARY_HEADER,
	<DEVNAME>_RX_BINARY_PAYLOAD,
	<DEVNAME>_RX_BINARY_CHECKSUM,
};

/**
 * Mutable runtime data.
 * This struct is allocated statically per instance.
 */
struct <devname>_data {
	const struct device *dev;

	/* UART receive state */
	enum <devname>_rx_state rx_state;
	uint8_t nmea_buf[<DEVNAME>_NMEA_MAX_LEN + 1];
	uint16_t nmea_len;
	uint8_t binary_buf[<DEVNAME>_MAX_PACKET_SIZE];
	uint16_t binary_len;
	uint16_t binary_expected;

	/* GNSS NMEA 0183 match data */
	struct gnss_nmea0183_match_data match_data;

	/* Work item for processing received sentences */
	struct k_work nmea_work;

	/* Cached navigation data */
	struct gnss_data gnss_data;

#if CONFIG_GNSS_SATELLITES
	struct gnss_satellite satellites[CONFIG_GNSS_NMEA0183_MATCH_MAX_SATELLITES];
	uint16_t num_satellites;
#endif

	/* Fix rate in milliseconds */
	uint32_t fix_rate_ms;

	/* Current navigation mode */
	enum gnss_navigation_mode nav_mode;

	/* Enabled GNSS systems bitmask */
	gnss_systems_t enabled_systems;

	/* Mutex for protecting shared state */
	struct k_mutex lock;
};

/* ------------------------------------------------------------------ */
/* Forward Declarations                                               */
/* ------------------------------------------------------------------ */

static void <devname>_uart_isr(const struct device *uart_dev,
			       void *user_data);
static void <devname>_nmea_work_handler(struct k_work *work);

/* ------------------------------------------------------------------ */
/* UART Communication Helpers                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief Send raw data to the GNSS receiver over UART.
 *
 * @param dev  Driver device instance.
 * @param data Buffer to send.
 * @param len  Number of bytes to send.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_uart_send(const struct device *dev,
			       const uint8_t *data, size_t len)
{
	const struct <devname>_config *cfg = dev->config;

	for (size_t i = 0; i < len; i++) {
		uart_poll_out(cfg->uart_dev, data[i]);
	}

	return 0;
}

/**
 * @brief Calculate Fletcher-8 checksum over binary protocol data.
 *
 * @param data   Data buffer (class, id, length, payload).
 * @param length Number of bytes to checksum.
 * @param ck_a   Output checksum byte A.
 * @param ck_b   Output checksum byte B.
 */
static void <devname>_calc_checksum(const uint8_t *data, uint16_t length,
				    uint8_t *ck_a, uint8_t *ck_b)
{
	uint8_t a = 0;
	uint8_t b = 0;

	for (uint16_t i = 0; i < length; i++) {
		a += data[i];
		b += a;
	}

	*ck_a = a;
	*ck_b = b;
}

/**
 * @brief Send a vendor binary protocol packet.
 *
 * Builds a complete packet with sync bytes, header, payload, and
 * checksum, then transmits it over UART.
 *
 * @param dev     Driver device instance.
 * @param cls     Message class.
 * @param id      Message ID.
 * @param payload Payload buffer (may be NULL if length is 0).
 * @param length  Payload length in bytes.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_send_binary(const struct device *dev, uint8_t cls,
				 uint8_t id, const uint8_t *payload,
				 uint16_t length)
{
	uint8_t header[<DEVNAME>_HEADER_SIZE];
	uint8_t ck_a, ck_b;
	int ret;

	if (length > (<DEVNAME>_MAX_PACKET_SIZE -
		      <DEVNAME>_HEADER_SIZE - <DEVNAME>_CHECKSUM_SIZE)) {
		return -ERANGE;
	}

	/* Build header */
	header[0] = <DEVNAME>_SYNC_1;
	header[1] = <DEVNAME>_SYNC_2;
	header[2] = cls;
	header[3] = id;
	header[4] = length & 0xFF;
	header[5] = (length >> 8) & 0xFF;

	/* Send header */
	ret = <devname>_uart_send(dev, header, sizeof(header));
	if (ret < 0) {
		return ret;
	}

	/* Send payload */
	if (payload && length > 0) {
		ret = <devname>_uart_send(dev, payload, length);
		if (ret < 0) {
			return ret;
		}
	}

	/* Calculate checksum over class + id + length + payload */
	<devname>_calc_checksum(&header[2], 4, &ck_a, &ck_b);
	if (payload && length > 0) {
		for (uint16_t i = 0; i < length; i++) {
			ck_a += payload[i];
			ck_b += ck_a;
		}
	}

	/* Send checksum */
	uint8_t ck[2] = { ck_a, ck_b };

	return <devname>_uart_send(dev, ck, sizeof(ck));
}

/**
 * @brief Send a configuration key-value pair using vendor binary protocol.
 *
 * @param dev   Driver device instance.
 * @param key   Configuration key ID (32-bit).
 * @param value Configuration value.
 * @param size  Value size in bytes (1, 2, or 4).
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_set_config_val(const struct device *dev,
				    uint32_t key, uint32_t value,
				    uint8_t size)
{
	/*
	 * Build a VALSET message:
	 *   version (1) + layers (1) + reserved (2) + key (4) + value (1-4)
	 */
	uint8_t payload[12];
	uint16_t len = 0;

	/* Version */
	payload[len++] = 0x00;
	/* Layers: RAM only */
	payload[len++] = <DEVNAME>_CFG_LAYER_RAM;
	/* Reserved */
	payload[len++] = 0x00;
	payload[len++] = 0x00;

	/* Key (little-endian) */
	sys_put_le32(key, &payload[len]);
	len += 4;

	/* Value (little-endian, variable size) */
	switch (size) {
	case 1:
		payload[len++] = (uint8_t)value;
		break;
	case 2:
		sys_put_le16((uint16_t)value, &payload[len]);
		len += 2;
		break;
	case 4:
		sys_put_le32(value, &payload[len]);
		len += 4;
		break;
	default:
		return -EINVAL;
	}

	return <devname>_send_binary(dev, <DEVNAME>_CLASS_CFG, 0x8A,
				     payload, len);
}

/* ------------------------------------------------------------------ */
/* NMEA Sentence Processing                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief Process a complete NMEA sentence.
 *
 * Called from the work queue context when a full NMEA sentence has
 * been received. Dispatches the sentence to the NMEA 0183 match
 * system for parsing.
 *
 * @param dev      Driver device instance.
 * @param sentence Null-terminated NMEA sentence string.
 */
static void <devname>_process_nmea(const struct device *dev,
				   const char *sentence)
{
	struct <devname>_data *data = dev->data;

	LOG_DBG("NMEA: %s", sentence);

	/*
	 * Use the Zephyr GNSS NMEA 0183 match system to dispatch
	 * the sentence. This system parses GGA, RMC, GSV, and GSA
	 * sentences and populates gnss_data / gnss_satellite structs.
	 */
	gnss_nmea0183_match_line(&data->match_data, sentence);
}

/**
 * @brief NMEA work queue handler.
 *
 * Processes buffered NMEA sentences outside ISR context.
 */
static void <devname>_nmea_work_handler(struct k_work *work)
{
	struct <devname>_data *data =
		CONTAINER_OF(work, struct <devname>_data, nmea_work);
	const struct device *dev = data->dev;

	k_mutex_lock(&data->lock, K_FOREVER);

	/*
	 * Null-terminate and process the NMEA sentence.
	 * The buffer was filled by the ISR.
	 */
	if (data->nmea_len > 0) {
		data->nmea_buf[data->nmea_len] = '\0';
		<devname>_process_nmea(dev, (const char *)data->nmea_buf);
		data->nmea_len = 0;
	}

	k_mutex_unlock(&data->lock);
}

/* ------------------------------------------------------------------ */
/* NMEA 0183 Match Callbacks                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief Callback invoked when a GGA sentence is parsed.
 *
 * Populates the navigation data structure with position, altitude,
 * fix quality, and satellite count from the GGA sentence.
 */
static void <devname>_gga_handler(struct gnss_nmea0183_match_data *match,
				  const char *sentence)
{
	struct <devname>_data *data =
		CONTAINER_OF(match, struct <devname>_data, match_data);
	const struct device *dev = data->dev;
	struct gnss_data gnss_data = { 0 };
	int ret;

	ret = gnss_nmea0183_parse_gga(sentence, &gnss_data.nav_data);
	if (ret < 0) {
		LOG_WRN("GGA parse failed: %d", ret);
		return;
	}

	gnss_data.info.fix_status = gnss_data.nav_data.fix_quality > 0
		? GNSS_FIX_STATUS_GNSS_FIX
		: GNSS_FIX_STATUS_NO_FIX;
	gnss_data.info.fix_timestamp = k_uptime_get();

	k_mutex_lock(&data->lock, K_FOREVER);
	data->gnss_data.nav_data = gnss_data.nav_data;
	data->gnss_data.info.fix_status = gnss_data.info.fix_status;
	data->gnss_data.info.fix_timestamp = gnss_data.info.fix_timestamp;
	k_mutex_unlock(&data->lock);

	gnss_publish_data(dev, &gnss_data);
}

/**
 * @brief Callback invoked when an RMC sentence is parsed.
 *
 * Populates the navigation data structure with time, date, speed,
 * and heading from the RMC sentence.
 */
static void <devname>_rmc_handler(struct gnss_nmea0183_match_data *match,
				  const char *sentence)
{
	struct <devname>_data *data =
		CONTAINER_OF(match, struct <devname>_data, match_data);
	struct gnss_data gnss_data = { 0 };
	int ret;

	ret = gnss_nmea0183_parse_rmc(sentence, &gnss_data.nav_data);
	if (ret < 0) {
		LOG_WRN("RMC parse failed: %d", ret);
		return;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->gnss_data.nav_data.utc = gnss_data.nav_data.utc;
	data->gnss_data.nav_data.speed = gnss_data.nav_data.speed;
	data->gnss_data.nav_data.bearing = gnss_data.nav_data.bearing;
	k_mutex_unlock(&data->lock);
}

#if CONFIG_GNSS_SATELLITES
/**
 * @brief Callback invoked when a GSV sentence is parsed.
 *
 * Populates per-satellite information (PRN, elevation, azimuth, SNR)
 * from the GSV sentence and publishes when the last GSV message in
 * a sequence is received.
 */
static void <devname>_gsv_handler(struct gnss_nmea0183_match_data *match,
				  const char *sentence)
{
	struct <devname>_data *data =
		CONTAINER_OF(match, struct <devname>_data, match_data);
	const struct device *dev = data->dev;
	int ret;
	bool is_last;

	ret = gnss_nmea0183_parse_gsv(sentence,
				      data->satellites,
				      ARRAY_SIZE(data->satellites),
				      &data->num_satellites,
				      &is_last);
	if (ret < 0) {
		LOG_WRN("GSV parse failed: %d", ret);
		return;
	}

	if (is_last) {
		gnss_publish_satellites(dev, data->satellites,
				       data->num_satellites);
	}
}
#endif /* CONFIG_GNSS_SATELLITES */

/*
 * Register NMEA sentence match callbacks.
 * This macro-based system dispatches individual sentence types
 * to their respective handler functions.
 */
GNSS_NMEA0183_MATCH_CALLBACK_DEFINE(gnss_<devname>_gga,
				    "$??GGA", <devname>_gga_handler);
GNSS_NMEA0183_MATCH_CALLBACK_DEFINE(gnss_<devname>_rmc,
				    "$??RMC", <devname>_rmc_handler);
#if CONFIG_GNSS_SATELLITES
GNSS_NMEA0183_MATCH_CALLBACK_DEFINE(gnss_<devname>_gsv,
				    "$??GSV", <devname>_gsv_handler);
#endif

/* ------------------------------------------------------------------ */
/* UART ISR and Receive State Machine                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief UART interrupt service routine.
 *
 * Receives characters one at a time and assembles complete NMEA
 * sentences. When a full sentence is detected (terminated by CR+LF),
 * it submits a work item for processing outside ISR context.
 *
 * For devices supporting a vendor binary protocol, the ISR also
 * detects binary sync bytes and routes to binary packet assembly.
 */
static void <devname>_uart_isr(const struct device *uart_dev,
			       void *user_data)
{
	const struct device *dev = user_data;
	struct <devname>_data *data = dev->data;
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		switch (data->rx_state) {
		case <DEVNAME>_RX_IDLE:
			if (c == <DEVNAME>_NMEA_START) {
				/* Start of NMEA sentence */
				data->nmea_buf[0] = c;
				data->nmea_len = 1;
				data->rx_state = <DEVNAME>_RX_NMEA;
			} else if (c == <DEVNAME>_SYNC_1) {
				/* Possible binary packet sync */
				data->binary_buf[0] = c;
				data->binary_len = 1;
				data->rx_state = <DEVNAME>_RX_BINARY_SYNC2;
			}
			break;

		case <DEVNAME>_RX_NMEA:
			if (data->nmea_len < <DEVNAME>_NMEA_MAX_LEN) {
				data->nmea_buf[data->nmea_len++] = c;
			}

			if (c == <DEVNAME>_NMEA_END_LF) {
				/*
				 * Complete NMEA sentence received.
				 * Strip CR+LF and submit for processing.
				 */
				if (data->nmea_len >= 2) {
					data->nmea_len -= 2;
				}
				data->nmea_buf[data->nmea_len] = '\0';
				k_work_submit(&data->nmea_work);
				data->rx_state = <DEVNAME>_RX_IDLE;
			} else if (data->nmea_len >= <DEVNAME>_NMEA_MAX_LEN) {
				/* Sentence too long, discard */
				LOG_WRN("NMEA sentence overflow, discarding");
				data->nmea_len = 0;
				data->rx_state = <DEVNAME>_RX_IDLE;
			}
			break;

		case <DEVNAME>_RX_BINARY_SYNC2:
			if (c == <DEVNAME>_SYNC_2) {
				data->binary_buf[data->binary_len++] = c;
				data->rx_state = <DEVNAME>_RX_BINARY_HEADER;
			} else {
				/* Not a valid binary packet */
				data->binary_len = 0;
				data->rx_state = <DEVNAME>_RX_IDLE;
			}
			break;

		case <DEVNAME>_RX_BINARY_HEADER:
			data->binary_buf[data->binary_len++] = c;
			if (data->binary_len >= <DEVNAME>_HEADER_SIZE) {
				/* Header complete, extract payload length */
				data->binary_expected =
					sys_get_le16(&data->binary_buf[4]);
				if (data->binary_expected >
				    (<DEVNAME>_MAX_PACKET_SIZE -
				     <DEVNAME>_HEADER_SIZE -
				     <DEVNAME>_CHECKSUM_SIZE)) {
					LOG_WRN("Binary packet too large: %u",
						data->binary_expected);
					data->binary_len = 0;
					data->rx_state = <DEVNAME>_RX_IDLE;
				} else {
					data->rx_state =
						<DEVNAME>_RX_BINARY_PAYLOAD;
				}
			}
			break;

		case <DEVNAME>_RX_BINARY_PAYLOAD:
			data->binary_buf[data->binary_len++] = c;
			if (data->binary_len >=
			    <DEVNAME>_HEADER_SIZE + data->binary_expected) {
				data->rx_state = <DEVNAME>_RX_BINARY_CHECKSUM;
			}
			break;

		case <DEVNAME>_RX_BINARY_CHECKSUM:
			data->binary_buf[data->binary_len++] = c;
			if (data->binary_len >=
			    <DEVNAME>_HEADER_SIZE +
			    data->binary_expected +
			    <DEVNAME>_CHECKSUM_SIZE) {
				/*
				 * Full binary packet received.
				 * Verify checksum and process.
				 * TODO: Submit to work queue for processing.
				 */
				uint8_t ck_a, ck_b;

				<devname>_calc_checksum(
					&data->binary_buf[2],
					4 + data->binary_expected,
					&ck_a, &ck_b);
				uint16_t ck_offset =
					<DEVNAME>_HEADER_SIZE +
					data->binary_expected;
				if (ck_a == data->binary_buf[ck_offset] &&
				    ck_b == data->binary_buf[ck_offset + 1]) {
					LOG_DBG("Valid binary packet: "
						"cls=0x%02X id=0x%02X "
						"len=%u",
						data->binary_buf[2],
						data->binary_buf[3],
						data->binary_expected);
				} else {
					LOG_WRN("Binary checksum mismatch");
				}
				data->binary_len = 0;
				data->rx_state = <DEVNAME>_RX_IDLE;
			}
			break;
		}
	}
}

/* ------------------------------------------------------------------ */
/* GNSS Subsystem API Implementation                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief Set the position fix rate.
 *
 * Configures how often the receiver computes a navigation solution.
 *
 * @param dev     Driver device instance.
 * @param fix_rate Fix rate in milliseconds (e.g., 1000 for 1 Hz).
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_set_fix_rate(const struct device *dev,
				  uint32_t fix_rate)
{
	struct <devname>_data *data = dev->data;
	int ret;

	if (fix_rate == 0) {
		return -EINVAL;
	}

	/*
	 * Send configuration command to the receiver.
	 * For binary-capable devices, use the vendor config protocol.
	 * For NMEA-only devices, use proprietary NMEA commands.
	 */
	ret = <devname>_set_config_val(dev, <DEVNAME>_CFG_RATE_MEAS,
				       fix_rate, 2);
	if (ret < 0) {
		LOG_ERR("Failed to set fix rate: %d", ret);
		return ret;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->fix_rate_ms = fix_rate;
	k_mutex_unlock(&data->lock);

	return 0;
}

/**
 * @brief Get the current position fix rate.
 *
 * @param dev      Driver device instance.
 * @param fix_rate Pointer to store the fix rate in milliseconds.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_get_fix_rate(const struct device *dev,
				  uint32_t *fix_rate)
{
	struct <devname>_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	*fix_rate = data->fix_rate_ms;
	k_mutex_unlock(&data->lock);

	return 0;
}

/**
 * @brief Map Zephyr navigation mode to vendor dynamic model.
 */
static int <devname>_nav_mode_to_dynmodel(enum gnss_navigation_mode mode)
{
	switch (mode) {
	case GNSS_NAVIGATION_MODE_ZERO_DYNAMICS:
		return <DEVNAME>_DYNMODEL_STATIONARY;
	case GNSS_NAVIGATION_MODE_LOW_DYNAMICS:
		return <DEVNAME>_DYNMODEL_PEDESTRIAN;
	case GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS:
		return <DEVNAME>_DYNMODEL_AUTOMOTIVE;
	case GNSS_NAVIGATION_MODE_HIGH_DYNAMICS:
		return <DEVNAME>_DYNMODEL_AIRBORNE_1G;
	default:
		return -EINVAL;
	}
}

/**
 * @brief Set the navigation mode (dynamic model).
 *
 * The navigation mode controls the receiver's motion model assumptions,
 * affecting filter behaviour, fix quality, and power consumption.
 *
 * @param dev  Driver device instance.
 * @param mode Navigation mode.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_set_navigation_mode(const struct device *dev,
					 enum gnss_navigation_mode mode)
{
	struct <devname>_data *data = dev->data;
	int dynmodel;
	int ret;

	dynmodel = <devname>_nav_mode_to_dynmodel(mode);
	if (dynmodel < 0) {
		return dynmodel;
	}

	ret = <devname>_set_config_val(dev, <DEVNAME>_CFG_NAVSPG_DYNMODEL,
				       (uint32_t)dynmodel, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set navigation mode: %d", ret);
		return ret;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->nav_mode = mode;
	k_mutex_unlock(&data->lock);

	return 0;
}

/**
 * @brief Get the current navigation mode.
 *
 * @param dev  Driver device instance.
 * @param mode Pointer to store the navigation mode.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_get_navigation_mode(const struct device *dev,
					 enum gnss_navigation_mode *mode)
{
	struct <devname>_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	*mode = data->nav_mode;
	k_mutex_unlock(&data->lock);

	return 0;
}

/**
 * @brief Set enabled GNSS constellation systems.
 *
 * Controls which satellite constellations the receiver uses for
 * position fixes (GPS, GLONASS, Galileo, BeiDou).
 *
 * @param dev     Driver device instance.
 * @param systems Bitmask of GNSS systems to enable.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_set_enabled_systems(const struct device *dev,
					 gnss_systems_t systems)
{
	struct <devname>_data *data = dev->data;
	int ret;

	/* Configure each constellation independently */
	ret = <devname>_set_config_val(dev, <DEVNAME>_CFG_SIGNAL_GPS_ENA,
				       (systems & GNSS_SYSTEM_GPS) ? 1 : 0,
				       1);
	if (ret < 0) {
		return ret;
	}

	ret = <devname>_set_config_val(dev, <DEVNAME>_CFG_SIGNAL_GLO_ENA,
				       (systems & GNSS_SYSTEM_GLONASS) ? 1 : 0,
				       1);
	if (ret < 0) {
		return ret;
	}

	ret = <devname>_set_config_val(dev, <DEVNAME>_CFG_SIGNAL_GAL_ENA,
				       (systems & GNSS_SYSTEM_GALILEO) ? 1 : 0,
				       1);
	if (ret < 0) {
		return ret;
	}

	ret = <devname>_set_config_val(dev, <DEVNAME>_CFG_SIGNAL_BDS_ENA,
				       (systems & GNSS_SYSTEM_BEIDOU) ? 1 : 0,
				       1);
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->enabled_systems = systems;
	k_mutex_unlock(&data->lock);

	return 0;
}

/**
 * @brief Get currently enabled GNSS systems.
 *
 * @param dev     Driver device instance.
 * @param systems Pointer to store the systems bitmask.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_get_enabled_systems(const struct device *dev,
					 gnss_systems_t *systems)
{
	struct <devname>_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	*systems = data->enabled_systems;
	k_mutex_unlock(&data->lock);

	return 0;
}

/**
 * @brief Get supported GNSS constellation systems.
 *
 * Returns a bitmask of all constellations the hardware supports.
 * This is a static capability of the receiver and does not change.
 *
 * @param dev     Driver device instance.
 * @param systems Pointer to store the supported systems bitmask.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_get_supported_systems(const struct device *dev,
					   gnss_systems_t *systems)
{
	ARG_UNUSED(dev);

	/*
	 * Return the full set of constellations supported by the
	 * hardware. Adjust this bitmask based on the actual receiver
	 * capabilities.
	 */
	*systems = GNSS_SYSTEM_GPS |
		   GNSS_SYSTEM_GLONASS |
		   GNSS_SYSTEM_GALILEO |
		   GNSS_SYSTEM_BEIDOU;

	return 0;
}

/* ------------------------------------------------------------------ */
/* Hardware Reset                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief Perform hardware reset via GPIO.
 *
 * Asserts the reset pin low, waits, then releases. The receiver
 * requires a stabilization delay after reset before accepting
 * commands.
 *
 * @param dev Driver device instance.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_hw_reset(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	int ret;

	if (cfg->reset_gpio.port == NULL) {
		return 0; /* No reset GPIO configured */
	}

	ret = gpio_pin_set_dt(&cfg->reset_gpio, 1); /* Assert reset */
	if (ret < 0) {
		return ret;
	}

	k_msleep(100);

	ret = gpio_pin_set_dt(&cfg->reset_gpio, 0); /* Release reset */
	if (ret < 0) {
		return ret;
	}

	k_msleep(<DEVNAME>_RESET_DELAY_MS);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Initialization                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief Device initialization function.
 *
 * Called automatically at boot for each DT instance. Sets up UART
 * communication with interrupt-driven receive, configures optional
 * GPIO pins (PPS, reset), and initializes the NMEA parsing state
 * machine.
 */
static int <devname>_init(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	data->dev = dev;

	/* Initialize mutex */
	k_mutex_init(&data->lock);

	/* Verify UART bus is ready */
	if (!device_is_ready(cfg->uart_dev)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	/* Initialize NMEA receive state machine */
	data->rx_state = <DEVNAME>_RX_IDLE;
	data->nmea_len = 0;
	data->binary_len = 0;

	/* Set defaults */
	data->fix_rate_ms = 1000; /* 1 Hz */
	data->nav_mode = GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS;
	data->enabled_systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS |
				GNSS_SYSTEM_GALILEO | GNSS_SYSTEM_BEIDOU;

	/* Initialize work item for NMEA processing */
	k_work_init(&data->nmea_work, <devname>_nmea_work_handler);

	/* Configure optional PPS GPIO as input */
	if (cfg->pps_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->pps_gpio)) {
			LOG_ERR("PPS GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->pps_gpio, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Failed to configure PPS GPIO: %d", ret);
			return ret;
		}
	}

	/* Configure optional hardware reset GPIO */
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

		/* Perform hardware reset */
		ret = <devname>_hw_reset(dev);
		if (ret < 0) {
			LOG_WRN("Hardware reset failed: %d (continuing)", ret);
		}
	}

	/* Set up UART interrupt-driven receive */
	uart_irq_callback_user_data_set(cfg->uart_dev,
					<devname>_uart_isr,
					(void *)dev);
	uart_irq_rx_enable(cfg->uart_dev);

	LOG_INF("<DEVNAME> GNSS receiver initialized on %s",
		cfg->uart_dev->name);

	return 0;
}

/* ------------------------------------------------------------------ */
/* API Struct                                                         */
/* ------------------------------------------------------------------ */

/*
 * Use the DEVICE_API macro. This creates a static const struct that
 * the Zephyr GNSS subsystem uses to dispatch calls.
 */
static DEVICE_API(gnss, <devname>_api) = {
	.set_fix_rate = <devname>_set_fix_rate,
	.get_fix_rate = <devname>_get_fix_rate,
	.set_navigation_mode = <devname>_set_navigation_mode,
	.get_navigation_mode = <devname>_get_navigation_mode,
	.set_enabled_systems = <devname>_set_enabled_systems,
	.get_enabled_systems = <devname>_get_enabled_systems,
	.get_supported_systems = <devname>_get_supported_systems,
};

/* ------------------------------------------------------------------ */
/* Instance Macros                                                    */
/* ------------------------------------------------------------------ */

/*
 * These macros are expanded once per DT instance with status "okay".
 * They create the config, data, and DEVICE_DT_INST_DEFINE entries.
 *
 * GNSS devices use UART, so the config references the parent UART
 * bus device from devicetree rather than an SPI or I2C spec.
 */

#define <DEVNAME>_PPS_GPIO_INIT(n)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pps_gpios),		\
		(.pps_gpio = GPIO_DT_SPEC_INST_GET(n, pps_gpios),),	\
		(.pps_gpio = { 0 },))

#define <DEVNAME>_RESET_GPIO_INIT(n)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, reset_gpios),		\
		(.reset_gpio = GPIO_DT_SPEC_INST_GET(n, reset_gpios),),\
		(.reset_gpio = { 0 },))

#define <DEVNAME>_INIT(n)						\
	static struct <devname>_data <devname>_data_##n;		\
									\
	static const struct <devname>_config <devname>_config_##n = {	\
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(n)),		\
		.uart_baudrate = DT_INST_PROP(n, current_speed),	\
		<DEVNAME>_PPS_GPIO_INIT(n)				\
		<DEVNAME>_RESET_GPIO_INIT(n)				\
		.pps_frequency = DT_INST_PROP_OR(n,			\
					pps_frequency, 1),		\
		.pps_pulse_length_ms = DT_INST_PROP_OR(n,		\
					pps_pulse_length_ms, 100),	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, <devname>_init, NULL,			\
			      &<devname>_data_##n,			\
			      &<devname>_config_##n,			\
			      POST_KERNEL,				\
			      CONFIG_GNSS_INIT_PRIORITY,		\
			      &<devname>_api);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_INIT)
```

### Key elements explained

| Element | Purpose |
|---------|---------|
| `DT_DRV_COMPAT` | Must match the compatible string with commas replaced by underscores |
| `struct <devname>_config` | Compile-time config from DT (stored in ROM) |
| `struct <devname>_data` | Mutable runtime state (one per instance) |
| `DEVICE_API(gnss, ...)` | Typed API struct -- replaces an untyped `struct gnss_driver_api` literal |
| `DEVICE_DT_GET(DT_INST_BUS(n))` | Gets the parent UART device handle from DT |
| `GPIO_DT_SPEC_INST_GET()` | Pulls GPIO port/pin/flags from DT |
| `DEVICE_DT_INST_DEFINE()` | Registers the device with Zephyr's device model |
| `DT_INST_FOREACH_STATUS_OKAY()` | Instantiates one driver per DT node with `status = "okay"` |
| `LOG_MODULE_REGISTER()` | Creates a logging module; level controlled by `CONFIG_GNSS_LOG_LEVEL` |
| `gnss_publish_data()` | Publishes navigation data to all registered callbacks |
| `gnss_publish_satellites()` | Publishes satellite info to all registered callbacks |
| `GNSS_NMEA0183_MATCH_CALLBACK_DEFINE()` | Registers handlers for specific NMEA sentence types |

---

## 8. PPS Interrupt Support (Optional)

If the receiver has a PPS output, implement a GPIO callback to capture
precise timing edges for synchronization applications.

```c
/* Add to data struct: */
struct <devname>_data {
	/* ... existing fields ... */
	struct gpio_callback pps_cb;
	uint64_t pps_timestamp_us;
	struct k_sem pps_sem;
};

/* PPS interrupt handler: */
static void <devname>_pps_handler(const struct device *port,
				  struct gpio_callback *cb,
				  gpio_port_pins_t pins)
{
	struct <devname>_data *data =
		CONTAINER_OF(cb, struct <devname>_data, pps_cb);

	data->pps_timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks());
	k_sem_give(&data->pps_sem);
}

/* In init(), set up the PPS interrupt: */
static int <devname>_init_pps(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	int ret;

	if (cfg->pps_gpio.port == NULL) {
		return 0; /* No PPS GPIO configured */
	}

	k_sem_init(&data->pps_sem, 0, 1);

	gpio_init_callback(&data->pps_cb, <devname>_pps_handler,
			   BIT(cfg->pps_gpio.pin));

	ret = gpio_add_callback(cfg->pps_gpio.port, &data->pps_cb);
	if (ret < 0) {
		return ret;
	}

	return gpio_pin_interrupt_configure_dt(&cfg->pps_gpio,
					       GPIO_INT_EDGE_TO_ACTIVE);
}

/**
 * @brief Wait for the next PPS pulse.
 *
 * Blocks until a PPS rising edge is detected or timeout expires.
 *
 * @param dev     Driver device instance.
 * @param timeout Maximum wait time.
 * @return PPS timestamp in microseconds, or negative errno on failure.
 */
static int64_t <devname>_wait_pps(const struct device *dev,
				  k_timeout_t timeout)
{
	struct <devname>_data *data = dev->data;
	int ret;

	ret = k_sem_take(&data->pps_sem, timeout);
	if (ret < 0) {
		return ret;
	}

	return (int64_t)data->pps_timestamp_us;
}
```

---

## 9. Test Skeleton

### 9.1 Test Metadata (`tests/drivers/gnss/<devname>/testcase.yaml`)

```yaml
tests:
  drivers.gnss.<devname>:
    tags:
      - drivers
      - gnss
    depends_on: uart
    platform_allow: native_sim
    integration_platforms:
      - native_sim
```

### 9.2 Test Project Config (`tests/drivers/gnss/<devname>/prj.conf`)

```
CONFIG_ZTEST=y
CONFIG_GNSS=y
CONFIG_GNSS_<DEVNAME>=y
CONFIG_GNSS_NMEA0183=y
CONFIG_GNSS_NMEA0183_MATCH=y
CONFIG_GNSS_SATELLITES=y
CONFIG_UART_INTERRUPT_DRIVEN=y
CONFIG_LOG=y
```

### 9.3 DT Overlay (`tests/drivers/gnss/<devname>/boards/native_sim.overlay`)

```dts
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

&uart1 {
	status = "okay";
	current-speed = <9600>;

	<devname>_test: <devname> {
		compatible = "adi,<devname>";
		current-speed = <9600>;
	};
};
```

### 9.4 Test Source (`tests/drivers/gnss/<devname>/src/main.c`)

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/device.h>

#define GNSS_NODE DT_NODELABEL(<devname>_test)
static const struct device *gnss_dev = DEVICE_DT_GET(GNSS_NODE);

static K_SEM_DEFINE(data_ready_sem, 0, 1);
static struct gnss_data received_data;

/* Data callback to receive navigation fixes */
static void test_gnss_data_cb(const struct device *dev,
			      const struct gnss_data *data)
{
	received_data = *data;
	k_sem_give(&data_ready_sem);
}

GNSS_DATA_CALLBACK_DEFINE(DEVICE_DT_GET(GNSS_NODE), test_gnss_data_cb);

ZTEST(gnss_<devname>, test_device_ready)
{
	zassert_true(device_is_ready(gnss_dev),
		     "GNSS device not ready");
}

ZTEST(gnss_<devname>, test_get_supported_systems)
{
	gnss_systems_t systems;
	int ret;

	ret = gnss_get_supported_systems(gnss_dev, &systems);
	zassert_ok(ret, "gnss_get_supported_systems failed: %d", ret);
	zassert_true(systems & GNSS_SYSTEM_GPS,
		     "GPS should be supported");
}

ZTEST(gnss_<devname>, test_set_fix_rate)
{
	uint32_t fix_rate;
	int ret;

	ret = gnss_set_fix_rate(gnss_dev, 1000);
	zassert_ok(ret, "gnss_set_fix_rate failed: %d", ret);

	ret = gnss_get_fix_rate(gnss_dev, &fix_rate);
	zassert_ok(ret, "gnss_get_fix_rate failed: %d", ret);
	zassert_equal(fix_rate, 1000,
		      "Expected 1000 ms fix rate, got %u", fix_rate);
}

ZTEST(gnss_<devname>, test_set_fix_rate_zero)
{
	int ret;

	ret = gnss_set_fix_rate(gnss_dev, 0);
	zassert_equal(ret, -EINVAL,
		      "Expected -EINVAL for zero fix rate, got %d", ret);
}

ZTEST(gnss_<devname>, test_set_navigation_mode)
{
	enum gnss_navigation_mode mode;
	int ret;

	ret = gnss_set_navigation_mode(gnss_dev,
				       GNSS_NAVIGATION_MODE_LOW_DYNAMICS);
	zassert_ok(ret, "gnss_set_navigation_mode failed: %d", ret);

	ret = gnss_get_navigation_mode(gnss_dev, &mode);
	zassert_ok(ret, "gnss_get_navigation_mode failed: %d", ret);
	zassert_equal(mode, GNSS_NAVIGATION_MODE_LOW_DYNAMICS,
		      "Expected LOW_DYNAMICS, got %d", mode);
}

ZTEST(gnss_<devname>, test_set_enabled_systems)
{
	gnss_systems_t systems;
	gnss_systems_t expected = GNSS_SYSTEM_GPS | GNSS_SYSTEM_GALILEO;
	int ret;

	ret = gnss_set_enabled_systems(gnss_dev, expected);
	zassert_ok(ret, "gnss_set_enabled_systems failed: %d", ret);

	ret = gnss_get_enabled_systems(gnss_dev, &systems);
	zassert_ok(ret, "gnss_get_enabled_systems failed: %d", ret);
	zassert_equal(systems, expected,
		      "Systems mismatch: 0x%X vs 0x%X",
		      systems, expected);
}

ZTEST_SUITE(gnss_<devname>, NULL, NULL, NULL, NULL, NULL);
```

### 9.5 UART Emulator Pattern (Optional)

For proper testing on `native_sim`, create a UART emulator that feeds
NMEA sentences to the driver:

```c
/*
 * tests/drivers/gnss/<devname>/src/<devname>_emul.c
 *
 * Minimal UART emulator for <DEVNAME> GNSS tests.
 * Sends pre-recorded NMEA sentences to exercise the driver's
 * parsing and callback logic.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

/* Example NMEA sentences for testing */
static const char *test_nmea_sentences[] = {
	"$GNGGA,120000.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,,,,*47\r\n",
	"$GNRMC,120000.00,A,4807.038,N,01131.000,E,0.0,0.0,150525,,,A*6D\r\n",
	"$GPGSV,3,1,12,01,40,083,41,02,17,308,42,03,58,207,47,04,12,348,38*78\r\n",
	NULL,
};

/**
 * @brief Feed test NMEA data to the GNSS driver.
 *
 * Call this from a test to inject known-good NMEA sentences
 * and verify that the driver parses and publishes correctly.
 */
void <devname>_emul_send_nmea(const struct device *uart_dev)
{
	for (int i = 0; test_nmea_sentences[i] != NULL; i++) {
		const char *s = test_nmea_sentences[i];

		for (size_t j = 0; j < strlen(s); j++) {
			/*
			 * Simulate UART receive by invoking the ISR
			 * callback with each character.
			 */
			/* Implementation depends on the UART emulation
			 * framework used by the test platform. */
		}
	}
}
```

---

## 10. Key Conventions

1. **Coding style** -- Linux kernel style. Tabs for indentation (not
   spaces). 80-column soft limit, 100-column hard limit. Opening braces
   on the same line for control structures, newline before `{` for
   functions.

2. **No dynamic allocation** -- all per-instance state is declared
   statically via the `DEVICE_DT_INST_DEFINE()` instantiation macros.
   Never call `k_malloc()` or `k_calloc()` in a driver.

3. **Config vs Data** -- `config` is compile-time-constant (from DT)
   and stored in flash/ROM. `data` is mutable runtime state stored in
   RAM. Access them via `dev->config` and `dev->data`.

4. **Devicetree macros** -- use `DT_INST_*` macros (which rely on
   `DT_DRV_COMPAT`) rather than hardcoding node paths.

5. **UART communication** -- GNSS drivers use UART, not SPI/I2C.
   Use `uart-device.yaml` as the DT binding include and
   `DEVICE_DT_GET(DT_INST_BUS(n))` to obtain the parent UART device.
   Always use `UART_INTERRUPT_DRIVEN` for non-blocking receive.

6. **NMEA parsing** -- use the built-in `gnss_nmea0183.h` helpers
   (`gnss_nmea0183_parse_gga()`, `gnss_nmea0183_parse_rmc()`,
   `gnss_nmea0183_parse_gsv()`) rather than writing custom parsers.
   Register sentence handlers via `GNSS_NMEA0183_MATCH_CALLBACK_DEFINE()`.

7. **Callback-based data delivery** -- GNSS data flows from driver
   to application via `gnss_publish_data()` and
   `gnss_publish_satellites()`. Applications register callbacks with
   `GNSS_DATA_CALLBACK_DEFINE()` and `GNSS_SATELLITES_CALLBACK_DEFINE()`.
   There is no poll/read API.

8. **Error codes** -- return negative `errno` values (`-EINVAL`,
   `-ENOTSUP`, `-ENODEV`, `-ENOMEM`, etc.). Never return positive
   error codes.

9. **Logging** -- use `LOG_MODULE_REGISTER(<devname>, CONFIG_GNSS_LOG_LEVEL)`.
   Use `LOG_ERR` for errors, `LOG_WRN` for warnings, `LOG_INF` for
   informational messages, `LOG_DBG` for debug messages. Never use
   `printk()` in a driver.

10. **Byte order** -- use `sys_get_le16()` / `sys_put_le16()` (or be
    variants) from `<zephyr/sys/byteorder.h>` instead of manual
    shifting. Most GNSS binary protocols use little-endian encoding.

11. **Bit manipulation** -- use `BIT()`, `GENMASK()`, `FIELD_PREP()`,
    `FIELD_GET()` from `<zephyr/sys/util.h>`. These are the Zephyr
    equivalents of the no-OS `NO_OS_BIT()`, `NO_OS_GENMASK()`,
    `no_os_field_prep()`, and `no_os_field_get()` macros.

12. **SPDX headers** -- every file needs a copyright line and
    `SPDX-License-Identifier: Apache-2.0`. No lengthy BSD-3-Clause
    blocks -- Zephyr uses the SPDX short form.

13. **DEVICE_API macro** -- always use `DEVICE_API(gnss, <devname>_api)`
    for the API struct declaration, not a raw `struct gnss_driver_api`
    assignment.

14. **Init priority** -- use `CONFIG_GNSS_INIT_PRIORITY` (default 80)
    unless the driver depends on another subsystem that initializes
    later. GNSS drivers typically run at `POST_KERNEL` level since they
    depend on UART being available.

15. **Thread safety** -- protect shared state accessed from both the
    UART ISR context and the API call context using `k_mutex`. Use
    `k_work` to defer NMEA processing from ISR to thread context.

16. **ISR constraints** -- never block, allocate memory, or perform
    lengthy operations in the UART ISR. Buffer received characters and
    submit work items for deferred processing.

---

## 11. Commit Message Format

Zephyr follows a strict commit message format. Each commit must have a
subsystem prefix, a short subject, and an informative body.

### Adding a new driver (typically 3-4 commits):

```
# Commit 1: Devicetree binding
dts: bindings: add binding for Analog Devices <DEVNAME> GNSS receiver

Add devicetree binding for the Analog Devices <DEVNAME>
multi-constellation GNSS receiver module with UART interface.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 2: Driver implementation
drivers: gnss: add Analog Devices <DEVNAME> driver

Add support for the Analog Devices <DEVNAME> multi-constellation
GNSS receiver module. The driver implements the Zephyr GNSS
subsystem API with UART-based NMEA 0183 parsing and optional
vendor binary protocol support for GPS, GLONASS, Galileo, and
BeiDou constellations.

Signed-off-by: Your Name <your.name@analog.com>
```

```
# Commit 3: Tests
tests: drivers: gnss: add tests for <DEVNAME>

Add unit tests for the <DEVNAME> GNSS driver covering device
readiness, fix rate configuration, navigation mode, and
constellation system management.

Signed-off-by: Your Name <your.name@analog.com>
```

### Key commit message rules:

- Subject line: max 72 characters, no trailing period
- Prefix: matches the path (e.g., `drivers: gnss:`, `dts: bindings:`)
- Body: wrapped at 75 characters, explains the "why"
- Must include `Signed-off-by:` (DCO requirement)
- Use imperative mood ("add", not "added" or "adds")
