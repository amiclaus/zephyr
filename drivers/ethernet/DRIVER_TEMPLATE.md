# Network/Ethernet Zephyr RTOS Driver Template

Reference driver: `zephyr/drivers/ethernet/eth_adin2111.c`

This template covers every file needed to add a new network/ethernet driver
to Zephyr RTOS. Replace `<devname>` with the part number (e.g., `adin2111`),
`<DEVNAME>` with its uppercase form (e.g., `ADIN2111`), and `<DevName>` with
the CamelCase form (e.g., `Adin2111`) throughout.

The Zephyr ethernet subsystem provides a standardised `struct ethernet_api`
that all ethernet drivers must implement. The driver registers with the
network stack via `ETH_NET_DEVICE_INIT()` and communicates over SPI using
the Zephyr SPI API (`struct spi_dt_spec`). MAC/PHY configuration, frame
TX/RX, and link management all flow through the ethernet API callbacks.

---

## 1. Naming & Placement Conventions

| Item | Pattern | Example |
|---|---|---|
| Driver source | `drivers/ethernet/eth_<devname>.c` | `eth_adin2111.c` |
| Kconfig | `drivers/ethernet/Kconfig.adi_<devname>` | `Kconfig.adi_adin2111` |
| Devicetree binding | `dts/bindings/ethernet/adi,<devname>.yaml` | `adi,adin2111.yaml` |
| DT compatible | `"adi,<devname>"` | `"adi,adin2111"` |
| Config symbol | `CONFIG_ETH_<DEVNAME>` | `CONFIG_ETH_ADIN2111` |
| Log module | `LOG_MODULE_REGISTER(eth_<devname>)` | `LOG_MODULE_REGISTER(eth_adin2111)` |
| Struct prefix | `<devname>_` | `adin2111_` |
| Port struct | `<devname>_port_data` | `adin2111_port_data` |
| Register defines | `<DEVNAME>_<REG>` | `ADIN2111_STATUS0` |
| Register masks | `<DEVNAME>_<FIELD>_MASK` | `ADIN2111_LINK_STATUS_MASK` |
| PHY driver | `drivers/ethernet/phy/phy_<devname>.c` | `phy_adin2111.c` |

---

## 2. File Checklist

```
zephyr/
    drivers/ethernet/
        eth_<devname>.c               # Main MAC driver
        Kconfig.adi_<devname>         # Kconfig fragment
    drivers/ethernet/phy/
        phy_<devname>.c               # PHY driver (if not using generic)
    dts/bindings/ethernet/
        adi,<devname>.yaml            # Devicetree binding
        adi,<devname>-port.yaml       # Per-port binding (multi-port devices)
    samples/net/<devname>/
        prj.conf                      # Sample project config
        boards/<board>.overlay        # Board-specific DT overlay
        src/main.c                    # Sample application
        CMakeLists.txt                # CMake build file
        sample.yaml                   # Twister test metadata
    tests/drivers/ethernet/<devname>/
        testcase.yaml                 # Twister test definition
        prj.conf                      # Test project config
        src/main.c                    # Test source
```

---

## 3. Devicetree Binding (`adi,<devname>.yaml`)

```yaml
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

description: |
  ADI <DEVNAME> 10BASE-T1L Ethernet MAC-PHY with SPI interface.

  Example devicetree node:

    &spi1 {
        status = "okay";
        cs-gpios = <&gpio0 4 GPIO_ACTIVE_LOW>;

        <devname>: <devname>@0 {
            compatible = "adi,<devname>";
            reg = <0>;
            spi-max-frequency = <25000000>;
            int-gpios = <&gpio0 5 GPIO_ACTIVE_LOW>;
            reset-gpios = <&gpio0 6 GPIO_ACTIVE_LOW>;
            status = "okay";

            port1 {
                compatible = "adi,<devname>-port";
                local-mac-address = [CA 2F B7 10 23 63];
            };
            /* For dual-port devices: */
            /* port2 {
                compatible = "adi,<devname>-port";
                local-mac-address = [CA 2F B7 10 23 64];
            }; */
        };
    };

compatible: "adi,<devname>"

include: spi-device.yaml

properties:
  int-gpios:
    type: phandle-array
    required: true
    description: |
      Interrupt GPIO pin. The interrupt is active low.

  reset-gpios:
    type: phandle-array
    description: |
      Optional hardware reset GPIO pin. Active low.

child-binding:
  description: Ethernet port on the <DEVNAME>.
  compatible: "adi,<devname>-port"
  properties:
    local-mac-address:
      type: uint8-array
      description: |
        The 6-byte MAC address assigned to this port.
```

---

## 4. Kconfig (`Kconfig.adi_<devname>`)

```kconfig
# Copyright (c) YYYY Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

menuconfig ETH_<DEVNAME>
	bool "<DEVNAME> 10BASE-T1L Ethernet MAC-PHY"
	default y
	depends on DT_HAS_ADI_<DEVNAME>_ENABLED
	select SPI
	select NET_L2_ETHERNET
	select GPIO
	help
	  Enable driver for ADI <DEVNAME> 10BASE-T1L single-pair
	  ethernet MAC-PHY with SPI interface.

if ETH_<DEVNAME>

config ETH_<DEVNAME>_IRQ_THREAD_STACK_SIZE
	int "Stack size for the <DEVNAME> IRQ thread"
	default 2048
	help
	  Stack size of the internal thread processing device interrupts.

config ETH_<DEVNAME>_IRQ_THREAD_PRIO
	int "Priority for the <DEVNAME> IRQ thread"
	default 2
	help
	  Priority level of the internal thread processing device
	  interrupts. A lower number indicates a higher priority.

config ETH_<DEVNAME>_TIMEOUT
	int "IP buffer timeout in milliseconds"
	default 100
	help
	  Timeout in milliseconds for network buffer allocation.

endif # ETH_<DEVNAME>
```

---

## 5. Driver Header Structures & Register Definitions

Zephyr ethernet drivers are typically single-file (`eth_<devname>.c`), with
all structures defined internally. However, the following shows the logical
organisation of types. Place these at the top of `eth_<devname>.c`.

```c
/*
 * Copyright (c) YYYY Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_<devname>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_<devname>, CONFIG_ETHERNET_LOG_LEVEL);

#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

/* ------------------------------------------------------------------ */
/* Register Map                                                       */
/* ------------------------------------------------------------------ */

#define <DEVNAME>_STATUS0                   0x00u
#define <DEVNAME>_STATUS1                   0x01u
#define <DEVNAME>_CONFIG0                   0x04u
#define <DEVNAME>_CONFIG2                   0x06u
#define <DEVNAME>_IMASK0                    0x0Cu
#define <DEVNAME>_IMASK1                    0x0Du
#define <DEVNAME>_TX_FSIZE                  0x30u
#define <DEVNAME>_TX                        0x31u
#define <DEVNAME>_TX_SPACE                  0x32u
#define <DEVNAME>_RX_FSIZE                  0x90u
#define <DEVNAME>_RX                        0x91u
#define <DEVNAME>_SOFT_RST                  0x3Cu
#define <DEVNAME>_MAC_ADDR_FILT_UPR(n)     (0x50u + 2u * (n))
#define <DEVNAME>_MAC_ADDR_FILT_LWR(n)     (0x51u + 2u * (n))
#define <DEVNAME>_MDIO_ACC                  0x20u
#define <DEVNAME>_PHY_ID                    0x10u

/* ------------------------------------------------------------------ */
/* Register Field Masks                                               */
/* ------------------------------------------------------------------ */

#define <DEVNAME>_CD_MASK                   BIT(15)
#define <DEVNAME>_RW_MASK                   BIT(13)
#define <DEVNAME>_ADDR_MASK                 GENMASK(12, 0)
#define <DEVNAME>_LINK_STATUS_MASK          BIT(0)
#define <DEVNAME>_RESETC_MASK               BIT(6)
#define <DEVNAME>_RX_RDY_MASK              BIT(4)
#define <DEVNAME>_TX_RDY_MASK              BIT(3)
#define <DEVNAME>_CRC_APPEND_MASK          BIT(5)
#define <DEVNAME>_FWD_UNK2HOST_MASK        BIT(2)

/* MDIO transaction fields */
#define <DEVNAME>_MDIO_TRDONE              BIT(31)
#define <DEVNAME>_MDIO_ST                  GENMASK(29, 28)
#define <DEVNAME>_MDIO_OP                  GENMASK(27, 26)
#define <DEVNAME>_MDIO_PRTAD              GENMASK(25, 21)
#define <DEVNAME>_MDIO_DEVAD              GENMASK(20, 16)
#define <DEVNAME>_MDIO_DATA               GENMASK(15, 0)

/* Frame constants */
#define <DEVNAME>_ETH_ALEN                 6u
#define <DEVNAME>_FCS_LEN                  4u
#define <DEVNAME>_FRAME_HEADER_LEN         2u
#define <DEVNAME>_BUFF_LEN                 1530u
#define <DEVNAME>_ETH_HDR_LEN             14u
#define <DEVNAME>_SPI_HEADER_LEN           2u

/* MDIO operation codes */
#define <DEVNAME>_MDIO_OP_WR               0x1u
#define <DEVNAME>_MDIO_OP_RD               0x3u

/* Expected device identification */
#define <DEVNAME>_PHY_ID_VAL               0xXXXXXXXXu

/* ------------------------------------------------------------------ */
/* Data Structures                                                    */
/* ------------------------------------------------------------------ */

/**
 * @brief Per-port runtime data.
 *
 * For multi-port devices (e.g., ADIN2111 with 2 ports), each port has
 * its own net_if, MAC address, and carrier state.
 */
struct <devname>_port_data {
	/** Back-reference to parent device. */
	const struct device *dev;
	/** Network interface associated with this port. */
	struct net_if *iface;
	/** Port index (0-based). */
	uint8_t port_idx;
};

/**
 * @brief Per-port static configuration from devicetree.
 */
struct <devname>_port_config {
	/** MAC address from devicetree local-mac-address property. */
	uint8_t mac_addr[<DEVNAME>_ETH_ALEN];
};

/**
 * @brief Device runtime data (shared across all ports).
 */
struct <devname>_data {
	/** IRQ processing thread stack. */
	K_KERNEL_STACK_MEMBER(thread_stack,
			      CONFIG_ETH_<DEVNAME>_IRQ_THREAD_STACK_SIZE);
	/** IRQ processing thread. */
	struct k_thread thread;
	/** Semaphore signalled on device interrupt. */
	struct k_sem offload_sem;
	/** Lock protecting SPI transactions. */
	struct k_mutex lock;
	/** GPIO interrupt callback. */
	struct gpio_callback gpio_int_cb;
	/** Shared SPI TX/RX buffer. */
	uint8_t buf[<DEVNAME>_BUFF_LEN + <DEVNAME>_SPI_HEADER_LEN + 4u];
	/** Number of ports on this device instance. */
	uint8_t port_count;
	/** Array of per-port runtime data. */
	struct <devname>_port_data port[2];
	/** Link state cached from last IRQ. */
	bool link_up;
};

/**
 * @brief Device static configuration from devicetree.
 */
struct <devname>_config {
	/** SPI bus specification from devicetree. */
	struct spi_dt_spec spi;
	/** Interrupt GPIO specification. */
	struct gpio_dt_spec interrupt;
	/** Reset GPIO specification (optional). */
	struct gpio_dt_spec reset;
	/** Number of child port nodes. */
	uint8_t port_count;
	/** Per-port static configs. */
	const struct <devname>_port_config *port_configs;
};
```

---

## 6. SPI Register Access

All register access goes through SPI. The control word format uses a
2-byte header with address, CD (control/data), and RW bits.

```c
/* ------------------------------------------------------------------ */
/* SPI Register Read / Write                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief Read a 32-bit register over SPI.
 *
 * @param dev   Zephyr device pointer.
 * @param addr  13-bit register address.
 * @param val   Pointer to store the read value.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_reg_read(const struct device *dev, uint16_t addr,
			      uint32_t *val)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint16_t header;
	int ret;

	const struct spi_buf tx_buf = {
		.buf = data->buf,
		.len = 7u,
	};
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1u };
	const struct spi_buf rx_buf = {
		.buf = data->buf,
		.len = 7u,
	};
	const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1u };

	header = (addr & <DEVNAME>_ADDR_MASK) | <DEVNAME>_CD_MASK;
	sys_put_be16(header, data->buf);
	data->buf[2] = 0x00u;                /* Turn-around byte */
	memset(&data->buf[3], 0, 4u);

	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
	if (ret < 0) {
		LOG_ERR("SPI read failed (addr=0x%04X): %d", addr, ret);
		return ret;
	}

	*val = sys_get_be32(&data->buf[3]);
	return 0;
}

/**
 * @brief Write a 32-bit register over SPI.
 *
 * @param dev   Zephyr device pointer.
 * @param addr  13-bit register address.
 * @param val   Value to write.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_reg_write(const struct device *dev, uint16_t addr,
			       uint32_t val)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint16_t header;

	const struct spi_buf tx_buf = {
		.buf = data->buf,
		.len = 6u,
	};
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1u };

	header = (addr & <DEVNAME>_ADDR_MASK) |
		 <DEVNAME>_CD_MASK |
		 <DEVNAME>_RW_MASK;
	sys_put_be16(header, data->buf);
	sys_put_be32(val, &data->buf[2]);

	int ret = spi_write_dt(&cfg->spi, &tx);

	if (ret < 0) {
		LOG_ERR("SPI write failed (addr=0x%04X): %d", addr, ret);
	}

	return ret;
}

/**
 * @brief Read-modify-write a register field.
 *
 * @param dev   Zephyr device pointer.
 * @param addr  Register address.
 * @param mask  Field bitmask.
 * @param val   New field value (pre-shifted into mask position).
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_reg_update(const struct device *dev, uint16_t addr,
				uint32_t mask, uint32_t val)
{
	uint32_t reg_val;
	int ret;

	ret = <devname>_reg_read(dev, addr, &reg_val);
	if (ret < 0) {
		return ret;
	}

	reg_val = (reg_val & ~mask) | (val & mask);
	return <devname>_reg_write(dev, addr, reg_val);
}
```

---

## 7. MAC / PHY Configuration

```c
/* ------------------------------------------------------------------ */
/* MAC Address Filtering                                              */
/* ------------------------------------------------------------------ */

/**
 * @brief Program a MAC address into a filter slot.
 *
 * @param dev   Zephyr device pointer.
 * @param slot  Filter slot index (0 = unicast, 1 = broadcast, ...).
 * @param addr  6-byte MAC address.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_set_mac_filter(const struct device *dev, uint8_t slot,
				    const uint8_t *addr)
{
	uint32_t upper, lower;
	int ret;

	/*
	 * Upper register: MAC[0..1] in bits [15:0], enable bit, port mask.
	 * Lower register: MAC[2..5] in bits [31:0].
	 * Adjust bit layout to match your device's datasheet.
	 */
	upper = ((uint32_t)addr[0] << 8) | addr[1];
	upper |= BIT(31);       /* Filter enable */
	upper |= BIT(30);       /* Apply to host (port 0) */

	lower = ((uint32_t)addr[2] << 24) |
		((uint32_t)addr[3] << 16) |
		((uint32_t)addr[4] << 8)  |
		 (uint32_t)addr[5];

	ret = <devname>_reg_write(dev, <DEVNAME>_MAC_ADDR_FILT_UPR(slot),
				  upper);
	if (ret < 0) {
		return ret;
	}

	return <devname>_reg_write(dev, <DEVNAME>_MAC_ADDR_FILT_LWR(slot),
				   lower);
}

/**
 * @brief Configure broadcast filter (slot 1 by convention).
 *
 * @param dev     Zephyr device pointer.
 * @param enable  true to accept broadcast frames, false to drop.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_set_broadcast_filter(const struct device *dev,
					  bool enable)
{
	if (enable) {
		static const uint8_t bcast[<DEVNAME>_ETH_ALEN] = {
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
		};
		return <devname>_set_mac_filter(dev, 1, bcast);
	}

	/* Clear filter slot 1 to drop broadcast. */
	return <devname>_reg_write(dev, <DEVNAME>_MAC_ADDR_FILT_UPR(1), 0u);
}

/* ------------------------------------------------------------------ */
/* MDIO / PHY Register Access                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief Write a PHY register via MDIO (clause 22), tunnelled over SPI.
 *
 * @param dev     Zephyr device pointer.
 * @param phy_id  PHY address on the MDIO bus.
 * @param reg     PHY register address.
 * @param val     Value to write.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_mdio_write(const struct device *dev, uint8_t phy_id,
				uint8_t reg, uint16_t val)
{
	uint32_t mdio_val;
	uint32_t acc;
	int ret;

	acc = FIELD_PREP(<DEVNAME>_MDIO_ST, 0x1u) |
	      FIELD_PREP(<DEVNAME>_MDIO_OP, <DEVNAME>_MDIO_OP_WR) |
	      FIELD_PREP(<DEVNAME>_MDIO_PRTAD, phy_id) |
	      FIELD_PREP(<DEVNAME>_MDIO_DEVAD, reg) |
	      FIELD_PREP(<DEVNAME>_MDIO_DATA, val);

	ret = <devname>_reg_write(dev, <DEVNAME>_MDIO_ACC, acc);
	if (ret < 0) {
		return ret;
	}

	/* Poll for MDIO transaction completion. */
	do {
		ret = <devname>_reg_read(dev, <DEVNAME>_MDIO_ACC, &mdio_val);
		if (ret < 0) {
			return ret;
		}
	} while (!(mdio_val & <DEVNAME>_MDIO_TRDONE));

	return 0;
}

/**
 * @brief Read a PHY register via MDIO (clause 22), tunnelled over SPI.
 *
 * @param dev     Zephyr device pointer.
 * @param phy_id  PHY address on the MDIO bus.
 * @param reg     PHY register address.
 * @param val     Pointer to store the read value.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_mdio_read(const struct device *dev, uint8_t phy_id,
			       uint8_t reg, uint16_t *val)
{
	uint32_t mdio_val;
	uint32_t acc;
	int ret;

	acc = FIELD_PREP(<DEVNAME>_MDIO_ST, 0x1u) |
	      FIELD_PREP(<DEVNAME>_MDIO_OP, <DEVNAME>_MDIO_OP_RD) |
	      FIELD_PREP(<DEVNAME>_MDIO_PRTAD, phy_id) |
	      FIELD_PREP(<DEVNAME>_MDIO_DEVAD, reg);

	ret = <devname>_reg_write(dev, <DEVNAME>_MDIO_ACC, acc);
	if (ret < 0) {
		return ret;
	}

	do {
		ret = <devname>_reg_read(dev, <DEVNAME>_MDIO_ACC, &mdio_val);
		if (ret < 0) {
			return ret;
		}
	} while (!(mdio_val & <DEVNAME>_MDIO_TRDONE));

	*val = FIELD_GET(<DEVNAME>_MDIO_DATA, mdio_val);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Hardware Reset                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief Assert hardware reset via GPIO, or fall back to software reset.
 *
 * @param dev  Zephyr device pointer.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_hw_reset(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	int ret;

	if (cfg->reset.port != NULL) {
		/* Hardware reset: drive low for 10 ms, then release. */
		ret = gpio_pin_set_dt(&cfg->reset, 1);  /* active low */
		if (ret < 0) {
			return ret;
		}

		k_msleep(10);

		ret = gpio_pin_set_dt(&cfg->reset, 0);
		if (ret < 0) {
			return ret;
		}

		/* Wait for MAC/PHY to initialise after reset. */
		k_msleep(90);
		return 0;
	}

	/* Software reset fallback. */
	ret = <devname>_reg_write(dev, <DEVNAME>_SOFT_RST, 0x1u);
	if (ret < 0) {
		return ret;
	}

	k_msleep(90);
	return 0;
}
```

---

## 8. Frame TX / RX

```c
/* ------------------------------------------------------------------ */
/* Frame Transmit                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief Transmit a network packet through the device TX FIFO.
 *
 * This is called by the Zephyr network stack via the ethernet API
 * send callback. The packet is linearised, framed with a port header,
 * and burst-written to the TX FIFO register over SPI.
 *
 * @param dev  Zephyr device pointer.
 * @param pkt  Network packet to transmit.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_send(const struct device *dev, struct net_pkt *pkt)
{
	struct <devname>_data *data = dev->data;
	const struct <devname>_config *cfg = dev->config;
	uint16_t frame_len;
	uint16_t padded_len;
	uint16_t padding = 0u;
	uint32_t tx_space;
	uint16_t header;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	frame_len = net_pkt_get_len(pkt);

	/* Ethernet minimum frame size is 64 bytes including FCS. */
	if (frame_len + <DEVNAME>_FCS_LEN < 64u) {
		padding = 64u - (frame_len + <DEVNAME>_FCS_LEN);
	}

	padded_len = frame_len + padding + <DEVNAME>_FRAME_HEADER_LEN;

	/* Check available TX FIFO space. */
	ret = <devname>_reg_read(dev, <DEVNAME>_TX_SPACE, &tx_space);
	if (ret < 0) {
		goto unlock;
	}

	if (padded_len > (2u * tx_space)) {
		LOG_WRN("TX FIFO full, frame dropped");
		ret = -EAGAIN;
		goto unlock;
	}

	/* Write frame size register. */
	ret = <devname>_reg_write(dev, <DEVNAME>_TX_FSIZE, padded_len);
	if (ret < 0) {
		goto unlock;
	}

	/* Build SPI header for TX data write. */
	header = (<DEVNAME>_TX & <DEVNAME>_ADDR_MASK) |
		 <DEVNAME>_CD_MASK |
		 <DEVNAME>_RW_MASK;
	sys_put_be16(header, data->buf);

	/* Port header (port 0 for single-port devices). */
	sys_put_be16(0u, &data->buf[<DEVNAME>_SPI_HEADER_LEN]);

	/* Linearise the packet into the SPI buffer after headers. */
	size_t offset = <DEVNAME>_SPI_HEADER_LEN + <DEVNAME>_FRAME_HEADER_LEN;

	if (net_pkt_read(pkt, &data->buf[offset], frame_len) < 0) {
		LOG_ERR("Failed to read packet data");
		ret = -EIO;
		goto unlock;
	}

	/* Zero-fill any padding bytes. */
	if (padding > 0u) {
		memset(&data->buf[offset + frame_len], 0, padding);
	}

	/* Burst SPI write. */
	size_t total_len = ROUND_UP(padded_len + <DEVNAME>_SPI_HEADER_LEN, 4u);
	const struct spi_buf tx_buf = {
		.buf = data->buf,
		.len = total_len,
	};
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1u };

	ret = spi_write_dt(&cfg->spi, &tx_set);
	if (ret < 0) {
		LOG_ERR("TX SPI write failed: %d", ret);
	}

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Frame Receive                                                      */
/* ------------------------------------------------------------------ */

/**
 * @brief Read a frame from the RX FIFO and deliver it to the network stack.
 *
 * Called from the IRQ thread when the RX_RDY interrupt fires.
 *
 * @param dev  Zephyr device pointer.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_recv(const struct device *dev)
{
	struct <devname>_data *data = dev->data;
	const struct <devname>_config *cfg = dev->config;
	uint32_t frame_size;
	uint16_t header;
	uint16_t port_hdr;
	uint8_t port_idx;
	size_t rounded_len;
	size_t payload_len;
	struct net_pkt *pkt;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = <devname>_reg_read(dev, <DEVNAME>_RX_FSIZE, &frame_size);
	if (ret < 0 || frame_size < (<DEVNAME>_FRAME_HEADER_LEN +
				     <DEVNAME>_FCS_LEN)) {
		goto unlock;
	}

	/* Build SPI read header. */
	header = (<DEVNAME>_RX & <DEVNAME>_ADDR_MASK) | <DEVNAME>_CD_MASK;
	memset(data->buf, 0, sizeof(data->buf));
	sys_put_be16(header, data->buf);
	data->buf[2] = 0x00u;   /* Turn-around */

	rounded_len = ROUND_UP(frame_size, 4u);
	size_t xfer_len = rounded_len + 3u;   /* +3 for SPI read header */

	const struct spi_buf tx_buf = { .buf = data->buf, .len = xfer_len };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1u };
	const struct spi_buf rx_buf = { .buf = data->buf, .len = xfer_len };
	const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1u };

	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
	if (ret < 0) {
		LOG_ERR("RX SPI read failed: %d", ret);
		goto unlock;
	}

	/* Extract port header (2 bytes after SPI read header). */
	port_hdr = sys_get_be16(&data->buf[3]);
	port_idx = port_hdr & 0x01u;    /* Bit 0 identifies the port */

	/* Ethernet frame starts after SPI header (3) + port header (2). */
	size_t frame_offset = 3u + <DEVNAME>_FRAME_HEADER_LEN;
	payload_len = frame_size - <DEVNAME>_FRAME_HEADER_LEN;

	/* Allocate a net_pkt and copy the frame data. */
	pkt = net_pkt_rx_alloc_with_buffer(data->port[port_idx].iface,
					   payload_len, AF_UNSPEC, 0,
					   K_MSEC(CONFIG_ETH_<DEVNAME>_TIMEOUT));
	if (!pkt) {
		LOG_ERR("Failed to allocate RX packet");
		ret = -ENOMEM;
		goto unlock;
	}

	ret = net_pkt_write(pkt, &data->buf[frame_offset], payload_len);
	if (ret < 0) {
		LOG_ERR("Failed to write packet data");
		net_pkt_unref(pkt);
		goto unlock;
	}

	k_mutex_unlock(&data->lock);

	/* Hand off to the network stack. */
	ret = net_recv_data(data->port[port_idx].iface, pkt);
	if (ret < 0) {
		LOG_ERR("net_recv_data failed: %d", ret);
		net_pkt_unref(pkt);
	}

	return ret;

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}
```

---

## 9. Interrupt Handling & IRQ Thread

```c
/* ------------------------------------------------------------------ */
/* GPIO Interrupt Callback                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief GPIO interrupt callback -- signals the offload thread.
 *
 * The device asserts its INT pin (active low) when it has status to
 * report (RX ready, link change, errors). The actual processing
 * happens in a dedicated kernel thread to avoid doing SPI from ISR
 * context.
 */
static void <devname>_int_callback(const struct device *gpio_dev,
				   struct gpio_callback *cb,
				   uint32_t pins)
{
	struct <devname>_data *data =
		CONTAINER_OF(cb, struct <devname>_data, gpio_int_cb);

	k_sem_give(&data->offload_sem);
}

/* ------------------------------------------------------------------ */
/* IRQ Processing Thread                                              */
/* ------------------------------------------------------------------ */

/**
 * @brief Thread that processes device interrupts.
 *
 * Waits on the offload semaphore, reads the device status register to
 * determine the interrupt source, and dispatches to the appropriate
 * handler (RX, link change, error).
 */
static void <devname>_offload_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct <devname>_data *data = dev->data;
	uint32_t status;
	int ret;

	while (true) {
		k_sem_take(&data->offload_sem, K_FOREVER);

		k_mutex_lock(&data->lock, K_FOREVER);
		ret = <devname>_reg_read(dev, <DEVNAME>_STATUS1, &status);
		k_mutex_unlock(&data->lock);

		if (ret < 0) {
			LOG_ERR("Failed to read status: %d", ret);
			continue;
		}

		/* Handle link change. */
		if (status & <DEVNAME>_LINK_STATUS_MASK) {
			bool link_up = !!(status & <DEVNAME>_LINK_STATUS_MASK);

			if (link_up != data->link_up) {
				data->link_up = link_up;
				for (uint8_t i = 0u; i < data->port_count;
				     i++) {
					if (link_up) {
						net_eth_carrier_on(
							data->port[i].iface);
					} else {
						net_eth_carrier_off(
							data->port[i].iface);
					}
				}
				LOG_INF("Link %s",
					link_up ? "up" : "down");
			}
		}

		/* Handle RX ready. */
		if (status & <DEVNAME>_RX_RDY_MASK) {
			ret = <devname>_recv(dev);
			if (ret < 0 && ret != -EAGAIN) {
				LOG_ERR("RX failed: %d", ret);
			}
		}

		/* Clear handled interrupts by writing status back. */
		k_mutex_lock(&data->lock, K_FOREVER);
		(void)<devname>_reg_write(dev, <DEVNAME>_STATUS1, status);
		k_mutex_unlock(&data->lock);
	}
}
```

---

## 10. Ethernet API & Device Initialisation

```c
/* ------------------------------------------------------------------ */
/* Ethernet API Callbacks                                             */
/* ------------------------------------------------------------------ */

/**
 * @brief Get device capabilities reported to the network stack.
 *
 * @param dev  Zephyr device pointer.
 * @return Bitmask of ethernet hardware capabilities.
 */
static enum ethernet_hw_caps <devname>_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return ETHERNET_LINK_10BASE_T |
	       ETHERNET_PROMISC_MODE;
	/*
	 * Add more as applicable:
	 *   ETHERNET_HW_FILTERING
	 *   ETHERNET_HW_VLAN
	 *   ETHERNET_LINK_100BASE_T
	 */
}

/**
 * @brief Set device configuration (promiscuous mode, MAC filter, etc.).
 *
 * @param dev   Zephyr device pointer.
 * @param type  Configuration type.
 * @param config Configuration value.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_set_config(const struct device *dev,
				enum ethernet_config_type type,
				const struct ethernet_config *config)
{
	switch (type) {
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		return <devname>_reg_update(
			dev, <DEVNAME>_CONFIG0,
			<DEVNAME>_FWD_UNK2HOST_MASK,
			config->promisc_mode ?
				<DEVNAME>_FWD_UNK2HOST_MASK : 0u);

	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		return <devname>_set_mac_filter(
			dev, 0u, config->mac_address.addr);

	default:
		return -ENOTSUP;
	}
}

/**
 * @brief Ethernet API send wrapper.
 *
 * The network stack calls this to transmit a packet. For multi-port
 * devices, the port index can be derived from the network interface.
 */
static int <devname>_port_send(const struct device *dev,
			       struct net_pkt *pkt)
{
	/* For single-port, delegate directly. For multi-port, encode
	 * port index in the frame header inside <devname>_send(). */
	return <devname>_send(dev, pkt);
}

/**
 * @brief Network interface initialisation callback.
 *
 * Called by the network stack when the interface is brought up. Sets
 * the MAC address and marks the carrier as initially off.
 */
static void <devname>_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint8_t port_idx;

	/* Determine which port this interface belongs to. */
	for (port_idx = 0u; port_idx < data->port_count; port_idx++) {
		if (data->port[port_idx].iface == NULL) {
			data->port[port_idx].iface = iface;
			break;
		}
	}

	if (port_idx >= data->port_count) {
		LOG_ERR("No free port slot for iface");
		return;
	}

	const struct <devname>_port_config *pcfg =
		&cfg->port_configs[port_idx];

	net_if_set_link_addr(iface, (uint8_t *)pcfg->mac_addr,
			     <DEVNAME>_ETH_ALEN, NET_LINK_ETHERNET);
	ethernet_init(iface);

	/* Carrier starts off until link is detected. */
	net_eth_carrier_off(iface);
}

/**
 * @brief The ethernet API vtable wired into the device instance.
 */
static const struct ethernet_api <devname>_api = {
	.iface_api.init = <devname>_iface_init,
	.get_capabilities = <devname>_get_capabilities,
	.set_config = <devname>_set_config,
	.send = <devname>_port_send,
};

/* ------------------------------------------------------------------ */
/* Device Initialisation                                              */
/* ------------------------------------------------------------------ */

/**
 * @brief Initialise the <DEVNAME> device at boot.
 *
 * Called automatically by the Zephyr device model during system init.
 * Configures GPIOs, resets the device, verifies PHY ID, sets up MAC
 * filters, configures interrupts, and spawns the IRQ processing thread.
 *
 * @param dev  Zephyr device pointer.
 * @return 0 on success, negative errno on failure.
 */
static int <devname>_init(const struct device *dev)
{
	const struct <devname>_config *cfg = dev->config;
	struct <devname>_data *data = dev->data;
	uint32_t phy_id;
	int ret;

	data->port_count = cfg->port_count;

	/* Validate SPI bus is ready. */
	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	k_mutex_init(&data->lock);
	k_sem_init(&data->offload_sem, 0, 1);

	/* ---- Reset GPIO (optional) ---- */
	if (cfg->reset.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->reset)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->reset,
					    GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			return ret;
		}

		ret = <devname>_hw_reset(dev);
		if (ret < 0) {
			return ret;
		}
	} else {
		/* Software reset. */
		ret = <devname>_reg_write(dev, <DEVNAME>_SOFT_RST, 0x1u);
		if (ret < 0) {
			return ret;
		}
		k_msleep(90);
	}

	/* ---- Verify device identity ---- */
	ret = <devname>_reg_read(dev, <DEVNAME>_PHY_ID, &phy_id);
	if (ret < 0) {
		return ret;
	}

	if (phy_id != <DEVNAME>_PHY_ID_VAL) {
		LOG_ERR("Unexpected PHY ID: 0x%08X (expected 0x%08X)",
			phy_id, <DEVNAME>_PHY_ID_VAL);
		return -ENODEV;
	}

	LOG_INF("PHY ID: 0x%08X", phy_id);

	/* ---- Program MAC address filters ---- */
	for (uint8_t i = 0u; i < cfg->port_count; i++) {
		ret = <devname>_set_mac_filter(dev, i,
					       cfg->port_configs[i].mac_addr);
		if (ret < 0) {
			return ret;
		}
		data->port[i].dev = dev;
		data->port[i].port_idx = i;
	}

	/* Accept broadcast frames. */
	ret = <devname>_set_broadcast_filter(dev, true);
	if (ret < 0) {
		return ret;
	}

	/* ---- Configure CRC append / initial config ---- */
	ret = <devname>_reg_update(dev, <DEVNAME>_CONFIG0,
				   <DEVNAME>_CRC_APPEND_MASK,
				   <DEVNAME>_CRC_APPEND_MASK);
	if (ret < 0) {
		return ret;
	}

	/* ---- Interrupt GPIO ---- */
	if (!gpio_is_ready_dt(&cfg->interrupt)) {
		LOG_ERR("Interrupt GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->interrupt, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&data->gpio_int_cb,
			   <devname>_int_callback,
			   BIT(cfg->interrupt.pin));

	ret = gpio_add_callback(cfg->interrupt.port, &data->gpio_int_cb);
	if (ret < 0) {
		return ret;
	}

	/* ---- Enable device interrupts ---- */
	ret = <devname>_reg_write(dev, <DEVNAME>_IMASK0, 0u);
	if (ret < 0) {
		return ret;
	}

	ret = <devname>_reg_write(dev, <DEVNAME>_IMASK1,
				  ~(<DEVNAME>_RX_RDY_MASK |
				    <DEVNAME>_LINK_STATUS_MASK |
				    <DEVNAME>_TX_RDY_MASK));
	if (ret < 0) {
		return ret;
	}

	/* ---- Spawn the IRQ processing thread ---- */
	k_thread_create(&data->thread, data->thread_stack,
			CONFIG_ETH_<DEVNAME>_IRQ_THREAD_STACK_SIZE,
			<devname>_offload_thread,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ETH_<DEVNAME>_IRQ_THREAD_PRIO),
			0, K_NO_WAIT);

	k_thread_name_set(&data->thread, "eth_<devname>");

	LOG_INF("<DEVNAME> initialised (%u port(s))", cfg->port_count);

	return 0;
}
```

---

## 11. Device Instantiation Macros

```c
/* ------------------------------------------------------------------ */
/* Devicetree-driven instantiation                                    */
/* ------------------------------------------------------------------ */

/*
 * Per-port config array, built from devicetree child nodes.
 * Each child with compatible "adi,<devname>-port" provides a
 * local-mac-address.
 */
#define <DEVNAME>_PORT_CONFIG(port_node)                                     \
	{                                                                    \
		.mac_addr = DT_PROP_OR(port_node, local_mac_address,         \
				       {0}),                                 \
	},

#define <DEVNAME>_PORT_CONFIGS(inst)                                         \
	static const struct <devname>_port_config                            \
		<devname>_port_cfgs_##inst[] = {                             \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(inst,                      \
						  <DEVNAME>_PORT_CONFIG)     \
	};

/*
 * Main device instantiation macro. Called once per devicetree instance
 * of the "adi,<devname>" compatible.
 */
#define <DEVNAME>_DEFINE(inst)                                               \
									     \
	<DEVNAME>_PORT_CONFIGS(inst)                                         \
									     \
	static struct <devname>_data <devname>_data_##inst;                  \
									     \
	static const struct <devname>_config <devname>_config_##inst = {     \
		.spi = SPI_DT_SPEC_INST_GET(inst,                            \
					    SPI_OP_MODE_MASTER |             \
					    SPI_TRANSFER_MSB |               \
					    SPI_WORD_SET(8), 0),             \
		.interrupt = GPIO_DT_SPEC_INST_GET(inst, int_gpios),         \
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),   \
		.port_count = DT_INST_CHILD_NUM_STATUS_OKAY(inst),           \
		.port_configs = <devname>_port_cfgs_##inst,                  \
	};                                                                   \
									     \
	ETH_NET_DEVICE_INIT(inst,                                            \
			    "<devname>_" #inst,                               \
			    <devname>_init,                                   \
			    NULL,                                             \
			    &<devname>_data_##inst,                           \
			    &<devname>_config_##inst,                         \
			    CONFIG_ETH_INIT_PRIORITY,                         \
			    &<devname>_api,                                   \
			    NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(<DEVNAME>_DEFINE)
```

### Key Points for Device Instantiation

1. **`DT_DRV_COMPAT`** must be set to `adi_<devname>` (underscores, not
   commas) at the top of the file, before any `DT_INST_*` macro usage.

2. **`ETH_NET_DEVICE_INIT()`** wires together:
   - The init function (`<devname>_init`)
   - Runtime data (`<devname>_data_##inst`)
   - Static config (`<devname>_config_##inst`)
   - The ethernet API vtable (`<devname>_api`)
   - The MTU (`NET_ETH_MTU` = 1500)

3. **`SPI_DT_SPEC_INST_GET()`** extracts the full SPI bus configuration
   (controller, chip-select, frequency) from the devicetree node.

4. **`GPIO_DT_SPEC_INST_GET()`** extracts GPIO pin, port, and active
   level from `int-gpios` and `reset-gpios` properties.

5. **`DT_INST_FOREACH_CHILD_STATUS_OKAY()`** iterates over child port
   nodes to build the per-port config array (MAC addresses).

6. **Multi-instance** -- `DT_INST_FOREACH_STATUS_OKAY()` creates one
   full device instance for each `adi,<devname>` node in the devicetree
   that has `status = "okay"`.

---

## Comparison: no-OS vs. Zephyr Ethernet Driver

| Aspect | no-OS | Zephyr |
|---|---|---|
| Communication | `no_os_spi_transfer()` | `spi_transceive_dt()` / `spi_write_dt()` |
| Device handle | `struct <devname>_desc *` (heap-allocated) | `const struct device *` (static, DT-driven) |
| Init pattern | `<devname>_init(**desc, *param)` + goto cleanup | `<devname>_init(dev)` returning errno |
| Cleanup | `<devname>_remove(desc)` | Not needed (static lifetime) |
| Frame TX | `<devname>_write_fifo(desc, port, eth_buff)` | `<devname>_send(dev, net_pkt)` via ethernet API |
| Frame RX | `<devname>_read_fifo(desc, port, eth_buff)` | `<devname>_recv(dev)` + `net_recv_data()` |
| Buffer model | Raw `uint8_t *` payload in `<devname>_eth_buff` | `struct net_pkt` with fragment chain |
| Interrupt | Polled in main loop | GPIO IRQ + dedicated `k_thread` |
| Config source | C struct (`init_param`) | Devicetree + Kconfig |
| MAC address | `init_param.mac_address[]` | DT `local-mac-address` property |
| GPIO | `no_os_gpio_*()` | `gpio_pin_*_dt()` |
| Locking | None (single-threaded) | `k_mutex` around SPI transactions |
| Logging | `pr_info()` / `pr_err()` | `LOG_INF()` / `LOG_ERR()` |
| Byte order | `no_os_put_unaligned_be*()` | `sys_put_be*()` / `sys_get_be*()` |
| Bit fields | `no_os_field_prep()` / `no_os_field_get()` | `FIELD_PREP()` / `FIELD_GET()` |
| Registration | Manual in example `main()` | `ETH_NET_DEVICE_INIT()` macro |
