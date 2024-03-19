/*
 * Copyright (c) 2024 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_adin6310, CONFIG_ETHERNET_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include "zephyr/sys/util.h"

#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>

#include "eth_adin6310.h"
#include "SMP_stack_api.h"
#include "SES_port_api.h"
#include "SES_switch.h"
#include "SES_event.h"
#include "SES_frame_api.h"
#include "SES_interface_management.h"

static struct k_sem semaphores[50];
struct k_sem reader_thread_sem;
K_THREAD_STACK_DEFINE(stack_area, 20000);
K_SEM_DEFINE(read_done_sem, 1, 1);
K_MUTEX_DEFINE(spi_mutex);

static struct k_thread read_thread;
static k_tid_t spi_read_tid;

const SES_portInit_t initializePorts_p[] = {
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1100, {true, 0, 0, SES_phySpeed10, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1100, {true, 0, 1, SES_phySpeed10, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1300, {true, 0, 6, SES_phySpeed1000, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1300, {true, 0, 5, SES_phySpeed1000, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1100, {true, 0, 2, SES_phySpeed10, SES_phyDuplexModeFull, SES_autoMdix}},
	{ 1, SES_rgmiiMode, { 0, 0, 0 }, 1, SES_phyADIN1100, {true, 0, 3, SES_phySpeed10, SES_phyDuplexModeFull, SES_autoMdix}}
};

struct adin6310_priv {
        struct k_sem msg_recv_sem;
};

void* SES_PORT_CreateSemaphore(int initCount, int maxCount)
{
	struct k_sem *ret_sem;
	struct k_sem sem;

	for (int i = 0; i < 50; i++) {
		if (semaphores[i].limit == 0) {
			k_sem_init(&semaphores[i], initCount, maxCount);

			return &semaphores[i];
		}
	}

	return 0;
}

int32_t SES_PORT_WaitSemaphore(int* semaphore_p, int timeout)
{
	struct k_sem *sem = (struct k_sem *)semaphore_p;
	int ret;

	ret = k_sem_take(sem, K_MSEC(timeout));
	if (ret == -ETIMEDOUT)
		return SES_PORT_TIMEOUT;

	return SES_PORT_OK;
}

int32_t SES_PORT_SignalSemaphore(int* semaphore_p)
{
	struct k_sem *sem = (struct k_sem *)semaphore_p;

	if (sem == NULL)
		return SES_PORT_INVALID_PARAM;

	k_sem_give(sem);

	return 1;
}

int32_t SES_PORT_DeleteSemaphore(int* semaphore_p)
{
	struct k_sem *sem = (struct k_sem *)semaphore_p;

	memset(sem, 0x0, sizeof(*sem));

	return 1;
}

int32_t SES_PORT_Malloc(void **buf_pp, int size)
{
	if (!buf_pp)
		return SES_PORT_BUF_ERR;
	
	*buf_pp = k_malloc(size);

	return 0;
}

int32_t SES_PORT_Free(void *buf_p)
{
	if (!buf_p)
		return SES_PORT_BUF_ERR;

	k_free(buf_p);

	return 0;
}

static int adin6310_spi_init(void *param_p,
                      SES_PORT_intfType_t *intfType_p,
                      uint8_t *srcMac_p)
{
	*intfType_p = SES_PORT_spiInterface;

	return 0;
}

static int adin6310_spi_remove(int intfHandle)
{
	return 0;
}

static int adin6310_spi_write(int intfHandle, int size, void *data_p)
{
	const struct spi_dt_spec dev_spi = SPI_DT_SPEC_GET(DT_NODELABEL(adin6310),
							   SPI_WORD_SET(8) |
							   SPI_OP_MODE_MASTER |
							   SPI_TRANSFER_MSB |
							   SPI_MODE_GET(0), 0);
	struct spi_buf tx_buf;
	struct spi_buf_set tx;
	uint8_t *txb;
	int ret = 0;

	k_mutex_lock(&spi_mutex, K_FOREVER);
	tx_buf.len = size + 1;
	txb = k_calloc(size + 1, 1);
	if (!txb) {
		LOG_ERR("Calloc error %d txb", (uint32_t)txb);
		ret = -ENOMEM;
		goto free_txb;
	}

	tx_buf.buf = txb;
	tx.buffers = &tx_buf;
	tx.count = 1;

	txb[0] = ADIN6310_SPI_WR_HEADER;
	memcpy(&txb[1], data_p, size);
	ret = spi_write_dt(&dev_spi, &tx);
	
	SES_PORT_Free(data_p);
free_txb:
	k_free(txb);
	k_mutex_unlock(&spi_mutex);

	return ret;
}

static int adin6310_read_msg(uint8_t *buf, uint32_t len)
{
	int ret = 0;
	uint8_t *xfer_buf_rx;
	uint8_t *xfer_buf_tx;
	struct spi_buf rx_buf;
	struct spi_buf tx_buf;
	struct spi_buf_set rx_buf_set;
	struct spi_buf_set tx_buf_set;
	const struct spi_dt_spec dev_spi = SPI_DT_SPEC_GET(DT_NODELABEL(adin6310),
							   SPI_WORD_SET(8) |
							   SPI_OP_MODE_MASTER |
							   SPI_TRANSFER_MSB |
							   SPI_MODE_GET(0), 0);

	k_mutex_lock(&spi_mutex, K_FOREVER);
	xfer_buf_rx = k_calloc(len + 1, sizeof(*xfer_buf_rx));
	if (!xfer_buf_rx) {
		LOG_ERR("Calloc error %d xfer_buf_rx", (uint32_t)xfer_buf_rx);
		ret = -ENOMEM;
		goto mutex_unlock;
	}

	xfer_buf_tx = k_calloc(len + 1, sizeof(*xfer_buf_tx));
	if (!xfer_buf_tx) {
		LOG_ERR("Calloc error %d xfer_buf_tx", (uint32_t)xfer_buf_tx);
		ret = -ENOMEM;
		goto free_rx;
	}

	xfer_buf_tx[0] = ADIN6310_SPI_RD_HEADER;

	rx_buf.len = len + 1;
	rx_buf.buf = xfer_buf_rx;
	tx_buf.len = len + 1;
	tx_buf.buf = xfer_buf_tx;

	rx_buf_set.buffers = &rx_buf;
	rx_buf_set.count = 1;
	tx_buf_set.buffers = &tx_buf;
	tx_buf_set.count = 1;

	ret = spi_transceive_dt(&dev_spi, &tx_buf_set, &rx_buf_set);
	if (ret)
		goto free_tx;

	memcpy(buf, &xfer_buf_rx[1], len);

free_tx:
	k_free(xfer_buf_tx);
free_rx:
	k_free(xfer_buf_rx);
mutex_unlock:
	k_mutex_unlock(&spi_mutex);

	return ret;
}

static int adin6310_read_message(int tbl_index)
{
	uint32_t frame_type;
	uint8_t *frame_buf;
	uint32_t padded_len;
	uint32_t rx_len;
	uint8_t header[4];
	int ret;

	ret = adin6310_read_msg(header, 4);
	if (ret)
		return ret;

	rx_len = ((header[1] & 0xF) << 8) | header[0];
	frame_type = (header[1] & 0xF0) >> 4;

	padded_len = rx_len + 0x3;
	padded_len &= ~GENMASK(1, 0);

	frame_buf = k_calloc(padded_len, sizeof(*frame_buf));
	if (!frame_buf)
		return -ENOMEM;

	ret = adin6310_read_msg(frame_buf, padded_len);
	if (ret)
		goto free_frame_buf;

	ret = SES_ReceiveMessage(tbl_index,
			SES_PORT_spiInterface,
			rx_len,
			(void*)frame_buf,
			(-1),
			(0),
			NULL);

free_frame_buf:
		k_free(frame_buf);

	return ret;
}

static void adin6310_msg_recv(void *p1, void *p2, void *p3)
{
	k_thread_custom_data_set(p1);

	while (1) {
		k_sem_take(&reader_thread_sem, K_FOREVER);
		adin6310_read_message(0);
	};
}

static void adin6310_int_rdy(const struct device *port,
			     struct gpio_callback *cb,
		      	     gpio_port_pins_t pins)
{
	struct adin6310_data *data = CONTAINER_OF(cb, struct adin6310_data, gpio_int_callback);

	k_sem_give(&reader_thread_sem);
}

static void adin6310_rx_callback(int32_t frame_len, uint8_t *frame, SES_frameAttributes_t *attrs)
{
	struct net_pkt *pkt;
	struct net_if *iface;
	struct adin6310_data *data = k_thread_custom_data_get();
	struct adin6310_port_data *port = data->port_data[attrs->port];
	int ret;

	iface = port->iface;
	if (!net_if_is_carrier_ok(iface))
		net_eth_carrier_on(iface);

	pkt = net_pkt_rx_alloc_with_buffer(iface, frame_len, AF_UNSPEC, 0, K_FOREVER);
	if (!pkt) {
		LOG_ERR("Error on net_pkt alloc\n");
		return;
	}

	ret = net_pkt_write(pkt, frame, frame_len);
	if (ret) {
		net_pkt_unref(pkt);
		LOG_ERR("Port %u failed to fill RX frame", attrs->port);
		return;
	}

	ret = net_recv_data(iface, pkt);
	if (ret) {
		net_pkt_unref(pkt);
		LOG_ERR("Port %u failed to enqueue frame to RX queue",
			attrs->port);
		return;
	}
}

static int adin6310_port_send(const struct device *dev, struct net_pkt *pkt)
{
	struct adin6310_port_data *data = dev->data;
	size_t pkt_len = net_pkt_get_len(pkt);
	uint8_t *pkt_buf;
	int ret;

	SES_transmitFrameData_t txData_p = {
		.frameType = SES_standardFrame,
		.byteCount = pkt_len,
		.cb_p = 0,
		.cbParam_p = 0,
		.pad = 0,
		.ses = {
			.generateFcs = 1,
			.xmitPriority = 0,
			.egressPortMap = BIT(data->id),
			.transformId = SES_NO_TRANSFORM,
			.attributeRequest = 0,
		}
	};

	if (!net_if_is_carrier_ok(pkt->iface))
		net_if_carrier_on(pkt->iface);

	pkt_buf = k_calloc(pkt_len, sizeof(*pkt_buf));
	if (!pkt_buf) {
		LOG_ERR("Port %d failed to alloc packet buffer", data->id);
		return -ENOMEM;
	}

	txData_p.data_p = pkt_buf;
	ret = net_pkt_read(pkt, pkt_buf, pkt_len);
	if (ret < 0) {
		LOG_ERR("Port %d failed to read PKT into TX buffer", data->id);
		goto free_pkt_buf;
	}

	ret = SES_XmitFrame(&txData_p);
	if (ret)
		LOG_ERR("Port %d failed to send packet", data->id);

free_pkt_buf:
	k_free(pkt_buf);

	return ret;
}

static int adin6310_add_filter(uint8_t mac_addr[6])
{
	return SES_RxSesFramesByMac(mac_addr, SES_REQUEST_PORT_ATTRIBUTE, adin6310_rx_callback);
}

static enum ethernet_hw_caps adin6310_port_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_1000BASE_T | ETHERNET_LINK_100BASE_T | ETHERNET_LINK_10BASE_T;
}

static int adin6310_port_set_config(const struct device *dev,
				    enum ethernet_config_type type,
				    const struct ethernet_config *config)
{
	int ret = -ENOTSUP;

	if (type == ETHERNET_CONFIG_TYPE_MAC_ADDRESS) {
		ret = adin6310_add_filter((uint8_t *)&config->mac_address.addr[0]);
		if (ret)
			return ret;
	}

	return ret;
}

static void adin6310_port_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct adin6310_port_data *data = dev->data;
	struct adin6310_port_config *config = dev->config;
	struct adin6310_data *adin_priv = config->net_device;
	static bool subscribed = false;
	int ret;

	if (!device_is_ready(config->adin)) {
		LOG_ERR("ADIN is not ready, can't init port %u iface",
			config->id);
		return;
	}

	data->adin = config->adin;
	data->id = config->id;
	data->net_device = config->net_device;
	data->name = config->name;
	data->initialized = false;
	data->ethernet_port = config->ethernet_port;
	memcpy(data->mac_addr, config->mac_addr, 6);

	net_if_set_link_addr(iface, config->mac_addr, 6, NET_LINK_ETHERNET);
	ethernet_init(iface);
	net_if_carrier_off(iface);

	ret = SES_RxSesFramesByMac(config->mac_addr, SES_REQUEST_PORT_ATTRIBUTE, adin6310_rx_callback);
	if (ret) {
		LOG_ERR("Error (%d) configuring MAC filter for port %d", ret, config->id);
		return;
	}

	data->initialized = true;
	data->iface = iface;
	adin_priv->port_data[data->id] = data;
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *adin6310_port_get_stats(const struct device *dev)
{
	struct adin6310_data *data = dev->data;

	return NULL;
}
#endif /* CONFIG_NET_STATISTICS_ETHERNET */

static int adin6310_set_broadcast_route(const struct device *dev)
{
	uint8_t lookupPriority = 1;
	uint8_t macAddr[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	uint8_t macMask[6] = { 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00 };
	int16_t vlanId = SES_DYNTBL_NO_VLAN;
	uint16_t vlanMask = 0xFFF;
	uint8_t sourceEntry = 0;
	uint8_t override = 0;
	uint8_t sourceOverride = 0;
	uint8_t hsrOrPrpSupervisory = 0;
	SES_dynTblLookup_t lookupType = SES_dynamicTblLookupBasic;
	uint8_t portMap = 0x3F;
	uint8_t sendToAE = 1;
	uint8_t danp = 0;
	uint8_t cutThrough = 0;
	uint8_t syntTimestamp = 0;
	uint8_t localTimestamp = 0;
	SES_dynTblSeqMgmt_t sequenceMgmt = dynamicTblNoSequenceOp;
	int rxSequenceSet = 0;
	int transmitFilter = SES_IGNORE_FIELD;
	int receiveFilter = SES_IGNORE_FIELD;
	int index;
	int ret;

	ret = SES_RxSesFramesByMac(macAddr, SES_REQUEST_PORT_ATTRIBUTE, adin6310_rx_callback);
	if (ret) {
		LOG_ERR("RX MAC config error\n");
		return ret;
	}

	return SES_AddStaticTableEntryEx(lookupPriority,
					macAddr,
					macMask,
					vlanId,
					vlanMask,
					sourceEntry,
					override,
					sourceOverride,
					hsrOrPrpSupervisory,
					lookupType,
					portMap,
					sendToAE,
					danp,
					cutThrough,
					syntTimestamp,
					localTimestamp,
					sequenceMgmt,
					rxSequenceSet,
					transmitFilter,
					receiveFilter,
					&index);
}

static int adin6310_get_eth_port(const struct device *dev,
				 const struct adin6310_port_config **port)
{
	const struct adin6310_config *config = dev->config;

	for (int i = 0; i < ADIN6310_NUM_PORTS; i++) {
		if (config->port_config[i].ethernet_port) {
			*port = &config->port_config[i];

			return 0;
		}
	}

	return -ENODEV;
}

static int adin6310_init(const struct device *dev)
{
        const struct adin6310_config *cfg = dev->config;
	const struct adin6310_port_config *eth_port;
	struct adin6310_data *priv = dev->data;
	sesID_t dev_id;
	int iface;
	int ret;

	SES_driverFunctions_t comm_callbacks = {
		.init_p = adin6310_spi_init,
		.release_p = adin6310_spi_remove,
		.sendMessage_p = adin6310_spi_write,
	};

        if (!spi_is_ready_dt(&cfg->spi)) {
                LOG_ERR("SPI bus %s not ready", cfg->spi.bus->name);
		return -ENODEV;
        }

        if (!gpio_is_ready_dt(&cfg->interrupt)) {
		LOG_ERR("Interrupt GPIO device %s is not ready",
			cfg->interrupt.port->name);
		return -ENODEV;
        }

        if (!gpio_is_ready_dt(&cfg->reset)) {
		LOG_ERR("Interrupt GPIO device %s is not ready",
			cfg->reset.port->name);
		return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&cfg->interrupt, GPIO_INPUT);
        if (ret) {
        	LOG_ERR("Failed to configure interrupt GPIO, %d", ret);
		return ret;
        }

        ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
        if (ret) {
        	LOG_ERR("Failed to configure reset GPIO, %d", ret);
		return ret;
        }

        gpio_pin_set_dt(&cfg->reset, 1);
        k_busy_wait(100);
        gpio_pin_set_dt(&cfg->reset, 0);
        k_busy_wait(100);

	k_sem_init(&reader_thread_sem, 0, 1);
	spi_read_tid = k_thread_create(&read_thread, stack_area,
					       10000,
					       adin6310_msg_recv, priv, NULL, NULL,
					       K_PRIO_PREEMPT(0), K_ESSENTIAL, K_FOREVER);
	k_thread_name_set(spi_read_tid, "ADIN6310");

	gpio_init_callback(&priv->gpio_int_callback, adin6310_int_rdy, BIT(cfg->interrupt.pin));
	gpio_add_callback(cfg->interrupt.port, &priv->gpio_int_callback);

	k_thread_start(spi_read_tid);

	priv->interrupt = &cfg->interrupt;
	ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret)
		return ret;

	ret = adin6310_get_eth_port(dev, &eth_port);
	if (ret)
		return ret;

	ret = SES_Init();
	if (ret) {
		LOG_ERR("SES_Init error %d\n", ret);
		return ret;
	}
	ret = SES_AddHwInterface(NULL, &comm_callbacks, &iface);
	if (ret) {
		LOG_ERR("SES_AddHwInterface error %d\n", ret);
		return ret;
	}

	ret = SES_AddDevice(iface, eth_port->mac_addr, &dev_id);
	if (ret) {
		LOG_ERR("SES_AddDevice error %d\n", ret);
		return ret;
	}

	ret = SES_MX_InitializePorts(dev_id, ADIN6310_NUM_PORTS, initializePorts_p);
	if (ret) {
		LOG_ERR("SES_MX_InitializePorts error %d\n", ret);
		return ret;
	}

	return adin6310_set_broadcast_route(dev);
}

static const struct ethernet_api adin6310_port_api = {
	.iface_api.init = adin6310_port_iface_init,
	.get_capabilities = adin6310_port_get_capabilities,
	.set_config = adin6310_port_set_config,
	.send = adin6310_port_send,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = adin6310_port_get_stats,
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
};

#define ADIN6310_SPI_OPERATION ((uint16_t)(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8)))

#define ADIN6310_STR(x)		#x

#define ADIN6310_PRIV_PORT_CONFIG(parent, inst, idx)	\
	adin6310_port_config_##parent##_##inst

#define ADIN6310_PORT_INIT(parent, inst, idx)								\
	static struct adin6310_port_data adin6310_port_data_##parent##_##inst = {0};			\
	static const struct adin6310_port_config adin6310_port_config_##parent##_##inst = {		\
		.adin = DEVICE_DT_INST_GET(parent),							\
		.id = idx,										\
		.net_device = &adin6310_data_##parent,							\
		.name = "port_" #idx,									\
		.mac_addr = DT_PROP(DT_CHILD(DT_DRV_INST(parent), inst), local_mac_address),		\
		.ethernet_port = DT_PROP(DT_CHILD(DT_DRV_INST(parent), inst), ethernet_port),		\
	}; 												\
	NET_DEVICE_INIT_INSTANCE(parent##_port_##idx, "port_" #idx, idx,				\
				 NULL, NULL, &adin6310_port_data_##parent##_##inst,			\
				 &adin6310_port_config_##parent##_##inst, CONFIG_ETH_INIT_PRIORITY,	\
				 &adin6310_port_api, ETHERNET_L2,					\
				 NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);

#define ADIN6310_DT_INST(inst)									\
	static struct adin6310_data adin6310_data_##inst = {0};					\
	static const struct adin6310_config adin6310_config_##inst;				\
	DEVICE_DT_DEFINE	(DT_DRV_INST(inst), adin6310_init, NULL,			\
				&adin6310_data_##inst, &adin6310_config_##inst,			\
				POST_KERNEL, CONFIG_ETH_INIT_PRIORITY,				\
				NULL);								\
	ADIN6310_PORT_INIT(inst, port0, 0)							\
	ADIN6310_PORT_INIT(inst, port1, 1)							\
	ADIN6310_PORT_INIT(inst, port2, 2)							\
	ADIN6310_PORT_INIT(inst, port3, 3)							\
	ADIN6310_PORT_INIT(inst, port4, 4)							\
	ADIN6310_PORT_INIT(inst, port5, 5)							\
	static const struct adin6310_config adin6310_config_##inst = {				\
		.id = ADIN6310,									\
		.spi = SPI_DT_SPEC_INST_GET(inst, ADIN6310_SPI_OPERATION, 1),			\
		.interrupt = GPIO_DT_SPEC_INST_GET(inst, int_gpios),				\
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),			\
		.port_config = {								\
			ADIN6310_PRIV_PORT_CONFIG(inst, port0, 0),				\
			ADIN6310_PRIV_PORT_CONFIG(inst, port1, 1),				\
			ADIN6310_PRIV_PORT_CONFIG(inst, port2, 2),				\
			ADIN6310_PRIV_PORT_CONFIG(inst, port3, 3),				\
			ADIN6310_PRIV_PORT_CONFIG(inst, port4, 4),				\
			ADIN6310_PRIV_PORT_CONFIG(inst, port5, 5),				\
		}										\
	};
	

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_adin6310 
DT_INST_FOREACH_STATUS_OKAY(ADIN6310_DT_INST)
