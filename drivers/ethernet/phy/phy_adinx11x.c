/*
 * Copyright (c) 2023 PHOENIX CONTACT Electronics GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(phy_adinx11x, CONFIG_PHY_LOG_LEVEL);

#define DT_DRV_COMPAT adi_adinx11x_phy

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/mdio/mdio_adinx11x.h>

/* PHYs out of reset check retry delay */
#define ADINX11X_PHY_AWAIT_DELAY_POLL_US			15U
/* Number of retries for PHYs out of reset check */
#define ADINX11X_PHY_AWAIT_RETRY_COUNT				200U

/* PHY's software powerdown check retry delay */
#define ADINX11X_PHY_SFT_PD_DELAY_POLL_US			15U
/* Number of retries for PHY's software powerdown check */
#define ADINX11X_PHY_SFT_PD_RETRY_COUNT				200U

/* PHYs autonegotiation complete timeout */
#define ADINX11X_AN_COMPLETE_AWAIT_TIMEOUT_MS			3000U

/* ADINX11X PHY identifier */
#define ADIN2111_PHY_ID						0x0283BCA1U
#define ADIN1110_PHY_ID						0x0283BC91U

/* 10BASE-T1L PMA Status Register */
#define ADINX11X_PHY_PMA_STATUS					0x000108F7U
/* Indicates PHY support of 10BASE-T1L high voltage (2.4V) tx level op mode */
#define ADINX11X_PHY_PMA_STATUS_B10L_TX_LVL_HI_ABLE		BIT(12)

/* BASE-T1 Autonegotiation Control Register */
#define ADINX11X_PHY_AN_CONTROL					0x00070200U
/* Autonegotiation Enable */
#define ADINX11X_PHY_AN_CONTROL_AN_EN				BIT(12)
/* Autonegotiation Restart */
#define ADINX11X_PHY_AN_CONTROL_AN_RESTART			BIT(9)

/* BASE-T1 Autonegotiation Status Register */
#define ADINX11X_PHY_AN_STATUS					0x00070201U
/* Autonegotiation Complete */
#define ADINX11X_PHY_AN_STATUS_AN_COMPLETE			BIT(5)
/* Link Status */
#define ADINX11X_PHY_AN_STATUS_AN_LINK_STATUS			BIT(2)

/* 10BASE-T1 Autonegotiation Advertisement Register */
#define ADINX11X_PHY_AN_ADV_ABILITY_H				0x00070204U
/* Advertise PHY capability of 2.4V tx level op mode */
#define ADINX11X_PHY_AN_ADV_ABILITY_H_B10L_TX_LVL_HI_ABL	BIT(13)
/* Advertise PHY request of 2.4V tx level op mode */
#define ADINX11X_PHY_AN_ADV_ABILITY_H_B10L_TX_LVL_HI_REQ	BIT(12)

/* System Interrupt Mask Register */
#define ADINX11X_PHY_CRSM_IRQ_MASK				0x001E0020U
/* System Interrupt Status Register */
#define ADINX11X_PHY_CRSM_IRQ_STATUS				0x001E0010U
/**
 * Mask of reserved interrupts that indicates a fatal error in the system.
 *
 * There is inconsistency between RM and ADI driver example:
 *   - RM mask 0x6FFF
 *   - ADI driver example mask 0x2BFF
 *
 * The value from the example doesn't include reserved bits 10 and 14.
 * The tests show that PHY is still functioning when bit 10 is raised.
 *
 * Here the value from ADI driver example is used instead of RM.
 */
#define ADINX11X_PHY_CRSM_IRQ_STATUS_FATAL_ERR			0x2BFFU

/* PHY Subsystem Interrupt Mask Register */
#define ADINX11X_PHY_SUBSYS_IRQ_MASK				0x001F0021U
/* PHY Subsystem Interrupt Status Register */
#define ADINX11X_PHY_SUBSYS_IRQ_STATUS				0x001F0011U
/* Link Status Change */
#define ADINX11X_PHY_SUBSYS_IRQ_STATUS_LINK_STAT_CHNG_LH	BIT(1)

/* Software Power-down Control Register */
#define ADINX11X_PHY_CRSM_SFT_PD_CNTRL				0x001E8812U
/* System Status Register */
#define ADINX11X_PHY_CRSM_STAT					0x001E8818U
/* Software Power-down Status */
#define ADINX11X_CRSM_STAT_CRSM_SFT_PD_RDY			BIT(1)

/* LED Control Register */
#define ADINX11X_PHY_LED_CNTRL					0x001E8C82U
/* LED 1 Enable */
#define ADINX11X_PHY_LED_CNTRL_LED1_EN				BIT(15)
/* LED 0 Enable */
#define ADINX11X_PHY_LED_CNTRL_LED0_EN				BIT(7)

struct phy_adinx11x_config {
	const struct device *mdio;
	uint8_t phy_addr;
	bool led0_en;
	bool led1_en;
	bool tx_24v;
};

struct phy_adinx11x_data {
	struct phy_link_state state;
	struct k_sem sem;
};

static inline int phy_adinx11x_c22_read(const struct device *dev, uint16_t reg,
					uint16_t *val)
{
	const struct phy_adinx11x_config *const cfg = dev->config;

	return mdio_read(cfg->mdio, cfg->phy_addr, reg, val);
}

static inline int phy_adinx11x_c22_write(const struct device *dev, uint16_t reg,
					 uint16_t val)
{
	const struct phy_adinx11x_config *const cfg = dev->config;

	return mdio_write(cfg->mdio, cfg->phy_addr, reg, val);
}

static inline int phy_adinx11x_c45_write(const struct device *dev, uint32_t reg,
					 uint16_t val)
{
	const struct phy_adinx11x_config *cfg = dev->config;

	return adinx11x_mdio_c45_write(cfg->mdio, cfg->phy_addr, ((reg >> 16U) & 0x1FU),
				       (reg & UINT16_MAX), val);
}

static inline int phy_adinx11x_c45_read(const struct device *dev, uint32_t reg,
					uint16_t *val)
{
	const struct phy_adinx11x_config *cfg = dev->config;

	return adinx11x_mdio_c45_read(cfg->mdio, cfg->phy_addr, ((reg >> 16U) & 0x1FU),
				      (reg & 0xFFFFU), val);
}

static int phy_adinx11x_reg_read(const struct device *dev, uint16_t reg_addr,
				 uint32_t *data)
{
	const struct phy_adinx11x_config *cfg = dev->config;
	int ret;

	mdio_bus_enable(cfg->mdio);

	ret = phy_adinx11x_c22_read(dev, reg_addr, (uint16_t *) data);

	mdio_bus_disable(cfg->mdio);

	return ret;
}

static int phy_adinx11x_reg_write(const struct device *dev, uint16_t reg_addr,
				  uint32_t data)
{
	const struct phy_adinx11x_config *cfg = dev->config;
	int ret;

	mdio_bus_enable(cfg->mdio);

	ret = phy_adinx11x_c22_write(dev, reg_addr, (uint16_t) data);

	mdio_bus_disable(cfg->mdio);

	return ret;
}

static int phy_adinx11x_await_phy(const struct device *dev)
{
	int ret;
	uint32_t count;
	uint16_t val;

	/**
	 * Port 2 PHY comes out of reset after Port 1 PHY,
	 * wait until both are out of reset.
	 * Reading Port 2 PHY registers returns 0s until
	 * it comes out from reset.
	 */
	for (count = 0U; count < ADINX11X_PHY_AWAIT_RETRY_COUNT; ++count) {
		ret = phy_adinx11x_c45_read(dev, ADINX11X_PHY_CRSM_IRQ_MASK, &val);
		if (ret >= 0) {
			if (val != 0U) {
				break;
			}
			ret = -ETIMEDOUT;
		}
		k_sleep(K_USEC(ADINX11X_PHY_AWAIT_DELAY_POLL_US));
	}

	return ret;
}

static int phy_adinx11x_an_state_read(const struct device *dev)
{
	struct phy_adinx11x_data *const data = dev->data;
	uint16_t bmsr;
	int ret;

	/* read twice to get actual link status, latch low */
	ret = phy_adinx11x_c22_read(dev, MII_BMSR, &bmsr);
	if (ret < 0) {
		return ret;
	}
	ret = phy_adinx11x_c22_read(dev, MII_BMSR, &bmsr);
	if (ret < 0) {
		return ret;
	}

	data->state.is_up = !!(bmsr & MII_BMSR_LINK_STATUS);

	return 0;
}

int phy_adinx11x_handle_phy_irq(const struct device *dev,
				struct phy_link_state *state)
{
	struct phy_adinx11x_data *const data = dev->data;
	uint16_t subsys_status;
	int ret;

	ret = phy_adinx11x_c45_read(dev, ADINX11X_PHY_SUBSYS_IRQ_STATUS,
				     &subsys_status);
	if (ret < 0) {
		return ret;
	}

	if ((subsys_status & ADINX11X_PHY_SUBSYS_IRQ_STATUS_LINK_STAT_CHNG_LH) == 0U) {
		/* nothing to process */
		return -EAGAIN;
	}

	k_sem_take(&data->sem, K_FOREVER);

	ret = phy_adinx11x_an_state_read(dev);

	memcpy(state, &data->state, sizeof(struct phy_link_state));

	k_sem_give(&data->sem);

	return ret;
}

static int phy_adinx11x_sft_pd(const struct device *dev, bool enter)
{
	int ret;
	uint32_t count;
	const uint16_t expected = enter ? ADINX11X_CRSM_STAT_CRSM_SFT_PD_RDY : 0U;
	uint16_t val;

	ret = phy_adinx11x_c45_write(dev, ADINX11X_PHY_CRSM_SFT_PD_CNTRL,
				      enter ? 1U : 0U);
	if (ret < 0) {
		return ret;
	}

	for (count = 0U; count < ADINX11X_PHY_SFT_PD_RETRY_COUNT; ++count) {
		ret = phy_adinx11x_c45_read(dev, ADINX11X_PHY_CRSM_STAT,
					     &val);
		if (ret >= 0) {
			if ((val & ADINX11X_CRSM_STAT_CRSM_SFT_PD_RDY) == expected) {
				break;
			}
			ret = -ETIMEDOUT;
		}
		k_sleep(K_USEC(ADINX11X_PHY_SFT_PD_DELAY_POLL_US));
	}

	return ret;
}

static int phy_adinx11x_id(const struct device *dev, uint32_t *phy_id)
{
	uint16_t val;

	if (phy_adinx11x_c22_read(dev, MII_PHYID1R, &val) < 0) {
		return -EIO;
	}

	*phy_id = (val & UINT16_MAX) << 16;

	if (phy_adinx11x_c22_read(dev, MII_PHYID2R, &val) < 0) {
		return -EIO;
	}

	*phy_id |= (val & UINT16_MAX);

	return 0;
}

static int phy_adinx11x_get_link_state(const struct device *dev,
				       struct phy_link_state *state)
{
	struct phy_adinx11x_data *const data = dev->data;

	k_sem_take(&data->sem, K_FOREVER);

	memcpy(state, &data->state, sizeof(struct phy_link_state));

	k_sem_give(&data->sem);

	return 0;
}

static int phy_adinx11x_cfg_link(const struct device *dev,
				 enum phy_link_speed adv_speeds)
{
	ARG_UNUSED(dev);

	if (!!(adv_speeds & LINK_FULL_10BASE_T)) {
		return 0;
	}

	return -ENOTSUP;
}

static int phy_adinx11x_init(const struct device *dev)
{
	const struct phy_adinx11x_config *const cfg = dev->config;
	struct phy_adinx11x_data *const data = dev->data;
	uint32_t phy_id;
	uint16_t val;
	bool tx_24v_supported = false;
	int ret;

	data->state.is_up = false;
	data->state.speed = LINK_FULL_10BASE_T;

	ret = phy_adinx11x_await_phy(dev);
	if (ret < 0) {
		LOG_ERR("PHY %u didn't come out of reset, %d",
			cfg->phy_addr, ret);
		return -ENODEV;
	}

	ret = phy_adinx11x_id(dev, &phy_id);
	if (ret < 0) {
		LOG_ERR("Failed to read PHY %u ID, %d",
			cfg->phy_addr, ret);
		return -ENODEV;
	}

	if (phy_id != ADIN2111_PHY_ID && phy_id != ADIN1110_PHY_ID) {
		LOG_ERR("PHY %u unexpected PHY ID %X", cfg->phy_addr, phy_id);
		return -EINVAL;
	}

	LOG_INF("PHY %u ID %X", cfg->phy_addr, phy_id);

	/* enter software powerdown */
	ret = phy_adinx11x_sft_pd(dev, true);
	if (ret < 0) {
		return ret;
	}

	/* disable interrupts */
	ret = phy_adinx11x_c45_write(dev, ADINX11X_PHY_CRSM_IRQ_MASK, 0U);
	if (ret < 0) {
		return ret;
	}

	/* enable link status change irq */
	ret = phy_adinx11x_c45_write(dev, ADINX11X_PHY_SUBSYS_IRQ_MASK,
				     ADINX11X_PHY_SUBSYS_IRQ_STATUS_LINK_STAT_CHNG_LH);
	if (ret < 0) {
		return ret;
	}

	/* clear PHY IRQ status before enabling ADIN IRQs */
	ret = phy_adinx11x_c45_read(dev, ADINX11X_PHY_CRSM_IRQ_STATUS, &val);
	if (ret < 0) {
		return ret;
	}

	if (val & ADINX11X_PHY_CRSM_IRQ_STATUS_FATAL_ERR) {
		LOG_ERR("PHY %u CRSM reports fatal system error", cfg->phy_addr);
		return -ENODEV;
	}

	ret = phy_adinx11x_c45_read(dev, ADINX11X_PHY_SUBSYS_IRQ_STATUS, &val);
	if (ret < 0) {
		return ret;
	}

	if (!cfg->led0_en || !cfg->led1_en) {
		ret = phy_adinx11x_c45_read(dev, ADINX11X_PHY_LED_CNTRL, &val);
		if (ret < 0) {
			return ret;
		}
		if (!cfg->led0_en) {
			val &= ~(ADINX11X_PHY_LED_CNTRL_LED0_EN);
		}
		if (!cfg->led1_en) {
			val &= ~(ADINX11X_PHY_LED_CNTRL_LED1_EN);
		}
		ret = phy_adinx11x_c45_write(dev, ADINX11X_PHY_LED_CNTRL, val);
		if (ret < 0) {
			return ret;
		}
	}

	/* check 2.4V support */
	ret = phy_adinx11x_c45_read(dev, ADINX11X_PHY_PMA_STATUS, &val);
	if (ret < 0) {
		return ret;
	}

	tx_24v_supported = !!(val & ADINX11X_PHY_PMA_STATUS_B10L_TX_LVL_HI_ABLE);

	LOG_INF("PHY %u 2.4V mode %s", cfg->phy_addr,
		tx_24v_supported ? "supported" : "not supported");

	if (!cfg->tx_24v & tx_24v_supported) {
		LOG_ERR("PHY %u 2.4V mode supported, but not enabled",
			cfg->phy_addr);
	}

	/* config 2.4V auto-negotiation */
	ret = phy_adinx11x_c45_read(dev, ADINX11X_PHY_AN_ADV_ABILITY_H,
					&val);
	if (ret < 0) {
		return ret;
	}

	if (tx_24v_supported) {
		val |= ADINX11X_PHY_AN_ADV_ABILITY_H_B10L_TX_LVL_HI_ABL;
	} else {
		val &= ~ADINX11X_PHY_AN_ADV_ABILITY_H_B10L_TX_LVL_HI_ABL;
	}

	if (cfg->tx_24v) {
		if (!tx_24v_supported) {
			LOG_ERR("PHY %u 2.4V mode enabled, but not supported",
				cfg->phy_addr);
			return -EINVAL;
		}

		val |= ADINX11X_PHY_AN_ADV_ABILITY_H_B10L_TX_LVL_HI_REQ;
	} else {
		val &= ~ADINX11X_PHY_AN_ADV_ABILITY_H_B10L_TX_LVL_HI_REQ;
	}

	ret = phy_adinx11x_c45_write(dev, ADINX11X_PHY_AN_ADV_ABILITY_H,
				     val);
	if (ret < 0) {
		return ret;
	}

	/* enable auto-negotiation */
	ret = phy_adinx11x_c45_write(dev, ADINX11X_PHY_AN_CONTROL,
				     ADINX11X_PHY_AN_CONTROL_AN_EN);
	if (ret < 0) {
		return ret;
	}

	/**
	 * done, PHY is in software powerdown (SFT PD)
	 * exit software powerdown, PHY 1 has to exit before PHY 2
	 * correct PHY order is expected to be in DTS to guarantee that
	 */
	return phy_adinx11x_sft_pd(dev, false);
}

static int phy_adinx11x_link_cb_set(const struct device *dev, phy_callback_t cb,
				    void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
	return -ENOTSUP;
}

static const struct ethphy_driver_api phy_adinx11x_api = {
	.get_link = phy_adinx11x_get_link_state,
	.cfg_link = phy_adinx11x_cfg_link,
	.link_cb_set = phy_adinx11x_link_cb_set,
	.read = phy_adinx11x_reg_read,
	.write = phy_adinx11x_reg_write,
};

#define ADINX11X_PHY_INITIALIZE(n)						\
	static const struct phy_adinx11x_config phy_adinx11x_config_##n = {	\
		.mdio = DEVICE_DT_GET(DT_INST_BUS(n)),				\
		.phy_addr = DT_INST_REG_ADDR(n),				\
		.led0_en = DT_INST_PROP(n, led0_en),				\
		.led1_en = DT_INST_PROP(n, led1_en),				\
		.tx_24v = !(DT_INST_PROP(n, disable_tx_mode_24v)),		\
	};									\
	static struct phy_adinx11x_data phy_adinx11x_data_##n = {		\
		.sem = Z_SEM_INITIALIZER(phy_adinx11x_data_##n.sem, 1, 1),	\
	};									\
	DEVICE_DT_INST_DEFINE(n, &phy_adinx11x_init, NULL,			\
			      &phy_adinx11x_data_##n, &phy_adinx11x_config_##n, \
			      POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,		\
			      &phy_adinx11x_api);

DT_INST_FOREACH_STATUS_OKAY(ADINX11X_PHY_INITIALIZE)
