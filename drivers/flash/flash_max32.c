/*
 * Copyright (c) 2023 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT  adi_max32_flash_controller
#define ADI_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_WRITE_BLK_SZ DT_PROP(ADI_FLASH_NODE, write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(ADI_FLASH_NODE, erase_block_size)

#define FLASH_BASE DT_REG_ADDR(ADI_FLASH_NODE)
#define FLASH_SIZE DT_REG_SIZE(ADI_FLASH_NODE)

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>

#include "flc.h"

struct max32_flash_dev_data {
	struct k_sem sem;
};

static int api_read(const struct device *dev, off_t address, void *buffer, size_t length)
{
	ARG_UNUSED(dev);

	address += FLASH_BASE;
	MXC_FLC_Read(address, buffer, length);
	return 0;
}

static int api_write(const struct device *dev, off_t address, const void *buffer, size_t length)
{
	int ret = 0;
	struct max32_flash_dev_data *data = dev->data;

	k_sem_take(&data->sem, K_FOREVER);

	address += FLASH_BASE;
	ret = MXC_FLC_Write(address, length, (uint32_t *)buffer);

	k_sem_give(&data->sem);

	return ret;
}

static int api_erase(const struct device *dev, off_t start, size_t len)
{
	int ret = 0;
	struct max32_flash_dev_data *data = dev->data;
	uint32_t page_size = FLASH_ERASE_BLK_SZ;
	uint32_t addr = (start + FLASH_BASE);

	k_sem_take(&data->sem, K_FOREVER);

	while (len) {
		ret = MXC_FLC_PageErase(addr);
		if (ret) {
			break;
		}

		addr += page_size;
		if (len > page_size) {
			len -= page_size;
		} else {
			len = 0;
		}
	}

	k_sem_give(&data->sem);

	return ret;
}

#if CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_max32_pages_layout = {
	.pages_count = FLASH_SIZE / FLASH_ERASE_BLK_SZ,
	.pages_size = FLASH_ERASE_BLK_SZ,
};

static void api_page_layout(const struct device *dev, const struct flash_pages_layout **layout,
			    size_t *layout_size)
{
	*layout = &flash_max32_pages_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *api_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	static const struct flash_parameters parameters = {
		.write_block_size = FLASH_WRITE_BLK_SZ,
		.erase_value = 0xff,
	};

	return &parameters;
}

static int flash_max32_init(const struct device *dev)
{
	int ret;
	struct max32_flash_dev_data *data = dev->data;

	ret = MXC_FLC_Init();

	/* Mutex for flash controller */
	k_sem_init(&data->sem, 1, 1);

	return ret;
}

static const struct flash_driver_api flash_max32_driver_api = {
	.read = api_read,
	.write = api_write,
	.erase = api_erase,
	.get_parameters = api_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = api_page_layout,
#endif
};

static struct max32_flash_dev_data flash_max32_data;

DEVICE_DT_INST_DEFINE(0, flash_max32_init, NULL, &flash_max32_data, NULL, POST_KERNEL,
		      CONFIG_FLASH_INIT_PRIORITY, &flash_max32_driver_api);
