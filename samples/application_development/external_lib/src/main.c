/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* hello world example: calling functions from a static library */


#include <zephyr/kernel.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <stdio.h>
#include <string.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

#define SPI_MESSAGE 0xA5
#define SPI1_NODE   DT_NODELABEL(spidev)

const struct spi_config spi_cfg = {
    .frequency = 2000000u,
    .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
    .cs = SPI_CS_CONTROL_INIT(DT_NODELABEL(spi1), 10),
};

uint8_t get_pec_byte(uint8_t data, uint8_t seed)
{
	uint8_t pec = seed;
	uint8_t din, in0, in1, in2;
	int bit;
	for(bit=7; bit>=0; bit--)
	{
		din = (data>>bit) & 0x01;
		in0 = din ^ ( (pec>>7) & 0x01 );
		in1 = in0 ^ ( pec & 0x01);
		in2 = in0 ^ ( (pec>>1) & 0x01 );
		pec = (pec << 1);
		pec &= ~(0x07);
		pec = pec | in0 | (in1<<1) | (in2<<2);
	}
	return pec;
}

void ltc4296_read(uint8_t reg, uint16_t *data)
{
    const struct spi_dt_spec dev_spi = SPI_DT_SPEC_GET(SPI1_NODE, SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_GET(0), 0);

    uint8_t w_buf[5] = {0};
    uint8_t r_buf[5] = {0};

    const struct spi_buf tx_buf = { .buf = w_buf, .len = 5 };
	const struct spi_buf rx_buf = { .buf = r_buf, .len = 5};
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1U };
	const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1U };

    w_buf[0] = reg << 1 | 0x1;
    w_buf[1] = get_pec_byte(w_buf[0], 0x41);

    // printf ("first buf %x second buf %x", w_buf[0], w_buf[1]);

    spi_transceive_dt(&dev_spi, &tx, &rx);

    *data = r_buf[2] << 8 | r_buf[3];
}

void ltc4296_write(uint8_t reg, uint16_t data)
{
    const struct spi_dt_spec dev_spi = SPI_DT_SPEC_GET(SPI1_NODE, SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_GET(0), 0);

    uint8_t w_buf[5] = {0};

    const struct spi_buf tx_buf = { .buf = w_buf, .len = 5 };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1U };

    w_buf[0] = reg << 1 | 0x0;
    w_buf[1] = get_pec_byte(w_buf[0], 0x41);
    w_buf[2] = data >> 8;
    w_buf[3] = data & 0xFF;
    w_buf[4] = get_pec_byte(w_buf[3], get_pec_byte(w_buf[2], 0x41));

    spi_write_dt(&dev_spi, &tx);
}

void main(void)
{
    uint16_t data;
    int comp_data;
    
    ltc4296_write(0x8, 0x7300);
    k_sleep(K_MSEC(1000));

    ltc4296_write(0x8, 0x0005);
    ltc4296_read(0x8, &data);

    printf("Check if unlocked. Read value %x\n", data);

    ltc4296_write(0x44, 0x0109);
    ltc4296_write(0x43, 0x00E1);

    ltc4296_write(0x0A, 0x0041);

    ltc4296_read(0x0B, &data);

    printf("Get ADC raw data (gadcdat) value: %x\n", data);

    comp_data = ((data & 0xFFF) - 2049) * 35230 / 1000000;

    printf("Get ADC computed data (gadcdat) value: %d\n", comp_data);

    ltc4296_read(0x46, &data);

    printf("Get ADC raw data (p3adcdat) %x\n", data);

    comp_data = ((data & 0xFFF) - 2049) / 15;

    printf("Get ADC computed data (p3adcdat) value: %d\n", comp_data);
}
