/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)


/* static int spi_write_test_msg(void)
{
	static uint8_t counter = 0;
	static uint8_t tx_buffer[2];
	static uint8_t rx_buffer[2];

	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = sizeof(tx_buffer)
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(rx_buffer),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	// Update the TX buffer with a rolling counter
	tx_buffer[0] = 0xA4;
	tx_buffer[1] = 0xA5;
	printk("SPI TX: 0x%.2x, 0x%.2x\n", tx_buffer[0], tx_buffer[1]);

	// Reset signal
	k_poll_signal_reset(&spi_done_sig);
	
	// Start transaction
	int error = spi_transceive_async(spi_dev, &spi_cfg, &tx, &rx, &spi_done_sig);
	if(error != 0){
		printk("SPI transceive error: %i\n", error);
		return error;
	}

	// Wait for the done signal to be raised and log the rx buffer
	int spi_signaled, spi_result;
	do{
		k_poll_signal_check(&spi_done_sig, &spi_signaled, &spi_result);
	} while(spi_signaled == 0);
	printk("SPI RX: 0x%.2x, 0x%.2x\n", rx_buffer[0], rx_buffer[1]);
	return 0;
} */



/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);



#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)
#define SPI1_NODE           DT_NODELABEL(my_spi_master)
void main(void)
{
	struct device * spi1_dev = DEVICE_DT_GET(MY_SPI_MASTER);
	if(!device_is_ready(spi1_dev)) {
		printk("SPI master device not ready!\n");
	}
	struct spi_cs_control spim_cs = {
		.gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master)),
		.delay = 0,
	};

    struct spi_config spi_cfg = {
        .frequency = 2000000,
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
				 SPI_MODE_CPOL | SPI_MODE_CPHA,
		.slave = 0,
		.cs = &spim_cs,
    };
	uint8_t   spi_tx_buf[24]; // spitx buffer
	uint8_t   spi_rx_buf[1]; // spirx buffer
	struct spi_buf tx_buf_arr = {.buf = spi_tx_buf, .len=sizeof(spi_tx_buf)};
    struct spi_buf_set tx = {.buffers = &tx_buf_arr, .count = 1};
	struct spi_buf rx_buf_arr = {.buf = spi_rx_buf, .len = sizeof(spi_rx_buf)};
    struct spi_buf_set rx  = {.buffers = &rx_buf_arr, .count = 1};
	spi_tx_buf[0] = 0xAE;
    spi_tx_buf[1] = 0xD5;
	spi_tx_buf[2] = 0x80;
	spi_tx_buf[3] = 0xA8;
	spi_tx_buf[4] = 63;
    spi_tx_buf[5] = 0xD3;
	spi_tx_buf[6] = 0x40 | 0x0;
	spi_tx_buf[7] = 0x10;
	spi_tx_buf[8] = 0x20;
    spi_tx_buf[9] = 0x0;
	spi_tx_buf[10] = 0xA0 | 1;
	spi_tx_buf[11] = 0xC8;
	spi_tx_buf[12] = 0xDA;
    spi_tx_buf[13] = 0x12;
	spi_tx_buf[14] = 0x81;
	spi_tx_buf[15] = 0x9F;
	spi_tx_buf[16] = 0xD9;
    spi_tx_buf[17] = 0x22;
	spi_tx_buf[18] = 0xDB;
	spi_tx_buf[19] = 0x40;
	spi_tx_buf[20] = 0xA4;
    spi_tx_buf[21] = 0xA6;
	spi_tx_buf[22] = 0x2E;
	spi_tx_buf[23] = 0xAF;
	for (int i = 0; i < 24; i++)
	{
		printk("SPI TX: 0x%.2x\n", spi_tx_buf[i]);
	}
	int error = spi_write(spi1_dev,&spi_cfg,&tx);
	spi_transceive(spi1_dev, &spi_cfg, &tx, &rx);
	if(error != 0){
		printk("SPI transceive error: %i\n", error);
	}
    while (true) {
		int error = spi_write(spi1_dev,&spi_cfg,&tx);
		if(error != 0){
			printk("SPI transceive error: %i\n", error);
		}
		error = spi_read(spi1_dev,&spi_cfg,&rx);
		if (error != 0)
		{
			printk("SPI transceive error: %i\n", error);
		}
		printk("SPI RX: 0x%.2x\n", spi_rx_buf[0]);
		k_sleep(K_MSEC(1000));

	}
}
