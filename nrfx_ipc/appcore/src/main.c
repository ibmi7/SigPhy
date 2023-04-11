/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <nrfx_ipc.h>
#include <stdio.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

LOG_MODULE_REGISTER(ipc_app, LOG_LEVEL_INF);

#define IPC_DATA_MAX_SIZE 0x1000/2  ////4kB RAM should be enough
#define SHARE_RAM_BASE_ADDR (0x20080000 - IPC_DATA_MAX_SIZE * 2 )
#define MAGIC_VALID 0x20220408  
#define CH_NO_SEND 0
#define CH_NO_RECEIVE 1
#define IPC_DATA_HEADER_LEN 16
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define SPI1_NODE           DT_NODELABEL(my_spi_master)

uint8_t   spi_tx_buf[24]; // spitx buffer
	uint8_t   spi_rx_buf[1]; // spirx buffer
	struct spi_buf tx_buf_arr = {.buf = spi_tx_buf, .len=sizeof(spi_tx_buf)};
    struct spi_buf_set tx = {.buffers = &tx_buf_arr, .count = 1};
	struct spi_buf rx_buf_arr = {.buf = spi_rx_buf, .len = sizeof(spi_rx_buf)};
    struct spi_buf_set rx  = {.buffers = &rx_buf_arr, .count = 1};

//////////////////////// IPC /////////////////////////////////////
typedef struct
{
    uint32_t valid;
	uint32_t busy;     
    uint32_t len; 
    void * data;             
} nrfx_ipc_data_t;

nrfx_ipc_data_t * ipc_tx_buf = (nrfx_ipc_data_t *) SHARE_RAM_BASE_ADDR;
nrfx_ipc_data_t * ipc_rx_buf = (nrfx_ipc_data_t *) (SHARE_RAM_BASE_ADDR+IPC_DATA_MAX_SIZE);

static void nrfx_ipc_handler(uint8_t event_mask, void *p_context)
{
	LOG_INF("event_mask %d", event_mask);
	if (event_mask == CH_NO_RECEIVE) {
		// we just print out the data
		if (ipc_rx_buf->valid != MAGIC_VALID)
		{
			LOG_WRN("invalid ipc data %x", ipc_rx_buf->valid);			
		}
		else
		{
			LOG_HEXDUMP_INF(ipc_rx_buf->data, ipc_rx_buf->len, "Received: ");
			/* after processe is done, you must reset the buffer to prepare for next receive. 
			 otherwise, you cannot get the next receive */
			ipc_rx_buf->valid = 0;
			ipc_rx_buf->busy = 0;
		}
	}
}

int nrfx_ipc_send(const void *data, int size)
{
	if (size > (IPC_DATA_MAX_SIZE - IPC_DATA_HEADER_LEN) )
	{
		return -EINVAL;
	}
	if (ipc_tx_buf->valid == MAGIC_VALID && ipc_tx_buf->busy == 1)
	{
		LOG_ERR("ipc is busy");
		return -EBUSY;
	}
	ipc_tx_buf->valid = MAGIC_VALID;
	ipc_tx_buf->busy = 1;
	ipc_tx_buf->len = size;
	memcpy(ipc_tx_buf->data, data, size);
	nrfx_ipc_signal(CH_NO_SEND);
	return 0;
}

void send_to_net(void)
{
	int ret;
	static uint8_t cnt;
	char test_str[20];
	//snprintf(&spi_rx_buf[0], 1, "SPI I am from APP %c", cnt++);
	spi_rx_buf[0]=0;
	ret = nrfx_ipc_send(spi_rx_buf[0], 1);
	if (ret)
	{
		LOG_ERR("nrfx_ipc_send error %d", ret);
	}
	else
	{
		LOG_INF("sent successfully %x", cnt-1);
	}	
}

void main(void)
{
	//init spi
	struct device * spi1_dev = DEVICE_DT_GET(MY_SPI_MASTER);
	if(!device_is_ready(spi1_dev)) {
		printk("SPI master device not ready!\n");
	}
	struct spi_cs_control spim_cs = {
		.gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master)),
		.delay = 0,
	};

    struct spi_config spi_cfg = {
        .frequency = 4000000,
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
				 SPI_MODE_CPOL | SPI_MODE_CPHA,
		.slave = 0,
		.cs = &spim_cs,
    };
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

	//init ipc
	LOG_INF("dual core communication sample at %s %s", __TIME__, __DATE__);

	nrfx_ipc_init(0, nrfx_ipc_handler, NULL);
	IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_IPC), 4,
		    nrfx_isr, nrfx_ipc_irq_handler, 0);
	ipc_tx_buf->data = (void *)((uint32_t) ipc_tx_buf + IPC_DATA_HEADER_LEN);
	ipc_rx_buf->data = (void *)((uint32_t) ipc_rx_buf + IPC_DATA_HEADER_LEN);
	ipc_tx_buf->valid = 0;
	ipc_tx_buf->busy = 0;
	ipc_rx_buf->valid = 0;
	ipc_rx_buf->busy = 0;		

	nrf_ipc_send_config_set(NRF_IPC, CH_NO_SEND, 1 << CH_NO_SEND);
	nrf_ipc_receive_config_set(NRF_IPC, CH_NO_RECEIVE, 1 << CH_NO_RECEIVE);
	nrf_ipc_int_enable(NRF_IPC, 1 << CH_NO_RECEIVE);

	LOG_INF("ipc init done");

	while (1) {                
        LOG_INF("app core start to send");
        send_to_net();
		int error = spi_write(spi1_dev,&spi_cfg,&tx);
		if(error != 0){
			printk("SPI transceive error: %i\n", error);
		}
		error = spi_read(spi1_dev,&spi_cfg,&rx);
		if (error != 0)
		{
			printk("SPI transceive error: %i\n", error);
		}
		printk("SPI RX: %d\n", spi_rx_buf[0]);

        k_sleep(K_SECONDS(1)); 
	}
}
