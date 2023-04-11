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
#include <nrfx_spim.h>
#include "ADS1298.h"
#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)
#define SPI1_NODE           DT_NODELABEL(my_spi_master)

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define DREADY_PIN 5
//define spi device
struct device * spi1_dev = DEVICE_DT_GET(MY_SPI_MASTER);
struct spi_cs_control spim_cs = {
	.gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master)),
	.delay = 0,
};

struct spi_config spi_cfg = {
	.frequency = NRF_SPIM_FREQ_2M,
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
				SPI_MODE_CPOL | SPI_MODE_CPHA,
	.slave = 0,
	.cs = &spim_cs,
};

uint8_t rx_buffer[27], tx_buffer[8];
struct spi_buf tx_buf_arr = {.buf = tx_buffer, .len=sizeof(tx_buffer)};
struct spi_buf_set tx = {.buffers = &tx_buffer, .count = 1};
struct spi_buf rx_buf_arr = {.buf = rx_buffer, .len = sizeof(rx_buffer)};
struct spi_buf_set rx  = {.buffers = &rx_buffer, .count = 1};
const struct device * dev;
uint8_t sample_number = 0;


bool ADS1298_send_command(uint8_t command) {
	if(!device_is_ready(spi1_dev)) {
		printk("SPI master device not ready!\n");
		return false;
	}
	tx_buffer[0] = command;

	int err = spi_write(spi1_dev,&spi_cfg,&tx);
	if(err < 0) {
		printk("send command err: %d\r\n", err);
		return false;
	}
	return true;
}

/* function de read bloquant */
uint8_t ADS1298_read_data() {
	while( gpio_pin_get(dev, DREADY_PIN) == 1 );
	spi_read(spi1_dev,&spi_cfg,&rx);
	return rx_buffer[0];
}

bool ADS1298_receive_data() {
        spi_read(spi1_dev,&spi_cfg,&rx);
        return true;
}

/* bool ADS1298_poll_new_data(ADS1298_data_t *data) {
    if(new_data_available) {
        new_data_available = false;
        state = processing_data;

        *data = ADS1298_convert_data();
       
        state = idle;
        return true;
    } else {
        return false;
    }
} */



/* bool get_spi_busy() {
    return state != idle;
} */



uint8_t ads1298_read_ID()             
{
                ADS1298_send_command(0x20);// OPCODE1 = 0x20 = 0b0010 0000 : 001 = commande READ RREG |  0 0000 = Adresse du registe ID 
                k_usleep(64);
		ADS1298_send_command(0x00); // OPCODE2 = number of register -1 = 0= 0x00
                k_usleep(64);
	        return ADS1298_read_data();  
}


bool ads1298_write_command(uint8_t val)
{	
	ADS1298_send_command(val);
	return true;	
}

bool ads1298_write_register(uint8_t reg, uint8_t val)
{
    ads1298_write_multiple_register(reg, &val, 1);
    return true;
}

bool ads1298_write_multiple_register(uint8_t reg, uint8_t* val, uint8_t num)
  {	
		ADS1298_send_command(ADS129X_CMD_WREG | (reg & 0x1F));
                k_usleep(64);
		ADS1298_send_command((num - 1) & 0x1F);
                k_usleep(64);
	
		for(int i = 0; i < num; i++)
                    {
                      ADS1298_send_command(*(val + i));
                      k_usleep(64);
                    }

		return true;	
  }
  
bool ADS1298_send_start() {
    return ADS1298_send_command(COMMAND_START);
}
bool ADS1298_send_stop() {
    return ADS1298_send_command(COMMAND_STOP);
}
bool ADS1298_send_read_continuous() {
    return ADS1298_send_command(COMMAND_RDATAC);
}

bool ADS1298_send_read_data() {
    return ADS1298_send_command(COMMAND_RDATA);
}

bool ADS1298_send_stop_read_continuous() {
    return ADS1298_send_command(COMMAND_SDATAC);
}

bool ADS1298_send_reset() {
    return ADS1298_send_command(COMMAND_RESET);
}

bool ADS1298_send_wakeup() {
    return ADS1298_send_command(COMMAND_WAKEUP);
}


void config_ADS1298()
{
	// Ajouter les fonction pour configurer ADS1298 
	ads1298_write_register(ADS129X_REG_CONFIG1, 0x00); // HR mode + 500 SPS
	ads1298_write_register(ADS129X_REG_CONFIG2, 0x00); // default register
	ads1298_write_register(ADS129X_REG_CONFIG3, 0xE0); // internal power ref = 4 V
	ads1298_write_register(ADS129X_REG_CONFIG4, 0x00); // default register



	// GAIN_ONE = 0x10 pour fixer le gain de PGA � 1 ( PGA = amplificateur � gain programmable )
	ads1298_write_register(ADS129X_REG_CH1SET, GAIN_ONE);
	ads1298_write_register(ADS129X_REG_CH2SET, GAIN_ONE);
	ads1298_write_register(ADS129X_REG_CH3SET, GAIN_ONE);
	ads1298_write_register(ADS129X_REG_CH4SET, GAIN_ONE);
	ads1298_write_register(ADS129X_REG_CH5SET, TURN_OFF_CHANNEL);
	ads1298_write_register(ADS129X_REG_CH6SET, TURN_OFF_CHANNEL);
	ads1298_write_register(ADS129X_REG_CH7SET, TURN_OFF_CHANNEL);
	ads1298_write_register(ADS129X_REG_CH8SET, TURN_OFF_CHANNEL);

}

#define PIN_RST 26

void reset_ADS1298()
{
        int err = 0; 
        static int ret0;
        const struct device *dev0 = device_get_binding("GPIO_0");

        if (dev0 == NULL ){
                printk("Error: cannot find reset pin of SPI\n");
                return;
        }
        // configurer les pins avec les p�riph�riques, r�ussi si le retour est 0
        ret0 = gpio_pin_configure(dev0, PIN_RST, GPIO_OUTPUT_ACTIVE);

        gpio_pin_set(dev0, PIN_RST, 1);   
        k_usleep(10);  // wait 18 * TCLK = 18 * 514ns 
        gpio_pin_set(dev0, PIN_RST, 0);
        k_msleep(500); 
        gpio_pin_set(dev0, PIN_RST, 1);
        k_msleep(500);
}



void main(void)
{
	// get the GPIO device
	dev = device_get_binding("DReady");
	if (dev == NULL) {
		printk("Error: didn't find GPIO device\n");
		return;
	}
	// configure the LED pin as output
	//gpio_pin_configure(dev, DREADY_PIN, GPIO_INPUT);
	printk("ADS1298 DREADY pin configured\n");
 	reset_ADS1298();
	/*printk("ADS1298 reset\n");
	k_usleep(9); // 18 * 514ns
	int err = ADS1298_send_wakeup();
	if (err == true){printk("ADS1298_send_wakeup success\n");}
	else{printk("ADS1298_send_wakeup failed \n");}
	k_msleep(1000);

	// cette commande est necessaire avant de lire des registre datasheet ADS1298 page 64 
	err = ADS1298_send_stop_read_continuous();
	if (err == true){printk("ADS1298_send_stop_read_continuous success\n");}
	else{printk("ADS1298_send_stop_read_continuous failed \n");}
	k_msleep(500);

	// lire le registre ID:
	uint8_t regid = ads1298_read_ID();
	printk("ADS1298_ID %x\n", regid); // ID = 0x92 datasheet page 66
	k_msleep(1000);

	// Configurer ADS1298
	config_ADS1298 ();
	while (true) {
		// (Re)Lancer une  conversion: 
		err = ADS1298_send_start();
		k_usleep(350);  // 584*periode clk ads
		
		// envoyer opcode RDATA et recevoir les donn�es:
		err = ADS1298_send_read_data();
		k_usleep(10);  // 8*TSCLK
		ADS1298_receive_data();
		for (int i = 0; i < 27; i++){
			printk("%4x\n", rx_buffer[i]);
		}
		k_usleep(110);
	} */
}
