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
#include <zephyr/sys/printk.h>
#include <nrfx_spim.h>
#include "ADS1298.h"
#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)
#define SPI1_NODE           DT_NODELABEL(my_spi_master)

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define DREADY_PIN 5
#define T_CLK  514 // max value 514ns see p17
#define T_CSCC 0.000000006 //CS pin low to first SCLK, setup time
#define T_CSH 2*T_CLK //CS pin high pulse
#define T_SDECODE 4*T_CLK //Commande decode time
#define T_DSHD 16*T_CLK //START pin low or STOP opcode to complete current conv
#define WAIT_RST  18*T_CLK//wait 18 clock cycles
#define wait_time 4  //8th sclk falling edge
#define wait_time2 24*T_CLK //wait 24clock cycles
//define spi device
struct device * spi1_dev = DEVICE_DT_GET(MY_SPI_MASTER);
struct spi_cs_control spim_cs = {
	.gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master)),
	.delay = 0,
};

struct spi_config spi_cfg = {
	.frequency = 20000000,
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
				SPI_MODE_CPOL | SPI_MODE_CPHA,
	.slave = 0,
	.cs = &spim_cs,
};

uint8_t rx_buffer[27], tx_buffer[8];
struct spi_buf tx_buf_arr = {.buf = tx_buffer, .len= sizeof(tx_buffer)};
struct spi_buf_set tx = {.buffers = &tx_buf_arr, .count = 1};
struct spi_buf rx_buf_arr = {.buf = rx_buffer, .len = sizeof(rx_buffer)};
struct spi_buf_set rx  = {.buffers = &rx_buf_arr, .count = 1};
const struct device * dev;
uint8_t sample_number = 0;

/*******************************************************************************
 * CONFIG
*/
void set_config_ADS1298()
{
    //pc.printf("Setting Configuration\n");
    //Registres globaux
    ADS1298_send_command(WRITE_CONFIG_3_REGISTER); // must be first config reg to be set
    k_usleep(T_SDECODE);
    ADS1298_send_command(0);
    k_usleep(T_SDECODE);
    ADS1298_send_command(0xE0);    //Int ref buff|5V AVCC
    k_usleep(wait_time2);

    ADS1298_send_command(WRITE_CONFIG_1_REGISTER);
    k_usleep(T_SDECODE);
    ADS1298_send_command(0);
    k_usleep(T_SDECODE);
    ADS1298_send_command(0xC3); // 0XC3==4KSPS // HR|Readback|SPS
    k_usleep(wait_time2);

    ADS1298_send_command(WRITE_CONFIG_2_REGISTER);
    k_usleep(T_SDECODE);
    ADS1298_send_command(0);
    k_usleep(T_SDECODE);
    ADS1298_send_command(0x22); // not really used
    k_usleep(wait_time2);

/*     ADS1298_send_command(WRITE_CHANNEL_6_SET_REGISTER);
    k_usleep(T_SDECODE);
    ADS1298_send_command(0x02); // Y_BYTE_READ_WRITE
    k_msleep(wait_time);
    for (int i = 6; i <= 8; i++) {
        ADS1298_send_command(0x81); //PD+Short
        k_msleep(wait_time);
    }
    k_usleep(wait_time2);
 */
    ADS1298_send_command(WRITE_CHANNEL_1_SET_REGISTER);
    k_usleep(T_SDECODE);
    ADS1298_send_command(0x04); // X_BYTE_READ_WRITE
    k_msleep(wait_time);
    for (int i = 1; i <= 5; i++) {
        ADS1298_send_command(GAIN_ONE); //turn on channels
        k_msleep(wait_time);
    }
    //pc.printf("DONE!\n");
}
/*******************************************************************************
    Config ADS1298
*******************************************************************************/
#define RESET DT_ALIAS(led1)
static const struct gpio_dt_spec rst = GPIO_DT_SPEC_GET(RESET, gpios);

void wait_T_POR(void)
{
    for(int i=0; i<=18; i++) {
        k_usleep(T_CLK);
    }
}

void ADS1298_init()
{
	printk("In init\n");
	int err = 0; 
	int ret0;
	// configurer les pins avec les p�riph�riques, r�ussi si le retour est 0
	ret0 = gpio_pin_configure_dt(&rst, GPIO_OUTPUT_LOW);
	gpio_pin_set_dt(&rst, 0);
	wait_T_POR();
	gpio_pin_set_dt(&rst, 1);
	k_msleep(500); 
	gpio_pin_set_dt(&rst, 0);
	k_msleep(500);
    //pc.printf("WAKEUP\n");
    ADS1298_send_command(COMMAND_WAKEUP);
	k_msleep(wait_time);
    //pc.printf("RESET\n");
    k_sleep(K_SECONDS(1));
    ADS1298_send_command(COMMAND_SDATAC);
    //pc.printf("SDATAC\n");
    //get_ID();
	printk("OK until set_config\n");
    set_config_ADS1298();
    //get_config_ADS1298();

    ADS1298_send_command(SET_READ_DATA_CONTINUOUSLY);
    k_msleep(wait_time);
    ADS1298_send_command(START_RESTART_CONVERSION);
    //pc.printf("START\n");
    k_usleep(T_DSHD);
}

bool ADS1298_send_command(uint8_t command) {
	if(!device_is_ready(spi1_dev)) {
		printk("SPI master device not ready!\n");
		return false;
	}
	tx_buffer[0] = command;
	printk("TX : %d\n",tx_buffer[0]);
	int err = spi_write(spi1_dev,&spi_cfg,&tx);
	if(err < 0) {
		printk("send command err: %d\r\n", err);
		return false;
	}
	return true;
}

/* function de read bloquant */
uint8_t ADS1298_read_data() {
	spi_read(spi1_dev,&spi_cfg,&rx);
	return rx_buffer[0];
}

bool ADS1298_receive_data() {
	int err = 0; 
	int ret0;
	// configurer les pins avec les p�riph�riques, r�ussi si le retour est 0
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
        ads1298_write_register(ADS129X_REG_CH2SET, TURN_OFF_CHANNEL);
        ads1298_write_register(ADS129X_REG_CH3SET, TURN_OFF_CHANNEL);
        ads1298_write_register(ADS129X_REG_CH4SET, TURN_OFF_CHANNEL);
/*         ads1298_write_register(ADS129X_REG_CH5SET, TURN_OFF_CHANNEL);
        ads1298_write_register(ADS129X_REG_CH6SET, TURN_OFF_CHANNEL);
        ads1298_write_register(ADS129X_REG_CH7SET, TURN_OFF_CHANNEL);
        ads1298_write_register(ADS129X_REG_CH8SET, TURN_OFF_CHANNEL); */

}

void reset_ADS1298()
{
        
        gpio_pin_set_dt(&rst, 0);
        k_usleep(10);  // wait 18 * TCLK = 18 * 514ns 
        gpio_pin_set_dt(&rst, 1);
        k_msleep(500); 
        gpio_pin_set_dt(&rst, 0);
        k_msleep(500);
}


void main(void)
{
	printk("Init\n");
	ADS1298_init();
	printk("Init done\n");
	while (true) {
		
		ADS1298_read_data();
		k_usleep(110);
		for (int i = 0; i < 27; i++)
			printk("RX : 0x%.2x\n", rx_buffer[i]);
		k_sleep(K_MSEC(1000));
	} 
}