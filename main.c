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
#include <hal/nrf_saadc.h>
#include <zephyr/drivers/adc.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>


#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

/*
const struct device *adc_device;

#define ADC_DEVICE_NAME DT_ADC_0_NAME
#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID 0
#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN0


#define BUFFER_SIZE 1
static int16_t m_sample_buffer[BUFFER_SIZE];

static const struct adc_channel_cfg m_1st_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_1ST_CHANNEL_ID, 
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
    .input_positive = ADC_1ST_CHANNEL_INPUT,
#endif
};


const struct adc_sequence_options sequence_opts = {
    .interval_us = 0,
    .callback = NULL,
    .user_data = NULL,
    .extra_samplings = 0,
}; 

const struct adc_sequence sequence = {
	.options = &sequence_opts,
	.channels = BIT(ADC_1ST_CHANNEL_ID),
	.buffer = m_sample_buffer,
	.buffer_size = sizeof(m_sample_buffer),
	.resolution = ADC_RESOLUTION,
};


static int adc_sample(void)
{
    printk("je suis ds adc_sample\n");
	int ret;
	float adc_voltage1;
	ret = adc_read(adc_device, &sequence);
	for (int i = 0; i < BUFFER_SIZE; i++) {
		char buf[20];
		sprintf(buf, "%d", m_sample_buffer[i]);
		printk("ADC: %s\n", buf);
		adc_voltage1 = (float)(((float)m_sample_buffer[i] / 16384.0f) * 5);
		printk("ADC_voltage : %f\n", adc_voltage1);
	}

	printf("\n");
	k_msleep(100);
	return ret;

   
} 


void a_sleep(void){
	printk("je suis ds a_sleep\n");
	for (int i = 0; i < 100; i++) {
		adc_sample();
		k_msleep(100);
	}
}
*/

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)	



#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)
#define SPI1_NODE           DT_NODELABEL(my_spi_master)
#define PIN0 DT_ALIAS(led0)
#define PIN1 DT_ALIAS(led1)
#define PIN2 DT_ALIAS(led2)
#define PIN3 DT_ALIAS(led3)


static const struct gpio_dt_spec mosfet1 = GPIO_DT_SPEC_GET(PIN0, gpios);
static const struct gpio_dt_spec mosfet2 = GPIO_DT_SPEC_GET(PIN1, gpios);
static const struct gpio_dt_spec mosfet3 = GPIO_DT_SPEC_GET(PIN2, gpios);
static const struct gpio_dt_spec mosfet4 = GPIO_DT_SPEC_GET(PIN3, gpios);



void adc_sample(int err,int16_t buf, struct adc_sequence sequence){
	//printk("ADC reading:\n");
		for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
			int32_t val_mv;

			/* printk("- %s, channel %d: ",
			       adc_channels[i].dev->name,
			       adc_channels[i].channel_id); */

			(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

			err = adc_read(adc_channels[i].dev, &sequence);
			if (err < 0) {
				printk("Could not read (%d)\n", err);
				continue;
			} else {
				//printk("%"PRId16, buf);
			}

			/* conversion to mV may not be supported, skip if not */
			val_mv = buf;
			err = adc_raw_to_millivolts_dt(&adc_channels[i],
						       &val_mv);
			if (err < 0) {
				printk(" (value in mV not available)\n");
			} else {
				printk("%"PRId32"\n", val_mv);
			}
		}

		k_sleep(K_MSEC(50));
}

void main(void)
{
	int err;
	int16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};
	printk("ADC sample app\n");
	printk("Number of channels: %d\n", ARRAY_SIZE(adc_channels));
	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!device_is_ready(adc_channels[i].dev)) {
			printk("ADC controller device not ready\n");
			return;
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			printk("Could not setup channel #%d (%d)\n", i, err);
			return;
		}
	}

	printk("OK2\n");
	int ret1 = gpio_pin_configure_dt(&mosfet1, GPIO_OUTPUT_ACTIVE);
	int ret2 = gpio_pin_configure_dt(&mosfet2, GPIO_OUTPUT_ACTIVE);
	int ret3 = gpio_pin_configure_dt(&mosfet3, GPIO_OUTPUT_ACTIVE);
	int ret4 = gpio_pin_configure_dt(&mosfet4, GPIO_OUTPUT_ACTIVE);


	/* adc_device = &__device_dts_ord_75;
	adc_channel_setup(adc_device, &m_1st_channel_cfg); */
	/*
	gpio_pin_set_dt(&mosfet, 1);
	gpio_pin_set_dt(&mosfet, 0); 
	*/
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
	uint8_t   spi_tx_buf[2]; // spitx buffer
	uint8_t   spi_rx_buf[1]; // spirx buffer
	struct spi_buf tx_buf_arr = {.buf = spi_tx_buf, .len=sizeof(spi_tx_buf)};
    struct spi_buf_set tx = {.buffers = &tx_buf_arr, .count = 1};
	struct spi_buf rx_buf_arr = {.buf = spi_rx_buf, .len = sizeof(spi_rx_buf)};
    struct spi_buf_set rx  = {.buffers = &rx_buf_arr, .count = 1};
	/* int err;
	adc_dev = device_get_binding("ADC_0");
    if (!adc_dev) {
        printk("device_get_binding ADC_0 failed\n");
    }
    err = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
    if (err) {
        printk("Error in adc setup: %d\n", err);
    }
	printk("c'est fini\n"); */
	
	spi_transceive(spi1_dev, &spi_cfg, &tx, &rx);

	
	gpio_pin_set_dt(&mosfet1, 0);
	k_sleep(K_MSEC(10));
	gpio_pin_set_dt(&mosfet2, 0);
	k_sleep(K_MSEC(10));
	gpio_pin_set_dt(&mosfet3, 0);
	k_sleep(K_MSEC(10));
	gpio_pin_set_dt(&mosfet4, 0);
	k_sleep(K_MSEC(10));

	spi_tx_buf[0] = 2;
	spi_tx_buf[1] = 50;
	spi_write(spi1_dev,&spi_cfg,&tx);
	k_sleep(K_MSEC(10));
	spi_tx_buf[0] = 1;
	spi_tx_buf[1] = 50;
	spi_write(spi1_dev,&spi_cfg,&tx);
	k_sleep(K_MSEC(10));
	spi_tx_buf[0] = 3;
	spi_tx_buf[1] = 200;
	spi_write(spi1_dev,&spi_cfg,&tx);
	k_sleep(K_MSEC(10));
	spi_tx_buf[0] = 0;
	spi_tx_buf[1] = 50;
	spi_write(spi1_dev,&spi_cfg,&tx);
	k_sleep(K_MSEC(10));



	/* gpio_pin_set_dt(&mosfet1, 1);	//t1 en haut à gauche 
	k_sleep(K_MSEC(10));  

	gpio_pin_set_dt(&mosfet2, 1);	//t1 en haut à gauche 
	k_sleep(K_MSEC(10));  
	
	spi_tx_buf[0] = 0;
	spi_tx_buf[1] = 50;
	spi_write(spi1_dev,&spi_cfg,&tx);

	k_sleep(K_MSEC(10)); 
	spi_tx_buf[0] = 1;
	spi_tx_buf[1] = 80;
	spi_write(spi1_dev,&spi_cfg,&tx);  */



    while (1) {
		//gpio_pin_set_dt(&mosfet, ok);
		//printk("Etat transistor : %d\n", ok); 
		
		/* int error = spi_write(spi1_dev,&spi_cfg,&tx);
		if(error != 0){
			printk("SPI transceive error: %i\n", error);
		} */
		//a_sleep();
		//k_sleep(K_MSEC(5));

		
		//printk("LED %d SPI TX: %d\n", spi_tx_buf[0], spi_tx_buf[1]);

	gpio_pin_set_dt(&mosfet3, 1);	//t1 en haut à gauche 
	k_sleep(K_MSEC(10)); 

	/* spi_tx_buf[0] = 2;
	spi_tx_buf[1] = 50;
	spi_write(spi1_dev,&spi_cfg,&tx); */
	printk("led1\n");
	for(int i = 0; i< 4; i++){
		adc_sample(err,buf,sequence);
		K_MSEC(1);
		//printk("%d\n",i);
		 
	}
	//printk("c'est fini LED 1 \n");
	gpio_pin_set_dt(&mosfet3, 0);	//t1 en haut à gauche 
	k_sleep(K_MSEC(10)); 

	gpio_pin_set_dt(&mosfet2, 1);	//t1 en haut à gauche 
	k_sleep(K_MSEC(10));  
	
	/* spi_tx_buf[0] = 1;
	spi_tx_buf[1] = 50;
	spi_write(spi1_dev,&spi_cfg,&tx); */
	printk("led2\n");
	for(int i = 0; i< 4; i++){
		adc_sample(err,buf,sequence);
		K_MSEC(1);
		//printk("%d\n",i);  
	}
	//printk("c'est fini LED 2 \n");
	gpio_pin_set_dt(&mosfet2, 0);	//t1 en haut à gauche 
	k_sleep(K_MSEC(10));

	gpio_pin_set_dt(&mosfet4, 1);	//t1 en haut à gauche 
	k_sleep(K_MSEC(10));  
	
	/* spi_tx_buf[0] = 3;
	spi_tx_buf[1] = 200;
	spi_write(spi1_dev,&spi_cfg,&tx); */
	printk("led3\n");
	for(int i = 0; i< 4; i++){
		adc_sample(err,buf,sequence);
		K_MSEC(1);
		//printk("%d\n",i);  
	}
	//printk("c'est fini LED 2 \n");
	gpio_pin_set_dt(&mosfet4, 0);	//t1 en haut à gauche 
	k_sleep(K_MSEC(10));

	gpio_pin_set_dt(&mosfet1, 1);	//t1 en haut à gauche 
	k_sleep(K_MSEC(10));  
	
	/* spi_tx_buf[0] = 0;
	spi_tx_buf[1] = 50;
	spi_write(spi1_dev,&spi_cfg,&tx); */
	printk("led4\n");
	for(int i = 0; i< 4; i++){
		adc_sample(err,buf,sequence);
		K_MSEC(1);
		//printk("%d\n",i);  
	}
	//printk("c'est fini LED 2 \n");
	gpio_pin_set_dt(&mosfet1, 0);	//t1 en haut à gauche 
	k_sleep(K_MSEC(10));
	printk("ok\n");
		
		/* for(int i = 0; i < 225; i++)	//augmenter
		{
			spi_tx_buf[1] = 255 - i;
			//printk("LED %d SPI TX: %d\n", spi_tx_buf[0], spi_tx_buf[1]);
			int error = spi_write(spi1_dev,&spi_cfg,&tx);
			if (error != 0)
			{
				printk("SPI write error: %i\n", error);
			}
			 k_sleep(K_MSEC(10));
			 adc_sample(err,buf,sequence);
			 //a_sleep();
		} */
		/* for(int i = 30; i < 255; i++)	// diminuer
		{
			spi_tx_buf[1] = i;
			//printk("LED %d SPI TX: %d\n", spi_tx_buf[0], spi_tx_buf[1]);
			int error = spi_write(spi1_dev,&spi_cfg,&tx);
			if (error != 0)
			{
				printk("SPI write error: %i\n", error);
			}
			k_sleep(K_MSEC(10));
			adc_sample(err,buf,sequence);
			//a_sleep();
		}
	
		gpio_pin_set_dt(&mosfet1, 0); */
		//k_sleep(K_MSEC(10));   
		
		/* spi_tx_buf[0] = 1; //p2 en haut à gauche
		gpio_pin_set_dt(&mosfet2, 1); //t2 en haut à droite p0.17
		k_sleep(K_MSEC(10));

		for(int i = 0; i < 225; i++)
		{
			spi_tx_buf[1] = 255 - i;
			printk("LED 1 SPI TX: %d\n", spi_tx_buf[1]);
			int error = spi_write(spi1_dev,&spi_cfg,&tx);
			if (error != 0)
			{
				printk("SPI write error: %i\n", error);
			}
			 k_sleep(K_MSEC(20));
			a_sleep();
		}

		for(int i = 30; i < 255; i++)
		{
			spi_tx_buf[1] = i;
			printk("LED 1 SPI TX: %d\n", spi_tx_buf[1]);
			int error = spi_write(spi1_dev,&spi_cfg,&tx);
			if (error != 0)
			{
				printk("SPI write error: %i\n", error);
			}
			k_sleep(K_MSEC(20));
			a_sleep();
		}

		gpio_pin_set_dt(&mosfet2, 0);
		k_sleep(K_MSEC(10));
		spi_tx_buf[0] = 2; //p3 en bas à droite
		gpio_pin_set_dt(&mosfet3, 1); //t3 en bas à droite p0.21
		k_sleep(K_MSEC(10));

		for(int i = 0; i < 225; i++)
		{
			spi_tx_buf[1] = 255 - i;
			printk("LED 2 SPI TX: %d\n", spi_tx_buf[1]);
			int error = spi_write(spi1_dev,&spi_cfg,&tx);
			if (error != 0)
			{
				printk("SPI write error: %i\n", error);
			}
			 k_sleep(K_MSEC(20));
			a_sleep();
		}

		for(int i = 30; i < 255; i++)
		{
			spi_tx_buf[1] = i;
			printk("LED 2 SPI TX: %d\n", spi_tx_buf[1]);
			int error = spi_write(spi1_dev,&spi_cfg,&tx);
			if (error != 0)
			{
				printk("SPI write error: %i\n", error);
			}
			k_sleep(K_MSEC(20));
			a_sleep();
		}

		gpio_pin_set_dt(&mosfet3, 0);
		k_sleep(K_MSEC(10));
		spi_tx_buf[0] = 3; // p4 en bas à gauche
		gpio_pin_set_dt(&mosfet4, 1); //t4 en haut à gauche p0.28
		k_sleep(K_MSEC(10));
		for(int i = 0; i < 225; i++)
		{
			spi_tx_buf[1] = 255 - i;
			printk("LED 3 SPI TX: %d\n", spi_tx_buf[1]);
			int error = spi_write(spi1_dev,&spi_cfg,&tx);
			if (error != 0)
			{
				printk("SPI write error: %i\n", error);
			}
			k_sleep(K_MSEC(20));
			a_sleep();
		}

		for(int i = 30; i < 255; i++)
		{
			spi_tx_buf[1] = i;
			printk("LED 3 SPI TX: %d\n", spi_tx_buf[1]);
			int error = spi_write(spi1_dev,&spi_cfg,&tx);
			if (error != 0)
			{
				printk("SPI write error: %i\n", error);
			}
			k_sleep(K_MSEC(20));
			a_sleep();
		}
		
		gpio_pin_set_dt(&mosfet4, 0);
		k_sleep(K_MSEC(10)); */
		
	}
	
}
