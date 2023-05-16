/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/net/socket.h>
#include <zephyr/types.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/usb/usb_device.h>
#include <stdio.h>
#include <string.h>
#include "ADS1298.c"

#define TIMER DT_NODELABEL(rtc0)

static volatile bool flag = false;
static volatile bool flag_run = false;

const static struct device *dev0, *dev1;

/* L'identifiant des noeuds de Devicetree pour l'alias "ledextn" n allant de 1 ? 4. */
#define LEDEXT0_NODE DT_ALIAS(ledext0)
#define LEDEXT1_NODE DT_ALIAS(ledext1)
#define LEDEXT2_NODE DT_ALIAS(ledext2)
#define LEDEXT3_NODE DT_ALIAS(ledext3)
#define LEDEXT4_NODE DT_ALIAS(ledext4)
#define LEDEXT5_NODE DT_ALIAS(ledext5)
#if DT_NODE_HAS_STATUS(LEDEXT0_NODE, okay)
#define LEDEXT0	DT_GPIO_LABEL(LEDEXT0_NODE, gpios)
#define PIN0	DT_GPIO_PIN(LEDEXT0_NODE, gpios)
#define FLAGS0	DT_GPIO_FLAGS(LEDETX0_NODE, gpios)
#else
/* Sinon, une erreur de construction signifie que la carte n'est pas configur?e pour clignoter une LED.*/
#error "Unsupported board: ledext0 devicetree alias is not defined"
#define LEDEXT0	""
#define PIN0	0
#define FLAGS0	0
#endif


void reset_ADS1298()
{
        int err = 0; 
        static int ret0;
        // lier les leds avec les p�riph�riques
        dev0 = device_get_binding(LEDEXT0);

        if (dev0 == NULL ){
                printk("Error: cannot find reset pin of SPI\n");
                return;
        }
       
        // configurer les pins avec les p�riph�riques, r�ussi si le retour est 0
        ret0 = gpio_pin_configure(dev0, PIN0, GPIO_OUTPUT_ACTIVE | FLAGS0);
        if (ret0 < 0 ){
                printk("Error: pin reset SPI can't be configured\n");
                return;
        }
        gpio_pin_set(dev0, PIN0, 1);   
        k_usleep(10);  // wait 18 * TCLK = 18 * 514ns 
        gpio_pin_set(dev0, PIN0, 0);
        k_msleep(500); 
        gpio_pin_set(dev0, PIN0, 1);
        k_msleep(500);
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

}

static void test_counter_interrupt_fn(const struct device *counter_dev,
				      uint8_t chan_id, uint32_t ticks,
				      void *user_data)
{
                    struct counter_alarm_cfg *config = user_data;
                    int err;

                    flag = true;
                    //printk("blalalalalalalala,%d \n",(int)flag);

                    /* Set a new alarm with a double length duration */
                    config->ticks = config->ticks;

                    err = counter_set_channel_alarm(counter_dev, 0,
					user_data);
                    if (err != 0) {
                            printk("Alarm could not be set\n");
                    }
}




void main(void)
{
	static int ret1;
	dev1 = device_get_binding("GPIO_0");
	if (dev1 == NULL) {
		printk("Error: cannot find one or more devices\n"); 
		return;
	}
	ret1 = gpio_pin_configure(dev1, 26, GPIO_OUTPUT_ACTIVE | 0);
	if (ret1 < 0) {
		printk("Error: One or more leds can't confiigure\n");
	return;
	}
	k_usleep(1000000);
	/************************Timer Interruption*****************************/
	const struct device *counter_dev;
	counter_dev = DEVICE_DT_GET(TIMER);
	if (counter_dev == NULL) {
		printk("Device not found\n");
		return;
	}
	counter_start(counter_dev);
	struct counter_alarm_cfg alarm_cfg1;
	alarm_cfg1.flags = 0;
	alarm_cfg1.ticks = counter_us_to_ticks(counter_dev, 10000);
	alarm_cfg1.callback = test_counter_interrupt_fn;
	alarm_cfg1.user_data = &alarm_cfg1;

	int err = counter_set_channel_alarm(counter_dev, 0,&alarm_cfg1);

	printk("Set alarm in %u msec (%u ticks)\n",
		(uint32_t)(counter_ticks_to_us(counter_dev,
					alarm_cfg1.ticks) / MSEC_PER_SEC),
		alarm_cfg1.ticks);

	if (-EINVAL == err) {
	printk("Alarm settings invalid\n");
	} else if (-ENOTSUP == err) {
		printk("Alarm setting request not supported\n");
	} else if (err != 0) {
		printk("Error\n");
	}
	        /************************ADS1298 congifuration*****************************/
	 // initialisation:
	printk("ADS_init : %s\n", ADS1298_init() ? "OK" : "FAIL");
	k_msleep(10);
		// eteindre puis rallumer l'ads
	printk("ADS1298_power_off\n");
	gpio_pin_set(dev1, 26, 0);
	k_msleep(1000);
	gpio_pin_set(dev1, 26, 1);
	printk("ADS Powered ON\n");
	k_msleep(1000);
	// Faire appel � la fonction reset_ADS1298() pour faire le reset.
	reset_ADS1298();
	printk("ADS1298_reset success\n");
	k_usleep(9); // 18 * 514ns
	err = ADS1298_send_wakeup();
	printk("HERE\n");
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
	printk("ADS1298_config success\n");
	for (;;){ 
		flag = true;
          if(flag == 1){
			k_usleep(200);//ite marcher le led1 avec son �tat d�fini durant le temps d�fini

			// (Re)Lancer une  conversion: 
			err = ADS1298_send_start();
			k_usleep(350);  // 584*periode clk ads
			
			// envoyer opcode RDATA et recevoir les donn�es:
			err = ADS1298_send_read_data();
			k_usleep(10);  // 8*TSCLK
			ADS1298_receive_data();
			k_usleep(110);

			
			//Extraire les donn�es de chaque channel � partir du beffer(rx_buffer)
			//data = rxbuf2channels();
			printk("ecg %d\n", (rx_buffer[3] << 16 | rx_buffer[4] << 8 | rx_buffer[5]));
			printk("diode %d\n", (rx_buffer[6] << 16 | rx_buffer[7] << 8 | rx_buffer[8]));
		}
		k_sleep(K_MSEC(10));
	}

}
