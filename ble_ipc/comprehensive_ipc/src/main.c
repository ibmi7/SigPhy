/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <dk_buttons_and_leds.h>
#include <nrf.h>
#include <nrfx.h>
#include <zephyr/drivers/uart.h>
#ifdef CONFIG_MCUMGR_CMD_IMG_MGMT
#include "img_mgmt/img_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_CMD_OS_MGMT
#include "os_mgmt/os_mgmt.h"
#endif
#ifdef CONFIG_IPC_SMP_BT
#include "ipc_app_smp_bt.h"
#include <img_mgmt/img_mgmt_impl.h>
#endif
#ifdef CONFIG_NRF_DFU
#include "nrf_dfu_settings.h"
#include "nrf_dfu.h"
#include <zephyr/sys/reboot.h>
#include "nrf_dfu.h"
#include "nrf_dfu_validation.h"
#endif
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/pm/device.h>

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define ERASE_DELAY_AFTER_BOOT 30   //unit: s
static struct k_work_delayable blinky_work;

K_SEM_DEFINE(sem_ipc_tx, 0, 1);
K_SEM_DEFINE(sem_spi_txrx, 0, 1);

/* Stack definition for application workqueue */
K_THREAD_STACK_DEFINE(application_stack_area,
		      1024);
static struct k_work_q application_work_q;

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_CTLR(LED0_NODE, gpios)
#define LED0_PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define LED0_FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif

#ifdef CONFIG_PM_DEVICE
const struct device *devUart0;
const struct device *devUart1;
static void get_device_handles(void)
{
	devUart0 = device_get_binding("UART_0");
    devUart1 = device_get_binding("UART_1");
}

void set_device_pm_state(void)
{
	int err;
	enum pm_device_state pm_state;

	if (devUart1)
	{
		pm_device_state_get(devUart1, &pm_state);
		if (pm_state == PM_DEVICE_STATE_SUSPENDED)
		{
			LOG_INF("UART1 is in low power state. We activate it");
			err = pm_device_action_run(devUart1,PM_DEVICE_ACTION_RESUME);
			if (err) {
				LOG_ERR("UART1 enable failed");			
			}
			else
			{
				LOG_INF("## UART1 is active now ##");
			}		
		}
		else if (pm_state == PM_DEVICE_STATE_ACTIVE)
		{
			LOG_INF("UART1 is in active state. We suspend it");

#if CONFIG_UART_ASYNC_API && CONFIG_UART_1_NRF_HW_ASYNC
		((const struct uart_driver_api *)devUart1->api)->rx_disable(devUart1);
#endif			
			err = pm_device_action_run(devUart1,	PM_DEVICE_ACTION_SUSPEND);
			if (err) {
				LOG_ERR("UART1 disable failed");
			}
			else
			{
				LOG_INF("## UART1 is suspended now ##");
			}		
		}
	}

	pm_device_state_get(devUart0, &pm_state);
	if (pm_state == PM_DEVICE_STATE_SUSPENDED)
	{
		LOG_INF("UART0 is in low power state. We activate it");
		err = pm_device_action_run(devUart0,	PM_DEVICE_ACTION_RESUME);
		if (err) {
			LOG_ERR("UART0 enable failed");			
		}
		else
		{
			LOG_INF("## UART0 is active now ##");
		}		
	}
	else if (pm_state == PM_DEVICE_STATE_ACTIVE)
	{
		LOG_INF("UART0 is in active state. We suspend it");
		//print out all the pending logging messages
		while(log_process());

#if CONFIG_UART_ASYNC_API && CONFIG_UART_0_NRF_HW_ASYNC
		((const struct uart_driver_api *)devUart0->api)->rx_disable(devUart0);
#endif
		err = pm_device_action_run(devUart0,	PM_DEVICE_ACTION_SUSPEND);
		if (err) {
			LOG_ERR("UART0 disable failed");
		}
		else
		{
			LOG_INF("## UART0 is suspended now ##");
		}		
	}

}

#endif //CONFIG_PM_DEVICE

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;
	
	if (buttons & DK_BTN1_MSK) {
		LOG_INF("button1 isr");
		k_sem_give(&sem_ipc_tx);		
	}

	if (buttons & DK_BTN2_MSK) {
		LOG_INF("button2 isr");
		k_sem_give(&sem_spi_txrx);
	}

	if (buttons & DK_BTN3_MSK) {
		LOG_INF("button3 isr");
#ifdef CONFIG_PM_DEVICE		
		set_device_pm_state();
#endif		
	}			
}

const struct device *ledDev;
bool led_is_on = true;

static void blinky_work_fn(struct k_work *work)
{
    //printk("blinky fn in system workqueue\n");
	gpio_pin_set(ledDev, LED0_PIN, (int)led_is_on);
	led_is_on = !led_is_on;

	k_work_reschedule_for_queue(&application_work_q, &blinky_work,
					   	K_SECONDS(2));

}

static void assign_io_to_netcore(void)
{
	NRF_P0_S->PIN_CNF[9] = (GPIO_PIN_CNF_MCUSEL_NetworkMCU <<
					GPIO_PIN_CNF_MCUSEL_Pos);
}

#ifdef CONFIG_NRF_DFU
/**@brief Function for handling DFU events.
 */
static void dfu_observer(nrf_dfu_evt_type_t evt_type)
{
    switch (evt_type)
    {
        case NRF_DFU_EVT_DFU_STARTED:
        case NRF_DFU_EVT_OBJECT_RECEIVED:

            break;
        case NRF_DFU_EVT_DFU_COMPLETED:
        case NRF_DFU_EVT_DFU_ABORTED:
			LOG_INF("resetting...");
			while(log_process());
            sys_reboot(SYS_REBOOT_WARM);
            break;
        case NRF_DFU_EVT_TRANSPORT_DEACTIVATED:
            // Reset the internal state of the DFU settings to the last stored state.
			LOG_INF("NRF_DFU_EVT_TRANSPORT_DEACTIVATED");
            nrf_dfu_settings_reinit();
            break;
        default:
            break;
    }

}

int dfu_init(void)
{
    int ret_val;

    ret_val = nrf_dfu_settings_init(true);
    if (ret_val != NRF_SUCCESS)
	{
		LOG_WRN("dfu settings init err %d", ret_val);
	}

    ret_val = nrf_dfu_init(dfu_observer);
    if (ret_val != NRF_SUCCESS)
	{
		LOG_WRN("dfu init err %d", ret_val);
	}

    return ret_val;
}
#endif //CONFIG_NRF_DFU

void main(void)
{
	int err;	

	LOG_INF("### comprehensive example @ appcore version v0.5 compiled at %s %s\n", __TIME__, __DATE__);
	assign_io_to_netcore();

	ledDev = DEVICE_DT_GET(LED0);
	if (ledDev == NULL) {
		return;
	}

#ifdef CONFIG_PM_DEVICE
	get_device_handles();
#endif

	err = gpio_pin_configure(ledDev, LED0_PIN, GPIO_OUTPUT_ACTIVE | LED0_FLAGS);
	if (err < 0) {
		printk("gpio configure error %d \n", err);
		return;
	}

	err = dk_buttons_init(button_changed);
	if (err) {
		printk("Cannot init buttons (err: %d)", err);
	}

	k_work_queue_start(&application_work_q, application_stack_area,
			   K_THREAD_STACK_SIZEOF(application_stack_area), 10,
			   NULL);
	k_work_init_delayable(&blinky_work, blinky_work_fn);
	k_work_reschedule_for_queue(&application_work_q, &blinky_work,
					   	K_MSEC(20));	

#ifdef CONFIG_IPC_SMP_BT
	LOG_INF("## IPC SMP BT DFU example ##");	
#ifdef CONFIG_MCUMGR_CMD_OS_MGMT
	os_mgmt_register_group();
#endif	
#ifdef CONFIG_MCUMGR_CMD_IMG_MGMT
	img_mgmt_register_group();
#endif //CONFIG_MCUMGR_CMD_IMG_MGMT
#endif  //CONFIG_IPC_SMP_BT

#ifdef CONFIG_NRF_DFU
	err = dfu_init();
	if (err) {
		LOG_ERR("dfu service init err %d", err);		
	}
#endif //CONFIG_NRF_DFU

	//main thread 
	while(1)
	{
		//add your code here
		k_sleep(K_SECONDS(5));
		LOG_INF("main thread");
	}
}
