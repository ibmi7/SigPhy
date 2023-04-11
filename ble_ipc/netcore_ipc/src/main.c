/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <zephyr/settings/settings.h>
#include <stdio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#ifdef CONFIG_IPC_SMP_BT
#include "ipc_net_smp_bt.h"
#endif
#include "ipc_net_api.h"
#include "ipc_lib.h"

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK


static K_SEM_DEFINE(ble_conn_ok, 0, 1);

struct bt_conn *current_conn;
// static struct bt_conn *auth_conn;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/*
static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};
*/


K_SEM_DEFINE(sem_nus_op, 0, 1);

const struct device *gpio_dev;
#define EXT_INT_IO	9  //p0.09
#define EXT_INT_CONF	(GPIO_INPUT | GPIO_PULL_UP)
static struct gpio_callback ext_int_cb_data;

#ifdef CONFIG_BT_GATT_CLIENT
static struct bt_gatt_exchange_params exchange_params;

static void exchange_func(struct bt_conn *conn, uint8_t att_err,
			  struct bt_gatt_exchange_params *params)
{
	if (att_err)
	{
		LOG_ERR("MTU exchange failed");
	}
	else
	{
		LOG_INF("MTU updated to %d", bt_gatt_get_mtu(conn));		
	}
	
}
#endif

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];	

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", (addr));

	current_conn = bt_conn_ref(conn);

	k_sem_give(&ble_conn_ok);

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	if (bt_conn_set_security(conn, BT_SECURITY_L2)) {
		LOG_INF("Failed to set security");
	}	
#endif

	err = net2app_send_conn_status(1);
	if (err) {
		LOG_ERR("send connected err %d", err);
	}	

#ifdef CONFIG_BT_GATT_CLIENT		
	exchange_params.func = exchange_func;
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_ERR("MTU exchange failed");
	} 		
#endif
	
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", (addr), reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;		
	}

	int err = net2app_send_conn_status(0);
	if (err) {
		LOG_ERR("send disconnected err %d", err);
	}	

}

static void bt_nus_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{	
	char addr[BT_ADDR_LE_STR_LEN] = {0};
	int err;	

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", (addr));
	LOG_HEXDUMP_INF(data, len, "data:");

	err = net2app_send_nus((uint8_t *)data, len);
	if (err) {
		LOG_ERR("network to application core nus send err %d", err);
	}	

}

static struct bt_nus_cb nus_cb = {
	.received = bt_nus_receive_cb,
};

void ext_int_isr(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	LOG_INF("button4 pressed and going to send nus packet\n");
    k_sem_give(&sem_nus_op);    
}

void setup_ext_int(void)
{
	int err;

	gpio_dev = device_get_binding(DT_NODE_FULL_NAME(DT_NODELABEL(gpio0)));
	if (gpio_dev == NULL) {
		printk("GPIO_0 bind error");
		return;
	}
	err = gpio_pin_configure(gpio_dev, EXT_INT_IO,
				EXT_INT_CONF);
	if (err) {
		printk("GPIO_0 config error: %d", err);
		return;
	}

	err = gpio_pin_interrupt_configure(gpio_dev, EXT_INT_IO,
					   GPIO_INT_EDGE_FALLING);
	if (err) {
		printk("GPIO_0 enable callback error: %d", err);
	}

	gpio_init_callback(&ext_int_cb_data, ext_int_isr,
			BIT(EXT_INT_IO));
	err = gpio_add_callback(gpio_dev, &ext_int_cb_data);
	if (err) {
		printk("GPIO_0 add callback error: %d", err);
		return;
	}
}

static int send_to_app(void)
{	
	int ret;
	static uint8_t cnt;
	char test_str[20];

	test_str[0] = 0x1;
	snprintf(&test_str[1], 16, "I am from NET %c", cnt++);
	ret = net2app_test(test_str, 16);
	if (ret)
	{
		LOG_ERR("ipc test error %d", ret);
	}
	else
	{
		LOG_DBG("ipc test done %x", cnt-1);
	}

	return ret;	
}


#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", (addr),
			level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", (addr),
			level, err);
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
// static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
// {
// 	char addr[BT_ADDR_LE_STR_LEN];

// 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// 	LOG_INF("Passkey for %s: %06u", (addr), passkey);
// }

// static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
// {
// 	char addr[BT_ADDR_LE_STR_LEN];

// 	auth_conn = bt_conn_ref(conn);

// 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// 	LOG_INF("Passkey for %s: %06u", (addr), passkey);
// 	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
// }


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", (addr));
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", (addr),
		bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", (addr),
		reason);
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = NULL, //auth_passkey_display,
	.passkey_confirm = NULL, //auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

// static void num_comp_reply(bool accept)
// {
// 	if (accept) {
// 		bt_conn_auth_passkey_confirm(auth_conn);
// 		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
// 	} else {
// 		bt_conn_auth_cancel(auth_conn);
// 		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
// 	}

// 	bt_conn_unref(auth_conn);
// 	auth_conn = NULL;
// }

// void button_changed(uint32_t button_state, uint32_t has_changed)
// {
// 	uint32_t buttons = button_state & has_changed;

// 	if (auth_conn) {
// 		if (buttons & KEY_PASSKEY_ACCEPT) {
// 			num_comp_reply(true);
// 		}

// 		if (buttons & KEY_PASSKEY_REJECT) {
// 			num_comp_reply(false);
// 		}
// 	}
// }

#else
static struct bt_conn_auth_cb conn_auth_callbacks;
#endif

void main(void)
{
	
	int err = 0;
	
	LOG_INF("### netcore firmware compiled at %s %s", __TIME__, __DATE__);
	
	setup_ext_int();

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			printk("Failed to register authorization callbacks.\n");
			return;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			printk("Failed to register authorization info callbacks.\n");
			return;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_INF("bt enable error %d", err);		
		return;
	}
	
#ifdef CONFIG_IPC_SMP_BT
	/* Initialize the Bluetooth mcumgr transport. */
	smp_bt_register_ipc();
#endif

	LOG_INF("Bluetooth initialized");	

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL,
			      0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
	}

	LOG_INF("Started NUS example on netcore");

	for (;;) {		
		k_sleep(K_SECONDS(5));
		LOG_INF("netcore main");
		send_to_app();	
	}
}

void ble_write_thread(void)
{
	static char data[20];
	static uint8_t cnt;
	int err;

	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_conn_ok, K_FOREVER);
	cnt = 0;

	for (;;) {
		k_sem_take(&sem_nus_op, K_FOREVER);		
		snprintf(data, 12, "HelloNet%d", cnt++);		
		data[11] = 0;		
		err = bt_nus_send(NULL, data, 12);
		if (err) {
			LOG_WRN("bt_nus_send err %d", err);
		}
			
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);
