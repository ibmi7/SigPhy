/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include "uart_async_adapter.h"
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>

#include <zephyr/logging/log.h>

////////////////////////// SPI ///////////////////////////
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/printk.h>
#include "ADS1298.c"
#include <zephyr/sys/util.h>

#define TIMER DT_NODELABEL(rtc0)

static volatile bool flag = false;
static volatile bool flag_run = false;
int current_led = 1;

const static struct device *dev0, *dev1;

/* L'identifiant des noeuds de Devicetree pour l'alias "ledextn" n allant de 1 ? 4. */
#define LEDEXT0_NODE DT_ALIAS(ledext0)
#define LEDEXT1_NODE DT_ALIAS(ledext1) //led0
#define LEDEXT2_NODE DT_ALIAS(ledext2) //led1
#define LEDEXT3_NODE DT_ALIAS(ledext3) //led2
#define LEDEXT4_NODE DT_ALIAS(ledext4) 
#define LEDEXT5_NODE DT_ALIAS(ledext5) //led3
#define PIN4 DT_ALIAS(ledext1)
#define PIN1 DT_ALIAS(ledext2)
#define PIN2 DT_ALIAS(ledext3)
#define PIN3 DT_ALIAS(ledext5)
#if DT_NODE_HAS_STATUS(LEDEXT0_NODE, okay)
#define LEDEXT0	DT_GPIO_LABEL(LEDEXT0_NODE, gpios)
#define PIN0	DT_GPIO_PIN(LEDEXT0_NODE, gpios)
#define FLAGS0	DT_GPIO_FLAGS(LEDETX0_NODE, gpios)
K_MUTEX_DEFINE(my_mutex);
int flagenvoi = 0;
static const struct gpio_dt_spec mosfet1 = GPIO_DT_SPEC_GET(PIN4, gpios);
static const struct gpio_dt_spec mosfet2 = GPIO_DT_SPEC_GET(PIN1, gpios);
static const struct gpio_dt_spec mosfet3 = GPIO_DT_SPEC_GET(PIN2, gpios);
static const struct gpio_dt_spec mosfet4 = GPIO_DT_SPEC_GET(PIN3, gpios);
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
        ads1298_write_register(ADS129X_REG_CH2SET, GAIN_ONE);
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

/////////////////////////// SPI //////////////////////////

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 1);
static K_SEM_DEFINE(sem_spi_txrx, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#endif

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
					   data);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;

		if (disable_req) {
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		    (evt->data.rx.buf[buf->len - 1] == '\r')) {
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data);

		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
				   data);

		uart_tx(uart, &buf->data[aborted_len],
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		err = usb_enable(NULL);
		if (err && (err != -EALREADY)) {
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);


	if (IS_ENABLED(CONFIG_BT_NUS_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		k_free(rx);
		return -ENOMEM;
	}
	printk("HERE\n");
	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), 50);
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}
	
	return err;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr,
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
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		if (data[0] == 's') flag_run = !flag_run;

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}
/////////////////// SPI /////////////////////////
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. 
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);*/



#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)
#define SPI1_NODE           DT_NODELABEL(my_spi_master)
/////////////////// SPI /////////////////////////
void main(void)
{
	printk("IN MAIN printk\n");
	LOG_INF("IN MAIN LOG_INF\n");
	int retnirs1 = gpio_pin_configure_dt(&mosfet1, GPIO_OUTPUT_ACTIVE);
	int retnirs2 = gpio_pin_configure_dt(&mosfet2, GPIO_OUTPUT_ACTIVE);
	int retnirs3 = gpio_pin_configure_dt(&mosfet3, GPIO_OUTPUT_ACTIVE);
	int retnirs4 = gpio_pin_configure_dt(&mosfet4, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set_dt(&mosfet1, 0);
	k_sleep(K_MSEC(100));
	gpio_pin_set_dt(&mosfet2, 0);
	k_sleep(K_MSEC(100));
	gpio_pin_set_dt(&mosfet3, 0);
	k_sleep(K_MSEC(100));
	gpio_pin_set_dt(&mosfet4, 0);
	k_sleep(K_MSEC(10));
	int blink_status = 0;
	int err = 0;

	configure_gpio();

	err = uart_init();
	if (err) {
		error();
	}

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
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}
	//SPI init
	k_sem_give(&sem_spi_txrx);
}
	uint8_t   spi_rx_buf[1]; // spirx buffer

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);

	for(;;) {
		if(flagenvoi){
			/* Wait indefinitely for data to be sent over bluetooth */
			/* struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
								K_FOREVER); */
			/* Send the data over BLE via the NUS service */
			printk("ecg %d\n", (rx_buffer[3] << 16 | rx_buffer[4] << 8 | rx_buffer[5]));
			printk("diode %d\n", (rx_buffer[6] << 16 | rx_buffer[7] << 8 | rx_buffer[8]));
			char buf[50];
			sprintf(buf, "ecg %d\nled%d %d\n", (rx_buffer[3] << 16 | rx_buffer[4] << 8 | rx_buffer[5]),current_led, (rx_buffer[6] << 16 | rx_buffer[7] << 8 | rx_buffer[8]));
			if (bt_nus_send(NULL, buf, strlen(buf))) {
				LOG_WRN("Failed to send data over BLE connection");
			}
			k_mutex_lock(&my_mutex, K_FOREVER);
			flagenvoi = 0 ;
			k_mutex_unlock(&my_mutex);
			//k_free(buf);
		}
	}
}

void ads_thread(void){
	printk("IN ADS\n");
	k_sem_take(&sem_spi_txrx, K_FOREVER);
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
	///////////////////// NIRS ////////////////////////////////////////////
	printk("OK2\n");

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


	///////////////////// NIRS ////////////////////////////////////////////
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
          if(!flag_run && flag){
			flag = false;
			k_usleep(200);//ite marcher le led1 avec son �tat d�fini durant le temps d�fini

			// (Re)Lancer une  conversion:
			current_led = 1; 
			gpio_pin_set_dt(&mosfet3, 1);	//t1 en haut à gauche 
			//k_sleep(K_MSEC(10));
			k_usleep(350);
			err = ADS1298_send_start();
			k_usleep(350);  // 584*periode clk ads
			
			// envoyer opcode RDATA et recevoir les donn�es:
			err = ADS1298_send_read_data();
			k_usleep(10);  // 8*TSCLK
			ADS1298_receive_data();
			k_mutex_lock(&my_mutex, K_FOREVER);
			flagenvoi = 1;
			k_mutex_unlock(&my_mutex);
			k_usleep(110);
			gpio_pin_set_dt(&mosfet3, 0);	//t1 en haut à gauche 
			k_sleep(K_MSEC(2000)); 

			k_usleep(200);//ite marcher le led1 avec son �tat d�fini durant le temps d�fini

			// (Re)Lancer une  conversion: 
			current_led = 2;
			gpio_pin_set_dt(&mosfet2, 1);	//t1 en haut à gauche 
			k_usleep(350);
			err = ADS1298_send_start();
			k_usleep(350);  // 584*periode clk ads
			
			// envoyer opcode RDATA et recevoir les donn�es:
			err = ADS1298_send_read_data();
			k_usleep(10);  // 8*TSCLK
			ADS1298_receive_data();
			k_mutex_lock(&my_mutex, K_FOREVER);
			flagenvoi = 1;
			k_mutex_unlock(&my_mutex);
			k_usleep(110);
			gpio_pin_set_dt(&mosfet2, 0);	//t1 en haut à gauche 
			k_sleep(K_MSEC(2000)); 

			k_usleep(200);//ite marcher le led1 avec son �tat d�fini durant le temps d�fini

			// (Re)Lancer une  conversion: 
			current_led = 3;
			gpio_pin_set_dt(&mosfet4, 1);	//t1 en haut à gauche 
			k_usleep(350);
			err = ADS1298_send_start();
			k_usleep(350);  // 584*periode clk ads
			
			// envoyer opcode RDATA et recevoir les donn�es:
			err = ADS1298_send_read_data();
			k_usleep(10);  // 8*TSCLK
			ADS1298_receive_data();
			k_mutex_lock(&my_mutex, K_FOREVER);
			flagenvoi = 1;
			k_mutex_unlock(&my_mutex);
			k_usleep(110);
			gpio_pin_set_dt(&mosfet4, 0);	//t1 en haut à gauche 
			k_sleep(K_MSEC(2000)); 

			k_usleep(200);//ite marcher le led1 avec son �tat d�fini durant le temps d�fini

			// (Re)Lancer une  conversion: 
			current_led = 4;
			gpio_pin_set_dt(&mosfet1, 1);	//t1 en haut à gauche 
			k_usleep(350);
			err = ADS1298_send_start();
			k_usleep(350);  // 584*periode clk ads
			
			// envoyer opcode RDATA et recevoir les donn�es:
			err = ADS1298_send_read_data();
			k_usleep(10);  // 8*TSCLK
			ADS1298_receive_data();
			k_mutex_lock(&my_mutex, K_FOREVER);
			flagenvoi = 1;
			k_mutex_unlock(&my_mutex);
			k_usleep(110);
			gpio_pin_set_dt(&mosfet1, 0);	//t1 en haut à gauche 
			//k_sleep(K_MSEC(10)); 
			k_sleep(K_MSEC(2000));
			///////////////////////// NIRS ////////////////////////////////////
		  }
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(ads_thread_id, STACKSIZE, ads_thread, NULL, NULL,
		NULL, 6, 0, 0);