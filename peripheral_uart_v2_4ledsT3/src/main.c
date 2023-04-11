/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include "uart_async_adapter.h"

#include <net/socket.h>
#include <kernel.h>
#include <zephyr/types.h>
#include <zephyr.h>
#include <drivers/uart.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <drivers/counter.h>
#include <usb/usb_device.h>

#include <device.h>
#include <devicetree.h>
#include <soc.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/gap.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <settings/settings.h>

#include <stdio.h>
#include <string.h>

#include <logging/log.h>

#include <hal/nrf_saadc.h>

#include <sys/printk.h>
//#include <hal/nrf_gpio.h>

#include <sys/util.h>
#include <inttypes.h>

#include "ADS1298.c"
#include <nrfx_systick.h>



#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL1 50
#define RUN_LED_BLINK_INTERVAL2 50

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

#define INTERVAL_MIN	0x140	/* 0x140 : 320 units, 400 ms -> 0x014 : 20 units*/
#define INTERVAL_MAX	0x140	/* 320 units, 400 ms */

static struct bt_le_conn_param *conn_param =
	BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, 0, 400);

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static const struct device *uart;
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

const struct device *adc_dev;
const static struct device *dev0, *dev1, *dev2, *dev3, *dev4, *dev5;
static uint16_t res_adc = 300, tempo;
static volatile bool flag = false;
static volatile bool flag_run = false;
static int blink_status = 0;
static int cpt2 = 0;

/* L'identifiant des noeuds de Devicetree pour l'alias "ledextn" n allant de 1 ? 4. */
#define LEDEXT0_NODE DT_ALIAS(ledext0)
#define LEDEXT1_NODE DT_ALIAS(ledext1)
#define LEDEXT2_NODE DT_ALIAS(ledext2)
#define LEDEXT3_NODE DT_ALIAS(ledext3)
#define LEDEXT4_NODE DT_ALIAS(ledext4)
#define LEDEXT5_NODE DT_ALIAS(ledext5)

/* si l'identifiant de led existe, on prend les information configur?es du led */
#if DT_NODE_HAS_STATUS(LEDEXT0_NODE, okay)
#define LEDEXT0	DT_GPIO_LABEL(LEDEXT0_NODE, gpios)
#define PIN0	DT_GPIO_PIN(LEDEXT0_NODE, gpios)
#define FLAGS0	DT_GPIO_FLAGS(LEDETX0_NODE, gpios)
#else
/* Sinon, une erreur de construction signifie que la carte n'est pas configur?e pour clignoter une LED.*/
#error "Unsupported board: ledext0 devicetree alias is not defined"
#define LEDETX0	""
#define PIN0	0
#define FLAGS0	0
#endif

#if DT_NODE_HAS_STATUS(LEDEXT1_NODE, okay)
#define LEDEXT1	DT_GPIO_LABEL(LEDEXT1_NODE, gpios)
#define PIN1	DT_GPIO_PIN(LEDEXT1_NODE, gpios)
#define FLAGS1	DT_GPIO_FLAGS(LEDETX1_NODE, gpios)
#else
/* Sinon, une erreur de construction signifie que la carte n'est pas configur?e pour clignoter une LED.*/
#error "Unsupported board: ledext1 devicetree alias is not defined"
#define LEDETX1	""
#define PIN1	0
#define FLAGS1	0
#endif

#if DT_NODE_HAS_STATUS(LEDEXT2_NODE, okay)
#define LEDEXT2	DT_GPIO_LABEL(LEDEXT2_NODE, gpios)
#define PIN2	DT_GPIO_PIN(LEDEXT2_NODE, gpios)
#define FLAGS2	DT_GPIO_FLAGS(LEDETX2_NODE, gpios)
#else
#error "Unsupported board: ledext2 devicetree alias is not defined"
#define LEDETX2	""
#define PIN2	0
#define FLAGS2	0
#endif

#if DT_NODE_HAS_STATUS(LEDEXT3_NODE, okay)
#define LEDEXT3	DT_GPIO_LABEL(LEDEXT3_NODE, gpios)
#define PIN3	DT_GPIO_PIN(LEDEXT3_NODE, gpios)
#define FLAGS3	DT_GPIO_FLAGS(LEDETX3_NODE, gpios)
#else
#error "Unsupported board: ledext3 devicetree alias is not defined"
#define LEDETX3	""
#define PIN3	0
#define FLAGS3	0
#endif

#if DT_NODE_HAS_STATUS(LEDEXT4_NODE, okay)
#define LEDEXT4	DT_GPIO_LABEL(LEDEXT4_NODE, gpios)
#define PIN4	DT_GPIO_PIN(LEDEXT4_NODE, gpios)
#define FLAGS4	DT_GPIO_FLAGS(LEDETX4_NODE, gpios)
#else
#error "Unsupported board: ledext4 devicetree alias is not defined"
#define LEDETX4	""
#define PIN4	0
#define FLAGS4	0
#endif


#if DT_NODE_HAS_STATUS(LEDEXT5_NODE, okay)
#define LEDEXT5	DT_GPIO_LABEL(LEDEXT5_NODE, gpios)
#define PIN5	DT_GPIO_PIN(LEDEXT5_NODE, gpios)
#define FLAGS5	DT_GPIO_FLAGS(LEDETX5_NODE, gpios)
#else
#error "Unsupported board: ledext4 devicetree alias is not defined"
#define LEDETX5	""
#define PIN5	0
#define FLAGS5	0
#endif

#define DELAY 10000     //10ms
#define ALARM_CHANNEL_ID 0

struct counter_alarm_cfg alarm_cfg1;

#if defined(CONFIG_BOARD_ATSAMD20_XPRO)
#define TIMER DT_LABEL(DT_NODELABEL(tc4))
#elif defined(CONFIG_SOC_FAMILY_SAM)
#define TIMER DT_LABEL(DT_NODELABEL(tc0))
#elif defined(CONFIG_COUNTER_RTC0)
#define TIMER DT_LABEL(DT_NODELABEL(rtc0))
#elif defined(CONFIG_COUNTER_RTC_STM32)
#define TIMER DT_LABEL(DT_INST(0, st_stm32_rtc))
#elif defined(CONFIG_COUNTER_NATIVE_POSIX)
#define TIMER DT_LABEL(DT_NODELABEL(counter0))
#elif defined(CONFIG_COUNTER_XLNX_AXI_TIMER)
#define TIMER DT_LABEL(DT_INST(0, xlnx_xps_timer_1_00_a))
#elif defined(CONFIG_COUNTER_ESP32)
#define TIMER DT_LABEL(DT_NODELABEL(timer0))
#endif

// d�finir les information de ADC0
#define ADC_DEVICE_NAME DT_ADC_0_NAME
#define ADC_RESOLUTION 14
#define ADC_RESOLUTION2 14
#define OVER_SAMPLING 8
#define ADC_GAIN ADC_GAIN_1_6 
#define ADC_REFERENCE ADC_REF_INTERNAL
//#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_1ST_CHANNEL_ID 0
#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN0
#define ADC_2ND_CHANNEL_ID 2
#define ADC_2ND_CHANNEL_INPUT NRF_SAADC_INPUT_AIN2

#define BUFFER_SIZE 1
static uint16_t m_sample_buffer1[BUFFER_SIZE];
static uint16_t m_sample_buffer2[BUFFER_SIZE];
/*
const struct adc_sequence_options sequence_opts = {
	.interval_us = 0,
	.callback = NULL,
	.user_data = NULL,
	.extra_samplings = 7,
};
*/
// configurer un adc avec son premier channel = AIN0 (P0.4)
static struct adc_channel_cfg m_1st_channel_cfg = {
	.gain = ADC_GAIN_2,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive = ADC_1ST_CHANNEL_INPUT,
#endif
};

static struct adc_channel_cfg m_2nd_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_2ND_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive = ADC_2ND_CHANNEL_INPUT,
#endif
};

static uint16_t adc_sample(void)
{
	int ret;
        //uint32_t accum = 0;
        //uint16_t res = 0;

        // d�finir la s�quence d'�chantillonage du channel 1 (AIN0)
	const struct adc_sequence sequence1 = {
                //.options = &sequence_opts,
		.channels = BIT(ADC_1ST_CHANNEL_ID),
		.buffer = m_sample_buffer1,
		.buffer_size = sizeof(m_sample_buffer1),
		.resolution = ADC_RESOLUTION2,
                .oversampling = OVER_SAMPLING,
                //.calibrate = true,
	};

	if (!adc_dev) {
		return -1;
	}
/*
        for(int i=0; i<4; i++){
                ret = adc_read(adc_dev, &sequence1);
                accum += (uint32_t)m_sample_buffer1[0];      
        }

        accum = accum/4;

        res = (uint16_t)accum;*/
 
        ret = adc_read(adc_dev, &sequence1);

        //printk("%u\n", m_sample_buffer1[0]); // valeur analogique

        /* Affichage la valeur tension de AIN0 */
	//for (int i = 0; i < BUFFER_SIZE; i++) {
		float adc_voltage1 = 0;
                float T;
 //               uint8_t LSB, HSB;
 //               uint16_t res;
 //               HSB = (uint8_t)( (m_sample_buffer1[i]>>8)&0x0003 );
 //               LSB = (uint8_t)m_sample_buffer1[i];
 //               res = (uint16_t)HSB*256+(uint16_t)LSB;
		adc_voltage1 = (float)(((float) m_sample_buffer1[0] / 4096.0f) * 3.0f); // convertir le valeur analogique en num�rique
                adc_voltage1 = adc_voltage1 - 0.066f;

                T = ( adc_voltage1 / (3.0f - adc_voltage1) * 69.0f - 1) * 1000.0f / 3.9083f;
 //               printk("%u", res); // valeur analogique
		printk("ADC raw value for channel 1: %d\n", m_sample_buffer1[0]); // valeur analogique
		//printf("Measured voltage AIN0: %f V\n", adc_voltage1); // valeur num�rique
                //printf("Measured T : %f �C\n", T); // valeur num�rique
	//}

	return m_sample_buffer1[0];
        //return res;
}

static uint16_t adc_sample2(void)
{
	int ret;

        // d�finir la s�quence d'�chantillonage du channel 2 (AIN2)
        const struct adc_sequence sequence2 = {
		.channels = BIT(ADC_2ND_CHANNEL_ID),
		.buffer = m_sample_buffer2,
		.buffer_size = sizeof(m_sample_buffer2),
		.resolution = ADC_RESOLUTION,
                .oversampling = OVER_SAMPLING,
	};

	if (!adc_dev) {
		return -1;
	}

        ret = adc_read(adc_dev, &sequence2);

		float adc_voltage2 = 0;

		adc_voltage2 = (float)(((float) m_sample_buffer2[0] / 16384.0f) * 3000.0f); // convertir le valeur analogique en num�rique
 //               printk("%u", res); // valeur analogique
		printk("ADC raw value for channel 2: %x\n", m_sample_buffer2[0]); // valeur analogique
		printf("Measured voltage AIN2: %f mV\n", adc_voltage2); // valeur num�rique

	return m_sample_buffer2[0];
}


/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;

int mode = 1;

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
        int err;
        //printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
        printk("Button pressed at %" PRIu32 "\n", k_uptime_get_32());

        switch(mode){
                case 0 :
                m_1st_channel_cfg.gain = ADC_GAIN_1_3;
                mode++;
                printk("ADC gain = 1/3\n");
                break;

                case 1 :
                m_1st_channel_cfg.gain = ADC_GAIN_1;
                mode++;
                printk("ADC gain = 1\n");
                break;

                case 2 :
                m_1st_channel_cfg.gain = ADC_GAIN_2;
                mode++;
                printk("ADC gain = 2\n");
                break;

                case 3:
                m_1st_channel_cfg.gain = ADC_GAIN_4;
                mode = 0;
                printk("ADC gain = 4\n");
                break;
        }

        err = adc_channel_setup(adc_dev, &m_1st_channel_cfg); // configurer le premier channel du adc
	if (err) {
		printk("Error in adc setup 1st channel: %d\n", err);
	}
}

#if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#endif

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static uint8_t *current_buf;
	static size_t aborted_len;
	static bool buf_release;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("tx_done");
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
		LOG_DBG("rx_rdy");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;
		buf_release = false;

		if (buf->len == UART_BUF_SIZE) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
			  (evt->data.rx.buf[buf->len - 1] == '\r')) {
			k_fifo_put(&fifo_uart_rx_data, buf);
			current_buf = evt->data.rx.buf;
			buf_release = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("rx_disabled");
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
		LOG_DBG("rx_buf_request");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("rx_buf_released");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data);
		if (buf_release && (current_buf != evt->data.rx_buf.buf)) {
			k_free(buf);
			buf_release = false;
			current_buf = NULL;
		}

		break;

	case UART_TX_ABORTED:
			LOG_DBG("tx_aborted");
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

	uart = device_get_binding(CONFIG_BT_NUS_UART_DEV);
	if (!uart) {
		return -ENXIO;
	}

	if (IS_ENABLED(CONFIG_USB)) {
		err = usb_enable(NULL);
		if (err) {
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
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	return uart_rx_enable(uart, rx->data, sizeof(rx->data), 50);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", log_strdup(addr));
/*
        k_usleep(500000);
        dk_set_led_on(CON_STATUS_LED);
        gpio_pin_set(dev1, PIN1, 1);
        gpio_pin_set(dev2, PIN2, 1);
        k_usleep(1000000);
        dk_set_led_off(CON_STATUS_LED);
        gpio_pin_set(dev1, PIN1, 0);
        gpio_pin_set(dev2, PIN2, 0);
*/
	current_conn = bt_conn_ref(conn);

        //******************************SL : configure PHY to 2M******************************
        struct bt_conn_le_phy_param *phy;
        struct bt_conn_le_data_len_param *data_length;

        phy = BT_CONN_LE_PHY_PARAM_2M;

        data_length = BT_LE_DATA_LEN_PARAM_MAX;
        //data_length->tx_max_time = BT_GAP_DATA_TIME_DEFAULT;

        err = bt_conn_le_phy_update(current_conn, phy);
	if (err) {
		printk("\nFailed to configure PHY param\n");
	}
        //
/*
/*
        //******************************SL : configure conn param******************************
        err = bt_conn_le_param_update(current_conn, conn_param);
        if (err) {
                printk("\nFailed to configure conn param\n");
	}
        //

        //SL : configure data length
        err = bt_conn_le_data_len_update(current_conn, data_length);
	if (err) {
		printk("\nFailed to configure data length\n");
		}
        //
*/
	//dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", log_strdup(addr), reason);

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
		LOG_INF("Security changed: %s level %u", log_strdup(addr),
			level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", log_strdup(addr),
			level, err);
	}
}
#endif

static struct bt_conn_cb conn_callbacks = {
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

	LOG_INF("Passkey for %s: %06u", log_strdup(addr), passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", log_strdup(addr), passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", log_strdup(addr));
}


static void pairing_confirm(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	bt_conn_auth_pairing_confirm(conn);

	LOG_INF("Pairing confirmed: %s", log_strdup(addr));
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", log_strdup(addr),
		bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", log_strdup(addr),
		reason);
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
	.pairing_confirm = pairing_confirm,
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
        int count = 0;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", log_strdup(addr));

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return;
		}
                count++;
                //printk("count=%u ", count);

                if(data[0] == 'g')  flag_run = !flag_run;    //SL : start to send data

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
                if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}
                //printk("tx->length=%u ", tx->len);
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
		/* <sd v dsd d  gg>n for ever */
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

                    err = counter_set_channel_alarm(counter_dev, ALARM_CHANNEL_ID,
					user_data);
                    if (err != 0) {
                            printk("Alarm could not be set\n");
                    }
}

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
        ads1298_write_register(ADS129X_REG_CH3SET, GAIN_ONE);
        ads1298_write_register(ADS129X_REG_CH4SET, GAIN_ONE);
        ads1298_write_register(ADS129X_REG_CH5SET, TURN_OFF_CHANNEL);
        ads1298_write_register(ADS129X_REG_CH6SET, TURN_OFF_CHANNEL);
        ads1298_write_register(ADS129X_REG_CH7SET, TURN_OFF_CHANNEL);
        ads1298_write_register(ADS129X_REG_CH8SET, TURN_OFF_CHANNEL);

}


/* cette fonction permet de recuperer les datas de chaque chennel � partir du beffer */
data_8chs_t rxbuf2channels()
{
          data_8chs_t data;

          data.channel_1 = rx_buffer[3] << 16 | rx_buffer[4] << 8 | rx_buffer[5];
          data.channel_2 = rx_buffer[6] << 16 | rx_buffer[7] << 8 | rx_buffer[8];
          data.channel_3 = rx_buffer[9] << 16 | rx_buffer[10] << 8 | rx_buffer[11];
          data.channel_4 = rx_buffer[12] << 16 | rx_buffer[13] << 8 | rx_buffer[14];
          data.channel_5 = rx_buffer[15] << 16 | rx_buffer[16] << 8 | rx_buffer[17];
          data.channel_6 = rx_buffer[18] << 16 | rx_buffer[19] << 8 | rx_buffer[20];
          data.channel_7 = rx_buffer[21] << 16 | rx_buffer[22] << 8 | rx_buffer[23];
          data.channel_8 = rx_buffer[24] << 16 | rx_buffer[25] << 8 | rx_buffer[26];


          return data;
} 


void main(void)
{
	//static int blink_status = 0;
	int err = 0;

	configure_gpio();
        //nrf_gpio_cfg_output(17);

        //*************************************************************************************
        // d�finit l'�tat de led en allum�
	bool led_is_on1 = false;
	bool led_is_on2 = false;
	bool led_is_on3 = false;
	bool led_is_on4 = false;
	bool led_is_on5 = false;

        static int ret1, ret2, ret3, ret4, ret5;

        // lier les leds avec les p�riph�riques
        dev1 = device_get_binding(LEDEXT1);
        dev2 = device_get_binding(LEDEXT2);
        dev3 = device_get_binding(LEDEXT3);
        dev4 = device_get_binding(LEDEXT4);
	dev5 = device_get_binding(LEDEXT5);

	if (dev1 == NULL || dev2 == NULL || dev3 == NULL || dev4 == NULL || dev5==NULL) {
                printk("Error: cannot find one or more devices\n"); 
		return;
	}

        // configurer les pins avec les p?riph?riques, r?ussi si le retour est 0
	ret1 = gpio_pin_configure(dev1, PIN1, GPIO_OUTPUT_ACTIVE | FLAGS1);
	ret2 = gpio_pin_configure(dev2, PIN2, GPIO_OUTPUT_ACTIVE | FLAGS2);
	ret3 = gpio_pin_configure(dev3, PIN3, GPIO_OUTPUT_ACTIVE | FLAGS3);
	ret4 = gpio_pin_configure(dev4, PIN4, GPIO_OUTPUT_ACTIVE | FLAGS4);
	ret5 = gpio_pin_configure(dev5, PIN5, GPIO_OUTPUT_ACTIVE | FLAGS5);

	if (ret1 < 0 || ret2 < 0 || ret3 < 0 || ret4 < 0 || ret5<0) {
                printk("Error: One or more leds can't confiigure\n");
		return;
	}

        gpio_pin_set(dev1, PIN1, 1);
        gpio_pin_set(dev2, PIN2, 1);
        gpio_pin_set(dev3, PIN3, 1);
        gpio_pin_set(dev4, PIN4, 1);
        gpio_pin_set(dev5, PIN5, 1);

        //*************************************************************************************


        k_usleep(1000000);

	err = uart_init();
	if (err) {
		error();
	}
        
	bt_conn_cb_register(&conn_callbacks);

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		bt_conn_auth_cb_register(&conn_auth_callbacks);
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

        /********************button******************************/
        if (!device_is_ready(button.port)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return;
	}

	err = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (err != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       err, button.port->name, button.pin);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			err, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);
        /********************ADC******************************/


 //       printk("nRF53 SAADC sampling AIN0 (P0.04) et AIN2 (P0.06)\n");

	//adc_dev = device_get_binding("ADC_0"); // lier le adc avec le p��riph��rique
	//if (!adc_dev) {
	//	printk("device_get_binding ADC_0 failed\n");
	//}
	//err = adc_channel_setup(adc_dev, &m_1st_channel_cfg); // configurer le premier channel du adc
	//if (err) {
	//	printk("Error in adc setup 1st channel: %d\n", err);
	//}

 //       err = adc_channel_setup(adc_dev, &m_2nd_channel_cfg);
	//if (err) {
	//	printk("Error in adc setup 2nd channel: %d\n", err);
	//}

        /************************Timer Interruption*****************************/
        const struct device *counter_dev;
        counter_dev = device_get_binding(TIMER);
        if (counter_dev == NULL) {
		printk("Device not found\n");
		return;
	}

        counter_start(counter_dev);

	alarm_cfg1.flags = 0;
	alarm_cfg1.ticks = counter_us_to_ticks(counter_dev, DELAY);
	alarm_cfg1.callback = test_counter_interrupt_fn;
	alarm_cfg1.user_data = &alarm_cfg1;

        err = counter_set_channel_alarm(counter_dev, ALARM_CHANNEL_ID,
					&alarm_cfg1);

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
	err = ADS1298_init();
        if (err == true){printk("SPI init ADS1298_init success\n");}
        else{printk("SPI init ADS1298_init failed \n");}
         
        k_msleep(10);

        // Faire appel � la fonction reset_ADS1298() pour faire le reset.
        reset_ADS1298();
        k_usleep(9); // 18 * 514ns

        err = ADS1298_send_wakeup();
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

        /**************************************************************************/
 
        dk_set_led(RUN_STATUS_LED, 1);
        /* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);
        //struct bt_conn_info info = {0};//SL
        int cpt = 0;
        //head of the send packet
        uint8_t dummy[244];
        dummy[0] = 0xff;
        dummy[1] = 0xff;
        dummy[2] = 0xff;
        uint32_t time = 0;
        uint32_t time_std = 0;
        uint32_t temp_t = 0;

        /* Calibrage de d�calage de d�clenchement
         * Comme cela g�n�re un �v�nement _Done et _result
         * Le premier r�sultat sera incorrect.
         */
	NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
        k_usleep(20000);

        float Temp1 = 0.0;
        int32_t Temp2 = 0;
        int32_t Temp3 = 0;
        int32_t Temp_t = 0;
        uint16_t T1;
        data_8chs_t data;

	for (;;){ 

                //dk_set_led(RUN_STATUS_LED, (blink_status) % 2);
                //printk("flag=%d \n",(int)flag);
          while(!flag_run) {
                    k_sleep(K_SECONDS(1));
                    time_std = k_uptime_get_32();  
          }

          if(flag == 1){
                    flag = false;

                        /*turn on led1*/
			gpio_pin_set(dev1, PIN1, 0); // mettre le led1 marcher avec son �tat d�fini
                	//led_is_on1 = true; // d�finir l'�tat de led1 en �teint
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

          dummy[cpt*36+3] = rx_buffer[3];  //MSB
          dummy[cpt*36+4] = rx_buffer[4];
          dummy[cpt*36+5] = rx_buffer[5];   //LSB

          dummy[cpt*36+6] = rx_buffer[6];  //MSB
          dummy[cpt*36+7] = rx_buffer[7];
          dummy[cpt*36+8] = rx_buffer[8];   //LSB

                        //k_usleep(100);

			/*turn off led1 and switch to led2*/
			gpio_pin_set(dev1, PIN1, 1);
                	led_is_on1 = false;
                	k_usleep(400);

			gpio_pin_set(dev2, PIN2, 0);
                	led_is_on2 = true;
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

          dummy[cpt*36+9] = rx_buffer[3];  //MSB
          dummy[cpt*36+10] = rx_buffer[4];
          dummy[cpt*36+11] = rx_buffer[5];   //LSB

          dummy[cpt*36+12] = rx_buffer[6];  //MSB
          dummy[cpt*36+13] = rx_buffer[7];
          dummy[cpt*36+14] = rx_buffer[8];   //LSB

                        //k_usleep(100);

			/*turn off led2 and swith to led3*/
			gpio_pin_set(dev2, PIN2, 1);
	                //led_is_on2 = false;               
                        k_usleep(400);

                        /*turn on led3*/
			gpio_pin_set(dev3, PIN3, 0); // mettre le led1 marcher avec son �tat d�fini
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

          dummy[cpt*36+15] = rx_buffer[3];  //MSB
          dummy[cpt*36+16] = rx_buffer[4];
          dummy[cpt*36+17] = rx_buffer[5];   //LSB

          dummy[cpt*36+18] = rx_buffer[6];  //MSB
          dummy[cpt*36+19] = rx_buffer[7];
          dummy[cpt*36+20] = rx_buffer[8];   //LSB

                        //k_usleep(100);

			/*turn off led3 and switch to led4*/
			gpio_pin_set(dev3, PIN3, 1);
                	k_usleep(400);


                        /*turn on led4*/
			gpio_pin_set(dev4, PIN4, 0); // mettre le led1 marcher avec son �tat d�fini
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

          dummy[cpt*36+21] = rx_buffer[3];  //MSB
          dummy[cpt*36+22] = rx_buffer[4];
          dummy[cpt*36+23] = rx_buffer[5];   //LSB

          dummy[cpt*36+24] = rx_buffer[6];  //MSB
          dummy[cpt*36+25] = rx_buffer[7];
          dummy[cpt*36+26] = rx_buffer[8];   //LSB

                        //k_usleep(100);

			/*turn off led4 and switch to led5*/
			gpio_pin_set(dev4, PIN4, 1);
                	k_usleep(400);


                        /*turn on led5*/
			gpio_pin_set(dev5, PIN5, 0); // mettre le led1 marcher avec son �tat d�fini
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

          dummy[cpt*36+27] = rx_buffer[3];  //MSB
          dummy[cpt*36+28] = rx_buffer[4];
          dummy[cpt*36+29] = rx_buffer[5];   //LSB

          dummy[cpt*36+30] = rx_buffer[6];  //MSB
          dummy[cpt*36+31] = rx_buffer[7];
          dummy[cpt*36+32] = rx_buffer[8];   //LSB

                        //k_usleep(330);

			/*turn off led5*/
			gpio_pin_set(dev5, PIN5, 1);
                	k_usleep(300);

                        /*Dark*/

          // (Re)Lancer une  conversion: 
          err = ADS1298_send_start();
          k_usleep(350);  // 584*periode clk ads
          
          // envoyer opcode RDATA et recevoir les donn�es:
          err = ADS1298_send_read_data();
          k_usleep(10);  // 8*TSCLK
          ADS1298_receive_data();
          k_usleep(110);

          
          //Extraire les donn�es de chaque channel � partir du beffer(rx_buffer)
          data = rxbuf2channels();

          dummy[cpt*36+33] = rx_buffer[3];  //MSB
          dummy[cpt*36+34] = rx_buffer[4];
          dummy[cpt*36+35] = rx_buffer[5];   //LSB

          dummy[cpt*36+36] = rx_buffer[6];  //MSB
          dummy[cpt*36+37] = rx_buffer[7];
          dummy[cpt*36+38] = rx_buffer[8];   //LSB

          Temp2 = Temp2 + data.channel_3;
          Temp3 = Temp3 + data.channel_4;

          k_usleep(100);

         cpt++;

          if(cpt == 5){
                            cpt = 0;
                            bt_nus_send(current_conn, dummy, 193);

                            
                            time = k_uptime_get_32();
                            time = time - time_std;
                            temp_t = time >> 24;
                            dummy[183] = (uint8_t)temp_t;
                            temp_t = time >> 16;
                            dummy[184] = (uint8_t)temp_t;
                            temp_t = time >> 8;
                            dummy[185] = (uint8_t)temp_t;
                            dummy[186] = (uint8_t)time;
                            //Temperature1
                            Temp2 = Temp2 / 5;
                            Temp_t = Temp2 >> 16;
                            dummy[187] = (uint8_t)Temp_t;
                            Temp_t = Temp2 >> 8;
                            dummy[188] = (uint8_t)Temp_t;
                            dummy[189] = (uint8_t)Temp2;
                            Temp2 = 0;
                            Temp_t = 0;
                             //Temperature2
                            Temp3 = Temp3 / 5;
                            Temp_t = Temp3 >> 16;
                            dummy[190] = (uint8_t)Temp_t;
                            Temp_t = Temp3 >> 8;
                            dummy[191] = (uint8_t)Temp_t;
                            dummy[192] = (uint8_t)Temp3;
                            Temp3 = 0;
                            Temp_t = 0;
          }

          // attendre avant de lancer une nouvelle converssion:
          //k_msleep(100);

                //    gpio_pin_set(dev1, PIN1, (int)led_is_on1); // mettre le led1 marcher avec son ��tat d��fini
                //    led_is_on1 = ! led_is_on1;

                //    for(int i=0; i<8; i++){
                //            T1 = adc_sample();
                //            Temp1 = Temp1 + (float) T1;
                //    }

                //    Temp1 = Temp1 / 8.0f;
                //    //T1 = adc_sample();
                //    //Temp1 = (float) T1;

                //    printf("raw data : %f\n", Temp1);

                //    Temp1 = (Temp1 / 16384.0f) * 0.25f; // convertir le valeur analogique en num�rique
                //    Temp1 = Temp1 + 0.003f;
                //    printf("Current : %f A\n", Temp1);

                //    Temp1 = ( Temp1 / (3.0f - Temp1) * 69.0f - 1) * 1000.0f / 3.9083f;

                //    printf("Measured T : %f C\n", Temp1);
                //    Temp1 = 0.0;
                

                    //res_adc = adc_sample2();   //adc2

   //                     /*turn on led1*/
			//gpio_pin_set(dev1, PIN1, (int)led_is_on1); // mettre le led1 marcher avec son ��tat d��fini
   //             	led_is_on1 = false; // d��finir l'��tat de led1 en ��teint
   //             	k_usleep(500);//ite marcher le led1 avec son ��tat d��fini durant le temps d��fini  

			///*turn off led1 and switch to led2*/
			//gpio_pin_set(dev1, PIN1, (int)led_is_on1);
   //             	led_is_on1 = true;
   //             	k_usleep(500);

			//gpio_pin_set(dev2, PIN2, (int)led_is_on2);
   //             	led_is_on2 = false;
   //             	k_usleep(500);

			///*turn off led2 and swith to led3*/
			//gpio_pin_set(dev2, PIN2, (int)led_is_on2);
	  //              led_is_on2 = true;               
   //                     k_usleep(500);

			//gpio_pin_set(dev3, PIN3, (int)led_is_on3);
			//led_is_on3 = false;
			//k_usleep(100);
                    
			///*turn off led3 and switch to led4*/

			//gpio_pin_set(dev3, PIN3, (int)led_is_on3);
   //             	led_is_on3 = true;
   //            		k_usleep(500);

			//gpio_pin_set(dev4, PIN4, (int)led_is_on4);
   //             	led_is_on4 = false;
   //             	k_usleep(500);

			///*turn off led4 and switch to led5*/
			//gpio_pin_set(dev4, PIN4, (int)led_is_on4);
   //             	led_is_on4 = true;
   //             	k_usleep(500);

			//gpio_pin_set(dev5, PIN5, (int)led_is_on5);
   //             	led_is_on5 = false;
   //             	k_usleep(500);

			///*switch off led5*/
			//gpio_pin_set(dev5, PIN5, (int)led_is_on5);
   //             	led_is_on5 = true;
   //             	k_usleep(500);

/*
                    res_adc = adc_sample();   //adc
                    tempo = res_adc >> 8;
                    dummy[cpt*4+2] = (uint8_t)tempo;
                    dummy[cpt*4+3] = (uint8_t)res_adc;

                    k_usleep(220);

                    res_adc = adc_sample();
                    tempo = res_adc >> 8;
                    dummy[cpt*4+4] = (uint8_t)tempo;
                    dummy[cpt*4+5] = (uint8_t)res_adc;
*/
/*                    gpio_pin_set(dev1, PIN1, (++blink_status) % 2); // turn on LED1
                    k_usleep(RUN_LED_BLINK_INTERVAL1);
                    res_adc = adc_sample();   //adc
                    tempo = res_adc >> 8;
                    dummy[cpt*4+2] = (uint8_t)tempo;
                    dummy[cpt*4+3] = (uint8_t)res_adc;
                    gpio_pin_set(dev1, PIN1, (++blink_status) % 2); // turn off LED1

                    k_usleep(170);

                    gpio_pin_set(dev2, PIN2, (++blink_status) % 2);
                    k_usleep(RUN_LED_BLINK_INTERVAL1);
                    res_adc = adc_sample();
                    tempo = res_adc >> 8;
                    dummy[cpt*4+4] = (uint8_t)tempo;
                    dummy[cpt*4+5] = (uint8_t)res_adc;
                    gpio_pin_set(dev2, PIN2, (++blink_status) % 2);

                    cpt++;
                    //printk("%u %u ", dummy[0], dummy[1]);
                    if(cpt == 40){
                            cpt = 0;
                            bt_nus_send(current_conn, dummy, 166);

                            time = k_uptime_get_32();
                            time = time - time_std;
                            temp_t = time >> 24;
                            dummy[162] = (uint8_t)temp_t;
                            temp_t = time >> 16;
                            dummy[163] = (uint8_t)temp_t;
                            temp_t = time >> 8;
                            dummy[164] = (uint8_t)temp_t;
                            dummy[165] = (uint8_t)time;
                    }*/
                }


                /*		if (err) {
			//printk("Error in adc sampling: %d\n", err);
		}*/
                //tempo = (res_adc & 0x03ff) >> 8;
                
                //printk("%u ", res_adc); // valeur analogique
                //nrf_gpio_pin_set(17);
                //nrf_gpio_pin_clear(17);
/*
                //SL: get conn info
                err = bt_conn_get_info(current_conn, &info);
                if (err) {
                        printk("\nfail to get conn info\n");
                        //return err;
                }

                printk("The LE tx PHY is : %u \n", info.le.phy->tx_phy);
                printk("The LE rx PHY is : %u \n", info.le.phy->rx_phy);
                printk("The LE conn is : %u (%u us)\n", info.le.interval, info.le.interval*1250);
                printk("The LE tx data length is : %u \n", info.le.data_len->tx_max_len);
                printk("The LE tx available MTU is: %u \n", bt_nus_get_mtu(current_conn));
                printk("The LE rx data length is : %u \n", info.le.data_len->rx_max_len);
                printk("The LE data payload time : %u us\n\n", info.le.data_len->tx_max_time);
        //*/
	}
}
/*
void ble_write_thread(void)
{    

        int err;
        /* Don't go any further until BLE is initialized */
//	k_sem_take(&ble_init_ok, K_FOREVER);

/*
        static char dummy[244];
        uint64_t stamp;
        int64_t delta;
        uint32_t data = 244000;
        int j;

        for (int i=0; i<30; i++)
        {
                dummy[i*8] = 'S';
                dummy[i*8+1] = 'o';
                dummy[i*8+2] = 'n';
                dummy[i*8+3] = 'g';
                dummy[i*8+4] = 'l';
                dummy[i*8+5] = 'i';
                dummy[i*8+6] = 'n';
                dummy[i*8+7] = ' ';
        }

        dummy[240] = 'e';
        dummy[241] = 'n';
        dummy[242] = 'd';
        dummy[243] = ' ';
*/
//	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
/*		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		if (bt_nus_send(NULL, buf->data, buf->len)) {
			LOG_WRN("Failed to send data over BLE connection");
		}
                
		k_free(buf);
*/                /* get cycle stamp */
/*                stamp = k_uptime_get_32();

                for(j=0; j<1000; j++){
                bt_nus_send(current_conn, dummy, 244);
                //bt_nus_send(NULL, dummy, 244);
                }

                delta = k_uptime_delta(&stamp);
                printk("[local] sent %u bytes (%u KB) in %lld ms at %llu bps\n",
                data, data / 1024, delta, ((uint64_t)data * 8000 / delta));
*/
//	}
//}

//K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
	//	NULL, PRIORITY, 0, 0);
