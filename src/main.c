/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include "uart_async_adapter.h"

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>
#include <zephyr/pm/pm.h>

#include <stdio.h>

#include <zephyr/logging/log.h>

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
#define SPI_READ_REQ DK_BTN3_MSK
#define IRQ_LINE_REQ DK_BTN4_MSK
#define DBG_REQ DK_BTN2_MSK
#define CENT_REQ DK_BTN1_MSK


#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 1);


static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

//static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
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



// it is not working like this
//static struct gpio_callback button_cb_data;
//static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(DT_ALIAS(swn), gpios,
//							      {0});

#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)
struct spi_cs_control spim_cs = {
	.gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master)),
	.delay = 0,
};

static const struct device * spi_hci_dev; 

int stop_command_from_central = 0;
int get_command_from_central = 0;

/*
 * This is the SPI bus controller device used to exchange data with
 * the SPI-based BT controller.
 */
/* Needs to be aligned with the SPI master buffer size */
#define SPI_MAX_MSG_LEN        256
#define SPI_DATA_TO_GET			21
#define SPI_WAV_DATA        	200

#define DEVICE_ID (0x00)
char gap_device_id = 99;

volatile static uint8_t rxmsg[SPI_MAX_MSG_LEN];
static struct spi_buf rx;
const static struct spi_buf_set rx_bufs = {
	.buffers = &rx,
	.count = 1,
};

volatile static uint8_t txmsg[SPI_MAX_MSG_LEN];
static struct spi_buf tx;
const static struct spi_buf_set tx_bufs = {
	.buffers = &tx,
	.count = 1,
};





static struct spi_config spi_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER,
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
		LOG_INF("00 k_free uart_data_t");

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

//		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
//			LOG_WRN("Failed to send data over UART");
//		}

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
//			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		LOG_INF("00 Malloc uart_data_t");

		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

//		uart_rx_enable(uart, buf->data, sizeof(buf->data),
//			       UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		LOG_INF("01 Malloc uart_data_t");

		if (buf) {
			buf->len = 0;
//			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
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
			LOG_INF("01 k_free uart_data_t");

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

//		uart_tx(uart, &buf->data[aborted_len],
//			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	LOG_INF("03 Malloc uart_data_t");

	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

//	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
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
	int err = 0;
	char addr[BT_ADDR_LE_STR_LEN] = {0};
	struct uart_data_t *buf;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s, len: %d", addr, len);
	for (int i =0;i<len;i++) LOG_INF("%d, ", data[i]);

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));
		LOG_INF("04 Malloc uart_data_t");

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

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
//		if ((pos == len) && (data[len - 1] == '\r')) {
//			tx->data[tx->len] = '\n';
//			tx->len++;
//		}

// do not send anything!!!! uart not here
//		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
//		if (err) {
//			k_fifo_put(&fifo_uart_tx_data, tx);
//		}

		// check the command
		if (tx->len == 4){
			if((tx->data[0] == 51) && (tx->data[1] == 51) && (tx->data[2] == 51)){
				LOG_INF("Get data");	// command 333
				get_command_from_central = 1;
			} else
			if((tx->data[0] == 54) && (tx->data[1] == 54) && (tx->data[2] == 54)){
				LOG_INF("Stop command");
				stop_command_from_central = 1;
			} else
			if((tx->data[0] == 57) && (tx->data[1] == 57) && (tx->data[2] == 57)){
				LOG_INF("ID COMMAND");	// command 999
				buf = k_malloc(sizeof(*buf));
				LOG_INF("05 Malloc uart_data_t");

				buf->len = 4;
				buf->data[0] = DEVICE_ID;
				buf->data[1] = gap_device_id;
				buf->data[2] = 3;
				buf->data[3] = 4;

				k_fifo_put(&fifo_uart_rx_data, buf);

			} else { 
				// do something 
			}
		}

//		// send back the command
//		if (bt_nus_send(NULL, tx->data, tx->len)) {
//			LOG_WRN("Failed to send data over BLE connection");
//		}

		k_free(tx);
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

#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */



void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */
	if(buttons & SPI_READ_REQ){
		LOG_INF("Button pressed: %d", buttons);
		send_data_to_central();
	}
	if(buttons & IRQ_LINE_REQ){
		LOG_INF("IRQ line from GAP!");
		get_data_gap();
	}
	if(buttons & DBG_REQ){
		LOG_INF("Stop Command!");
		stop_command_from_central = 1;

	}
	if(buttons & CENT_REQ){
		LOG_INF("Request Data!");
		get_command_from_central = 1;
	}
	
	LOG_INF("button: %d", buttons);

}




void get_data_gap(){

	int ack_from_gap = 0;
	
	for (int i =0; i<SPI_DATA_TO_GET;i++){
		txmsg[i] = (unsigned char) i;
		rxmsg[i] = 0;
	}

	if (stop_command_from_central){
		txmsg[1] = 0x6;
		txmsg[2] = 0x6;
		txmsg[3] = 0x6;
	} else if (get_command_from_central){
		txmsg[1] = 0x3;
		txmsg[2] = 0x3;
		txmsg[3] = 0x3;
	}


	tx.buf = txmsg;
	tx.len = SPI_DATA_TO_GET;
	rx.buf = rxmsg;
	rx.len = SPI_DATA_TO_GET;
	int ret = spi_transceive(spi_hci_dev, &spi_cfg, &tx_bufs, &rx_bufs);
	if (ret < 0) {
		LOG_ERR("SPI transceive error: %d", ret);
	}


	LOG_INF("Received data: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", 
			rxmsg[0], rxmsg[1], rxmsg[2], rxmsg[3], rxmsg[4], 
			rxmsg[5], rxmsg[6], rxmsg[7], rxmsg[8], rxmsg[9], 
			rxmsg[10], rxmsg[11], rxmsg[12], rxmsg[13], rxmsg[14], 
			rxmsg[15], rxmsg[16], rxmsg[17], rxmsg[18], rxmsg[19], rxmsg[20]);

	if ((rxmsg[1] == 0x05) && (rxmsg[2] == 0x06) && (rxmsg[3] == 0x07) && (rxmsg[4] == 0x08) && 
		(rxmsg[5] == 0x05) && (rxmsg[6] == 0x06) && (rxmsg[7] == 0x07)){
			gap_device_id = rxmsg[8];
			ack_from_gap = 1;
	}


	if (ack_from_gap == 0){
		// clear only if not an ack exchange
		if (stop_command_from_central){
			stop_command_from_central = 0;
		} else if (get_command_from_central){
			get_command_from_central = 0;
		}
	}


}

void send_data_to_central(){

	for (int i =0; i<SPI_DATA_TO_GET;i++){
			txmsg[i] = (unsigned char) i;
		}
		tx.buf = txmsg;
		tx.len = SPI_DATA_TO_GET;
		rx.buf = rxmsg;
		rx.len = SPI_DATA_TO_GET;
		int ret = spi_transceive(spi_hci_dev, &spi_cfg, &tx_bufs, &rx_bufs);
		if (ret < 0) {
			LOG_ERR("SPI transceive error: %d", ret);
		} else if (bt_nus_send(NULL, rx.buf, rx.len)) 
			LOG_WRN("Failed to send data over BLE connection");
}







void main(void)
{
	int blink_status = 0;
	int err = 0;
	uint32_t button_state = 0;
	uint32_t has_changed = 0;

	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}

	spi_hci_dev = DEVICE_DT_GET(MY_SPI_MASTER);
	if (!device_is_ready(spi_hci_dev)) {
		LOG_ERR("SPI bus %s is not ready", spi_hci_dev->name);
		return -EINVAL;
	}
	// SPI config settings
	spi_cfg.frequency = 10000000;
	spi_cfg.slave = 0;
	spi_cfg.cs = &spim_cs;


	/*
// I wasn't able to make it work like this -> I used the IRQ of buttom4
// FW reset inside the IRQ when SPI_send 					      
	gpio_pin_configure_dt(&button, GPIO_INPUT );

	gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_RISING);
							
	gpio_init_callback(&button_cb_data, BUTTON_PinInHandler, BIT(button.pin));

	gpio_add_callback(button.port, &button_cb_data);

	gpio_pin_get_dt(&button);
	
*/

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

	/* Prevent deep sleep (system off) from being entered */
//    pm_constraint_set(PM_STATE_SOFT_OFF);

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
//	rc=pm_device_state_set(cons, PM_DEVICE_STATE_LOW_POWER,NULL,NULL); 

//	rc=pm_state_set(cons, PM_DEVICE_STATE_LOW_POWER,NULL,NULL); 
//	k_sleep(K_MSEC(WAITTIME));
//	k_sleep(K_MSEC(WAITTIME));  //Added 2 waititmes because advertising is set to 2 sec and want to capture it advertising in PPK-2
	

//	for (;;) {
//		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
//		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
//	}
	dk_read_buttons(&button_state, &has_changed);
	button_changed(button_state, has_changed);

//	pm_state_force(0, &((struct pm_state_info){PM_STATE_STANDBY, 0, 0, 0}));



}




void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);

	for (;;) {
		// Wait indefinitely for data to be sent over bluetooth 
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		if (bt_nus_send(NULL, buf->data, buf->len)) {
			LOG_WRN("Failed to send data over BLE connection");
		}

		k_free(buf);
		LOG_INF("02 k_free uart_data_t");

	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);
