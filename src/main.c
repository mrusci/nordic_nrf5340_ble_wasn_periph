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

#define N_READ_TASK 21
static K_SEM_DEFINE(ble_send_loop, 0, N_READ_TASK);

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


/*
 * This is the SPI bus controller device used to exchange data with
 * the SPI-based BT controller.
 */
/* Needs to be aligned with the SPI master buffer size */
#define SPI_MAX_MSG_LEN        256
#define SPI_DATA_TO_GET			21
#define SPI_WAV_DATA        	200

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
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

//	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

/*
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
		// Implement API adapter 
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
			// Give CPU resources to low priority threads.
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
		// Free the rx buffer only because the tx buffer will be handled in the callback
		k_free(rx);
	}

	return err;
}
*/



static K_SEM_DEFINE(througput_test_ready, 0, 3);
static uint8_t payload_length=24;
#define INTERVAL_MIN   10     
#define INTERVAL_MAX  10   
#define TOTAL_PACKETS 300

static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}


static struct bt_le_conn_param *conn_param = BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, 0, 400);
static int update_connection_parameters(void)
{	
	int err;
	err = bt_conn_le_param_update(current_conn, conn_param);
		if (err) {
			LOG_ERR("Cannot update conneciton parameter (err: %d)", err);
			return err;
		}
	LOG_INF("Connection parameters update requested");
	return 0;
}
static void conn_params_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	uint32_t interval_int= interval*1.25;
	LOG_INF("Conn params updated: interval %d ms, latency %d, timeout: %d0 ms",interval_int, latency, timeout);

	if (interval== INTERVAL_MIN) 
	{
		k_sem_give(&througput_test_ready);
	}
}

static void le_data_length_updated(struct bt_conn *conn,
				   struct bt_conn_le_data_len_info *info)
{
	LOG_INF("LE data len updated: TX (len: %d time: %d)"
	       " RX (len: %d time: %d)\n", info->tx_max_len,
	       info->tx_max_time, info->rx_max_len, info->rx_max_time);
	

}


static void MTU_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (!err) {
		LOG_INF("MTU exchange done. "); 
		payload_length=bt_gatt_get_mtu(current_conn)-3; //3 bytes ATT header
		LOG_INF("payload_length = %d ", payload_length); 

	} else {
		LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
	}
}
static void request_mtu_exchange(void)
{	int err;
	static struct bt_gatt_exchange_params exchange_params;
	exchange_params.func = MTU_exchange_cb;

	err = bt_gatt_exchange_mtu(current_conn, &exchange_params);
	if (err) {
		LOG_WRN("MTU exchange failed (err %d)", err);
	} else {
		LOG_INF("MTU exchange pending");
	}
	

}


static void request_data_len_update(void)
{
	int err;
	err = bt_conn_le_data_len_update(current_conn, BT_LE_DATA_LEN_PARAM_MAX);
		if (err) {
			LOG_ERR("LE data length update request failed: %d",  err);
		}
}
static void request_phy_update(void)
{
	int err;

	err = bt_conn_le_phy_update(current_conn, BT_CONN_LE_PHY_PARAM_2M);
		if (err) {
			LOG_ERR("Phy update request failed: %d",  err);
		}
}
static void le_phy_updated(struct bt_conn *conn,
			   struct bt_conn_le_phy_info *param)
{
	LOG_INF("LE PHY updated: TX PHY %s, RX PHY %s\n",
	       phy2str(param->tx_phy), phy2str(param->rx_phy));
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

	//Delays added to avoid collision (in case the central also send request), should be better with a state machine. 
	update_connection_parameters();
	k_sleep(K_MSEC(500));
	request_mtu_exchange();
	k_sleep(K_MSEC(500));
	request_data_len_update();
	k_sleep(K_MSEC(500));
	request_phy_update();
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
	.le_param_updated= conn_params_updated,
	.le_phy_updated = le_phy_updated,
	.le_data_len_updated = le_data_length_updated,
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

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

//		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}
}

volatile int cnt_sent = 0;
static void bt_sent_cb(struct bt_conn *conn)
{	
	LOG_INF("Sent Data: %d", cnt_sent);
	if (cnt_sent>0){
		cnt_sent--;
		k_sem_give(&ble_send_loop);
	}
}


static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
	.sent = bt_sent_cb,
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

int stop_command_from_central = 0;
int get_command_from_central = 0;

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
		LOG_INF("Get WAV and send via BLE!");
		send_wav();
	}
	if(buttons & CENT_REQ){
		LOG_INF("Central ask data!");
		get_command_from_central = 1;
	}
	
	LOG_INF("button: %d", buttons);

}



void send_wav(){
	cnt_sent = 2*(4*16000)/200;  // 100;
	for(int j=0;j<N_READ_TASK;j++)
		k_sem_give(&ble_send_loop);
}

void get_data_gap(){

	for (int i =0; i<SPI_DATA_TO_GET;i++){
		txmsg[i] = (unsigned char) i;
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

	if (stop_command_from_central){
		stop_command_from_central = 0;
	} else if (get_command_from_central){
		send_wav();
		get_command_from_central = 0;
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




void wav_write_thread(void)
{
	while(1){
		/* Don't go any further until BLE is initialized */
		k_sem_take(&ble_send_loop, K_FOREVER);
		LOG_INF("Sent package: %d", cnt_sent);

		for (int i =0; i<SPI_WAV_DATA;i++){
				txmsg[i] = (unsigned char) i;
		}
		tx.buf = txmsg;
		tx.len = SPI_WAV_DATA + 1;
		rx.buf = rxmsg;
		rx.len = SPI_WAV_DATA + 1 ;
		int ret = spi_transceive(spi_hci_dev, &spi_cfg, &tx_bufs, &rx_bufs);
		//LOG_INF("First four bytes %d %d %d %d", rxmsg[0], rxmsg[1],rxmsg[2],rxmsg[3]);
		//LOG_INF("Going to send via BLE: %d", rx.len);
		if (ret < 0) {
			LOG_ERR("SPI transceive error: %d", ret );
		} else{
			ret = bt_nus_send(NULL, rx.buf, rx.len);
			if (ret) 
				LOG_WRN("Failed to send data over BLE connection: %d", ret);
		} 

	}

}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, wav_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);




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

//	k_sem_give(&ble_init_ok);

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
	}
}

//K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
//		NULL, PRIORITY, 0, 0);
