/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Service Client sample
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <zephyr/settings/settings.h>

#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME central_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

/* UART payload buffer element size. */
#define UART_BUF_SIZE 20

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define NUS_WRITE_TIMEOUT K_MSEC(150)
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_RX_TIMEOUT 50000 /* Wait for RX complete event time in microseconds. */

#define MAX_CONN 3

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work[MAX_CONN];

struct uart_data_t {
	void *fifo_reserved;
	uint8_t  data[UART_BUF_SIZE];
	uint16_t len;
};

static struct k_fifo fifo_uart_rx_data[MAX_CONN];
static struct k_fifo fifo_uart_tx_data[MAX_CONN];
static struct k_sem nus_write_sem[MAX_CONN];

static struct bt_conn *default_conn[MAX_CONN];
static struct bt_nus_client nus_client[MAX_CONN];

void init_fifos_and_sems(void)
{
    for (int i = 0; i < MAX_CONN; i++) {
        k_fifo_init(&fifo_uart_rx_data[i]);
        k_fifo_init(&fifo_uart_tx_data[i]);
        k_sem_init(&nus_write_sem[i], 0, 1);
    }
}

static void ble_data_sent(struct bt_nus_client *nus, uint8_t err,
					const uint8_t *const data, uint16_t len)
{
	int conn_index = nus - nus_client;
	k_sem_give(&nus_write_sem[conn_index]);

	if (err) {
		LOG_WRN("ATT error code: 0x%02X", err);
	}
}

static uint8_t ble_data_received(struct bt_nus_client *nus,
						const uint8_t *data, uint16_t len)
{
	int conn_index = nus - nus_client;
	int err;

	for (uint16_t i = 0; i < len; i++) {
		LOG_INF("Byte %d: 0x%02X (decimal: %d)", i, data[i], data[i]);
	}

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return BT_GATT_ITER_CONTINUE;
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

		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		for (size_t i = 0; i < tx->len; i++) {
			LOG_INF("Received data in uart as raw integer: %d", tx->data[i]);	
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data[conn_index], tx);
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;
	int conn_index = *(int *)user_data;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data[0]);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf,
					   struct uart_data_t,
					   data[0]);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data[conn_index], K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
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
			k_work_reschedule(&uart_work[conn_index], UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_RX_TIMEOUT);

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
				   data[0]);

		
		for (size_t i = 0; i < buf->len; i++) {
			if ((buf->data[i] == '\n') || (buf->data[i] == '\r')) {
				buf->data[i] = '\0';
		
				break;
			}
		}

		LOG_INF("Data received: %s", buf->data);

		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data[conn_index], buf);
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
				   data[0]);

		uart_tx(uart, &buf->data[aborted_len],
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	int conn_index = item - uart_work;
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work[conn_index], UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_RX_TIMEOUT);
}

static int uart_init(void)
{
	int err;

	if (!device_is_ready(uart)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	for (int i = 0; i < MAX_CONN; i++) {
		k_work_init_delayable(&uart_work[i], uart_work_handler);
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	k_work_reschedule(&uart_work[0], K_NO_WAIT);

	return 0;
}

static void discovery_completed(struct bt_gatt_dm *dm,
					void *context)
{
	int err;
	int conn_index = *(int *)context;
	struct bt_nus_client *nus = &nus_client[conn_index];

	err = bt_nus_handles_assign(dm, nus);
	if (err) {
		LOG_ERR("Could not init NUS client object, error: %d",
			err);
	}

	bt_gatt_dm_data_print(dm);
	bt_gatt_dm_done(dm);

	printk("Service discovery completed\n");
}

static void discovery_service_not_found(struct bt_conn *conn,
					  void *context)
{
	printk("Service not found\n");
	bt_gatt_dm_data_release(context);
}

static void discovery_error(struct bt_conn *conn,
				int err,
				void *context)
{
	printk("The discovery procedure failed, err %d\n", err);
	bt_gatt_dm_data_release(context);
}

static void gatt_discover(struct bt_conn *conn)
{
	static const struct bt_gatt_dm_cb discover_cb = {
		.completed = discovery_completed,
		.service_not_found = discovery_service_not_found,
		.error_found = discovery_error,
	};

	int err;
	int conn_index = *(int *)bt_conn_get_user_data(conn);
	err = bt_gatt_dm_start(conn,
				BT_UUID_NUS_SERVICE,
				&discover_cb,
				&conn_index);
	if (err) {
		printk("Could not start the discovery procedure, error "
		       "code: %d\n", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;
	int conn_index = -1;

	for (int i = 0; i < MAX_CONN; i++) {
		if (default_conn[i] == NULL) {
			default_conn[i] = bt_conn_ref(conn);
			conn_index = i;
			break;
		}
	}

	if (conn_index == -1) {
		LOG_ERR("No free connection slots available");
		return;
	}

	if (conn_err) {
		printk("Failed to connect to peripheral: %u\n", conn_err);
		bt_conn_unref(default_conn[conn_index]);
		default_conn[conn_index] = NULL;
		return;
	}

	printk("Connected: %d\n", conn_index);

	gatt_discover(conn);
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;
	int conn_index = -1;

	// Find an available slot for the new connection
	for (int i = 0; i < MAX_CONN; i++) {
		if (default_conn[i] == NULL) {
			default_conn[i] = bt_conn_ref(conn);
			conn_index = i;
			break;
		}
	}

	if (conn_index == -1) {
		LOG_ERR("No free connection slots available");
		return;
	}

	if (conn_err) {
		printk("Failed to connect to peripheral: %u\n", conn_err);
		bt_conn_unref(default_conn[conn_index]);
		default_conn[conn_index] = NULL;
		return;
	}

	printk("Connected: %d\n", conn_index);

	// Define MTU exchange parameters
	static struct bt_gatt_exchange_params exchange_params;
	exchange_params.func = exchange_func;

	// Start MTU exchange
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_WRN("MTU exchange failed (err %d)", err);
	}

	// Start GATT service discovery after MTU exchange
	err = gatt_discover(conn);
	if (err) {
		LOG_WRN("GATT discovery failed (err %d)", err);
	}
}


static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int conn_index = -1;

	for (int i = 0; i < MAX_CONN; i++) {
		if (default_conn[i] == conn) {
			conn_index = i;
			break;
		}
	}

	if (conn_index == -1) {
		LOG_ERR("No matching connection found");
		return;
	}

	printk("Disconnected (reason: %u)\n", reason);

	bt_conn_unref(default_conn[conn_index]);
	default_conn[conn_index] = NULL;
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	printk("Filter matched. Address: %s connectable: %d\n",
	       bt_addr_le_str(&device_info->recv_info->addr), connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	printk("Connecting failed\n");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	printk("Connecting...\n");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, scan_connecting);

void main(void)
{
	int err;

	printk("Starting Bluetooth Central UART example\n");

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return;
	}

	err = uart_init();
	if (err) {
		LOG_ERR("UART initialization failed (err: %d)\n", err);
		return;
	}

	bt_conn_cb_register(&conn_callbacks);
	bt_nus_client_cb_register(&nus_client[0], ble_data_received, ble_data_sent);

	init_fifos_and_sems();

	bt_scan_init(NULL);
	bt_scan_cb_register(&scan_cb);

	struct bt_scan_init_param scan_init = {
		.connect_if_match = 1,
		.scan_param = NULL,
		.conn_param = NULL
	};

	bt_scan_init(&scan_init);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_NUS_SERVICE);
	if (err) {
		printk("Scanning filters cannot be set (err %d)\n", err);
		return;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		printk("Filters cannot be turned on (err %d)\n", err);
		return;
	}

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}
}
