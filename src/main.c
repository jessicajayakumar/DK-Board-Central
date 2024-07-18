/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Service Client sample
 */

/**
 * This file includes various header files required for the DK-Board-Central project.
 * Each header file serves a specific purpose as described below:
 *
 * - errno.h: Defines macros for reporting and retrieving error conditions.
 * - zephyr/kernel.h: Provides kernel-related functions and definitions.
 * - zephyr/device.h: Defines device-related functions and structures.
 * - zephyr/devicetree.h: Provides access to the device tree.
 * - zephyr/sys/byteorder.h: Defines byte order conversion functions.
 * - zephyr/sys/printk.h: Provides printing functions for debugging.
 * - zephyr/bluetooth/bluetooth.h: Main Bluetooth stack API.
 * - zephyr/bluetooth/hci.h: HCI (Host Controller Interface) API.
 * - zephyr/bluetooth/conn.h: Connection management API.
 * - zephyr/bluetooth/uuid.h: UUID (Universally Unique Identifier) API.
 * - zephyr/bluetooth/gatt.h: GATT (Generic Attribute Profile) API.
 * - bluetooth/services/nus.h: Nordic UART Service (NUS) API.
 * - bluetooth/services/nus_client.h: NUS client API.
 * - bluetooth/gatt_dm.h: GATT Discovery Manager API.
 * - bluetooth/scan.h: Bluetooth scanning API.
 * - zephyr/settings/settings.h: Provides access to the settings subsystem.
 * - zephyr/drivers/uart.h: UART (Universal Asynchronous Receiver-Transmitter) API.
 * - zephyr/logging/log.h: Logging API.
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

#define LOG_MODULE_NAME DKB_central_uart // Define the name of the logging module
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG); // Register the logging module with the specified name and debug log level

/* UART payload buffer element size. */
#define UART_BUF_SIZE 20 // Define the size of the UART payload buffer

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK // Define the button mask for accepting a passkey
#define KEY_PASSKEY_REJECT DK_BTN2_MSK // Define the button mask for rejecting a passkey

#define NUS_WRITE_TIMEOUT K_MSEC(150) // Define the timeout for writing data to the NUS client
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50) // Define the delay for waiting for a UART buffer
#define UART_RX_TIMEOUT 50000 /* Wait for RX complete event time in microseconds. */ // Define the timeout for receiving data over UART

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart)); // Get the UART device from the device tree
static struct k_work_delayable uart_work; // Define a delayable work item for UART processing

K_SEM_DEFINE(nus_write_sem, 0, 1); // Define a semaphore for synchronizing NUS write operations

struct uart_data_t {
	void *fifo_reserved; // Reserved field for use with FIFOs
	uint8_t  data[UART_BUF_SIZE]; // Buffer to hold UART payload data
	uint16_t len; // Length of the UART payload data
};

static K_FIFO_DEFINE(fifo_uart_tx_data);  // Define a static First-In-First-Out (FIFO) buffer for transmitting UART data
static K_FIFO_DEFINE(fifo_uart_rx_data);  // Define a static FIFO buffer for receiving UART data

static struct bt_conn *default_conn;  // Declare a pointer to a Bluetooth connection structure
static struct bt_nus_client nus_client;  // Declare a Bluetooth NUS (Nordic UART Service) client structure


static struct bt_uuid_16 discover_uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;
static struct bt_gatt_write_params write_params_ctrl;

#define BT_UUID_TSS_VAL BT_UUID_128_ENCODE(0x00001560, 0x1212, 0xefde, 0x1560, 0x785feabcd123)
#define BT_UUID_TSS BT_UUID_DECLARE_128(BT_UUID_TSS_VAL)

#define BT_UUID_16_FBS_VAL 0x1560
#define BT_UUID_16_FBS_CTRL_VAL 0x1561
#define BT_UUID_16_FBS_AVGS_VAL 0x1562
#define BT_UUID_16_FBS BT_UUID_DECLARE_16(BT_UUID_16_FBS_VAL)
#define BT_UUID_16_FBS_CTRL BT_UUID_DECLARE_16(BT_UUID_16_FBS_CTRL_VAL)
#define BT_UUID_16_FBS_AVGS BT_UUID_DECLARE_16(BT_UUID_16_FBS_AVGS_VAL)




static void write_func_ctrl(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params){
	if (err) {
		printk("GATT write to peripheral failed (err %d)\n", err);
	} else {
		printk("Successful GATT write to peripheral\n\n");
	}
}


static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{

	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	const char *msg_string = (const char *) data;

	// printk("[NOTIFICATION] data %p length %u\n", data, length);
	LOG_INF("[NOTIFICATION - AVGS] message: %s", msg_string);

	// not needed to dissect string message at gateway, this can be done on Rasberry Pi with "sscanf" function
	strcpy((char *) tx_buf, msg_string);
	int err = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_US);
	if (err) {
		printk("Failed to transmit over UART (err %d)", err);
	}

	return BT_GATT_ITER_CONTINUE;
}


static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_HRS)) {
		memcpy(&discover_uuid, BT_UUID_HRS_MEASUREMENT, sizeof(discover_uuid));
		discover_params.uuid = &discover_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
				BT_UUID_HRS_MEASUREMENT)) {
		memcpy(&discover_uuid, BT_UUID_GATT_CCC, sizeof(discover_uuid));
		discover_params.uuid = &discover_uuid.uuid;
		discover_params.start_handle = attr->handle + 2;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else {
		subscribe_params.notify = notify_func;
		subscribe_params.value = BT_GATT_CCC_NOTIFY;
		subscribe_params.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params);
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("[SUBSCRIBED]\n");
		}

		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_STOP;
}

static void ble_data_sent(struct bt_nus_client *nus, uint8_t err,
                    const uint8_t *const data, uint16_t len)
{
    ARG_UNUSED(nus);  // Mark the 'nus' parameter as unused to avoid compiler warnings
    ARG_UNUSED(data);  // Mark the 'data' parameter as unused to avoid compiler warnings
    ARG_UNUSED(len);  // Mark the 'len' parameter as unused to avoid compiler warnings

    k_sem_give(&nus_write_sem);  // Release the semaphore to indicate that the NUS data has been sent

    if (err) {
        LOG_WRN("ATT error code: 0x%02X", err);  // Log a warning message with the ATT (Attribute Protocol) error code
    }
}

// Function to handle received BLE data and transmit it over UART.
static uint8_t ble_data_received(struct bt_nus_client *nus,
                                 const uint8_t *data, uint16_t len)
{
	LOg_INF("Received data from BLE: %s", data);
    ARG_UNUSED(nus); // Macro to avoid unused parameter warning for 'nus'.

    int err;

    // Loop through the received data.
    for (uint16_t pos = 0; pos != len;) {
        // Allocate memory for UART data structure.
        struct uart_data_t *tx = k_malloc(sizeof(*tx)); 
		LOG_INF("Allocated memory for UART data structure");

        // Check if memory allocation failed.
        if (!tx) {
            LOG_WRN("Not able to allocate UART send data buffer");
            return BT_GATT_ITER_CONTINUE; // Continue BLE GATT iteration despite the error.
        }

        // Calculate the maximum data size for UART transmission, reserving 1 byte for LF if needed.
        size_t tx_data_size = sizeof(tx->data) - 1;

        // Determine the length of data to be copied to UART buffer.
        if ((len - pos) > tx_data_size) {
            tx->len = tx_data_size;
        } else {
            tx->len = (len - pos);
        }

        // Copy the data from BLE buffer to UART buffer.
        memcpy(tx->data, &data[pos], tx->len);
		LOG_INF("Copied data from BLE buffer to UART buffer");

        // Update the position in the BLE buffer.
        pos += tx->len;

        // If this is the last chunk and it ends with CR, append LF to UART data.
        if ((pos == len) && (data[len - 1] == '\r')) {
            tx->data[tx->len] = '\n'; // Append LF character.
            tx->len++; // Increment length to account for the added LF.
			LOG_INF("Appended LF character to UART data");
        }

        // Attempt to transmit the data over UART.
        err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		LOG_INF("Transmitted data over UART");
        if (err) {
            // If transmission fails, put the data in a FIFO queue for later transmission.
            k_fifo_put(&fifo_uart_tx_data, tx);
        }
    }
	LOG_INF("Finished processing BLE data");
    // Continue BLE GATT iteration.
    return BT_GATT_ITER_CONTINUE;
}

// UART event callback function.
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    // Macro to suppress unused parameter warnings for 'dev'.
    ARG_UNUSED(dev);

    // Static variables to hold the length of aborted transmission and pointer to the aborted buffer.
    static size_t aborted_len;
    struct uart_data_t *buf;
    static uint8_t *aborted_buf;
    static bool disable_req; // Static variable to track if UART RX should be disabled.

    // Switch statement to handle different types of UART events.
    switch (evt->type) {

    case UART_TX_DONE: // Case for UART transmission done event.
        LOG_DBG("UART_TX_DONE"); // Log debug message indicating transmission completion.

        // Check if the transmission length is 0 or the buffer is NULL, indicating nothing to process.
        if ((evt->data.tx.len == 0) ||
            (!evt->data.tx.buf)) {
			LOG_INF("Transmission length is 0 or buffer is NULL");
            return; // Exit the function early.
        }

        // Check if there was a previously aborted transmission.
        if (aborted_buf) {
            // Retrieve the uart_data_t structure from the aborted buffer.
			LOG_INF("Retrieving uart_data_t structure from the aborted buffer");
            buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data[0]);
            aborted_buf = NULL; // Clear the pointer to the aborted buffer.
            aborted_len = 0; // Reset the length of the aborted transmission.
        } else {
            // Retrieve the uart_data_t structure from the current transmission buffer.
			LOG_INF("Retrieving uart_data_t structure from the current transmission buffer");)
            buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t, data[0]);
        }

        // Free the memory allocated for the uart_data_t structure now that transmission is complete.
        k_free(buf);

        // Attempt to get the next buffer from the FIFO queue for UART transmission.
        buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		LOG_INF("Getting the next buffer from the FIFO queue for UART transmission");
        if (!buf) {
            return; // If no buffer is available, exit the function early.
        }

        // Attempt to transmit the next buffer over UART.
		LOG_INF("Transmitting the next buffer over UART");
        if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
            // Log a warning if the transmission fails.
            LOG_WRN("Failed to send data over UART");
        }

        break;

	case UART_RX_RDY: // Case for UART receive ready event.
    LOG_DBG("UART_RX_RDY"); // Log debug message indicating data is ready to be received.
    // Retrieve the uart_data_t structure from the received buffer.
	LOG_INF("Retrieving uart_data_t structure from the received buffer");
    buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
    // Update the length of data received.
    buf->len += evt->data.rx.len;

    // Check if there's a request to disable UART RX.
    if (disable_req) {
        return; // Exit the function early if disable request is set.
    }

    // Check if the last received character is a newline or carriage return.
    if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
        (evt->data.rx.buf[buf->len - 1] == '\r')) {
        disable_req = true; // Set request to disable UART RX.
        uart_rx_disable(uart); // Disable UART RX.
    }

    break;

	case UART_RX_DISABLED: // Case for UART receive disabled event.
		LOG_DBG("UART_RX_DISABLED"); // Log debug message indicating UART RX has been disabled.
		disable_req = false; // Reset the disable request flag.

		// Allocate memory for a new receive buffer.
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0; // Initialize the length of the buffer.
		} else {
			// Log warning if memory allocation fails.
			LOG_WRN("Not able to allocate UART receive buffer");
			// Reschedule a work item to retry buffer allocation after a delay.
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			LOG_INF("Rescheduled work item to retry buffer allocation after a delay");
			return;
		}

		// Enable UART RX with the new buffer.
		uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_RX_TIMEOUT);
		LONG_INF("Enabled UART RX with the new buffer");

		break;

	case UART_RX_BUF_REQUEST: // Case for UART receive buffer request event.
		LOG_DBG("UART_RX_BUF_REQUEST"); // Log debug message indicating a new buffer is requested for RX.
		// Allocate memory for the new receive buffer.
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0; // Initialize the length of the buffer.
			// Provide the new buffer to UART.
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
			LOG_INF("Provided the new buffer to UART");
		} else {
			// Log warning if memory allocation fails.
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED: // Case for UART receive buffer released event.
		LOG_DBG("UART_RX_BUF_RELEASED"); // Log debug message indicating a receive buffer has been released.
		// Retrieve the uart_data_t structure from the released buffer.
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, data[0]);
		LOG_INF("Retrieved uart_data_t structure from the released buffer");

		// Check if the buffer contains received data.
		if (buf->len > 0) {
			// Put the buffer with data into a FIFO queue for processing.
			k_fifo_put(&fifo_uart_rx_data, buf);
			LOG_INF("Put the buffer with data into a FIFO queue for processing");
		} else {
			// Free the buffer if it's empty.
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED: // Case for UART transmission aborted event.
		LOG_DBG("UART_TX_ABORTED"); // Log debug message indicating a transmission has been aborted.
		// Check if there's no previously aborted buffer.
		if (!aborted_buf) {
			// Save the pointer to the aborted buffer.
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}
		LOG_INF("Saved the pointer to the aborted buffer");

		// Update the length of the aborted transmission.
		aborted_len += evt->data.tx.len;
		// Retrieve the uart_data_t structure from the aborted buffer.
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data[0]);

		// Attempt to transmit the remaining data from the aborted buffer.
		uart_tx(uart, &buf->data[aborted_len], buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default: // Default case for unhandled UART events.
		break;
	}
}

// Work handler function for UART. It's scheduled to run in response to certain events.
static void uart_work_handler(struct k_work *item)
{
	LONG_INF("UART work handler called"); // Log an informational message indicating the UART work handler has been called.
    struct uart_data_t *buf; // Define a pointer to a UART data structure.

    // Allocate memory for the UART buffer.
    buf = k_malloc(sizeof(*buf));
    if (buf) {
        buf->len = 0; // Initialize the length of the buffer to 0.
    } else {
        // Log a warning if memory allocation fails.
        LOG_WRN("Not able to allocate UART receive buffer");
        // Reschedule the work item if the buffer allocation fails, with a delay defined by UART_WAIT_FOR_BUF_DELAY.
        k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
        return; // Exit the function early if memory allocation fails.
    }

    // Enable UART reception, specifying the buffer, its size, and a timeout.
    uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_RX_TIMEOUT);
	LOG_INF("Enabled UART reception, uart_work_handler function");
}

// Function to initialize UART.
static int uart_init(void)
{
	LONG_INEF("Initializing UART"); // Log an informational message indicating UART initialization.
    int err; // Variable to store error codes.
    struct uart_data_t *rx; // Define a pointer for the receive buffer.

    // Check if the UART device is ready.
    if (!device_is_ready(uart)) {
        LOG_ERR("UART device not ready"); // Log an error if the device is not ready.
        return -ENODEV; // Return an error code indicating the device is not found.
    }

    // Allocate memory for the receive buffer.
    rx = k_malloc(sizeof(*rx));
    if (rx) {
        rx->len = 0; // Initialize the length of the buffer to 0.
    } else {
        return -ENOMEM; // Return an error code indicating out of memory.
    }

    // Initialize a delayable work item with the uart_work_handler function.
    k_work_init_delayable(&uart_work, uart_work_handler);
	LONG_INF("Initialized delayable work item with the uart_work_handler function");

    // Set the UART callback function.
    err = uart_callback_set(uart, uart_cb, NULL);
    if (err) {
        return err; // Return the error code if callback setting fails.
    }

    // Enable UART reception, specifying the buffer, its size, and a timeout.
    return uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_RX_TIMEOUT);
	LOG_INF("Enabled UART reception, uart init function");
}

static void discovery_complete(struct bt_gatt_dm *dm,
			       void *context)
{
	struct bt_nus_client *nus = context;
	LOG_INF("Service discovery completed");

	bt_gatt_dm_data_print(dm);

	bt_nus_handles_assign(dm, nus);
	bt_nus_subscribe_receive(nus);

	bt_gatt_dm_data_release(dm);
}

static void discovery_service_not_found(struct bt_conn *conn,
					void *context)
{
	LOG_INF("Service not found");
}

static void discovery_error(struct bt_conn *conn,
			    int err,
			    void *context)
{
	LOG_WRN("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
	.completed         = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found       = discovery_error,
};

static void gatt_discover(struct bt_conn *conn)
{
	int err;

	if (conn != default_conn) {
		return;
	}

	err = bt_gatt_dm_start(conn,
			       BT_UUID_NUS_SERVICE,
			       &discovery_cb,
			       &nus_client);
	if (err) {
		LOG_ERR("could not start the discovery procedure, error "
			"code: %d", err);
	}
}

static void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (!err) {
		LOG_INF("MTU exchange done");
	} else {
		LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		LOG_INF("Failed to connect to %s (%d)", addr, conn_err);

		if (default_conn == conn) {
			bt_conn_unref(default_conn);
			default_conn = NULL;

			err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
			if (err) {
				LOG_ERR("Scanning failed to start (err %d)",
					err);
			}
		}

		return;
	}

	LOG_INF("Connected: %s", addr);

	static struct bt_gatt_exchange_params exchange_params;

	exchange_params.func = exchange_func;
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_WRN("MTU exchange failed (err %d)", err);
	}

	err = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (err) {
		LOG_WRN("Failed to set security: %d", err);

		gatt_discover(conn);
	}

	err = bt_scan_stop();
	if ((!err) && (err != -EALREADY)) {
		LOG_ERR("Stop LE scan failed (err %d)", err);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (default_conn != conn) {
		return;
	}

	bt_conn_unref(default_conn);
	default_conn = NULL;

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)",
			err);
	}
}

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

	gatt_discover(conn);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed
};

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	LOG_INF("Filters matched. Address: %s connectable: %d",
		addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_WRN("Connecting failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	default_conn = bt_conn_ref(conn);
}

static int nus_client_init(void)
{
	int err;
	struct bt_nus_client_init_param init = {
		.cb = {
			.received = ble_data_received,
			.sent = ble_data_sent,
		}
	};

	err = bt_nus_client_init(&nus_client, &init);
	if (err) {
		LOG_ERR("NUS Client initialization failed (err %d)", err);
		return err;
	}

	LOG_INF("NUS Client module initialized");
	return err;
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
		scan_connecting_error, scan_connecting);

static int scan_init(void)
{
	int err;
	struct bt_scan_init_param scan_init = {
		.connect_if_match = 1,
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_NUS_SERVICE);
	if (err) {
		LOG_ERR("Scanning filters cannot be set (err %d)", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		LOG_ERR("Filters cannot be turned on (err %d)", err);
		return err;
	}

	LOG_INF("Scan module initialized");
	return err;
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

	LOG_WRN("Pairing failed conn: %s, reason %d", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

int main(void)
{
	int err;

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		LOG_ERR("Failed to register authorization callbacks.");
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("Failed to register authorization info callbacks.\n");
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}
	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = uart_init();
	if (err != 0) {
		LOG_ERR("uart_init failed (err %d)", err);
		return 0;
	}

	err = scan_init();
	if (err != 0) {
		LOG_ERR("scan_init failed (err %d)", err);
		return 0;
	}

	err = nus_client_init();
	if (err != 0) {
		LOG_ERR("nus_client_init failed (err %d)", err);
		return 0;
	}

	printk("Starting Bluetooth Central UART example\n");

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return 0;
	}

	LOG_INF("Scanning successfully started");

	struct uart_data_t nus_data = {
		.len = 0,
	};

	for (;;) {
		/* Wait indefinitely for data to be sent over Bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		int plen = MIN(sizeof(nus_data.data) - nus_data.len, buf->len);
		int loc = 0;

		while (plen > 0) {
			memcpy(&nus_data.data[nus_data.len], &buf->data[loc], plen);
			nus_data.len += plen;
			loc += plen;
			if (nus_data.len >= sizeof(nus_data.data) ||
			   (nus_data.data[nus_data.len - 1] == '\n') ||
			   (nus_data.data[nus_data.len - 1] == '\r')) {
				err = bt_nus_client_send(&nus_client, nus_data.data, nus_data.len);
				if (err) {
					LOG_WRN("Failed to send data over BLE connection"
						"(err %d)", err);
				}

				err = k_sem_take(&nus_write_sem, NUS_WRITE_TIMEOUT);
				if (err) {
					LOG_WRN("NUS send timeout");
				}

				nus_data.len = 0;
			}

			plen = MIN(sizeof(nus_data.data), buf->len - loc);
		}

		k_free(buf);
	}
}
