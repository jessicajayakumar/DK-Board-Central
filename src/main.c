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

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK


K_SEM_DEFINE(nus_write_sem, 0, 1);

#define MAX_CONNECTIONS 2

static struct bt_conn *default_conn[MAX_CONNECTIONS]= {NULL, NULL};
static struct bt_nus_client nus_client[MAX_CONNECTIONS];

//FreeBot Identity 23: E7:B0:4C:D9:30:3C
//Freebot Identity 21: D7:5A:2C:13:E3:7C


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
	int i;

	for (i = 0; i < MAX_CONNECTIONS; i++) {
		if (default_conn[i] == conn) {
			err = bt_gatt_dm_start(conn,
					       BT_UUID_NUS_SERVICE,
					       &discovery_cb,
					       &nus_client[i]);
			if (err) {
				LOG_ERR("could not start the discovery procedure, error "
					"code: %d", err);
			}
			break;
		}
	}

	if (i == MAX_CONNECTIONS) {
		LOG_WRN("No available slot for connection");
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
	int err, i;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		LOG_INF("Failed to connect to %s (%d)", addr, conn_err);

		for (i=0; i<MAX_CONNECTIONS;i++) {
			if (default_conn[i] == conn) {
				bt_conn_unref(default_conn[i]);
				default_conn[i] = NULL;

				err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
				if (err) {
					LOG_ERR("Scanning failed to start (err %d)",
						err);
				}
			}

			return;
		}
	}

	LOG_INF("Connected: %s", addr);

	for (i = 0; i < MAX_CONNECTIONS; i++) {
		if (default_conn[i] == NULL) {
			default_conn[i] = bt_conn_ref(conn);
				
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
			break;
		}
	}

	if (i == MAX_CONNECTIONS) {
		LOG_ERR("No available slots for new connection");
		bt_conn_disconnect(conn, BT_HCI_ERR_CONN_LIMIT_EXCEEDED);
	}

	bool scanning = false;
	for (i = 0; i < MAX_CONNECTIONS; i++) {
		if (default_conn[i] == NULL) {
			scanning = true;
			break;
		}
	}

	if (scanning){
		err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
				if (err) {
					LOG_ERR("Scanning failed to start (err %d)",
						err);
				}
	}

	if (!scanning) {
		err = bt_scan_stop();
		if ((!err) && (err != -EALREADY)) {
			LOG_ERR("Stop LE scan failed (err %d)", err);
		}
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err, i;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	for (i = 0; i < MAX_CONNECTIONS; i++) {
		if (default_conn[i] != conn) {
			return;
		}
	}

	for (i = 0; i < MAX_CONNECTIONS; i++) {
		if (default_conn[i] == conn) {
			bt_conn_unref(default_conn[i]);
			default_conn[i] = NULL;
			break;
		}
	}
		
	bool scanning = false;
	for (i = 0; i < MAX_CONNECTIONS; i++) {
		if (default_conn[i] == NULL) {
			scanning = true;
			break;
		}
	}

	if (scanning) {
		err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
		if (err) {
			LOG_ERR("Scanning failed to start (err %d)",
				err);
		}
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
	int i;

	for (i=0;i<MAX_CONNECTIONS;i++) {
		if (default_conn[i] == NULL) {
			default_conn[i] = bt_conn_ref(conn);
			break;
		}
	}

	if (i == MAX_CONNECTIONS) {
        LOG_WRN("Connection limit reached, cannot connect to more devices");
        bt_conn_disconnect(conn, BT_HCI_ERR_CONN_LIMIT_EXCEEDED);
    }
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

	err = scan_init();
	if (err != 0) {
		LOG_ERR("scan_init failed (err %d)", err);
		return 0;
	}


	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return 0;
	}

	LOG_INF("Scanning successfully started");

	while (1) {
		k_sleep(K_MSEC(1000));
	}
}
