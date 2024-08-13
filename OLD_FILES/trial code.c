#define MAX_CONNECTIONS 2

static struct bt_conn *connections[MAX_CONNECTIONS] = {NULL, NULL};
static struct bt_nus_client nus_clients[MAX_CONNECTIONS];

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    int err, i;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err) {
        LOG_INF("Failed to connect to %s (%d)", addr, conn_err);
        return;
    }

    LOG_INF("Connected: %s", addr);

    // Find an available slot for the new connection
    for (i = 0; i < MAX_CONNECTIONS; i++) {
        if (connections[i] == NULL) {
            connections[i] = bt_conn_ref(conn);

            static struct bt_gatt_exchange_params exchange_params;
            exchange_params.func = exchange_func;
            err = bt_gatt_exchange_mtu(conn, &exchange_params);
            if (err) {
                LOG_WRN("MTU exchange failed (err %d)", err);
            }

            err = bt_conn_set_security(conn, BT_SECURITY_L2);
            if (err) {
                LOG_WRN("Failed to set security: %d", err);
                // Optionally, handle security error
            }

            // Start GATT discovery
            gatt_discover(conn);
            break;
        }
    }

    if (i == MAX_CONNECTIONS) {
        LOG_ERR("No available slots for new connection");
        bt_conn_disconnect(conn, BT_HCI_ERR_CONN_LIMIT_EXCEEDED);
    }

    // Stop scanning if the maximum number of connections is reached
    bool scanning = false;
    for (i = 0; i < MAX_CONNECTIONS; i++) {
        if (connections[i] == NULL) {
            scanning = true;
            break;
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
    int i;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s (reason %u)", addr, reason);

    // Find and remove the disconnected connection from the array
    for (i = 0; i < MAX_CONNECTIONS; i++) {
        if (connections[i] == conn) {
            bt_conn_unref(connections[i]);
            connections[i] = NULL;
            break;
        }
    }

    // Restart scanning if there are fewer than MAX_CONNECTIONS
    bool scanning = false;
    for (i = 0; i < MAX_CONNECTIONS; i++) {
        if (connections[i] == NULL) {
            scanning = true;
            break;
        }
    }

    if (scanning) {
        int err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
        if (err) {
            LOG_ERR("Scanning failed to start (err %d)", err);
        }
    }
}


static void gatt_discover(struct bt_conn *conn)
{
    int i;

    for (i = 0; i < MAX_CONNECTIONS; i++) {
        if (connections[i] == conn) {
            int err = bt_gatt_dm_start(conn, BT_UUID_NUS_SERVICE, &discovery_cb, &nus_clients[i]);
            if (err) {
                LOG_ERR("could not start the discovery procedure, error code: %d", err);
            }
            break;
        }
    }
}

static void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
    if (!err) {
        LOG_INF("MTU exchange done for connection %p", (void *)conn);
    } else {
        LOG_WRN("MTU exchange failed for connection %p (err %" PRIu8 ")", (void *)conn, err);
    }
}

static void discovery_complete(struct bt_gatt_dm *dm, void *context)
{
    struct bt_nus_client *nus = context;
    LOG_INF("Service discovery completed");

    bt_gatt_dm_data_print(dm);

    bt_nus_handles_assign(dm, nus);
    bt_nus_subscribe_receive(nus);

    bt_gatt_dm_data_release(dm);
}


static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        LOG_INF("Security changed: %s level %u", addr, level);
    } else {
        LOG_WRN("Security failed: %s level %u err %d", addr, level, err);
    }

    gatt_discover(conn);  // Trigger GATT discovery for the connection
}

static void scan_filter_match(struct bt_scan_device_info *device_info, struct bt_scan_filter_match *filter_match, bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    LOG_INF("Filters matched. Address: %s, Connectable: %d", addr, connectable);
}


static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
    LOG_WRN("Connecting failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn)
{
    // Handle up to 2 connections
    int i;
    for (i = 0; i < MAX_CONNECTIONS; i++) {
        if (connections[i] == NULL) {
            connections[i] = bt_conn_ref(conn);
            break;
        }
    }

    if (i == MAX_CONNECTIONS) {
        LOG_WRN("Connection limit reached, cannot connect to more devices");
        bt_conn_disconnect(conn, BT_HCI_ERR_CONN_LIMIT_EXCEEDED);
    }
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
