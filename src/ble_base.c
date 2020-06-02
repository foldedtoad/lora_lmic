/* 
 *  ble_base.c 
 */
#include <zephyr.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>

#include "ble_policy.h"
#include "ble_base.h"

#define LOG_LEVEL 3 //CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(ble_base);

static bool connect_state = false;

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
struct bt_conn * default_conn;

static const struct bt_data advert[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, PASTE_UUID_SERVICE, PASTE_UUID_BASE)
};

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
static void connected_cb(struct bt_conn * conn, u8_t err)
{
    if (err) {
        LOG_INF("Connection failed: err %u", err);
    }
    else {
        default_conn = bt_conn_ref(conn);

        printk("Connected\n");

        connect_state = true;
    }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
static void disconnected_cb(struct bt_conn * conn, u8_t reason)
{
    LOG_INF("Disconnected: reason %u", reason);

    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
    connect_state = false;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/

static struct bt_conn_cb conn_callbacks = {
    .connected    = connected_cb,
    .disconnected = disconnected_cb,
};

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
bool ble_is_connected(void)
{
    return connect_state;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
int ble_disconnect(void)
{
    LOG_INF("%s", __func__);

    if (default_conn) {
        struct bt_conn * conn = default_conn;

        bt_conn_unref(default_conn);
        
        default_conn = NULL;
        
        k_sleep(50);  // wait for notifications to complete
        
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
    return 0;
}

/*---------------------------------------------------------------------------*/
/*  Callback                                                                 */
/*---------------------------------------------------------------------------*/
static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("Bluetooth initialization failed: %d", err);
        return;
    }

    LOG_INF("Bluetooth initialized OK");
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
int ble_start_advertising(void)
{
    int err;

    static struct bt_data scan[] = {0};

    scan->type     = BT_DATA_NAME_COMPLETE;
    scan->data_len = DeviceIdLen;
    scan->data     = DeviceId;

    err = bt_le_adv_start(BT_LE_ADV_CONN,
                          advert, ARRAY_SIZE(advert),
                          scan,   ARRAY_SIZE(scan));
    if (err) {
        LOG_ERR("Start advertising failed: %d", err);
        return err;
    }

    LOG_INF("Start advertising OK");
    return 0;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
int ble_stop_advertising(void)
{
    int err;

    err = bt_le_adv_stop();
    if (err) {
        LOG_ERR("Stop advertising failed: %d", err);
        return err;
    }

    LOG_INF("Stop advertising OK");
    return 0;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing cancelled: %s", addr);
}

static struct bt_conn_auth_cb  auth_cb_display = {
    .cancel = auth_cancel,
};

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void bas_notify(void)
{
    u8_t battery_level = bt_gatt_bas_get_battery_level();

    battery_level--;

    if (!battery_level) {
        battery_level = 100U;
    }

    bt_gatt_bas_set_battery_level(battery_level);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
int ble_base_init(void)
{
    int err;

    err = bt_enable(bt_ready);
    if (err) {
        LOG_INF("Bluetooth initialization failed: %d", err);
        return err;
    }

    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_cb_register(&auth_cb_display);

    return 0;
}
