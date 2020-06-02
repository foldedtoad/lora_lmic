/* 
 *  ble_policy.c
 */
#include <zephyr/types.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <zephyr.h>

#include "ble_policy.h"
#include "ble_base.h"
#include "event.h"
#include "buttons_id.h"

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(ble_policy);

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
typedef struct ble_msg {
    ble_event_t event;
    u32_t       data;
} ble_msg_t;

#define QUEUE_ELEMENTS       8
#define ALIGNMENT            4  // 32-bit alignment

K_MSGQ_DEFINE(ble_queue, sizeof(ble_msg_t), QUEUE_ELEMENTS, ALIGNMENT);

int  DeviceIdLen = 0;
char DeviceId [MAX_DEVICEID_STRING_LEN];

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
#define STACKSIZE 1024

struct k_thread ble_queue_thread;
K_THREAD_STACK_DEFINE(ble_queue_stack, STACKSIZE);

static k_tid_t tBleQ;

static bool active = false;

static struct k_work disconnect_work;

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
int ble_command(u32_t data)
{
    int status = 0;
    u8_t bytes[sizeof(u32_t)];
    
    memcpy(bytes, &data, sizeof(bytes));

    u32_t command = bytes[3];
    int params  = (((bytes[0] << 16) + (bytes[1] << 8) + bytes[2]) << 8) >> 8;

    switch (command) {

        case BLE_CMD__STOP:
            LOG_INF("%s: STOP", __func__);
            status = 0;
            break;

        case BLE_CMD__UNLOAD:
            LOG_INF("%s: UNLOAD", __func__);
            status = 0;
            break;

        case BLE_CMD__LOAD:
            LOG_INF("%s: LOAD", __func__);
            status = 0;
            break;

        case BLE_CMD__LEFT:
            LOG_INF("%s: LEFT %d", __func__, params);
            status = 0;
            break;

        case BLE_CMD__RIGHT:
            LOG_INF("%s: RIGHT %d", __func__, params);
            // TBD
            status = 0;
            break;

        case BLE_CMD__UP:
            LOG_INF("%s: UP %d", __func__, params);
            status = 0;
            break;

        case BLE_CMD__DOWN:
            LOG_INF("%s: DOWN %d", __func__, params);
            status = 0;
            break;

        case BLE_CMD__DISPENSE:
            LOG_INF("%s: DISPENSE", __func__);
            status = 0;
            break;

        default:
            LOG_INF("%s: <unknown> 0x%08x : command(%08X), params(%08X)", 
                    __func__, data, command, params);
            status = 0;
            break;
    }

    return status;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
static void disconnect_work_cb(struct k_work * work)
{
    ble_disconnect();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void ble_policy_tasks(void)
{
    while (active) {

        k_sleep(MSEC_PER_SEC);

        if (ble_is_connected()) {
            /* Battery level (simulation) */
            bas_notify();
        }
    }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
int ble_enqueue_msg(ble_event_t event, u32_t data)
{
    ble_msg_t msg;

    msg.event = event;
    msg.data  = data;

    while (k_msgq_put(&ble_queue, &msg, K_NO_WAIT) != 0) {
        LOG_ERR("%s: k_msgq_put error: purging queue", __func__);
        k_msgq_purge(&ble_queue);
        return -EIO;
    }
    return 0;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
static void ble_queue_service(void)
{
    int status;
    ble_msg_t msg;
    
    LOG_INF("%s: started", __func__);

    while (1) {

        k_msgq_get(&ble_queue, &msg, K_FOREVER);

        switch (msg.event) {

            case BLE_EVENT__VOICE:
                status = ble_command(msg.data);
                ble_operation_complete(msg.event, status);
                break;

            default:
                ble_operation_complete(msg.event, -EIO);
                break;
        }
    }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void ble_device_name(void)
{
    u32_t deviceid = NRF_FICR->DEVICEID[0];

    sprintf(DeviceId, "%s_%08x", CONFIG_BT_DEVICE_NAME, deviceid);

    DeviceIdLen = strlen(DeviceId);

    printk("DeviceId: %s\n", DeviceId);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
static int ble_queue_init(void)
{
    int status = 0;

    tBleQ = k_thread_create(&ble_queue_thread, 
                            ble_queue_stack, STACKSIZE,
                            (k_thread_entry_t)ble_queue_service, 
                            NULL, NULL, NULL, -1, K_USER, K_FOREVER);

    k_thread_start(&ble_queue_thread);

    return status;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void ble_button_event(buttons_id_t id)
{
    //LOG_INF("%s: button %d", __func__, id);

    switch (id) {

        case BTN1_ID:
            paste_notify(0x01, 0x02);
            break;

        default:
            LOG_INF("%s: unknown: %d", __func__, id);
            break;
    }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void ble_operation_complete(ble_event_t event, u32_t code)
{
    paste_notify(event, code);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
int ble_policy_init(void)
{
    int status;

    /*
     *  Create unique device identifier.
     */
    ble_device_name();

    /*
     *  Start base BLE 
     */
    status = ble_base_init();
    if (status < 0) {
        return status;
    }

    /*
     *  Start queue service on its own thread.
     */
    ble_queue_init();
    if (status < 0) {
        return status;
    }

    disconnect_work.handler = disconnect_work_cb;

    /*
     *  Indicate all is initialized.
     */
    active = true;

    /* 
     *  Start polling tasks: thread should never return. Keep last.
     */
    ble_policy_tasks();

    return -EINVAL;
}
