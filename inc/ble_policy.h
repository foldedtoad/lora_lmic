/*
 *  ble_policy.h
 */
#ifndef __BLE_POLICY_H__
#define __BLE_POLICY_H__

#include "buttons.h"

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/

typedef enum {
    BLE_EVENT__INVALID = 0,
    BLE_EVENT__VOICE,
    BLE_EVENT__LAST
} ble_event_t;

typedef enum {
    BLE_CMD__BAD_CMD  = 0,
    BLE_CMD__BAD_ACT  = 1,
    BLE_CMD__STOP     = 0x10,
    BLE_CMD__UNLOAD   = 0x20,
    BLE_CMD__LOAD     = 0x30,
    BLE_CMD__LEFT     = 0x40,
    BLE_CMD__RIGHT    = 0x41,
    BLE_CMD__UP       = 0x42,
    BLE_CMD__DOWN     = 0x43,
    BLE_CMD__DISPENSE = 0x50,
} ble_cmd_t;

#define MAX_DEVICEID_STRING_LEN 16

extern int  DeviceIdLen;
extern char DeviceId [MAX_DEVICEID_STRING_LEN];

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
int  ble_enqueue_msg(ble_event_t charact, u32_t data);
void ble_operation_complete(ble_event_t charact, u32_t code);
void ble_button_event(buttons_id_t id);
int  ble_policy_init(void);
void ble_device_name(void);

#endif  // __BLE_POLICY_H__