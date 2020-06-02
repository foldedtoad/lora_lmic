/*
 *  ble_uuids.h
 */
#ifndef __BLE_UUIDS_H__
#define __BLE_UUIDS_H__

/*
 *   4910FFA9-8D54-4699-851B-7E83A872xxxx
 */
#define PASTE_UUID_BASE 0x72,0xA8,0x83,0x7E,0x1B,0x85,0x99,0x46,0x54,0x8D,0xA9,0xFF,0x10,0x49

#define PASTE_UUID_SERVICE            0x00,0x00
#define PASTE_UUID_NOTIFY             0x01,0x00
#define PASTE_UUID_VOICE              0x02,0x00

/*
 *  Service UUID:
 */
#define BT_UUID_PASTE_SERVICE \
    BT_UUID_DECLARE_128(PASTE_UUID_SERVICE, PASTE_UUID_BASE)

/*
 *  Characteristics UUIDs:
 */       
#define BT_UUID_PASTE_NOTIFY   \
    BT_UUID_DECLARE_128(PASTE_UUID_NOTIFY, PASTE_UUID_BASE)

#define BT_UUID_PASTE_VOICE   \
    BT_UUID_DECLARE_128(PASTE_UUID_VOICE, PASTE_UUID_BASE)

#endif  // __BLE_UUIDS_H__