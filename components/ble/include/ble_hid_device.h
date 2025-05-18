
// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif /**< _cplusplus */
// // HID Keyboard/Keypad Usage IDs (subset of the codes available in the USB HID Usage Tables spec)

/**
 * @brief Modifier key bit mask for HID keyboard
 */
typedef enum {
    BLE_HID_KEYBOARD_CMD_NONE           = 0,
    BLE_HID_KEYBOARD_CMD_LEFT_CONTROL   = 1 << 0,
    BLE_HID_KEYBOARD_CMD_LEFT_SHIFT     = 1 << 1,
    BLE_HID_KEYBOARD_CMD_LEFT_ALT       = 1 << 2,
    BLE_HID_KEYBOARD_CMD_LEFT_GUI       = 1 << 3,
    BLE_HID_KEYBOARD_CMD_RIGHT_CONTROL  = 1 << 4,
    BLE_HID_KEYBOARD_CMD_RIGHT_SHIFT    = 1 << 5,
    BLE_HID_KEYBOARD_CMD_RIGHT_ALT      = 1 << 6,
    BLE_HID_KEYBOARD_CMD_RIGHT_GUI      = 1 << 7,
} ble_hid_keyboard_cmd_t;

typedef enum {
    BLE_HID_CONSUMER_BUTTON_POWER          = 48,  /**< Power */
    BLE_HID_CONSUMER_BUTTON_RESET          = 49,  /**< Reset */
    BLE_HID_CONSUMER_BUTTON_SLEEP          = 50,  /**< Sleep */

    BLE_HID_CONSUMER_BUTTON_MENU           = 64,  /**< Menu */
    BLE_HID_CONSUMER_BUTTON_SELECTION      = 128, /**< Selection */
    BLE_HID_CONSUMER_BUTTON_ASSIGN_SEL     = 129, /**< Assign Selection */
    BLE_HID_CONSUMER_BUTTON_MODE_STEP      = 130, /**< Mode Step */
    BLE_HID_CONSUMER_BUTTON_RECALL_LAST    = 131, /**< Recall Last */
    BLE_HID_CONSUMER_BUTTON_QUIT           = 148, /**< Quit */
    BLE_HID_CONSUMER_BUTTON_HELP           = 149, /**< Help */
    BLE_HID_CONSUMER_BUTTON_CHANNEL_UP     = 156, /**< Channel Increment */
    BLE_HID_CONSUMER_BUTTON_CHANNEL_DOWN   = 157, /**< Channel Decrement */

    BLE_HID_CONSUMER_BUTTON_PLAY           = 176, /**< Play */
    BLE_HID_CONSUMER_BUTTON_PAUSE          = 177, /**< Pause */
    BLE_HID_CONSUMER_BUTTON_RECORD         = 178, /**< Record */
    BLE_HID_CONSUMER_BUTTON_FAST_FORWARD   = 179, /**< Fast Forward */
    BLE_HID_CONSUMER_BUTTON_REWIND         = 180, /**< Rewind */
    BLE_HID_CONSUMER_BUTTON_SCAN_NEXT_TRK  = 181, /**< Scan Next Track */
    BLE_HID_CONSUMER_BUTTON_SCAN_PREV_TRK  = 182, /**< Scan Previous Track */
    BLE_HID_CONSUMER_BUTTON_STOP           = 183, /**< Stop */
    BLE_HID_CONSUMER_BUTTON_EJECT          = 184, /**< Eject */
    BLE_HID_CONSUMER_BUTTON_RANDOM_PLAY    = 185, /**< Random Play */
    BLE_HID_CONSUMER_BUTTON_SELECT_DISC    = 186, /**< Select Disk */
    BLE_HID_CONSUMER_BUTTON_ENTER_DISC     = 187, /**< Enter Disc */
    BLE_HID_CONSUMER_BUTTON_REPEAT         = 188, /**< Repeat */
    BLE_HID_CONSUMER_BUTTON_STOP_EJECT     = 204, /**< Stop/Eject */
    BLE_HID_CONSUMER_BUTTON_PLAY_PAUSE     = 205, /**< Play/Pause */
    BLE_HID_CONSUMER_BUTTON_PLAY_SKIP      = 206, /**< Play/Skip */

    BLE_HID_CONSUMER_BUTTON_VOLUME         = 224, /**< Volume */
    BLE_HID_CONSUMER_BUTTON_BALANCE        = 225, /**< Balance */
    BLE_HID_CONSUMER_BUTTON_MUTE           = 226, /**< Mute */
    BLE_HID_CONSUMER_BUTTON_BASS           = 227, /**< Bass */
    BLE_HID_CONSUMER_BUTTON_VOLUME_UP      = 233, /**< Volume Increment */
    BLE_HID_CONSUMER_BUTTON_VOLUME_DOWN    = 234, /**< Volume Decrement */
} ble_hid_consumer_button_t;


/**
 * @brief Mouse button bit mask
 */
typedef enum {
    BLE_HID_MOUSE_BUTTON_NONE   = 0,        /**< No button pressed */
    BLE_HID_MOUSE_BUTTON_LEFT   = 1 << 0,   /**< Left button */
    BLE_HID_MOUSE_BUTTON_RIGHT  = 1 << 1,   /**< Right button */
    BLE_HID_MOUSE_BUTTON_MIDDLE = 1 << 2,   /**< Middle button, typically the mouse wheel */
    BLE_HID_MOUSE_BUTTON_SIDE1  = 1 << 3,   /**< Side button 1 */
    BLE_HID_MOUSE_BUTTON_SIDE2  = 1 << 4,   /**< Side button 2 */
} ble_hid_mouse_button_t;

esp_err_t ble_hid_device_init(const char *device_name);

esp_err_t ble_hid_device_deinit(void);

esp_err_t ble_hid_keyboard_send(ble_hid_keyboard_cmd_t key_mask, char key);
esp_err_t ble_hid_mouse_send(ble_hid_mouse_button_t button, int8_t mickeys_x, int8_t mickeys_y, int8_t wheel);
esp_err_t ble_hid_consumer_send(ble_hid_consumer_button_t button, bool pressed);

bool ble_hid_is_connected(void);

#ifdef __cplusplus
}
#endif /**< _cplusplus */
