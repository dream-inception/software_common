/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "hid_dev.h"

#include "ble_hid_device.h"

/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "ESP-HID"

static char *TAG = "ble_hid_device";
static char g_device_name[32] = HIDD_DEVICE_NAME;
static esp_bd_addr_t s_cached_remote_bda = {0x0,};

static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static bool g_hid_dev_init_flag = false;

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_ble_gap_set_device_name(g_device_name);
                esp_ble_gap_config_adv_data(&hidd_adv_data);
            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	     break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            hid_conn_id = param->connect.conn_id;
	        memcpy(s_cached_remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(TAG, param->vendor_write.data, param->vendor_write.length);
            break;
        }
        case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
            ESP_LOGI(TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
            ESP_LOG_BUFFER_HEX(TAG, param->led_write.data, param->led_write.length);
            break;
        }
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
     case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;
     case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

#define CASE(a, b, c)  \
                case a: \
				*special_key_mask = b;  \
				*keyboard_cmd = c; \
                break;\

static void char_to_code(char ch, key_mask_t *special_key_mask, uint8_t *keyboard_cmd)
{
	// Check if lower or upper case
	if(ch >= 'a' && ch <= 'z')
	{
		*special_key_mask = 0;
		// convert ch to HID letter, starting at a = 4
		*keyboard_cmd = (uint8_t)(4 + (ch - 'a'));
	}
	else if(ch >= 'A' && ch <= 'Z')
	{
		// Add left shift
		*special_key_mask = LEFT_SHIFT_KEY_MASK;
		// convert ch to lower case
		ch = ch - ('A'-'a');
		// convert ch to HID letter, starting at a = 4
		*keyboard_cmd = (uint8_t)(4 + (ch - 'a'));
	}
	else if(ch >= '0' && ch <= '9') // Check if number
	{
		*special_key_mask = 0;
		// convert ch to HID number, starting at 1 = 30, 0 = 39
		if(ch == '0')
		{
			*keyboard_cmd = 39;
		}
		else
		{
			*keyboard_cmd = (uint8_t)(30 + (ch - '1'));
		}
	}
	else // not a letter nor a number
	{
		switch(ch)
		{
			CASE('>', LEFT_SHIFT_KEY_MASK, HID_KEY_DOT);
			CASE('@', LEFT_SHIFT_KEY_MASK, HID_KEY_1);
			CASE('!', LEFT_SHIFT_KEY_MASK, HID_KEY_2);
			CASE('#', LEFT_SHIFT_KEY_MASK, HID_KEY_3);
			CASE('$', LEFT_SHIFT_KEY_MASK, HID_KEY_4);
			CASE('%', LEFT_SHIFT_KEY_MASK, HID_KEY_5);
			CASE('^', LEFT_SHIFT_KEY_MASK, HID_KEY_6);
            CASE('&', LEFT_SHIFT_KEY_MASK, HID_KEY_7);
            CASE('*', LEFT_SHIFT_KEY_MASK, HID_KEY_8);
            CASE('(', LEFT_SHIFT_KEY_MASK, HID_KEY_9);
            CASE(')', LEFT_SHIFT_KEY_MASK, HID_KEY_0);
            CASE(' ', 0, HID_KEY_SPACEBAR);
			CASE('.', 0,HID_KEY_DOT);
            CASE('\n', 0, HID_KEY_RETURN);
			CASE('?', LEFT_SHIFT_KEY_MASK, HID_KEY_FWD_SLASH);
			CASE('/', 0 ,HID_KEY_FWD_SLASH);
			CASE('\\', 0, HID_KEY_BACK_SLASH);
			CASE('|', LEFT_SHIFT_KEY_MASK, HID_KEY_BACK_SLASH);
			CASE(',', 0, HID_KEY_COMMA);
			CASE('<', LEFT_SHIFT_KEY_MASK, HID_KEY_COMMA);
			CASE('-', 0, HID_KEY_MINUS);
			CASE('_', LEFT_SHIFT_KEY_MASK, HID_KEY_MINUS);
			CASE('=', 0, HID_KEY_EQUAL);
			CASE('+', LEFT_SHIFT_KEY_MASK, HID_KEY_EQUAL);
			CASE(8, 0, HID_KEY_DELETE); // backspace
			CASE('\t', 0, HID_KEY_TAB);
			default:
				*special_key_mask = 0;
				*keyboard_cmd = ch;
		}
	}
}

esp_err_t ble_hid_keyboard_send(ble_hid_keyboard_cmd_t key_mask, char key)
{
    key_mask_t special_key_mask = 0;
    uint8_t keyboard_cmd = 0;
    char_to_code(key, &special_key_mask, &keyboard_cmd);

    if (keyboard_cmd == 0) {
        keyboard_cmd = key_mask;
    }

    esp_hidd_send_keyboard_value(hid_conn_id, special_key_mask, &keyboard_cmd, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    keyboard_cmd = 0;
    special_key_mask = 0;
    esp_hidd_send_keyboard_value(hid_conn_id, special_key_mask, &keyboard_cmd, 1);

    return ESP_OK;
}

esp_err_t ble_hid_mouse_send(ble_hid_mouse_button_t button, int8_t mickeys_x, int8_t mickeys_y, int8_t wheel)
{
    if (sec_conn == false) {
        return ESP_FAIL;
    }

    esp_hidd_send_mouse_value(hid_conn_id, button, mickeys_x, mickeys_y);
    return ESP_OK;
}

esp_err_t ble_hid_consumer_send(ble_hid_consumer_button_t button, bool pressed)
{
    if (sec_conn == false) {
        return ESP_FAIL;
    }

    esp_hidd_send_consumer_value(hid_conn_id, button, pressed);

    return ESP_OK;
}


esp_err_t ble_hid_device_init(const char *device_name)
{
    if (g_hid_dev_init_flag == true) {
        return ESP_FAIL;
    }

    if (device_name == NULL) {
        ESP_LOGE(TAG, "device_name is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    strlcpy(g_device_name, device_name, sizeof(g_device_name));

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed", __func__);
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed", __func__);
        return ret;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s init bluedroid failed", __func__);
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluedroid failed", __func__);
        return ret;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(TAG, "%s init bluedroid failed", __func__);
        return ret;
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    g_hid_dev_init_flag = true;

    return ESP_OK;
}


esp_err_t ble_hid_device_deinit() 
{
    if (g_hid_dev_init_flag == false) {
        return ESP_FAIL;
    }

    if (sec_conn) {
        esp_ble_gap_disconnect(s_cached_remote_bda);
    }

    esp_err_t ret = ESP_OK;

    if((ret = esp_hidd_profile_deinit()) != ESP_OK) {
        ESP_LOGE(TAG, "%s init bluedroid failed", __func__);
        return ret;
    }

    ret = esp_bluedroid_disable();
    if (ret) {
        ESP_LOGE(TAG, "%s deinit bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ret = esp_bluedroid_deinit();
    if (ret) {
        ESP_LOGE(TAG, "%s deinit bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    
    ret = esp_bt_controller_disable();
    if (ret) {
        ESP_LOGE(TAG, "%s disable bt controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_deinit();
    if (ret) {
        ESP_LOGE(TAG, "%s deinit bt controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    g_hid_dev_init_flag = false;

    return ESP_OK;
}

bool ble_hid_is_connected()
{
    return sec_conn;
}
