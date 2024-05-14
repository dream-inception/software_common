// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "iot_button.h"
#include "sensor_vibration.h"

static const char *TAG = "sensor_vibration";
static uint32_t g_vibration_count = 0;

static void vibration_event_cb(void *arg, void *data)
{
    g_vibration_count++;
    ESP_LOGD(TAG, "Button event %d, vibration_count: %u", (int)data, g_vibration_count);
}

esp_err_t sensor_vibration_read(uint32_t *count)
{
    *count = g_vibration_count;
    g_vibration_count = 0;

    return ESP_OK;
}

esp_err_t sensor_vibration_init(const sensor_vibration_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "sensor vibration config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->gpio_num == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "sensor vibration gpio is invalid");
        return ESP_ERR_INVALID_ARG;
    }


    button_config_t btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num    = config->gpio_num,
            .active_level = config->action_level,
        },
    };
    button_handle_t btn = iot_button_create(&btn_cfg);
    esp_err_t ret = iot_button_register_cb(btn, BUTTON_PRESS_DOWN, vibration_event_cb, (void *)BUTTON_PRESS_DOWN);

    return ret;
}
