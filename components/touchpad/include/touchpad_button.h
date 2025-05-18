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

#include "esp_err.h"
#include "driver/gpio.h"

#include "touch_element/touch_button.h"

#ifdef __cplusplus
extern "C" {
#endif /**< _cplusplus */

#define TOUCHPAD_BUTTON_CONFIG_DEFAULT() { \
        .max_channel = 8, \
        .longpress_ms = 3000, \
    }

typedef struct {
    uint8_t max_channel;
    uint32_t longpress_ms;
} touchpad_button_config_t;


typedef enum {
    TOUCHPAD_BUTTON_EVT_NONE = -1,        //!< No event
    TOUCHPAD_BUTTON_EVT_ON_PRESS,         //!< Button Press event
    TOUCHPAD_BUTTON_EVT_ON_RELEASE,       //!< Button Release event
    TOUCHPAD_BUTTON_EVT_ON_LONGPRESS,     //!< Button LongPress event
    TOUCHPAD_BUTTON_EVT_MAX
} touchpad_button_event_t;

typedef void (* touchpad_button_cb_t)(gpio_num_t gpio, touchpad_button_event_t event);

esp_err_t touchpad_button_init(const touchpad_button_config_t *config);

esp_err_t touchpad_button_deinit(void);

esp_err_t touchpad_button_create(gpio_num_t gpio, float sensitivity, touchpad_button_cb_t cb);

esp_err_t touchpad_button_delete(gpio_num_t gpio);

#ifdef __cplusplus
}
#endif /**< _cplusplus */
