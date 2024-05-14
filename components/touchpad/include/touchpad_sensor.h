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
#include "driver/touch_pad.h"

#ifdef __cplusplus
extern "C" {
#endif /**< _cplusplus */

#define TOUCHPAD_SENSOR_CONFIG_DEFAULT()   { \
        .grade = TOUCH_PAD_DENOISE_BIT4, \
        .cap_level = TOUCH_PAD_DENOISE_CAP_L4, \
    }

/**
 * @brief Touch index type
 */
typedef struct {
    touch_pad_denoise_grade_t grade;
    touch_pad_denoise_cap_t cap_level;
} touchpad_sensor_config_t;

esp_err_t touchpad_sensor_init(const touchpad_sensor_config_t *config);
esp_err_t touchpad_sensor_deinit(void);

esp_err_t touchpad_sensor_create(gpio_num_t gpio);


esp_err_t touchpad_sensor_read(gpio_num_t gpio_num, uint32_t *value);

#ifdef __cplusplus
}
#endif /**< _cplusplus */
