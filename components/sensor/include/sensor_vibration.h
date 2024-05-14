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

#ifdef __cplusplus
extern "C" {
#endif /**< _cplusplus */

#define SENSOR_VIBRATION_CONFIG_DEFAULT()   { \
        .gpio_num = GPIO_NUM_NC, \
        .action_level = 1, \
    }

typedef struct {
    gpio_num_t gpio_num;
    bool action_level;
} sensor_vibration_config_t;

esp_err_t sensor_vibration_init(const sensor_vibration_config_t *config);

esp_err_t sensor_vibration_read(uint32_t *count);

#ifdef __cplusplus
}
#endif /**< _cplusplus */
