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
#include "driver/rmt_types.h"

#ifdef __cplusplus
extern "C" {
#endif /**< _cplusplus */


typedef struct {
    gpio_num_t tx;
    gpio_num_t rx;
} ir_config_t;

#define IR_CONFIG_DEFAULT() { \
        .tx = GPIO_NUM_NC, \
        .rx = GPIO_NUM_NC, \
    }

esp_err_t ir_init(const ir_config_t *config);

esp_err_t ir_recv(rmt_symbol_word_t **data, size_t *size);

esp_err_t ir_send(const rmt_symbol_word_t *data, size_t size);

esp_err_t ir_deinit(void);

#ifdef __cplusplus
}
#endif /**< _cplusplus */
