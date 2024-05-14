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
#include "esp_mn_speech_commands.h"


#ifdef __cplusplus
extern "C" {
#endif /**< _cplusplus */

#define SPEECH_MIC_CONFIG_DEFAULT() { \
        .mic_gpio = { \
                      .i2s_bclk = GPIO_NUM_39, \
                      .i2s_ws = GPIO_NUM_38, \
                      .i2s_din = GPIO_NUM_40, \
                    }, \
    }

typedef struct {
    struct {
        gpio_num_t i2s_bclk;
        gpio_num_t i2s_ws;
        gpio_num_t i2s_din;
    } mic_gpio;
} speech_mic_config_t;


esp_err_t speech_mic_init(const speech_mic_config_t *config);

esp_err_t speech_mic_record(int **audio_data, size_t *audio_len);

esp_err_t speech_recognition_init();

esp_err_t speech_recognition_wakenet(const int16_t *audio_data, size_t audio_len, bool *wakenet_state);

esp_err_t speech_recognition_command(const int16_t *audio_data, size_t audio_len, int *commands_id);


#ifdef __cplusplus
}
#endif /**< _cplusplus */
