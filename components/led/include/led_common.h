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
#include "led_strip.h"

#ifdef __cplusplus
extern "C" {
#endif /**< _cplusplus */


esp_err_t led_gamma_table_create(uint32_t *data, size_t size, uint32_t range, float correction);

esp_err_t led_hsv2rgb(uint16_t hue, uint8_t saturation, uint8_t value,
                      uint8_t *red, uint8_t *green, uint8_t *blue);


esp_err_t led_rgb2hsv(uint8_t red, uint8_t green, uint8_t blue,
                      uint16_t *hue, uint8_t *saturation, uint8_t *value);

#ifdef __cplusplus
}
#endif /**< _cplusplus */