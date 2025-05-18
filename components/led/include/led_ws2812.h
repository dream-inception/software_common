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


#define LED_WS2812_CONFIG_DEFAULT() { \
        .max_channel = 5, \
        .pixel_format = LED_PIXEL_FORMAT_GRB, \
    }

typedef struct {
    uint8_t max_channel; /*!< Maximum number of channels */
    led_pixel_format_t pixel_format; /*!< LED pixel format */
} led_ws2812_config_t;

/**
 * @brief Initialize the eye led
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t led_ws2812_init(const led_ws2812_config_t *config);


esp_err_t led_ws2812_create(gpio_num_t gpio, uint32_t num);


/**
 * @brief Set the eye led color
 *
 * @param r red value
 * @param g green value
 * @param b blue value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t led_ws2812_set_rgb(gpio_num_t gpio, uint32_t index,
                             uint8_t r, uint8_t g, uint8_t b, uint32_t fade_ms);

esp_err_t led_ws2812_set_hsv(gpio_num_t gpio, uint32_t index,
                             uint16_t h, uint8_t s, uint8_t v, uint32_t fade_ms);

esp_err_t led_ws2812_set_buffer(gpio_num_t gpio_num, uint32_t index, uint32_t color);
esp_err_t led_ws2812_refresh(gpio_num_t gpio_num);

/**
 * @brief Start blinking LED
 *
 * @param r red value
 * @param g green value
 * @param b blue value
 * @param period_ms blinking period in milliseconds
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_STATE LED strip is not initialized
 */
esp_err_t led_ws2812_blink_start(gpio_num_t gpio_num, uint32_t index,
                                 uint8_t red, uint8_t green, uint8_t blue,
                                 uint32_t period_ms);

/**
 * @brief Stop blinking LED
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_STATE LED strip is not initialized
 */
esp_err_t led_ws2812_blink_stop(gpio_num_t gpio_num);

/**
 * @brief Deinitialize the eye led
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t led_ws2812_deinit(void);

#ifdef __cplusplus
}
#endif /**< _cplusplus */
