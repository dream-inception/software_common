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
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif /**< _cplusplus */


#define LED_PWM_BRIGHTNESS_MAX      (100)
#define LED_PWM_BRIGTHNESS_INVALID  (-1)

#define LED_PWM_CONFIG_DEFAULT() { \
        .max_channel = 4, \
        .speed_mode = LEDC_LOW_SPEED_MODE, \
        .duty_resolution = LEDC_TIMER_13_BIT, \
        .timer_num = LEDC_TIMER_0, \
        .freq_hz = 5000, \
        .duty_inversion = false, \
    }

typedef struct {
    uint8_t max_channel;
    bool duty_inversion;

    ledc_mode_t speed_mode;
    ledc_timer_bit_t duty_resolution;
    ledc_timer_t timer_num;
    uint32_t freq_hz;
} led_pwm_config_t;


esp_err_t led_pwm_init(const led_pwm_config_t *config);

esp_err_t led_pwm_deinit(void);

esp_err_t led_pwm_create(gpio_num_t gpio);

esp_err_t led_pwm_delete(gpio_num_t gpio);

esp_err_t led_pwm_set(gpio_num_t gpio, uint8_t duty, uint32_t fade_ms);

esp_err_t led_pwm_blink_start(gpio_num_t gpio, uint8_t duty, uint32_t period_ms);

esp_err_t led_pwm_blink_stop(gpio_num_t gpio);


#ifdef __cplusplus
}
#endif /**< _cplusplus */
