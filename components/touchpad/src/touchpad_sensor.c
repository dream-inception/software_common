/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "touchpad_sensor.h"

static const char *TAG = "touchpad_sensor";

esp_err_t touchpad_sensor_read(gpio_num_t gpio_num, uint32_t *value)
{
    return touch_pad_read_raw_data(gpio_num, value);
}


esp_err_t touchpad_sensor_create(gpio_num_t gpio_num)
{
    touch_pad_config(gpio_num);
    return ESP_OK;
}

esp_err_t touchpad_sensor_init(const touchpad_sensor_config_t *config)
{
    /* Initialize touch pad peripheral. */
    // touch_pad_init();

    /* Denoise setting at TouchSensor 0. */
    touch_pad_denoise_t denoise = {
        /* The bits to be cancelled are determined according to the noise level. */
        .grade = config->grade,
        .cap_level = config->cap_level,
    };
    touch_pad_denoise_set_config(&denoise);
    touch_pad_denoise_enable();
    ESP_LOGI(TAG, "Denoise function init");

    /* Enable touch sensor clock. Work mode is "timer trigger". */
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_fsm_start();

    return ESP_OK;
}

esp_err_t touchpad_sensor_deinit(void)
{
    touch_pad_deinit();
    return ESP_OK;
}
