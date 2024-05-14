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

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "driver/ledc.h"
#include "esp_timer.h"

#include "led_pwm.h"
#include "led_common.h"
#include <math.h>


#define DATA_MULTIPLE    10

#define GAMMA_CORRECTION 0.8                               /**< Gamma curve parameter */
#define GAMMA_TABLE_SIZE ((LED_PWM_BRIGHTNESS_MAX * 10) + 1) /**< Gamma table size, used for led fade*/
#define DUTY_SET_CYCLE   (20)                                /**< Set duty cycle */


#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE

#define LEDC_DUTY_RES          LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY              (8191) // Set duty to 100%. (2 ** 13) - 1 = 4095
#define LEDC_FREQUENCY         (5000) // Frequency in Hertz. Set frequency at 5 kHz

typedef struct {
    int cur;
    int final;
    int step;
    int cycle;
    size_t num;
} led_fade_data_t;

typedef struct {
    gpio_num_t gpio;
    led_fade_data_t fade_config;
} led_info_t;

static bool g_timer_running_flag = false;

static const char *TAG = "led_pwm";

static led_pwm_config_t *g_led_config;
static led_info_t **g_led_info;
static esp_timer_handle_t g_time_handle;
static uint32_t *g_gamma_table;

static int led_gpio_to_channel(gpio_num_t gpio)
{
    int channel = -1;

    for (int i = 0; i < g_led_config->max_channel; i++) {
        if (g_led_info[i] && g_led_info[i]->gpio == gpio) {
            channel = i;
            break;
        }
    }

    return channel;
}

static void led_pwm_fade_timercb(void *arg)
{
    uint8_t idle_channel_num = 0;

    for (int i = 0; i < g_led_config->max_channel; i++) {
        led_info_t *led_info         = g_led_info[i];
        led_fade_data_t *fade_config = &led_info->fade_config;

        if (led_info == NULL || led_info->fade_config.num == 0) {
            idle_channel_num++;
            continue;
        }

        // ESP_LOGW(TAG, "[%s, %d] %d, num: %d", __func__, __LINE__, i, fade_config->num);

        fade_config->cur += fade_config->step;
        fade_config->num--;
        int duty = fade_config->cur;

        if (fade_config->cycle == 0) {
            if (fade_config->num == 0) {
                duty = fade_config->final;
            }
        } else {
            if (fade_config->num <= 0
                    || fade_config->cur >= fade_config->final
                    || fade_config->cur <= 0) {
                fade_config->num = fade_config->cycle;
                fade_config->step *= -1;

                if (fade_config->cur > fade_config->final) {
                    fade_config->cur = fade_config->final;
                } else if (fade_config->cur < 0) {
                    fade_config->cur = 0;
                }
            }
        }

        uint32_t gamma_duty = g_led_config->duty_inversion ? LEDC_DUTY - g_gamma_table[duty / 100] : g_gamma_table[duty / 100];
        ledc_set_duty(LEDC_LS_MODE, i, gamma_duty);
        ledc_update_duty(LEDC_LS_MODE, i);
    }

    if (idle_channel_num == g_led_config->max_channel) {
        g_timer_running_flag = false;
        esp_timer_stop(g_time_handle);
    }
}

esp_err_t led_pwm_set(gpio_num_t gpio, uint8_t duty, uint32_t fade_ms)
{
    int channel = led_gpio_to_channel(gpio);

    if (channel == -1) {
        ESP_LOGE(TAG, "gpio is not initialized");
        return ESP_ERR_INVALID_ARG;
    }

    led_info_t *led_info = g_led_info[channel];
    led_fade_data_t *fade_config = &led_info->fade_config;
    fade_config->final = duty * 1000;
    fade_config->num   = (fade_ms > DUTY_SET_CYCLE) ? fade_ms / DUTY_SET_CYCLE : 1;;
    fade_config->cycle = 0;
    fade_config->step  = (fade_config->final - fade_config->cur) / fade_config->num;

    if (g_timer_running_flag == false) {
        if (fade_config->num > 1) {
            g_timer_running_flag = true;
            esp_timer_start_periodic(g_time_handle, DUTY_SET_CYCLE * 1000U);
        } else {
            led_pwm_fade_timercb(NULL);
        }
    }

    return ESP_OK;
}

esp_err_t led_pwm_blink_start(gpio_num_t gpio, uint8_t duty, uint32_t period_ms)
{
    ESP_LOGD(TAG, "gpio: %d, duty: %d, period_ms: %d", gpio, duty, period_ms);

    int channel = led_gpio_to_channel(gpio);

    if (channel == -1) {
        ESP_LOGE(TAG, "gpio is not initialized");
        return ESP_ERR_INVALID_ARG;
    }

    led_info_t *led_info = g_led_info[channel];
    led_fade_data_t *fade_config = &led_info->fade_config;
    fade_config->final = duty * 1000;
    fade_config->num   = fade_config->cycle = period_ms / 2 / DUTY_SET_CYCLE;
    fade_config->step  = fade_config->final / fade_config->cycle;

    if (fade_config->cur > fade_config->final) {
        fade_config->step *= -1;
    }

    ESP_LOGD(TAG, "final: %d, cycle: %d, num: %d, step: %d",
             fade_config->final, fade_config->cycle, fade_config->num, fade_config->step);

    if (g_timer_running_flag == false) {
        g_timer_running_flag = true;
        esp_timer_start_periodic(g_time_handle, DUTY_SET_CYCLE * 1000U);
    }

    return ESP_OK;
}


esp_err_t led_pwm_blink_stop(gpio_num_t gpio)
{
    int channel = led_gpio_to_channel(gpio);

    if (channel == -1) {
        ESP_LOGE(TAG, "gpio is not initialized");
        return ESP_ERR_INVALID_ARG;
    }

    led_info_t *led_info = g_led_info[channel];
    led_fade_data_t *fade_config = &led_info->fade_config;
    fade_config->cycle = fade_config->num = 0;
    fade_config->final = 0;

    return ESP_OK;
}

esp_err_t led_pwm_create(gpio_num_t gpio)
{
    if (g_led_config == NULL) {
        ESP_LOGE(TAG, "led_pwm is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    int channel = -1;

    for (int i = 0; i < g_led_config->max_channel; i++) {
        if (g_led_info[i] == NULL) {
            channel = i;
            break;
        } else if (g_led_info[i]->gpio == gpio) {
            ESP_LOGE(TAG, "gpio is already used");
            return ESP_ERR_INVALID_ARG;
        }
    }

    if (channel == -1) {
        ESP_LOGE(TAG, "No more LED strip can be initialized");
        return ESP_ERR_INVALID_STATE;
    }

    led_info_t *led_info = g_led_info[channel] = calloc(1, sizeof(led_info_t));
    led_info->gpio = gpio;

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = g_led_config->duty_resolution,
        .freq_hz = g_led_config->freq_hz,
        .speed_mode = g_led_config->speed_mode,
        .timer_num = g_led_config->timer_num,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .duty = 0,
        .speed_mode = g_led_config->speed_mode,
        .hpoint = 0,
        .timer_sel = g_led_config->timer_num,
        .channel = channel,
        .gpio_num = gpio,
    };

    ledc_channel_config(&ledc_channel);

    return ESP_OK;
}

esp_err_t led_pwm_delete(gpio_num_t gpio)
{
    int channel = led_gpio_to_channel(gpio);

    if (channel == -1) {
        ESP_LOGE(TAG, "gpio is not initialized");
        return ESP_ERR_INVALID_ARG;
    }

    free(g_led_info[channel]);
    g_led_info[channel] = NULL;

    free(g_gamma_table);
    g_gamma_table = NULL;

    return ESP_OK;
}

esp_err_t led_pwm_init(const led_pwm_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    g_led_config = (led_pwm_config_t *)calloc(1, sizeof(led_pwm_config_t));
    memcpy(g_led_config, config, sizeof(led_pwm_config_t));

    g_led_info = (led_info_t **)calloc(g_led_config->max_channel, sizeof(led_info_t *));


    if (g_gamma_table == NULL) {
        g_gamma_table = calloc(GAMMA_TABLE_SIZE, sizeof(uint32_t));
        led_gamma_table_create(g_gamma_table, GAMMA_TABLE_SIZE, LEDC_DUTY, 1 / 1.8);
    }

    esp_timer_create_args_t timer_cfg = {
        .name = "led_pwm_fade",
        .callback = led_pwm_fade_timercb,
        .dispatch_method = ESP_TIMER_TASK,
    };

    esp_timer_create(&timer_cfg, &g_time_handle);

    return ESP_OK;
}

esp_err_t led_pwm_deinit()
{
    if (g_led_config == NULL) {
        ESP_LOGE(TAG, "led_pwm is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < g_led_config->max_channel; i++) {
        if (g_led_info[i] != NULL) {
            free(g_led_info[i]);
            g_led_info[i] = NULL;
        }
    }

    free(g_led_info);
    g_led_info = NULL;
    free(g_led_config);
    g_led_config = NULL;

    return ESP_OK;
}