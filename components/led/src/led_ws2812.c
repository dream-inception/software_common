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
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "led_strip.h"
#include "led_ws2812.h"
#include "led_common.h"


// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define DATA_MULTIPLE    1
#define LED_STRIP_RMT_RES_HZ    (10 * 1000 * 1000)
#define GAMMA_TABLE_SIZE        (255 * DATA_MULTIPLE + 1)
#define DUTY_SET_CYCLE          (20)  /**< Set duty cycle */

typedef struct {
    struct {
        int cur;
        int final;
        int step;
    } color[3];
    int cycle;
    int num;
} led_fade_data_t;

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    gpio_num_t gpio;
    uint32_t num;
    uint32_t index;
    led_strip_handle_t handle;
    led_fade_data_t fade_config;
} led_info_t;

static const char *TAG    = "led_ws2812";
static led_ws2812_config_t *g_led_config;
static led_info_t **g_led_info;
static uint32_t *g_gamma_table;
static bool g_timer_running_flag = false;
static esp_timer_handle_t g_time_handle;

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

esp_err_t led_ws2812_create(gpio_num_t gpio, uint32_t num)
{
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
    led_info->num = num;

    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .led_pixel_format = g_led_config->pixel_format, // Pixel format of your LED strip
        .led_model        = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
        .strip_gpio_num   = led_info->gpio,       // GPIO number
        .max_leds         = led_info->num,             // Number of LED pixels
    };

    if (channel < 3) {
        led_strip_rmt_config_t rmt_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
            .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
            .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
        };

        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_info->handle));
    } else {
        led_strip_spi_config_t spi_config = {
            .clk_src = SPI_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
            .flags.with_dma = true, // Using DMA can improve performance and help drive more LEDs
            .spi_bus = SPI2_HOST + (channel - 2),   // SPI bus ID
        };
        ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_info->handle));
    }

    led_strip_clear(led_info->handle);

    return ESP_OK;
}

esp_err_t led_ws2812_delete(gpio_num_t gpio_num)
{
    if (g_led_config == NULL) {
        ESP_LOGE(TAG, "led_ws2812 is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    int channel = led_gpio_to_channel(gpio_num);

    if (channel == -1) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_led_info[channel]->handle != NULL) {
        led_strip_del(g_led_info[channel]->handle);
    }

    free(g_led_info[channel]);
    g_led_info[channel] = NULL;

    return ESP_OK;
}


static void led_ws2812_fade_timercb(void *arg)
{
    uint8_t idle_channel_num = 0;

    for (int i = 0; i < g_led_config->max_channel; i++) {
        led_info_t *led_info         = g_led_info[i];
        led_fade_data_t *fade_config = &led_info->fade_config;

        if (led_info == NULL || led_info->fade_config.num == 0) {
            idle_channel_num++;
            continue;
        }

        for (int j = 0; j < 3; j++) {
            fade_config->color[j].cur += fade_config->color[j].step;

            if (fade_config->color[j].cur < 0) {
                fade_config->color[j].cur = 0;
            } else if ((fade_config->color[j].step > 0 && fade_config->color[j].cur >= fade_config->color[j].final) 
                || (fade_config->cycle == 0 && fade_config->color[j].step < 0 && fade_config->color[j].cur <= fade_config->color[j].final )
                || (fade_config->cycle == 0 && fade_config->num == 1)) {
                fade_config->color[j].cur = fade_config->color[j].final;
            }
        }

        // ESP_LOGW(TAG, "color: %d, step: %d, num: %d, cycle: %d",
        //  fade_config->color[1].cur, fade_config->color[1].step, fade_config->num, fade_config->cycle);
        // ESP_LOGW(TAG, "step: %d, %d, %d", fade_config->color[0].step, fade_config->color[1].step, fade_config->color[2].step);
        fade_config->num--;

        // for (int k = 0; k < 255; k++) {

        //     printf("%d ", g_gamma_table[k]);
        //     if (k % 10 == 0) {
        //         printf("\n");
        //     }
        // }
        // printf("\n");
        
        // ESP_LOGW(TAG, "color: %d, %d, %d", fade_config->color[0].cur, fade_config->color[1].cur, fade_config->color[2].cur);
        uint8_t r = g_gamma_table[(fade_config->color[0].cur + 500) / 1000];
        uint8_t g = g_gamma_table[(fade_config->color[1].cur + 500) / 1000];
        uint8_t b = g_gamma_table[(fade_config->color[2].cur + 500) / 1000];

        // uint8_t r = fade_config->color[0].cur / 1000;
        // uint8_t g = fade_config->color[1].cur / 1000;
        // uint8_t b = fade_config->color[2].cur / 1000;

        if (fade_config->cycle) {
            if (fade_config->num <= 0) {
                fade_config->num = fade_config->cycle;

                for (int i = 0; i < 3; ++i) {
                    fade_config->color[i].step *= -1;
                }
            }
        }

        if (led_info->index == led_info->num) {
            for (int i = 0; i < led_info->num; i++) {
                led_strip_set_pixel(led_info->handle, i, r, g, b);
            }
        } else {
            led_strip_set_pixel(led_info->handle, led_info->index, r, g, b);
        }

        led_strip_refresh(led_info->handle);
    }

    if (g_timer_running_flag && idle_channel_num == g_led_config->max_channel) {
        g_timer_running_flag = false;
        esp_timer_stop(g_time_handle);
    }
}

esp_err_t led_ws2812_set_rgb(gpio_num_t gpio_num, uint32_t index, uint8_t red, uint8_t green, uint8_t blue, uint32_t fade_ms)
{
    // ESP_LOGI(TAG, "led_ws2812_set_rgb, gpio: %d, index: %d, r: %d, g: %d, b: %d, fade_ms: %d", gpio_num, index, red, green, blue, fade_ms);
    int channel = led_gpio_to_channel(gpio_num);

    if (channel == -1) {
        return ESP_ERR_INVALID_ARG;
    }

    if (index > g_led_info[channel]->num) {
        ESP_LOGE(TAG, "index is invalid, max_leds = %d", g_led_info[channel]->num);
        return ESP_ERR_INVALID_ARG;
    }

    led_info_t *led_info = g_led_info[channel];
    led_fade_data_t *fade_config = &led_info->fade_config;

// TODO: Implement fade_ms

    if (fade_config->cycle) {
        led_ws2812_blink_stop(gpio_num);
    }

    if (fade_ms == 0) {
         red = g_gamma_table[red];
         green = g_gamma_table[green];
         blue = g_gamma_table[blue];
        if (index == led_info->num) {
            for (int i = 0; i < led_info->num; i++) {
                led_strip_set_pixel(led_info->handle, i, red, green, blue);
            }
        } else {
            led_strip_set_pixel(led_info->handle, index, red, green, blue);
        }

        led_strip_refresh(led_info->handle);

        return ESP_OK;
    }

    fade_config->num = (fade_ms > DUTY_SET_CYCLE) ? fade_ms / DUTY_SET_CYCLE : 1;
    fade_config->cycle = 0;
    fade_config->color[0].final = red * 1000;
    fade_config->color[1].final = green * 1000;
    fade_config->color[2].final = blue * 1000;
    led_info->index = index;

    for (int i = 0; i < 3; i++) {
        fade_config->color[i].step = (fade_config->color[i].final - fade_config->color[i].cur) / fade_config->num;
    }
    // ESP_LOGW(TAG, "fade_config, cur: %d, %d, %d", fade_config->color[0].cur, fade_config->color[1].cur, fade_config->color[2].cur);
    // ESP_LOGW(TAG, "fade_config, final: %d, %d, %d", fade_config->color[0].final, fade_config->color[1].final, fade_config->color[2].final);
    // ESP_LOGW(TAG, "fade_config, step: %d, %d, %d", fade_config->color[0].step, fade_config->color[1].step, fade_config->color[2].step);
    // ESP_LOGW(TAG, "fade_config, num: %d", fade_config->num);

    if (g_timer_running_flag == false) {
        if (fade_config->num > 1) {
            g_timer_running_flag = true;
            esp_timer_start_periodic(g_time_handle, DUTY_SET_CYCLE * 1000U);
        } else {
            led_ws2812_fade_timercb(NULL);
        }
    }

    return ESP_OK;
}

esp_err_t led_ws2812_set_buffer(gpio_num_t gpio_num, uint32_t index, uint32_t color)
{
    int channel = led_gpio_to_channel(gpio_num);

    if (channel == -1) {
        return ESP_ERR_INVALID_ARG;
    }

    if (index > g_led_info[channel]->num) {
        ESP_LOGE(TAG, "index is invalid, max_leds = %d", g_led_info[channel]->num);
        return ESP_ERR_INVALID_ARG;
    }

    led_info_t *led_info = g_led_info[channel];
    led_fade_data_t *fade_config = &led_info->fade_config;

    if (fade_config->cycle) {
        led_ws2812_blink_stop(gpio_num);
    }

    uint8_t red = (color >> 16) & 0xFF;
    uint8_t green = (color >> 8) & 0xFF;
    uint8_t blue = color & 0xFF;
    // ESP_LOGI(TAG, "led_ws2812_set_buffer, gpio: %d, index: %d, r: %d, g: %d, b: %d", gpio_num, index, red, green, blue);

    red = g_gamma_table[red];
    green = g_gamma_table[green];
    blue = g_gamma_table[blue];

    if (index == led_info->num) {
        for (int i = 0; i < led_info->num; i++) {
            led_strip_set_pixel(led_info->handle, i, red, green, blue);
        }
    } else {
        led_strip_set_pixel(led_info->handle, index, red, green, blue);
    }

    return ESP_OK;
}

esp_err_t led_ws2812_refresh(gpio_num_t gpio_num)
{
    int channel = led_gpio_to_channel(gpio_num);

    if (channel == -1) {
        return ESP_ERR_INVALID_ARG;
    }

    led_info_t *led_info = g_led_info[channel];
    led_fade_data_t *fade_config = &led_info->fade_config;

    if (fade_config->cycle) {
        led_ws2812_blink_stop(gpio_num);
    }

    led_strip_refresh(led_info->handle);
    return ESP_OK;
}

esp_err_t led_ws2812_blink_start(gpio_num_t gpio, uint32_t index, uint8_t red, uint8_t green, uint8_t blue, uint32_t period_ms)
{
    ESP_LOGI(TAG, "led_ws2812_blink_start, gpio: %d, index: %d, r: %d, g: %d, b: %d, period_ms: %d", gpio, index, red, green, blue, period_ms);

    int channel = led_gpio_to_channel(gpio);

    if (channel == -1) {
        ESP_LOGE(TAG, "gpio_num: %d is invalid", gpio);
        return ESP_ERR_INVALID_ARG;
    }

    led_info_t *led_info = g_led_info[channel];
    led_fade_data_t *fade_config = &led_info->fade_config;

    if (fade_config->cycle) {
        led_ws2812_blink_stop(gpio);
    }

    led_info->index = index;
    fade_config->num = period_ms / 2 / DUTY_SET_CYCLE;
    fade_config->cycle = fade_config->num;
    fade_config->color[0].final = red * 1000;
    fade_config->color[1].final = green * 1000;
    fade_config->color[2].final = blue * 1000;

    for (int i = 0; i < 3; i++) {
        fade_config->color[i].step = fade_config->color[i].final / fade_config->num;

        if (fade_config->color[i].cur > fade_config->color[i].final) {
            fade_config->color[i].step *= -1;
        }
    }

    for (int i = 0; i < 3; i++) {
        ESP_LOGW(TAG, "fade_config, num: %d, color[%d], cur: %d, final: %d, step: %d", fade_config->num,
                i, fade_config->color[i].cur, fade_config->color[i].final, fade_config->color[i].step);
    }

    if (g_timer_running_flag == false) {
        g_timer_running_flag = true;
        esp_timer_start_periodic(g_time_handle, DUTY_SET_CYCLE * 1000U);
    }

    return ESP_OK;
}

esp_err_t led_ws2812_fade_start()
{
    if (g_timer_running_flag == false) {
        g_timer_running_flag = true;
        esp_timer_start_periodic(g_time_handle, DUTY_SET_CYCLE * 1000U);
    }

    return ESP_OK;
}

esp_err_t led_ws2812_blink_stop(gpio_num_t gpio_num)
{
    int channel = led_gpio_to_channel(gpio_num);

    if (channel == -1) {
        ESP_LOGE(TAG, "gpio_num is invalid");
        return ESP_ERR_INVALID_ARG;
    }

    led_info_t *led_info = g_led_info[channel];
    led_fade_data_t *fade_config = &led_info->fade_config;
    fade_config->num = 0;
    fade_config->cycle = 0;

    vTaskDelay(DUTY_SET_CYCLE / portTICK_PERIOD_MS);

    led_ws2812_set_rgb(gpio_num, led_info->index, 0, 0, 0, 0);
    ESP_LOGW(TAG, "led_ws2812_blink_stop, gpio: %d", gpio_num);

    return ESP_OK;
}

esp_err_t led_ws2812_set_hsv(gpio_num_t gpio, uint32_t index, uint16_t hue, uint8_t saturation, uint8_t value, uint32_t fade_ms)
{
    uint8_t r, g, b;
    led_hsv2rgb(hue, saturation, value, &r, &g, &b);
    return led_ws2812_set_rgb(gpio, index, r, g, b, fade_ms);
}

esp_err_t led_ws2812_init(const led_ws2812_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    g_led_config = (led_ws2812_config_t *)calloc(1, sizeof(led_ws2812_config_t));
    memcpy(g_led_config, config, sizeof(led_ws2812_config_t));

    g_led_info = calloc(g_led_config->max_channel, sizeof(led_info_t *));

    if (g_gamma_table == NULL) {
        g_gamma_table = calloc(GAMMA_TABLE_SIZE, sizeof(uint32_t));
        led_gamma_table_create(g_gamma_table, GAMMA_TABLE_SIZE, GAMMA_TABLE_SIZE, 1 / 1.8);
    }

    if (g_time_handle == NULL) {
        const esp_timer_create_args_t led_ws2812_fade_timer_args = {
            .callback = led_ws2812_fade_timercb,
            .name = "led_ws2812_fade_timer"
        };

        ESP_ERROR_CHECK(esp_timer_create(&led_ws2812_fade_timer_args, &g_time_handle));
    }

    return ESP_OK;
}


esp_err_t led_ws2812_deinit()
{
    if (g_led_config == NULL) {
        ESP_LOGE(TAG, "led_ws2812 is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < g_led_config->max_channel; i++) {
        if (g_led_info[i] != NULL) {
            if (g_led_info[i]->handle != NULL) {
                led_strip_del(g_led_info[i]->handle);
            }

            free(g_led_info[i]);
        }
    }

    free(g_led_info);
    g_led_info = NULL;
    free(g_led_config);
    g_led_config = NULL;

    return ESP_OK;
}
