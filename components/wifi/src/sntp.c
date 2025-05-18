// Copyright 2021 Espressif Systems (Shanghai) PTE LTD
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

#include <esp_log.h>

#include "esp_sntp.h"

#define REF_TIME        1577808000 /* 2020-01-01 00:00:00 */
#define DEFAULT_TICKS   (2000 / portTICK_PERIOD_MS) /* 2 seconds in ticks */

static bool g_init_done = false;
static const char *TAG = "sntp";

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event, sec=%llu", tv->tv_sec);
    settimeofday(tv, NULL);
}

esp_err_t sntp_start()
{
    if (g_init_done) {
        ESP_LOGI(TAG, "SNTP already initialized.");
        return ESP_OK;
    }

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "ntp.aliyun.com");
    esp_sntp_setservername(1, "time.asia.apple.com");
    esp_sntp_setservername(2, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    esp_sntp_init();

    /* Set timezone to China Standard Time */
    setenv("TZ", "CST-8", 1);
    tzset();

    g_init_done = true;

    return ESP_OK;
}

bool sntp_check(void)
{
    time_t now;
    time(&now);

    if (now > REF_TIME) {
        return true;
    }

    return false;
}

esp_err_t sntp_wait(uint32_t timeout_ms)
{
    if (!g_init_done) {
        ESP_LOGW(TAG, "Time sync not initialised using 'sntp_start'");
    }

    ESP_LOGW(TAG, "Waiting for time to be synchronized. This may take time.");
    uint32_t ticks_remaining = timeout_ms / portTICK_PERIOD_MS;
    uint32_t ticks = DEFAULT_TICKS;

    while (ticks_remaining > 0) {
        if (sntp_check() == true) {
            break;
        }

        ESP_LOGD(TAG, "Time not synchronized yet. Retrying...");
        ticks = ticks_remaining < DEFAULT_TICKS ? ticks_remaining : DEFAULT_TICKS;
        ticks_remaining -= ticks;
        vTaskDelay(ticks);
    }

    /* Check if timeout_ms expired and time is not synchronized yet. */
    if (sntp_check() == false) {
        ESP_LOGE(TAG, "Time not synchronized within the provided ticks: %lu", timeout_ms);
        return ESP_FAIL;
    }

    /* Get current time */
    struct tm timeinfo;
    char strftime_buf[64];
    time_t now;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current UTC time is: %s", strftime_buf);
    return ESP_OK;
}
