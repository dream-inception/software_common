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

esp_err_t wifi_init(void);

esp_err_t wifi_sta_set_config(const char *ssid, const char *password, uint32_t wait_ms);

esp_err_t wifi_sta_get_config(char *ssid, char *password);

esp_err_t wifi_sta_reconnect(bool enable);

bool wifi_sta_is_connected(void);

esp_err_t wifi_ap_set_config(const char *ssid, const char *password);

esp_err_t wifi_ap_nat_enable(bool enable);
    
esp_err_t wifi_ap_dns_redirect_enable(bool enable);

#ifdef __cplusplus
}
#endif /**< _cplusplus */
