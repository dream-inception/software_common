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

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "lwip/ip_addr.h"

#include "lwip/lwip_napt.h"

#include "dns_server.h"


#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_DISCONNECTED_BIT BIT1

static const char *TAG = "wifi";
static bool g_reconnect_flag = true;
static EventGroupHandle_t s_wifi_event_group = NULL;


/* Event handler for catching system events */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    // ESP_LOGW(TAG, "event_base: %s, event_id: %ld", event_base, event_id);

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (g_reconnect_flag) {
            ESP_LOGI(TAG, "sta disconnect, g_reconnect_flag...");
            esp_wifi_connect();
        } else {
            // ESP_LOGI(TAG, "sta disconnect");
        }

        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;
        ESP_LOGI(TAG, "Connected to %s (bssid: "MACSTR", channel: %d)", event->ssid,
                 MAC2STR(event->bssid), event->channel);
    }  else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

esp_err_t wifi_init()
{
    esp_event_loop_create_default();
    ESP_ERROR_CHECK(esp_netif_init());

    if (!s_wifi_event_group) {
        s_wifi_event_group = xEventGroupCreate();
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    // esp_err_t esp_wifi_set_country_code(const char *country, bool ieee80211d_enabled);
    ESP_ERROR_CHECK(esp_wifi_set_country_code("CN", false));


    return ESP_OK;
}

bool wifi_sta_is_connected()
{
    return xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT;
}

esp_err_t wifi_sta_set_config(const char *ssid, const char *password, uint32_t wait_ms)
{
    if (password && strlen(password) && strlen(password) < 8) {
        ESP_LOGE(TAG, "password less than 8");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGW(TAG, "ssid: %s, password: %s, wait_ms: %ld", ssid, password, wait_ms);
    int bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, true, 0);

    if (bits & WIFI_CONNECTED_BIT) {
        g_reconnect_flag = false;
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        xEventGroupWaitBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT, false, true, portMAX_DELAY);
    }

    g_reconnect_flag = true;

    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode);
    static esp_netif_t *s_netif_sta = NULL;

    if ((mode & WIFI_MODE_STA) == 0 && s_netif_sta == NULL) {
        s_netif_sta = esp_netif_create_default_wifi_sta();
        mode |= WIFI_MODE_STA;
        ESP_ERROR_CHECK(esp_wifi_set_mode(mode));
    }

    wifi_config_t wifi_config = {0};
    strlcpy((char *) wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    if (password) {
        strlcpy((char *) wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_wifi_connect();

    if (wait_ms) {
        int bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, true, pdMS_TO_TICKS(wait_ms));

        if (!(bits & WIFI_CONNECTED_BIT)) {
            return ESP_ERR_TIMEOUT;
        }
    }

    return ESP_OK;
}

esp_err_t wifi_sta_get_config(char *ssid, char *password)
{
    wifi_config_t wifi_config = {0};
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &wifi_config));

    if (ssid) {
        strlcpy(ssid, (char *) wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid));
    }

    if (password) {
        strlcpy(password, (char *) wifi_config.sta.password, sizeof(wifi_config.sta.password));
    }

    return ESP_OK;
}

esp_err_t wifi_sta_reconnect(bool enable)
{
    g_reconnect_flag = enable;
    return ESP_OK;
}

esp_err_t wifi_ap_set_config(const char *ssid, const char *password)
{
    if (password && strlen(password) < 8) {
        ESP_LOGE(TAG, "password less than 8");
        return ESP_FAIL;
    }

    static esp_netif_t *s_netif_ap = NULL;

    if (!s_netif_ap) {
        s_netif_ap = esp_netif_create_default_wifi_ap();
    }

    uint8_t channel = 13;

    if (wifi_sta_is_connected()) {
        wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
        esp_wifi_get_channel(&channel, &second);
    }

    ESP_LOGW(TAG, "ssid: %s, password: %s, channel: %d", 
             ssid, password? password: "NULL", channel);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len       = strlen(ssid),
            .channel        = channel,
            .max_connection = 3,
        },
    };
    
    strlcpy((char *) wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));

    if (password && strlen(password)) {
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
        strlcpy((char *) wifi_config.ap.password, password, sizeof(wifi_config.ap.password));
    }

    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 0, 1);        // 192.168.0.1
    IP4_ADDR(&ip_info.gw, 192, 168, 0, 1);        // 192.168.0.1
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0); // 255.255.255.0

    esp_netif_dhcps_stop(s_netif_ap);
    esp_netif_set_ip_info(s_netif_ap, &ip_info);
    
    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode);
    mode |= WIFI_MODE_AP;

    ESP_ERROR_CHECK(esp_wifi_set_mode(mode));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    // ESP_ERROR_CHECK(esp_wifi_start());

    // esp_netif_dhcps_start(s_netif_ap);

    return ESP_OK;
}

#if CONFIG_LWIP_IP_FORWARD && CONFIG_LWIP_IPV4_NAPT

esp_err_t wifi_ap_nat_enable(bool enable)
{
    esp_netif_t *netif_ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_ip_info_t ap_ip;
    ESP_ERROR_CHECK(esp_netif_get_ip_info(netif_ap, &ap_ip));

    // ESP_LOGW(TAG, "AP IP: " IPSTR, IP2STR(&ap_ip.ip));
    ip_napt_enable(ap_ip.ip.addr, enable);

    return ESP_OK;
}

#endif

static const ip_addr_t ipaddr = IPADDR4_INIT_BYTES(192, 168, 0, 1);
/* handle any DNS requests from dns-server */
static bool dns_query_proc(const char *name, ip_addr_t *addr)
{
    /**
     * captive: enerate_204, cp.a, hotspot-detect.html
     */
#if CONFIG_LWIP_IPV6
    ESP_LOGD(TAG, "name: %s, ip_addr: " IPSTR, name, IP2STR(&addr->u_addr.ip4));
#else
    ESP_LOGW(TAG, "name: %s, ip_addr: " IPSTR, name, IP2STR(addr));
#endif

    *addr = ipaddr;
    return true;
}

esp_err_t wifi_ap_dns_redirect_enable(bool enable)
{
    esp_err_t ret = ESP_OK;
    static bool s_dns_redirect_flag = false;

    if (enable && s_dns_redirect_flag == false) {
        esp_netif_t *netif_ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        esp_netif_ip_info_t ap_ip;
        ESP_ERROR_CHECK(esp_netif_get_ip_info(netif_ap, &ap_ip));

        if (dnserv_init(&ipaddr, DNS_SERVER_PORT, dns_query_proc) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start DNS server");
            return ESP_FAIL;
        }

        s_dns_redirect_flag = true;
        ESP_LOGI(TAG, "DNS server started");
    } else {
        dnserv_free();
        s_dns_redirect_flag = false;
        ESP_LOGI(TAG, "DNS server stopped");
    }

    return ret;
}
