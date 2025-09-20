#include "wifi_service.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_check.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "wifi_service";

static wifi_status_callback_t s_status_callback = NULL;
static bool s_wifi_connected = false;
static char s_wifi_ssid[32] = {0};
static char s_wifi_password[64] = {0};

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "Wi-Fi station started, beginning scan...");
            esp_wifi_scan_start(NULL, false);
            break;

        case WIFI_EVENT_SCAN_DONE:
        {
            ESP_LOGI(TAG, "Wi-Fi scan finished.");
            uint16_t ap_count = 0;
            esp_wifi_scan_get_ap_num(&ap_count);
            if (ap_count == 0)
            {
                ESP_LOGW(TAG, "No APs found. Retrying scan...");
                vTaskDelay(pdMS_TO_TICKS(2000));
                esp_wifi_scan_start(NULL, false);
                break;
            }

            wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_count);
            if (!ap_list)
            {
                ESP_LOGE(TAG, "Failed to allocate memory for AP list");
                break;
            }
            ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_list));

            bool found = false;
            for (int i = 0; i < ap_count; i++)
            {
                ESP_LOGI(TAG, "Found AP: %s, RSSI: %d, Channel: %d",
                         ap_list[i].ssid, ap_list[i].rssi, ap_list[i].primary);

                if (strcmp((char *)ap_list[i].ssid, s_wifi_ssid) == 0)
                {
                    ESP_LOGI(TAG, "Found target AP '%s'. Attempting to connect...", s_wifi_ssid);
                    wifi_config_t wifi_config;
                    memset(&wifi_config, 0, sizeof(wifi_config_t));
                    strcpy((char *)wifi_config.sta.ssid, s_wifi_ssid);
                    strcpy((char *)wifi_config.sta.password, s_wifi_password);

                    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
                    ESP_ERROR_CHECK(esp_wifi_connect());
                    found = true;
                    break;
                }
            }
            free(ap_list);

            if (!found)
            {
                ESP_LOGW(TAG, "Target AP '%s' not found. Retrying scan after delay...", s_wifi_ssid);
                vTaskDelay(pdMS_TO_TICKS(5000));
                esp_wifi_scan_start(NULL, false);
            }
            break;
        }

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "Connected to AP: %s", s_wifi_ssid);
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected from AP. Will retry connection...");
            s_wifi_connected = false;
            if (s_status_callback) {
                s_status_callback(false);
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
            esp_wifi_scan_start(NULL, false);
            break;

        default:
            break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_connected = true;
        
        if (s_status_callback) {
            s_status_callback(true);
        }
    }
}

esp_err_t wifi_service_init(const char *ssid, const char *password, wifi_status_callback_t status_cb)
{
    if (!ssid || !password) {
        return ESP_ERR_INVALID_ARG;
    }

    s_status_callback = status_cb;
    strncpy(s_wifi_ssid, ssid, sizeof(s_wifi_ssid) - 1);
    strncpy(s_wifi_password, password, sizeof(s_wifi_password) - 1);

    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "NVS init failed");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop create failed");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "WiFi init failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL), TAG, "WiFi event register failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL), TAG, "IP event register failed");

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "WiFi set mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "WiFi start failed");

    ESP_LOGI(TAG, "WiFi service initialized successfully");
    return ESP_OK;
}

bool wifi_service_is_connected(void)
{
    return s_wifi_connected;
}