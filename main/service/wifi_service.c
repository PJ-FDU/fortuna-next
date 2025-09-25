#include "wifi_service.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "wifi_service";

// 外部回调暂时不使用，内部实现简单日志回调
// static wifi_status_callback_t s_status_callback = NULL;
static bool s_wifi_connected = false;
static char s_wifi_ssid[32];
static char s_wifi_password[64];

/* 重试节流/指数退避参数与定时器 */
static esp_timer_handle_t s_scan_timer = NULL;
static int s_scan_retry_count = 0;
static const int s_scan_base_ms = 2000;   // 初始重试间隔 2s
static const int s_scan_max_ms = 60000;   // 最大退避 60s

static void scan_timer_cb(void *arg)
{
    ESP_LOGD(TAG, "scan_timer_cb: starting WiFi scan");
    esp_wifi_scan_start(NULL, false);
}

static void schedule_scan_with_backoff(bool reset)
{
    if (!s_scan_timer) {
        /* 保护性：若未创建定时器，则立即发起扫描 */
        esp_wifi_scan_start(NULL, false);
        return;
    }
    if (reset) {
        s_scan_retry_count = 0;
        esp_timer_stop(s_scan_timer);
        return;
    }
    /* 阶梯式增大间隔：base * 2^retry, 上限 s_scan_max_ms */
    if (s_scan_retry_count < 30) s_scan_retry_count++;
    int64_t delay_ms = (int64_t)s_scan_base_ms * (1LL << (s_scan_retry_count > 10 ? 10 : s_scan_retry_count));
    if (delay_ms > s_scan_max_ms) delay_ms = s_scan_max_ms;
    ESP_LOGI(TAG, "scheduling next scan in %lld ms (retry_count=%d)", delay_ms, s_scan_retry_count);
    esp_timer_start_once(s_scan_timer, delay_ms * 1000);
}

/* 辅助：安全复制，保证以 '\0' 结尾 */
static void strncpy_safe(char *dst, const char *src, size_t len)
{
    if (!dst || len == 0) return;
    if (!src) { dst[0] = '\0'; return; }
    strncpy(dst, src, len - 1);
    dst[len - 1] = '\0';
}

/* 工作队列：把重工作放到独立任务处理，避免阻塞系统事件任务 */
typedef enum {
    WIFI_WORK_SCAN_DONE = 1,
} wifi_work_t;

static QueueHandle_t s_wifi_work_queue = NULL;

static void wifi_worker_task(void *arg)
{
    wifi_work_t work;
    while (1) {
        if (xQueueReceive(s_wifi_work_queue, &work, portMAX_DELAY) == pdTRUE) {
            if (work == WIFI_WORK_SCAN_DONE) {
                ESP_LOGI(TAG, "worker: handling scan done");
                uint16_t ap_count = 0;
                esp_wifi_scan_get_ap_num(&ap_count);
                if (ap_count == 0) {
                    ESP_LOGW(TAG, "worker: No APs found.");
                    schedule_scan_with_backoff(false);
                    continue;
                }

                wifi_ap_record_t *ap_list = calloc(ap_count, sizeof(wifi_ap_record_t));
                if (!ap_list) {
                    ESP_LOGE(TAG, "worker: Failed to allocate memory for AP list (count=%d)", ap_count);
                    continue;
                }
                if (esp_wifi_scan_get_ap_records(&ap_count, ap_list) != ESP_OK) {
                    ESP_LOGE(TAG, "worker: Failed to get AP records");
                    free(ap_list);
                    continue;
                }

                bool found = false;
                for (uint16_t i = 0; i < ap_count; i++) {
                    ESP_LOGD(TAG, "worker: Found AP: %s (RSSI=%d)", ap_list[i].ssid, ap_list[i].rssi);
                    if (strncmp((const char *)ap_list[i].ssid, s_wifi_ssid, sizeof(ap_list[i].ssid)) == 0) {
                        ESP_LOGI(TAG, "worker: Found target AP '%s'. Attempting to connect...", s_wifi_ssid);
                        wifi_config_t wifi_config = {0};
                        strncpy_safe((char *)wifi_config.sta.ssid, s_wifi_ssid, sizeof(wifi_config.sta.ssid));
                        strncpy_safe((char *)wifi_config.sta.password, s_wifi_password, sizeof(wifi_config.sta.password));
                        esp_err_t err = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
                        if (err == ESP_OK) {
                            esp_wifi_connect();
                        } else {
                            ESP_LOGE(TAG, "worker: esp_wifi_set_config failed: %d", err);
                        }
                        found = true;
                        break;
                    }
                }
                free(ap_list);

                if (!found) {
                    ESP_LOGW(TAG, "worker: Target AP '%s' not found. Scheduling retry.", s_wifi_ssid);
                    schedule_scan_with_backoff(false);
                } else {
                    schedule_scan_with_backoff(true);
                }
            }
        }
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "Wi-Fi station started, beginning scan...");
            schedule_scan_with_backoff(true); /* reset retry counter */
            esp_wifi_scan_start(NULL, false);
            break;
        case WIFI_EVENT_SCAN_DONE:
            ESP_LOGI(TAG, "Wi-Fi scan finished. posting work to worker task");
            if (s_wifi_work_queue) {
                wifi_work_t w = WIFI_WORK_SCAN_DONE;
                if (xQueueSend(s_wifi_work_queue, &w, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "work queue full, dropping scan_done event");
                }
            } else {
                /* fallback: if no queue, do immediate short processing (minimal) */
                schedule_scan_with_backoff(false);
            }
            break;

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "Connected to AP: %s", s_wifi_ssid);
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected from AP. Scheduling retry scan...");
            s_wifi_connected = false;
            /* 内部日志通知 */
            ESP_LOGI(TAG, "wifi status: DISCONNECTED");
            schedule_scan_with_backoff(false);
            break;

        default:
            break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_connected = true;
        ESP_LOGI(TAG, "wifi status: CONNECTED");
    }
}

esp_err_t wifi_service_init(const char *ssid, const char *password, wifi_status_callback_t status_cb)
{
    if (!ssid || !password) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)status_cb; // 忽略外部回调，保留参数以兼容接口
    strncpy_safe(s_wifi_ssid, ssid, sizeof(s_wifi_ssid));
    strncpy_safe(s_wifi_password, password, sizeof(s_wifi_password));

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
    /* 创建退避定时器（单次回调） */
    if (!s_scan_timer) {
        const esp_timer_create_args_t targs = {
            .callback = &scan_timer_cb,
            .arg = NULL,
            .name = "wifi_scan_timer"
        };
        if (esp_timer_create(&targs, &s_scan_timer) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to create scan timer (continuing without backoff)");
            s_scan_timer = NULL;
        }
    }

    /* 创建工作队列和任务，用于处理 scan_done 的重工作，避免阻塞事件回调 */
    if (!s_wifi_work_queue) {
        s_wifi_work_queue = xQueueCreate(4, sizeof(int));
        if (s_wifi_work_queue) {
            BaseType_t ok = xTaskCreate(wifi_worker_task, "wifi_worker", 4096, NULL, configMAX_PRIORITIES - 5, NULL);
            if (ok != pdPASS) {
                ESP_LOGW(TAG, "Failed to create wifi_worker task");
            }
        } else {
            ESP_LOGW(TAG, "Failed to create wifi work queue");
        }
    }

    ESP_LOGI(TAG, "WiFi service initialized successfully");
    return ESP_OK;
}

bool wifi_service_is_connected(void)
{
    return s_wifi_connected;
}