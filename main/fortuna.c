#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>

#include "i2c_service.h"
#include "io_expander_service.h"
#include "lcd_service.h"
#include "lcd_touch_service.h"
#include "lvgl_service.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"
#include <math.h>
#include "lvgl_screens/home.h" /* 抽离出的星盘创建函数 */
#include "i2s_service.h"
#include "mic_service.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_system.h"
#include "nvs_flash.h"

static const char *TAG = "FORTUNA";

#define WIFI_SSID "ziroom_3501A" // Wi-Fi 名称
#define WIFI_PASS "4001001111"   // Wi-Fi 密码

static bool s_wifi_connected = false;

// 统一的 Wi-Fi 事件处理函数
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            // Wi-Fi Station 模式启动后，开始扫描网络
            ESP_LOGI(TAG, "Wi-Fi station started, beginning scan...");
            esp_wifi_scan_start(NULL, false); // false 表示非阻塞扫描
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
                esp_wifi_scan_start(NULL, false); // 重新扫描
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
                // 打印所有扫描到的 AP
                ESP_LOGI(TAG, "Found AP: %s, RSSI: %d, Channel: %d",
                         ap_list[i].ssid, ap_list[i].rssi, ap_list[i].primary);

                if (strcmp((char *)ap_list[i].ssid, WIFI_SSID) == 0)
                {
                    ESP_LOGI(TAG, "Found target AP '%s'. Attempting to connect...", WIFI_SSID);
                    // 找到目标，设置配置并连接
                    wifi_config_t wifi_config;
                    memset(&wifi_config, 0, sizeof(wifi_config_t));
                    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
                    strcpy((char *)wifi_config.sta.password, WIFI_PASS);

                    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
                    ESP_ERROR_CHECK(esp_wifi_connect());
                    found = true;
                    break; // 退出循环
                }
            }
            free(ap_list);

            if (!found)
            {
                ESP_LOGW(TAG, "Target AP '%s' not found. Retrying scan after delay...", WIFI_SSID);
                vTaskDelay(pdMS_TO_TICKS(5000)); // 等待5秒后重试
                esp_wifi_scan_start(NULL, false);
            }
            break;
        }

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "Connected to AP: %s", WIFI_SSID);
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected from AP. Will retry connection...");
            s_wifi_connected = false;
            // 重新开始扫描流程
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

        // 网络就绪！在这里启动依赖网络的服务
        ESP_LOGI(TAG, "Network is ready, starting mic_service...");
        esp_err_t ret = mic_service_start();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "mic_service_start failed: %s", esp_err_to_name(ret));
        }
    }
}

// 配置并启动 Wi-Fi
static void wifi_init_and_scan(void)
{
    // 1. 初始化底层依赖
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // 2. 初始化 Wi-Fi，修复宏使用方式
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 3. 注册统一的事件处理器
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // 4. 设置模式并启动 Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 启动流程完成，后续操作由事件驱动
    ESP_LOGI(TAG, "Wi-Fi initialization complete. Event-driven process will now take over.");
}

/*===========================*
 *      背景样式（屏幕渐变）
 *===========================*/
static lv_style_t style_scr_bg;

static void style_scr_bg_init(void)
{
    lv_style_init(&style_scr_bg);
    lv_style_set_bg_opa(&style_scr_bg, LV_OPA_COVER);
    /* 竖向线性渐变（上浅下深）；可按需改色 */
    lv_style_set_bg_color(&style_scr_bg, lv_color_hex(0x5A6FFF));
    lv_style_set_bg_grad_color(&style_scr_bg, lv_color_hex(0x000000));
    lv_style_set_bg_grad_dir(&style_scr_bg, LV_GRAD_DIR_VER);
}

/* astro_create 相关实现已移动到 lvgl_screens/home.c */

/*===========================*
 *           app_main
 *===========================*/
void app_main(void)
{
    ESP_LOGI(TAG, "System initialization started");

    /* 1) I2C 总线 & IO 扩展（TCA9554） */
    ESP_ERROR_CHECK(esp_i2c_service_init());
    ESP_ERROR_CHECK(esp_io_expander_service_init());

    /* 2) LCD（QSPI + SPD2010 面板） */
    ESP_ERROR_CHECK(lcd_service_init());
    ESP_LOGI(TAG, "LCD ready");

    /* 3) 触摸（如果需要） */
    // ESP_ERROR_CHECK(lcd_touch_service_init());
    // lcd_touch_service_debug_once();

    /* 4) LVGL 服务 */
    ESP_ERROR_CHECK(lvgl_service_init(
        lcd_service_get_panel(),
        lcd_service_get_panel_io(),
        NULL));

    /* ========== 创建 UI ========== */
    lvgl_port_lock(portMAX_DELAY);

    /* 屏幕渐变背景样式 */
    style_scr_bg_init();
    lv_obj_add_style(lv_screen_active(), &style_scr_bg, 0);

    /* 星盘主体（412×412，自绘） */
    astro_create(lv_screen_active());

    lvgl_port_unlock();

    /* 5) Wi-Fi 连接 */
    wifi_init_and_scan();

    /* 6) I2S 服务（麦克风数据采集）*/
    i2s_service_cfg_t cfg = {
        .port = I2S_NUM_1,
        .gpio_bclk = 15,
        .gpio_ws = 2,
        .gpio_din = 39,
        .gpio_mclk = I2S_GPIO_UNUSED,
        .gpio_dout = I2S_GPIO_UNUSED,
        .sample_rate = 16000,
        .data_bits = I2S_DATA_BIT_WIDTH_32BIT,
        .slot_mode = I2S_SLOT_MODE_MONO,
        .slot_mask = I2S_STD_SLOT_RIGHT,
        .frame_ms = 20,
        .shift_bits = 14,
        .print_head = 16, // 在网络传输时，通常关闭终端打印
    };
    ESP_ERROR_CHECK(i2s_service_start(&cfg));

    // 注意：mic_service_start() 已经从这里移除，
    // 它将在 wifi_event_handler 中获取到 IP 后被自动调用。

    ESP_LOGI(TAG, "System initialization completed. Waiting for events.");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 主任务可以轻松一点
    }
}