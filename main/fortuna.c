#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

// 硬件服务
#include "i2c_service.h"
#include "io_expander_service.h"
#include "lcd_service.h"

// 高级服务
// #include "wifi_service.h"
#include "ui_system_service.h"
// #include "audio_system_service.h"
// #include "ui/voice_overlay.h"
#include "lcd_touch_service.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_partition.h" // 添加分区API
#include "esp_heap_caps.h" // 添加内存查看
#include "esp_psram.h"
#include "esp_spiffs.h"
#include "dirent.h"

#include "lvgl.h" // LVGL主头文件

static const char *TAG = "FORTUNA";

#define WIFI_SSID "ziroom_3501A" // Wi-Fi 名称
#define WIFI_PASS "4001001111"   // Wi-Fi 密码


// VAD状态变化回调函数
// static void on_vad_state_changed(bool vad_active)
// {
//     ESP_LOGI(TAG, "VAD state changed: %s", vad_active ? "ACTIVE" : "INACTIVE");

//     if (vad_active)
//     {
//         voice_overlay_show();
//     }
//     else
//     {
//         voice_overlay_hide();
//     }
// }

// WiFi连接状态回调函数
// static void on_wifi_status_changed(bool connected)
// {
//     ESP_LOGI(TAG, "WiFi status changed: %s", connected ? "CONNECTED" : "DISCONNECTED");

//     if (connected)
//     {
//         // 网络就绪，启动音频网络服务
//         audio_system_start_network_services();
//     }
// }

/*===========================*
 *           app_main
 *===========================*/
void app_main(void)
{
    ESP_LOGI(TAG, "=== Fortuna System Starting ===");

    // 1. 初始化硬件服务
    ESP_LOGI(TAG, "Initializing hardware services...");
    ESP_ERROR_CHECK(esp_i2c_service_init());
    ESP_ERROR_CHECK(esp_io_expander_service_init());
    ESP_ERROR_CHECK(lcd_service_init());
    // ESP_ERROR_CHECK(lcd_touch_service_init());

    // 2. 初始化UI系统
    ESP_LOGI(TAG, "Initializing UI system...");
    /* 获取面板句柄并传给 UI 系统；统一错误处理以便更明显地定位未初始化情况 */
    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_io_handle_t panel_io = NULL;
    // esp_lcd_touch_handle_t touch = NULL;
    ESP_ERROR_CHECK(lcd_service_get_panel(&panel));
    ESP_ERROR_CHECK(lcd_service_get_panel_io(&panel_io));
    // ESP_ERROR_CHECK(lcd_touch_service_get_handle(&touch));
    ESP_ERROR_CHECK(ui_system_init(panel, panel_io, NULL));

    // 3. 初始化音频系统
    // ESP_LOGI(TAG, "Initializing audio system...");
    // ESP_ERROR_CHECK(audio_system_init(on_vad_state_changed, NULL));

    // 4. 初始化WiFi服务
    // ESP_LOGI(TAG, "Initializing WiFi service...");
    // ESP_ERROR_CHECK(wifi_service_init(WIFI_SSID, WIFI_PASS, on_wifi_status_changed));

    // ESP_LOGI(TAG, "=== System initialization completed ===");
    // ESP_LOGI(TAG, "System is running, waiting for events...");

    // 主循环 - 系统空闲
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}