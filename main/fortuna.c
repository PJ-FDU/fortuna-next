#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"

// 硬件服务
#include "i2c_service.h"
#include "mic_service.h"
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
#include "driver/gpio.h"

#include "lvgl.h" // LVGL主头文件

#define TAG "fortuna"

#define WIFI_SSID "ziroom_3501A" // Wi-Fi 名称
#define WIFI_PASS "4001001111"   // Wi-Fi 密码

static void on_audio_after_wakeup(const int16_t *pcm, size_t samples)
{
    ESP_LOGI(TAG, "on_audio_after_wakeup");
    static uint32_t idx = 0;
    if ((++idx % 10) != 0)
        return; // 每 ~320ms 打印一次
    int n = samples < 10 ? samples : 10;
    printf("[PCM] %u:", samples);
    for (int i = 0; i < n; ++i)
        printf(" %d", pcm[i]);
    printf("\n");
}

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
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
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

    // 启动触摸服务任务

    // 3. 初始化音频系统
    // ESP_LOGI(TAG, "Initializing audio system...");
    // ESP_ERROR_CHECK(audio_system_init(on_vad_state_changed, NULL));

    // 4. 初始化WiFi服务
    // ESP_LOGI(TAG, "Initializing WiFi service...");
    // ESP_ERROR_CHECK(wifi_service_init(WIFI_SSID, WIFI_PASS, on_wifi_status_changed));

    // ESP_LOGI(TAG, "=== System initialization completed ===");
    // ESP_LOGI(TAG, "System is running, waiting for events...");

    mic_service_cfg_t mic = {
        .rx_port = I2S_NUM_1,
        .gpio_bclk = GPIO_NUM_15, // MIC_SCK
        .gpio_ws = GPIO_NUM_2,    // MIC_WS (LRCK)
        .gpio_din = GPIO_NUM_39,  // MIC_SD
        .gpio_mclk = I2S_GPIO_UNUSED,

        .sample_rate = 16000,
        .data_bits = I2S_DATA_BIT_WIDTH_32BIT, // 推荐
        .slot_mode = I2S_SLOT_MODE_MONO,
        .slot_mask = I2S_STD_SLOT_RIGHT, // 如无声再试 LEFT
        .frame_ms = 20,
        .shift_bits = 14, // 32→16 右移
        .print_head = 0,

        .awake_silence_back_ms = 800,
    };

    mic_service_set_callback(on_audio_after_wakeup);
    ESP_ERROR_CHECK(mic_service_start(&mic));

    // 主循环 - 系统空闲
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}