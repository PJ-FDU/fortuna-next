#include "audio_system_service.h"
#include "i2s_service.h"
#include "mic_service.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "audio_system";

static vad_callback_t s_vad_callback = NULL;
static network_ready_callback_t s_network_callback = NULL;

esp_err_t audio_system_init(vad_callback_t vad_cb, network_ready_callback_t network_cb)
{
    ESP_LOGI(TAG, "Initializing audio system...");

    s_vad_callback = vad_cb;
    s_network_callback = network_cb;

    // I2S服务配置
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
        .print_head = 0, // 在网络传输时，通常关闭终端打印
    };
    
    // 设置VAD状态变化回调
    if (s_vad_callback) {
        i2s_service_set_vad_callback(s_vad_callback);
    }
    
    ESP_RETURN_ON_ERROR(i2s_service_start(&cfg), TAG, "I2S service start failed");

    ESP_LOGI(TAG, "Audio system initialized successfully");
    return ESP_OK;
}

esp_err_t audio_system_start_network_services(void)
{
    ESP_LOGI(TAG, "Starting network-dependent audio services...");

    esp_err_t ret = mic_service_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mic_service_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Network audio services started successfully");
    return ESP_OK;
}