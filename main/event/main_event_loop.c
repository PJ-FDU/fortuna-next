#include "esp_event.h"
#include "main_event_loop.h"

#define TAG "main_event_loop"

esp_err_t main_event_loop_init(void)
{
    // 创建默认事件循环
    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to create default event loop: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Default event loop created or already exists");

    return ESP_OK;
}