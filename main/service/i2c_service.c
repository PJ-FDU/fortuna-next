#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "i2c_service.h"

#define TAG "I2C_SERVICE"

static i2c_master_bus_handle_t bus_handle = NULL;

esp_err_t esp_i2c_service_init(void)
{
    if (bus_handle != NULL)
    {
        ESP_LOGW(TAG, "I2C service already initialized");
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC,                  // 时钟源，默认即可
        .i2c_port = I2C_PORT_NUM,                   // I2C 端口号（如 I2C_NUM_0）
        .scl_io_num = I2C_SCL_GPIO,                 // SCL 引脚号
        .sda_io_num = I2C_SDA_GPIO,                 // SDA 引脚号
        .glitch_ignore_cnt = I2C_GLITCH_IGNORE_CNT, // 抖动滤波，建议默认
        .flags.enable_internal_pullup = true,       // 启用内部上拉，部分板卡可省略外部上拉
    };

    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C master bus initialized successfully");
    return ESP_OK;
}
