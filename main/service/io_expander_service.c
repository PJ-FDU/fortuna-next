#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_io_expander_tca9554.h"

#include "i2c_service.h"
#include "io_expander_service.h"

#define TAG "IO_EXPANDER_SERVICE"

static i2c_master_bus_handle_t i2c_master_bus_handle = NULL;

// 全局变量定义 - 供其他文件访问
esp_io_expander_handle_t io_expander_handle = NULL;

esp_err_t esp_io_expander_service_init(void)
{
    if (io_expander_handle != NULL)
    {
        ESP_LOGW(TAG, "IO Expander already initialized");
        return ESP_OK;
    }

    // 假设I2C总线已经通过i2c_service初始化
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(PIN_I2C_PORT, &i2c_master_bus_handle));
    if (i2c_master_bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C bus not initialized. Please initialize I2C service first.");
        return ESP_ERR_INVALID_STATE;
    }

    // 创建TCA9554 IO扩展器实例，假设地址为0x20
    esp_err_t err = esp_io_expander_new_i2c_tca9554(i2c_master_bus_handle, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &io_expander_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create IO Expander: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "IO Expander initialized successfully");
    return ESP_OK;
}
