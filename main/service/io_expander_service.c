// 清理并统一 io_expander_service 的包含与实现，避免重复定义
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_io_expander_tca9554.h"

#include "i2c_service.h"
#include "io_expander_service.h"

#define TAG "IO_EXPANDER_SERVICE"

// I2C 总线句柄（本地，仅用于初始化）
static i2c_master_bus_handle_t i2c_master_bus_handle = NULL;

// IO 扩展器全局句柄，供其他模块访问（在头文件中声明为 extern）
esp_io_expander_handle_t io_expander_handle = NULL;

/**
 * @brief 初始化 IO 扩展器服务
 *
 * 步骤：
 * 1. 检查是否已初始化，避免重复创建实例。
 * 2. 获取 I2C 总线句柄（要求 I2C 服务已先初始化）。
 * 3. 创建 TCA9554 IO 扩展器实例并保存全局句柄。
 *
 * @return ESP_OK 成功，其他错误码表示失败。
 */
esp_err_t esp_io_expander_service_init(void)
{
    if (io_expander_handle != NULL)
    {
        ESP_LOGW(TAG, "IO Expander already initialized");
        return ESP_OK;
    }

    // 获取 I2C 总线句柄，避免在这处使用 ESP_ERROR_CHECK 直接导致重启；改为捕获并返回错误
    esp_err_t err = i2c_master_get_bus_handle(PIN_I2C_PORT, &i2c_master_bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_get_bus_handle failed: %s", esp_err_to_name(err));
        return err;
    }

    if (i2c_master_bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C bus not initialized. Please initialize I2C service first.");
        return ESP_ERR_INVALID_STATE;
    }

    // 创建 TCA9554 实例，默认地址选择 ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000 (0x20)
    err = esp_io_expander_new_i2c_tca9554(
        i2c_master_bus_handle,
        ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
        &io_expander_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create IO Expander: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "IO Expander initialized successfully");
    return ESP_OK;
}

/**
 * @brief 获取 IO 扩展器全局句柄的安全接口
 *
 * @param out_handle 输出参数，成功时填充句柄
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG 参数为 NULL
 *         ESP_ERR_INVALID_STATE 尚未初始化
 */
esp_err_t esp_io_expander_service_get_handle(esp_io_expander_handle_t *out_handle)
{
    if (out_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (io_expander_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    *out_handle = io_expander_handle;
    return ESP_OK;
}
