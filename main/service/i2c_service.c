// I2C 服务实现
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "i2c_service.h"

#define TAG "I2C_SERVICE"

// I2C 总线句柄，初始化后用于后续 I2C 操作
static i2c_master_bus_handle_t bus_handle = NULL;

/**
 * @brief 初始化 I2C 主总线
 *
 * 1. 检查是否已初始化，避免重复初始化。
 * 2. 配置 I2C 主总线参数（端口、引脚、时钟源、内部上拉等）。
 * 3. 创建 I2C 主总线，返回初始化结果。
 *
 * @return ESP_OK 初始化成功
 *         其他错误码 失败
 */
esp_err_t esp_i2c_service_init(void)
{
    // 已初始化则直接返回，防止重复初始化
    if (bus_handle != NULL)
    {
        ESP_LOGW(TAG, "I2C bus already initialized");
        return ESP_OK;
    }

    // 配置 I2C 主总线参数
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,    // 时钟源，默认即可
        .i2c_port = PIN_I2C_PORT,             // I2C 端口号（如 I2C_NUM_0）
        .scl_io_num = PIN_I2C_SCL,            // SCL 引脚号
        .sda_io_num = PIN_I2C_SDA,            // SDA 引脚号
        .glitch_ignore_cnt = 7,               // 抖动滤波，建议默认
        .flags.enable_internal_pullup = true, // 启用内部上拉，部分板卡可省略外部上拉
    };

    // 创建 I2C 主总线
    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C master bus initialized successfully");
    return ESP_OK;
}
// Note: use driver function i2c_master_get_bus_handle() when a bus handle is needed.