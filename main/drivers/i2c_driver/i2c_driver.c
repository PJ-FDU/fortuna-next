#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c_master.h"

#include "i2c_driver.h"

static const char *TAG = "i2c_driver";

static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;

esp_err_t i2c_driver_init(void)
{
    if (s_i2c_bus_handle != NULL)
    {
        ESP_LOGW(TAG, "I2C driver has already been initialized");
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_master_bus_config = {
        .clk_source = I2C_CLK_SRC,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_io_num = I2C_SDA_GPIO,
        .glitch_ignore_cnt = I2C_GLITCH_IGNORE_CNT,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&i2c_master_bus_config, &s_i2c_bus_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C driver has initialized successfully");
    return ESP_OK;
}

esp_err_t i2c_driver_get_bus_handle(i2c_master_bus_handle_t *handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_i2c_bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C bus not initialized. Please call i2c_driver_init() first.");
        return ESP_ERR_INVALID_STATE;
    }
    *handle = s_i2c_bus_handle;
    return ESP_OK;
}
