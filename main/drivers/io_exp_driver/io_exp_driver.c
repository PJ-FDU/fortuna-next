#include "esp_log.h"

#include "i2c_driver.h"

#include "esp_io_expander_tca9554.h"

#include "io_exp_driver.h"

static const char *TAG = "io_exp_driver";

static esp_io_expander_handle_t s_io_expander_handle = NULL;

esp_err_t io_exp_driver_init(void)
{
    if (s_io_expander_handle != NULL)
    {
        ESP_LOGW(TAG, "IO Expander driver has already been initialized");
        return ESP_OK;
    }
    esp_err_t err = ESP_OK;
    i2c_master_bus_handle_t i2c_master_bus_handle = NULL;
    err = i2c_master_get_bus_handle(I2C_NUM_0, &i2c_master_bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Get i2c bus handle failed: %s", esp_err_to_name(err));
        return err;
    }
    if (i2c_master_bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C bus not initialized. Please initialize I2C driver first.");
        return ESP_ERR_INVALID_STATE;
    }
    ESP_ERROR_CHECK(esp_io_expander_new_i2c_tca9554(
        i2c_master_bus_handle,
        ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
        &s_io_expander_handle));
    ESP_LOGI(TAG, "IO Expander initialized successfully");
    return ESP_OK;
}

esp_err_t io_exp_driver_get_handle(esp_io_expander_handle_t *handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_io_expander_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }
    *handle = s_io_expander_handle;
    return ESP_OK;
}
