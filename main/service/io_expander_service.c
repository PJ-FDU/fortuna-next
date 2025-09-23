#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_io_expander_tca9554.h"

#include "i2c_service.h"
#include "io_expander_service.h"

#define TAG "IO_EXPANDER_SERVICE"

static esp_io_expander_handle_t s_io_expander_handle = NULL;

esp_err_t esp_io_expander_service_init(void)
{
    if (s_io_expander_handle != NULL)
    {
        ESP_LOGW(TAG, "IO Expander already initialized");
        return ESP_OK;
    }

    i2c_master_bus_handle_t i2c_master_bus_handle = NULL;
    esp_err_t err = i2c_master_get_bus_handle(I2C_PORT_NUM, &i2c_master_bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Get i2c bus handle failed: %s", esp_err_to_name(err));
        return err;
    }
    if (i2c_master_bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C bus not initialized. Please initialize I2C service first.");
        return ESP_ERR_INVALID_STATE;
    }

    err = esp_io_expander_new_i2c_tca9554(
        i2c_master_bus_handle,
        ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
        &s_io_expander_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create IO Expander: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "IO Expander initialized successfully");
    return ESP_OK;
}

esp_err_t esp_io_expander_service_get_handle(esp_io_expander_handle_t *io_expander_handle)
{
    if (io_expander_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_io_expander_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    *io_expander_handle = s_io_expander_handle;
    return ESP_OK;
}
