#include "esp_err.h"
#include "esp_log.h"

#include "pcf85063a.h"

#include "i2c_service.h"
#include "pcf85063a_service.h"

#define TAG "PCF85063A_SERVICE"

pcf85063a_dev_t pcf_dev;

pcf85063a_datetime_t pcf_datetime;

esp_err_t pcf85063a_service_init(void)
{
    i2c_master_bus_handle_t i2c_master_bus_handle = NULL;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(PIN_I2C_PORT, &i2c_master_bus_handle));

    ESP_ERROR_CHECK(pcf85063a_init(&pcf_dev, i2c_master_bus_handle, PCF85063A_ADDRESS));

    // 创建时间结构体并初始化
    pcf85063a_datetime_t init_time = {
        .year = 2025,
        .month = 9,
        .day = 13,
        .dotw = 6, // 星期五 (0=周日, 6=周六)
        .hour = 12,
        .min = 0,
        .sec = 0};
    ESP_ERROR_CHECK(pcf85063a_set_time_date(&pcf_dev, init_time));

    ESP_LOGI(TAG, "PCF85063A initialized successfully");
    return ESP_OK;
};