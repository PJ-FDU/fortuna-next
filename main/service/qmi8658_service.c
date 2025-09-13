#include <stdbool.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "qmi8658.h"

#include "i2c_service.h"
#include "io_expander_service.h"
#include "qmi8658_service.h"

#define TAG "QMI8658_SERVICE"

// 全局变量定义 - 供其他文件访问
qmi8658_dev_t qmi_dev;
qmi8658_data_t qmi_data;

esp_err_t qmi8658_service_init(void)
{

    i2c_master_bus_handle_t i2c_master_bus_handle = NULL;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(PIN_I2C_PORT, &i2c_master_bus_handle));

    ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander_handle, (1u << TCA_PIN_IMU_INT1), 0));
    ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander_handle, (1u << TCA_PIN_IMU_INT2), 0));

    ESP_ERROR_CHECK(qmi8658_init(&qmi_dev, i2c_master_bus_handle, QMI8658_ADDR));

    ESP_ERROR_CHECK(qmi8658_set_accel_range(&qmi_dev, QMI8658_ACCEL_RANGE_8G));
    ESP_ERROR_CHECK(qmi8658_set_accel_odr(&qmi_dev, QMI8658_ACCEL_ODR_250HZ));
    ESP_ERROR_CHECK(qmi8658_set_gyro_range(&qmi_dev, QMI8658_GYRO_RANGE_512DPS));
    ESP_ERROR_CHECK(qmi8658_set_gyro_odr(&qmi_dev, QMI8658_GYRO_ODR_250HZ));

    qmi8658_set_accel_unit_mps2(&qmi_dev, false);
    qmi8658_set_gyro_unit_rads(&qmi_dev, false);
    qmi8658_set_display_precision(&qmi_dev, 4);

    ESP_LOGI(TAG, "QMI8658 initialized successfully");
    return ESP_OK;
};

void qmi8658_task(void *arg)
{
    ESP_LOGI(TAG, "QMI8658 task started");

    int last1 = 1, last2 = 1;
    int64_t t_prev = esp_timer_get_time();

    while (1)
    {
        // 读取TCA9554引脚电平 - 使用正确的API
        uint32_t level_mask = 0;

        esp_err_t ret1 = esp_io_expander_get_level(io_expander_handle, (1u << TCA_PIN_IMU_INT1), &level_mask);
        int v1 = (ret1 == ESP_OK && (level_mask & (1u << TCA_PIN_IMU_INT1))) ? 1 : 0;

        esp_err_t ret2 = esp_io_expander_get_level(io_expander_handle, (1u << TCA_PIN_IMU_INT2), &level_mask);
        int v2 = (ret2 == ESP_OK && (level_mask & (1u << TCA_PIN_IMU_INT2))) ? 1 : 0;

        bool edge = ((last1 == 1 && v1 == 0) || (last2 == 1 && v2 == 0)); // 低跳沿
        last1 = v1;
        last2 = v2;

        bool do_read = edge;
        if (!do_read)
        {
            bool ready = false;
            if (qmi8658_is_data_ready(&qmi_dev, &ready) == ESP_OK && ready)
                do_read = true;
        }

        if (do_read)
        {
            if (qmi8658_read_sensor_data(&qmi_dev, &qmi_data) == ESP_OK)
            {
                ESP_LOGI(TAG, "Acc[g]=%.4f %.4f %.4f  Gyro[dps]=%.4f %.4f %.4f  Temp=%.2f  Ts=%lu",
                         qmi_data.accelX, qmi_data.accelY, qmi_data.accelZ,
                         qmi_data.gyroX, qmi_data.gyroY, qmi_data.gyroZ,
                         qmi_data.temperature, (unsigned long)qmi_data.timestamp);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}