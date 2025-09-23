#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_err.h"

#include "qmi8658.h"

#define TCA_PIN_IMU_INT1 4
#define TCA_PIN_IMU_INT2 3

#define QMI8658_ADDR QMI8658_ADDRESS_HIGH

    extern qmi8658_dev_t qmi_dev;

    extern qmi8658_data_t qmi_data;

    /**
     * @brief 初始化 QMI8658 IMU 服务
     *
     * 负责获取 I2C 总线、配置传感器并启动后台采样任务（若需要）。
     */
    esp_err_t qmi8658_service_init(void);

    /**
     * @brief IMU 后台任务入口（由系统任务创建时调用）
     */
    void qmi8658_task(void *arg);

#ifdef __cplusplus
}
#endif