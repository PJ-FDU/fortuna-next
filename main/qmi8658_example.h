#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// QMI8658完整示例的对外接口
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化QMI8658 IMU系统
 * 
 * @return esp_err_t ESP_OK成功，其他值表示错误
 */
esp_err_t qmi8658_system_init(void);

/**
 * @brief 启动QMI8658数据采集任务
 * 
 * @return esp_err_t ESP_OK成功，其他值表示错误
 */
esp_err_t qmi8658_start_data_collection(void);

/**
 * @brief 停止QMI8658数据采集任务
 * 
 * @return esp_err_t ESP_OK成功，其他值表示错误
 */
esp_err_t qmi8658_stop_data_collection(void);

#ifdef __cplusplus
}
#endif