

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_err.h"
#include "esp_io_expander.h"

    /**
     * @brief IO 扩展器全局句柄
     * 用于其他模块访问 IO 扩展器实例
     */
    extern esp_io_expander_handle_t io_expander_handle;

    /**
     * @brief 初始化 IO 扩展器服务
     *
     * 需确保 I2C 服务已初始化
     *
     * @return ESP_OK 成功
     *         其他错误码 失败
     */
    esp_err_t esp_io_expander_service_init(void);

/**
 * @brief 获取 IO 扩展器服务的全局句柄
 *
 * 提供一个安全的方式让其他模块访问已初始化的 io_expander_handle。
 *
 * @param out_handle 输出参数，成功时填充扩展器句柄
 * @return ESP_OK 成功
 *         ESP_ERR_INVALID_ARG out_handle 为 NULL
 *         ESP_ERR_INVALID_STATE 扩展器尚未初始化
 */
esp_err_t esp_io_expander_service_get_handle(esp_io_expander_handle_t *out_handle);

#ifdef __cplusplus
}
#endif