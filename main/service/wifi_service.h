#pragma once

#include "esp_err.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WiFi连接状态回调函数类型
 * @param connected true表示已连接，false表示断开连接
 */
typedef void (*wifi_status_callback_t)(bool connected);

/**
 * @brief 初始化并启动WiFi连接
 * @param ssid WiFi网络名称
 * @param password WiFi密码
 * @param status_cb WiFi状态变化回调函数
 * @return esp_err_t 操作结果
 */
esp_err_t wifi_service_init(const char *ssid, const char *password, wifi_status_callback_t status_cb);

/**
 * @brief 获取当前WiFi连接状态
 * @return true表示已连接，false表示未连接
 */
bool wifi_service_is_connected(void);

#ifdef __cplusplus
}
#endif