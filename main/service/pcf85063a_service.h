#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_err.h"

#include "pcf85063a.h"

/**
 * @brief 初始化 PCF85063A 服务
 *
 * - 获取 I2C 总线句柄并初始化底层驱动
 * - 尝试读取 RTC 时间；如果设备返回无效时间，则写入合理的默认时间
 * - 本函数可重入（idempotent）：若服务已初始化则返回 ESP_OK 并不做重复初始化
 */
esp_err_t pcf85063a_service_init(void);

/**
 * @brief 读取缓存的 RTC 时间（线程安全）
 *
 * - 读取在初始化或写入后填充的缓存时间，以避免每次调用都进行 I2C 读取
 * - 若服务未初始化，返回 ESP_ERR_INVALID_STATE
 */
esp_err_t pcf85063a_service_read_datetime(pcf85063a_datetime_t *out_dt);

/**
 * @brief 写入 RTC 时间（线程安全）
 *
 * - 将提供的时间写入设备，成功后更新内部缓存
 * - 写操作与读缓存通过内部互斥保护序列化
 */
esp_err_t pcf85063a_service_write_datetime(const pcf85063a_datetime_t *in_dt);

/**
 * @brief 卸载/关闭服务并释放资源
 *
 * - 可安全多次调用；后续调用为 no-op
 */
esp_err_t pcf85063a_service_teardown(void);

#ifdef __cplusplus
}
#endif