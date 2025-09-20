#pragma once

#include "esp_err.h"
#include "i2s_service.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief VAD状态变化回调函数类型
 * @param vad_active true表示检测到语音，false表示静音
 */
typedef void (*vad_callback_t)(bool vad_active);

/**
 * @brief 网络就绪回调函数类型
 */
typedef void (*network_ready_callback_t)(void);

/**
 * @brief 初始化音频系统（I2S + VAD + MIC服务）
 * @param vad_cb VAD状态变化回调
 * @param network_cb 网络就绪回调
 * @return esp_err_t 操作结果
 */
esp_err_t audio_system_init(vad_callback_t vad_cb, network_ready_callback_t network_cb);

/**
 * @brief 启动网络相关的音频服务（在WiFi连接后调用）
 * @return esp_err_t 操作结果
 */
esp_err_t audio_system_start_network_services(void);

#ifdef __cplusplus
}
#endif