#ifndef MIC_SERVICE_H
#define MIC_SERVICE_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

// 定义每帧最大字节数，确保结构体大小固定
#define MIC_FRAME_BYTES_MAX (2048)

    /**
     * @brief 音频帧结构体
     *
     * 用于在队列中传递音频数据。
     */
    typedef struct
    {
        size_t nbytes;                     /**< 当前帧的有效字节数 */
        uint8_t data[MIC_FRAME_BYTES_MAX]; /**< 音频数据缓冲区 */
    } mic_frame_t;

    /**
     * @brief 启动麦克风 WebSocket 服务
     *
     * 初始化并连接到 WebSocket 服务器，准备发送音频数据。
     *
     * @return
     *      - ESP_OK: 成功
     *      - 其他: 失败
     */
    esp_err_t mic_service_start(void);

    /**
     * @brief 停止麦克风 WebSocket 服务
     *
     * 断开连接并释放所有相关资源。
     */
    void mic_service_stop(void);

    /**
     * @brief 发送一帧 PCM16 音频数据
     *
     * 将采集到的 PCM 数据放入发送队列，由后台任务发送出去。
     * 这是一个非阻塞函数，如果队列已满，会立即返回错误。
     *
     * @param pcm       指向 PCM16 数据的指针
     * @param nsamples  样本数 (不是字节数)
     *
     * @return
     *      - ESP_OK: 成功放入队列
     *      - ESP_ERR_INVALID_STATE: 服务未运行
     *      - ESP_ERR_TIMEOUT: 队列已满，数据被丢弃
     */
    esp_err_t mic_service_send_pcm16(const int16_t *pcm, size_t nsamples);

#ifdef __cplusplus
}
#endif

#endif // MIC_SERVICE_H