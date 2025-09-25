#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* 唤醒后音频帧回调（PCM16 单声道） */
    typedef void (*mic_on_audio_cb_t)(const int16_t *pcm, size_t samples);

    typedef struct
    {
        // I2S（麦克风）引脚：默认 WS=GPIO2, BCLK=GPIO15, DIN=GPIO39
        i2s_port_t rx_port;
        gpio_num_t gpio_bclk;
        gpio_num_t gpio_ws;
        gpio_num_t gpio_din;
        gpio_num_t gpio_mclk;

        int sample_rate;                // 16000
        i2s_data_bit_width_t data_bits; // 推荐 32（容器），或 16（已确认时）
        i2s_slot_mode_t slot_mode;      // I2S_SLOT_MODE_MONO
        i2s_std_slot_mask_t slot_mask;  // I2S_STD_SLOT_RIGHT（或 LEFT）
        int frame_ms;                   // 仅用于日志/回退节拍，真正以 AFE chunk 为准
        int shift_bits;                 // 32→16 的右移量（常用 14/16）
        int print_head;                 // 调试帧头打印次数

        // AFE 行为
        int awake_silence_back_ms; // 唤醒后静音多久回退（默认 800ms）
    } mic_service_cfg_t;

    esp_err_t mic_service_start(const mic_service_cfg_t *cfg);
    void mic_service_stop(void);

    void mic_service_set_callback(mic_on_audio_cb_t cb);
    bool mic_service_is_awake(void);
    void mic_service_sleep(void);

#ifdef __cplusplus
}
#endif
