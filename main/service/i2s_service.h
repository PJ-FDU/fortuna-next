#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        i2s_port_t port;
        int sample_rate;
        i2s_data_bit_width_t data_bits; // 推荐 32bit 容器
        i2s_slot_mode_t slot_mode;      // I2S_SLOT_MODE_MONO
        i2s_std_slot_mask_t slot_mask;  // I2S_STD_SLOT_RIGHT 或 LEFT
        gpio_num_t gpio_mclk;
        gpio_num_t gpio_bclk;
        gpio_num_t gpio_ws;
        gpio_num_t gpio_dout;
        gpio_num_t gpio_din;
        bool invert_mclk;
        bool invert_bclk;
        bool invert_ws;
    } i2s_service_rx_cfg_t;

    typedef struct
    {
        i2s_chan_handle_t rx;
    } i2s_service_handle_t;

    esp_err_t i2s_service_rx_create(const i2s_service_rx_cfg_t *cfg, i2s_service_handle_t *out);
    void i2s_service_destroy(i2s_service_handle_t *h);

    /* 阻塞读取 bytes（timeout_ms<0 表示永久阻塞） */
    esp_err_t i2s_service_read_bytes(i2s_service_handle_t *h, void *buf, size_t bytes,
                                     size_t *out_bytes, int timeout_ms);

    /* 读取 samples 个样本并转成 PCM16（src_bits=16 直拷贝；=32 右移 shift_bits） */
    esp_err_t i2s_service_read_pcm16(i2s_service_handle_t *h,
                                     int16_t *dst_pcm16, size_t samples,
                                     i2s_data_bit_width_t src_bits, int shift_bits,
                                     int timeout_ms);

#ifdef __cplusplus
}
#endif
