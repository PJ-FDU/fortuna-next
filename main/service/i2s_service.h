#pragma once
#include "esp_err.h"
#include "driver/i2s_std.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        i2s_port_t port; // I2S_NUM_0 / I2S_NUM_1
        int gpio_bclk;   // BCLK
        int gpio_ws;     // LRCK/WS
        int gpio_din;    // DIN
        int gpio_mclk;   // 如无可用 I2S_GPIO_UNUSED
        int gpio_dout;   // 采集用不到，I2S_GPIO_UNUSED

        uint32_t sample_rate;           // 例如 16000
        i2s_data_bit_width_t data_bits; // 容器位宽，常见 I2S_DATA_BIT_WIDTH_32BIT
        i2s_slot_mode_t slot_mode;      // I2S_SLOT_MODE_MONO
        i2s_std_slot_mask_t slot_mask;  // I2S_STD_SLOT_RIGHT 或 I2S_STD_SLOT_LEFT

        uint32_t frame_ms; // 每帧时长，默认 20ms
        int shift_bits;    // 把32位容器右移到int16，有些器件用14/16等
        int print_head;    // 每帧打印前多少个样本，0表示不打印
    } i2s_service_cfg_t;

    /** 初始化并开启 I²S 采集 */
    esp_err_t i2s_service_start(const i2s_service_cfg_t *cfg);

    /** 停止采集并释放资源 */
    void i2s_service_stop(void);

    /**
     * @brief VAD状态变化回调函数类型
     */
    typedef void (*vad_state_callback_t)(bool vad_active);

    /**
     * @brief 设置VAD状态变化回调函数
     */
    void i2s_service_set_vad_callback(vad_state_callback_t callback);

    /**
     * @brief 启用或禁用VAD功能
     */
    void i2s_service_enable_vad(bool enable);

    /**
     * @brief 获取当前VAD状态
     */
    bool i2s_service_is_vad_active(void);

#ifdef __cplusplus
}
#endif
