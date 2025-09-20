#include "i2s_service.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"

// 先尝试包含ESP-SR的VAD头文件 - 根据实际情况调整
#ifdef CONFIG_ESP32_S3_BOX
#include "esp_vad.h"
#define VAD_AVAILABLE 1
#else
// 如果VAD不可用，我们先用简单的能量检测
#define VAD_AVAILABLE 0
#endif

#include "mic_service.h"

static const char *TAG = "i2s_service";

static i2s_service_cfg_t s_cfg;
static i2s_chan_handle_t s_rx = NULL;
static TaskHandle_t s_task = NULL;
static volatile bool s_running = false;

// VAD 相关变量
#if VAD_AVAILABLE
static vad_handle_t s_vad_handle = NULL;
#endif
static bool s_vad_enabled = true;
static bool s_vad_active = false;
static int s_silence_frames = 0;
static int s_voice_frames = 0;

// VAD状态变化回调
typedef void (*vad_state_callback_t)(bool vad_active);
static vad_state_callback_t s_vad_callback = NULL;

// VAD 配置参数
#define VAD_SILENCE_THRESHOLD 15   // 连续静音帧数阈值（约300ms）
#define VAD_VOICE_THRESHOLD 3      // 连续语音帧数阈值（约60ms）
#define VAD_ENERGY_THRESHOLD 500.0 // 简单能量阈值（fallback）

static inline int16_t conv_s32_to_s16(int32_t x, int shift_bits)
{
    return (int16_t)(x >> shift_bits);
}

// VAD状态更新函数
static void update_vad_state(bool new_state)
{
    if (s_vad_active != new_state) {
        s_vad_active = new_state;
        
        if (new_state) {
            ESP_LOGI(TAG, "Voice activity detected");
        } else {
            ESP_LOGI(TAG, "Voice activity ended");
        }
        
        // 调用回调函数通知UI更新
        if (s_vad_callback) {
            s_vad_callback(new_state);
        }
    }
}

#if VAD_AVAILABLE
static esp_err_t vad_init(void)
{
    if (!s_vad_enabled)
    {
        return ESP_OK;
    }

    // 尝试使用ESP-SR VAD - 这里需要根据实际的API调整
    s_vad_handle = vad_create(VAD_MODE_0); // 或者其他可用的模式

    if (!s_vad_handle)
    {
        ESP_LOGE(TAG, "ESP-SR VAD create failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ESP-SR VAD initialized successfully");
    return ESP_OK;
}

static void vad_destroy_handle(void)
{
    if (s_vad_handle)
    {
        vad_destroy(s_vad_handle);
        s_vad_handle = NULL;
        ESP_LOGI(TAG, "ESP-SR VAD destroyed");
    }
}

static bool vad_process_frame(int16_t *audio_data, size_t samples)
{
    if (!s_vad_handle)
    {
        return false;
    }

    // 根据实际的ESP-SR API调用VAD
    vad_state_t result = vad_process(s_vad_handle, audio_data);
    return (result == VAD_SPEECH);
}

#else

// Fallback: 简单的能量检测VAD
static esp_err_t vad_init(void)
{
    ESP_LOGI(TAG, "Using simple energy-based VAD (ESP-SR not available)");
    return ESP_OK;
}

static void vad_destroy_handle(void)
{
    ESP_LOGI(TAG, "Simple VAD cleanup");
}

static bool vad_process_frame(int16_t *audio_data, size_t samples)
{
    // 计算RMS能量
    double acc = 0.0;
    for (size_t i = 0; i < samples; i++)
    {
        acc += (double)audio_data[i] * (double)audio_data[i];
    }
    double rms = sqrt(acc / samples);

    return (rms > VAD_ENERGY_THRESHOLD);
}

#endif

static esp_err_t i2s_init_std_rx(const i2s_service_cfg_t *c)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(c->port, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &s_rx), TAG, "new_channel");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(c->sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(c->data_bits, c->slot_mode),
        .gpio_cfg = {
            .mclk = c->gpio_mclk,
            .bclk = c->gpio_bclk,
            .ws = c->gpio_ws,
            .dout = c->gpio_dout,
            .din = c->gpio_din,
            .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
        },
    };
    std_cfg.slot_cfg.slot_mask = c->slot_mask;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx, &std_cfg), TAG, "init_std");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx), TAG, "enable");

    ESP_LOGI(TAG, "I2S ready: %lu Hz, %d-bit container, %s, %s slot (BCLK=%d WS=%d DIN=%d)",
             (unsigned long)c->sample_rate, (int)c->data_bits * 8,
             (c->slot_mode == I2S_SLOT_MODE_MONO ? "MONO" : "STEREO"),
             (c->slot_mask == I2S_STD_SLOT_RIGHT ? "RIGHT" : "LEFT"),
             c->gpio_bclk, c->gpio_ws, c->gpio_din);
    return ESP_OK;
}

static void task_i2s_capture(void *arg)
{
    // 动态计算帧大小
    const uint32_t samples_per_frame = (s_cfg.sample_rate * s_cfg.frame_ms) / 1000;
    const uint32_t bytes_per_frame_32 = samples_per_frame * sizeof(int32_t);

    int32_t *buf32 = heap_caps_malloc(bytes_per_frame_32, MALLOC_CAP_DEFAULT);
    int16_t *buf16 = heap_caps_malloc(samples_per_frame * sizeof(int16_t), MALLOC_CAP_DEFAULT);

    if (!buf32 || !buf16)
    {
        ESP_LOGE(TAG, "malloc failed");
        s_running = false;
        if (buf32)
            free(buf32);
        if (buf16)
            free(buf16);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Capture frame: %lu ms, %lu samples, VAD: %s",
             (unsigned long)s_cfg.frame_ms, (unsigned long)samples_per_frame,
             s_vad_enabled ? "ENABLED" : "DISABLED");

    while (s_running)
    {
        size_t nread = 0;
        esp_err_t err = i2s_channel_read(s_rx, buf32, bytes_per_frame_32, &nread, portMAX_DELAY);
        if (err != ESP_OK || nread != bytes_per_frame_32)
        {
            ESP_LOGW(TAG, "read err=%d nread=%u", err, (unsigned)nread);
            continue;
        }

        // 转换为 int16 并计算统计信息
        double acc = 0.0;
        int16_t min_val = INT16_MAX, max_val = INT16_MIN;

        for (uint32_t i = 0; i < samples_per_frame; ++i)
        {
            int16_t s = conv_s32_to_s16(buf32[i], s_cfg.shift_bits);
            buf16[i] = s;

            if (s < min_val)
                min_val = s;
            if (s > max_val)
                max_val = s;
            acc += (double)s * (double)s;
        }

        double rms = sqrt(acc / samples_per_frame);
        double dbfs = (rms > 0.5) ? 20.0 * log10(rms / 32768.0) : -120.0;

        // VAD 检测
        bool voice_detected = false;
        if (s_vad_enabled)
        {
            voice_detected = vad_process_frame(buf16, samples_per_frame);

            if (voice_detected)
            {
                s_voice_frames++;
                s_silence_frames = 0;

                if (!s_vad_active && s_voice_frames >= VAD_VOICE_THRESHOLD)
                {
                    update_vad_state(true);
                }
            }
            else
            {
                s_silence_frames++;
                s_voice_frames = 0;

                if (s_vad_active && s_silence_frames >= VAD_SILENCE_THRESHOLD)
                {
                    update_vad_state(false);
                }
            }
        }
        else
        {
            if (!s_vad_active) {
                update_vad_state(true); // 如果VAD未启用，设置为激活状态
            }
        }

        // 打印状态信息
        // if (s_cfg.print_head > 0)
        // {
        //     printfPCM("[PCM16] ");
        //     uint32_t head = (s_cfg.print_head < (int)samples_per_frame) ? s_cfg.print_head : samples_per_frame;
        //     for (uint32_t i = 0; i < head; ++i)
        //         printf("%d ", (int)buf16[i]);
        //     printf("| RMS=%.1f dBFS=%.1f VAD=%s\n", rms, dbfs, s_vad_active ? "ACTIVE" : "SILENT");
        // }
        // else
        // {
        //     printf("RMS=%.1f dBFS=%.1f VAD=%s\n", rms, dbfs, s_vad_active ? "ACTIVE" : "SILENT");
        // }

        // 只在VAD激活时发送音频数据
        if (s_vad_active)
        {
            esp_err_t ret = mic_service_send_pcm16(buf16, samples_per_frame);
            if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT && ret != ESP_ERR_INVALID_STATE)
            {
                ESP_LOGW(TAG, "Failed to send audio frame: %s", esp_err_to_name(ret));
            }
        }
    }

    free(buf32);
    free(buf16);
    vTaskDelete(NULL);
}

esp_err_t i2s_service_start(const i2s_service_cfg_t *cfg)
{
    if (s_running)
    {
        return ESP_OK;
    }

    // 默认参数
    i2s_service_cfg_t def = {
        .port = I2S_NUM_1,
        .gpio_bclk = 15,
        .gpio_ws = 2,
        .gpio_din = 39,
        .gpio_mclk = I2S_GPIO_UNUSED,
        .gpio_dout = I2S_GPIO_UNUSED,
        .sample_rate = 16000,
        .data_bits = I2S_DATA_BIT_WIDTH_32BIT,
        .slot_mode = I2S_SLOT_MODE_MONO,
        .slot_mask = I2S_STD_SLOT_RIGHT,
        .frame_ms = 20,
        .shift_bits = 14,
        .print_head = 0,
    };

    s_cfg = def;
    if (cfg)
    {
        s_cfg = *cfg;
    }

    // 验证采样率
    if (s_vad_enabled && s_cfg.sample_rate != 16000)
    {
        ESP_LOGW(TAG, "VAD works best with 16kHz sample rate, current: %lu Hz",
                 (unsigned long)s_cfg.sample_rate);
    }

    ESP_RETURN_ON_ERROR(i2s_init_std_rx(&s_cfg), TAG, "i2s init");

    // 初始化VAD
    if (s_vad_enabled)
    {
        ESP_RETURN_ON_ERROR(vad_init(), TAG, "vad init");
    }

    s_running = true;
    xTaskCreatePinnedToCore(task_i2s_capture, "i2s_cap", 4 * 1024, NULL, 5, &s_task, 0);

    return ESP_OK;
}

void i2s_service_stop(void)
{
    s_running = false;

    if (s_task)
    {
        vTaskDelete(s_task);
        s_task = NULL;
    }

    if (s_rx)
    {
        i2s_channel_disable(s_rx);
        i2s_del_channel(s_rx);
        s_rx = NULL;
    }

    // 销毁VAD
    vad_destroy_handle();

    // 重置VAD状态
    s_vad_active = false;
    s_silence_frames = 0;
    s_voice_frames = 0;
}

void i2s_service_enable_vad(bool enable)
{
    if (enable != s_vad_enabled)
    {
        s_vad_enabled = enable;
        ESP_LOGI(TAG, "VAD %s", enable ? "enabled" : "disabled");
        if (!enable)
        {
            s_vad_active = true; // 禁用VAD时，默认一直发送
        }
    }
}

bool i2s_service_is_vad_active(void)
{
    return s_vad_active;
}

void i2s_service_set_vad_callback(vad_state_callback_t callback)
{
    s_vad_callback = callback;
    ESP_LOGI(TAG, "VAD callback %s", callback ? "registered" : "cleared");
}