#include "i2s_service.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"

#include "mic_service.h"

static const char *TAG = "i2s_service";

static i2s_service_cfg_t s_cfg;
static i2s_chan_handle_t s_rx = NULL;
static TaskHandle_t s_task = NULL;
static volatile bool s_running = false;

static inline int16_t conv_s32_to_s16(int32_t x, int shift_bits)
{
    return (int16_t)(x >> shift_bits);
}

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

    ESP_LOGI(TAG, "Capture frame: %lu ms, %lu samples", (unsigned long)s_cfg.frame_ms, (unsigned long)samples_per_frame);

    while (s_running)
    {
        size_t nread = 0;
        esp_err_t err = i2s_channel_read(s_rx, buf32, bytes_per_frame_32, &nread, portMAX_DELAY);
        if (err != ESP_OK || nread != bytes_per_frame_32)
        {
            ESP_LOGW(TAG, "read err=%d nread=%u", err, (unsigned)nread);
            continue;
        }

        // 转换为 int16 并计算 RMS（电平值）
        double acc = 0.0;
        int16_t min_val = INT16_MAX, max_val = INT16_MIN; // 正确的初始值

        for (uint32_t i = 0; i < samples_per_frame; ++i)
        {
            int16_t s = conv_s32_to_s16(buf32[i], s_cfg.shift_bits); // 32-bit 转 16-bit
            buf16[i] = s;

            // 更新最大值和最小值
            if (s < min_val)
                min_val = s;
            if (s > max_val)
                max_val = s;

            // 计算 RMS
            acc += (double)s * (double)s;
        }

        double rms = sqrt(acc / samples_per_frame);
        double dbfs = (rms > 0.5) ? 20.0 * log10(rms / 32768.0) : -120.0;

        // 打印每一帧的前16个样本和电平值
        ESP_LOGI(TAG, "Min sample: %d, Max sample: %d", min_val, max_val);
        ESP_LOGI(TAG, "RMS=%.1f dBFS=%.1f", rms, dbfs);

        // 打印 PCM16 数据
        if (s_cfg.print_head > 0)
        {
            printf("[PCM16] ");
            uint32_t head = (s_cfg.print_head < (int)samples_per_frame) ? s_cfg.print_head : samples_per_frame;
            for (uint32_t i = 0; i < head; ++i)
                printf("%d ", (int)buf16[i]);
            printf("| RMS=%.1f dBFS=%.1f\n", rms, dbfs);
        }
        else
        {
            printf("RMS=%.1f dBFS=%.1f\n", rms, dbfs);
        }

        // 发送音频数据
        mic_service_send_pcm16(buf16, samples_per_frame); // 发送 PCM16 数据
    }

    free(buf32);
    free(buf16);
    vTaskDelete(NULL);
}

esp_err_t i2s_service_start(const i2s_service_cfg_t *cfg)
{
    if (s_running)
        return ESP_OK;
    // 默认参数（给没填的字段一个合理默认）
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
        .print_head = 16,
    };
    s_cfg = def;
    if (cfg)
    {
        // 覆盖用户传入的配置（简单逐项赋值）
        s_cfg = *cfg;
    }

    ESP_RETURN_ON_ERROR(i2s_init_std_rx(&s_cfg), TAG, "i2s init");
    s_running = true;
    xTaskCreatePinnedToCore(task_i2s_capture, "i2s_cap", 3 * 1024, NULL, 5, &s_task, 0);
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
}