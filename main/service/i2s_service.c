#include "i2s_service.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "i2s_service";

esp_err_t i2s_service_rx_create(const i2s_service_rx_cfg_t *cfg, i2s_service_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && out, ESP_ERR_INVALID_ARG, TAG, "null arg");
    *out = (i2s_service_handle_t){0};

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(cfg->port, I2S_ROLE_MASTER);
    i2s_chan_handle_t rx = NULL;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &rx), TAG, "new_channel");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(cfg->sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(cfg->data_bits, cfg->slot_mode),
        .gpio_cfg = {
            .mclk = cfg->gpio_mclk,
            .bclk = cfg->gpio_bclk,
            .ws = cfg->gpio_ws,
            .dout = cfg->gpio_dout,
            .din = cfg->gpio_din,
            .invert_flags = {
                .mclk_inv = cfg->invert_mclk,
                .bclk_inv = cfg->invert_bclk,
                .ws_inv = cfg->invert_ws,
            },
        },
    };
    std_cfg.slot_cfg.slot_mask = cfg->slot_mask;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx, &std_cfg), TAG, "std init");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx), TAG, "rx enable");

    out->rx = rx;
    ESP_LOGI(TAG, "RX ready: port=%d %dHz bits=%d mode=%d", cfg->port, cfg->sample_rate, cfg->data_bits, cfg->slot_mode);
    ESP_LOGI(TAG, "RX cfg: sr=%d, bits=%d, slot=%s, bclk=%d, ws=%d, din=%d",
             cfg->sample_rate, cfg->data_bits,
             (cfg->slot_mask == I2S_STD_SLOT_RIGHT ? "RIGHT" : "LEFT"),
             cfg->gpio_bclk, cfg->gpio_ws, cfg->gpio_din);
    return ESP_OK;
}

void i2s_service_destroy(i2s_service_handle_t *h)
{
    if (!h)
        return;
    if (h->rx)
    {
        i2s_channel_disable(h->rx);
        i2s_del_channel(h->rx);
        h->rx = NULL;
    }
}

esp_err_t i2s_service_read_bytes(i2s_service_handle_t *h, void *buf, size_t bytes,
                                 size_t *out_bytes, int timeout_ms)
{
    if (!h || !h->rx || !buf || bytes == 0)
        return ESP_ERR_INVALID_ARG;

    uint8_t *p = (uint8_t *)buf;
    size_t got_total = 0;
    TickType_t deadline = 0;

    if (timeout_ms >= 0)
    {
        deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    }

    while (got_total < bytes)
    {
        size_t got = 0;
        TickType_t to_ticks;
        if (timeout_ms < 0)
        {
            to_ticks = portMAX_DELAY; // 无限等待直到读满
        }
        else
        {
            TickType_t now = xTaskGetTickCount();
            if ((int32_t)(deadline - now) <= 0)
            {
                if (out_bytes)
                    *out_bytes = got_total;
                return ESP_ERR_TIMEOUT;
            }
            to_ticks = deadline - now;
        }

        esp_err_t err = i2s_channel_read(h->rx, p + got_total, bytes - got_total, &got, to_ticks);
        if (err == ESP_OK)
        {
            got_total += got;
        }
        else if (err == ESP_ERR_TIMEOUT)
        {
            if (out_bytes)
                *out_bytes = got_total;
            return err;
        }
        else
        {
            if (out_bytes)
                *out_bytes = got_total;
            return err;
        }
    }

    if (out_bytes)
        *out_bytes = got_total;
    return ESP_OK;
}

esp_err_t i2s_service_read_pcm16(i2s_service_handle_t *h,
                                 int16_t *dst_pcm16, size_t samples,
                                 i2s_data_bit_width_t src_bits, int shift_bits,
                                 int timeout_ms)
{
    if (!h || !h->rx || !dst_pcm16 || samples == 0)
        return ESP_ERR_INVALID_ARG;

    if (src_bits == I2S_DATA_BIT_WIDTH_16BIT)
    {
        size_t need = samples * sizeof(int16_t), got = 0;
        esp_err_t e = i2s_service_read_bytes(h, dst_pcm16, need, &got, timeout_ms);
        return (e == ESP_OK && got == need) ? ESP_OK : e;
    }
    else
    {
        size_t need = samples * sizeof(int32_t), got = 0;
        int32_t *tmp = heap_caps_malloc(need, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!tmp)
            return ESP_ERR_NO_MEM;

        esp_err_t e = i2s_service_read_bytes(h, tmp, need, &got, timeout_ms);
        if (e == ESP_OK && got == need)
        {
            for (size_t i = 0; i < samples; ++i)
                dst_pcm16[i] = (int16_t)(tmp[i] >> shift_bits);
        }
        heap_caps_free(tmp);
        return (e == ESP_OK && got == need) ? ESP_OK : e;
    }
}
