#include "mic_service.h"
#include "i2s_service.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include <math.h>

/* ===== AFE v2 ===== */
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"

#define TAG "mic_service"

/* ===== VAD helpers: 兼容不同版本的枚举命名 ===== */
static inline bool vad_is_begin(int s)
{
#if defined(VAD_SPEECH_BEGIN)
    return s == VAD_SPEECH_BEGIN;
#elif defined(AFE_VAD_SPEECH_BEGIN)
    return s == AFE_VAD_SPEECH_BEGIN;
#else
    return false;
#endif
}

static inline bool vad_is_end(int s)
{
#if defined(VAD_SPEECH_END)
    return s == VAD_SPEECH_END;
#elif defined(AFE_VAD_SPEECH_END)
    return s == AFE_VAD_SPEECH_END;
#else
    return false;
#endif
}

static inline bool vad_is_speech(int s)
{
#if defined(VAD_SPEECH)
    return s == VAD_SPEECH || vad_is_begin(s);
#elif defined(AFE_VAD_SPEECH)
    return s == AFE_VAD_SPEECH || vad_is_begin(s);
#else
    /* 许多实现里 0=静音，>0=有声；兜底策略 */
    return s > 0;
#endif
}

static inline bool vad_is_silence(int s)
{
#if defined(VAD_SILENCE)
    return s == VAD_SILENCE;
#elif defined(AFE_VAD_SILENCE)
    return s == AFE_VAD_SILENCE;
#else
    return !vad_is_speech(s) && !vad_is_begin(s); // 兜底
#endif
}

typedef enum
{
    ST_SLEEP = 0,
    ST_AWAKE
} mic_state_t;

static mic_service_cfg_t s_cfg;
static i2s_service_handle_t s_i2s = {0};

/* AFE v2 句柄 */
static esp_afe_sr_iface_t *s_afe_if = NULL; // ops
static esp_afe_sr_data_t *s_afe = NULL;     // data
static srmodel_list_t *s_models = NULL;
static afe_config_t *s_afe_cfg = NULL;

/* 任务与状态 */
static TaskHandle_t s_task = NULL;
static mic_on_audio_cb_t s_cb = NULL;
static mic_state_t s_state = ST_SLEEP;
static int64_t s_last_voice_ts = 0;

static inline void set_awake(void)
{
    s_state = ST_AWAKE;
    s_last_voice_ts = esp_timer_get_time();
}
static inline void set_sleep(void)
{
    s_state = ST_SLEEP;
    if (s_afe_if && s_afe)
        s_afe_if->reset_buffer(s_afe);
}

void mic_service_set_callback(mic_on_audio_cb_t cb) { s_cb = cb; }
bool mic_service_is_awake(void) { return s_state == ST_AWAKE; }
void mic_service_sleep(void) { set_sleep(); }

/* 默认的唤醒后回调：打印前几个 PCM 值（从 fortuna.c 移动过来） */
static void on_audio_after_wakeup(const int16_t *pcm, size_t samples)
{
    static uint32_t idx = 0;
    if ((++idx % 10) != 0)
        return; // 每 ~320ms 打印一次
    int n = samples < 10 ? samples : 10;
    printf("[PCM] %u:", samples);
    for (int i = 0; i < n; ++i)
        printf(" %d", pcm[i]);
    printf("\n");
}

/* 简化初始化：内部使用默认 mic 配置并启动服务（供 app_main 调用） */
esp_err_t mic_service_init(void)
{
    mic_service_cfg_t mic = {
        .rx_port = I2S_NUM_1,
        .gpio_bclk = GPIO_NUM_15, // MIC_SCK
        .gpio_ws = GPIO_NUM_2,    // MIC_WS (LRCK)
        .gpio_din = GPIO_NUM_39,  // MIC_SD
        .gpio_mclk = I2S_GPIO_UNUSED,

        .sample_rate = 16000,
        .data_bits = I2S_DATA_BIT_WIDTH_32BIT, // 推荐
        .slot_mode = I2S_SLOT_MODE_MONO,
        .slot_mask = I2S_STD_SLOT_RIGHT, // 如无声再试 LEFT
        .frame_ms = 20,
        .shift_bits = 14, // 32→16 右移
        .print_head = 0,

        .awake_silence_back_ms = 800,
    };

    mic_service_set_callback(on_audio_after_wakeup);
    return mic_service_start(&mic);
}

/* === AFE v2 初始化：input_format="M"（单通道，仅麦克风） === */
static esp_err_t afe_v2_init(void)
{
    /* 1) 加载模型（把 "model" 改为你实际的模型目录名） */
    s_models = esp_srmodel_init("model");
    ESP_RETURN_ON_FALSE(s_models, ESP_FAIL, TAG, "esp_srmodel_init failed (check model path)");

    /* 2) 仅麦克风通道：input_format = "M"；SR 前端，高性能模式 */
    s_afe_cfg = afe_config_init("M", s_models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    ESP_RETURN_ON_FALSE(s_afe_cfg, ESP_FAIL, TAG, "afe_config_init failed");

    /* （可选）打印全部配置，便于核对 */
    afe_config_print(s_afe_cfg);

    /* 3) 功能开关：只开 VAD + WakeNet；调试期先关 NS/AEC/SE，确认能唤醒后再按需开启 */
    s_afe_cfg->wakenet_init = true;
    s_afe_cfg->vad_init = true;
    s_afe_cfg->vad_mode = VAD_MODE_3; // 0~4，数字越大越敏感
    s_afe_cfg->se_init = false;       // 单麦一般不需要
    s_afe_cfg->ns_init = false;       // 先关，避免影响阈值
    s_afe_cfg->aec_init = false;      // 单通道 M 不启 AEC

    /* 采样配置：单通道输入（与 I2S 读取一致） */
    s_afe_cfg->pcm_config.sample_rate = s_cfg.sample_rate;
    s_afe_cfg->pcm_config.total_ch_num = 1;

    /* （可选）提高输出线性增益，若电平偏小可开到 2.0f 左右 */
    s_afe_cfg->afe_linear_gain = 2.0f; // 需要时可改为 2.0f

    /* 4) 获取 AFE 接口（ops）并创建实例（data） */
    s_afe_if = esp_afe_handle_from_config(s_afe_cfg);
    ESP_RETURN_ON_FALSE(s_afe_if, ESP_FAIL, TAG, "esp_afe_handle_from_config failed");
    s_afe = s_afe_if->create_from_config(s_afe_cfg);
    ESP_RETURN_ON_FALSE(s_afe, ESP_FAIL, TAG, "create_from_config failed");

    /* （可选）降低唤醒阈值（越小越敏感；按库实际范围设置） */
    if (s_afe_if->set_wakenet_threshold)
    {
        float thr = 0.45f;                                       // 你也可以改成 0.50f / 0.55f 等做AB对比
        int r1 = s_afe_if->set_wakenet_threshold(s_afe, 1, thr); // wakenet1
        int r2 = s_afe_if->set_wakenet_threshold(s_afe, 2, thr); // wakenet2（无则返回失败也无妨）
        ESP_LOGI(TAG, "WakeNet threshold set: wn1=%d, wn2=%d (thr=%.2f)", r1, r2, thr);
    }

    /* 5) 打印 AFE 期望的通道/块大小/采样率 */
    int fch = s_afe_if->get_feed_channel_num(s_afe);
    int fsz = s_afe_if->get_feed_chunksize(s_afe);
    int osz = s_afe_if->get_fetch_chunksize(s_afe);
    int fsr = s_afe_if->get_samp_rate(s_afe);
    ESP_LOGI(TAG, "AFE expects: feed_ch=%d, feed_chunk=%d, fetch_chunk=%d, sr=%d",
             fch, fsz, osz, fsr);

    ESP_LOGI(TAG, "AFE v2 (M) ok: VAD_mode=%d, SR=%d", (int)s_afe_cfg->vad_mode, s_cfg.sample_rate);
    return ESP_OK;
}

static void mic_task(void *arg)
{
    /* 读取 AFE 要求的喂入参数 */
    int feed_ch = s_afe_if->get_feed_channel_num(s_afe);
    int feed_chunk = s_afe_if->get_feed_chunksize(s_afe);
    int fetch_chunk = s_afe_if->get_fetch_chunksize(s_afe);
    if (feed_ch <= 0)
        feed_ch = 1;
    if (feed_chunk <= 0)
        feed_chunk = (s_cfg.sample_rate * (s_cfg.frame_ms > 0 ? s_cfg.frame_ms : 20)) / 1000;
    if (fetch_chunk <= 0)
        fetch_chunk = feed_chunk;

    /* 单通道 M 的输入缓冲 */
    int16_t *mic_frame = heap_caps_malloc(feed_chunk * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!mic_frame)
    {
        ESP_LOGE(TAG, "no mem for mic_frame");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "AFE feed: ch=%d, chunk=%d; fetch chunk=%d", feed_ch, feed_chunk, fetch_chunk);

    /* 电平统计辅助 */
    uint32_t frames = 0;

    while (1)
    {
        /* 1) 从 I2S 取一帧（注意：按 feed_chunk 点数读取；-1 表示阻塞直到读满） */
        if (i2s_service_read_pcm16(&s_i2s, mic_frame, feed_chunk,
                                   s_cfg.data_bits, s_cfg.shift_bits, -1) != ESP_OK)
        {
            vTaskDelay(1);
            continue;
        }

        /* 2) 每秒打印一次电平，确认是否有进音（peak/rms） */
        frames++;
        int peak = 0;
        int64_t sumsq = 0;
        for (int i = 0; i < feed_chunk; ++i)
        {
            int v = mic_frame[i];
            int av = v >= 0 ? v : -v;
            if (av > peak)
                peak = av;
            sumsq += (int32_t)v * v;
        }
        int fps = (s_cfg.sample_rate + feed_chunk / 2) / feed_chunk;
        if (fps <= 0)
            fps = 32;
        if (frames % fps == 0)
        {
            float rms = sqrtf((float)sumsq / (float)feed_chunk);
            float peak_db = 20.0f * log10f((peak > 0 ? peak : 1) / 32768.0f);
            ESP_LOGI(TAG, "MIC level: peak=%d (%.1f dBFS), rms=%.1f", peak, peak_db, rms);
        }

        /* 3) 喂入 AFE（单通道 M：直接喂 mic_frame） */
        s_afe_if->feed(s_afe, mic_frame);

        /* 4) 拉取 AFE 输出 */
        afe_fetch_result_t *res = s_afe_if->fetch(s_afe);
        if (!res)
        {
            vTaskDelay(1);
            continue;
        }

        ESP_LOGD(TAG, "AFE fetch: vad=%d, wn=%d, trig_ch=%d, out_bytes=%d",
                 res->vad_state, res->wakeup_state, res->trigger_channel_id, res->data_size);

        /* 5) 唤醒：命中唤醒词时打印“wake up success” */
        if ((res->wakeup_state == WAKENET_DETECTED) || (res->wakeup_state == WAKENET_CHANNEL_VERIFIED))
        {
            if (s_state == ST_SLEEP)
            {
                ESP_LOGI(TAG, "wake up success");
                s_state = ST_AWAKE;
                s_last_voice_ts = esp_timer_get_time();
            }
        }

        /* ===== 官方 VAD 判定（兼容多版本枚举） ===== */
        bool vad_begin = false, vad_end = false, vad_speech = false, vad_silence = false;

/* 显式 BEGIN/END（若库定义了这些枚举，则使用它们） */
#if defined(VAD_SPEECH_BEGIN)
        vad_begin = (res->vad_state == VAD_SPEECH_BEGIN);
#elif defined(AFE_VAD_SPEECH_BEGIN)
        vad_begin = (res->vad_state == AFE_VAD_SPEECH_BEGIN);
#endif

#if defined(VAD_SPEECH_END)
        vad_end = (res->vad_state == VAD_SPEECH_END);
#elif defined(AFE_VAD_SPEECH_END)
        vad_end = (res->vad_state == AFE_VAD_SPEECH_END);
#endif

/* 有声 / 静音 状态（若无显式枚举，兜底用 >0 当作“有声”） */
#if defined(VAD_SPEECH)
        vad_speech = (res->vad_state == VAD_SPEECH) || vad_begin;
#elif defined(AFE_VAD_SPEECH)
        vad_speech = (res->vad_state == AFE_VAD_SPEECH) || vad_begin;
#else
        vad_speech = (res->vad_state > 0);
#endif

#if defined(VAD_SILENCE)
        vad_silence = (res->vad_state == VAD_SILENCE);
#elif defined(AFE_VAD_SILENCE)
        vad_silence = (res->vad_state == AFE_VAD_SILENCE);
#else
        vad_silence = !vad_speech && !vad_begin; // 兜底
#endif

        /* 6) AWAKE 阶段：VAD 控制回调与休眠 */
        if (s_state == ST_AWAKE)
        {
            /* 6.1 明确的 VAD 结束：立刻“vad off”并休眠 */
            if (vad_end)
            {
                ESP_LOGI(TAG, "vad off");
                s_afe_if->reset_buffer(s_afe);
                s_state = ST_SLEEP;
                vTaskDelay(1);
                continue;
            }

            /* 6.2 有声帧：回调并刷新“最后有声时间” */
            if (vad_speech && res->data && res->data_size > 0)
            {
                size_t nsamp = res->data_size / sizeof(int16_t);
                s_last_voice_ts = esp_timer_get_time();
                if (s_cb)
                    s_cb((const int16_t *)res->data, nsamp);
            }

            /* 6.3 静音帧（或库未提供 END）：按超时回退 */
            if (vad_silence)
            {
                int64_t idle_ms = (esp_timer_get_time() - s_last_voice_ts) / 1000;
                int back_ms = s_cfg.awake_silence_back_ms > 0 ? s_cfg.awake_silence_back_ms : 800;
                if (idle_ms >= back_ms)
                {
                    ESP_LOGI(TAG, "vad off");
                    s_afe_if->reset_buffer(s_afe);
                    s_state = ST_SLEEP;
                    vTaskDelay(1);
                    continue;
                }
            }
        }

        /* 7) 待唤醒时轻睡一拍，避免忙跑 */
        if (s_state == ST_SLEEP)
            vTaskDelay(1);
    }
}

/* ===== 对外接口 ===== */

esp_err_t mic_service_start(const mic_service_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg null");
    s_cfg = *cfg;

    if (s_cfg.frame_ms <= 0)
        s_cfg.frame_ms = 20;
    if (s_cfg.sample_rate <= 0)
        s_cfg.sample_rate = 16000;
    if (s_cfg.awake_silence_back_ms <= 0)
        s_cfg.awake_silence_back_ms = 800;

    /* 1) I2S RX（建议用 32bit 容器 + shift_bits=14；slot 先试 RIGHT） */
    i2s_service_rx_cfg_t rxcfg = {
        .port = s_cfg.rx_port,
        .sample_rate = s_cfg.sample_rate,
        .data_bits = s_cfg.data_bits,
        .slot_mode = s_cfg.slot_mode,
        .slot_mask = s_cfg.slot_mask,
        .gpio_mclk = s_cfg.gpio_mclk,
        .gpio_bclk = s_cfg.gpio_bclk,
        .gpio_ws = s_cfg.gpio_ws,
        .gpio_dout = I2S_GPIO_UNUSED,
        .gpio_din = s_cfg.gpio_din,
        .invert_mclk = false,
        .invert_bclk = false,
        .invert_ws = false,
    };
    ESP_RETURN_ON_ERROR(i2s_service_rx_create(&rxcfg, &s_i2s), TAG, "i2s rx create");

    /* 2) AFE v2（单通道 M） */
    ESP_RETURN_ON_ERROR(afe_v2_init(), TAG, "afe v2 init");

    /* 3) 初始待唤醒 + 任务（core1, prio=4 更稳） */
    set_sleep();
    BaseType_t ok = xTaskCreatePinnedToCore(mic_task, "mic_task", 8 * 1024, NULL, 4, &s_task, 1);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_FAIL, TAG, "task create failed");

    ESP_LOGI(TAG, "MIC started (M): %dHz, bits=%d, shift=%d, slot=%s",
             s_cfg.sample_rate, s_cfg.data_bits, s_cfg.shift_bits,
             (s_cfg.slot_mask == I2S_STD_SLOT_RIGHT ? "RIGHT" : "LEFT"));
    return ESP_OK;
}

void mic_service_stop(void)
{
    if (s_task)
    {
        vTaskDelete(s_task);
        s_task = NULL;
    }

    if (s_afe_if && s_afe)
    {
        s_afe_if->destroy(s_afe);
        s_afe = NULL;
    }
    if (s_afe_cfg)
    {
        afe_config_free(s_afe_cfg);
        s_afe_cfg = NULL;
    }
    if (s_models)
    {
        esp_srmodel_deinit(s_models);
        s_models = NULL;
    }

    i2s_service_destroy(&s_i2s);
    s_state = ST_SLEEP;

    ESP_LOGI(TAG, "MIC stopped");
}
