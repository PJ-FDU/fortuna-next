#include "voice_overlay.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "voice_overlay";

// 消息队列用于LVGL任务间通信
typedef struct {
    bool show;
} overlay_msg_t;

static QueueHandle_t overlay_queue = NULL;
static lv_timer_t *overlay_timer = NULL;

static lv_obj_t *overlay_bg = NULL;      // 背景遮罩
static lv_obj_t *overlay_label = NULL;   // 文本标签
static lv_obj_t *loading_spinner = NULL; // 加载动画

// 前向声明
static void overlay_timer_cb(lv_timer_t *timer);

void voice_overlay_init(void)
{
    if (overlay_bg)
    {
        ESP_LOGW(TAG, "Voice overlay already initialized");
        return;
    }

    // 获取当前活动屏幕 - LVGL 9.x API
    lv_obj_t *screen = lv_screen_active();

    // 创建全屏背景遮罩
    overlay_bg = lv_obj_create(screen);
    lv_obj_set_size(overlay_bg, lv_pct(100), lv_pct(100));
    lv_obj_set_pos(overlay_bg, 0, 0);
    lv_obj_set_style_bg_color(overlay_bg, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(overlay_bg, LV_OPA_70, 0); // 70%透明度
    lv_obj_set_style_border_width(overlay_bg, 0, 0);
    lv_obj_set_style_radius(overlay_bg, 0, 0);
    lv_obj_remove_flag(overlay_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(overlay_bg, LV_OBJ_FLAG_HIDDEN); // 初始隐藏

    // 创建加载动画 - 使用spinner
    loading_spinner = lv_spinner_create(overlay_bg);
    lv_obj_set_size(loading_spinner, 60, 60);
    lv_obj_center(loading_spinner);
    lv_obj_set_y(loading_spinner, lv_obj_get_y(loading_spinner) - 30); // 向上偏移
    lv_obj_set_style_arc_color(loading_spinner, lv_color_hex(0x00FF88), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(loading_spinner, 6, LV_PART_INDICATOR);

    // 创建文本标签
    overlay_label = lv_label_create(overlay_bg);
    lv_label_set_text(overlay_label, "Recognizing...");
    lv_obj_set_style_text_color(overlay_label, lv_color_white(), 0);

    // 优先使用中文字体，如果不可用则使用英文字体兜底
#if LV_FONT_SOURCE_HAN_SANS_SC_16_CJK
    lv_obj_set_style_text_font(overlay_label, &lv_font_source_han_sans_sc_16_cjk, 0);
#elif LV_FONT_MONTSERRAT_16
    lv_obj_set_style_text_font(overlay_label, &lv_font_montserrat_16, 0);
#else
    lv_obj_set_style_text_font(overlay_label, &lv_font_montserrat_14, 0);
#endif

    lv_obj_center(overlay_label);
    lv_obj_set_y(overlay_label, lv_obj_get_y(overlay_label) + 50); // 在spinner下方

    // 创建消息队列
    overlay_queue = xQueueCreate(10, sizeof(overlay_msg_t));
    if (!overlay_queue) {
        ESP_LOGE(TAG, "Failed to create overlay message queue");
        return;
    }

    // 创建定时器处理消息队列（在LVGL任务中运行，无需锁定）
    overlay_timer = lv_timer_create(overlay_timer_cb, 50, NULL);
    if (!overlay_timer) {
        ESP_LOGE(TAG, "Failed to create overlay timer");
        return;
    }

    ESP_LOGI(TAG, "Voice overlay initialized with message queue and timer");
}

// 处理队列消息的函数 - 在LVGL任务上下文中调用
static void process_overlay_messages(void)
{
    if (!overlay_queue || !overlay_bg) {
        return;
    }

    overlay_msg_t msg;
    while (xQueueReceive(overlay_queue, &msg, 0) == pdTRUE) {
        // 直接操作UI，无需锁定（已在LVGL任务中）
        if (msg.show) {
            lv_obj_remove_flag(overlay_bg, LV_OBJ_FLAG_HIDDEN);
            ESP_LOGI(TAG, "Voice overlay shown (no lock)");
        } else {
            lv_obj_add_flag(overlay_bg, LV_OBJ_FLAG_HIDDEN);
            ESP_LOGI(TAG, "Voice overlay hidden (no lock)");
        }
    }
}

static void overlay_timer_cb(lv_timer_t *timer)
{
    process_overlay_messages();
}

void voice_overlay_show(void)
{
    ESP_LOGI(TAG, "voice_overlay_show() called");
    if (!overlay_bg || !overlay_queue)
    {
        ESP_LOGE(TAG, "Voice overlay not initialized properly");
        return;
    }

    overlay_msg_t msg = { .show = true };
    
    // 发送消息到队列，无需锁定
    if (xQueueSend(overlay_queue, &msg, pdMS_TO_TICKS(10)) == pdTRUE) {
        ESP_LOGI(TAG, "Show message sent to queue");
    } else {
        ESP_LOGW(TAG, "Failed to send show message - queue full?");
    }
}

void voice_overlay_hide(void)
{
    ESP_LOGI(TAG, "voice_overlay_hide() called");
    if (!overlay_bg || !overlay_queue)
    {
        ESP_LOGE(TAG, "Voice overlay not initialized properly");
        return;
    }

    overlay_msg_t msg = { .show = false };
    
    // 发送消息到队列，无需锁定
    if (xQueueSend(overlay_queue, &msg, pdMS_TO_TICKS(10)) == pdTRUE) {
        ESP_LOGI(TAG, "Hide message sent to queue");
    } else {
        ESP_LOGW(TAG, "Failed to send hide message - queue full?");
    }
}

void voice_overlay_set_text(const char *text)
{
    if (!overlay_label)
    {
        ESP_LOGE(TAG, "Voice overlay not initialized");
        return;
    }

    lv_label_set_text(overlay_label, text);
    lv_obj_center(overlay_label);
    lv_obj_set_y(overlay_label, lv_obj_get_y(overlay_label) + 50);
}