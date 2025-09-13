#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "i2c_service.h"
#include "io_expander_service.h"
#include "lcd_service.h"
#include "lcd_touch_service.h"
#include "lvgl_service.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

static const char *TAG = "FORTUNA";

static lv_obj_t *s_status_label = NULL;
static lv_obj_t *s_touch_dot = NULL;

/* 触摸事件：按下/移动时更新坐标与小圆点位置 */
static void touch_layer_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING)
        return;

    lv_indev_t *indev = lv_event_get_indev(e);
    if (!indev)
        return;

    lv_point_t p;
    lv_indev_get_point(indev, &p);

    /* 更新状态文本 */
    char buf[64];
    lv_snprintf(buf, sizeof(buf), "Touch: (%d, %d)", (int)p.x, (int)p.y);
    lv_label_set_text(s_status_label, buf);

    /* 移动小圆点到触摸位置（让圆心对齐手指） */
    if (s_touch_dot)
    {
        int32_t x = p.x - (lv_obj_get_width(s_touch_dot) / 2);
        int32_t y = p.y - (lv_obj_get_height(s_touch_dot) / 2);
        lv_obj_set_pos(s_touch_dot, x, y);
        lv_obj_clear_flag(s_touch_dot, LV_OBJ_FLAG_HIDDEN);
    }
}

/* 渐变动画执行回调 */
static void bg_gradient_anim_cb(void *var, int32_t v)
{
    lv_obj_t *obj = (lv_obj_t *)var;

    /* 动态调整渐变的起始和结束颜色，创建动画效果 */
    lv_color_t start_color, end_color;

    if (v < 128)
    {
        /* 前半段：从原渐变到反向渐变 */
        uint8_t progress = (v * 255) / 128;
        start_color = lv_color_mix(lv_color_hex(0x3A1C71), lv_color_hex(0xFDBB2D), progress);
        end_color = lv_color_mix(lv_color_hex(0xFDBB2D), lv_color_hex(0x3A1C71), progress);
    }
    else
    {
        /* 后半段：回到原始渐变 */
        uint8_t progress = ((v - 128) * 255) / 127;
        start_color = lv_color_mix(lv_color_hex(0xFDBB2D), lv_color_hex(0x3A1C71), progress);
        end_color = lv_color_mix(lv_color_hex(0x3A1C71), lv_color_hex(0xFDBB2D), progress);
    }

    /* 设置渐变 */
    lv_obj_set_style_bg_grad_dir(obj, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_bg_grad_color(obj, start_color, 0);
    lv_obj_set_style_bg_color(obj, end_color, 0);
}

/* LVGL 定时器回调：创建渐变动画效果 */
static void ui_blink_timer_cb(lv_timer_t *timer)
{
    static bool toggle = false;
    static lv_anim_t bg_anim;

    lv_obj_t *scr = lv_screen_active();

    /* 停止之前的动画 */
    lv_anim_del(&bg_anim, NULL);

    /* 创建渐变动画 */
    lv_anim_init(&bg_anim);
    lv_anim_set_var(&bg_anim, scr);
    lv_anim_set_exec_cb(&bg_anim, bg_gradient_anim_cb);
    lv_anim_set_duration(&bg_anim, 1000); // 1000ms 平滑过渡
    lv_anim_set_path_cb(&bg_anim, lv_anim_path_ease_in_out);

    if (toggle)
    {
        lv_anim_set_values(&bg_anim, 0, 255); // 正向渐变
    }
    else
    {
        lv_anim_set_values(&bg_anim, 255, 0); // 反向渐变
    }

    lv_anim_start(&bg_anim);
    toggle = !toggle;
}

void app_main(void)
{
    ESP_LOGI(TAG, "System initialization started");

    /* 检查 PSRAM 状态 */
    // size_t psram_size = esp_psram_get_size();
    // if (psram_size > 0) {
    //     ESP_LOGI(TAG, "PSRAM detected: %d MB", (int)(psram_size / 1024 / 1024));
    // } else {
    //     ESP_LOGW(TAG, "No PSRAM detected - may have memory issues with XRGB8888");
    // }

    /* 1) I2C 总线 & IO 扩展（TCA9554） */
    ESP_ERROR_CHECK(esp_i2c_service_init());
    ESP_ERROR_CHECK(esp_io_expander_service_init());

    /* 2) LCD（QSPI + SPD2010 面板） */
    ESP_ERROR_CHECK(lcd_service_init());
    ESP_LOGI(TAG, "LCD ready");

    /* 3) 触摸（SPD2010 I2C，通过 lcd_touch_service 内部的中断/轮询读取） */
    // ESP_ERROR_CHECK(lcd_touch_service_init());
    /* 打一发状态/版本，确认控制器进入 point mode */
    // lcd_touch_service_debug_once();

    /* 4) LVGL 服务（esp_lvgl_port 管理 flush 和触摸 indev） */
    ESP_ERROR_CHECK(lvgl_service_init(
        lcd_service_get_panel(),
        lcd_service_get_panel_io(),
        NULL));

    /* 5) 构建一个极简 UI：标题 + 状态标签 + 全屏透明触摸层 + 跟随小圆点 */
    if (lvgl_port_lock(500))
    {
        lv_obj_t *scr = lv_screen_active();

        /* 设置初始渐变背景：linear-gradient(90deg, #FDBB2D 0%, #3A1C71 100%) */
        lv_obj_set_style_bg_grad_dir(scr, LV_GRAD_DIR_HOR, 0);          // 水平渐变
        lv_obj_set_style_bg_color(scr, lv_color_hex(0xFDBB2D), 0);      // 起始颜色：金黄色
        lv_obj_set_style_bg_grad_color(scr, lv_color_hex(0x3A1C71), 0); // 结束颜色：深紫色

        lv_obj_t *title = lv_label_create(scr);
        lv_label_set_text(title, "SPD2010 - Display & Touch Self Test");
        lv_obj_set_style_text_color(title, lv_color_white(), 0);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

        s_status_label = lv_label_create(scr);
        lv_label_set_text(s_status_label, "Touch: (---, ---)");
        lv_obj_set_style_text_color(s_status_label, lv_color_white(), 0);
        lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 32);

        /* 小圆点（默认隐藏，按下时显示/跟随） */
        s_touch_dot = lv_obj_create(scr);
        lv_obj_set_size(s_touch_dot, 14, 14);
        lv_obj_set_style_radius(s_touch_dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(s_touch_dot, lv_color_hex(0xFF4081), 0);
        lv_obj_add_flag(s_touch_dot, LV_OBJ_FLAG_HIDDEN);
        /* 无边框/阴影，纯色 */
        lv_obj_set_style_border_width(s_touch_dot, 0, 0);
        lv_obj_set_style_shadow_width(s_touch_dot, 0, 0);

        /* 全屏透明触摸层：接收触摸事件 */
        lv_obj_t *touch_layer = lv_obj_create(scr);
        lv_obj_remove_style_all(touch_layer); // 完全透明
        lv_obj_set_size(touch_layer, LV_PCT(100), LV_PCT(100));
        lv_obj_add_flag(touch_layer, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(touch_layer, touch_layer_event_cb, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(touch_layer, touch_layer_event_cb, LV_EVENT_PRESSING, NULL);

        lvgl_port_unlock();
    }

    /* 6) 创建 LVGL 定时器来处理背景闪烁，避免任务冲突 */
    lv_timer_t *blink_timer = lv_timer_create(ui_blink_timer_cb, 2000, NULL); // 2秒一次，更温和
    lv_timer_set_repeat_count(blink_timer, -1);                               // 无限重复

    ESP_LOGI(TAG, "Self-test UI is running. Tap/drag to see coordinates.");
}
