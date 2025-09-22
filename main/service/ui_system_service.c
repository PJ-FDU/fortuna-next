#include "ui_system_service.h"
#include "lvgl_service.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_check.h"
#include "lvgl_screens/home.h"
// #include "ui/voice_overlay.h"
#include "esp_log.h"

#define SCREEN_SIZE 412
#define RADIUS (SCREEN_SIZE / 2)

static const char *TAG = "ui_system";
LV_FONT_DECLARE(lv_font_montserrat_96);


/**
 * @brief 创建矩形区域并设置渐变背景
 */
void create_gradient_rect(void)
{

    // 获取当前活跃的屏幕
    lv_obj_t *scr = lv_scr_act();

    // 创建一个容器作为矩形区域
    lv_obj_t *rect_obj = lv_obj_create(scr);

    // 移除默认样式（在设置尺寸前清除可能影响尺寸与布局的样式）
    // lv_obj_remove_style_all(rect_obj);

    lv_obj_set_size(rect_obj, SCREEN_SIZE, SCREEN_SIZE);
    /* Prefer explicit align which is more robust across LVGL versions/configs */
    lv_obj_align(rect_obj, LV_ALIGN_CENTER, 0, 0);

    /* Diagnostic logs: parent and object sizes/positions to help debug centering */
    ESP_LOGI(TAG, "scr size: %d x %d", lv_obj_get_width(scr), lv_obj_get_height(scr));
    ESP_LOGI(TAG, "rect_obj size (after remove_style & set_size): %d x %d", lv_obj_get_width(rect_obj), lv_obj_get_height(rect_obj));
    ESP_LOGI(TAG, "rect_obj pos after align (after remove_style & set_size): (%d, %d)", lv_obj_get_x(rect_obj), lv_obj_get_y(rect_obj));

    // 设置矩形区域的基本样式
    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_border_width(&style, 0);                  // 无边框
    lv_style_set_bg_color(&style, lv_color_hex(0x008793)); // 背景色（备用）
    lv_style_set_bg_opa(&style, LV_OPA_COVER);             // 透明背景（使用自定义绘制）
    lv_style_set_pad_all(&style, 0);                       // 无内边距

    lv_obj_add_style(rect_obj, &style, 0);

    /* 在 rect_obj 内创建中心时间标签 "12:34"，使用已编译的 Montserrat-48 字体 */
    lv_obj_t *time_label = lv_label_create(rect_obj);
    /* 使用已启用并编译进镜像的字体（montserrat_48） */
    lv_obj_set_style_text_font(time_label, &lv_font_montserrat_96, 0);
    lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF), 0);
    /* LV_OPA_20 太透明，改为覆盖不透明以保证可见 */
    lv_obj_set_style_text_opa(time_label, LV_OPA_COVER, 0);
    lv_label_set_text(time_label, "12:34");
    /* 居中时间标签到 rect_obj 中心 */
    lv_obj_align(time_label, LV_ALIGN_TOP_MID, 0, 110);

    // 强制重绘
    lv_obj_invalidate(rect_obj);
}

// 背景样式
// static lv_style_t style_scr_bg;

// static void style_scr_bg_init(void)
// {
//     lv_style_init(&style_scr_bg);
//     lv_style_set_bg_opa(&style_scr_bg, LV_OPA_COVER);
//     /* 竖向线性渐变（上浅下深） */
//     lv_style_set_bg_color(&style_scr_bg, lv_color_hex(0x5A6FFF));
//     lv_style_set_bg_grad_color(&style_scr_bg, lv_color_hex(0x000000));
//     lv_style_set_bg_grad_dir(&style_scr_bg, LV_GRAD_DIR_VER);
// }

esp_err_t ui_system_init(esp_lcd_panel_handle_t panel,
                         esp_lcd_panel_io_handle_t panel_io,
                         esp_lcd_touch_handle_t touch)
{
    ESP_LOGI(TAG, "Initializing UI system...");

    // 初始化LVGL服务
    ESP_RETURN_ON_ERROR(lvgl_service_init(panel, panel_io, touch), TAG, "LVGL service init failed");

    // 创建UI界面
    lvgl_port_lock(portMAX_DELAY);

    // 屏幕渐变背景样式
    // style_scr_bg_init();
    // lv_obj_add_style(lv_screen_active(), &style_scr_bg, 0);

    // 星盘主体（412×412，自绘）
    // astro_create(lv_screen_active());

    // 初始化语音识别遮罩
    // voice_overlay_init();

    create_gradient_rect();

    lvgl_port_unlock();

    ESP_LOGI(TAG, "UI system initialized successfully");
    return ESP_OK;
}