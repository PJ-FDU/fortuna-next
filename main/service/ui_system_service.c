#include "ui_system_service.h"
#include "lvgl_service.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_check.h"
#include "lvgl_screens/home.h"
#include "ui/voice_overlay.h"
#include "esp_log.h"

static const char *TAG = "ui_system";

// 背景样式
static lv_style_t style_scr_bg;

static void style_scr_bg_init(void)
{
    lv_style_init(&style_scr_bg);
    lv_style_set_bg_opa(&style_scr_bg, LV_OPA_COVER);
    /* 竖向线性渐变（上浅下深） */
    lv_style_set_bg_color(&style_scr_bg, lv_color_hex(0x5A6FFF));
    lv_style_set_bg_grad_color(&style_scr_bg, lv_color_hex(0x000000));
    lv_style_set_bg_grad_dir(&style_scr_bg, LV_GRAD_DIR_VER);
}

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
    style_scr_bg_init();
    lv_obj_add_style(lv_screen_active(), &style_scr_bg, 0);

    // 星盘主体（412×412，自绘）
    astro_create(lv_screen_active());

    // 初始化语音识别遮罩
    voice_overlay_init();

    lvgl_port_unlock();

    ESP_LOGI(TAG, "UI system initialized successfully");
    return ESP_OK;
}