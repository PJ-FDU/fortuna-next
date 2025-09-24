#include "ui_system_service.h"
#include "lvgl_service.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_check.h"
#include "lvgl_screens/home.h"
// #include "ui/voice_overlay.h"
#include "esp_log.h"
#include "string.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include <dirent.h>

#define SCREEN_SIZE 412
#define RADIUS (SCREEN_SIZE / 2)

static const char *TAG = "ui_system";
LV_FONT_DECLARE(bacchus_extrabold_96);
LV_FONT_DECLARE(bacchus_extrabold_128);

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

    /* 使用 45° 线性渐变背景（从左上到右下），使用反射扩展以实现平滑过渡）*/
    /* linear-gradient(90deg, #3F2B96 0%, #A8C0FF 100%) but mapped from left-bottom to right-top */
    static const lv_color_t grad_colors[2] = {
        LV_COLOR_MAKE(0x3F, 0x2B, 0x96), /* #3F2B96 */
        LV_COLOR_MAKE(0xA8, 0xC0, 0xFF), /* #A8C0FF */
    };

    static lv_style_t style_with_linear_gradient_bg;
    static lv_grad_dsc_t linear_gradient_dsc; /* gradient descriptor must be static */

    lv_style_init(&style_with_linear_gradient_bg);
    lv_grad_init_stops(&linear_gradient_dsc, grad_colors, NULL, NULL, sizeof(grad_colors) / sizeof(lv_color_t));
    /* 从左下 (0%,100%) 到右上 (100%,0%)，即从左下角到右上角的对角线 */
    lv_grad_linear_init(&linear_gradient_dsc, lv_pct(0), lv_pct(100), lv_pct(100), lv_pct(0), LV_GRAD_EXTEND_REFLECT);
    lv_style_set_bg_grad(&style_with_linear_gradient_bg, &linear_gradient_dsc);
    lv_style_set_bg_opa(&style_with_linear_gradient_bg, LV_OPA_COVER);
    lv_style_set_border_width(&style_with_linear_gradient_bg, 0);
    lv_style_set_pad_all(&style_with_linear_gradient_bg, 0);

    lv_obj_add_style(rect_obj, &style_with_linear_gradient_bg, 0);

    /* 在 rect_obj 内创建中心时间标签 "12:34"，使用已编译的 Montserrat-48 字体 */
    lv_obj_t *time_label = lv_label_create(rect_obj);
    /* 使用已启用并编译进镜像的字体（montserrat_48） */
    lv_obj_set_style_text_font(time_label, &bacchus_extrabold_96, 0);
    lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF), 0);
    /* LV_OPA_20 太透明，改为覆盖不透明以保证可见 */
    lv_obj_set_style_text_opa(time_label, LV_OPA_50, 0);
    lv_label_set_text(time_label, "09:49");
    /* 居中时间标签到 rect_obj 中心 */
    lv_obj_align(time_label, LV_ALIGN_TOP_MID, 0, 110);

    // 强制重绘
    lv_obj_invalidate(rect_obj);
}

/**
 * @brief 确保 SPIFFS 已挂载，如果未挂载则注册并挂载到 /spiffs
 */
static esp_err_t ensure_spiffs_mounted(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "spiffs",
        .max_files = 5,
        .format_if_mount_failed = false,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret == ESP_ERR_INVALID_STATE)
    {
        /* Already mounted */
        return ESP_OK;
    }
    return ret;
}

/**
 * @brief 在当前屏幕上创建一个全屏图片背景，图片来自 SPIFFS 的 /spiffs/bg.png
 * 图片尺寸恰好为 412x412，函数会创建 lv_image 并设置为填充容器中心
 */
void create_img_bg(void)
{
    /* 确保 SPIFFS 可用 */
    esp_err_t err = ensure_spiffs_mounted();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(err));
        return;
    }

    lv_image_cache_resize(0, true);
    lv_obj_t *scr = lv_scr_act();

    /* 创建 image 对象，使用绝对路径从 SPIFFS 加载 */
    lv_obj_t *img = lv_image_create(scr);
    lv_image_set_src(img, "S:/bg_sm.png");
    const int32_t w = lv_obj_get_self_width(img);
    const int32_t h = lv_obj_get_self_height(img);
    ESP_LOGI(TAG, "Background image size: %d x %d", w, h);
    lv_obj_set_size(img, SCREEN_SIZE, SCREEN_SIZE);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
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

    /* 创建并显示背景图片（来自 SPIFFS /spiffs/bg.png） */
    // create_img_bg();

    create_gradient_rect();

    lvgl_port_unlock();

    ESP_LOGI(TAG, "UI system initialized successfully");
    return ESP_OK;
}