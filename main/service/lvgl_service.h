#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "lvgl.h"

    /** 初始化 LVGL（基于 esp_lvgl_port），并把 LCD 与触摸接入 */
    esp_err_t lvgl_service_init(esp_lcd_panel_handle_t panel,
                                esp_lcd_panel_io_handle_t io,
                                esp_lcd_touch_handle_t touch);

    /** 取 LVGL 的显示与触摸句柄（可用于自定义 UI/输入） */
    lv_display_t *lvgl_service_get_display(void);
    lv_indev_t *lvgl_service_get_touch_indev(void);

#ifdef __cplusplus
}
#endif
