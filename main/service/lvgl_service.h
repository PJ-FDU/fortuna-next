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

    /**
     * @brief 初始化 LVGL + 显示 + 触摸
     * @param panel  来自 lcd_service 的 esp_lcd_panel_handle_t
     * @param io     来自 lcd_service 的 esp_lcd_panel_io_handle_t
     * @param touch  来自 lcd_touch_service 的 esp_lcd_touch_handle_t（可为 NULL）
     */
    esp_err_t lvgl_service_init(esp_lcd_panel_handle_t panel,
                                esp_lcd_panel_io_handle_t io,
                                esp_lcd_touch_handle_t touch);

    /** 取 LVGL 显示与触摸句柄 */
    lv_display_t *lvgl_service_get_display(void);
    lv_indev_t *lvgl_service_get_touch_indev(void);

#ifdef __cplusplus
}
#endif
