#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化UI系统（LVGL + 背景 + 星盘 + 语音遮罩）
 * @param panel LCD面板句柄
 * @param panel_io LCD面板IO句柄
 * @param touch 触摸句柄（可以为NULL）
 * @return esp_err_t 操作结果
 */
esp_err_t ui_system_init(esp_lcd_panel_handle_t panel, 
                         esp_lcd_panel_io_handle_t panel_io, 
                         esp_lcd_touch_handle_t touch);

#ifdef __cplusplus
}
#endif