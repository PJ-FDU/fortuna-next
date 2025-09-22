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

/* ====== 可配置宏（默认值，可按需修改或移入 Kconfig） ====== */
/* LVGL 运行任务参数 */
#define LVGL_TASK_PRIORITY (5)
#define LVGL_TASK_STACK_BYTES (8 * 1024)
#define LVGL_TASK_AFFINITY (-1)
#define LVGL_TIMER_PERIOD_MS (2)

/* 屏幕条带缓冲相关：以行数表示的缓冲高度（用于计算 buffer_size） */
#define LVGL_DISP_BUFFER_LINES (412)
#define LVGL_DOUBLE_BUFFER (1)

/* 每像素字节数：LVGL9+ 使用 RGB888（3 字节），否则常用 RGB565（2 字节） */
#define LVGL_COLOR_BYTES (3)

    /**
     * @brief 初始化 LVGL（基于 esp_lvgl_port），并把 LCD 与触摸接入
     *
     * 参数：
     *  - panel: 已创建并初始化的 `esp_lcd_panel_handle_t`（必须非 NULL）
     *  - io:    对应的面板 IO 句柄（必须非 NULL）
     *  - touch: 可选的触摸句柄（NULL 表示不启用触摸）
     *
     * 返回：ESP_OK 表示成功，其他错误码表示初始化失败。注意：此函数
     * 会在底层调用 `lvgl_port_init`，失败会返回对应错误码；调用方应
     * 检查返回值并在失败时进行处理。
     */
    esp_err_t lvgl_service_init(esp_lcd_panel_handle_t panel,
                                esp_lcd_panel_io_handle_t io,
                                esp_lcd_touch_handle_t touch);

    /**
     * @brief 获取 LVGL 的显示句柄（安全接口）
     *
     * 使用 out 参数返回句柄，并通过返回值报告状态：
     *  - ESP_OK: 成功并填充 out_display
     *  - ESP_ERR_INVALID_ARG: out_display 为 NULL
     *  - ESP_ERR_INVALID_STATE: LVGL 尚未初始化或显示未注册
     */
    esp_err_t lvgl_service_get_display(lv_display_t **out_display);

    /**
     * @brief 获取 LVGL 的触摸输入设备句柄（安全接口）
     */
    esp_err_t lvgl_service_get_touch_indev(lv_indev_t **out_indev);

#ifdef __cplusplus
}
#endif
