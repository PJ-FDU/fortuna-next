#include "lvgl_service.h"
#include "lcd_service.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_err.h"
#include "esp_check.h"

/* Module tag for logging */
static const char *TAG = "lvgl_service";

/*
 * Internal module-owned handles
 * - s_disp: LVGL display object returned by the esp_lvgl_port layer.
 * - s_indev: LVGL input device for touch (optional, NULL if no touch registered).
 *
 * Ownership: these handles are owned by the lvgl_service module. Callers must not
 * free them. Use the safe getters below to obtain the handles.
 */
static lv_display_t *s_disp = NULL;
static lv_indev_t *s_indev = NULL;

/* SPD2010 要求：物理 X 方向以 4 像素对齐
 *  LVGL9 不再有 rounder_cb，改为在 LV_EVENT_INVALIDATE_AREA 里调整刷新区域
 *  注意：我们**没有**做 panel 的 swap_xy（SPD2010 不支持），所以这里始终对齐 X。
 */
static void spd2010_rounder_event_cb(lv_event_t *e)
{
    lv_area_t *area = (lv_area_t *)lv_event_get_param(e);
    if (!area)
        return;

    /* x1 向下对齐到 4N，x2 向上对齐到 4N+3 */
    area->x1 = (area->x1 & ~0x3);
    area->x2 = (area->x2 | 0x3);

    if (area->x1 < 0)
        area->x1 = 0;
    if (area->x2 >= LCD_H_RES)
        area->x2 = LCD_H_RES - 1;
}

/**
 * @brief Initialize LVGL + esp_lvgl_port and register the display/touch
 *
 * Notes on error behavior:
 * - This function uses ESP_ERROR_CHECK for critical initialization steps. If any
 *   of those calls fail, the macro will assert/abort (depending on IDF
 *   configuration). That means initialization failures are treated as fatal
 *   conditions here. If you prefer non-fatal behavior (returning the error to
 *   the caller), replace those ESP_ERROR_CHECK calls with explicit checks and
 *   return appropriate esp_err_t values.
 *
 * Parameters:
 *  - panel: already-created & configured esp_lcd_panel_handle_t (must be non-NULL)
 *  - io: associated esp_lcd_panel_io_handle_t for the panel (must be non-NULL)
 *  - touch: optional touch handle (NULL to disable touch)
 *
 * Returns:
 *  - ESP_OK on success (LVGL initialized and display registered)
 *  - If an error occurs and ESP_ERROR_CHECK is not configured to abort, a
 *    non-ESP_OK value may be returned; otherwise the system will assert/abort.
 */
esp_err_t lvgl_service_init(esp_lcd_panel_handle_t panel,
                            esp_lcd_panel_io_handle_t io,
                            esp_lcd_touch_handle_t touch)
{
    /* 参数校验（使用断言式检查：失败将触发 ESP_ERROR_CHECK 的行为） */
    ESP_ERROR_CHECK(panel != NULL ? ESP_OK : ESP_ERR_INVALID_ARG);
    ESP_ERROR_CHECK(io != NULL ? ESP_OK : ESP_ERR_INVALID_ARG);

    /* 1) 初始化 LVGL 端口（创建 LVGL 任务与 tick 定时器） */
    const lvgl_port_cfg_t port_cfg = {
        .task_priority = LVGL_TASK_PRIORITY,
        .task_stack = LVGL_TASK_STACK_BYTES,
        .task_affinity = LVGL_TASK_AFFINITY,
        .task_max_sleep_ms = 500,
        .timer_period_ms = LVGL_TIMER_PERIOD_MS,
    };
    /* lvgl_port_init 为关键步骤，失败时按策略使用 ESP_ERROR_CHECK 处理 */
    ESP_ERROR_CHECK(lvgl_port_init(&port_cfg));

    /* 2) 注册显示（使用条带缓冲；XRGB8888 支持透明通道，转换到RGB888面板；不做 SW 旋转） */
    /* 2) 注册显示（使用条带缓冲；buffer_size 根据宏计算） */
    size_t buffer_size = (size_t)LCD_H_RES * LVGL_DISP_BUFFER_LINES * LVGL_COLOR_BYTES;
    lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io,
        .panel_handle = panel,
        .control_handle = NULL,

        /* buffer_size 由屏宽与若干行高度决定，节省内存同时保证刷新吞吐 */
        .buffer_size = buffer_size,
        .double_buffer = LVGL_DOUBLE_BUFFER,
        .trans_size = 0,

        .hres = LCD_H_RES,
        .vres = LCD_V_RES,

        .monochrome = false,

        .rotation = {.swap_xy = false, .mirror_x = false, .mirror_y = false},

        .color_format = LV_COLOR_FORMAT_RGB888,
        .flags = {
            .buff_dma = 0,
            .buff_spiram = 1,
            .sw_rotate = 0,
            .swap_bytes = 0,
            .full_refresh = 0,
            .direct_mode = 0,
        },
    };

    /*
     * 2) 注册显示。
     * buffer_size 依据宏 LVGL_DISP_BUFFER_LINES 与 LVGL_COLOR_BYTES 计算，
     * 以获得合适的条带缓冲大小。该计算旨在在内存与刷新吞吐之间取得折中。
     */
    s_disp = lvgl_port_add_disp(&disp_cfg);
    /* 如果注册失败，这里使用 ESP_ERROR_CHECK 以便在开发/测试阶段揭露问题 */
    ESP_ERROR_CHECK(s_disp != NULL ? ESP_OK : ESP_FAIL);

    /* 3) SPD2010 的 4 像素 X 对齐：注册到 LVGL 9 的 INVALIDATE 事件 */
    lv_display_add_event_cb(s_disp, spd2010_rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);

    /* 4) 触摸（若传入句柄则注册） */
    if (touch)
    {
        const lvgl_port_touch_cfg_t tcfg = {
            .disp = s_disp,
            .handle = touch,
        };
        s_indev = lvgl_port_add_touch(&tcfg);
        if (!s_indev)
        {
            ESP_LOGW(TAG, "lvgl_port_add_touch failed, continue without input");
        }
    }

    /* 5) 打印内存信息，便于调优 */
    multi_heap_info_t heap_i = {0}, heap_ps = {0};
    heap_caps_get_info(&heap_i, MALLOC_CAP_INTERNAL);
    heap_caps_get_info(&heap_ps, MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "Internal RAM: free=%u KB, largest=%u KB",
             (unsigned)(heap_i.total_free_bytes / 1024),
             (unsigned)(heap_i.largest_free_block / 1024));
    ESP_LOGI(TAG, "PSRAM:        free=%u KB, largest=%u KB",
             (unsigned)(heap_ps.total_free_bytes / 1024),
             (unsigned)(heap_ps.largest_free_block / 1024));

    ESP_LOGI(TAG, "LVGL ready");
    return ESP_OK;
}

esp_err_t lvgl_service_get_display(lv_display_t **out_display)
{
    if (out_display == NULL)
        return ESP_ERR_INVALID_ARG;
    if (s_disp == NULL)
        return ESP_ERR_INVALID_STATE;
    *out_display = s_disp;
    return ESP_OK;
}

esp_err_t lvgl_service_get_touch_indev(lv_indev_t **out_indev)
{
    if (out_indev == NULL)
        return ESP_ERR_INVALID_ARG;
    if (s_indev == NULL)
        return ESP_ERR_INVALID_STATE;
    *out_indev = s_indev;
    return ESP_OK;
}
