#include "lvgl_service.h"
#include "lcd_service.h" // 提供 LCD 硬件访问接口
#include "esp_lvgl_port.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "lvgl_service";

static lv_display_t *s_disp = NULL;
static lv_indev_t *s_indev = NULL;
static bool s_swap_xy = false; // 若启用硬件 swap_xy，则在回调里改到 Y 轴做对齐

/**
 * LVGL 9.3：在 LV_EVENT_INVALIDATE_AREA 事件中对齐刷新区域
 * SPD2010 要求：物理 X 方向按 4 像素对齐
 *   x1 向下对齐到 4N，x2 向上对齐到 4N+3
 * 若硬件设置了 swap_xy，那么“物理 X”映射到 LVGL 的 Y 轴
 */
static void spd2010_rounder_event_cb(lv_event_t *e)
{
    lv_area_t *area = (lv_area_t *)lv_event_get_param(e);
    if (!area)
        return;

    if (!s_swap_xy)
    {
        area->x1 = (area->x1 & ~0x3);
        area->x2 = (area->x2 | 0x3);
        if (area->x2 >= LCD_H_RES)
            area->x2 = LCD_H_RES - 1;
    }
    else
    {
        area->y1 = (area->y1 & ~0x3);
        area->y2 = (area->y2 | 0x3);
        if (area->y2 >= LCD_V_RES)
            area->y2 = LCD_V_RES - 1;
    }
}

esp_err_t lvgl_service_init(esp_lcd_panel_handle_t panel,
                            esp_lcd_panel_io_handle_t io,
                            esp_lcd_touch_handle_t touch)
{
    /* 1) 初始化 LVGL 端口（创建 LVGL 任务与 tick 定时器） */
    const lvgl_port_cfg_t port_cfg = {
        .task_priority = 5,
        .task_stack = 8 * 1024,
        .task_affinity = -1,
        .task_max_sleep_ms = 500,
        .timer_period_ms = 2, // 2ms tick（你的工程就是 2ms）
    };
    ESP_ERROR_CHECK(lvgl_port_init(&port_cfg));

    /* 2) 注册显示：esp_lvgl_port 负责 flush 与 LVGL 缓冲 */
    lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io,
        .panel_handle = panel,
        .control_handle = NULL,

        /* 用“整屏宽度 × 行数”的条带，X 自然满足 4 对齐；可按内存余量调整行数 */
        .buffer_size = LCD_H_RES * 40, // 412 * 40 行；PSRAM 充足可调到 *80/*120
        .double_buffer = true,
        .trans_size = 0, // 0=不额外分配 SRAM 中转

        .hres = LCD_H_RES, // 412
        .vres = LCD_V_RES, // 412

        .monochrome = false,

        .rotation = {.swap_xy = false, .mirror_x = false, .mirror_y = false},

#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565, // 使用 16-bit RGB565 节省内存
#endif
        .flags = {
            .buff_dma = 1,    // LVGL buffer 使用 DMA 能力内存
            .buff_spiram = 0, // 暂时禁用 PSRAM，使用内部RAM
            .sw_rotate = 0,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = 0, // XRGB8888 通常不需要字节交换
#endif
            .full_refresh = 0, // 保持分块刷新（条带）
            .direct_mode = 0,
        },
    };

    s_disp = lvgl_port_add_disp(&disp_cfg);

    /* 3) 使用 LVGL 9 的事件在“区域失效”时做 4 像素对齐 */
    s_swap_xy = disp_cfg.rotation.swap_xy ? true : false;
    lv_display_add_event_cb(s_disp, spd2010_rounder_event_cb, LV_EVENT_INVALIDATE_AREA, s_disp);

    /* 4) 注册触摸（可选；未接 INT 也能轮询） */
    if (touch)
    {
        const lvgl_port_touch_cfg_t tcfg = {
            .disp = s_disp,
            .handle = touch,
        };
        s_indev = lvgl_port_add_touch(&tcfg);
    }

    /* 5) 简单验证 UI：紫底 + 白字 */
    // lv_obj_t *scr = lv_screen_active();
    // lv_obj_set_style_bg_color(scr, lv_color_hex(0x9400D3), 0);
    // lv_obj_t *label = lv_label_create(scr);
    // lv_label_set_text(label, "Hello SPD2010 (LVGL 9.3)");
    // lv_obj_set_style_text_color(label, lv_color_white(), 0);
    // lv_obj_center(label);

    /* 输出内存使用信息 */
    multi_heap_info_t heap_info_internal, heap_info_psram;

    heap_caps_get_info(&heap_info_internal, MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "Internal RAM: free=%d KB, largest_block=%d KB",
             (int)heap_info_internal.total_free_bytes / 1024,
             (int)heap_info_internal.largest_free_block / 1024);

    heap_caps_get_info(&heap_info_psram, MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "PSRAM: free=%d KB, largest_block=%d KB",
             (int)heap_info_psram.total_free_bytes / 1024,
             (int)heap_info_psram.largest_free_block / 1024);

    ESP_LOGI(TAG, "LVGL ready with RGB565 color format (internal RAM)");
    return ESP_OK;
}

lv_display_t *lvgl_service_get_display(void) { return s_disp; }
lv_indev_t *lvgl_service_get_touch_indev(void) { return s_indev; }
