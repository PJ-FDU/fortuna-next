#include "lvgl_service.h"
#include "lcd_service.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "lvgl_service";

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
        .timer_period_ms = 2, // 2ms tick
    };
    ESP_ERROR_CHECK(lvgl_port_init(&port_cfg));

    /* 2) 注册显示（使用条带缓冲；XRGB8888 支持透明通道，转换到RGB888面板；不做 SW 旋转） */
    lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io,
        .panel_handle = panel,
        .control_handle = NULL,

        /* 以“整屏宽度 * 若干行”作为缓冲；按需调大/双缓冲（看可用内存） */
        .buffer_size = LCD_H_RES * 20, // 20 行条带
        .double_buffer = false,
        .trans_size = 0,

        .hres = LCD_H_RES,
        .vres = LCD_V_RES,

        .monochrome = false,

        .rotation = {.swap_xy = false, .mirror_x = false, .mirror_y = false},

#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565, // RGB565 最稳定的格式
#endif
        .flags = {
            .buff_dma = 1,    // RGB565支持DMA，最佳性能
            .buff_spiram = 0, // 未启用PSRAM
            .sw_rotate = 0,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = 1, // 尝试字节交换，可能解决花屏问题
#endif
            .full_refresh = 0,
            .direct_mode = 0,
        },
    };

    s_disp = lvgl_port_add_disp(&disp_cfg);
    if (!s_disp)
    {
        ESP_LOGE(TAG, "lvgl_port_add_disp failed");
        return ESP_FAIL;
    }

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

lv_display_t *lvgl_service_get_display(void) { return s_disp; }
lv_indev_t *lvgl_service_get_touch_indev(void) { return s_indev; }
