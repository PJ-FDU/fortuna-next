#include "lvgl_service.h"
#include "lcd_service.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_err.h"
#include "esp_check.h"
#include "lcd_touch_service.h"

/* 模块日志标识 */
static const char *TAG = "lvgl_service";

/*
 * 模块内部持有的句柄
 * - s_disp: 由 esp_lvgl_port 层返回的 LVGL 显示对象句柄
 * - s_indev: LVGL 的触摸输入设备句柄（可选，未注册时为 NULL）
 *
 * 所有权：这些句柄由 lvgl_service 模块拥有，调用方不得释放它们。
 * 如需访问请使用下面提供的安全 getter 接口。
 */
static lv_display_t *s_disp = NULL;
static lv_indev_t *s_indev = NULL;

/* Task bridge: wait on lcd_touch_service semaphore (interrupt-driven) and wake LVGL task
 * so that LVGL will call the registered read_cb to fetch touch coordinates.
 */
static TaskHandle_t s_lvgl_touch_bridge_task = NULL;
static void lvgl_touch_bridge_task(void *arg)
{
    SemaphoreHandle_t sem = (SemaphoreHandle_t)arg;
    if (sem == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

    for (;;)
    {
        if (xSemaphoreTake(sem, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI(TAG, "lvgl_touch_bridge: semaphore taken");
            if (s_indev)
            {
                /* 唤醒 LVGL 任务以处理触摸；lvgl_port_task_wake 由 esp_lvgl_port 提供 */
                lvgl_port_task_wake(LVGL_PORT_EVENT_TOUCH, s_indev);
                ESP_LOGI(TAG, "lvgl_touch_bridge: lvgl_port_task_wake called");
            }
        }
    }
}

/*
 * SPD2010 面板要求：物理 X 方向的绘制区域需按 4 像素对齐。
 * LVGL9 不再使用 rounder_cb，因此在 LV_EVENT_INVALIDATE_AREA 事件中
 * 对要刷新的区域进行对齐处理。
 * 注意：本项目没有对 panel 进行 swap_xy（SPD2010 不支持），因此只对 X 方向对齐。
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
 * @brief 初始化 LVGL 与 esp_lvgl_port 并注册显示与触摸
 *
 * 关于错误处理：
 * - 本函数在关键初始化步骤使用 ESP_ERROR_CHECK。若这些调用失败，
 *   宏将根据 IDF 配置触发断言/中止（assert/abort）。因此在此将初始化
 *   失败视为致命错误。如果希望采用非致命的方式（返回错误给调用方），
 *   请将 ESP_ERROR_CHECK 替换为显式的错误检查并返回相应的 esp_err_t。
 *
 * 参数：
 *  - panel: 已创建并配置的 esp_lcd_panel_handle_t（不能为空）
 *  - io: 与 panel 关联的 esp_lcd_panel_io_handle_t（不能为空）
 *  - touch: 可选的触摸句柄（传 NULL 则禁用触摸）
 *
 * 返回：
 *  - 成功返回 ESP_OK（LVGL 初始化并注册显示成功）
 *  - 若发生错误且 ESP_ERROR_CHECK 不触发中止，会返回对应的错误码
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

    /* 2) 注册显示（使用条带缓冲；buffer_size 根据宏计算）
     * 注：使用条带缓冲可以在节省内存的同时保证一定的刷新吞吐量。
     */
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

    /* 3) SPD2010 的 X 方向 4 像素对齐：注册到 LVGL9 的 INVALIDATE 事件 */
    lv_display_add_event_cb(s_disp, spd2010_rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);

    /* 4) 触摸注册（若传入触摸句柄则注册） */
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
        else
        {
            /* If the touch driver uses an interrupt semaphore, create a bridge task that
             * waits on that semaphore and wakes LVGL when an interrupt occurs. This
             * supports the project's lcd_touch_service which provides a semaphore.
             */
            SemaphoreHandle_t tp_sem = lcd_touch_service_get_int_semaphore();
            if (tp_sem != NULL)
            {
                if (s_lvgl_touch_bridge_task == NULL)
                {
                    BaseType_t ret = xTaskCreatePinnedToCore(lvgl_touch_bridge_task, "lvgl_tp_bridge", 2048, (void *)tp_sem, tcfg.disp ? 5 : 3, &s_lvgl_touch_bridge_task, tcfg.disp ? 0 : tskNO_AFFINITY);
                    if (ret != pdPASS)
                    {
                        ESP_LOGW(TAG, "Failed to create lvgl touch bridge task");
                    }
                }
            }
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
