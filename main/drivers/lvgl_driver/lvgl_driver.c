#include "esp_log.h"

#include "esp_lvgl_port.h"
#include "esp_lvgl_port_disp.h"
#include "esp_lvgl_port_touch.h"
#include "lvgl.h"

#include "lcd_disp_driver.h"
#include "lcd_touch_driver.h"
#include "lvgl_driver.h"

static const char *TAG = "lvgl_driver";

static void lvgl_port_rounder_callback(lv_event_t *e)
{
    if (e == NULL)
        return;

    lv_area_t *area = (lv_area_t *)lv_event_get_param(e);
    if (area == NULL)
        return;

    lv_coord_t x1 = area->x1;
    lv_coord_t x2 = area->x2;

    if (x1 < 0)
        x1 = 0;
    if (x2 < 0)
        x2 = 0;

    x1 = (x1 / 4) * 4;
    x2 = (x2 / 4) * 4 + 3;

    if (x2 >= (lv_coord_t)LCD_H_RES)
        x2 = (lv_coord_t)LCD_H_RES - 1;
    if (x1 >= (lv_coord_t)LCD_H_RES)
        x1 = (lv_coord_t)LCD_H_RES - 1;

    if (x2 < x1)
        x2 = x1;

    area->x1 = x1;
    area->x2 = x2;
}

esp_err_t lvgl_driver_init(void)
{
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    esp_lcd_panel_io_handle_t lcd_panel_io_handle = NULL;
    esp_err_t err = lcd_disp_driver_get_panel_io_handle(&lcd_panel_io_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get LCD panel IO handle: %s", esp_err_to_name(err));
        return err;
    }
    if (lcd_panel_io_handle == NULL)
    {
        ESP_LOGE(TAG, "LCD panel IO handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    esp_lcd_panel_handle_t lcd_panel_handle = NULL;
    err = lcd_disp_driver_get_panel_handle(&lcd_panel_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get LCD panel handle: %s", esp_err_to_name(err));
        return err;
    }
    if (lcd_panel_handle == NULL)
    {
        ESP_LOGE(TAG, "LCD panel handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    static lv_disp_t *s_lv_disp;

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_panel_io_handle,
        .panel_handle = lcd_panel_handle,
        .control_handle = NULL,
        .trans_size = 0,
        .buffer_size = LCD_H_RES * LCD_V_RES * 3,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB888,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
            .direct_mode = false,
            .full_refresh = false,
            .sw_rotate = false,
            .swap_bytes = false,
        }};
    s_lv_disp = lvgl_port_add_disp(&disp_cfg);
    lv_display_add_event_cb(s_lv_disp, lvgl_port_rounder_callback, LV_EVENT_INVALIDATE_AREA, NULL);

    esp_lcd_touch_handle_t lcd_touch_handle;
    err = lcd_touch_driver_get_handle(&lcd_touch_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get LCD touch handle: %s", esp_err_to_name(err));
        return err;
    }
    if (lcd_touch_handle == NULL)
    {
        ESP_LOGW(TAG, "LCD touch handle is NULL, touch input will be disabled");
        return ESP_OK;
    }

    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = s_lv_disp,
        .handle = lcd_touch_handle,
    };
    lv_indev_t *touch_handle = lvgl_port_add_touch(&touch_cfg);

    return ESP_OK;
}