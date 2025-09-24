#include "esp_err.h"

#include "lvgl.h"
#include "app_list_screen.h"

esp_err_t create_app_list_screen(lv_obj_t *out_app_list_screen)
{
    if (out_app_list_screen == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    lv_obj_t *screen = lv_obj_create(out_app_list_screen);
    if (screen == NULL)
    {
        return ESP_FAIL;
    }

    lv_obj_set_size(screen, 412, 412);
    lv_obj_align(screen, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x0000ff), 0);

    *out_app_list_screen = screen;
    return ESP_OK;
}
