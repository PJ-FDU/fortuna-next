#include "esp_err.h"

#include "lvgl.h"
#include "home_screen.h"

esp_err_t create_home_screen(lv_obj_t *out_home_screen)
{
    if (out_home_screen == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // 创建一个新的对象作为主屏幕容器
    lv_obj_t *home_screen = lv_obj_create(out_home_screen);
    if (home_screen == NULL)
    {
        return ESP_FAIL;
    }

    // 设置主屏幕容器的大小和位置
    lv_obj_set_size(home_screen, 412, 412);
    lv_obj_align(home_screen, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(home_screen, lv_color_hex(0x00ff00), 0);

    // 返回创建的主屏幕对象
    *out_home_screen = home_screen;
    return ESP_OK;
}