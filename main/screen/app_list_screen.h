#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_err.h"
#include "lvgl.h"

esp_err_t create_app_list_screen(lv_obj_t *out_app_list_screen);

#ifdef __cplusplus
}
#endif