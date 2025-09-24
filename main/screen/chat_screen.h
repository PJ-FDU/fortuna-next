#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_err.h"
#include "lvgl.h"

esp_err_t create_chat_screen(lv_obj_t *out_chat_screen);

#ifdef __cplusplus
}
#endif