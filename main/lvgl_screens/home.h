#pragma once

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 创建星盘对象（412x412） */
lv_obj_t *astro_create(lv_obj_t *parent);

#ifdef __cplusplus
}
#endif
