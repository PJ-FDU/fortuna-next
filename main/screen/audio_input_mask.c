#include "esp_err.h"
#include "lvgl.h"
#include "audio_input_mask.h"

static const char *TAG = "audio_input_mask";

static lv_obj_t *s_audio_input_mask = NULL;

/* style for inner glow */
static lv_style_t s_audio_input_mask_style;

esp_err_t show_audio_input_mask(void)
{
    if (s_audio_input_mask != NULL)
    {
        lv_obj_del(s_audio_input_mask);
        s_audio_input_mask = NULL;
    }
    s_audio_input_mask = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_audio_input_mask, 412, 412);
    /* make the parent rounded and enable clipping so child shadow won't overflow */
    lv_obj_set_style_radius(s_audio_input_mask, 206, 0);
    lv_obj_set_style_clip_corner(s_audio_input_mask, true, 0);
    lv_obj_set_style_bg_color(s_audio_input_mask, lv_color_hex(0x000000), 0);
    /* 35% opacity ≈ 0.35 * 255 = 89 */
    lv_obj_set_style_bg_opa(s_audio_input_mask, LV_OPA_70, 0);
    lv_obj_invalidate(s_audio_input_mask);
    return ESP_OK;
}

esp_err_t hide_audio_input_mask(void)
{
    if (s_audio_input_mask != NULL)
    {
        lv_obj_del(s_audio_input_mask);
        s_audio_input_mask = NULL;
    }
    return ESP_OK;
}