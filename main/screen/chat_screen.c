#include "esp_err.h"

#include "lvgl.h"
#include "chat_screen.h"

esp_err_t create_chat_screen(lv_obj_t *out_chat_screen)
{
	if (out_chat_screen == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}

	lv_obj_t *screen = lv_obj_create(out_chat_screen);
	if (screen == NULL)
	{
		return ESP_FAIL;
	}

	lv_obj_set_size(screen, 412, 412);
	lv_obj_align(screen, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_bg_color(screen, lv_color_hex(0xff0000), 0);

	*out_chat_screen = screen;
	return ESP_OK;
}
