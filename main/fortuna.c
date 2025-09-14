#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>

#include "i2c_service.h"
#include "io_expander_service.h"
#include "lcd_service.h"
#include "lcd_touch_service.h"
#include "lvgl_service.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"
#include <math.h>
#include "lvgl_screens/home.h"  /* 抽离出的星盘创建函数 */

static const char *TAG = "FORTUNA";

/*===========================*
 *      背景样式（屏幕渐变）
 *===========================*/
static lv_style_t style_scr_bg;

static void style_scr_bg_init(void)
{
    lv_style_init(&style_scr_bg);
    lv_style_set_bg_opa(&style_scr_bg, LV_OPA_COVER);
    /* 竖向线性渐变（上浅下深）；可按需改色 */
    lv_style_set_bg_color(&style_scr_bg, lv_color_hex(0x5A6FFF));
    lv_style_set_bg_grad_color(&style_scr_bg, lv_color_hex(0x000000));
    lv_style_set_bg_grad_dir(&style_scr_bg, LV_GRAD_DIR_VER);
}

/* astro_create 相关实现已移动到 lvgl_screens/home.c */

/*===========================*
 *           app_main
 *===========================*/
void app_main(void)
{
    ESP_LOGI(TAG, "System initialization started");

    /* 1) I2C 总线 & IO 扩展（TCA9554） */
    ESP_ERROR_CHECK(esp_i2c_service_init());
    ESP_ERROR_CHECK(esp_io_expander_service_init());

    /* 2) LCD（QSPI + SPD2010 面板） */
    ESP_ERROR_CHECK(lcd_service_init());
    ESP_LOGI(TAG, "LCD ready");

    /* 3) 触摸（如果需要） */
    // ESP_ERROR_CHECK(lcd_touch_service_init());
    // lcd_touch_service_debug_once();

    /* 4) LVGL 服务 */
    ESP_ERROR_CHECK(lvgl_service_init(
        lcd_service_get_panel(),
        lcd_service_get_panel_io(),
        NULL));

    /* ========== 创建 UI ========== */
    lvgl_port_lock(portMAX_DELAY);

    /* 屏幕渐变背景样式 */
    style_scr_bg_init();
    lv_obj_add_style(lv_screen_active(), &style_scr_bg, 0);

    /* 星盘主体（412×412，自绘） */
    astro_create(lv_screen_active());

    lvgl_port_unlock();

    ESP_LOGI(TAG, "System initialization completed");
}
