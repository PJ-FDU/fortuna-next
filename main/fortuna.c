#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_partition.h" // 添加分区API
#include "esp_heap_caps.h" // 添加内存查看
#include "esp_psram.h"
#include "esp_spiffs.h"
#include "dirent.h"
#include "driver/gpio.h"

#include "i2c_driver.h"
#include "io_exp_driver.h"
#include "lcd_disp_driver.h"
#include "lcd_touch_driver.h"
#include "lvgl_driver.h"

#include "lvgl.h" // LVGL主头文件
#include "esp_lvgl_port.h"

static const char *TAG = "main_fortuna";
static lv_obj_t *s_status_label = NULL;

static esp_err_t ensure_spiffs_mounted(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "spiffs",
        .max_files = 5,
        .format_if_mount_failed = false,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret == ESP_ERR_INVALID_STATE)
    {
        return ESP_OK;
    }
    return ret;
}

static void create_img_bg(void)
{
    esp_err_t err = ensure_spiffs_mounted();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(err));
        return;
    }

    lv_image_cache_resize(0, true);
    lv_obj_t *scr = lv_scr_act();

    lv_obj_t *img = lv_image_create(scr);
    lv_image_set_src(img, "S:/bg.png");
    const int32_t w = lv_obj_get_self_width(img);
    const int32_t h = lv_obj_get_self_height(img);
    ESP_LOGI(TAG, "Background image size: %" LV_PRId32 " x %" LV_PRId32, w, h);
    lv_obj_set_size(img, LV_PCT(100), LV_PCT(100));
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_move_background(img);
}

static const char *gesture_dir_to_text(lv_dir_t dir)
{
    switch (dir)
    {
    case LV_DIR_LEFT:
        return "Gesture: Left";
    case LV_DIR_RIGHT:
        return "Gesture: Right";
    case LV_DIR_TOP:
        return "Gesture: Up";
    case LV_DIR_BOTTOM:
        return "Gesture: Down";
    default:
        return "Gesture: Unknown";
    }
}

static void screen_event_cb(lv_event_t *e)
{
    if (!e)
    {
        return;
    }

    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_SHORT_CLICKED)
    {
        lv_indev_t *indev = lv_event_get_indev(e);
        if (!indev)
        {
            indev = lv_indev_get_act();
        }
        if (!indev || s_status_label == NULL)
        {
            return;
        }

        lv_point_t p;
        lv_indev_get_point(indev, &p);
        lv_label_set_text_fmt(s_status_label, "x:%" LV_PRId32 " y:%" LV_PRId32, p.x, p.y);
    }
    else if (code == LV_EVENT_GESTURE)
    {
        if (s_status_label == NULL)
        {
            return;
        }
        lv_indev_t *indev = lv_event_get_indev(e);
        if (!indev)
        {
            indev = lv_indev_get_act();
        }
        lv_dir_t dir = indev ? lv_indev_get_gesture_dir(indev) : LV_DIR_NONE;
        lv_label_set_text(s_status_label, gesture_dir_to_text(dir));
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "=== Fortuna System Starting ===");

    ESP_ERROR_CHECK(i2c_driver_init());
    ESP_ERROR_CHECK(io_exp_driver_init());
    ESP_ERROR_CHECK(lcd_disp_driver_init());
    ESP_ERROR_CHECK(lcd_touch_driver_init());
    ESP_ERROR_CHECK(lvgl_driver_init());

    // 在这里进行i2c scan
    i2c_master_bus_handle_t i2c_master_bus_handle = NULL;
    esp_err_t i2c_master_get_bus_handle_err = i2c_master_get_bus_handle(I2C_NUM_0, &i2c_master_bus_handle);
    if (i2c_master_get_bus_handle_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Get I2C bus handle failed: %s", esp_err_to_name(i2c_master_get_bus_handle_err));
        return;
    }
    if (i2c_master_bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C bus not initialized. Please initialize I2C driver first.");
        return;
    }
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(i2c_master_bus_handle, addr, 10) == ESP_OK) {
            ESP_LOGI("I2C_DBG", "device at 0x%02X", addr);
        }
    }

    // 写个最简单的示例
    if (lvgl_port_lock(0))
    {
        create_img_bg();

        s_status_label = lv_label_create(lv_scr_act());
        lv_label_set_text(s_status_label, "Hello, Fortuna!");
        lv_obj_align(s_status_label, LV_ALIGN_CENTER, 0, 12);
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xffffff), LV_PART_MAIN);
        lv_obj_set_style_text_font(s_status_label, lv_theme_get_font_large(lv_scr_act()), LV_PART_MAIN);
        lv_obj_set_style_text_align(s_status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

        lv_obj_add_flag(lv_scr_act(), LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(lv_scr_act(), screen_event_cb, LV_EVENT_ALL, NULL);

        lvgl_port_unlock();
    }
    else
    {
        ESP_LOGE(TAG, "Failed to lock LVGL port for UI init");
    }

    
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
