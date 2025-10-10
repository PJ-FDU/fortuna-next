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

static const char *TAG = "main_fortuna";

static void screen_click_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Screen clicked event");
    if (!e) return;
    /* Try to get current input device (first registered) */
    lv_indev_t *indev = lv_indev_get_next(NULL);
    if (!indev) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);
    ESP_LOGI(TAG, "Touch at: x=%d y=%d", p.x, p.y);
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
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello, Fortuna!");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_event_cb(lv_scr_act(), screen_click_cb, LV_EVENT_CLICKED, NULL);

    
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}