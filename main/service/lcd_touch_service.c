#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_lcd_io_i2c.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_spd2010.h"

#include "i2c_service.h"
#include "lcd_service.h"
#include "lcd_touch_service.h"
#include "io_expander_service.h"

#define TAG "lcd_touch_service"

static esp_lcd_touch_handle_t s_lcd_touch_handle = NULL;
static esp_lcd_panel_io_handle_t s_lcd_panel_io_handle = NULL;
static SemaphoreHandle_t s_lcd_touch_sem = NULL;

static void IRAM_ATTR lcd_touch_service_isr_callback(esp_lcd_touch_handle_t lcd_touch_handle)
{
    if (s_lcd_touch_sem)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(s_lcd_touch_sem, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
    }
}

static esp_err_t lcd_touch_service_rst_exio_init(void)
{
    esp_io_expander_handle_t io_expander_handle = NULL;
    esp_err_t err = esp_io_expander_service_get_handle(&io_expander_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "IO expander not ready, %s", esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander_handle, LCD_TOUCH_RST_EXIO, IO_EXPANDER_OUTPUT));

    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander_handle, LCD_TOUCH_RST_EXIO, 0));
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander_handle, LCD_TOUCH_RST_EXIO, 1));
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "TP RST (EIO%d) inited", LCD_TOUCH_RST_EXIO);

    return ESP_OK;
}

SemaphoreHandle_t lcd_touch_service_get_int_semaphore(void)
{
    return s_lcd_touch_sem;
}

esp_err_t lcd_touch_service_get_handle(esp_lcd_touch_handle_t *lcd_touch_handle)
{
    if (lcd_touch_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lcd_touch_handle == NULL)
    {
        *lcd_touch_handle = NULL;
        return ESP_ERR_INVALID_STATE;
    }
    *lcd_touch_handle = s_lcd_touch_handle;
    return ESP_OK;
}

esp_err_t lcd_touch_service_init(void)
{
    ESP_LOGI(TAG, "Init LCD touch");

    ESP_ERROR_CHECK(lcd_touch_service_rst_exio_init());
    vTaskDelay(pdMS_TO_TICKS(300));

    i2c_master_bus_handle_t i2c_master_bus_handle = NULL;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(I2C_PORT_NUM, &i2c_master_bus_handle));
    ESP_LOGI(TAG, "I2C bus ready");

    esp_lcd_panel_io_i2c_config_t lcd_panel_io_i2c_config = ESP_LCD_TOUCH_IO_I2C_SPD2010_CONFIG();
    lcd_panel_io_i2c_config.dev_addr = LCD_TOUCH_I2C_ADDR;
    lcd_panel_io_i2c_config.scl_speed_hz = LCD_TOUCH_I2C_CLK_HZ;
    lcd_panel_io_i2c_config.lcd_cmd_bits = 8;
    lcd_panel_io_i2c_config.lcd_param_bits = 8;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(i2c_master_bus_handle, &lcd_panel_io_i2c_config, &s_lcd_panel_io_handle));
    ESP_LOGI(TAG, "I2C panel IO ready");

    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1ULL << LCD_TOUCH_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
    ESP_LOGI(TAG, "TP INT (GPIO%d) inited", LCD_TOUCH_INT_GPIO);

    esp_lcd_touch_config_t lcd_touch_config = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = LCD_TOUCH_INT_GPIO,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .interrupt_callback = lcd_touch_service_isr_callback,
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_spd2010(s_lcd_panel_io_handle, &lcd_touch_config, &s_lcd_touch_handle));
    ESP_LOGI(TAG, "LCD touch handle ready");

    s_lcd_touch_sem = xSemaphoreCreateBinary();
    if (s_lcd_touch_sem == NULL)
    {
        ESP_LOGE(TAG, "failed to create touch semaphore");
        return ESP_ERR_NO_MEM;
    }
    // ESP_ERROR_CHECK(esp_lcd_touch_register_interrupt_callback_with_data(s_lcd_touch_handle, lcd_touch_service_isr_callback, NULL));

    ESP_LOGI(TAG, "LCD touch inited");

    return ESP_OK;
}
