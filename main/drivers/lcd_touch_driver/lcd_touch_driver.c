#include "esp_log.h"
#include "esp_err.h"

#include "esp_lcd_io_i2c.h"
#include "esp_lcd_touch_spd2010.h"
#include "driver/i2c_master.h"

#include "lcd_touch_driver.h"
#include "lcd_disp_driver.h"
#include "io_exp_driver.h"

static const char *TAG = "lcd_touch_driver";

static esp_lcd_touch_handle_t s_touch_handle = NULL;
static esp_lcd_panel_io_handle_t s_touch_io_handle = NULL;

static esp_err_t lcd_touch_driver_init_rst_exio(void)
{
    ESP_LOGI(TAG, "Init LCD touch RST (IO Expander pin %d)", LCD_TOUCH_RST_EXIO);

    esp_io_expander_handle_t esp_io_expander_handle = NULL;
    esp_err_t err = io_exp_driver_get_handle(&esp_io_expander_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get IO Expander handle: %s", esp_err_to_name(err));
        return err;
    }
    if (esp_io_expander_handle == NULL)
    {
        ESP_LOGE(TAG, "IO Expander handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_ERROR_CHECK(esp_io_expander_set_dir(esp_io_expander_handle, LCD_TOUCH_RST_EXIO, IO_EXPANDER_OUTPUT));

    ESP_ERROR_CHECK(esp_io_expander_set_level(esp_io_expander_handle, LCD_TOUCH_RST_EXIO, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(esp_io_expander_set_level(esp_io_expander_handle, LCD_TOUCH_RST_EXIO, 1));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "LCD touch RST (IO Expander pin %d) inited", LCD_TOUCH_RST_EXIO);

    return ESP_OK;
}

esp_err_t lcd_touch_driver_init(void)
{
    if (s_touch_handle != NULL)
    {
        ESP_LOGW(TAG, "LCD touch driver has already been initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Init LCD touch");

    ESP_ERROR_CHECK(lcd_touch_driver_init_rst_exio());

    i2c_master_bus_handle_t i2c_master_bus_handle = NULL;
    esp_err_t i2c_master_get_bus_handle_err = i2c_master_get_bus_handle(I2C_NUM_0, &i2c_master_bus_handle);
    if (i2c_master_get_bus_handle_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Get i2c bus handle failed: %s", esp_err_to_name(i2c_master_get_bus_handle_err));
        return i2c_master_get_bus_handle_err;
    }
    if (i2c_master_bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C bus not initialized. Please initialize I2C driver first.");
        return ESP_ERR_INVALID_STATE;
    }

    esp_lcd_panel_io_i2c_config_t touch_io_cfg = ESP_LCD_TOUCH_IO_I2C_SPD2010_CONFIG();
    touch_io_cfg.scl_speed_hz = LCD_TOUCH_I2C_CLK_HZ;
    touch_io_cfg.lcd_cmd_bits = 16;
    touch_io_cfg.lcd_param_bits = 8;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(i2c_master_bus_handle, &touch_io_cfg, &s_touch_io_handle));
    // ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &touch_io_cfg, &s_touch_io_handle));
    ESP_LOGI(TAG, "I2C panel IO ready");

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
        .interrupt_callback = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_spd2010(s_touch_io_handle, &lcd_touch_config, &s_touch_handle));
    ESP_LOGI(TAG, "LCD touch handle ready");

    return ESP_OK;
}

esp_err_t lcd_touch_driver_get_handle(esp_lcd_touch_handle_t *lcd_touch_handle)
{
    if (lcd_touch_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_touch_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }
    *lcd_touch_handle = s_touch_handle;
    return ESP_OK;
}

esp_err_t lcd_touch_driver_get_io_handle(esp_lcd_panel_io_handle_t *panel_io_handle)
{
    if (panel_io_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_touch_io_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }
    *panel_io_handle = s_touch_io_handle;
    return ESP_OK;
}
