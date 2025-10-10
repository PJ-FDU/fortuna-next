#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#include "esp_io_expander.h"
#include "esp_lcd_touch.h"

#define LCD_TOUCH_I2C_CLK_HZ (100000)
#define LCD_TOUCH_INT_GPIO (GPIO_NUM_4)
#define LCD_TOUCH_RST_EXIO (IO_EXPANDER_PIN_NUM_0)

#ifdef __cplusplus
extern "C"
{
#endif

    esp_err_t lcd_touch_driver_init(void);

    esp_err_t lcd_touch_driver_get_handle(esp_lcd_touch_handle_t *lcd_touch_handle);

    esp_err_t lcd_touch_driver_get_io_handle(esp_lcd_panel_io_handle_t *panel_io_handle);

#ifdef __cplusplus
}
#endif
