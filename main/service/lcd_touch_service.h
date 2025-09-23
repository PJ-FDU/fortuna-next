#pragma once
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_io_expander.h"
#include "esp_lcd_touch.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define LCD_TOUCH_I2C_ADDR (0x53)
#define LCD_TOUCH_I2C_CLK_HZ (100000)
#define LCD_TOUCH_INT_GPIO (GPIO_NUM_4)
#define LCD_TOUCH_RST_EXIO (IO_EXPANDER_PIN_NUM_0)

    SemaphoreHandle_t lcd_touch_service_get_int_semaphore(void);

    esp_err_t lcd_touch_service_get_handle(esp_lcd_touch_handle_t *lcd_touch_handle);

    esp_err_t lcd_touch_service_init(void);

#ifdef __cplusplus
}
#endif
