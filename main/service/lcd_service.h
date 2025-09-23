#pragma once

#include "esp_err.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_spd2010.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define LCD_H_RES (412)
#define LCD_V_RES (412)

#define LCD_SPI_HOST (SPI2_HOST)
#define LCD_SPI_CLK_HZ (80 * 1000 * 1000)
#define LCD_SPI_TRANS_QUEUE_SIZE (10)
#define LCD_SPI_CMD_BITS (32)
#define LCD_SPI_PARAM_BITS (8)

#define LCD_RST_GPIO (GPIO_NUM_NC)
#define LCD_BL_PWM_GPIO (GPIO_NUM_5)
#define LCD_TE_GPIO (GPIO_NUM_18)
#define LCD_PIN_GPIO (GPIO_NUM_40)
#define LCD_D0_GPIO (GPIO_NUM_46)
#define LCD_D1_GPIO (GPIO_NUM_45)
#define LCD_D2_GPIO (GPIO_NUM_42)
#define LCD_D3_GPIO (GPIO_NUM_41)
#define LCD_CS_GPIO (GPIO_NUM_21)

#define LCD_RST_EXIO (2)

#define LCD_BL_PWM_LEVEL (1)
#define LCD_BL_PWM_BRIGHTNESS_MIN (0)
#define LCD_BL_PWM_BRIGHTNESS_MAX (100)
#define LCD_BL_PWM_BRIGHTNESS_DEFAULT (70)

#define LCD_BL_PWM_CHANNEL (LEDC_CHANNEL_0)
#define LCD_BL_PWM_MODE (LEDC_LOW_SPEED_MODE)
#define LCD_BL_PWM_TIMER (LEDC_TIMER_0)
#define LCD_BL_PWM_FREQ_HZ (5000)
#define LCD_BL_PWM_DUTY_RES (LEDC_TIMER_13_BIT)

    esp_err_t lcd_service_init(void);

    esp_err_t lcd_service_set_bl(uint8_t percent);

    esp_err_t lcd_service_get_panel(esp_lcd_panel_handle_t *lcd_panel);

    esp_err_t lcd_service_get_panel_io(esp_lcd_panel_io_handle_t *lcd_panel_io);

#ifdef __cplusplus
}
#endif
