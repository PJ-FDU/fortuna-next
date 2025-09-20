#pragma once

#include "esp_err.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_spd2010.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ====== 面板参数 ====== */
#define LCD_H_RES (412)
#define LCD_V_RES (412)
#define LCD_COLOR_BITS (24) // RGB565

/* QSPI：SPI2 + 4Data */
#define LCD_SPI_HOST (SPI2_HOST)
#define LCD_SPI_CLK_HZ (80 * 1000 * 1000) // 降低到40MHz排除时序问题
#define LCD_SPI_TRANS_QUEUE_SZ (10)
#define LCD_SPI_CMD_BITS (32)
#define LCD_SPI_PARAM_BITS (8)

/* 引脚（与你示例一致） */
#define LCD_PIN_RST (-1)
#define LCD_PIN_TE (18)
#define LCD_PIN_SCK (40)
#define LCD_PIN_D0 (46)
#define LCD_PIN_D1 (45)
#define LCD_PIN_D2 (42)
#define LCD_PIN_D3 (41)
#define LCD_PIN_CS (21)

/* 面板复位改用 IO 扩展器：指定 TCA9554 的哪一个引脚做 LCD RST */
#define IOE_LCD_RST_PIN (2) /* TCA9554 的 P2，按需改 */

/* 背光 */
#define LCD_PIN_BK_LIGHT (5)
#define LCD_BKLIGHT_ON_LEVEL (1)
#define LCD_BKLIGHT_MAX (100)

/* LEDC 参数 */
#define LCD_LEDC_TIMER LEDC_TIMER_0
#define LCD_LEDC_MODE LEDC_LOW_SPEED_MODE
#define LCD_LEDC_CHANNEL LEDC_CHANNEL_0
#define LCD_LEDC_FREQ_HZ (5000)
#define LCD_LEDC_DUTY_RES LEDC_TIMER_13_BIT

    /* ========= 对外 API ========= */

    /* 由本模块实现：通过 TCA9554 做外部硬复位 */
    void lcd_service_external_reset(void);

    /* 初始化 SPI 总线、QSPI 面板 IO、SPD2010 面板；返回 ESP_OK 则可用 */
    esp_err_t lcd_service_init(void);

    /* 背光初始化与设置（0~100） */
    esp_err_t lcd_service_backlight_init(void);
    esp_err_t lcd_service_set_backlight(uint8_t percent);

    /* 取句柄 */
    esp_lcd_panel_handle_t lcd_service_get_panel(void);
    esp_lcd_panel_io_handle_t lcd_service_get_panel_io(void);

#ifdef __cplusplus
}
#endif
