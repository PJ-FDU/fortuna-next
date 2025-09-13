#pragma once
#include "esp_err.h"
#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ===== 可按需在 CMake 或 menuconfig 覆盖的默认参数 ===== */

/* SPD2010 7-bit I2C 地址（你的板子扫描为 0x53） */
#ifndef TOUCH_I2C_ADDR
#define TOUCH_I2C_ADDR (0x53)
#endif

/* I2C 频率（先用较低频率确保稳定性） */
#ifndef TOUCH_I2C_CLK_HZ
#define TOUCH_I2C_CLK_HZ (100000)  // 降至100kHz调试
#endif

/* 若触摸 INT 线接到 SoC GPIO，填对应管脚号；未接/不确定时设为 -1 走纯轮询 */
#ifndef TOUCH_INT_GPIO
#define TOUCH_INT_GPIO (-1) // 你确认接了 GPIO4 时可改为 4
#endif

/* 触摸 RST 连接在 TCA9554 的哪个引脚：
   你的硬件说明 “TP RST = Extend IO 1”，即 TCA9554 的 P1 => 写 1
   如果实际是 P0，请改成 0 */
#ifndef LCD_TOUCH_RST_EIO
#define LCD_TOUCH_RST_EIO (1)
#endif

/* 上电是否打印一次 I2C 扫描（仅诊断用途；生产可关） */
#ifndef TOUCH_SCAN_ON_BOOT
#define TOUCH_SCAN_ON_BOOT (1)
#endif

    /* ===== 对外 API ===== */
    esp_err_t lcd_touch_service_init(void);
    esp_lcd_touch_handle_t lcd_touch_service_get_handle(void);

    /* 诊断：手动打印一次状态/版本（可选） */
    void lcd_touch_service_debug_once(void);

#ifdef __cplusplus
}
#endif
