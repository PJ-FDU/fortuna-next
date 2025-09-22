/* lcd_service.h
 * 简化的 LCD 服务头：负责 SPI/面板初始化与背光控制。
 * 说明：把具体 GPIO 引脚使用 `driver/gpio.h` 中的 `GPIO_NUM_x` 宏来表达，
 * 避免裸数字以提高可读性和可移植性。
 */

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

/* ====== 面板/总线参数（保留必要的配置） ====== */
#define LCD_H_RES (412)
#define LCD_V_RES (412)

/* QSPI：SPI2 + 4Data */
#define LCD_SPI_HOST (SPI2_HOST)
/* 若遇到时序问题，可在配置或 Kconfig 中调低 */
#define LCD_SPI_CLK_HZ (80 * 1000 * 1000)
#define LCD_SPI_TRANS_QUEUE_SZ (10)
#define LCD_SPI_CMD_BITS (32)
#define LCD_SPI_PARAM_BITS (8)

/* 引脚定义：使用 driver/gpio.h 中的 GPIO_NUM_x 宏代替裸数字 */
#define LCD_PIN_RST GPIO_NUM_NC
#define LCD_PIN_TE GPIO_NUM_18
#define LCD_PIN_SCK GPIO_NUM_40
#define LCD_PIN_D0 GPIO_NUM_46
#define LCD_PIN_D1 GPIO_NUM_45
#define LCD_PIN_D2 GPIO_NUM_42
#define LCD_PIN_D3 GPIO_NUM_41
#define LCD_PIN_CS GPIO_NUM_21

/* 面板复位由 IO 扩展器控制：指定 TCA9554 的哪一个引脚做 LCD RST */
#define IOE_LCD_RST_PIN (2) /* TCA9554 的 P2，按需调整 */

/* 背光 GPIO 与参数 */
#define LCD_PIN_BK_LIGHT GPIO_NUM_5
#define LCD_BKLIGHT_ON_LEVEL (1)
#define LCD_BKLIGHT_MAX (100)

/* 默认背光亮度（0..LCD_BKLIGHT_MAX）。如需更改，可在此处修改或将其移入 Kconfig。 */
#define LCD_DEFAULT_BRIGHTNESS (70)

/* LEDC 参数 */
#define LCD_LEDC_TIMER LEDC_TIMER_0
#define LCD_LEDC_MODE LEDC_LOW_SPEED_MODE
#define LCD_LEDC_CHANNEL LEDC_CHANNEL_0
#define LCD_LEDC_FREQ_HZ (5000)
#define LCD_LEDC_DUTY_RES LEDC_TIMER_13_BIT

    /* ========= 对外 API ========= */

    /**
     * @brief 通过 TCA9554 做外部硬复位（如果 IO 扩展器可用）
     *
     * 该函数流程：
     *  - 尝试获取 io_expander 句柄（通过 esp_io_expander_service_get_handle）。
     *  - 若句柄可用，将指定的 IOE_LCD_RST_PIN 配置为输出并执行低电平复位序列。
     *  - 若 io_expander 不可用，函数会写一条警告日志并返回（不触发错误或重启）。
     *
     * 注意：该函数在复位期间会短暂调用 vTaskDelay，因此应在非实时关键路径调用。
     */
    void lcd_service_external_reset(void);

    /**
     * @brief 初始化 LCD 服务（SPI 总线、面板 IO、面板驱动、背光）
     *
     * 返回：ESP_OK 表示初始化成功，其他错误码表示某个底层步骤失败（例如 SPI/LEDC/面板创建失败）。
     * 调用者应对返回值进行检查并在失败时采取相应措施。
     */
    esp_err_t lcd_service_init(void);

    /**
     * @brief 初始化 LCD 服务（SPI 总线、面板 IO、面板驱动、背光）
     *
     * 说明：此函数将会完成面板 IO、面板驱动的创建与初始化，
     * 并在内部配置 LEDC（背光）以及将背光设置为 `LCD_DEFAULT_BRIGHTNESS`。
     * 因此调用方无需也不应再单独调用背光初始化函数。
     *
     * 返回：ESP_OK 表示初始化成功，其他错误码表示某个底层步骤失败
     *（例如 SPI/LEDC/面板创建失败）。调用者应对返回值进行检查并在失败时采取相应措施。
     */

    /**
     * @brief 设置背光亮度（0 ~ LCD_BKLIGHT_MAX）
     *
     * 参数：percent - 目标亮度，超过 LCD_BKLIGHT_MAX 将被截断。
     */
    esp_err_t lcd_service_set_backlight(uint8_t percent);

    /**
     * @brief 获取面板句柄（安全接口）
     *
     * 使用 out 参数返回句柄，并通过返回值报告状态：
     *  - ESP_OK: 成功并填充 out_panel
     *  - ESP_ERR_INVALID_ARG: out_panel 为 NULL
     *  - ESP_ERR_INVALID_STATE: 面板尚未初始化
     */
    esp_err_t lcd_service_get_panel(esp_lcd_panel_handle_t *out_panel);

    /**
     * @brief 获取面板 IO 句柄（安全接口）
     */
    esp_err_t lcd_service_get_panel_io(esp_lcd_panel_io_handle_t *out_io);

#ifdef __cplusplus
}
#endif
