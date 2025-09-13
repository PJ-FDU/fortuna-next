#include "lcd_touch_service.h"

#include "i2c_service.h"
#include "io_expander_service.h"

#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_lcd_io_i2c.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_spd2010.h" // 提供 ESP_LCD_TOUCH_IO_I2C_SPD2010_CONFIG()

#define TAG "lcd_touch_service"

/* ===== 静态句柄 ===== */
static esp_lcd_touch_handle_t s_touch = NULL;
static esp_lcd_panel_io_handle_t s_tp_io = NULL;

/* ===== 内部工具函数 ===== */

/* 用 TCA9554 将 TP RST 拉低/拉高（EIO = LCD_TOUCH_RST_EIO） */
static void tp_reset_via_eio(void)
{
    // 设为输出
    (void)esp_io_expander_set_dir(io_expander_handle, LCD_TOUCH_RST_EIO, IO_EXPANDER_OUTPUT);

    // 低保持 >= 10ms
    (void)esp_io_expander_set_level(io_expander_handle, LCD_TOUCH_RST_EIO, 0);
    ESP_LOGI(TAG, "TP RST (EIO%d) -> LOW", LCD_TOUCH_RST_EIO);
    vTaskDelay(pdMS_TO_TICKS(60));

    // 拉高等待芯片稳定
    (void)esp_io_expander_set_level(io_expander_handle, LCD_TOUCH_RST_EIO, 1);
    ESP_LOGI(TAG, "TP RST (EIO%d) -> HIGH", LCD_TOUCH_RST_EIO);
    vTaskDelay(pdMS_TO_TICKS(200));
}

/* 配置 INT GPIO（若使用） */
static void tp_int_gpio_prepare(void)
{
#if (TOUCH_INT_GPIO >= 0)
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << TOUCH_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1, // SPD2010 的 INT 通常为低有效，给上拉
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE};
    (void)gpio_config(&io);

    int lvl = gpio_get_level(TOUCH_INT_GPIO);
    ESP_LOGI("TP-INT", "initial level=%d (0=LOW/active)", lvl);
#else
    // 未使用 INT，纯轮询
#endif
}

/* 使用 IDF v5.5 自带的 “探测” API 扫描 I2C */
static void i2c_scan_once(i2c_master_bus_handle_t bus)
{
#if TOUCH_SCAN_ON_BOOT
    ESP_LOGI(TAG, "I2C scan start");
    for (uint8_t a = 0x08; a < 0x78; ++a)
    {
        if (i2c_master_probe(bus, a, 1000) == ESP_OK)
        {
            ESP_LOGI(TAG, "  found device at 0x%02X%s",
                     a,
                     (a == TOUCH_I2C_ADDR ? " <- SPD2010?" : (a == 0x20 ? " <- TCA9554?" : "")));
        }
    }
    ESP_LOGI(TAG, "I2C scan end");
#endif
}

/* 读一遍版本/状态（通过组件内部流程），仅打印，不影响功能 */
static void tp_try_print_version(void)
{
    if (!s_touch)
        return;

    /* 组件内部在创建时会读版本；这里做一次空读触发内部状态机 */
    (void)esp_lcd_touch_read_data(s_touch);

    /* 实际坐标读取（若无触摸，则 cnt=0） */
    uint16_t x[1], y[1], s[1];
    uint8_t cnt = 0;
    bool pressed = esp_lcd_touch_get_coordinates(s_touch, x, y, s, &cnt, 1);
    ESP_LOGI(TAG, "TP first read: pressed=%d, cnt=%u%s",
             (int)pressed, cnt, cnt ? " (OK)" : "");
}

/* ===== 对外函数实现 ===== */

esp_lcd_touch_handle_t lcd_touch_service_get_handle(void)
{
    return s_touch;
}

esp_err_t lcd_touch_service_init(void)
{
    ESP_RETURN_ON_FALSE(s_touch == NULL, ESP_ERR_INVALID_STATE, TAG, "touch already inited");

    /* 1) 通过 TCA9554 复位触摸 */
    tp_reset_via_eio();

    /* 1.5) 额外等待时间，确保芯片完全启动 */
    vTaskDelay(pdMS_TO_TICKS(300));  // 额外300ms等待
    ESP_LOGI(TAG, "Waiting for SPD2010 to be ready...");

    /* 2) 取 I2C 主总线 */
    i2c_master_bus_handle_t bus = NULL;
    ESP_RETURN_ON_ERROR(i2c_master_get_bus_handle(PIN_I2C_PORT, &bus), TAG, "get i2c bus");

    /* 3) 诊断：扫描总线（可关） */
    i2c_scan_once(bus);

    /* 4) 创建 I2C 面板 IO（手动配置以解决通信问题） */
    esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr = TOUCH_I2C_ADDR,
        .scl_speed_hz = TOUCH_I2C_CLK_HZ,
        .control_phase_bytes = 0,    // SPD2010 不需要控制字节
        .lcd_cmd_bits = 8,           // 命令8位
        .lcd_param_bits = 8,         // 参数8位
        .dc_bit_offset = 0,          // 无DC位
        .flags = {
            .dc_low_on_data = 0,     // 数据时DC保持高
            .disable_control_phase = 1,  // 禁用控制阶段，直接传输数据
        },
    };

    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c_v2(bus, &io_cfg, &s_tp_io),
                        TAG, "new_panel_io_i2c_v2");

    /* 5) 创建 SPD2010 触摸句柄 */
    esp_lcd_touch_config_t tcfg = {
        .x_max = 412,
        .y_max = 412,
        .rst_gpio_num = -1,             // 复位走 TCA9554，这里设 -1
        .int_gpio_num = TOUCH_INT_GPIO, // 若未接中断则为 -1
        .levels = {
            .reset = 0,     // 默认低复位；我们未使用
            .interrupt = 0, // SPD2010 INT 通常低有效
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    tp_int_gpio_prepare();

    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_spd2010(s_tp_io, &tcfg, &s_touch),
                        TAG, "new tp");

    /* 6) 额外打一遍版本/状态（可选） */
    tp_try_print_version();

    ESP_LOGI(TAG, "SPD2010 touch ready (addr=0x%02X, int_gpio=%d, rst_eio=%d)",
             TOUCH_I2C_ADDR, TOUCH_INT_GPIO, LCD_TOUCH_RST_EIO);
    return ESP_OK;
}

void lcd_touch_service_debug_once(void)
{
    tp_try_print_version();
}
