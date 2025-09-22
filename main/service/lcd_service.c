#include "lcd_service.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "io_expander_service.h" // IO 扩展器 service
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TAG "LCD_SERVICE"

/**
 * @brief Internal handles for the panel and its IO
 *
 * These are set during lcd_service_init() and returned via the safe getter
 * functions. Callers must not free these handles; they are owned by the
 * lcd_service module.
 */
static esp_lcd_panel_io_handle_t s_io = NULL;
static esp_lcd_panel_handle_t s_panel = NULL;
// 保护背光设置的互斥
static SemaphoreHandle_t s_bklight_mutex = NULL;

/* 通过 TCA9554 做硬复位 */
/**
 * @brief Perform an external hardware reset using the TCA9554 IO expander
 *
 * This function attempts to obtain the io_expander handle via
 * esp_io_expander_service_get_handle(). If the IO expander is not
 * available the function logs a warning and returns without error.
 *
 * Notes:
 * - The function uses vTaskDelay during the reset sequence and therefore
 *   should not be used in hard real-time contexts.
 */
void lcd_service_external_reset(void)
{
    // 通过安全 getter 获取 io_expander 句柄，避免直接访问全局变量
    esp_io_expander_handle_t handle = NULL;
    if (esp_io_expander_service_get_handle(&handle) != ESP_OK)
    {
        ESP_LOGW(TAG, "io_expander not ready, skipping external reset");
        return;
    }

    // 配置为输出并执行硬复位序列（低电平复位，随后拉高）
    esp_io_expander_set_dir(handle, IOE_LCD_RST_PIN, GPIO_MODE_OUTPUT);
    esp_io_expander_set_level(handle, IOE_LCD_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_io_expander_set_level(handle, IOE_LCD_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

// removed: lcd_service_backlight_init - backlight initialization is now
// performed inside lcd_service_init to keep initialization atomic.

/**
 * @brief Set backlight brightness
 *
 * Percent range: 0..LCD_BKLIGHT_MAX. Internally this maps to LEDC duty
 * resolution. This function may call LEDC APIs which can return errors.
 */

esp_err_t lcd_service_set_backlight(uint8_t percent)
{
    if (percent > LCD_BKLIGHT_MAX)
        percent = LCD_BKLIGHT_MAX;
    uint32_t max_duty = (1u << LCD_LEDC_DUTY_RES) - 1;
    uint32_t duty = (uint32_t)((percent * 1.0f / LCD_BKLIGHT_MAX) * max_duty + 0.5f);
    if (!LCD_BKLIGHT_ON_LEVEL)
        duty = max_duty - duty;
    if (s_bklight_mutex == NULL)
    {
        ESP_LOGE(TAG, "backlight mutex not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_bklight_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGW(TAG, "failed to take backlight mutex");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ESP_OK;
    if (ledc_set_duty(LCD_LEDC_MODE, LCD_LEDC_CHANNEL, duty) != ESP_OK)
    {
        err = ESP_FAIL;
    }
    else if (ledc_update_duty(LCD_LEDC_MODE, LCD_LEDC_CHANNEL) != ESP_OK)
    {
        err = ESP_FAIL;
    }

    xSemaphoreGive(s_bklight_mutex);
    return err;
}

esp_err_t lcd_service_init(void)
{
    ESP_LOGI(TAG, "Init SPI bus for QSPI");
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PIN_SCK,
        .data0_io_num = LCD_PIN_D0,
        .data1_io_num = LCD_PIN_D1,
        .data2_io_num = LCD_PIN_D2,
        .data3_io_num = LCD_PIN_D3,
        .max_transfer_sz = 4096,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi_bus_init");

    ESP_LOGI(TAG, "Create QSPI panel IO");
    esp_lcd_panel_io_spi_config_t io_spi_cfg = {
        .cs_gpio_num = LCD_PIN_CS,
        .dc_gpio_num = -1,
        .spi_mode = 3,
        .pclk_hz = LCD_SPI_CLK_HZ,
        .trans_queue_depth = LCD_SPI_TRANS_QUEUE_SZ,
        .lcd_cmd_bits = LCD_SPI_CMD_BITS,
        .lcd_param_bits = LCD_SPI_PARAM_BITS,
        .flags = {.quad_mode = 1},
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_spi_cfg, &s_io),
                        TAG, "new_panel_io_spi");

    ESP_LOGI(TAG, "Create SPD2010 panel");
    spd2010_vendor_config_t vcfg = {
        .flags = {
            .use_qspi_interface = 1, // QSPI 接口；SPI 则设为 0
        },
    };

    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_PIN_RST,              // 没有 RST 就用 -1
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR, // 或 LCD_RGB_ELEMENT_ORDER_BGR
        .bits_per_pixel = 24,                       // 16-bit RGB565 匹配LVGL
        .vendor_config = &vcfg,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_spd2010(s_io, &panel_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, false));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, 0, 0));

    // 初始化背光硬件（配置 LEDC timer & channel）并设置默认亮度
    ESP_RETURN_ON_ERROR(gpio_set_direction(LCD_PIN_BK_LIGHT, GPIO_MODE_OUTPUT), TAG, "bk gpio dir");
    ledc_timer_config_t tcfg = {
        .speed_mode = LCD_LEDC_MODE,
        .duty_resolution = LCD_LEDC_DUTY_RES,
        .timer_num = LCD_LEDC_TIMER,
        .freq_hz = LCD_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&tcfg), TAG, "ledc timer");

    ledc_channel_config_t ccfg = {
        .gpio_num = LCD_PIN_BK_LIGHT,
        .speed_mode = LCD_LEDC_MODE,
        .channel = LCD_LEDC_CHANNEL,
        .timer_sel = LCD_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ccfg), TAG, "ledc ch");

    // 创建保护背光设置的互斥
    s_bklight_mutex = xSemaphoreCreateMutex();
    if (s_bklight_mutex == NULL)
    {
        ESP_LOGE(TAG, "failed to create backlight mutex");
        return ESP_ERR_NO_MEM;
    }

    // 设置默认亮度（使用宏 LCD_DEFAULT_BRIGHTNESS）
    ESP_RETURN_ON_ERROR(lcd_service_set_backlight(LCD_DEFAULT_BRIGHTNESS), TAG, "set bk");

    ESP_LOGI(TAG, "LCD ready");
    return ESP_OK;
}

/**
 * @brief Get panel handle in a safe manner
 *
 * Returns ESP_OK and fills out_panel on success, or an error code if the
 * module has not been initialized or the out pointer is invalid.
 */

esp_err_t lcd_service_get_panel(esp_lcd_panel_handle_t *out_panel)
{
    if (out_panel == NULL)
        return ESP_ERR_INVALID_ARG;
    if (s_panel == NULL)
        return ESP_ERR_INVALID_STATE;
    *out_panel = s_panel;
    return ESP_OK;
}

esp_err_t lcd_service_get_panel_io(esp_lcd_panel_io_handle_t *out_io)
{
    if (out_io == NULL)
        return ESP_ERR_INVALID_ARG;
    if (s_io == NULL)
        return ESP_ERR_INVALID_STATE;
    *out_io = s_io;
    return ESP_OK;
}
