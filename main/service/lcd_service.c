#include "lcd_service.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "io_expander_service.h" // <- 你的 IO 扩展器 service

#define TAG "LCD_SERVICE"

static esp_lcd_panel_io_handle_t s_io = NULL;
static esp_lcd_panel_handle_t s_panel = NULL;

/* 通过 TCA9554 做硬复位 */
void lcd_service_external_reset(void)
{
    // 确保 io_expander 已初始化
    if (!io_expander_handle)
        return;

    // 配置为输出
    esp_io_expander_set_dir(io_expander_handle, IOE_LCD_RST_PIN, GPIO_MODE_OUTPUT);
    // 低电平复位
    esp_io_expander_set_level(io_expander_handle, IOE_LCD_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    // 拉高释放
    esp_io_expander_set_level(io_expander_handle, IOE_LCD_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

esp_err_t lcd_service_backlight_init(void)
{
    ESP_RETURN_ON_ERROR(gpio_set_direction(LCD_PIN_BK_LIGHT, GPIO_MODE_OUTPUT), TAG, "bk gpio dir");
    // LEDC 配置
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
    return ESP_OK;
}

esp_err_t lcd_service_set_backlight(uint8_t percent)
{
    if (percent > LCD_BKLIGHT_MAX)
        percent = LCD_BKLIGHT_MAX;
    uint32_t max_duty = (1u << LCD_LEDC_DUTY_RES) - 1;
    uint32_t duty = (uint32_t)((percent * 1.0f / LCD_BKLIGHT_MAX) * max_duty + 0.5f);
    if (!LCD_BKLIGHT_ON_LEVEL)
        duty = max_duty - duty;
    ESP_ERROR_CHECK(ledc_set_duty(LCD_LEDC_MODE, LCD_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LCD_LEDC_MODE, LCD_LEDC_CHANNEL));
    return ESP_OK;
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
        .max_transfer_sz = 2048,
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
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB, // 或 LCD_RGB_ELEMENT_ORDER_BGR
        .bits_per_pixel = 16,                       // 使用 16-bit RGB565
        .vendor_config = &vcfg,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_spd2010(s_io, &panel_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, false));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, 0, 0));

    ESP_ERROR_CHECK(lcd_service_backlight_init());
    ESP_ERROR_CHECK(lcd_service_set_backlight(70));

    ESP_LOGI(TAG, "LCD ready");
    return ESP_OK;
}

esp_lcd_panel_handle_t lcd_service_get_panel(void) { return s_panel; }
esp_lcd_panel_io_handle_t lcd_service_get_panel_io(void) { return s_io; }
