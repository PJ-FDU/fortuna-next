#include "esp_log.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_spd2010.h"

#include "esp_io_expander.h"
#include "esp_io_expander_tca9554.h"

#include "lcd_disp_driver.h"
#include "io_exp_driver.h"

static const char *TAG = "lcd_disp_driver";

static esp_lcd_panel_handle_t s_lcd_panel_handle = NULL;
static esp_lcd_panel_io_handle_t s_lcd_panel_io_handle = NULL;

static SemaphoreHandle_t s_backlight_mutex = NULL;

static esp_err_t lcd_disp_driver_init_backlight(void)
{
    ESP_LOGI(TAG, "Init LCD backlight (PWM on GPIO%d)", LCD_BL_PWM_GPIO);

    ESP_ERROR_CHECK(gpio_set_direction(LCD_BL_PWM_GPIO, GPIO_MODE_OUTPUT));

    ledc_timer_config_t timer_config = {
        .speed_mode = LCD_BL_PWM_MODE,
        .duty_resolution = LCD_BL_PWM_DUTY_RES,
        .timer_num = LCD_BL_PWM_TIMER,
        .freq_hz = LCD_BL_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {
        .gpio_num = LCD_BL_PWM_GPIO,
        .speed_mode = LCD_BL_PWM_MODE,
        .channel = LCD_BL_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LCD_BL_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    s_backlight_mutex = xSemaphoreCreateMutex();
    if (s_backlight_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create backlight mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "LCD backlight (PWM on GPIO%d) inited", LCD_BL_PWM_GPIO);
    return ESP_OK;
}

static esp_err_t lcd_disp_driver_init_rst_exio(void)
{
    ESP_LOGI(TAG, "Init LCD RST (IO Expander pin %d)", LCD_RST_EXIO);

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

    ESP_ERROR_CHECK(esp_io_expander_set_dir(esp_io_expander_handle, LCD_RST_EXIO, IO_EXPANDER_OUTPUT));

    ESP_ERROR_CHECK(esp_io_expander_set_level(esp_io_expander_handle, LCD_RST_EXIO, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(esp_io_expander_set_level(esp_io_expander_handle, LCD_RST_EXIO, 1));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "LCD RST (IO Expander pin %d) inited", LCD_RST_EXIO);
    return ESP_OK;
}

esp_err_t lcd_disp_driver_set_backlight(uint8_t percent)
{
    if (percent > LCD_BL_PWM_BRIGHTNESS_MAX)
    {
        percent = LCD_BL_PWM_BRIGHTNESS_MAX;
    }
    uint32_t duty_max = (1u << LCD_BL_PWM_DUTY_RES) - 1;
    uint32_t duty = (uint32_t)((percent * 1.0f / LCD_BL_PWM_BRIGHTNESS_MAX) * duty_max + 0.5f);
    if (LCD_BL_PWM_LEVEL == 0)
    {
        duty = duty_max - duty;
    }

    if (s_backlight_mutex == NULL)
    {
        ESP_LOGE(TAG, "Backlight mutex not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_backlight_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to take backlight mutex");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ledc_set_duty(LCD_BL_PWM_MODE, LCD_BL_PWM_CHANNEL, duty);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ledc_set_duty failed: %s", esp_err_to_name(err));
        xSemaphoreGive(s_backlight_mutex);
        return err;
    }

    err = ledc_update_duty(LCD_BL_PWM_MODE, LCD_BL_PWM_CHANNEL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ledc_update_duty failed: %s", esp_err_to_name(err));
        xSemaphoreGive(s_backlight_mutex);
        return err;
    }

    xSemaphoreGive(s_backlight_mutex);
    return ESP_OK;
}

esp_err_t lcd_disp_driver_init(void)
{

    if (s_lcd_panel_handle != NULL)
    {
        ESP_LOGW(TAG, "LCD display driver has already been initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Init LCD");

    spi_bus_config_t spi_bus_config = {
        .sclk_io_num = LCD_PIN_GPIO,
        .data0_io_num = LCD_D0_GPIO,
        .data1_io_num = LCD_D1_GPIO,
        .data2_io_num = LCD_D2_GPIO,
        .data3_io_num = LCD_D3_GPIO,
        .max_transfer_sz = 4096,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &spi_bus_config, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t esp_lcd_panel_io_spi_config = {
        .cs_gpio_num = LCD_CS_GPIO,
        .dc_gpio_num = GPIO_NUM_NC,
        .spi_mode = 3,
        .pclk_hz = LCD_SPI_CLK_HZ,
        .trans_queue_depth = LCD_SPI_TRANS_QUEUE_SIZE,
        .lcd_cmd_bits = LCD_SPI_CMD_BITS,
        .lcd_param_bits = LCD_SPI_PARAM_BITS,
        .flags = {.quad_mode = 1},
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &esp_lcd_panel_io_spi_config, &s_lcd_panel_io_handle));

    ESP_ERROR_CHECK(lcd_disp_driver_init_backlight());
    ESP_ERROR_CHECK(lcd_disp_driver_init_rst_exio());

    spd2010_vendor_config_t spd2010_vendor_config = {
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    esp_lcd_panel_dev_config_t lcd_panel_dev_config = {
        .reset_gpio_num = LCD_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 24,
        .vendor_config = &spd2010_vendor_config,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_spd2010(s_lcd_panel_io_handle, &lcd_panel_dev_config, &s_lcd_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_lcd_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_lcd_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_lcd_panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_lcd_panel_handle, 0, 0));
    ESP_ERROR_CHECK(lcd_disp_driver_set_backlight(LCD_BL_PWM_BRIGHTNESS_DEFAULT));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_lcd_panel_handle, true));

    ESP_LOGI(TAG, "LCD inited");
    return ESP_OK;
}

esp_err_t lcd_disp_driver_get_panel_handle(esp_lcd_panel_handle_t *esp_lcd_panel_handle)
{
    if (esp_lcd_panel_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lcd_panel_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }
    *esp_lcd_panel_handle = s_lcd_panel_handle;
    return ESP_OK;
}

esp_err_t lcd_disp_driver_get_panel_io_handle(esp_lcd_panel_io_handle_t *panel_io_handle)
{
    if (panel_io_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lcd_panel_io_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }
    *panel_io_handle = s_lcd_panel_io_handle;
    return ESP_OK;
}