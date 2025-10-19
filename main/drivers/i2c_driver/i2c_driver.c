#include <stdbool.h>

#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c_master.h"

#include "i2c_driver.h"

typedef struct
{
    i2c_port_t port;
    gpio_num_t scl_gpio;
    gpio_num_t sda_gpio;
    i2c_clock_source_t clk_src;
    uint32_t clk_hz;
    uint32_t glitch_ignore_cnt;
    bool enable_internal_pullup;
} i2c_driver_bus_config_entry_t;

static const char *TAG = "i2c_driver";

static const i2c_driver_bus_config_entry_t s_bus_configs[I2C_DRIVER_BUS_MAX] = {
    [I2C_DRIVER_BUS_MAIN] = {
        .port = I2C_NUM_0,
        .scl_gpio = I2C_BUS0_SCL_GPIO,
        .sda_gpio = I2C_BUS0_SDA_GPIO,
        .clk_src = I2C_BUS0_CLK_SRC,
        .clk_hz = I2C_BUS0_CLK_HZ,
        .glitch_ignore_cnt = I2C_BUS0_GLITCH_IGNORE_CNT,
        .enable_internal_pullup = I2C_BUS0_ENABLE_INTERNAL_PULLUP,
    },
    [I2C_DRIVER_BUS_AUX] = {
        .port = I2C_NUM_1,
        .scl_gpio = I2C_BUS1_SCL_GPIO,
        .sda_gpio = I2C_BUS1_SDA_GPIO,
        .clk_src = I2C_BUS1_CLK_SRC,
        .clk_hz = I2C_BUS1_CLK_HZ,
        .glitch_ignore_cnt = I2C_BUS1_GLITCH_IGNORE_CNT,
        .enable_internal_pullup = I2C_BUS1_ENABLE_INTERNAL_PULLUP,
    },
};

static i2c_master_bus_handle_t s_bus_handles[I2C_DRIVER_BUS_MAX] = {NULL};

static bool i2c_driver_config_is_valid(const i2c_driver_bus_config_entry_t *cfg)
{
    return cfg->scl_gpio != GPIO_NUM_NC && cfg->sda_gpio != GPIO_NUM_NC;
}

static esp_err_t i2c_driver_init_bus_internal(i2c_driver_bus_t bus)
{
    const i2c_driver_bus_config_entry_t *cfg = &s_bus_configs[bus];
    if (!i2c_driver_config_is_valid(cfg))
    {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_master_bus_config_t master_bus_cfg = {
        .clk_source = cfg->clk_src,
        .i2c_port = cfg->port,
        .scl_io_num = cfg->scl_gpio,
        .sda_io_num = cfg->sda_gpio,
        .glitch_ignore_cnt = cfg->glitch_ignore_cnt,
        .flags.enable_internal_pullup = cfg->enable_internal_pullup,
    };

    esp_err_t err = i2c_new_master_bus(&master_bus_cfg, &s_bus_handles[bus]);
    if (err != ESP_OK)
    {
        s_bus_handles[bus] = NULL;
        ESP_LOGE(TAG, "Failed to create I2C master bus %d: %s", (int)cfg->port, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C bus %d initialized: SCL=%d SDA=%d freq=%u", (int)cfg->port, cfg->scl_gpio, cfg->sda_gpio, cfg->clk_hz);
    return ESP_OK;
}

esp_err_t i2c_driver_init(void)
{
    esp_err_t result = ESP_OK;
    for (i2c_driver_bus_t bus = 0; bus < I2C_DRIVER_BUS_MAX; ++bus)
    {
        if (s_bus_handles[bus] != NULL)
        {
            continue;
        }

        if (!i2c_driver_config_is_valid(&s_bus_configs[bus]))
        {
            continue;
        }

        esp_err_t err = i2c_driver_init_bus_internal(bus);
        if (err != ESP_OK)
        {
            result = err;
        }
    }
    return result;
}

esp_err_t i2c_driver_init_bus(i2c_driver_bus_t bus)
{
    if (bus >= I2C_DRIVER_BUS_MAX)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_bus_handles[bus] != NULL)
    {
        return ESP_OK;
    }
    if (!i2c_driver_config_is_valid(&s_bus_configs[bus]))
    {
        ESP_LOGW(TAG, "I2C bus %d not configured, skip initialization", (int)s_bus_configs[bus].port);
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_driver_init_bus_internal(bus);
}

esp_err_t i2c_driver_get_bus_handle(i2c_driver_bus_t bus, i2c_master_bus_handle_t *handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (bus >= I2C_DRIVER_BUS_MAX)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_bus_handles[bus] == NULL)
    {
        esp_err_t err = i2c_driver_init_bus(bus);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    *handle = s_bus_handles[bus];
    return ESP_OK;
}
