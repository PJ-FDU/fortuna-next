#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifndef I2C_BUS0_SCL_GPIO
#define I2C_BUS0_SCL_GPIO GPIO_NUM_10
#endif

#ifndef I2C_BUS0_SDA_GPIO
#define I2C_BUS0_SDA_GPIO GPIO_NUM_11
#endif

#ifndef I2C_BUS0_CLK_SRC
#define I2C_BUS0_CLK_SRC I2C_CLK_SRC_DEFAULT
#endif

#ifndef I2C_BUS0_CLK_HZ
#define I2C_BUS0_CLK_HZ 400000
#endif

#ifndef I2C_BUS0_GLITCH_IGNORE_CNT
#define I2C_BUS0_GLITCH_IGNORE_CNT 7
#endif

#ifndef I2C_BUS0_ENABLE_INTERNAL_PULLUP
#define I2C_BUS0_ENABLE_INTERNAL_PULLUP 1
#endif

#ifndef I2C_BUS1_SCL_GPIO
#define I2C_BUS1_SCL_GPIO GPIO_NUM_NC
#endif

#ifndef I2C_BUS1_SDA_GPIO
#define I2C_BUS1_SDA_GPIO GPIO_NUM_NC
#endif

#ifndef I2C_BUS1_CLK_SRC
#define I2C_BUS1_CLK_SRC I2C_CLK_SRC_DEFAULT
#endif

#ifndef I2C_BUS1_CLK_HZ
#define I2C_BUS1_CLK_HZ 400000
#endif

#ifndef I2C_BUS1_GLITCH_IGNORE_CNT
#define I2C_BUS1_GLITCH_IGNORE_CNT 7
#endif

#ifndef I2C_BUS1_ENABLE_INTERNAL_PULLUP
#define I2C_BUS1_ENABLE_INTERNAL_PULLUP 1
#endif

typedef enum
{
    I2C_DRIVER_BUS_MAIN = 0,
    I2C_DRIVER_BUS_AUX,
    I2C_DRIVER_BUS_MAX
} i2c_driver_bus_t;

#ifdef __cplusplus
extern "C" {
#endif

    esp_err_t i2c_driver_init(void);

    esp_err_t i2c_driver_init_bus(i2c_driver_bus_t bus);

    esp_err_t i2c_driver_get_bus_handle(i2c_driver_bus_t bus, i2c_master_bus_handle_t *handle);

#ifdef __cplusplus
}
#endif
