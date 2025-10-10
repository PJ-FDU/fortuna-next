#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#define I2C_SCL_GPIO GPIO_NUM_10
#define I2C_SDA_GPIO GPIO_NUM_11
#define I2C_CLK_SRC I2C_CLK_SRC_DEFAULT
#define I2C_CLK_HZ 400000
#define I2C_GLITCH_IGNORE_CNT 7

#ifdef __cplusplus
extern "C" {
#endif

    esp_err_t i2c_driver_init(void);

    esp_err_t i2c_driver_get_bus_handle(i2c_master_bus_handle_t *handle);

#ifdef __cplusplus
}
#endif
