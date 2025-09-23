#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#define I2C_PORT_NUM I2C_NUM_0
#define I2C_SCL_GPIO GPIO_NUM_10
#define I2C_SDA_GPIO GPIO_NUM_11
#define I2C_CLK_SRC I2C_CLK_SRC_DEFAULT
#define I2C_CLK_HZ 400000
#define I2C_GLITCH_IGNORE_CNT 7

    esp_err_t esp_i2c_service_init(void);

#ifdef __cplusplus
}
#endif