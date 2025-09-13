#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#define PIN_I2C_PORT I2C_NUM_0
#define PIN_I2C_SCL GPIO_NUM_10
#define PIN_I2C_SDA GPIO_NUM_11

#define I2C_SPEED_HZ 400000

    esp_err_t esp_i2c_service_init(void);

#ifdef __cplusplus
}
#endif