#pragma once

#include "esp_err.h"
#include "esp_io_expander.h"

#ifdef __cplusplus
extern "C" {
#endif

    esp_err_t io_exp_driver_init(void);
    esp_err_t io_exp_driver_get_handle(esp_io_expander_handle_t *handle);

#ifdef __cplusplus
}
#endif