

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_err.h"
#include "esp_io_expander.h"

    esp_err_t esp_io_expander_service_init(void);

    esp_err_t esp_io_expander_service_get_handle(esp_io_expander_handle_t *io_expander_handle);

#ifdef __cplusplus
}
#endif