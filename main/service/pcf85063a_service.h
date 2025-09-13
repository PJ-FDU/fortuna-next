#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_err.h"

#include "pcf85063a.h"

    extern pcf85063a_dev_t pcf_dev;

    extern pcf85063a_datetime_t pcf_datetime;

    esp_err_t pcf85063a_service_init(void);

#ifdef __cplusplus
}
#endif