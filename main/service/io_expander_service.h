#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_err.h"
#include "esp_io_expander.h"

    // 全局变量声明 - 其他文件可以访问
    extern esp_io_expander_handle_t io_expander_handle;

    esp_err_t esp_io_expander_service_init(void);

#ifdef __cplusplus
}
#endif