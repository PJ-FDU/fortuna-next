#include "esp_err.h"
#include "esp_log.h"

#include "pcf85063a.h"

#include "i2c_service.h"
#include "pcf85063a_service.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

#define TAG "PCF85063A_SERVICE"
static pcf85063a_dev_t s_pcf_dev;
static pcf85063a_datetime_t s_pcf_datetime;
static SemaphoreHandle_t s_pcf_mutex = NULL;
static bool s_initialized = false;

/* Mutex take timeouts (ms) used by public APIs. Tweak as necessary. */
#define PCF85063A_SVC_MUTEX_TAKE_MS_READ  (100)
#define PCF85063A_SVC_MUTEX_TAKE_MS_WRITE (500)

esp_err_t pcf85063a_service_init(void)
{
    esp_err_t err = ESP_OK;
    i2c_master_bus_handle_t i2c_master_bus_handle = NULL;

    /* If already initialized, treat init as idempotent and return success. */
    if (s_initialized) {
        ESP_LOGW(TAG, "pcf85063a_service_init called but service already initialized");
        return ESP_OK;
    }

    // 获取 I2C 总线句柄（i2c_service 应提供安全 getter）
    err = i2c_master_get_bus_handle(PIN_I2C_PORT, &i2c_master_bus_handle);
    ESP_ERROR_CHECK(err);

    // 初始化 PCF85063A 设备结构
    err = pcf85063a_init(&s_pcf_dev, i2c_master_bus_handle, PCF85063A_ADDRESS);
    ESP_ERROR_CHECK(err);

    // 初始化互斥（保护缓存的日期/时间结构）
    s_pcf_mutex = xSemaphoreCreateMutex();
    if (s_pcf_mutex == NULL) {
        ESP_LOGE(TAG, "failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // 检查 RTC 是否已有有效时间（优先读取设备时间）
    pcf85063a_datetime_t read_dt = {0};
    err = pcf85063a_get_time_date(&s_pcf_dev, &read_dt);
    if (err == ESP_OK) {
        // 简单有效性校验：年份范围合理
        if (read_dt.year >= 2020 && read_dt.year <= 2099) {
            xSemaphoreTake(s_pcf_mutex, portMAX_DELAY);
            s_pcf_datetime = read_dt;
            xSemaphoreGive(s_pcf_mutex);
            ESP_LOGI(TAG, "PCF85063A RTC has valid time: %04d-%02d-%02d %02d:%02d:%02d",
                     read_dt.year, read_dt.month, read_dt.day, read_dt.hour, read_dt.min, read_dt.sec);
        } else {
            ESP_LOGW(TAG, "PCF85063A RTC returned out-of-range year (%d), will set default time", read_dt.year);
            // write default below
            read_dt.year = 0; // mark invalid
        }
    } else {
        ESP_LOGW(TAG, "pcf85063a_get_time_date failed (%d), will set default time", err);
    }

    if (read_dt.year == 0) {
        // 设置默认时间
        pcf85063a_datetime_t init_time = {
            .year = 2025,
            .month = 9,
            .day = 13,
            .dotw = 6,
            .hour = 12,
            .min = 0,
            .sec = 0
        };
        esp_err_t set_err = pcf85063a_set_time_date(&s_pcf_dev, init_time);
        if (set_err != ESP_OK) {
            ESP_LOGW(TAG, "pcf85063a_set_time_date failed (%d)", set_err);
        } else {
            xSemaphoreTake(s_pcf_mutex, portMAX_DELAY);
            s_pcf_datetime = init_time;
            xSemaphoreGive(s_pcf_mutex);
        }
    }

    s_initialized = true;
    ESP_LOGI(TAG, "PCF85063A initialized successfully");
    return ESP_OK;
}

/*
 * Internal helpers - keep these static so the module doesn't export extra symbols.
 * Public API surface is defined in pcf85063a_service.h (init/read/write/teardown).
 */
static esp_err_t pcf85063a_service_get_handle(pcf85063a_dev_t **out_dev)
{
    if (out_dev == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    *out_dev = &s_pcf_dev;
    return ESP_OK;
}

static esp_err_t pcf85063a_service_get_datetime(pcf85063a_datetime_t *out_dt)
{
    if (out_dt == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (s_pcf_mutex == NULL) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_pcf_mutex, pdMS_TO_TICKS(PCF85063A_SVC_MUTEX_TAKE_MS_READ)) != pdTRUE) return ESP_ERR_TIMEOUT;
    *out_dt = s_pcf_datetime;
    xSemaphoreGive(s_pcf_mutex);
    return ESP_OK;
}

static esp_err_t pcf85063a_service_set_datetime(const pcf85063a_datetime_t *in_dt)
{
    if (in_dt == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (s_pcf_mutex == NULL) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_pcf_mutex, pdMS_TO_TICKS(PCF85063A_SVC_MUTEX_TAKE_MS_WRITE)) != pdTRUE) return ESP_ERR_TIMEOUT;

    esp_err_t err = pcf85063a_set_time_date(&s_pcf_dev, *in_dt);
    if (err == ESP_OK) {
        s_pcf_datetime = *in_dt;
    }

    xSemaphoreGive(s_pcf_mutex);
    return err;
}

/* Public API wrappers (keeps the header small and stable) */
esp_err_t pcf85063a_service_read_datetime(pcf85063a_datetime_t *out_dt)
{
    return pcf85063a_service_get_datetime(out_dt);
}

esp_err_t pcf85063a_service_write_datetime(const pcf85063a_datetime_t *in_dt)
{
    return pcf85063a_service_set_datetime(in_dt);
}

esp_err_t pcf85063a_service_teardown(void)
{
    if (!s_initialized) return ESP_OK;
    // 尝试停用并释放资源
    if (s_pcf_mutex) {
        vSemaphoreDelete(s_pcf_mutex);
        s_pcf_mutex = NULL;
    }
    // 如果底层驱动提供 deinit 函数，调用它
#ifdef PCF85063A_DEINIT_AVAILABLE
    pcf85063a_deinit(&s_pcf_dev);
#endif
    s_initialized = false;
    ESP_LOGI(TAG, "PCF85063A service torn down");
    return ESP_OK;
}