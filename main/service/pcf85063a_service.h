#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_err.h"

#include "pcf85063a.h"

/*
 * Initialize the PCF85063A service.
 * - Obtains the I2C bus handle and initializes the underlying driver.
 * - Attempts to read the RTC time; if the device returns an invalid time,
 *   a reasonable default time is written.
 * - This function is idempotent: calling it when the service is already
 *   initialized will return ESP_OK and do nothing.
 */
esp_err_t pcf85063a_service_init(void);

/*
 * Read-only access to RTC time (thread-safe).
 * - Reads the cached datetime filled during init or after a successful write.
 * - This is intentionally a cached read to avoid doing I2C on every call.
 * - Returns ESP_ERR_INVALID_STATE if the service is not initialized.
 */
esp_err_t pcf85063a_service_read_datetime(pcf85063a_datetime_t *out_dt);

/*
 * Write time to RTC (thread-safe).
 * - Writes the provided datetime to the device and updates the internal cache
 *   on success.
 * - The write is serialized with the read cache via an internal mutex.
 */
esp_err_t pcf85063a_service_write_datetime(const pcf85063a_datetime_t *in_dt);

/*
 * Teardown service: deinit device and free resources.
 * - Safe to call multiple times; subsequent calls are no-ops.
 */
esp_err_t pcf85063a_service_teardown(void);

#ifdef __cplusplus
}
#endif