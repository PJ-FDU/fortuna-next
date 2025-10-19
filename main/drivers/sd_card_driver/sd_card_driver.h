#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "esp_io_expander.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"

#define SD_CARD_CLK_GPIO (GPIO_NUM_14)
#define SD_CARD_CMD_GPIO (GPIO_NUM_17)
#define SD_CARD_D0_GPIO (GPIO_NUM_16)
#define SD_CARD_D1_GPIO (GPIO_NUM_NC)
#define SD_CARD_D2_GPIO (GPIO_NUM_NC)
#define SD_CARD_D3_GPIO (GPIO_NUM_NC)
#define SD_CARD_CS_EXIO (IO_EXPANDER_PIN_NUM_2)
#define SD_CARD_DETECT_GPIO (GPIO_NUM_NC)
#define SD_CARD_WRITE_PROTECT_GPIO (GPIO_NUM_NC)

#define SD_CARD_MAX_FILES (10)
#define SD_CARD_ALLOCATION_UNIT (16 * 1024)
#define SD_CARD_FORMAT_IF_MOUNT_FAILED (false)
#define SD_CARD_DISK_STATUS_CHECK (true)
#define SD_CARD_USE_ONE_FAT (false)

#define SD_CARD_MOUNT_POINT "/sdcard"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t sd_card_driver_init(void);

esp_err_t sd_card_driver_deinit(void);

esp_err_t sd_card_driver_get_card(sdmmc_card_t **card);

bool sd_card_driver_is_mounted(void);

const char *sd_card_driver_mount_point(void);

#ifdef __cplusplus
}
#endif
