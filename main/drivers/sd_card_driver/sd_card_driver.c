#include <stdio.h>

#include "esp_log.h"
#include "esp_vfs_fat.h"

#include "driver/gpio.h"

#include "io_exp_driver.h"
#include "sd_card_driver.h"

static const char *TAG = "sd_card_driver";

static sdmmc_card_t *s_card = NULL;
static bool s_card_mounted = false;

static esp_err_t sd_card_driver_configure_cs_exio(void)
{
    esp_io_expander_handle_t expander = NULL;
    esp_err_t err = io_exp_driver_get_handle(&expander);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get IO expander handle: %s", esp_err_to_name(err));
        return err;
    }
    if (expander == NULL)
    {
        ESP_LOGE(TAG, "IO expander handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    err = esp_io_expander_set_dir(expander, SD_CARD_CS_EXIO, IO_EXPANDER_OUTPUT);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set CS pin direction: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_io_expander_set_level(expander, SD_CARD_CS_EXIO, 1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to deassert CS via IO expander: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Configured SD card CS on IO expander pin %llu", (unsigned long long)SD_CARD_CS_EXIO);
    return ESP_OK;
}

esp_err_t sd_card_driver_init(void)
{
    if (s_card_mounted)
    {
        ESP_LOGW(TAG, "SD card already mounted");
        return ESP_OK;
    }

    esp_err_t err = sd_card_driver_configure_cs_exio();
    if (err != ESP_OK)
    {
        return err;
    }

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = SD_CARD_CLK_GPIO;
    slot_config.cmd = SD_CARD_CMD_GPIO;
    slot_config.d0 = SD_CARD_D0_GPIO;
    slot_config.d1 = SD_CARD_D1_GPIO;
    slot_config.d2 = SD_CARD_D2_GPIO;
    slot_config.d3 = SD_CARD_D3_GPIO;
    slot_config.cd = SD_CARD_DETECT_GPIO;
    slot_config.wp = SD_CARD_WRITE_PROTECT_GPIO;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = SD_CARD_FORMAT_IF_MOUNT_FAILED,
        .max_files = SD_CARD_MAX_FILES,
        .allocation_unit_size = SD_CARD_ALLOCATION_UNIT,
        .disk_status_check_enable = SD_CARD_DISK_STATUS_CHECK,
        .use_one_fat = SD_CARD_USE_ONE_FAT,
    };

    s_card = NULL;
    ESP_LOGI(TAG, "Mounting SD card at %s", SD_CARD_MOUNT_POINT);
    err = esp_vfs_fat_sdmmc_mount(SD_CARD_MOUNT_POINT, &host, &slot_config, &mount_config, &s_card);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(err));
        return err;
    }

    s_card_mounted = true;
    sdmmc_card_print_info(stdout, s_card);
    return ESP_OK;
}

esp_err_t sd_card_driver_deinit(void)
{
    if (!s_card_mounted)
    {
        ESP_LOGW(TAG, "SD card not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = esp_vfs_fat_sdcard_unmount(SD_CARD_MOUNT_POINT, s_card);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(err));
        return err;
    }

    s_card = NULL;
    s_card_mounted = false;

    ESP_LOGI(TAG, "SD card unmounted");
    return ESP_OK;
}

esp_err_t sd_card_driver_get_card(sdmmc_card_t **card)
{
    if (card == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_card_mounted || s_card == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }
    *card = s_card;
    return ESP_OK;
}

bool sd_card_driver_is_mounted(void)
{
    return s_card_mounted;
}

const char *sd_card_driver_mount_point(void)
{
    return SD_CARD_MOUNT_POINT;
}
