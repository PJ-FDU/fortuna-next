# 设置源文件
set(SRCS
    "fortuna.c"

    # "qmi8658_complete_example.c"

    # LVGL界面
    "lvgl_screens/home.c"

    # UI组件
    "ui/voice_overlay.c"

    # Service层源文件
    "service/i2c_service.c"
    "service/i2s_service.c"
    "service/io_expander_service.c"
    "service/pcf85063a_service.c"
    "service/qmi8658_service.c"
    "service/lcd_service.c"
    "service/lcd_touch_service.c"
    "service/lvgl_service.c"
    "service/mic_service.c"
    "service/wifi_service.c"
    "service/ui_system_service.c"
    "service/audio_system_service.c"

    # Data层源文件
    # "data/data_manager.c"
    # "data/config_data.c"
    # "data/user_data.c"

    # LVGL界面源文件
    # 旧: "lvgl_screens/screen_home.c"
    # 旧: "lvgl_screens/screen_settings.c"
)

# 设置头文件包含目录
set(INCLUDE_DIRS
    "."
    "service"
    "data"
    "lvgl_screens"
    "ui"
)

# 注册组件
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES driver esp_psram esp_timer esp_wifi freertos nvs_flash spiffs fatfs wear_levelling
    PRIV_REQUIRES
)

# SPIFFS 数据上传配置
spiffs_create_partition_image(spiffs ../spiffs_data FLASH_IN_PROJECT)

# 设置编译器警告级别
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)
