# 设置源文件
set(SRCS
    "fortuna.c"

    # "qmi8658_complete_example.c"
    "font/bacchus_extrabold_128.c"
    "font/bacchus_extrabold_96.c"
    "font/lv_font_montserrat_96.c"

    # LVGL界面
    "lvgl_screens/home.c"

    # UI组件
    "ui/voice_overlay.c"

    "screen/audio_input_mask.c"

    # Service层源文件
    "service/i2c_service.c"
    "service/i2s_service.c"
    "service/io_expander_service.c"
    "service/pcf85063a_service.c"
    "service/qmi8658_service.c"
    "service/lcd_service.c"
    "service/lcd_touch_service.c"
    "service/lvgl_service.c"
    # "service/mic_service.c"
    # "service/wifi_service.c"
    "service/ui_system_service.c"
    # "service/audio_system_service.c"

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
    "font"
    "screen"
    "service"
    # "data"
    # "lvgl_screens"
    # "ui"
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

# 获取 managed component: lvgl__lvgl 的 target
idf_component_get_property(LVGL_LIB lvgl__lvgl COMPONENT_LIB)

# 告诉 LVGL 去哪里找 lv_conf.h
target_compile_definitions(${LVGL_LIB} PUBLIC LV_CONF_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h\")
target_include_directories(${LVGL_LIB} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Suppress -Werror=char-subscripts for the lvgl component so warnings from
# third-party sources don't fail the whole build. This avoids modifying upstream
# source files.
if(TARGET ${LVGL_LIB})
    target_compile_options(${LVGL_LIB} PRIVATE
        -Wno-char-subscripts
        -Wno-error=char-subscripts
    )
endif()

# 设置编译器警告级别
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)
