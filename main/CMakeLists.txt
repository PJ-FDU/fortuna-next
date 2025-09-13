# 设置源文件
set(SRCS
    "fortuna.c"
    # "qmi8658_complete_example.c"

    # Service层源文件
    "service/i2c_service.c"
    "service/io_expander_service.c"
    "service/pcf85063a_service.c"
    "service/qmi8658_service.c"
    "service/lcd_service.c"
    "service/lcd_touch_service.c"
    "service/lvgl_service.c"

    # Data层源文件
    # "data/data_manager.c"
    # "data/config_data.c"
    # "data/user_data.c"

    # LVGL界面源文件
    # "lvgl_screens/screen_home.c"
    # "lvgl_screens/screen_settings.c"
)

# 设置头文件包含目录
set(INCLUDE_DIRS
    "."
    "service"
    "data"
    "lvgl_screens"
)

# 注册组件
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES driver esp_timer freertos esp_psram
    PRIV_REQUIRES
)

# 设置编译器警告级别
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)
