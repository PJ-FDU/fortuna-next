# 设置源文件
set(SRCS
    "fortuna.c"
    "font/bacchus_extrabold_128.c"
    "font/bacchus_extrabold_96.c"
    "font/lv_font_montserrat_96.c"
    "drivers/i2c_driver/i2c_driver.c"
    "drivers/io_exp_driver/io_exp_driver.c"
    "drivers/lcd_disp_driver/lcd_disp_driver.c"
    "drivers/lcd_touch_driver/lcd_touch_driver.c"
    "drivers/lvgl_driver/lvgl_driver.c"
)

# 设置头文件包含目录
set(INCLUDE_DIRS
    "."
    "font"
    "drivers/i2c_driver"
    "drivers/io_exp_driver"
    "drivers/lcd_disp_driver"
    "drivers/lcd_touch_driver"
    "drivers/lvgl_driver"
)

# 注册组件
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES driver esp_psram esp_timer esp_wifi freertos nvs_flash spiffs fatfs wear_levelling esp_http_client
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
