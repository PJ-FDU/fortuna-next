/*
 * official_libs_example.c
 *
 * 这是一个使用Espressif官方标准库的示例代码，展示如何集成以下功能：
 * - TCA9554 IO扩展器
 * - SPD2010 LCD控制器
 * - SPD2010触摸控制器
 * - LVGL图形界面库
 *
 * 代码创建了一个简单的UI界面，包含一个标签和一个按钮，按下按钮可以控制IO扩展器的输出。
 *
 * 作者: AI助手
 * 日期: 2023-xx-xx
 */

#include <stdio.h>                     // 标准输入输出库
#include "freertos/FreeRTOS.h"         // FreeRTOS实时操作系统
#include "freertos/task.h"             // 任务管理
#include "esp_err.h"                   // ESP错误码
#include "esp_log.h"                   // ESP日志功能
#include "driver/gpio.h"               // GPIO驱动
#include "driver/spi_master.h"         // SPI主机驱动
#include "driver/i2c_master.h"         // I2C主机驱动
#include "esp_io_expander.h"           // IO扩展器抽象层
#include "esp_io_expander_tca9554.h"   // TCA9554 IO扩展器驱动
#include "esp_lcd_panel_io.h"          // LCD面板IO抽象层
#include "esp_lcd_panel_vendor.h"      // LCD面板厂商扩展
#include "esp_lcd_spd2010.h"           // SPD2010 LCD控制器驱动
#include "esp_lcd_touch.h"             // 触摸抽象层
#include "esp_lcd_touch_spd2010.h"     // SPD2010触摸控制器驱动
#include "lvgl.h"                      // LVGL图形库

// 日志标签
static const char *TAG = "OFFICIAL_LIBS_EXAMPLE";

// 硬件引脚定义
#define LCD_SPI_BUS      SPI2_HOST     // LCD使用的SPI总线
#define LCD_PIN_NUM_CS   21            // LCD片选引脚
#define LCD_PIN_NUM_DC   -1            // LCD数据/命令引脚(-1表示未使用)
#define LCD_PIN_NUM_RST  -1            // LCD复位引脚(-1表示未使用)
#define LCD_PIN_NUM_BK   5             // LCD背光控制引脚
#define TOUCH_I2C_BUS    I2C_NUM_0     // 触摸控制器使用的I2C总线
#define TOUCH_PIN_NUM_INT 4            // 触摸中断引脚
#define TOUCH_PIN_NUM_RST -1           // 触摸复位引脚(-1表示未使用)
#define IO_EXPANDER_I2C_BUS I2C_NUM_0  // IO扩展器使用的I2C总线
#define IO_EXPANDER_ADDR 0x38          // IO扩展器I2C地址

// 全局变量
esp_io_expander_handle_t io_expander = NULL;   // IO扩展器句柄
esp_lcd_panel_handle_t lcd_panel = NULL;       // LCD面板句柄
esp_lcd_touch_handle_t touch = NULL;           // 触摸控制器句柄
lv_disp_t *lv_disp = NULL;                     // LVGL显示驱动句柄
lv_indev_t *lv_indev = NULL;                   // LVGL输入设备句柄

/**
 * @brief 初始化TCA9554 IO扩展器
 *
 * 该函数负责初始化TCA9554 IO扩展器，包括创建I2C总线、
 * 初始化IO扩展器以及配置引脚方向和电平。
 *
 * @return
 *  - ESP_OK: 初始化成功
 *  - 其他错误码: 初始化失败
 */
static esp_err_t io_expander_init(void)
{
    ESP_LOGI(TAG, "Initializing TCA9554 IO expander");

    // 创建I2C总线配置
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,  // 使用默认时钟源
        .i2c_port = IO_EXPANDER_I2C_BUS,    // I2C端口号
        .sda_io_num = 8,                    // SDA引脚号
        .scl_io_num = 9,                    // SCL引脚号
        .glitch_ignore_cnt = 7,             // 忽略 glitch 计数
        .flags.enable_internal_pullup = true, // 启用内部上拉电阻
    };
    i2c_master_bus_handle_t i2c_bus = NULL;
    // 创建I2C总线
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));

    // 配置TCA9554 IO扩展器
    esp_io_expander_tca9554_config_t tca9554_config = {
        .i2c_bus = i2c_bus,         // I2C总线句柄
        .i2c_addr = IO_EXPANDER_ADDR, // IO扩展器I2C地址
        .reset_gpio_num = -1,       // 复位引脚(-1表示未使用)
    };
    // 创建IO扩展器实例
    ESP_ERROR_CHECK(esp_io_expander_new_i2c_tca9554(&tca9554_config, &io_expander));

    // 配置IO扩展器引脚（示例：将引脚0设为输出）
    ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander, 0, ESP_IO_EXPANDER_OUTPUT));
    // 设置引脚0输出高电平
    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, 0, 1));

    ESP_LOGI(TAG, "TCA9554 IO expander initialized successfully");
    return ESP_OK;
}

/**
 * @brief 初始化SPD2010 LCD控制器
 *
 * 该函数负责初始化SPD2010 LCD控制器，包括配置SPI总线、
 * 初始化LCD面板IO、配置LCD面板参数以及初始化LCD面板。
 *
 * @return
 *  - ESP_OK: 初始化成功
 *  - 其他错误码: 初始化失败
 */
static esp_err_t lcd_init(void)
{
    ESP_LOGI(TAG, "Initializing SPD2010 LCD");

    // 配置SPI总线
    spi_bus_config_t bus_config = {
        .sclk_io_num = 40,                 // SPI时钟引脚
        .mosi_io_num = 46,                 // MOSI引脚
        .miso_io_num = -1,                 // MISO引脚(-1表示未使用)
        .quadwp_io_num = 45,               // QSPI WP引脚
        .quadhd_io_num = 42,               // QSPI HD引脚
        .data4_io_num = 41,                // 数据引脚4
        .data5_io_num = -1,                // 数据引脚5(-1表示未使用)
        .data6_io_num = -1,                // 数据引脚6(-1表示未使用)
        .data7_io_num = -1,                // 数据引脚7(-1表示未使用)
        .max_transfer_sz = 4096,           // 最大传输大小
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_QUAD, // 主模式 | QSPI模式
    };
    // 初始化SPI总线
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_BUS, &bus_config, SPI_DMA_CH_AUTO));

    // 配置LCD面板IO
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = LCD_PIN_NUM_CS,     // 片选引脚
        .dc_gpio_num = LCD_PIN_NUM_DC,     // 数据/命令引脚
        .spi_mode = 0,                     // SPI模式0
        .pclk_hz = 80 * 1000 * 1000,       // 像素时钟频率(80MHz)
        .trans_queue_depth = 10,           // 传输队列深度
        .lcd_cmd_bits = 32,                // 命令位数
        .lcd_param_bits = 8,               // 参数位数
        .flags = {
            .dc_low_on_data = 0,           // 数据传输时DC引脚为高
            .quad_mode = 1,                // 启用QSPI模式
        },
    };
    esp_lcd_panel_io_handle_t io_handle = NULL;
    // 创建LCD面板IO
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_BUS, &io_config, &io_handle));

    // 配置LCD面板供应商特定参数
    spd2010_vendor_config_t vendor_config = {
        .flags = {
            .use_qspi_interface = 1,       // 使用QSPI接口
        },
    };
    // 配置LCD面板设备参数
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_NUM_RST, // 复位引脚
        .bits_per_pixel = 16,              // 每像素位数
        .vendor_config = &vendor_config,   // 供应商特定配置
    };
    // 创建SPD2010 LCD面板
    ESP_ERROR_CHECK(esp_lcd_new_panel_spd2010(io_handle, &panel_config, &lcd_panel));

    // 初始化LCD面板
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel));     // 复位LCD面板
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel));      // 初始化LCD面板
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel, true)); // 打开LCD显示

    // 配置背光
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,          // 输出模式
        .pin_bit_mask = 1ULL << LCD_PIN_NUM_BK // 背光引脚
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));       // 配置背光引脚
    ESP_ERROR_CHECK(gpio_set_level(LCD_PIN_NUM_BK, 1));  // 打开背光

    ESP_LOGI(TAG, "SPD2010 LCD initialized successfully");
    return ESP_OK;
}

/**
 * @brief 初始化SPD2010触摸控制器
 *
 * 该函数负责初始化SPD2010触摸控制器，包括配置触摸控制器参数
 * 和创建触摸控制器实例。
 *
 * @return
 *  - ESP_OK: 初始化成功
 *  - 其他错误码: 初始化失败
 */
static esp_err_t touch_init(void)
{
    ESP_LOGI(TAG, "Initializing SPD2010 touch");

    // 注意：这里假设触摸控制器与IO扩展器共用I2C总线
    // 如果需要独立的I2C总线，可以参考io_expander_init()函数中的I2C总线初始化

    // 配置触摸控制器
    esp_lcd_touch_spd2010_config_t touch_config = {
        .i2c_bus = IO_EXPANDER_I2C_BUS,  // I2C总线
        .i2c_addr = 0x53,                 // 触摸控制器I2C地址
        .rst_gpio_num = TOUCH_PIN_NUM_RST, // 复位引脚
        .int_gpio_num = TOUCH_PIN_NUM_INT, // 中断引脚
    };
    // 创建SPD2010触摸控制器实例
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_spd2010(&touch_config, &touch));

    ESP_LOGI(TAG, "SPD2010 touch initialized successfully");
    return ESP_OK;
}

/**
 * @brief 初始化LVGL图形库
 *
 * 该函数负责初始化LVGL图形库，包括初始化LVGL、分配显示缓冲区、
 * 注册显示驱动、注册触摸输入设备以及创建LVGL定时器。
 *
 * @return
 *  - ESP_OK: 初始化成功
 *  - 其他错误码: 初始化失败
 */
static esp_err_t lvgl_init(void)
{
    ESP_LOGI(TAG, "Initializing LVGL");

    // 初始化LVGL库
    lv_init();

    // 分配LVGL显示缓冲区（从SPI RAM分配）
    // 缓冲区大小为：宽度(412) * 高度(100) * 像素大小
    lv_color_t *buf1 = heap_caps_malloc(412 * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    lv_color_t *buf2 = heap_caps_malloc(412 * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    lv_disp_draw_buf_t disp_buf;
    // 初始化显示缓冲区
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, 412 * 100);

    // 注册显示驱动
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = 412;              // 水平分辨率
    disp_drv.ver_res = 412;              // 垂直分辨率
    // 刷新回调函数
    disp_drv.flush_cb = [](lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
        esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)drv->user_data;
        // 在LCD上绘制位图
        esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
        // 通知LVGL刷新完成
        lv_disp_flush_ready(drv);
    };
    disp_drv.draw_buf = &disp_buf;       // 显示缓冲区
    disp_drv.user_data = lcd_panel;      // 用户数据（LCD面板句柄）
    // 注册显示驱动
    lv_disp = lv_disp_drv_register(&disp_drv);

    // 注册触摸输入设备
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER; // 输入设备类型（指针）
    // 读取回调函数
    indev_drv.read_cb = [](lv_indev_drv_t *drv, lv_indev_data_t *data) {
        esp_lcd_touch_handle_t touch = (esp_lcd_touch_handle_t)drv->user_data;
        uint16_t x, y;
        bool touched;
        // 读取触摸数据
        esp_lcd_touch_read_data(touch);
        // 获取触摸坐标
        touched = esp_lcd_touch_get_coordinates(touch, &x, &y, NULL);
        // 设置输入数据
        data->point.x = x;
        data->point.y = y;
        data->state = touched ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    };
    indev_drv.user_data = touch;         // 用户数据（触摸控制器句柄）
    // 注册输入设备
    lv_indev = lv_indev_drv_register(&indev_drv);

    // 创建LVGL定时器（用于提供LVGL所需的tick）
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = [](void *arg) {
            // 增加LVGL的tick计数（每2ms增加2）
            lv_tick_inc(2);
        },
        .name = "lvgl_tick"               // 定时器名称
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    // 创建定时器
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    // 启动定时器（周期为2000us，即2ms）
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 2000));

    ESP_LOGI(TAG, "LVGL initialized successfully");
    return ESP_OK;
}

/**
 * @brief 创建简单的UI界面
 *
 * 该函数负责创建一个简单的UI界面，包括一个标签和一个按钮。
 * 按钮按下时会切换IO扩展器引脚0的输出状态。
 */
static void create_ui(void)
{
    // 创建一个标签
    lv_obj_t *label = lv_label_create(lv_scr_act()); // 在当前活动屏幕上创建标签
    lv_label_set_text(label, "Official Libraries Demo"); // 设置标签文本
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 20);     // 对齐到屏幕顶部中间，y偏移20像素

    // 创建一个按钮
    lv_obj_t *btn = lv_btn_create(lv_scr_act());      // 在当前活动屏幕上创建按钮
    lv_obj_set_size(btn, 120, 50);                    // 设置按钮大小(宽度120，高度50)
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);         // 对齐到屏幕中心

    // 按钮标签
    lv_obj_t *btn_label = lv_label_create(btn);       // 在按钮上创建标签
    lv_label_set_text(btn_label, "Press Me");        // 设置按钮标签文本
    lv_obj_center(btn_label);                         // 将标签居中显示在按钮上

    // 按钮回调函数（点击事件）
    lv_obj_add_event_cb(btn, [](lv_event_t *e) {
        static bool led_state = false;                // 静态变量，保存LED状态
        led_state = !led_state;                       // 切换LED状态
        // 控制IO扩展器输出
        esp_io_expander_set_level(io_expander, 0, led_state);
        ESP_LOGI(TAG, "Button pressed, LED state: %s", led_state ? "ON" : "OFF");
    }, LV_EVENT_CLICKED, NULL);
}

/**
 * @brief 应用程序入口函数
 *
 * 该函数是应用程序的入口点，负责初始化各组件、创建UI界面
 * 以及启动主循环。
 */
void app_main(void)
{
    // 初始化各组件
    ESP_ERROR_CHECK(io_expander_init());  // 初始化IO扩展器
    ESP_ERROR_CHECK(lcd_init());          // 初始化LCD
    ESP_ERROR_CHECK(touch_init());        // 初始化触摸控制器
    ESP_ERROR_CHECK(lvgl_init());         // 初始化LVGL

    // 创建UI界面
    create_ui();

    // 主循环
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));    // 延时10ms
        lv_timer_handler();               // 处理LVGL定时器事件
    }
}