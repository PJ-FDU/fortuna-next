// main.c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "esp_io_expander.h"
#include "esp_io_expander_tca9554.h"

// waveshare qmi8658 组件头
#include "qmi8658.h" // 来自 waveshare/qmi8658 组件

static const char *TAG = "QMI8658_TCA";

// ====== 你的硬件连接 ======
#define I2C_PORT I2C_NUM_0
#define PIN_I2C_SCL 10
#define PIN_I2C_SDA 11
#define I2C_SPEED_HZ 400000

// TCA9554：A2..A0 见实物焊接，常见 0x20～0x27；这里先用 0x20
#define TCA9554_ADDR 0x20
// IMU_INT2→Extend_IO4→P3，IMU_INT1→Extend_IO5→P4
#define TCA_PIN_IMU_INT2 3 // P3
#define TCA_PIN_IMU_INT1 4 // P4

// ====== 句柄 ======
static i2c_master_bus_handle_t s_bus = NULL;
static esp_io_expander_handle_t s_tca = NULL;
static qmi8658_dev_t s_qmi;   // waveshare 定义的设备句柄
static qmi8658_data_t s_data; // waveshare 定义的数据结构

// ====== I2C 新驱动：创建总线 ======
static void i2c_bus_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {.enable_internal_pullup = true}, // 板上最好仍放 2.2k~4.7k 外部上拉
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &s_bus));
    ESP_LOGI(TAG, "I2C bus ready (SCL=%d, SDA=%d)", PIN_I2C_SCL, PIN_I2C_SDA);
}

// ====== TCA9554 初始化：P3/P4 设为输入，轮询中断电平 ======
static void tca9554_init(void)
{
    esp_io_expander_tca9554_config_t cfg = {
        .i2c_bus = s_bus,
        .i2c_address = TCA9554_ADDR,
    };
    ESP_ERROR_CHECK(esp_io_expander_new_i2c_tca9554(&cfg, &s_tca));
    ESP_ERROR_CHECK(esp_io_expander_set_dir(s_tca, TCA_PIN_IMU_INT1, ESP_IO_EXPANDER_GPIO_DIR_INPUT));
    ESP_ERROR_CHECK(esp_io_expander_set_dir(s_tca, TCA_PIN_IMU_INT2, ESP_IO_EXPANDER_GPIO_DIR_INPUT));
    ESP_LOGI(TAG, "TCA9554 ready @0x%02X", TCA9554_ADDR);
}

static inline int tca_get_level(int pin)
{
    int level = 1;
    esp_io_expander_get_level(s_tca, pin, &level);
    return level; // 1=上拉空闲，0=被拉低（多为中断触发）
}

// ====== QMI8658 初始化（waveshare API） ======
static void qmi8658_init_and_config(void)
{
    // waveshare 提供了地址枚举：QMI8658_ADDRESS_LOW(0x6A)/HIGH(0x6B)；先试 0x6A，不行再试 0x6B
    esp_err_t ret = qmi8658_init(&s_qmi, s_bus, QMI8658_ADDRESS_LOW);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "QMI8658 @0x6A init fail (%d), try 0x6B", ret);
        ESP_ERROR_CHECK(qmi8658_init(&s_qmi, s_bus, QMI8658_ADDRESS_HIGH));
    }

    // 量程 / ODR（可按你之前代码的习惯调整）
    ESP_ERROR_CHECK(qmi8658_set_accel_range(&s_qmi, QMI8658_ACCEL_RANGE_8G));
    ESP_ERROR_CHECK(qmi8658_set_accel_odr(&s_qmi, QMI8658_ACCEL_ODR_250HZ));

    ESP_ERROR_CHECK(qmi8658_set_gyro_range(&s_qmi, QMI8658_GYRO_RANGE_512DPS));
    ESP_ERROR_CHECK(qmi8658_set_gyro_odr(&s_qmi, QMI8658_GYRO_ODR_250HZ));

    // 单位与显示精度：若想与之前“g/dps”一致，把下面两个 false（默认示例是 m/s^2 & rad/s）
    ESP_ERROR_CHECK(qmi8658_set_accel_unit_mps2(&s_qmi, false)); // false => g
    ESP_ERROR_CHECK(qmi8658_set_gyro_unit_rads(&s_qmi, false));  // false => dps
    ESP_ERROR_CHECK(qmi8658_set_display_precision(&s_qmi, 4));

    ESP_LOGI(TAG, "QMI8658 configured");
}

// ====== 任务：基于 INT 低跳沿触发读取 + 兜底定时 ======
static void imu_task(void *arg)
{
    int last1 = 1, last2 = 1;
    int64_t t_prev = esp_timer_get_time();

    while (1)
    {
        int v1 = tca_get_level(TCA_PIN_IMU_INT1);
        int v2 = tca_get_level(TCA_PIN_IMU_INT2);
        bool edge = ((last1 == 1 && v1 == 0) || (last2 == 1 && v2 == 0)); // 低跳沿判定
        last1 = v1;
        last2 = v2;

        // 兜底：即便没有中断，也定期检查 DATA_READY
        bool do_read = edge;
        if (!do_read)
        {
            bool ready = false;
            if (qmi8658_is_data_ready(&s_qmi, &ready) == ESP_OK && ready)
                do_read = true;
        }

        if (do_read)
        {
            if (qmi8658_read_sensor_data(&s_qmi, &s_data) == ESP_OK)
            {
                // 单位：上面配置为 g / dps；若改为 m/s^2、rad/s，请相应修改打印
                ESP_LOGI(TAG, "Acc[g]=%.4f %.4f %.4f  Gyro[dps]=%.4f %.4f %.4f  Temp=%.2f  Ts=%lu",
                         s_data.accelX, s_data.accelY, s_data.accelZ,
                         s_data.gyroX, s_data.gyroY, s_data.gyroZ,
                         s_data.temperature, (unsigned long)s_data.timestamp);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // 5 ms 轮询粒度（若把 TCA 的 INT 接到 MCU，可用中断替代）
    }
}

void app_main(void)
{
    i2c_bus_init();
    tca9554_init();
    qmi8658_init_and_config();
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}
