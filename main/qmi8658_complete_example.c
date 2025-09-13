#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

static const char *TAG = "QMI8658_Example";

// QMI8658 寄存器地址定义
#define QMI8658_I2C_ADDR            0x6B
#define QMI8658_WHO_AM_I            0x00
#define QMI8658_REVISION_ID         0x01
#define QMI8658_CTRL1               0x02
#define QMI8658_CTRL2               0x03
#define QMI8658_CTRL3               0x04
#define QMI8658_CTRL4               0x05
#define QMI8658_CTRL5               0x06
#define QMI8658_CTRL6               0x07
#define QMI8658_CTRL7               0x08
#define QMI8658_CTRL8               0x09
#define QMI8658_CTRL9               0x0A
#define QMI8658_STATUSINT           0x2D
#define QMI8658_STATUS0             0x2E
#define QMI8658_STATUS1             0x2F
#define QMI8658_TIMESTAMP_LOW       0x30
#define QMI8658_TIMESTAMP_MID       0x31
#define QMI8658_TIMESTAMP_HIGH      0x32
#define QMI8658_TEMP_L              0x33
#define QMI8658_TEMP_H              0x34
#define QMI8658_AX_L                0x35
#define QMI8658_AX_H                0x36
#define QMI8658_AY_L                0x37
#define QMI8658_AY_H                0x38
#define QMI8658_AZ_L                0x39
#define QMI8658_AZ_H                0x3A
#define QMI8658_GX_L                0x3B
#define QMI8658_GX_H                0x3C
#define QMI8658_GY_L                0x3D
#define QMI8658_GY_H                0x3E
#define QMI8658_GZ_L                0x3F
#define QMI8658_GZ_H                0x40

// I2C配置参数（匹配硬件规格）
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           11      // 根据I2C_Driver.h配置
#define I2C_MASTER_SCL_IO           10      // 根据I2C_Driver.h配置
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// 数学常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 数据结构定义
typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

typedef struct {
    float roll;    // 横滚角 (rad)
    float pitch;   // 俯仰角 (rad)  
    float yaw;     // 航向角 (rad)
} euler_t;

typedef struct {
    vector3_t accel;      // 加速度 (m/s^2)
    vector3_t gyro;       // 角速度 (rad/s)
    float temperature;    // 温度 (°C)
    euler_t orientation;  // 姿态角 (rad)
    vector3_t gravity;    // 重力向量
} imu_data_t;

// 简化版Kalman滤波器状态
typedef struct {
    float roll;
    float pitch;
    float roll_bias;
    float pitch_bias;
    float P[4][4];  // 误差协方差矩阵
    float Q;        // 过程噪声
    float R;        // 测量噪声
} kalman_filter_t;

// Madgwick滤波器状态
typedef struct {
    float q0, q1, q2, q3;  // 四元数
    float beta;            // 算法增益参数
} madgwick_filter_t;

// 全局变量
static kalman_filter_t kf = {0};
static madgwick_filter_t madgwick = {1.0f, 0.0f, 0.0f, 0.0f, 0.1f};
static SemaphoreHandle_t i2c_mutex = NULL;

// 传感器配置变量（与原代码保持一致）
static float accel_scale_factor = 4.0f / 32768.0f;  // ±4G量程
static float gyro_scale_factor = 64.0f / 32768.0f;  // ±64dps量程
static uint8_t qmi8658_device_addr = QMI8658_I2C_ADDR;  // 动态设备地址

// ======================== 函数声明 ========================
static esp_err_t qmi8658_read_accelerometer(vector3_t *accel);
static esp_err_t qmi8658_read_gyroscope(vector3_t *gyro);
static esp_err_t qmi8658_read_temperature(float *temperature);

// ======================== I2C通信函数 ========================

static esp_err_t i2c_master_write_slave(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t data_len)
{
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take I2C mutex for write");
        return ESP_ERR_TIMEOUT;
    }
    
    esp_err_t ret;
    if (data_len > 0) {
        // 使用与原始驱动相同的函数
        uint8_t buf[data_len + 1];
        buf[0] = reg_addr;
        memcpy(&buf[1], data, data_len);
        ret = i2c_master_write_to_device(I2C_MASTER_NUM, device_addr, buf, data_len + 1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    } else {
        // 只写寄存器地址
        ret = i2c_master_write_to_device(I2C_MASTER_NUM, device_addr, &reg_addr, 1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: device=0x%02X, reg=0x%02X, error=%s", 
                 device_addr, reg_addr, esp_err_to_name(ret));
    }
    
    xSemaphoreGive(i2c_mutex);
    return ret;
}

static esp_err_t i2c_master_read_slave(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t data_len)
{
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take I2C mutex for read");
        return ESP_ERR_TIMEOUT;
    }
    
    // 使用与原始驱动相同的函数
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, device_addr, &reg_addr, 1, data, data_len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: device=0x%02X, reg=0x%02X, len=%d, error=%s", 
                 device_addr, reg_addr, data_len, esp_err_to_name(ret));
    }
    
    xSemaphoreGive(i2c_mutex);
    return ret;
}

// ======================== 初始化函数 ========================

static esp_err_t i2c_master_init(void)
{
    ESP_LOGI(TAG, "Initializing I2C master on port %d", I2C_MASTER_NUM);
    ESP_LOGI(TAG, "SDA GPIO: %d, SCL GPIO: %d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

// I2C总线扫描函数
static void i2c_scan_bus(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    uint8_t devices_found = 0;
    
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, addr, NULL, 0, pdMS_TO_TICKS(50));
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
            devices_found++;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    if (devices_found == 0) {
        ESP_LOGW(TAG, "No I2C devices found on the bus");
        ESP_LOGW(TAG, "Check connections: SDA=GPIO%d, SCL=GPIO%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    } else {
        ESP_LOGI(TAG, "Found %d I2C device(s)", devices_found);
    }
}

static esp_err_t qmi8658_init(void)
{
    uint8_t who_am_i, revision_id;
    esp_err_t ret;
    uint8_t device_addr = QMI8658_I2C_ADDR;
    
    ESP_LOGI(TAG, "Initializing QMI8658...");
    ESP_LOGI(TAG, "Using I2C address: 0x%02X", device_addr);
    
    // 首先扫描I2C总线
    i2c_scan_bus();
    
    // 尝试读取WHO_AM_I寄存器验证设备
    ret = i2c_master_read_slave(device_addr, QMI8658_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I with addr 0x%02X: %s", device_addr, esp_err_to_name(ret));
        
        // 尝试另一个可能的地址
        device_addr = 0x6A; // QMI8658_H_SLAVE_ADDRESS from original code
        ESP_LOGI(TAG, "Trying alternative address: 0x%02X", device_addr);
        ret = i2c_master_read_slave(device_addr, QMI8658_WHO_AM_I, &who_am_i, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read WHO_AM_I with addr 0x%02X: %s", device_addr, esp_err_to_name(ret));
            return ret;
        }
    }
    
    // 读取REVISION_ID寄存器
    ret = i2c_master_read_slave(device_addr, QMI8658_REVISION_ID, &revision_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read REVISION_ID: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Device found! WHO_AM_I: 0x%02X, REVISION_ID: 0x%02X", who_am_i, revision_id);
    ESP_LOGI(TAG, "Using I2C address: 0x%02X", device_addr);
    
    // 更新全局设备地址变量
    qmi8658_device_addr = device_addr;
    
    // 参考原代码的初始化序列
    // 1. 设置CTRL1：启用自动地址递增，禁用低功耗振荡器
    uint8_t ctrl1_val = 0x40; // 启用自动地址递增
    ret = i2c_master_write_slave(device_addr, QMI8658_CTRL1, &ctrl1_val, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL1: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 2. 配置加速度计：±4G量程，8000Hz ODR（参考原代码默认值）
    uint8_t accel_config = 0x10; // ±4G量程 (ACC_RANGE_4G << 4) + 8000Hz ODR
    ret = i2c_master_write_slave(device_addr, QMI8658_CTRL2, &accel_config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 3. 配置陀螺仪：±64dps量程，8000Hz ODR（参考原代码默认值）
    uint8_t gyro_config = 0x20; // ±64dps量程 (GYR_RANGE_64DPS << 4) + 8000Hz ODR
    ret = i2c_master_write_slave(device_addr, QMI8658_CTRL3, &gyro_config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 4. 配置低通滤波器CTRL5：加速度计LPF_MODE_0，陀螺仪LPF_MODE_3
    uint8_t lpf_config = 0x61; // 陀螺仪LPF使能(0x10) + LPF_MODE_3(0x60) + 加速度计LPF使能(0x01)
    ret = i2c_master_write_slave(device_addr, QMI8658_CTRL5, &lpf_config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LPF: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 5. 禁用AttitudeEngine（CTRL6）
    uint8_t ctrl6_val = 0x00;
    ret = i2c_master_write_slave(device_addr, QMI8658_CTRL6, &ctrl6_val, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL6: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 6. 使能传感器：高速内部时钟，加速度计和陀螺仪全功率模式
    uint8_t enable = 0x43; // 高速时钟 + 加速度计和陀螺仪使能
    ret = i2c_master_write_slave(device_addr, QMI8658_CTRL7, &enable, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable sensors: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // 等待传感器稳定
    
    ESP_LOGI(TAG, "QMI8658 initialized successfully");
    ESP_LOGI(TAG, "Accelerometer: ±4G, 8000Hz ODR (scale factor: %.6f)", accel_scale_factor);
    ESP_LOGI(TAG, "Gyroscope: ±64dps, 8000Hz ODR (scale factor: %.6f)", gyro_scale_factor);
    
    return ESP_OK;
}

// 测试传感器连接和数据读取
static esp_err_t qmi8658_test_connection(void)
{
    ESP_LOGI(TAG, "Testing QMI8658 connection...");
    
    // 测试读取几个样本数据
    for (int i = 0; i < 5; i++) {
        vector3_t accel, gyro;
        float temperature;
        
        esp_err_t ret = qmi8658_read_accelerometer(&accel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Accelerometer test failed: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = qmi8658_read_gyroscope(&gyro);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Gyroscope test failed: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = qmi8658_read_temperature(&temperature);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Temperature test failed: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ESP_LOGI(TAG, "Sample %d - Accel: (%.3f, %.3f, %.3f) m/s², Gyro: (%.3f, %.3f, %.3f) rad/s, Temp: %.1f°C",
                 i + 1, accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, temperature);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "QMI8658 connection test passed!");
    return ESP_OK;
}

// ======================== 数据读取函数 ========================

// 读取加速度数据（模拟原代码的getAccelerometer函数）
static esp_err_t qmi8658_read_accelerometer(vector3_t *accel)
{
    uint8_t buf[6];
    esp_err_t ret = i2c_master_read_slave(qmi8658_device_addr, QMI8658_AX_L, buf, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 解析16位有符号数据（小端字节序）
    int16_t accel_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t accel_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t accel_z = (int16_t)((buf[5] << 8) | buf[4]);
    
    // 转换为实际物理值（g单位，然后转换为m/s²）
    accel->x = accel_x * accel_scale_factor * 9.80665f;
    accel->y = accel_y * accel_scale_factor * 9.80665f;
    accel->z = accel_z * accel_scale_factor * 9.80665f;
    
    return ESP_OK;
}

// 读取陀螺仪数据（模拟原代码的getGyroscope函数）
static esp_err_t qmi8658_read_gyroscope(vector3_t *gyro)
{
    uint8_t buf[6];
    esp_err_t ret = i2c_master_read_slave(qmi8658_device_addr, QMI8658_GX_L, buf, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 解析16位有符号数据（小端字节序）
    int16_t gyro_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t gyro_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t gyro_z = (int16_t)((buf[5] << 8) | buf[4]);
    
    // 转换为实际物理值（dps单位，然后转换为rad/s）
    gyro->x = gyro_x * gyro_scale_factor * M_PI / 180.0f;
    gyro->y = gyro_y * gyro_scale_factor * M_PI / 180.0f;
    gyro->z = gyro_z * gyro_scale_factor * M_PI / 180.0f;
    
    return ESP_OK;
}

// 读取温度数据
static esp_err_t qmi8658_read_temperature(float *temperature)
{
    uint8_t buf[2];
    esp_err_t ret = i2c_master_read_slave(qmi8658_device_addr, QMI8658_TEMP_L, buf, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 解析温度数据 (16位，LSB = 1/256°C)
    int16_t temp_raw = (int16_t)((buf[1] << 8) | buf[0]);
    *temperature = temp_raw / 256.0f;
    
    return ESP_OK;
}

static esp_err_t qmi8658_read_raw_data(imu_data_t *data)
{
    esp_err_t ret;
    
    // 分别读取各个传感器数据（模拟原代码风格）
    ret = qmi8658_read_temperature(&data->temperature);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = qmi8658_read_accelerometer(&data->accel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = qmi8658_read_gyroscope(&data->gyro);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope: %s", esp_err_to_name(ret));  
        return ret;
    }
    
    return ESP_OK;
}

// ======================== 滤波器函数 ========================

static void kalman_filter_init(kalman_filter_t *kf, float process_noise, float measurement_noise)
{
    // 初始化状态
    kf->roll = 0.0f;
    kf->pitch = 0.0f;
    kf->roll_bias = 0.0f;
    kf->pitch_bias = 0.0f;
    kf->Q = process_noise;
    kf->R = measurement_noise;
    
    // 初始化协方差矩阵
    memset(kf->P, 0, sizeof(kf->P));
    for (int i = 0; i < 4; i++) {
        kf->P[i][i] = 1.0f;
    }
}

static void kalman_filter_update(kalman_filter_t *kf, const vector3_t *accel, const vector3_t *gyro, float dt)
{
    // 简化实现：使用互补滤波器
    float alpha = 0.995f; // 更高采样率需要调整互补滤波系数
    
    // 从加速度计算姿态角
    float accel_roll = atan2f(accel->y, accel->z);
    float accel_pitch = atan2f(-accel->x, sqrtf(accel->y * accel->y + accel->z * accel->z));
    
    // 陀螺仪积分（去除偏置）
    kf->roll += (gyro->x - kf->roll_bias) * dt;
    kf->pitch += (gyro->y - kf->pitch_bias) * dt;
    
    // 互补滤波融合
    kf->roll = alpha * kf->roll + (1.0f - alpha) * accel_roll;
    kf->pitch = alpha * kf->pitch + (1.0f - alpha) * accel_pitch;
    
    // 更新偏置估计（简化，适应更高采样率）
    kf->roll_bias += 0.0001f * (accel_roll - kf->roll);
    kf->pitch_bias += 0.0001f * (accel_pitch - kf->pitch);
}

static void madgwick_filter_update(madgwick_filter_t *mf, const vector3_t *accel, const vector3_t *gyro, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    
    // 如果加速度向量为0，跳过更新
    if ((accel->x == 0.0f) && (accel->y == 0.0f) && (accel->z == 0.0f)) {
        return;
    }
    
    // 规范化加速度测量值
    recipNorm = 1.0f / sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    float ax = accel->x * recipNorm;
    float ay = accel->y * recipNorm;
    float az = accel->z * recipNorm;
    
    // 辅助变量以避免重复计算
    _2q0 = 2.0f * mf->q0;
    _2q1 = 2.0f * mf->q1;
    _2q2 = 2.0f * mf->q2;
    _2q3 = 2.0f * mf->q3;
    _4q0 = 4.0f * mf->q0;
    _4q1 = 4.0f * mf->q1;
    _4q2 = 4.0f * mf->q2;
    _8q1 = 8.0f * mf->q1;
    _8q2 = 8.0f * mf->q2;
    q0q0 = mf->q0 * mf->q0;
    q1q1 = mf->q1 * mf->q1;
    q2q2 = mf->q2 * mf->q2;
    q3q3 = mf->q3 * mf->q3;
    
    // 梯度下降算法校正步骤
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * mf->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * mf->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * mf->q3 - _2q1 * ax + 4.0f * q2q2 * mf->q3 - _2q2 * ay;
    
    recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    
    // 应用反馈步骤
    qDot1 = 0.5f * (-mf->q1 * gyro->x - mf->q2 * gyro->y - mf->q3 * gyro->z) - mf->beta * s0;
    qDot2 = 0.5f * (mf->q0 * gyro->x + mf->q2 * gyro->z - mf->q3 * gyro->y) - mf->beta * s1;
    qDot3 = 0.5f * (mf->q0 * gyro->y - mf->q1 * gyro->z + mf->q3 * gyro->x) - mf->beta * s2;
    qDot4 = 0.5f * (mf->q0 * gyro->z + mf->q1 * gyro->y - mf->q2 * gyro->x) - mf->beta * s3;
    
    // 积分四元数变化率
    mf->q0 += qDot1 * dt;
    mf->q1 += qDot2 * dt;
    mf->q2 += qDot3 * dt;
    mf->q3 += qDot4 * dt;
    
    // 规范化四元数
    recipNorm = 1.0f / sqrtf(mf->q0 * mf->q0 + mf->q1 * mf->q1 + mf->q2 * mf->q2 + mf->q3 * mf->q3);
    mf->q0 *= recipNorm;
    mf->q1 *= recipNorm;
    mf->q2 *= recipNorm;
    mf->q3 *= recipNorm;
}

// 四元数转欧拉角
static void quaternion_to_euler(const madgwick_filter_t *mf, euler_t *euler)
{
    // Roll (x轴旋转)
    float sinr_cosp = 2.0f * (mf->q0 * mf->q1 + mf->q2 * mf->q3);
    float cosr_cosp = 1.0f - 2.0f * (mf->q1 * mf->q1 + mf->q2 * mf->q2);
    euler->roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (y轴旋转)
    float sinp = 2.0f * (mf->q0 * mf->q2 - mf->q3 * mf->q1);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf(M_PI / 2.0f, sinp); // 使用90度如果超出范围
    } else {
        euler->pitch = asinf(sinp);
    }
    
    // Yaw (z轴旋转)
    float siny_cosp = 2.0f * (mf->q0 * mf->q3 + mf->q1 * mf->q2);
    float cosy_cosp = 1.0f - 2.0f * (mf->q2 * mf->q2 + mf->q3 * mf->q3);
    euler->yaw = atan2f(siny_cosp, cosy_cosp);
}

// 计算重力向量
static void calculate_gravity_vector(const euler_t *euler, vector3_t *gravity)
{
    gravity->x = sinf(euler->pitch);
    gravity->y = -cosf(euler->pitch) * sinf(euler->roll);
    gravity->z = -cosf(euler->pitch) * cosf(euler->roll);
}

// ======================== 主任务函数 ========================

static void imu_task(void *parameters)
{
    ESP_LOGI(TAG, "Starting IMU data collection task");
    
    // 初始化滤波器
    kalman_filter_init(&kf, 0.1f, 0.001f);
    
    // 打印CSV表头
    printf("# QMI8658 IMU Data Log\n");
    printf("# Time(s),AccelX(m/s2),AccelY(m/s2),AccelZ(m/s2),"
           "GyroX(rad/s),GyroY(rad/s),GyroZ(rad/s),Temp(C),"
           "KalmanRoll(rad),KalmanPitch(rad),KalmanYaw(rad),"
           "KalmanGravX,KalmanGravY,KalmanGravZ,"
           "MadgwickRoll(rad),MadgwickPitch(rad),MadgwickYaw(rad),"
           "MadgwickGravX,MadgwickGravY,MadgwickGravZ\n");
    
    int64_t t0 = esp_timer_get_time();
    uint32_t sample_count = 0;
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1)); // 1000Hz采样率，匹配8000Hz ODR
        
        int64_t t1 = esp_timer_get_time();
        float dt = (t1 - t0) / 1000000.0f; // 转换为秒
        t0 = t1;
        
        // 钳位时间步长防止异常值，适应更高采样率
        if (dt > 0.01f) dt = 0.01f;
        if (dt < 0.0001f) dt = 0.0001f;
        
        imu_data_t imu_data = {0};
        
        // 读取IMU数据
        esp_err_t ret = qmi8658_read_raw_data(&imu_data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read IMU data: %s", esp_err_to_name(ret));
            // 重新初始化传感器
            if (qmi8658_init() == ESP_OK) {
                ESP_LOGI(TAG, "Sensor reinitialized successfully");
            }
            continue;
        }
        
        // Kalman滤波处理
        kalman_filter_update(&kf, &imu_data.accel, &imu_data.gyro, dt);
        euler_t kalman_euler = {kf.roll, kf.pitch, 0.0f}; // yaw未实现
        vector3_t kalman_gravity;
        calculate_gravity_vector(&kalman_euler, &kalman_gravity);
        
        // Madgwick滤波处理
        madgwick_filter_update(&madgwick, &imu_data.accel, &imu_data.gyro, dt);
        euler_t madgwick_euler;
        quaternion_to_euler(&madgwick, &madgwick_euler);
        vector3_t madgwick_gravity;
        calculate_gravity_vector(&madgwick_euler, &madgwick_gravity);
        
        // 输出CSV格式数据
        printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,"
               "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
               "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               t1 / 1000000.0f,
               imu_data.accel.x, imu_data.accel.y, imu_data.accel.z,
               imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z,
               imu_data.temperature,
               kalman_euler.roll, kalman_euler.pitch, kalman_euler.yaw,
               kalman_gravity.x, kalman_gravity.y, kalman_gravity.z,
               madgwick_euler.roll, madgwick_euler.pitch, madgwick_euler.yaw,
               madgwick_gravity.x, madgwick_gravity.y, madgwick_gravity.z);
        
        sample_count++;
        
        // 每10000个样本输出一次统计信息
        if (sample_count % 10000 == 0) {
            ESP_LOGI(TAG, "Collected %lu samples, average rate: %.1f Hz", 
                     sample_count, sample_count * 1000000.0f / t1);
        }
    }
}

// ======================== 主应用程序 ========================

void app_main(void)
{
    ESP_LOGI(TAG, "=== QMI8658 IMU Complete Example ===");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    
    // 初始化I2C总线
    ESP_ERROR_CHECK(i2c_master_init());
    
    // 初始化QMI8658传感器
    ESP_ERROR_CHECK(qmi8658_init());
    
    // 测试传感器连接
    ESP_ERROR_CHECK(qmi8658_test_connection());
    
    // 创建IMU数据处理任务
    xTaskCreate(imu_task, "imu_task", 8192, NULL, 10, NULL);
    
    ESP_LOGI(TAG, "=== System initialized successfully ===");
    ESP_LOGI(TAG, "Data format: CSV with timestamp, accel(m/s²), gyro(rad/s), temp(°C), Kalman & Madgwick orientation");
    ESP_LOGI(TAG, "Sample rate: ~1000Hz");
    
    // 主循环：监控系统状态
    uint32_t loop_count = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 每10秒输出一次状态
        loop_count++;
        
        // 输出系统运行状态
        ESP_LOGI(TAG, "System running %lu min, Free heap: %lu bytes", 
                 loop_count * 10 / 60, esp_get_free_heap_size());
    }
}