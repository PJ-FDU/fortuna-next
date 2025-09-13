#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

static const char *TAG = "QMI8658_C_Example";

// QMI8658 寄存器定义
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
#define QMI8658_FIFO_WTM_TH         0x13
#define QMI8658_FIFO_CTRL           0x14
#define QMI8658_FIFO_COUNT          0x15
#define QMI8658_FIFO_STATUS         0x16
#define QMI8658_FIFO_DATA           0x17
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

// I2C配置
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           10
#define I2C_MASTER_SCL_IO           11
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// 数据结构定义
typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
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
    float roll_rate;
    float pitch_rate;
    float process_noise;
    float measurement_noise;
} kalman_filter_t;

// 简化版Madgwick滤波器状态
typedef struct {
    float q0, q1, q2, q3;  // 四元数
    float beta;            // 算法增益
} madgwick_filter_t;

// 全局变量
static kalman_filter_t kf = {0};
static madgwick_filter_t madgwick = {1.0f, 0.0f, 0.0f, 0.0f, 0.1f};
static SemaphoreHandle_t i2c_mutex = NULL;

// I2C读写函数
static esp_err_t i2c_master_write_slave(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t data_len)
{
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    if (data_len > 0) {
        i2c_master_write(cmd, data, data_len, true);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    xSemaphoreGive(i2c_mutex);
    return ret;
}

static esp_err_t i2c_master_read_slave(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t data_len)
{
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    if (data_len > 1) {
        i2c_master_read(cmd, data, data_len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + data_len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    xSemaphoreGive(i2c_mutex);
    return ret;
}

// I2C初始化
static esp_err_t i2c_master_init(void)
{
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
        return err;
    }
    
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        return err;
    }
    
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

// QMI8658初始化
static esp_err_t qmi8658_init(void)
{
    uint8_t who_am_i;
    esp_err_t ret;
    
    // 读取WHO_AM_I寄存器
    ret = i2c_master_read_slave(QMI8658_I2C_ADDR, QMI8658_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X", who_am_i);
    if (who_am_i != 0x05) {
        ESP_LOGE(TAG, "Invalid WHO_AM_I value");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // 软复位
    uint8_t reset_cmd = 0xB0;
    ret = i2c_master_write_slave(QMI8658_I2C_ADDR, QMI8658_CTRL1, &reset_cmd, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 配置加速度计：8G量程，250Hz ODR
    uint8_t accel_config = 0x05; // 8G, 250Hz
    ret = i2c_master_write_slave(QMI8658_I2C_ADDR, QMI8658_CTRL2, &accel_config, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 配置陀螺仪：512dps量程，250Hz ODR
    uint8_t gyro_config = 0x45; // 512dps, 250Hz
    ret = i2c_master_write_slave(QMI8658_I2C_ADDR, QMI8658_CTRL3, &gyro_config, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 使能加速度计和陀螺仪
    uint8_t enable = 0x03;
    ret = i2c_master_write_slave(QMI8658_I2C_ADDR, QMI8658_CTRL7, &enable, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    return ESP_OK;
}

// 读取原始数据
static esp_err_t qmi8658_read_raw_data(imu_data_t *data)
{
    uint8_t raw_data[14];
    esp_err_t ret;
    
    // 读取温度和IMU数据
    ret = i2c_master_read_slave(QMI8658_I2C_ADDR, QMI8658_TEMP_L, raw_data, 14);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 解析温度数据
    int16_t temp_raw = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    data->temperature = temp_raw / 256.0f;
    
    // 解析加速度数据 (8G量程，转换为m/s^2)
    int16_t accel_x = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    int16_t accel_y = (int16_t)((raw_data[5] << 8) | raw_data[4]);
    int16_t accel_z = (int16_t)((raw_data[7] << 8) | raw_data[6]);
    
    data->accel.x = (accel_x / 32768.0f) * 8.0f * 9.80665f;
    data->accel.y = (accel_y / 32768.0f) * 8.0f * 9.80665f;
    data->accel.z = (accel_z / 32768.0f) * 8.0f * 9.80665f;
    
    // 解析陀螺仪数据 (512dps量程，转换为rad/s)
    int16_t gyro_x = (int16_t)((raw_data[9] << 8) | raw_data[8]);
    int16_t gyro_y = (int16_t)((raw_data[11] << 8) | raw_data[10]);
    int16_t gyro_z = (int16_t)((raw_data[13] << 8) | raw_data[12]);
    
    data->gyro.x = (gyro_x / 32768.0f) * 512.0f * M_PI / 180.0f;
    data->gyro.y = (gyro_y / 32768.0f) * 512.0f * M_PI / 180.0f;
    data->gyro.z = (gyro_z / 32768.0f) * 512.0f * M_PI / 180.0f;
    
    return ESP_OK;
}

// 简化版Kalman滤波器初始化
static void kalman_filter_init(kalman_filter_t *kf, float process_noise, float measurement_noise)
{
    kf->roll = 0.0f;
    kf->pitch = 0.0f;
    kf->roll_rate = 0.0f;
    kf->pitch_rate = 0.0f;
    kf->process_noise = process_noise;
    kf->measurement_noise = measurement_noise;
}

// 简化版Kalman滤波器更新
static void kalman_filter_update(kalman_filter_t *kf, const vector3_t *accel, const vector3_t *gyro, float dt)
{
    // 简化实现：仅做互补滤波
    float alpha = 0.98f; // 互补滤波系数
    
    // 从加速度计算姿态角
    float accel_roll = atan2f(accel->y, accel->z);
    float accel_pitch = atan2f(-accel->x, sqrtf(accel->y * accel->y + accel->z * accel->z));
    
    // 陀螺仪积分
    kf->roll += gyro->x * dt;
    kf->pitch += gyro->y * dt;
    
    // 互补滤波
    kf->roll = alpha * kf->roll + (1.0f - alpha) * accel_roll;
    kf->pitch = alpha * kf->pitch + (1.0f - alpha) * accel_pitch;
}

// 简化版Madgwick滤波器更新
static void madgwick_filter_update(madgwick_filter_t *mf, const vector3_t *accel, const vector3_t *gyro, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    
    // 规范化加速度测量
    recipNorm = 1.0f / sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    float ax = accel->x * recipNorm;
    float ay = accel->y * recipNorm;
    float az = accel->z * recipNorm;
    
    // 辅助变量以避免重复算术
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
    float sinr_cosp = 2 * (mf->q0 * mf->q1 + mf->q2 * mf->q3);
    float cosr_cosp = 1 - 2 * (mf->q1 * mf->q1 + mf->q2 * mf->q2);
    euler->roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (y轴旋转)
    float sinp = 2 * (mf->q0 * mf->q2 - mf->q3 * mf->q1);
    if (fabsf(sinp) >= 1) {
        euler->pitch = copysignf(M_PI / 2, sinp); // 使用90度如果超出范围
    } else {
        euler->pitch = asinf(sinp);
    }
    
    // Yaw (z轴旋转)
    float siny_cosp = 2 * (mf->q0 * mf->q3 + mf->q1 * mf->q2);
    float cosy_cosp = 1 - 2 * (mf->q2 * mf->q2 + mf->q3 * mf->q3);
    euler->yaw = atan2f(siny_cosp, cosy_cosp);
}

// 计算重力向量
static void calculate_gravity_vector(const euler_t *euler, vector3_t *gravity)
{
    gravity->x = sinf(euler->pitch);
    gravity->y = -cosf(euler->pitch) * sinf(euler->roll);
    gravity->z = -cosf(euler->pitch) * cosf(euler->roll);
}

// IMU数据处理任务
static void imu_task(void *parameters)
{
    ESP_LOGI(TAG, "Starting IMU task");
    
    // 初始化滤波器
    kalman_filter_init(&kf, 0.1f, 0.001f);
    
    // 打印CSV表头
    printf("Time(s),AccelX(m/s2),AccelY(m/s2),AccelZ(m/s2),"
           "GyroX(rad/s),GyroY(rad/s),GyroZ(rad/s),Temp(C),"
           "KalmanRoll(rad),KalmanPitch(rad),KalmanYaw(rad),"
           "KalmanGravX,KalmanGravY,KalmanGravZ,"
           "MadgwickRoll(rad),MadgwickPitch(rad),MadgwickYaw(rad),"
           "MadgwickGravX,MadgwickGravY,MadgwickGravZ\n");
    
    int64_t t0 = esp_timer_get_time();
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz采样率
        
        int64_t t1 = esp_timer_get_time();
        float dt = (t1 - t0) / 1000000.0f; // 转换为秒
        t0 = t1;
        
        // 钳位时间步长
        if (dt > 0.05f) dt = 0.05f;
        if (dt < 0.001f) dt = 0.001f;
        
        imu_data_t imu_data = {0};
        
        // 读取IMU数据
        esp_err_t ret = qmi8658_read_raw_data(&imu_data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read IMU data: %s", esp_err_to_name(ret));
            continue;
        }
        
        // Kalman滤波
        kalman_filter_update(&kf, &imu_data.accel, &imu_data.gyro, dt);
        euler_t kalman_euler = {kf.roll, kf.pitch, 0.0f};
        vector3_t kalman_gravity;
        calculate_gravity_vector(&kalman_euler, &kalman_gravity);
        
        // Madgwick滤波
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
    }
}

// 主应用程序
void app_main(void)
{
    ESP_LOGI(TAG, "Starting QMI8658 C Example");
    
    // 初始化I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    // 初始化QMI8658
    ESP_ERROR_CHECK(qmi8658_init());
    ESP_LOGI(TAG, "QMI8658 initialized successfully");
    
    // 创建IMU任务
    xTaskCreate(imu_task, "imu_task", 8192, NULL, 10, NULL);
    
    ESP_LOGI(TAG, "IMU task started");
    
    // 主循环保持系统运行
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}