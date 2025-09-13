# QMI8658驱动代码调整说明

## 调整概述
基于原有的QMI8658.h和QMI8658.c驱动代码，对qmi8658_complete_example.c进行了以下关键调整，确保能够正确驱动QMI8658传感器。

## 主要调整内容

### 1. 初始化序列优化
```c
// 原配置（不正确）
uint8_t accel_config = 0x05; // ±8G, 250Hz
uint8_t gyro_config = 0x45;  // ±512dps, 250Hz

// 新配置（参考原代码）  
uint8_t accel_config = 0x10; // ±4G, 8000Hz ODR
uint8_t gyro_config = 0x20;  // ±64dps, 8000Hz ODR
```

**改进说明：**
- 使用与原代码相同的默认量程配置
- 启用高速采样率以获得更好的数据质量
- 添加正确的CTRL1寄存器配置（自动地址递增）
- 配置低通滤波器以减少噪声

### 2. 数据缩放因子修正
```c
// 加速度计缩放因子（±4G量程）
static float accel_scale_factor = 4.0f / 32768.0f;

// 陀螺仪缩放因子（±64dps量程） 
static float gyro_scale_factor = 64.0f / 32768.0f;
```

**改进说明：**
- 与原代码的accelScales和gyroScales变量保持一致
- 正确的物理单位转换
- 支持不同量程的动态配置

### 3. 数据读取方式优化
```c
// 新增独立的读取函数
static esp_err_t qmi8658_read_accelerometer(vector3_t *accel);
static esp_err_t qmi8658_read_gyroscope(vector3_t *gyro);
static esp_err_t qmi8658_read_temperature(float *temperature);
```

**改进说明：**
- 模拟原代码的getAccelerometer()和getGyroscope()函数风格
- 使用小端字节序解析（与原代码一致）
- 分离各传感器的读取操作，提高可靠性

### 4. 采样率和滤波器调整
```c
// 采样率调整
vTaskDelay(pdMS_TO_TICKS(1)); // 1000Hz采样率

// 互补滤波器系数调整
float alpha = 0.995f; // 适应高采样率

// 偏置估计步长调整
kf->roll_bias += 0.0001f * (accel_roll - kf->roll);
```

**改进说明：**
- 匹配8000Hz硬件ODR的高速采样
- 调整滤波器参数以适应更高的数据更新率
- 优化偏置估计的收敛速度

### 5. 寄存器配置完整性
```c
// 完整的初始化序列
CTRL1: 0x40  // 启用自动地址递增
CTRL2: 0x10  // 加速度计配置
CTRL3: 0x20  // 陀螺仪配置  
CTRL5: 0x61  // 低通滤波器配置
CTRL6: 0x00  // 禁用AttitudeEngine
CTRL7: 0x43  // 使能传感器和高速时钟
```

**改进说明：**
- 参考原代码的setState(sensor_running)实现
- 确保所有必要的控制寄存器都被正确配置
- 启用硬件优化特性（自动地址递增）

## 硬件兼容性
调整后的代码与原QMI8658驱动代码完全兼容：
- 相同的I2C地址（0x6B）
- 相同的寄存器操作方式
- 相同的数据格式和字节序
- 相同的物理单位转换

## 性能改进
- **更高精度**：使用±4G/±64dps量程，提供更好的分辨率
- **更低延迟**：1000Hz采样率，适合实时应用
- **更好的稳定性**：添加连接测试和错误恢复机制
- **更准确的滤波**：调整滤波器参数以匹配高采样率

## 验证方法
代码中添加了qmi8658_test_connection()函数：
- 验证I2C通信正常
- 检查数据读取功能
- 输出样本数据用于调试
- 确保所有传感器正常工作

## 使用建议
1. **硬件连接**：确保I2C引脚配置正确
2. **电源稳定**：QMI8658需要稳定的3.3V供电
3. **上拉电阻**：I2C总线需要适当的上拉电阻
4. **采样率调整**：可根据应用需求调整vTaskDelay时间

这些调整确保了qmi8658_complete_example.c能够与原有的QMI8658驱动代码保持完全兼容，同时提供更好的性能和可靠性。