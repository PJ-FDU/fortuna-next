# Fortuna SPD2010 变更记录

## 0.1.0 - 2025-05-21

- 从 `espressif/esp_lcd_touch_spd2010` 派生，保留 Fortuna 所需的中断清除逻辑
- 精简目录结构，移除组件服务相关文件与示例测试工程
- 更新 CMake 与 `idf_component.yml`，用于项目本地依赖
