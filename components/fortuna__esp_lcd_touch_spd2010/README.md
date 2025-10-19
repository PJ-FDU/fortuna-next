# Fortuna SPD2010 触摸驱动

本目录存放 Fortuna Next 项目使用的 SPD2010 触摸控制器驱动，基于 Espressif 官方组件
`esp_lcd_touch_spd2010` 定制而来。我们保留了项目所需的行为（例如读取完成后主动发送
Clear INT 指令），并剔除了对组件服务发布所需的元数据。

## 主要改动

- 精简 CMake 与 `idf_component.yml`，仅保留本地工程需要的依赖
- 移除上游的测试应用与发布脚本，避免额外构建时间
- 在触摸数据读取流程里保持 Fortuna 所需的中断清除逻辑（详见源码注释）

## 在工程中的使用

主程序组件已经在 `main/idf_component.yml` 中通过相对路径依赖本目录。若需在其他组件中
复用，可在对应的 `idf_component.yml` 中添加：

```yaml
dependencies:
  fortuna/esp_lcd_touch_spd2010:
    path: ../components/fortuna__esp_lcd_touch_spd2010
```

随后在代码中包含头文件：

```c
#include "esp_lcd_touch_spd2010.h"
```

并按 Espressif 官方接口创建与使用触摸驱动：

```c
esp_lcd_panel_io_handle_t io = /* I2C 面板 IO */;
esp_lcd_touch_handle_t tp = NULL;
esp_lcd_touch_config_t cfg = {
    .x_max = 480,
    .y_max = 480,
    .levels = {
        .reset = 0,
        .interrupt = 0,
    },
};
ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_spd2010(io, &cfg, &tp));
```

> 提示：驱动默认在读取数据后执行 `write_tp_clear_int_cmd`，以释放 SPD2010 的中断脚。若
> 需要调整时序，可查看 `esp_lcd_touch_spd2010.c` 中的注释进行修改。

## 升级与维护建议

- 若需要合并上游修复，可比较 `espressif/esp_lcd_touch_spd2010` 对应版本的源码后选择性
  cherry-pick，并更新 `CHANGELOG.md`
- 保留 `license.txt` 以满足原版权要求
- 修改完成后运行 `idf.py build`，确认编译无误
