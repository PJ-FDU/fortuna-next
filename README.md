# Fortuna Next (ESP32-S3) 项目

这是 Fortuna Next 的固件工程（基于 ESP-IDF），包含 LVGL 图形界面、触摸驱动、音频/外设示例和若干服务模块。此仓库包含示例应用、驱动和第三方组件（以 `managed_components/` 形式托管），适合用来在搭载 PSRAM 的 ESP 芯片上开发带触摸与显示的终端设备。

本文档覆盖：快速上手、构建与烧录步骤、关于 PSRAM（SPIRAM）与 LVGL 内存选择的建议、触摸（SPD2010）中断处理说明以及常见问题排查要点。

## 目录结构（摘要）
- `main/` - 应用代码、LVGL 配置（`lv_conf.h`）、服务（`service/`）等
- `example/` - 示例工程与外设演示
- `managed_components/` - 第三方组件（LVGL、驱动等）
- `spiffs_data/` - 打包在 SPIFFS 的资源（如图片）

关键文件：
- `main/fortuna.c` - 程序入口与主逻辑
- `main/lv_conf.h` - LVGL 配置（内存后端、大小等）
- `managed_components/lvgl__lvgl/` - LVGL 源码与内存后端实现
- `components/fortuna__esp_lcd_touch_spd2010/esp_lcd_touch_spd2010.c` - SPD2010 触摸驱动（含读数据与清中断实现）

## 开发环境要求
- ESP-IDF（与项目兼容的版本，请参照本仓库或你的本地 IDF 配置）
- 工具链（交叉编译器）
- Python（idf 工具需要）

## 快速开始（构建、烧写、串口监控）
在项目根目录下：

```bash
# 配置工程（推荐使用 menuconfig）
idf.py menuconfig

# 构建
idf.py build

# 烧录并打开串口监听（替换端口）
idf.py -p /dev/ttyUSB0 flash monitor
```

如果遇到 `ninja: error: loading 'build.ninja': No such file or directory`，请先运行 `idf.py set-target <esp32s3>`（若需要）或直接用 `idf.py build`，IDF 会负责生成 CMake 与 Ninja 文件。不要手工运行 `ninja` 除非已经通过 CMake 生成构建树。

## PSRAM（SPIRAM）与 LVGL 内存策略

项目通常运行在带外部 PSRAM 的 ESP 芯片上。关于把内存放到 PSRAM，有两种主要方法，选择取决于你希望的可控性与兼容性：

1) 显式能力分配（推荐，最可控）

- 使用 `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)`：只有调用此 API 的分配会落到 PSRAM，上限和失败处理可控。适用于大块临时缓冲（例如 PNG 解码缓冲）。
- 优点：风险小、不会影响系统中使用 `malloc()` 的其他库或驱动；可在失败时回退到内部堆。

2) 全局允许 `malloc()` 使用 PSRAM（便捷但风险更高）

- 在 `idf.py menuconfig` 中启用 `Support for external RAM (SPIRAM)` 与 `Enable malloc to use SPIRAM`（对应 `CONFIG_SPIRAM` 与 `CONFIG_SPIRAM_USE_MALLOC`）。
- 优点：不改代码即可让多数通过 `malloc()` 的分配能使用 PSRAM。
- 风险：影响全局行为；可能把延迟敏感或 DMA 需要的缓冲分配到 PSRAM，产生性能或兼容性问题。

3) LVGL 的 TLSF 池（另一个可控方案）

- 将 LVGL 配置为使用内置 TLSF（在 `lv_conf.h` 中选择 `LV_STDLIB_BUILTIN`），并通过 `LV_MEM_POOL_ALLOC` 或 `LV_MEM_ADR` 提供一个由 `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` 分配的连续内存池给 LVGL。这样 LVGL 的 `lv_malloc` 会从放在 PSRAM 的 TLSF 池分配。
- 优点：把 LVGL 的所有分配集中到 PSRAM 的一个连续池，更可预测。缺点：需修改 `lv_conf.h` 并小心选择哪些内存应位于 PSRAM。

建议流程：若只需要把 LodePNG（或其它大解码器）的临时缓冲放入 PSRAM，优先采用显式 `heap_caps_malloc` 在局部替换分配点；如果你想让 LVGL 整体使用 PSRAM，使用 LVGL 的 TLSF 池或谨慎开启 `CONFIG_SPIRAM_USE_MALLOC` 并做大量测试。

### 如何在 menuconfig 中设置（关键项）
- 确保启用 `Support for external RAM (SPIRAM)`（CONFIG_SPIRAM）
- 启用 `Enable SPIRAM capabilities allocation`（CONFIG_SPIRAM_USE_CAPS_ALLOC）以便使用 `heap_caps_malloc`，这通常会同时启用 PSRAM 驱动
- （可选）启用 `Enable malloc to use SPIRAM`（CONFIG_SPIRAM_USE_MALLOC）以让系统 `malloc()` 也可以分配到 PSRAM

## 触摸（SPD2010）中断与清中断行为说明

- 本仓库中 `components/fortuna__esp_lcd_touch_spd2010/esp_lcd_touch_spd2010.c` 的实现会在读取流程里通过 I2C 下发 Clear INT 命令（函数 `write_tp_clear_int_cmd`），触摸芯片收到命令后会释放其 INT 输出线，从而结束该次中断。
- 中断处理机制为：GPIO ISR 仅唤醒任务（或 LVGL 线程）；真正的 I2C 读/写与清中断操作在任务上下文中完成（这是合理且安全的做法，因为 I2C 不应在 ISR 中执行）。
- 如果需要避免重复触发，可以在任务里在读/清之前调用 `gpio_intr_disable()`，读/清完成后再 `gpio_intr_enable()`；请注意不同 IDF 版本对这些 API 在 ISR/任务中的可用性有差异，推荐把禁用/启用放在任务上下文以保证安全性。

关键代码参考：
- `esp_lcd_touch_read_data(...)` → 调用驱动 `tp_read_data(...)`（执行 I2C 读与在需要时调用 `write_tp_clear_int_cmd(...)`）
- LVGL 中断回调（`managed_components/espressif__esp_lvgl_port/src/lvgl9/esp_lvgl_port_touch.c`）：ISR 仅唤醒 LVGL 任务，随后 LVGL 的读回调会调用 `esp_lcd_touch_read_data(...)`。

## 调试与验证建议

- 验证 PSRAM 分配是否生效：在应用初始化打印 `heap_caps_get_free_size(MALLOC_CAP_SPIRAM)`，然后调用 `malloc()` 或 `heap_caps_malloc`，再比较变化，判断内存是否来自 PSRAM。
- 验证触摸中断与清除：通过示波器或在串口日志中观察 INT 引脚行为；在驱动中已有日志打印（`ESP_LOGD`），可打开更高日志等级观察读到的触摸数据与清中断命令发送情况。
- 常见问题：如果启用 `CONFIG_SPIRAM_USE_MALLOC` 后出现不稳定或性能回退，回退该配置并采用显式 `heap_caps_malloc` 将更安全。

## 常见问题排查

- build.ninja 丢失：运行 `idf.py build` 会自动生成构建系统。不要直接对未配置的源码目录运行 `ninja`。
- 串口看不到日志：确认波特率与串口设备，或者用 `idf.py -p /dev/ttyUSB0 monitor`。
- 触摸无响应：检查触摸中断 GPIO 是否配置正确、I2C 接线与地址是否匹配，并查看驱动日志（开启 DEBUG 级别）。

## 贡献与许可证
- 若想贡献代码请基于 `dev` 分支提交 PR。
- 本仓库中第三方组件（例如 LVGL、ESP 驱动）均遵循各自的开源许可证，主仓库请遵照相应 LICENSE。

---
若你希望我把之前讨论的“把 LVGL TLSF 池放到 PSRAM”的补丁写入仓库（或把 LodePNG 的分配器改为显式 `heap_caps_malloc`），我可以直接在仓库中创建补丁并运行一次简单的验证代码，告诉我想优先修改哪一项。
