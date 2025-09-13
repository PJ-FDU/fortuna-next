将你的PNG图片文件放在这个目录中，然后使用以下命令上传到ESP32的SPIFFS分区：

idf.py spiffs-flash

支持的文件类型：
- PNG图片
- SVG矢量图
- 字体文件
- 配置文件
- 任何小于SPIFFS分区大小的文件

文件组织建议：
images/
  ├── icons/
  ├── backgrounds/
  └── ui/
fonts/
config/
cache/