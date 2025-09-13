#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 星盘界面API
 * 
 * 这个模块提供了一个使用LVGL 9.3实现的交互式星盘界面，
 * 模拟HTML Canvas中的星盘效果，包括：
 * - 12星座符号和刻度
 * - 12宫位系统 
 * - 10个主要行星
 * - 相位线显示
 * - 1°/秒旋转动画
 * - 触摸交互控制
 */

/**
 * @brief 设置星盘旋转速度
 * @param speed 旋转速度（度/秒），可以是负值表示反向旋转
 */
void astro_chart_set_rotation_speed(float speed);

/**
 * @brief 切换旋转状态（启用/禁用）
 */
void astro_chart_toggle_rotation(void);

/**
 * @brief 重置旋转角度到0度
 */
void astro_chart_reset_rotation(void);

/**
 * @brief 更新行星数据
 * @param planet_index 行星索引 (0-9)
 *                     0=Sun, 1=Moon, 2=Mercury, 3=Venus, 4=Mars,
 *                     5=Jupiter, 6=Saturn, 7=Uranus, 8=Neptune, 9=Pluto
 * @param longitude 黄道经度 (0-360度)
 * @param speed 日运动速度 (度/天，负值表示逆行)
 */
void astro_chart_update_planet_data(int planet_index, float longitude, float speed);

/**
 * @brief 创建星盘界面
 * 
 * 这个函数会初始化整个星盘界面，包括：
 * - 创建412x412像素的圆形星盘
 * - 设置渐变背景
 * - 添加所有星盘元素
 * - 启动旋转动画
 * - 配置触摸交互
 * 
 * 触摸交互：
 * - 单击：暂停/恢复旋转
 * - 长按：重置旋转角度
 */
void create_astro_chart_ui(void);

#ifdef __cplusplus
}
#endif