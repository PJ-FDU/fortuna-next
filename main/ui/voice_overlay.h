#ifndef VOICE_OVERLAY_H
#define VOICE_OVERLAY_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化语音识别遮罩
 */
void voice_overlay_init(void);

/**
 * @brief 显示语音识别遮罩
 */
void voice_overlay_show(void);

/**
 * @brief 隐藏语音识别遮罩
 */
void voice_overlay_hide(void);

/**
 * @brief 更新遮罩文本
 */
void voice_overlay_set_text(const char *text);

#ifdef __cplusplus
}
#endif

#endif // VOICE_OVERLAY_H