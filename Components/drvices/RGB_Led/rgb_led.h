#ifndef RGB_LED_H
#define RGB_LED_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_init.h"

/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/
#define BLACK       (0xFF000000)    // 黑色
#define DARKGRAY    (0xFF555555)    // 深灰
#define LIGHTGRAY   (0xFFAAAAAA)    // 浅灰
#define WHITE       (0xFFFFFFFF)    // 白色
#define GRAY        (0xFF7F7F7F)    // 灰色
#define RED         (0xFFFF0000)    // 红色
#define GREEN       (0xFF00FF00)    // 绿色
#define BLUE        (0xFF0000FF)    // 蓝色
#define CYAN        (0xFF00FFFF)    // 青色
#define YELLOW      (0xFFFFFF00)    // 黄色
#define MAGENTA     (0xFFFF00FF)    // 洋红
#define ORANGE      (0xFFFF7F00)    // 橙色
#define PURPLE      (0xFF7F007F)    // 紫色
#define BROWN       (0xFF996633)    // 棕色
#define CLEAR       (0x00000000)    // 透明


/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void aRGB_led_show(uint32_t aRGB);

#endif  // RGB_LED_H

