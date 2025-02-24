#ifndef SOFTWARE_TIMER_H
#define SOFTWARE_TIMER_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "soft_timer.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef int32_t (*SoftTimer_Callback_t)(void *argc);

typedef struct
{
    uint8_t id;
    uint32_t ticks;
    void *argc;
    SoftTimer_Callback_t callback;
} SoftwareTimer_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void SoftwareTimerTaskInit(void);
int32_t SoftwareTimerRegister(SoftTimer_Callback_t callback_t, void *argc, uint32_t ticks);

#endif  // SOFTWARE_TIMER_H

