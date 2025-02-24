#ifndef BLOCKED_H
#define BLOCKED_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    NO_BLOCKED = 0,
    BLOCKED,
} BlockedState_t;

typedef struct
{
    uint32_t turn_time;
    uint32_t start_time;
    uint32_t blocked_time;
    uint32_t blocked_timer;
    uint32_t guard_timer;
} BlockedHandle_t;

/* 宏定义 --------------------------------------------------------------------*/
#define BLOCK_SPEED                 (1.0f)

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void Blocked_Reset(BlockedHandle_t* handle, uint32_t blocked_timer, uint32_t guard_timer);
BlockedState_t Blocked_Process(BlockedHandle_t* handle, fp32 speed);

#endif  // BLOCKED_H

