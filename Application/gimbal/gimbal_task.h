#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "gimbal_function.h"
/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/
#define ABS(x) (((x) > 0) ? (x) : (-(x)))
/* 扩展变量 ------------------------------------------------------------------*/
extern uint8_t aim_shoot_state;
/* 函数声明 ------------------------------------------------------------------*/
void GimbalTaskInit(void);
void GimbalOtherTaskInit(void);
VisionDatabase_t* VisionOtherData_Pointer(void);
VisionDatabase_t* VisionData_Pointer(void);
#endif  // GIMBAL_TASK_H

