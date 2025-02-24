#ifndef CHASSIS_FUNCTION_H
#define CHASSIS_FUNCTION_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "chassis_app.h"
#include "arm_math.h"
#include "user_lib.h"
#include "pid.h"

/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
extern float limit;
/* 函数声明 ------------------------------------------------------------------*/
void Chassis_MoveTransform(ChassisHandle_t* chassis_handle, fp32* chassis_vx, fp32* chassis_vy);
void Mecanum_Calculate(ChassisHandle_t* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw);
void Chassis_ControlCalc(ChassisHandle_t* chassis_handle);
void Chassis_LimitPower(ChassisHandle_t* chassis_handle);
void Chassis_LimitCap(ChassisHandle_t* chassis_handle);
//void SuperPowerSendData(ChassisHandle_t* chassis_handle, uint16_t chassis_power_buff);

#endif  // CHASSIS_FUNCTION_H

