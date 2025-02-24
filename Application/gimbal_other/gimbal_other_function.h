#ifndef GIMBAL_FUNCTION_H
#define GIMBAL_FUNCTION_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_other_app.h"
#include "ramp.h"
#include "arm_math.h"
#include "user_lib.h"
#include "kalman.h"
#include "kalman_filter.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
    fp32 angle[2];  //目标角度
    fp32 gv[2];    //角速度
    fp32 acc[2];   //角加速度
    fp32 out;
    uint32_t times;
//    fp32 time;
} AimCalcData_t;

typedef struct
{
    fp32 yaw_now;
    fp32 yaw_last;
    fp32 pitch_now;
    fp32 pitch_last;
    
    fp32 distance;
}AutoAimError_t;

typedef struct
{
    fp32 yaw_lpc;   //移动预测系数   38；35；30
    fp32 pitch_lpc;
    
    fp32 auto_err_yaw;//角度误差
    fp32 auto_err_pitch;
    
    fp32 kalman_filter_delay;//预测开启延时
    
    fp32 kalman_filter_yaw_speed_min;//速度过低或过高的判断标志
    fp32 kalman_filter_pitch_speed_min;
    fp32 kalman_filter_yaw_speed_max;
    fp32 kalman_filter_pitch_speed_max;
    
    fp32 kalman_filter_yaw_amplification;//预测增幅    130；125；115；135
    fp32 kalman_filter_pitch_amplification;
}AutoAimCoefficient_t;

typedef struct
{
    uint32_t delay_cnt;
    uint32_t freq;
    uint32_t last_time;
    fp32 last_position;
    fp32 speed;
    fp32 last_speed;
    fp32 processed_speed;
}Speed_Calc_Data_t;

typedef struct
{
    uint32_t auto_aim_shoot_l;
    uint32_t auto_aim_shoot_r;
    uint32_t auto_aim_shoot_stop;
}Aim_Shootdelay_t;
/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
fp32 Gimbal_PID_Calc(Gimbal_PID_t* pid, fp32 angle_ref, fp32 angle_fdb, fp32 speed_fdb);
void Gimbal_PID_Clear(Gimbal_PID_t* pid);
void GimbalMotorChangeProtect(GimbalMotor_t* motor);
fp32 Radians_Change(fp32 angle,fp32 radians);
void GimbalMotorControl(GimbalMotor_t* motor);
fp32 AngleTransform(fp32 target_angle, fp32 gyro_angle);
void AimReset(AimCalcData_t* data,fp32 angle_raf);
void AimCalc(AimCalcData_t* data, fp32 angle, fp32 time,fp32 angle_raf);
fp32 pitch_back(fp32 dcf);
fp32 Target_Speed_Calc(Speed_Calc_Data_t *Speed,uint32_t time,fp32 position);
fp32 Auto_Aim_RAMP(fp32 final, fp32 now, fp32 ramp);
#endif  // GIMBAL_FUNCTION_H

