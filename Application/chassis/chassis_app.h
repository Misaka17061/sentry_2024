#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "RemoteControl/remote_control.h"
#include "IMU/imu_driver.h"
#include "Motor/motor.h"
#include "pid.h"
#include "infantry_console.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    CHASSIS_RELAX = 0,          //安全模式
    CHASSIS_STOP,               //底盘停止
    CHASSIS_FOLLOW_GIMBAL,      //底盘跟随云台
    CHASSIS_SEPARATE_GIMBAL,    //底盘云台分离
		CHASSIS_SENTRY, 						//超级哨兵
    CHASSIS_SPIN,               //底盘旋转
	  CHASSIS_SUPSPIN,             //底盘超级旋转
    CHASSIS_HALF,          //底盘侧边
    CHASSIS_HALF_SPIN,      //甩尾
} ChassisCtrlMode_e;

typedef struct
{
    fp32 wheel_perimeter; /* the perimeter(mm) of wheel *///wheel周长
    fp32 wheeltrack;      /* wheel track distance(mm) */  //轮距
    fp32 wheelbase;       /* wheelbase distance(mm) */    //轴距
    fp32 rotate_x_offset; /* rotate offset(mm) relative to the x-axis of the chassis center */  //相对于底盘中心的x轴旋转偏移（mm）
    fp32 rotate_y_offset; /* rotate offset(mm) relative to the y-axis of the chassis center */  //相对于底盘中心的y轴旋转偏移（mm）
} MechanicalStructure_t;

//typedef struct
//{
//    int16_t gyro_angle;
//    int16_t gyro_palstance;
//    int32_t position_x_mm;
//    int32_t position_y_mm;
//    int16_t angle_deg;
//    int16_t v_x_mm;
//    int16_t v_y_mm;
//} ChassisInfo_t;

typedef struct
{
    MotorInfo_t*    motor_info;
    pid_t           pid;
    fp32            given_speed;
    fp32         current_set;
} ChassisMotor_t;

typedef struct
{
    Console_t*    console;
    IMU_Data_t*   imu;                      //底盘陀螺仪指针
    CAN_Object_t* chassis_can;              //

    ChassisCtrlMode_e       ctrl_mode;             //底盘控制状态
	  MechanicalStructure_t   structure;             //机械结构体
    ChassisMotor_t          chassis_motor[4];
    pid_t                   chassis_follow_pid;        //follow angle PID.
    pid_t                   super_power_limit_pid;

    fp32 vx;                      //
    fp32 vy;                      //
    fp32 vw;                      //
    fp32 wheel_rpm[4];
	
	  float Super_Power_ratio;         //超电充电比
    float Chassis_super_power;
    uint16_t enemy_distance;
	  uint16_t super_flag;
	uint16_t super_low_flag;
    fp32 gimbal_yaw_ecd_angle;
    fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.
    fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.
    fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.
} ChassisHandle_t;


typedef struct
{
    uint8_t u[8];
    uint16_t f;
	  uint8_t super_state;
	  uint16_t f2;
	  double ff;
}FormatTrans_t;

typedef struct 
{
	uint8_t v_x;
	uint8_t v_y;
	uint8_t v_x_2;
	uint8_t w;
	int i_max;
	int j_max;
}fuck_run_struct;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void ChassisAppConfig(void);

#endif  // CHASSIS_APP_H

