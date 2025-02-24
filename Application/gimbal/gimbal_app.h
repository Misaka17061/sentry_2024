#ifndef GIMBAL_APP_H
#define GIMBAL_APP_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "RemoteControl/remote_control.h"
#include "IMU/imu_driver.h"
#include "Motor/motor.h"
#include "pid.h"
#include "infantry_console.h"
#include "vision_protocol.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    GIMBAL_RELAX = 0,          //安全模式
    GIMBAL_INIT,
    GIMBAL_GYRO,
    GIMBAL_RELATIVE,
    GIMBAL_NORMAL,
	  GIMBAL_BACK,
    GIMBAL_VISION_AIM,
    GIMBAL_ADD,
	  GIMBAL_VISION_AIM2,
	  GIMBAL_ASSIST,
		GIMBAL_SENTRY,
} GimbalCtrlMode_e;

typedef enum
{
    RAW_VALUE_MODE = 0,
    GYRO_MODE,
    ENCONDE_MODE,
} GimbalMotorMode_e;

typedef struct
{
    fp32            relative_angle; /* unit: degree */
    fp32            gyro_angle;
    fp32            palstance;      /* uint: degree/s */
} GimbalSensor_t;

typedef struct
{
    pid_t           outer_pid;
    pid_t           inter_pid;
    fp32            angle_ref;
    fp32            angle_fdb;
    fp32            speed_ref;
    fp32            speed_fdb;
} Gimbal_PID_t;

typedef struct
{
    uint16_t        positive_offset_ecd;
    uint16_t        reverse_offset_ecd;
    uint16_t        positive45_offset_ecd;
} Offset_ecd;

typedef struct
{
    MotorInfo_t*    motor_info;
    Offset_ecd        offset_ecd;
    fp32            ecd_ratio;
    fp32            max_relative_angle;
    fp32            min_relative_angle;

    GimbalMotorMode_e mode;
    GimbalMotorMode_e last_mode;
    fp32            given_value;
	  fp32            last_given_value;
    GimbalSensor_t  sensor;
    Gimbal_PID_t    pid;

    int16_t         current_set;
	  int8_t          back_flag;
	  fp32            gyro_flag;
} GimbalMotor_t;

typedef struct
{
    Console_t*          console;
    IMU_Data_t*         imu;              //
    CAN_Object_t*       gimbal_can;       //
	
	  Comm_RobotInfo_t*    vision_tx_data;

	  Comm_VisionInfo_t*   vision_rx_data;
    
    Gimbal_CMD_e        last_cmd;
    GimbalCtrlMode_e    ctrl_mode;       //
    GimbalCtrlMode_e    last_ctrl_mode;  //
    
    pid_t               gimbal_vision_yaw_pid;
    pid_t               gimbal_vision_pitch_pid;

    GimbalMotor_t       yaw_motor;
    GimbalMotor_t       pitch_motor;
} GimbalHandle_t;

typedef union
{
    float vision_rx[4];
    uint8_t vdata[4];
}Rx_vision;


//STF_NEW
typedef enum
{
    VISIONCOMM_WRONG = 0,
    VISIONCOMM_SUCCESSED,
	  VISIONCOMM_INITIAL,
} VisionComm_e;

typedef enum
{
    AIM_NO = 0,
    AIM_RIGHT,
} VisionAim_e;

typedef struct
{
   VisionComm_e comm_state;
} VisionComm_t;

typedef struct
{
    fp32    pitch;
    fp32    yaw;
    fp32    last_pitch;
    fp32    last_yaw;    
    fp32    distance;  
    fp32 pitch_ratio;
    fp32 yaw_ratio;
    fp32 yaw_ang_vel;
   	VisionState_e state;     //VisionState_e  0x01->Comm_Successed
    VisionAim_e yaw_success;
    VisionAim_e pitch_success;
} VisionDatabase_t;

typedef struct
{
uint32_t pid_SAtime;
uint32_t aiming_time;
uint32_t stay_time;
uint32_t systeam_time;
fp32 tol_angle ;  //人为规定消抖角度范围(-tol_angle,tol_angle)
uint32_t tol_time ;//the aiming-done continueous time that user set 人为规定
uint32_t Sstart_time,Astart_time;//消抖持续时间,单次自瞄持续时间
int first_aim ;//初入消抖范围标志
int aim_flag ; //新的单次自瞄标志
pid_t Vision_PID;
float Ap_parm;
float Sp_parm;
//float Ai_parm = 8000;
//float Si_parm = 0.002;
int enable_paramSA;
}AutoAim_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void GimbalAppConfig(void);
void GimbalOtherAppConfig(void);
#endif  // GIMBAL_APP_H

