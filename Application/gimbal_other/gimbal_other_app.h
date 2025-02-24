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
} VisionDatabase_t;
/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void GimbalOtherAppConfig(void);
#endif  // GIMBAL_APP_H

