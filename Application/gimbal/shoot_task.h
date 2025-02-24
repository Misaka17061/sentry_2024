#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "RemoteControl/remote_control.h"
#include "Motor/motor.h"
#include "Motor/blocked.h"
#include "pid.h"
#include "infantry_console.h"
#include "vision_protocol.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    SHOOT_RELAX = 0,          //安全模式
    SHOOT_START,
    SHOOT_STOP,
} ShootCtrlMode_e;

typedef enum
{
    MAGAZINE_INIT_STATE = 0,
    MAGAZINE_OFF_STATE,
    MAGAZINE_ON_STATE,
} MagazineState_e;

typedef enum
{
    TRIGGER_END = 0,
    TRIGGER_BEGIN,
    TRIGGERING
} TriggerState_e;

typedef enum
{
    SHOOT_LEVEL1 = 0,
    SHOOT_LEVEL2,
    SHOOT_LEVEL3
} ShootLevel_e;

typedef struct
{
    MotorInfo_t*    motor_info;
    int32_t         offset_ecd;
    fp32            ecd_ratio;

    Double_PID_t    pid;
    fp32            speed;
    fp32            angle;
    fp32            set_speed;
    fp32            set_angle;
    BlockedHandle_t blocked_handle;

    int16_t         current_set;
} ShootMotor_t;

typedef struct
{
    MotorInfo_t*    motor_info;

    pid_t           pid;
    fp32            set_speed;

    int16_t         current_set;
} FrictionWheelMotor_t;

typedef struct
{
    Console_t*      console;
    CAN_Object_t*   shoot_can;    //
	
		Comm_RobotInfo_t*    vision_tx_data;

    ShootCtrlMode_e ctrl_mode;
    ShootMotor_t    magazine_motor;
    ShootMotor_t    trigger_motor;
    FrictionWheelMotor_t  fric_wheel_motor[2];
    
    int16_t         friction_wheel_speed_buff;
    
    MagazineState_e magazine_state;
    TriggerState_e  trigger_state;
    uint16_t        fire_bullet_number;
    uint16_t        bullet_count;

    uint16_t        shooter_heat_cooling_rate;
    uint16_t        shooter_heat_cooling_limit;
    uint16_t        shooter_speed_limit;
    uint16_t        shooter_heat;
		int16_t         shooter_heat2;
		float           shooter_speedfeedback;
	  uint16_t        trigger_last_angle;
    uint16_t        trigger_angle;
    uint16_t        shooter_heat_cooling_rate_k;		
		
		uint16_t        shoot_flag;
		float           shoot_speed;
	 uint16_t          magazine_pwm;
     uint16_t          magazine_initial_degree;
     uint16_t          magazine_90_degree;
} ShootHandle_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void ShootTaskInit(void);
void ShootOtherTaskInit(void);
#endif  // SHOOT_TASK_H

