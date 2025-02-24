#ifndef CONSOLE_H
#define CONSOLE_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "RemoteControl/remote_control.h"
#include "user_lib.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    PREPARE_MODE = 0,       //初始化
    NORMAL_MODE,            //正常运行模式
    SAFETY_MODE,              //安全模式（停止运动）
} CtrlMode_e;

typedef enum
{
    GIMBAL_RELEASE_CMD = 0,
    GIMBAL_INIT_CMD,
    GIMBAL_GYRO_CMD,
    GIMBAL_RELATIVE_CMD,
    GIMBAL_NORMAL_CMD,
	  GIMBAL_BACK_CMD,
    GIMBAL_VISION_AIM_CMD,
	  GIMBAL_VISION_AIM2_CMD,	
    GIMBAL_ADD_CMD,
	  GIMBAL_ASSIST_CMD,
		GIMBAL_SENTRY_CMD,
} Gimbal_CMD_e;

typedef enum
{
    UI_OFF_CMD = 0,
    UI_ON_CMD,
}UI_CMD_e;
typedef enum
{
  SUPERCAP_OFF_CMD=0,
	SUPERCAP_ON_CMD,
}SuperCap_CMD_e;
typedef enum
{
    CHASSIS_RELEASE_CMD = 0,
    CHASSIS_STOP_CMD,               //底盘停止
    CHASSIS_FOLLOW_GIMBAL_CMD,      //底盘跟随云台
    CHASSIS_SEPARATE_GIMBAL_CMD,    //底盘云台分离
		CHASSIS_SENTRY_CMD,							//超级哨兵模式
    CHASSIS_SPIN_CMD,               //底盘旋转
    CHASSIS_SUPSPIN_CMD,            //超级小陀螺
    CHASSIS_HALF_CMD,               //歪头
    CHASSIS_HALFSPIN_CMD,               //甩尾
} Chassis_CMD_e;

typedef enum
{
    SHOOT_RELEASE_CMD = 0,
    SHOOT_START_CMD,
    SHOOT_STOP_CMD,
} Shoot_CMD_e;


typedef enum
{
    MAGAZINE_INIT_CMD = 0,
    MAGAZINE_OFF_CMD,
    MAGAZINE_ON_CMD,
} Magazine_CMD_e;

typedef enum
{
    STOP_FIRE_CMD = 0, //停止射击
    ONE_FIRE_CMD,      //单发模式
	  TWO_FIRE_CMD,      //二连发模式
    RAPID_FIRE_CMD,    //无限制连发模式
} ShootFire_CMD_e;

typedef enum
{
    ON = 0,
    OFF,
}flag_e;
typedef enum
{
    LEFT=0,
    RIGHT,
}gimbal_motor_direction;
typedef enum
{
    flag_positive,
    flag_reverse,
}gimbal_motor_flag_e;

typedef struct
{
		/* space work*/
		Gimbal_CMD_e gimbal_other_cmd;
		Shoot_CMD_e shoot_other_cmd;
		int16_t friction_wheel_other_speed_buff;
		struct
    {
        fp32 pitch_v;
        fp32 yaw_v;
    } gimbal_other;
		 struct
    {
        ShootFire_CMD_e fire_cmd;
	  		ShootFire_CMD_e last_fire_cmd;
    } shoot_other;
		/* space work*/
		
    RC_Info_t* rc;
    CtrlMode_e ctrl_mode;
    Gimbal_CMD_e gimbal_cmd;
    Chassis_CMD_e chassis_cmd;
    Shoot_CMD_e shoot_cmd;
	  Magazine_CMD_e magazine_cmd;
	  SuperCap_CMD_e supercap_cmd;
    flag_e shift_flag;
    
    gimbal_motor_flag_e Reset_yaw;
    gimbal_motor_flag_e Reset_pitch;
	  gimbal_motor_direction  gimbal_motor_direction_cmd;
    
    
    int16_t friction_wheel_speed_buff;
    struct
    {
        fp32 vx;
        fp32 vy;
        fp32 vw;
    } chassis;

    struct
    {
        fp32 pitch_v;
        fp32 yaw_v;
    } gimbal;
		
    struct
    {
        ShootFire_CMD_e fire_cmd;
	  		ShootFire_CMD_e last_fire_cmd;
    } shoot;
    
    UI_CMD_e    ui_cmd;
    fp32 Yaw_comps;
		fp32 Pitch_comps;
    uint16_t aim_table;
		fp32  All_console;
    uint8_t spin90_flag;
		uint16_t Aiming_mode;
		uint16_t energy_mechanism_falg;
		uint16_t reset;
		
} Console_t;


/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void ConsoleTaskInit(void);
Console_t* Console_Pointer(void);

#endif  // CONSOLE_H

