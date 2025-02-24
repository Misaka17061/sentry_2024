#ifndef DETECT_TASK_H
#define DETECT_TASK_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_init.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    NO_OFFLINE = 0, /*!< system is normal */
    OFFLINE_CHASSIS_MOTOR1,
    OFFLINE_CHASSIS_MOTOR2,
    OFFLINE_CHASSIS_MOTOR3,
    OFFLINE_CHASSIS_MOTOR4,
	  
    OFFLINE_GIMBAL_PITCH,
    OFFLINE_GIMBAL_YAW,
    OFFLINE_GIMBAL_TURN_MOTOR,
    OFFLINE_FRICTION_WHEEL_MOTOR1,
    OFFLINE_FRICTION_WHEEL_MOTOR2,
    OFFLINE_TRIGGER_MOTOR,
		/*             work space          */
		OFFLINE_GIMBAL_OTHER_PITCH,
    OFFLINE_GIMBAL_OTHER_YAW,
    OFFLINE_GIMBAL_OTHER_TURN_MOTOR,
    OFFLINE_FRICTION_WHEEL_MOTOR1_OTHER,
    OFFLINE_FRICTION_WHEEL_MOTOR2_OTHER,
    OFFLINE_TRIGGER_MOTOR_OTHER,
	  /*             work space          */
    OFFLINE_MAGAZINE_MOTOR,

    OFFLINE_REFEREE_SYSTEM,
    OFFLINE_CHASSIS_INFO,
    OFFLINE_GIMBAL_INFO,
    /*             work space          */
    OFFLINE_GIMBAL_OTHER_INFO,		
    /*             work space          */
    OFFLINE_VISION_INFO,
    OFFLINE_DBUS,	  
    OFFLINE_EVENT_MAX_NUM,
} OfflineEvent_e;

typedef enum
{
    OFFLINE_ERROR_LEVEL = 0,
    OFFLINE_WARNING_LEVEL
} OfflineLevel_e;

typedef enum
{
    ONLINE_STATE = 0,
    OFFLINE_STATE,
} OfflineState_e;

typedef struct
{
    uint8_t enable;
    OfflineEvent_e event;
    OfflineLevel_e error_level;
    OfflineState_e online_state;
    uint32_t last_time;
    uint32_t offline_time;

    /* if offline event number is more than 1, beep_times equal minimum of all events. */
    /* max value is 0xFE */
    uint8_t beep_times;
} OfflineHandle_t;

/* 宏定义 --------------------------------------------------------------------*/
#ifndef ENABLE
#define ENABLE (1u)
#endif
#ifndef DISENABLE
#define DISENABLE (0u)
#endif
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void DetectTaskInit(void);
void OfflineHandle_Init(OfflineEvent_e event,  OfflineLevel_e error_level, uint32_t offline_time, uint32_t beep_times);
void OfflineHandle_TimeUpdate(OfflineEvent_e event);
uint8_t CheckDeviceIsOffline(OfflineEvent_e event);
uint32_t CheckDeviceRunTime(OfflineEvent_e event);
#endif  // DETECT_TASK_H

