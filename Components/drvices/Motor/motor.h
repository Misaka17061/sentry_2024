#ifndef MOTOR_H
#define MOTOR_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"
#include "bsp_can.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
    uint16_t ecd;
    uint16_t last_ecd;

    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;           //电机温度

    int32_t round_cnt;
    int32_t total_ecd;
    int32_t total_angle;

    int32_t ecd_raw_rate;

    uint32_t msg_cnt;
    uint16_t offset_ecd;
} MotorInfo_t;

/* 宏定义 --------------------------------------------------------------------*/
#define MOTOR_1TO4_CONTROL_STD_ID   0x200
#define MOTOR_5TO8_CONTROL_STD_ID   0x1FF
#define GM_APPEND_CONTROL_STD_ID    0x2FF   //云台电机增加的ID
#define MOTOR_QUICKLY_SET_ID        0x700   //电机快速设置ID

#define MOTOR_1_FEEDBACK_ID         0x201
#define MOTOR_2_FEEDBACK_ID         0x202
#define MOTOR_3_FEEDBACK_ID         0x203
#define MOTOR_4_FEEDBACK_ID         0x204
#define MOTOR_5_FEEDBACK_ID         0x205
#define MOTOR_6_FEEDBACK_ID         0x206
#define MOTOR_7_FEEDBACK_ID         0x207
#define MOTOR_8_FEEDBACK_ID         0x208
#define GIMBAL_MOTOR1_FEEDBACK_ID   0x209
#define GIMBAL_MOTOR2_FEEDBACK_ID   0x20A
#define GIMBAL_MOTOR3_FEEDBACK_ID   0x20B

#define M3508_MOTOR_MAX_CURRENT     (16000.0f)
#define GM6020_MOTOR_MAX_CURRENT    (30000.0f)
#define M2006_MOTOR_MAX_CURRENT     (4000.0f)
#define MOTOR_ENCODER_RANGE         (8192)
#define MOTOR_ENCODER_RANGE_HALF    (4096)

#define ENCODER_ANGLE_RATIO         (8192.0f / 360.0f)
#define M3508_REDUCTION_RATIO       (1.0f/19.0f)        //3508减速比
#define M3508S_REDUCTION_RATIO       (1.0f/13.8f)        //3508减速比
#define M2006_REDUCTION_RATIO       (1.0f/36.0f)        //2006减速比
#define M3508_MAX_RPM               (8500)              //3508末端最大转速(rpm)
#define BLOCK_SPEED                 (1.0f)
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void Motor_DataParse(MotorInfo_t *ptr, uint8_t data[]);
int16_t Motor_RelativePosition(int16_t ecd, int16_t center_offset);
void Motor_SendMessage(CAN_Object_t *obj, uint32_t std_id, int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4);
void Motor_QuicklySetID(CAN_Object_t *obj);

MotorInfo_t* ChassisMotor_Pointer(uint8_t i);
MotorInfo_t* GimbalMotorYaw_Pointer(void);
MotorInfo_t* GimbalMotorPitch_Pointer(void);
MotorInfo_t* FrictionWheelMotor_1_Pointer(void);
MotorInfo_t* FrictionWheelMotor_2_Pointer(void);
MotorInfo_t* TriggerMotor_Pointer(void);
MotorInfo_t* MagazineMotor_Pointer(void);

#endif  // MOTOR_H

