#ifndef USER_PROTOCOL_H
#define USER_PROTOCOL_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"
#include "comm_protocol.h"
#include "chassis/chassis_app.h"
#include "gimbal/gimbal_app.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    RC_DATA_CMD_ID           = 0x0001,
    CHASSIS_INFO_CMD_ID      = 0x0002,
    GIMBAL_INFO_CMD_ID       = 0x0003,
	  GIMBAL_OTHER_INFO_CMD_ID      = 0x0004
} USER_CMD_ID_e;

#pragma pack(push,1)
typedef struct
{
    ChassisCtrlMode_e mode;
} Comm_ChassisInfo_t;

typedef struct
{
    GimbalCtrlMode_e mode;
		VisionState_e state;
    /* unit: degree */
    fp32 pitch_ecd_angle;
    fp32 yaw_ecd_angle;
    fp32 pitch_gyro_angle;
    fp32 yaw_gyro_angle;
    
} Comm_GimbalInfo_t;
#pragma pack(pop)
/* 宏定义 --------------------------------------------------------------------*/
#define USER_PROTOCOL_HEADER_SOF     0xAA
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void UserProtocol_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len);
Comm_ChassisInfo_t* ChassisInfo_Pointer(void);
Comm_GimbalInfo_t* GimbalInfo_Pointer(void);
Comm_GimbalInfo_t* GimbalOtherInfo_Pointer(void);
#endif  // USER_PROTOCOL_H


