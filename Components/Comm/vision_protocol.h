#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"
#include "comm_protocol.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    VISION_DATA_CMD_ID          = 0x0001,
    ROBOT_DATA_CMD_ID           = 0x0002,
	ATTACK_DATA_CMD_ID          = 0x0000,
} VISION_CMD_ID_e;

typedef enum
{
    VISION_TRACK_LOSS       = 0x00,
    VISION_TRACK            = 0x01,
    VISION_TARGET_CHANGE    = 0x02,
    VISION_TARGET_SPIN      = 0x03,
	  VISION_FIRECTION_LOSS   = 0x04,
} VisionState_e;

typedef enum
{
    Hero        = 0x0001,
    Engineer    = 0x0002,
    Infantry    = 0x0004,
    Sentey      = 0x0008,
    AllRobot    = Hero|Engineer|Infantry|Sentey,
    Windmill    = 0x1000
} DetectTarget_e;

typedef enum
{
    Gray = 0,
    Red = 1,
    Blue = 2,
    AllColor = Red|Blue
} EnemyColor_e;

#pragma pack(push,1)
typedef struct
{
    uint8_t    data_head;    //头帧 默认0xAA
    uint8_t    pitch_int;     /* Pitch角度值整数部分 */
    uint8_t    pitch_dec;     /* Pitch角度值小数部分，精度两位 */
    uint8_t    yaw_int;       /* Yaw角度值整数部分 */
    uint8_t    yaw_dec;       /* Yaw角度值小数部分，精度两位 */
    uint8_t    distance_1;  //cm距离0-255
    uint8_t    distance_2;  //cm距离0-255    Total_Distance = distance_1+distance_2
   	uint8_t    state;     //VisionState_e  0x01->Comm_Successed
    //uint8_t robot_num;
    //uint8_t fire_cmd;
    //uint8_t update;
	uint8_t    data_tail;   //尾帧
} Comm_VisionInfo_t;

typedef struct
{
   	  uint8_t data_head;     //VisionState_e
	  uint8_t enemy_color;
	  fp32    speed;
	  fp32    yaw_relative_angle;
	  fp32    pitch_relative_angle;
	  uint16_t bullet_speed;
     uint8_t  data_tail;
    //uint8_t robot_num;
    //uint8_t fire_cmd;
    //uint8_t update;  
} Comm_RobotInfo_t;
#pragma pack(pop)
/* 宏定义 --------------------------------------------------------------------*/
#define VISION_PROTOCOL_HEADER_SOF     0x55
#define VISION_DATA_FIFO_SIZE          (256u)
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void VisionProtocol_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len);
Comm_VisionInfo_t* VisionInfo_Pointer(void);
Comm_RobotInfo_t* RobotInfo_Pointer(void);

#endif  // VISION_PROTOCOL_H

