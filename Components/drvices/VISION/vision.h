#ifndef __VISION_H__
#define __VISION_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    VISION_TEST1_CMD_ID           = 0x0001,       //视觉测试命令1
    VISION_TEST2_CMD_ID           = 0x0002,       //视觉测试命令2
} vision_cmd_id_e;

typedef struct     //视觉命令数据:0x0001 
{
    uint8_t test1;  //测试数据1   
} ext_vision_test1_t;

typedef struct     //视觉命令数据:0x0002 
{
    uint8_t test2;  //测试数据2  
} ext_vision_test2_t;

//typedef enum
//{
//    VISION_MODE_CMD_ID           = 0x0001,       //视觉控制模式
//    VISION_POSITION_CMD_ID       = 0x0002,       //视觉位置控制模式
//	  VISION_INCREMENTAL_CMD_ID    = 0X0003,       //视觉增量控制模式 
//} vision_cmd_id_e;

//typedef enum
//{
//	  NO_CTRL       =0,
//    POSITION_CTRL,
//    INCREMENTAL_CTRL,
//} vision_ctrlmode_t;

//typedef struct     //视觉控制模式数据:0x0001 
//{
//	uint8_t vision_ctrl_mode :1;  //视觉控制模式  0bit:  0不控制  1位置控制模式  2增量控制模式	
//} ext_vision_ctrl_t;

//typedef struct     //视觉位置控制数据:0x0002 
//{
//	uint32_t vision_position_pitch ;  //视觉pitch位置控制 float   
//  uint32_t vision_position_yaw ;  //视觉yaw位置控制 float 	
//} ext_vision_position_ctrl_t;

//typedef struct     //视觉增量控制数据:0x0003 
//{
//	uint32_t vision_incremental_pitch ;  //视觉pitch增量控制 float   
//  uint32_t vision_incremental_yaw ;  //视觉yaw增量控制 float 	
//} ext_vision_incremental_ctrl_t;
/* 宏定义 --------------------------------------------------------------------*/
#define VISION_HEADER_SOF            0xA0
#define VISION_FIFO_SIZE            (512u)
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void Vision_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len);
ext_vision_test1_t* Vision_Test1_Pointer(void);
ext_vision_test2_t* Vision_Test2_Pointer(void);
//uint8_t Vision_CtrlMode(void);
//ext_vision_position_ctrl_t* Vision_PositionCtrl_Pointer(void);
//ext_vision_incremental_ctrl_t* Vision_IncrementalCtrl_Pointer(void);
#endif  // __VISION_H__

