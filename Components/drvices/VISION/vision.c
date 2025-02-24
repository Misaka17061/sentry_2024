/* 包含头文件 ----------------------------------------------------------------*/
#include "vision.h"
#include <string.h>
#include <math.h>

/* 私有类型定义 --------------------------------------------------------------*/
ext_vision_test1_t vision_test1;
ext_vision_test2_t vision_test2;
//ext_vision_ctrl_t  vision_ctrl_mode;
//ext_vision_position_ctrl_t vision_position_ctrl;
//ext_vision_incremental_ctrl_t vision_incremental_ctrl;
/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
    
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void Vision_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len)
{
     switch(cmd_id)
		 {
			 case VISION_TEST1_CMD_ID:
			 {
				 memcpy(&vision_test1, data, sizeof(ext_vision_test1_t));
			 }break;
			 
			 case VISION_TEST2_CMD_ID:
			 {
				 memcpy(&vision_test2, data, sizeof(ext_vision_test2_t));
			 }break;
			 
//			 case VISION_MODE_CMD_ID:
//			 {
//				 mempy(&vision_ctrl_mode, data, sizeof(ext_vision_ctrl_t));
//			 }break;
//			 
//			 case VISION_POSITION_CMD_ID:
//			 {
//				 mempy(&vision_position_ctrl, data, sizeof(ext_vision_position_ctrl_t));
//			 }break;
//			 
//			 case VISION_INCREMENTAL_CMD_ID:
//			 {
//				 mempy(&vision_incremental_ctrl, data, sizeof(ext_vision_incremental_ctrl_t));
//			 }break;
		 }
}

/*************************************************
 * Function: Vision_Test1_Pointer
 * Description: 获取视觉测试数据1
 * Input: 无
 * Return: 视觉测试指针1
*************************************************/
ext_vision_test1_t* Vision_Test1_Pointer(void)
{
    return &vision_test1;
}

/*************************************************
 * Function: Vision_Test2_Pointer
 * Description: 获取视觉测试数据2
 * Input: 无
 * Return: 视觉测试指针2
*************************************************/
ext_vision_test2_t* Vision_Test2_Pointer(void)
{
    return &vision_test2;
}
///*************************************************
// * Function: Vision_CtrlMode
// * Description: 获取视觉控制模式
// * Input: 无
// * Return: 视觉控制模式 vision_ctrlmode_t
//*************************************************/
//uint8_t Vision_CtrlMode(void)
//{
//    return vision_ctrl_mode;
//}

///*************************************************
// * Function: Vision_PositionCtrl_Pointer
// * Description: 获取视觉控制数据
// * Input: 无
// * Return: 视觉位置控制指针
//*************************************************/
//ext_vision_position_ctrl_t* Vision_PositionCtrl_Pointer(void)
//{
//    return &vision_position_ctrl;
//}


///*************************************************
// * Function: Vision_IncrementalCtrl_Pointer
// * Description: 获取视觉控制数据
// * Input: 无
// * Return: 视觉增量控制指针
//*************************************************/
//ext_vision_incremental_ctrl_t* Vision_IncrementalCtrl_Pointer(void)
//{
//    return &vision_incremental_ctrl;
//}

