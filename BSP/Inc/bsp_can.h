#ifndef BSP_CAN_H
#define BSP_CAN_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "bsp_def.h"
#include "can.h"
#include "fifo.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef void (*BSP_CAN_RxCallback_t)(uint32_t std_id, uint8_t* data, uint32_t dlc);

typedef struct 
{
    /* Handle */
    CAN_HandleTypeDef* hcan;
    /* Tx */
    fifo_t tx_fifo;
    uint8_t* tx_fifo_buffer;
    uint8_t is_sending;
    /* Rx */
    BSP_CAN_RxCallback_t rx_callback;
}CAN_Object_t;

typedef struct
{
    uint32_t std_id;
    uint8_t dlc;
    uint8_t data[8];
}CAN_TxMsg_t;

/* 宏定义 --------------------------------------------------------------------*/
#define CAN_TX_FIFO_UNIT_NUM (256)
#define CAN_TX_FIFO_SIZE (CAN_TX_FIFO_UNIT_NUM * sizeof(CAN_TxMsg_t))

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void BSP_CAN_Init(CAN_HandleTypeDef* hcan, uint32_t active_it);
BSP_Status_e BSP_CAN_NewObject(CAN_Object_t* obj, CAN_HandleTypeDef* hcan, uint8_t* tx_fifo_buff, BSP_CAN_RxCallback_t fun);
BSP_Status_e BSP_CAN_SetRxCallback(CAN_Object_t* obj, BSP_CAN_RxCallback_t fun);
BSP_Status_e BSP_CAN_WriteData(CAN_HandleTypeDef* hcan, uint32_t std_id, uint8_t* data, uint16_t len);
BSP_Status_e BSP_CAN_TransmitData(CAN_Object_t* obj, uint32_t std_id, uint8_t* data, uint16_t len);

#endif  // BSP_CAN_H

