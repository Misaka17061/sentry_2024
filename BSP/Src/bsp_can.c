/*******************************************************************************
 * Robofuture RM Team
 * File name: bsp_can.c
 * Author: Zhb        Version: 1.0        Date: 2021/3/12
 * Description: 对CAN的收发进行封装，实现快速的接收与发送操作，需在CubeMX中配置CAN
  *                             的接收与发送回调
 * Function List:
 *   1. BSP_CAN_NewObject 用于创建CAN收发管理对象
 *   2. BSP_CAN_SetRxCallback 设置接收数据后触发的回调函数
 *   3. BSP_CAN_WriteData CAN发送数据写入
 *   4. BSP_CAN_TransmitData CAN发送数据写入
 * History:
 *      <author> <time>  <version > <desc>
 *        Zhb   21/03/12  1.0       首次提交
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_can.h"
#include <stdbool.h>
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define CAN_DEVICE  2 // CAN设备总数

/* 私有变量 ------------------------------------------------------------------*/
/*
 * 保存创建的CAN对象指针
 */
static CAN_Object_t* m_objects[CAN_DEVICE];

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void CAN_TransmitHandler(CAN_Object_t* obj);

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: BSP_CAN_FilterConfig
 * Description: 配置CAN过滤器
 * Input: hcan CAN指针
 *        filter_bank 过滤器组
 * Return: 无
*************************************************/
void BSP_CAN_FilterConfig(CAN_HandleTypeDef* hcan, uint32_t filter_bank)
{
    CAN_FilterTypeDef  filter_config;
    filter_config.FilterBank = filter_bank;
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_config.FilterIdHigh = 0x0000;
    filter_config.FilterIdLow = 0x0000;
    filter_config.FilterMaskIdHigh = 0x0000;
    filter_config.FilterMaskIdLow = 0x0000;
    filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter_config.FilterActivation = ENABLE;
    filter_config.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(hcan, &filter_config) != HAL_OK)
    {
        Error_Handler();
    }
}

/*************************************************
 * Function: BSP_CAN_Init
 * Description: 初始化CAN设备，并启动中断
 * Input: hcan CAN指针
 *        active_it 启动那些中断中断，从以下中选择
 *                  CAN_IT_TX_MAILBOX_EMPTY     发送邮箱空
 *                  CAN_IT_RX_FIFO0_MSG_PENDING
 *                  CAN_IT_RX_FIFO0_FULL
 *                  CAN_IT_RX_FIFO0_OVERRUN
 *                  CAN_IT_RX_FIFO1_MSG_PENDING
 *                  CAN_IT_RX_FIFO1_FULL
 *                  CAN_IT_RX_FIFO1_OVERRUN
 *                  CAN_IT_WAKEUP
 *                  CAN_IT_SLEEP_ACK
 *                  CAN_IT_ERROR_WARNING
 *                  CAN_IT_ERROR_PASSIVE
 *                  CAN_IT_BUSOFF
 *                  CAN_IT_LAST_ERROR_CODE
 *                  CAN_IT_ERROR
 * Return: 无
*************************************************/
void BSP_CAN_Init(CAN_HandleTypeDef* hcan, uint32_t active_it)
{
    if (hcan->Instance == CAN1)
    {
    	BSP_CAN_FilterConfig(hcan, 0);
    }
#ifdef CAN2
    else if (hcan->Instance == CAN2)
    {
    	BSP_CAN_FilterConfig(hcan, 14);
    }
#endif // CAN2
    
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        Error_Handler();
    }
    
    if (HAL_CAN_ActivateNotification(hcan, active_it) != HAL_OK)
    {
        Error_Handler();
    }
}

/*************************************************
 * Function: BSP_CAN_NewObject
 * Description: 创建CAN收发管理对象
 * Input: obj 对象指针
 *        hcan CAN指针
 *        tx_fifo_buff 发送FIFO缓存
 *        fun 接收回调
 * Return: BSP_OK 对象创建成功
 *         BSP_ERROR 对象创建失败
*************************************************/
BSP_Status_e BSP_CAN_NewObject(CAN_Object_t* obj, CAN_HandleTypeDef* hcan, uint8_t* tx_fifo_buff, BSP_CAN_RxCallback_t fun)
{
    if (obj == NULL)
		return BSP_ERROR;

    obj->hcan = hcan;
    obj->is_sending = 0;
    obj->tx_fifo_buffer = tx_fifo_buff;
    obj->rx_callback = fun;
    fifo_init(&(obj->tx_fifo), tx_fifo_buff, sizeof(CAN_TxMsg_t), CAN_TX_FIFO_UNIT_NUM);
    
    for (uint8_t i=0; i < CAN_DEVICE; i++)
    {
        if (m_objects[i] == NULL)
        {
        	BSP_CAN_Init(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
            m_objects[i] = obj;
            return BSP_OK;
        }
    }
    return BSP_ERROR;
}

/*************************************************
 * Function: BSP_CAN_SetRxCallback
 * Description: 设置接收回调
 * Input: obj 对象指针
 *        fun 接收回调
 * Return: BSP_OK 设置成功
 *         BSP_ERROR 设置失败
*************************************************/
BSP_Status_e BSP_CAN_SetRxCallback(CAN_Object_t* obj, BSP_CAN_RxCallback_t fun)
{
    if (obj == NULL)
        return BSP_ERROR;
    if (fun == NULL)
        return BSP_ERROR;

    obj->rx_callback = fun;
    return BSP_OK;
}

/*************************************************
 * Function: BSP_CAN_WriteData
 * Description: CAN发送数据写入
 * Input: hcan CAN指针，选择 hcan1 或 hcan2
 *        std_id CAN发送标识符
 *        data 发送数据指针
 *        len 发送长度
 * Return: BSP_OK 数据写入成功
 *         BSP_ERROR 数据写入失败
*************************************************/
BSP_Status_e BSP_CAN_WriteData(CAN_HandleTypeDef* hcan, uint32_t std_id, uint8_t* data, uint16_t len)
{
    if(hcan == NULL)
        return BSP_ERROR;

    for (uint8_t i=0; i < CAN_DEVICE; i++)
    {
        if (m_objects[i]->hcan->Instance == hcan->Instance)
        {
            return BSP_CAN_TransmitData(m_objects[i], std_id, data, len);
        }
    }
    return BSP_ERROR;
}

/*************************************************
 * Function: BSP_CAN_TransmitData
 * Description: CAN发送数据写入
 * Input: obj 对象指针
 *        std_id CAN发送标识符
 *        data 发送数据指针
 *        len 发送长度
 * Return: BSP_OK 数据写入成功
 *         BSP_ERROR 数据写入失败
*************************************************/
BSP_Status_e BSP_CAN_TransmitData(CAN_Object_t* obj, uint32_t std_id, uint8_t* data, uint16_t len)
{
    BSP_Status_e res = BSP_ERROR;
    uint8_t *send_ptr;
    uint16_t send_num;
    CAN_TxMsg_t msg;

    send_ptr = data;
    msg.std_id = std_id;
    send_num = 0;

    CRITICAL_SETCION_ENTER();
    while (send_num < len)
    {
        if (fifo_is_full(&(obj->tx_fifo)))
        {
            //can is error
            obj->is_sending = 0;
            break;
        }

        if (len - send_num >= 8)
        {
            msg.dlc = 8;
        }
        else
        {
            msg.dlc = len - send_num;
        }

        /* Copy data */
//		memcpy(msg.data, data, msg.dlc);
        *((uint32_t *)(msg.data)) = *((uint32_t *)(send_ptr));
        *((uint32_t *)(msg.data + 4)) = *((uint32_t *)(send_ptr + 4));

        send_ptr += msg.dlc;
        send_num += msg.dlc;

        fifo_put_noprotect(&(obj->tx_fifo), &msg);
        res = BSP_OK;
    }
    CRITICAL_SETCION_EXIT();

    if ((obj->is_sending) == 0 && (!(fifo_is_empty(&(obj->tx_fifo)))))
    {
        CAN_TransmitHandler(obj);
    }
    return res;
}

/*************************************************
 * Function: CAN_TransmitHandler
 * Description: CAN发送数据处理器，如果发送FIFO中有数据
  *                            会自动发送
 * Input: obj 对象指针
 * Return: 无
*************************************************/
static void CAN_TransmitHandler(CAN_Object_t* obj)
{
    CAN_TxMsg_t msg;
    CAN_TxHeaderTypeDef   header;
    uint32_t              mailbox;

    if(obj == NULL)
        return;
    CRITICAL_SETCION_ENTER();
    if (!fifo_is_empty(&(obj->tx_fifo)))
    {
        while (HAL_CAN_GetTxMailboxesFreeLevel(obj->hcan) && (!(fifo_is_empty(&(obj->tx_fifo)))))
        {
            fifo_get_noprotect(&(obj->tx_fifo), &msg);
            header.StdId = msg.std_id;
            header.IDE = CAN_ID_STD;
            header.RTR = CAN_RTR_DATA;
            header.DLC = msg.dlc;
            obj->is_sending = 1;
            HAL_CAN_AddTxMessage(obj->hcan, &header, msg.data, &mailbox);
        }
    }
    else
    {
        obj->is_sending = 0;
    }
    CRITICAL_SETCION_EXIT();
}

/*************************************************
 * Function: HAL_CAN_TxMailbox0CompleteCallback
 * Description: CAN发送邮箱0完成回调
 * Input: hcan CAN指针
 * Return: 无
 * Others: 中断函数
*************************************************/
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan)
{
    for (uint8_t i=0; (i < CAN_DEVICE); i++)
    {
        if (m_objects[i]->hcan->Instance == hcan->Instance)
        {
            CAN_TransmitHandler(m_objects[i]);
        }
    }
}

/*************************************************
 * Function: HAL_CAN_TxMailbox1CompleteCallback
 * Description: CAN发送邮箱1完成回调
 * Input: hcan CAN指针
 * Return: 无
 * Others: 中断函数
*************************************************/
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan)
{
    for (uint8_t i=0; (i < CAN_DEVICE); i++)
    {
        if (m_objects[i]->hcan->Instance == hcan->Instance)
        {
            CAN_TransmitHandler(m_objects[i]);
        }
    }
}

/*************************************************
 * Function: HAL_CAN_TxMailbox2CompleteCallback
 * Description: CAN发送邮箱2完成回调
 * Input: hcan CAN指针
 * Return: 无
 * Others: 中断函数
*************************************************/
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan)
{
    for (uint8_t i=0; (i < CAN_DEVICE); i++)
    {
        if (m_objects[i]->hcan->Instance == hcan->Instance)
        {
            CAN_TransmitHandler(m_objects[i]);
        }
    }
}

/*************************************************
 * Function: HAL_CAN_ErrorCallback
 * Description: CAN异常回调
 * Input: hcan CAN指针
 * Return: 无
 * Others: 中断函数
*************************************************/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
{
    HAL_CAN_ResetError(hcan);
}

/*************************************************
 * Function: HAL_CAN_RxFifo0MsgPendingCallback
 * Description: CAN接收FIFO0回调
 * Input: hcan CAN指针
 * Return: 无
 * Others: 中断函数
*************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef   rx_header;
    uint8_t               rx_data[8];
    /* Get RX message */
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    
    for (uint8_t i=0; i < CAN_DEVICE; i++)
    {
        if (m_objects[i]->hcan->Instance == hcan->Instance)
        {
            if (m_objects[i]->rx_callback != NULL)
                m_objects[i]->rx_callback(rx_header.StdId, rx_data, rx_header.DLC);
        }
    }
}

/*************************************************
 * Function: HAL_CAN_RxFifo1MsgPendingCallback
 * Description: CAN接收FIFO1回调
 * Input: hcan CAN指针
 * Return: 无
 * Others: 中断函数
*************************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef   rx_header;
    uint8_t               rx_data[8];
    /* Get RX message */
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    for (uint8_t i=0; i < CAN_DEVICE; i++)
    {
        if (m_objects[i]->hcan->Instance == hcan->Instance)
        {
            if (m_objects[i]->rx_callback != NULL)
                m_objects[i]->rx_callback(rx_header.StdId, rx_data, rx_header.DLC);
        }
    }
}

