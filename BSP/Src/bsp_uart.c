/*******************************************************************************
 * Robofuture RM Team
 * File name: bsp_uart.c
 * Author: Zhb        Version: 1.0        Date: 2021/3/12
 * Description: 对UART的收发进行封装，实现快速的接收与发送操作，需在CubeMX中配置UART
  *                             的接收与发送回调
 * Function List:
 *   1. BSP_UART_NewObject 创建UART管理对象
 *   2. BSP_UART_TransmitConfig UART发送配置
 *   3. BSP_UART_ReceiveConfig UART接收配置
 *   4. BSP_UART_SetRxCallback UART设置接收回调
 *   5. BSP_UART_WriteData UART发送数据写入
 *   6. BSP_UART_TransmitData UART发送数据写入
 * History:
 *      <author> <time>  <version > <desc>
 *        Zhb   21/03/12  1.0       首次提交
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_uart.h"
#include "usart.h"
#include <stdio.h>

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define UART_DEVICE  3  // 串口设备总数量

/* 私有变量 ------------------------------------------------------------------*/
/*
 * 保存创建的串口对象指针
 */
static UART_Object_t* m_objects[UART_DEVICE];

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void UART_TransmitHandler(UART_Object_t* obj);

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: BSP_UART_NewObject
 * Description: 创建UART收发管理对象
 * Input: obj 串口对象指针
 *        huart UART指针
 * Return: BSP_OK 对象创建成功
 *         BSP_ERROR 对象创建失败
*************************************************/
BSP_Status_e BSP_UART_NewObject(UART_Object_t* obj, UART_HandleTypeDef* huart)
{
    if(obj == NULL)
        return BSP_ERROR;

    memset(obj, 0, sizeof(UART_Object_t));
    obj->huart = huart;
    
    for (uint8_t i=0; i < UART_DEVICE; i++)
    {
        if (m_objects[i] == NULL)
        {
            m_objects[i] = obj;
            return BSP_OK;
        }
    }
    return BSP_ERROR;
}

/*************************************************
 * Function: BSP_UART_TransmitConfig
 * Description: UART发送配置
 * Input: obj 串口对象指针
 *        tx_buffer 发送缓存
 *        tx_buffer_size 发送缓存字节长度
 *        tx_fifo_buffer 发送FIFO缓存
 *        tx_fifo_size 发送FIFO字节长度
 * Return: BSP_OK 配置成功
 *         BSP_ERROR 配置失败
 * Others: FIFO缓存用来存储还未发送的数据
*************************************************/
BSP_Status_e BSP_UART_TransmitConfig(UART_Object_t* obj, uint8_t* tx_buffer, uint16_t tx_buffer_size,
                                       uint8_t* tx_fifo_buffer, uint16_t tx_fifo_size)
{
    if(obj == NULL)
        return BSP_ERROR;

    obj->tx_en = BSP_ENABLE;
    obj->tx_buffer = tx_buffer;         // DMA发送缓存
    obj->tx_buffer_size = tx_buffer_size;
    obj->tx_fifo_buffer = tx_fifo_buffer;
    obj->tx_fifo_size = tx_fifo_size;
    obj->is_sending = 0;
    fifo_s_init(&(obj->tx_fifo), tx_fifo_buffer, tx_fifo_size);

    return BSP_OK;
}

/*************************************************
 * Function: BSP_UART_ReceiveConfig
 * Description: UART接收配置
 * Input: obj 对象指针
 *        rx_buffer 接收缓存
 *        rx_buffer_size 接收缓存字节长度
 *        fun 接收后触发的回调函数
 * Return: BSP_OK 配置成功
 *         BSP_ERROR 配置失败
*************************************************/
BSP_Status_e BSP_UART_ReceiveConfig(UART_Object_t* obj, uint8_t* rx_buffer, uint16_t rx_buffer_size, UART_RxCallback_t fun)
{
    if(obj == NULL)
        return BSP_ERROR;

    obj->rx_en = BSP_ENABLE;
    obj->rx_buffer = rx_buffer;
    obj->rx_buffer_size = rx_buffer_size;
    obj->rx_callback = fun;

    __HAL_UART_CLEAR_IDLEFLAG(obj->huart);
    __HAL_UART_ENABLE_IT(obj->huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(obj->huart, obj->rx_buffer, obj->rx_buffer_size);

    return BSP_OK;
}

/*************************************************
 * Function: BSP_UART_SetRxCallback
 * Description: 设置回调函数
 * Input: obj 对象指针
 *        fun 接收后触发的回调函数
 * Return: BSP_OK 配置成功
 *         BSP_ERROR 配置失败
*************************************************/
BSP_Status_e BSP_UART_SetRxCallback(UART_Object_t* obj, UART_RxCallback_t fun)
{
    if (obj == NULL)
        return BSP_ERROR;
    if (fun == NULL)
        return BSP_ERROR;

    obj->rx_callback = fun;
    return BSP_OK;
}

/*************************************************
 * Function: BSP_UART_WriteData
 * Description: UART发送数据写入
 * Input: huart 串口指针
 *        data 发送数据指针
 *        len 发送数据长度
 * Return: BSP_OK 配置成功
 *         BSP_ERROR 配置失败
*************************************************/
BSP_Status_e BSP_UART_WriteData(UART_HandleTypeDef *huart, uint8_t* data, uint16_t len)
{
    if(huart == NULL)
        return BSP_ERROR;

    for (uint8_t i=0; i < UART_DEVICE; i++)
    {
        if (m_objects[i]->huart->Instance == huart->Instance)
            return BSP_UART_TransmitData(m_objects[i], data, len);
    }
    return BSP_ERROR;
}

/*************************************************
 * Function: BSP_UART_TransmitData
 * Description: UART发送数据写入
 * Input: obj 串口对象指针
 *        data 发送数据指针
 *        len 发送数据长度
 * Return: BSP_OK 配置成功
 *         BSP_ERROR 配置失败
*************************************************/
BSP_Status_e BSP_UART_TransmitData(UART_Object_t* obj, uint8_t* data, uint16_t len)
{
    uint16_t to_send_len;
    uint16_t to_tx_fifo_len;

    if (obj == NULL)
        return BSP_ERROR;
    if (obj->tx_en == BSP_DISABLE)
        return BSP_ERROR;

    if (obj->is_sending == 0)
    {
        if (len < obj->tx_buffer_size)
        {
            to_send_len = len;
            to_tx_fifo_len = 0;
        }
        else if (len < obj->tx_buffer_size + obj->tx_fifo_size)
        {
            to_send_len = obj->tx_buffer_size;
            to_tx_fifo_len = len - obj->tx_buffer_size;
        }
        else
        {
            to_send_len = obj->tx_buffer_size;
            to_tx_fifo_len = obj->tx_fifo_size;
        }
    }
    else
    {
        if (len < obj->tx_fifo_size)
        {
            to_send_len = 0;
            to_tx_fifo_len = len;
        }
        else
        {
            to_send_len = 0;
            to_tx_fifo_len = obj->tx_fifo_size;
        }
    }

    if (to_send_len > 0)
    {
        memcpy(obj->tx_buffer, data, to_send_len);
        obj->is_sending = 1;
        HAL_UART_Transmit_DMA(obj->huart, obj->tx_buffer, to_send_len);
    }

    if (to_tx_fifo_len > 0)
    {
        uint8_t len;
        len = fifo_s_puts(&(obj->tx_fifo), (char *)(data) + to_send_len, to_tx_fifo_len);

        if (len != to_tx_fifo_len)
        {
            return BSP_ERROR;
        }
    }
    return BSP_OK;
}

/*************************************************
 * Function: UART_TransmitHandler
 * Description: 串口数据发送处理器，当FIFO中有数据会
  *                             自动发送
 * Input: obj 串口对象指针
 * Return: 无
*************************************************/
static void UART_TransmitHandler(UART_Object_t* obj)
{
    uint16_t fifo_data_num = 0;
    uint16_t send_num = 0;

    if(obj == NULL)
        return;
    if (obj->tx_en == BSP_DISABLE)
        return;

    fifo_data_num = obj->tx_fifo.used_num;

    if (fifo_data_num != 0)
    {
        if (fifo_data_num < obj->tx_buffer_size)
        {
            send_num = fifo_data_num;
        }
        else
        {
            send_num = obj->tx_buffer_size;
        }
        fifo_s_gets(&(obj->tx_fifo), (char *)(obj->tx_buffer), send_num);
        obj->is_sending = 1;
        HAL_UART_Transmit_DMA(obj->huart, obj->tx_buffer, send_num);
    }
    else
    {
        obj->is_sending = 0;
    }
}

/*************************************************
 * Function: HAL_UART_TxCpltCallback
 * Description: 串口发送完成回调
 * Input: huart 串口指针
 * Return: 无
 * Others: 中断函数
*************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    UART_Object_t *obj = NULL;
    for (uint8_t i=0; i < UART_DEVICE; i++)
    {
        if (m_objects[i]->huart->Instance == huart->Instance)
        {
            obj = m_objects[i];
            obj->is_sending = 0;
            UART_TransmitHandler(obj);
        }
    }
}

/*************************************************
 * Function: HAL_UART_RxCpltCallback
 * Description: 串口接收完成回调函数
 * Input: huart 串口指针
 * Return: 无
 * Others: 中断函数
*************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    uint16_t read_length = 0;
    for (uint8_t i=0; i < UART_DEVICE; i++)
    {
        if (m_objects[i]->huart->Instance == huart->Instance)
        {
            read_length = m_objects[i]->rx_buffer_size - __HAL_DMA_GET_COUNTER(huart->hdmarx);
            /* TODO 暂时不需要在完成中断中处理 */
//           if (m_objects[i]->rx_callback != NULL)
 //              m_objects[i]->rx_callback(m_objects[i]->rx_buffer, read_length);
        }
    }
}

/*************************************************
 * Function: HAL_UART_RxCpltCallback
 * Description: 串口空闲中断回调函数
 * Input: huart 串口指针
 * Return: 无
 * Others: 中断函数
*************************************************/
void BSP_UART_IDLE_Callback(UART_HandleTypeDef* huart)
{
    uint16_t read_length = 0;

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)
            && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
        for (uint8_t i=0; i < UART_DEVICE; i++)
        {
            if (m_objects[i]->huart->Instance == huart->Instance)
            {
                read_length = m_objects[i]->rx_buffer_size - __HAL_DMA_GET_COUNTER(huart->hdmarx);
                if (m_objects[i]->rx_callback != NULL)
                    m_objects[i]->rx_callback(m_objects[i]->rx_buffer, read_length);
                HAL_UART_Receive_DMA(huart, m_objects[i]->rx_buffer, m_objects[i]->rx_buffer_size);
            }
        }
        __HAL_DMA_ENABLE(huart->hdmarx);
    }
    __HAL_UART_CLEAR_IDLEFLAG(huart);
}
