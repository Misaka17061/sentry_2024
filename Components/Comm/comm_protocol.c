/*******************************************************************************
 * Robofuture RM Team
 * File name: comm_protocol.c
 * Author: Zhb        Version: 1.0        Date: 2021/4/6
 * Description: 实现pid算法
 * Function List:
 *   1. Comm_ReceiveInit 通信接收句柄初始化
 *   2. Comm_ReceiveData 接收数据写入fifo队列
 *   3. Comm_ReceiveDataHandler 接收数据处理
 *   4. Comm_TransmitInit 通信发送句柄初始化
 *   5. Comm_TransmitData 发送数据写入fifo队列
 *   6. Comm_TransmitDataHandler 发送数据处理
 * History:
 *      <author> <time>  <version > <desc>
 *        Zhb   21/04/06  1.0       首次提交
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "comm_protocol.h"
#include "crc.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
static LIST_HEAD(receive_head); //接收链表头
static LIST_HEAD(transmit_head); //发送链表头
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void Comm_Unpack(ReceiveHandle_t* p_handle);
static void UnpackDataSolve(ReceiveHandle_t* p_handle, uint8_t* p_frame);

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: Comm_Unpack
 * Description: 通信协议解包
 * Input: p_handle 接收句柄指针
 * Return: 无
*************************************************/
static void Comm_Unpack(ReceiveHandle_t* p_handle)
{
    uint8_t byte = 0;
    while (fifo_s_used(&p_handle->fifo))
    {
        byte = fifo_s_get(&p_handle->fifo);//取该接收结构体缓存区字节类型变量，即头帧
        switch (p_handle->unpack_step)//判断解包步骤，初始皆为0
        {
            case STEP_HEADER_SOF://0
            {
                if (byte == p_handle->header_sof)//判断头帧是否正确
                {
                    p_handle->unpack_step = STEP_LENGTH_LOW;//更新解包步骤
                    p_handle->protocol_packet[p_handle->index++] = byte;//收录数据，index+1
                }
                else
                {
                    p_handle->index = 0;//头帧错误，index归零
                }
            }
            break;
            case STEP_LENGTH_LOW://处理数据长度的低位字节
            {
                p_handle->data_len = byte;
                p_handle->protocol_packet[p_handle->index++] = byte;//收录数据，index+1
                p_handle->unpack_step = STEP_LENGTH_HIGH;//更新解包步骤
            }
            break;

            case STEP_LENGTH_HIGH://处理数据长度的高位字节
            {
                p_handle->data_len |= (byte << 8);
                p_handle->protocol_packet[p_handle->index++] = byte;

                if (p_handle->data_len < (PROTOCOL_FRAME_MAX_SIZE - HEADER_CRC_CMDID_LEN))
                {
                    p_handle->unpack_step = STEP_FRAME_SEQ;
                }
                else
                {
                    p_handle->unpack_step = STEP_HEADER_SOF;
                    p_handle->index = 0;
                }
            }
            break;

            case STEP_FRAME_SEQ:
            {
                p_handle->protocol_packet[p_handle->index++] = byte;
                p_handle->unpack_step = STEP_HEADER_CRC8;//更新解包步骤：CRC8校验
            }
            break;

            case STEP_HEADER_CRC8:
            {
                p_handle->protocol_packet[p_handle->index++] = byte;

                if (p_handle->index == PROTOCOL_HEADER_SIZE)
                {
                    if (verify_crc8_check_sum(p_handle->protocol_packet, PROTOCOL_HEADER_SIZE))
                    {
                        p_handle->unpack_step = STEP_DATA_CRC16;
                    }
										//判断CRC8位与传输过来的是否相同
                    else
                    {
                        p_handle->unpack_step = STEP_HEADER_SOF;
                        p_handle->index = 0;
                   }
                }
            }
            break;

            case STEP_DATA_CRC16:
            {
                if (p_handle->index < (HEADER_CRC_CMDID_LEN + p_handle->data_len))
                {
									p_handle->protocol_packet[p_handle->index++] = byte;//小于则继续加入缓存区，即数据未读取完毕
                }
                if (p_handle->index >= (HEADER_CRC_CMDID_LEN + p_handle->data_len))
                {
                    p_handle->unpack_step = STEP_HEADER_SOF;
                    p_handle->index = 0;

                    if (verify_crc16_check_sum(p_handle->protocol_packet, HEADER_CRC_CMDID_LEN + p_handle->data_len))
                    {
                        UnpackDataSolve(p_handle, p_handle->protocol_packet);//解包
                    }
                }
            }
            break;

            default:
            {
                p_handle->unpack_step = STEP_HEADER_SOF;
                p_handle->index = 0;
            }
            break;
        }
    }
}

/*************************************************
 * Function: UnpackDataSolve
 * Description: 解包后处理调用
 * Input: p_handle 接收句柄指针
 *        p_frame 数据帧
 * Return: 无
*************************************************/
static void UnpackDataSolve(ReceiveHandle_t* p_handle, uint8_t* p_frame)
{
    FrameHeader_t *p_header = (FrameHeader_t*)p_frame;

//    uint16_t sof         = p_header->sof;
    uint16_t data_length = p_header->data_length;
    uint16_t cmd_id      = *(uint16_t *)(p_frame + PROTOCOL_HEADER_SIZE);
    uint8_t* data_addr   = p_frame + PROTOCOL_HEADER_SIZE + PROTOCOL_CMD_SIZE;//
//根据协议包头部定义中的命令标识长度（在这里定义为固定值PROTOCOL_CMD_SIZE）和指向协议包缓冲区的指针，计算出指向数据域的指针data_addr
    if(p_handle->func != NULL)
    {
			p_handle->func(cmd_id, data_addr, data_length);//执行相应的回调函数，使用memcpy内存复制至结构体内
    }
}

/*************************************************
 * Function: Comm_Pack
 * Description: 通信协议打包
 * Input: p_handle 发送句柄指针
 *        sof 帧头
 *        cmd_id 命令码
 *        p_data 数据指针
 *        len 数据长度
 * Return: 帧长度
*************************************************/
static uint16_t Comm_Pack(TransmitHandle_t* p_handle, uint8_t sof, uint16_t cmd_id, uint8_t* p_data, uint16_t len)
{
    uint16_t frame_length = PROTOCOL_HEADER_SIZE + PROTOCOL_CMD_SIZE + len + PROTOCOL_CRC16_SIZE;
    if (frame_length > PROTOCOL_FRAME_MAX_SIZE)
        return 0;
    FrameHeader_t *p_header = (FrameHeader_t*)(p_handle->protocol_packet);

    p_header->sof          = sof;
    p_header->data_length  = len;
//    p_header->seq          = p_handle->seq++;
    memcpy(p_handle->protocol_packet + PROTOCOL_HEADER_SIZE, (uint8_t*)&cmd_id, PROTOCOL_CMD_SIZE);
    append_crc8_check_sum(p_handle->protocol_packet, PROTOCOL_HEADER_SIZE);
    memcpy(p_handle->protocol_packet + PROTOCOL_HEADER_SIZE + PROTOCOL_CMD_SIZE, p_data, len);
    append_crc16_check_sum(p_handle->protocol_packet, frame_length);

    return frame_length;
}

/*************************************************
 * Function: Comm_ReceiveInit
 * Description: 通信接收句柄初始化
 * Input: p_handle 发送句柄指针
 *        header_sof 帧头
 *        rx_fifo_buffer 接收fifo缓存指针
 *        rx_fifo_size 接收fifo字节长
 *        func 接收处理钩子函数
 * Return: 无
*************************************************/
void Comm_ReceiveInit(ReceiveHandle_t* p_handle, uint8_t header_sof, uint8_t* rx_fifo_buffer, uint16_t rx_fifo_size, UnpackHookFunc_t func)
{
    if (p_handle == NULL)
        return;
    list_t* cur;
    list_for_each_prev(cur, &receive_head)
    {
        if (cur == &p_handle->list)
            return;
    }
    list_add(&p_handle->list, &receive_head);
    p_handle->header_sof = header_sof;
    fifo_s_init(&p_handle->fifo, rx_fifo_buffer, rx_fifo_size);
    p_handle->func = func;
}

/*************************************************
 * Function: Comm_ReceiveData
 * Description: 接收数据写入fifo队列
 * Input: p_handle 接收句柄指针
 *        p_data 数据指针
 *        len 数据长度
 * Return: 无
*************************************************/
void Comm_ReceiveData(ReceiveHandle_t* p_handle, uint8_t* p_data, uint16_t len)
{
    if (p_handle == NULL)
        return;

    if (fifo_s_free(&p_handle->fifo) > len)
        fifo_s_puts(&p_handle->fifo, (char *)p_data, len);
}

/*************************************************
 * Function: Comm_ReceiveDataHandler
 * Description: 接收数据处理
 * Input: 无
 * Return: 无
*************************************************/
void Comm_ReceiveDataHandler(void)
{
    ReceiveHandle_t* p_handle;
    list_t* cur_pos;
    list_for_each_prev(cur_pos, &receive_head)
    {
        p_handle = (ReceiveHandle_t *)cur_pos;
        Comm_Unpack(p_handle);
    }
}

/*************************************************
 * Function: Comm_TransmitInit
 * Description: 通信发送句柄初始化
 * Input: p_handle 发送句柄指针
 *        tx_fifo_buffer 发送fifo缓存指针
 *        tx_fifo_size 发送fifo字节长
 *        func 发送处理钩子函数
 * Return: 无
*************************************************/
void Comm_TransmitInit(TransmitHandle_t* p_handle, uint8_t* tx_fifo_buffer, uint16_t tx_fifo_size, TransmitHookFunc_t func)
{
    if (p_handle == NULL)
        return;
    list_t* cur;
    list_for_each_prev(cur, &transmit_head)
    {
        if (cur == &p_handle->list)
            return;
    }
    list_add(&p_handle->list, &transmit_head);
    fifo_s_init(&p_handle->fifo, tx_fifo_buffer, tx_fifo_size);
    p_handle->func = func;
}

/*************************************************
 * Function: Comm_TransmitData
 * Description: 通信发送句柄初始化
 * Input: p_handle 发送句柄指针
 *        header_sof 发送帧头
 *        cmd_id 协议命令码
 *        p_data 数据帧头
 *        len 数据长度
 * Return: 无
*************************************************/
void Comm_TransmitData(TransmitHandle_t* p_handle, uint8_t header_sof, uint16_t cmd_id, uint8_t* p_data, uint16_t len)
{
    if (p_handle == NULL)
        return;

    if (fifo_s_free(&p_handle->fifo) > len)
    {
        uint16_t frame_length = Comm_Pack(p_handle, header_sof, cmd_id, p_data, len);
        fifo_s_puts(&p_handle->fifo, (char*)p_handle->protocol_packet, frame_length);
    }
}

/*************************************************
 * Function: Comm_TransmitData_Vision
 * Description: 视觉通信发送句柄初始化
 * Input: p_handle 发送句柄指针
 *        header_sof 发送帧头
 *        cmd_id 协议命令码
 *        p_data 数据帧头
 *        len 数据长度
 * Return: 无
*************************************************/
void Comm_TransmitData_Vision(TransmitHandle_t* p_handle, uint8_t* p_data, uint16_t len)
{
    if (p_handle == NULL)
        return;

    if (fifo_s_free(&p_handle->fifo) > len)
    {
        fifo_s_puts(&p_handle->fifo, (char*)p_data, len);
    }
}

/*************************************************
 * Function: Comm_TransmitDataHandler
 * Description: 通信发送数据处理
 * Input: 无
 * Return: 无
*************************************************/
void Comm_TransmitDataHandler(void)
{
    uint8_t buff[PROTOCOL_FRAME_MAX_SIZE] = {0};
    TransmitHandle_t* p_handle;
    list_t* cur_pos;
    uint16_t used_len = 0;
    uint16_t free_len = 0;
    uint16_t send_len = 0;

    list_for_each_prev(cur_pos, &transmit_head)
    {
        p_handle = (TransmitHandle_t*)cur_pos;
        used_len = fifo_s_used(&p_handle->fifo);
        if (used_len)
        {
            if (p_handle->get_free_func != NULL)
            {
                free_len = p_handle->get_free_func();
                send_len = (free_len < used_len) ? free_len : used_len;
            }
            else
            {
                send_len = used_len;
            }

            if (send_len > PROTOCOL_FRAME_MAX_SIZE)
            {
                send_len = PROTOCOL_FRAME_MAX_SIZE;
            }

            fifo_s_gets(&p_handle->fifo, (char *)buff, send_len);
            if(p_handle->func != NULL)
            {
                p_handle->func(buff, send_len);
            }
        }
    }
}

