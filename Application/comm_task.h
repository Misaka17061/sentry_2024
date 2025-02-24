#ifndef COMM_TASK_H
#define COMM_TASK_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"
#include "comm_protocol.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
    uint16_t        cmd;
    void*           data;
    uint16_t        size;
    uint16_t        interval_time;
} Send_CMD_Config_t;

typedef struct
{
    uint16_t        cmd;
    void*           data;
    uint16_t        size;
    uint16_t        ticks_cnt;
    uint16_t        ticks_time;
} Send_CMD_Info_t;

#define SEND_CMD_MAX_NUM       (20)
typedef struct
{
    TransmitHandle_t* handle;
    uint8_t         header_sof;
    Send_CMD_Info_t config[SEND_CMD_MAX_NUM];
    uint16_t        period_time;
} Send_CMD_Handle_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void Comm_TaskInit(void);


#endif  // COMM_TASK_H

