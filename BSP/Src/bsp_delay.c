/*******************************************************************************
 * Robofuture RM Team
 * File name: bsp_delay.c
 * Author: Zhb        Version: 1.0        Date: 2021/3/12
 * Description: 为其他BSP程序提供延时函数
 * Function List:
 *   1. BSP_DelayInit 对模块中使用到的常量进行初始化
 *   2. BSP_DelayUs us级延时函数
 *   3. BSP_DelayMs ms级延时函数
 *   4. BSP_GetTime_ms 获取当前系统运行时间
 * History:
 *      <author> <time>  <version > <desc>
 *        Zhb   21/03/12  1.0       首次提交
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_delay.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
static uint8_t fac_us = 0;
static uint32_t fac_ms = 0;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: BSP_DelayInit
 * Description: 初始化延时参数
 * Return: 无
*************************************************/
void BSP_DelayInit(void)
{
    fac_us = SystemCoreClock / 1000000;
    fac_ms = SystemCoreClock / 1000;
}

/*************************************************
 * Function: BSP_DelayUs
 * Description: BSP模块us级延时
 * Input: nus 需要延时的时间
 * Return: 无
*************************************************/
void BSP_DelayUs(uint16_t nus)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nus * fac_us;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

/*************************************************
 * Function: BSP_DelayMs
 * Description: BSP模块ms级延时
 * Input: nms 需要延时的时间
 * Return: 无
*************************************************/
void BSP_DelayMs(uint16_t nms)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nms * fac_ms;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

/*************************************************
 * Function: BSP_GetTime_ms
 * Description: 获取系统运行时间 ms级
 * Return: 已运行的时间
*************************************************/
uint32_t BSP_GetTime_ms(void)
{
    return HAL_GetTick();
}


