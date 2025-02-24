/* 包含头文件 ----------------------------------------------------------------*/
#include "timer_task.h"
#include "soft_timer.h"
#include "cmsis_os.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
osThreadId TimerTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t timer_task_stack = 0;
#endif

SoftwareTimer_t soft_timer[TIMER_ELEMENT_NUM_MAX + 1];

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void SoftwareTimerTask(void const*argument)
{
    uint32_t period = osKernelSysTick();

    for(;;)
    {
        TimerISR_Hook();

        for (int i = 1; i < TIMER_ELEMENT_NUM_MAX + 1; i++)
        {
            if ((soft_timer[i].id != 0) && (soft_timer[i].callback != NULL))
            {
                if (soft_timer_check(soft_timer[i].id) == SOFT_TIMER_TIMEOUT)
                {
                    soft_timer[i].callback(soft_timer[i].argc);

                    soft_timer_update(soft_timer[i].id, soft_timer[i].ticks);
                }
            }
        }
        osDelayUntil(&period, 1);
#if INCLUDE_uxTaskGetStackHighWaterMark
        timer_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void SoftwareTimerTaskInit(void)
{
    soft_timer_init();
    osThreadDef(timer_task, SoftwareTimerTask, osPriorityNormal, 0, 1024);
    TimerTaskHandle = osThreadCreate(osThread(timer_task), NULL);
}

int32_t SoftwareTimerRegister(SoftTimer_Callback_t callback, void *argc, uint32_t ticks)
{
    for (int i = 1; i < TIMER_ELEMENT_NUM_MAX + 1; i++)
    {
        if (soft_timer[i].id == 0)
        {
            soft_timer[i].id = soft_timer_req(ticks);
            soft_timer[i].ticks = ticks;
            soft_timer[i].argc = argc;
            soft_timer[i].callback = callback;
            return i;
        }
    }
    return 0;
}

