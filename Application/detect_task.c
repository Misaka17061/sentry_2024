/* 包含头文件 ----------------------------------------------------------------*/
#include "detect_task.h"
#include "cmsis_os.h"
#include "infantry_def.h"
#include "bsp_init.h"
#include "Buzzer/buzzer.h"
#include "RGB_Led/rgb_led.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
osThreadId DetectTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t detect_task_stack = 0;
#endif

OfflineHandle_t offline_handle[OFFLINE_EVENT_MAX_NUM] = {NO_OFFLINE};

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
void BeepHandler(void);
void BeepTimesSet(uint8_t times);

/* 函数体 --------------------------------------------------------------------*/
void DetectTask(void const*argument)
{
    OfflineEvent_e display_event;
    uint32_t time_now;

    for(;;)
    {
        time_now = BSP_GetTime_ms();
        uint8_t error_level = 0XFF;
        display_event = NO_OFFLINE;
        for (int i = 1; i < OFFLINE_EVENT_MAX_NUM; i++)
        {
            if ((time_now - offline_handle[i].last_time > offline_handle[i].offline_time) && (offline_handle[i].enable))
            {
                offline_handle[i].online_state = OFFLINE_STATE;
                if (error_level > offline_handle[i].error_level)
                {
                    error_level = offline_handle[i].error_level;
                    display_event = (OfflineEvent_e)i;
                }
            }
            else
            {
                offline_handle[i].online_state = ONLINE_STATE;
            }
        }

        if (display_event != NO_OFFLINE)
        {
            BeepTimesSet(offline_handle[display_event].beep_times);
            if (offline_handle[display_event].error_level == OFFLINE_ERROR_LEVEL)
                aRGB_led_show(RED);
            else if (offline_handle[display_event].error_level == OFFLINE_WARNING_LEVEL)
                aRGB_led_show(BLUE);

        }
        else
        {
            aRGB_led_show(GREEN);
            BeepTimesSet(0);
        }

        BeepHandler();
        osDelay(DETECT_TASK_PERIOD);

#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void DetectTaskInit(void)
{
    osThreadDef(detect_task, DetectTask, osPriorityNormal, 0, 256);
    DetectTaskHandle = osThreadCreate(osThread(detect_task), NULL);
}

void OfflineHandle_Init(OfflineEvent_e event, OfflineLevel_e error_level, uint32_t offline_time, uint32_t beep_times)
{
    offline_handle[event].event = event;
    offline_handle[event].enable = ENABLE;
    offline_handle[event].error_level = error_level;
    offline_handle[event].online_state = OFFLINE_STATE;

    offline_handle[event].offline_time = offline_time;
    offline_handle[event].beep_times = beep_times;
}

void OfflineHandle_TimeUpdate(OfflineEvent_e event)
{
    offline_handle[event].last_time = BSP_GetTime_ms();
}

uint8_t CheckDeviceIsOffline(OfflineEvent_e event)
{
    return (offline_handle[event].online_state == OFFLINE_STATE);
}

uint32_t CheckDeviceRunTime(OfflineEvent_e event)
{   
    return offline_handle[event].last_time;
}
