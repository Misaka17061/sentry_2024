/* 包含头文件 ----------------------------------------------------------------*/
#include "start_task.h"
#include "cmsis_os.h"
#include "infantry_def.h"
#include "app_init.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t start_task_stack = 0;
#endif

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/**
  * @brief  Function implementing the startTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartTask(void const*argument)
{
    AppInit();
    for(;;)
    {
        osDelay(START_TASK_PERIOD);
#if INCLUDE_uxTaskGetStackHighWaterMark
        start_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
