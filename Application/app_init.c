/* 包含头文件 ----------------------------------------------------------------*/
#include "app_init.h"
#include "Buzzer/buzzer.h"
#include "imu_task.h"
#include "comm_task.h"
#include "timer_task.h"
#include "detect_task.h"

#include "chassis/chassis_app.h"
#include "chassis/chassis_task.h"
#include "gimbal/gimbal_app.h"
#include "gimbal/gimbal_task.h"
#include "gimbal/shoot_task.h"


/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
AppType_e app_type;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
void StartMusic(void);

/* 函数体 --------------------------------------------------------------------*/
void AppInit(void)
{
    BSP_Init();
	  //可根据需要加入配置（部分需提前在CUBE-MX内完成配置，详见BSP各.c文件说明）

    if (BSP_GPIO_ReadPin(&app_gpio))
    {
        app_type = GIMBAL_APP;
    }
    else
    {
        app_type = CHASSIS_APP;
    }
		
		if (0)
		{
				app_type = GIMBAL_OTHER_APP;
		}

    if (app_type == GIMBAL_APP)
    {
        HAL_Delay(1000);
    }
		if (app_type == GIMBAL_OTHER_APP)
		{
				HAL_Delay(3000);
		}
    //识别管脚执行底盘和云台，执行对应的初始化任务，第二云台需要手动改
		
    SoftwareTimerTaskInit();
    IMU_TaskInit();
    ConsoleTaskInit();
    Comm_TaskInit();
    DetectTaskInit();

    if (app_type == GIMBAL_APP)
    {
        GimbalAppConfig();
        ShootTaskInit();
        GimbalTaskInit();
	    StartMusic();
		    //启动音乐提示上电
    }
    else if (app_type == CHASSIS_APP)
    {
        ChassisAppConfig();
        ChassisTaskInit();
    }
		else if (app_type == GIMBAL_OTHER_APP)
		{
				GimbalOtherAppConfig();
			  ShootOtherTaskInit();
        GimbalOtherTaskInit();
			StartMusic();
				//启动音乐提示上电
		}
}

AppType_e GetAppType(void)
{
    return app_type;
}


void jbit(M x,uint16_t z)
{
	Buzzer_SetBeep(x,20);
	HAL_Delay(z);
	Buzzer_SetBeep(0,0);
	HAL_Delay(1);
}

void StartMusic(void)
{
    Buzzer_SetBeep(HDO, 150);   //蜂鸣器	
    HAL_Delay(100);            	//延时
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
    Buzzer_SetBeep(HDO, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
    Buzzer_SetBeep(bXI, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
	  Buzzer_SetBeep(HDO, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(200);
	  Buzzer_SetBeep(SO, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(200);
		Buzzer_SetBeep(SO, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
		Buzzer_SetBeep(HDO, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
		HAL_Delay(100);
		Buzzer_SetBeep(HFA, 150);
		HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
		HAL_Delay(100);
		Buzzer_SetBeep(HMI, 150);
		HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
		HAL_Delay(100);
		Buzzer_SetBeep(HDO, 150);
		HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
   
}
