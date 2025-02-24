/* 包含头文件 ----------------------------------------------------------------*/
#include "shoot_task.h"
#include "infantry_def.h"
#include "cmsis_os.h"
#include "math.h"
#include "bsp_init.h"
#include "detect_task.h"
#include "referee_system.h"
#include "gimbal/gimbal_app.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define MAGAZINE_MOTOR_INIT_SET         (1000)
#define MAGAZINE_MOTOR_POSITIVE_DIR     (-1.0f)
#define MAGAZINE_MOTOR_REDUCTION_RATIO  M2006_REDUCTION_RATIO
#define TRIGGER_MOTOR_POSITIVE_DIR      (-1.0f)
#define TRIGGER_MOTOR_REDUCTION_RATIO   M2006_REDUCTION_RATIO

#define LASER_ON()  BSP_GPIO_SetPwmValue(&laser_gpio, 8399)
#define LASER_OFF() BSP_GPIO_SetPwmValue(&laser_gpio, 0)

#define TRIGGER_PLATE_NUMBERS   (8.0f)
#define TRIGGER_BLOCKED_TIMER   (500)
#define REVERSE_TIMER           (300)
#define REVERSE_ANGLE           (15.0f)

#define ONE_BULLET_HEAT         (10u)

/* 私有变量 ------------------------------------------------------------------*/
osThreadId ShootOtherTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t shoot_task_stack = 0;
#endif

ShootHandle_t shoot_other_handle;

static uint16_t shoot_speed_limit[3] = {15, 18, 30};      
static uint16_t friction_wheel_speed[3] = {5900, 6400, 6900}; //4900-16.7m/s  5900-20m/s    7000-24m/s      8000-25m/s//6955   //4480   
                                        //测试500转
//4420,4920,7150

static uint16_t fire_flag=0;
uint16_t shoot_other_flag=0;



static uint16_t fire_flag;

static uint16_t two_interval_time;
static uint16_t two_all_time;
static uint16_t two_last_time;
static uint16_t two_now_time;
/* 扩展变量 ------------------------------------------------------------------*/
extern CAN_Object_t can2_obj;
extern GimbalHandle_t gimbal_other_handle;

/* 私有函数原形 --------------------------------------------------------------*/
static void ShootOtherSensorUpdata(void);
static void ShootOtherCtrlModeSwitch(void);
//static void MagazineOtherCtrlModeSwitch(void);

static void Shoot_Other_LaserCtrl(ShootCtrlMode_e mode);
//static void Shoot_Other_MagazineMotorCtrl(ShootHandle_t* handle);
static void Shoot_Other_TriggerMotorCtrl(ShootHandle_t* handle);
static void Shoot_Other_FrictionWheelMotorCtrl(ShootCtrlMode_e mode, FrictionWheelMotor_t motor[2]);

static void Shoot_HeatingLimit(ShootHandle_t* handle);
//static void Shoot_HeatingTime(ShootHandle_t* handle);

static void ShootOtherMotorSendCurrent(int16_t fric1_cur, int16_t fric2_cur, int16_t trigger_cur, int16_t magazine_cur);

/* 函数体 --------------------------------------------------------------------*/
void ShootOtherTask(void const*argument)
{
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    for(;;)
    {
        ShootOtherSensorUpdata();
        ShootOtherCtrlModeSwitch();
//	    MagazineOtherCtrlModeSwitch();

        Shoot_Other_LaserCtrl(shoot_other_handle.ctrl_mode);
//       Shoot_Other_MagazineMotorCtrl(&shoot_other_handle);
			
			
			//Shoot_HeatingTime(&shoot_other_handle);
		 //Shoot_HeatingLimit(&shoot_other_handle);
			
        Shoot_Other_TriggerMotorCtrl(&shoot_other_handle);
        Shoot_Other_FrictionWheelMotorCtrl(shoot_other_handle.ctrl_mode, shoot_other_handle.fric_wheel_motor);

        if (shoot_other_handle.ctrl_mode == SHOOT_RELAX)
        {
            shoot_other_handle.fric_wheel_motor[0].current_set = 0;
            shoot_other_handle.fric_wheel_motor[1].current_set = 0;
            shoot_other_handle.trigger_motor.current_set = 0;
        }
				
        ShootOtherMotorSendCurrent(shoot_other_handle.fric_wheel_motor[0].current_set,
                              shoot_other_handle.fric_wheel_motor[1].current_set,
                              TRIGGER_MOTOR_POSITIVE_DIR * shoot_other_handle.trigger_motor.current_set,
                              0);
        
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, shoot_other_handle.magazine_pwm);
        osDelay(SHOOT_TASK_PERIOD);

#if INCLUDE_uxTaskGetStackHighWaterMark
        shoot_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void ShootOtherTaskInit(void)
{
    shoot_other_handle.console = Console_Pointer();
    shoot_other_handle.shoot_can = &can2_obj;
	
		shoot_other_handle.vision_tx_data = RobotInfo_Pointer(); 
    shoot_other_handle.magazine_initial_degree = 20;
    shoot_other_handle.magazine_90_degree = 80;
    shoot_other_handle.magazine_motor.motor_info = MagazineMotor_Pointer();
    shoot_other_handle.magazine_state = MAGAZINE_INIT_STATE;
    shoot_other_handle.magazine_motor.ecd_ratio = MAGAZINE_MOTOR_POSITIVE_DIR * MAGAZINE_MOTOR_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
    pid_init(&shoot_other_handle.magazine_motor.pid.outer_pid, POSITION_PID, 2000.0f, 500.0f,
             30.0f, 0.1f, 80.0f);/*弹盖*/
    pid_init(&shoot_other_handle.magazine_motor.pid.inter_pid, POSITION_PID, 3000.0f, 500.0f,
             1.0f, 0.0f, 3.0f);
    Blocked_Reset(&shoot_other_handle.magazine_motor.blocked_handle, 2000, 3000);

    shoot_other_handle.trigger_motor.motor_info = TriggerMotor_Pointer();
    shoot_other_handle.trigger_state = TRIGGER_END;
    shoot_other_handle.trigger_motor.ecd_ratio = TRIGGER_MOTOR_POSITIVE_DIR * TRIGGER_MOTOR_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
    pid_init(&shoot_other_handle.trigger_motor.pid.outer_pid, POSITION_PID, 300.0f, 60.0f,
             14.0f, 0.0f, 20.5f);/*8，0，0*/  /*调速*//*慢加P，抖加D*//*拨弹盘*/  //8,0,25
    pid_init(&shoot_other_handle.trigger_motor.pid.inter_pid, POSITION_PID, M2006_MOTOR_MAX_CURRENT, 7000.0f,
             100.0f, 0.0f, 0.0f);/*100，0，0*/ /*调力*/
    Blocked_Reset(&shoot_other_handle.trigger_motor.blocked_handle, TRIGGER_BLOCKED_TIMER, 1000);

    shoot_other_handle.fric_wheel_motor[0].motor_info = FrictionWheelMotor_1_Pointer();
    shoot_other_handle.fric_wheel_motor[1].motor_info = FrictionWheelMotor_2_Pointer();
    for (uint8_t i = 0; i < 2; i++)/*摩擦轮*/
    {
        pid_init(&shoot_other_handle.fric_wheel_motor[i].pid, POSITION_PID, 10000, 500.0f,
                 40.0f, 0.1f, 15.0f);/*9,0,0*/
    }
    LASER_OFF();
		
		fire_flag=1;

    osThreadDef(shoot_other_task, ShootOtherTask, osPriorityNormal, 0, 256);
    ShootOtherTaskHandle = osThreadCreate(osThread(shoot_other_task), NULL);
}

static void ShootOtherSensorUpdata(void)
{
	
    shoot_other_handle.magazine_motor.speed = (fp32)shoot_other_handle.magazine_motor.motor_info->speed_rpm * MAGAZINE_MOTOR_REDUCTION_RATIO;
    shoot_other_handle.magazine_motor.angle = shoot_other_handle.magazine_motor.ecd_ratio
            * (fp32)(shoot_other_handle.magazine_motor.motor_info->total_ecd - shoot_other_handle.magazine_motor.offset_ecd);

    shoot_other_handle.trigger_motor.speed = (fp32)shoot_other_handle.trigger_motor.motor_info->speed_rpm * TRIGGER_MOTOR_REDUCTION_RATIO * TRIGGER_MOTOR_POSITIVE_DIR;
    shoot_other_handle.trigger_motor.angle = shoot_other_handle.trigger_motor.ecd_ratio
            * (fp32)(shoot_other_handle.trigger_motor.motor_info->total_ecd - shoot_other_handle.trigger_motor.offset_ecd);

    shoot_other_handle.friction_wheel_speed_buff = shoot_other_handle.console->friction_wheel_speed_buff;
    
  	shoot_other_handle.shoot_speed=(fabs(shoot_other_handle.fric_wheel_motor[0].motor_info->speed_rpm/60*FRIC_WHEEL_RADIUS/1000)
	                          +fabs(shoot_other_handle.fric_wheel_motor[1].motor_info->speed_rpm/60*FRIC_WHEEL_RADIUS/1000))/2;  //计算转速
	  
	
    if (!CheckDeviceIsOffline(OFFLINE_REFEREE_SYSTEM))
    {
        shoot_other_handle.shooter_heat_cooling_rate = RefereeSystem_RobotState_Pointer()->shooter_barrel_cooling_value;
        shoot_other_handle.shooter_heat_cooling_limit = RefereeSystem_RobotState_Pointer()->shooter_barrel_heat_limit;			
//			  shoot_handle.shooter_heat_cooling_rate = 10;
//        shoot_handle.shooter_heat_cooling_limit = 50;		            //！！！！！！！！！！！！
//			  shoot_other_handle.shooter_speed_limit = RefereeSystem_RobotState_Pointer()->shooter_id1_17mm_speed_limit;
        shoot_other_handle.shooter_heat = RefereeSystem_PowerHeatData_Pointer()->shooter_id2_17mm_cooling_heat;
			  shoot_other_handle.shooter_speedfeedback=RefereeSystem_SpeedData_Pointer()->initial_speed;
		  	shoot_other_handle.shooter_heat_cooling_rate_k=shoot_other_handle.shooter_heat_cooling_rate;
    }
    else
    {
        shoot_other_handle.shooter_heat_cooling_rate = 0;
        shoot_other_handle.shooter_heat_cooling_limit = 200;
        shoot_other_handle.shooter_heat = 0;
	    shoot_other_handle.shooter_speed_limit=30;
    }

}

static void ShootOtherCtrlModeSwitch(void)
{
    if (shoot_other_handle.console->shoot_other_cmd == SHOOT_RELEASE_CMD)
    {
        shoot_other_handle.ctrl_mode = SHOOT_RELAX;
    }
    else if (shoot_other_handle.console->shoot_other_cmd == SHOOT_START_CMD)
    {
        shoot_other_handle.ctrl_mode = SHOOT_START;
    }
    else if (shoot_other_handle.console->shoot_other_cmd == SHOOT_STOP_CMD)
    {
        shoot_other_handle.ctrl_mode = SHOOT_STOP;
    }
}

//static void MagazineOtherCtrlModeSwitch(void)
//{
//    if (shoot_other_handle.console->magazine_cmd == MAGAZINE_OFF_CMD)
//    {
//        shoot_other_handle.magazine_state = MAGAZINE_OFF_STATE;
//    }
//    else if (shoot_other_handle.console->magazine_cmd == MAGAZINE_ON_CMD)
//    {
//        shoot_other_handle.magazine_state = MAGAZINE_ON_STATE;
//    }
//}

static void Shoot_Other_LaserCtrl(ShootCtrlMode_e mode)
{
    if (mode == SHOOT_START)
    {
        LASER_ON();
    }
    else
    {
        LASER_OFF();
    }
}

//static void Shoot_Other_MagazineMotorCtrl(ShootHandle_t* handle)
//{
//    static fp32 zero_angle = 0.0f;
//    if (handle->magazine_state != MAGAZINE_INIT_STATE)
//    {
//        if (handle->magazine_state== MAGAZINE_OFF_STATE)
//        {
//            handle->magazine_pwm=shoot_other_handle.magazine_initial_degree;
//        }
//        else //if (handle->ctrl_mode == SHOOT_STOP)
//        {
//            handle->magazine_pwm=shoot_other_handle.magazine_90_degree;
//        }
//        
        
//        handle->magazine_motor.current_set = DoublePID_Calc(&handle->magazine_motor.pid,
//                                                            handle->magazine_motor.set_angle,
//                                                            handle->magazine_motor.angle,
//                                                            handle->magazine_motor.speed);
//    }
//    else
//    {
//        handle->magazine_motor.current_set = MAGAZINE_MOTOR_INIT_SET;
//        BlockedState_t blocked = Blocked_Process(&handle->magazine_motor.blocked_handle, handle->magazine_motor.speed);
//        if (blocked == BLOCKED)
//        {
//            handle->magazine_state = MAGAZINE_OFF_STATE;
//					  magazine_other_handle.magazine_state = MAGAZINE_OFF_STATE;
////            zero_angle = handle->magazine_motor.angle;
////            handle->magazine_motor.set_angle = handle->magazine_motor.angle;
//        }
//    }
//}

static void Shoot_Other_TriggerMotorCtrl(ShootHandle_t* handle)
{
    static int8_t max_bullet_nums = 0;             //最大子弹数
    static uint32_t reverse_time = 0;              //退弹时间
	
	         two_now_time=BSP_GetTime_ms();
					 two_interval_time=two_now_time-two_last_time;
					 two_last_time=two_now_time;
           two_all_time=two_all_time+two_interval_time;	
    if (handle->ctrl_mode == SHOOT_START)           //开始射击模式
    {
        if (handle->trigger_state == TRIGGER_END)       //拨弹结束状态
        {									
					if(handle->console->shoot_other.fire_cmd == TWO_FIRE_CMD)               //自动拨弹模式  //二连射
					{
						if(two_all_time>200||handle->console->shoot_other.fire_cmd!=handle->console->shoot_other.last_fire_cmd)  //0.2s间隔
						{
						    fire_flag=1;						
						}
						if(fire_flag==1)
						{
				        handle->fire_bullet_number = 1;
							  two_all_time=0;
							  fire_flag=0;
						}
						else
						{
					    handle->fire_bullet_number =0;
						}
					}
            /*可以发弹量 = (最大热量 - 枪口热量) / 一发子弹热量 */
					if(handle->console->shoot_other.fire_cmd == ONE_FIRE_CMD)   //单发热量计算
					{
					  max_bullet_nums = ((handle->shooter_heat_cooling_limit - handle->shooter_heat) / ONE_BULLET_HEAT)-2;     //shooter_heat2为自算热量  2
          }
					else if(handle->console->shoot_other.fire_cmd == RAPID_FIRE_CMD)   //五连发热量计算
					{ 
					  max_bullet_nums = ((handle->shooter_heat_cooling_limit - handle->shooter_heat) / ONE_BULLET_HEAT);
					}				
            if (handle->console->shoot_other.fire_cmd == ONE_FIRE_CMD && max_bullet_nums >= 1)
                handle->fire_bullet_number = 1;
            else if (handle->console->shoot_other.fire_cmd == RAPID_FIRE_CMD && max_bullet_nums >= 5)
                handle->fire_bullet_number = 5;

            if (handle->fire_bullet_number != 0)
                handle->trigger_state = TRIGGER_BEGIN;          //开始拨弹
        }
        else if (handle->trigger_state == TRIGGER_BEGIN)        //开始拨弹模式
        {
            reverse_time = 0;                                   //退弹时间为0
            if (handle->fire_bullet_number != 0)                //射弹数不为0
            {
                handle->trigger_state = TRIGGERING;             //拨弹中状态
							///////////zhe ge
                handle->trigger_motor.set_angle = handle->trigger_motor.angle + (360.0f / TRIGGER_PLATE_NUMBERS);   //拨弹电机角度设定为拨弹电机当前角度加 360/TRIGGER_PLATE_NUMBERS
                Blocked_Reset(&handle->trigger_motor.blocked_handle, TRIGGER_BLOCKED_TIMER, 1000);          //block初始化
            }
            else
            {
                handle->trigger_state = TRIGGER_END;            //拨弹结束
            }
        }
        else if (handle->trigger_state == TRIGGERING)           //拨弹中模式
        {
            BlockedState_t blocked = Blocked_Process(&handle->trigger_motor.blocked_handle, handle->trigger_motor.speed);  //判断是否卡弹
            if ( fabs(handle->trigger_motor.set_angle - handle->trigger_motor.angle) < 3.0f )        //如果设定角度 与当前角度小于0.5度
            {
                handle->fire_bullet_number--;                                                        //需要发弹数减一
                handle->trigger_state = TRIGGER_BEGIN;                                               //拨弹开始

            }
            else if (blocked == BLOCKED)                                                             //卡弹
            {
                if (reverse_time == 0)                                                               //退弹时间等于0
                {
                    reverse_time = BSP_GetTime_ms();                                                 //退弹时间等于当前系统时间
                    handle->trigger_motor.set_angle = handle->trigger_motor.angle - REVERSE_ANGLE;   //拨弹设定角度等于 拨弹电机当前角度减退弹角度
                }
                else if (BSP_GetTime_ms() - reverse_time > REVERSE_TIMER)                            //系统当前时间减退弹时间大于退弹计时器
                {
                    handle->fire_bullet_number--;                                                        //需要发弹数减一
                    handle->trigger_state = TRIGGER_BEGIN;                                          //拨弹开始
                }
            }
        }
    }
    else
    {
        handle->trigger_state = TRIGGER_END;                                                        //拨弹结束
        handle->trigger_motor.set_angle = handle->trigger_motor.angle;                              //拨弹设定角度等于拨弹电机当前角度
    }
    handle->trigger_motor.current_set = DoublePID_Calc(&handle->trigger_motor.pid,                  //拨弹电机输出电流设定
                                                       handle->trigger_motor.set_angle,
                                                       handle->trigger_motor.angle,
                                                       handle->trigger_motor.speed);
   	handle->console->shoot_other.last_fire_cmd=handle->console->shoot_other.fire_cmd;
}


static void Shoot_Other_FrictionWheelMotorCtrl(ShootCtrlMode_e mode, FrictionWheelMotor_t motor[2])   //摩擦轮射击模式
{
	
    if (mode == SHOOT_START)
    {
        for (uint8_t i = 0; i<3; i++)
        {
            if (shoot_speed_limit[i]  == shoot_other_handle.shooter_speed_limit)
            {
                motor[0].set_speed = -(friction_wheel_speed[i] + shoot_other_handle.friction_wheel_speed_buff);
                motor[1].set_speed = friction_wheel_speed[i] + shoot_other_handle.friction_wheel_speed_buff;

            }
        }
    }
    else
    {
        motor[0].set_speed = 0;
        motor[1].set_speed = 0;
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        motor[i].current_set = pid_calc(&motor[i].pid, motor[i].motor_info->speed_rpm, motor[i].set_speed);
    }
		
}

//static void Shoot_HeatingTime(ShootHandle_t* handle) //每秒热量冷却判定        
//{     	
//	if (handle->console->shoot_other.fire_cmd == ONE_FIRE_CMD)
//	{                
//		fire_flag=1;
//	}
//	else
//	{
//	  fire_flag=0;
//	}
//	if(handle->ctrl_mode == SHOOT_START||shoot_handle.shooter_heat!=0)
//	{
//	   errortime=BSP_GetTime_ms()-lasttimeflag;             //计算运行时间
//		 allerrortime=allerrortime+errortime;
//	   lasttimeflag=BSP_GetTime_ms();
//     if(allerrortime>1000)                //冷却判定       
//		 {
//		   shoot_handle.shooter_heat_cooling_rate = RefereeSystem_RobotState_Pointer()->shooter_id1_17mm_cooling_rate;
//			 allerrortime=0;
//		 }
//		 else
//		 {
//		   shoot_handle.shooter_heat_cooling_rate = 0;
//		 }	 
//		 if(fire_flag==1)      //射弹标志位
//		 {
//			 fire_flag=0;
//			 shoot_flag=1;
//		 }
//		  else
//			{				
//			 shoot_flag=0;
//			}
//  }
//}
static void Shoot_HeatingLimit(ShootHandle_t* handle)   //枪口热量计算
{	 
    if(handle->ctrl_mode == SHOOT_START||shoot_other_handle.shooter_heat!=0)     //开启射弹模式(枪口热量计算)
		{
			 shoot_other_handle.shooter_heat2 =shoot_other_handle.shooter_heat2+shoot_other_flag*10-shoot_other_handle.shooter_heat_cooling_rate; 			
			//枪口实际热量=上次枪口实际热量+实际射弹热量-冷却
			if(shoot_other_handle.shooter_heat2<30)
			{
			   shoot_other_handle.shooter_heat2=39;            //热量闭环预支
			}
		}
    if(shoot_other_handle.shooter_heat2-shoot_other_handle.shooter_heat>4*shoot_other_handle.shooter_heat_cooling_rate_k)  //弹仓无弹判定（枪口热量计算）  
			//(4为枪口热量系数可自行修改按情况)			
		{
		    shoot_other_handle.shooter_heat = RefereeSystem_PowerHeatData_Pointer()->shooter_id2_17mm_cooling_heat;	
			  shoot_other_handle.shooter_heat2 = shoot_other_handle.shooter_heat+29;
		}
}

static void ShootOtherMotorSendCurrent(int16_t fric1_cur, int16_t fric2_cur, int16_t trigger_cur, int16_t magazine_cur)
{
    Motor_SendMessage(shoot_other_handle.shoot_can, SHOOT_MOTOR_CONTROL_STD_ID, fric1_cur, fric2_cur, trigger_cur, magazine_cur);
}
