/* 包含头文件 ----------------------------------------------------------------*/
#include "infantry_console.h"
#include "gimbal/shoot_task.h"
#include "gimbal/gimbal_task.h"
#include "infantry_def.h"
#include "cmsis_os.h"
#include "RemoteControl/remote_control.h"
#include "VISION/vision.h"
#include "user_protocol.h"
#include "detect_task.h"
#include "ramp.h"
#include "math.h"
#include "gimbal_task.h"
#include "app_init.h"
/* 私有类型定义 --------------------------------------------------------------*/
uint16_t vision_flag;
uint16_t shoot5_flag;
uint16_t Automatic_flag;
uint16_t Console_mode;
uint16_t Aim2;

uint16_t shoot_error_time;
uint16_t shoot_now_time;
uint16_t shoot_last_time;

uint16_t super_k;

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
osThreadId ConsoleTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t console_task_stack = 0;
#endif

Console_t console;
extern GimbalHandle_t gimbal_handle;
//extern VisionDatabase_t vision_data;
fp32 yaw_target = 0.15f;
RC_Info_t last_rc;
RC_Switch_t wheel_switch;

ramp_v0_t front_back_ramp = RAMP_GEN_DAFAULT;
ramp_v0_t left_right_ramp = RAMP_GEN_DAFAULT;
ramp_v0_t shift_front_ramp = RAMP_GEN_DAFAULT;
ramp_v0_t shift_left_ramp = RAMP_GEN_DAFAULT;
uint8_t super_power_limit;
extern GimbalHandle_t gimbal_other_handle;
extern VisionDatabase_t vision_other_data;
extern AppType_e app_type;

/* 扩展变量 ------------------------------------------------------------------*/
void SuperPower_PowerCtrl(void);
void SuperPower_Handler(void);
/* 私有函数原形 --------------------------------------------------------------*/
static void RemoteControlWheelAction(void);
static void RemoteControl_Operation(void);
static void Keyboard_Operation(void);
static void Other_RemoteControl_Operation(void);

static void ConsoleSensorUpdata(void); //数据收集
static void	ConsoleCheckVariation(void);
/* 函数体 --------------------------------------------------------------------*/
void ConsoleTask(void const*argument)
{
    for(;;)
    {
        RemoteControlWheelAction();
			  ConsoleSensorUpdata();
			  ConsoleCheckVariation();
        switch (console.ctrl_mode)
        {
            case PREPARE_MODE:
            {
							if(app_type == GIMBAL_OTHER_APP)
							{
								if(GimbalOtherInfo_Pointer()->mode != GIMBAL_INIT && GimbalOtherInfo_Pointer()->mode != GIMBAL_RELAX)
								{
										console.ctrl_mode = NORMAL_MODE;
								}
								else
								{
										console.gimbal_other_cmd  = GIMBAL_INIT_CMD;
										console.chassis_cmd = CHASSIS_STOP_CMD;
										console.shoot_other_cmd = SHOOT_STOP_CMD;
								}
							}
							else 
							{
								if(GimbalInfo_Pointer()->mode != GIMBAL_INIT && GimbalInfo_Pointer()->mode != GIMBAL_RELAX)
								{
										console.ctrl_mode = NORMAL_MODE;
								}
								else
								{
                    console.gimbal_cmd  = GIMBAL_INIT_CMD;
                    console.chassis_cmd = CHASSIS_STOP_CMD;
                    console.shoot_cmd = SHOOT_STOP_CMD;
								}
							}
            }break;
            case NORMAL_MODE:
            {
								
						    if(console.gimbal_cmd != GIMBAL_SENTRY_CMD)
								{
									if (console.rc->sw1 == REMOTE_SWITCH_VALUE_DOWN )
									{ 
											RemoteControl_Operation();
									}
									else if(console.rc->sw1 == REMOTE_SWITCH_VALUE_UP)
									{
											Keyboard_Operation();
									}
									else if(console.rc->sw1 == REMOTE_SWITCH_VALUE_CENTRAL)
									{
											Other_RemoteControl_Operation();
									}
								}
								
								else
								{
									if(app_type == GIMBAL_OTHER_APP)
									{
											Other_RemoteControl_Operation();
									}
									else
									{
											RemoteControl_Operation();
									}
								}
									
            }break;
            case SAFETY_MODE:
            {
                if(!CheckDeviceIsOffline(OFFLINE_DBUS))
                {
                    console.ctrl_mode = PREPARE_MODE;
                }
                else
                {
                    console.gimbal_cmd  = GIMBAL_RELEASE_CMD;
										console.gimbal_other_cmd  = GIMBAL_RELEASE_CMD;
                    console.chassis_cmd  = CHASSIS_RELEASE_CMD;
                    console.shoot_cmd = SHOOT_RELEASE_CMD;
										console.shoot_other_cmd = SHOOT_RELEASE_CMD;
                }
            }break;
            default:
                break;
        }
        
        if(CheckDeviceIsOffline(OFFLINE_DBUS))
        {
            console.ctrl_mode = SAFETY_MODE;
        }
        last_rc = *console.rc;
        osDelay(CONSOLE_TASK_PERIOD);
#if INCLUDE_uxTaskGetStackHighWaterMark
        console_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void ConsoleTaskInit(void)
{
    console.rc = RC_GetDataPointer();
    console.ctrl_mode = PREPARE_MODE;
    console.chassis_cmd = CHASSIS_STOP_CMD;
    console.gimbal_cmd = GIMBAL_INIT_CMD;
		console.gimbal_other_cmd = GIMBAL_INIT_CMD;         //
    console.shoot_cmd = SHOOT_STOP_CMD;
		console.shoot_other_cmd = SHOOT_STOP_CMD;           //
    console.shoot.fire_cmd = STOP_FIRE_CMD;
		console.shoot_other.fire_cmd = STOP_FIRE_CMD;       //
	//
	  console.magazine_cmd = MAGAZINE_INIT_CMD;
    console.Reset_yaw = flag_positive;
    console.Reset_pitch = flag_positive;
    console.shift_flag = OFF;
    console.friction_wheel_speed_buff=0;
    console.aim_table = 0;                //1风车模式，0步兵模式
	  console.reset=0;

	  console.gimbal_motor_direction_cmd=RIGHT;
	 
	  shoot5_flag= 0;                  
    Automatic_flag=0;
  	Console_mode=0;
	  Aim2=1;
	
	  console.Yaw_comps=0;
	  console.Pitch_comps=0;
	
    ramp_v0_init(&front_back_ramp, (CHASSIS_ACCEL_TIME * 0.8)/CONSOLE_TASK_PERIOD);
    ramp_v0_init(&left_right_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);
    ramp_v0_init(&shift_front_ramp,CHASSIS_SHIFT_ACCEL_TIME/CONSOLE_TASK_PERIOD);
    ramp_v0_init(&shift_left_ramp,CHASSIS_SHIFT_ACCEL_TIME/CONSOLE_TASK_PERIOD);
    
    osThreadDef(console_task, ConsoleTask, osPriorityNormal, 0, 256);
    ConsoleTaskHandle = osThreadCreate(osThread(console_task), NULL);
		
		
}

Console_t* Console_Pointer(void)
{
    return &console;
}

static void RemoteControlWheelAction(void)
{
    static uint8_t wheel_sw = REMOTE_SWITCH_VALUE_CENTRAL;
    if (console.rc->wheel < -440)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_UP;
    }
    else if (console.rc->wheel > -220 && console.rc->wheel < 220)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_CENTRAL;
    }
    else if (console.rc->wheel > 440)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_DOWN;
    }
    RC_SwitchAction(&wheel_switch, wheel_sw);
		
}
static void ConsoleSensorUpdata(void)
{

}
static void RemoteControl_Operation(void)
{
    static uint32_t shoot_time = 0;
		VisionDatabase_t* shoot_auto_test = VisionData_Pointer();
		
	
    if((330 < console.rc->ch4 || 330 < console.rc->ch3) || (-330 > console.rc->ch4 || -330 > console.rc->ch3))
    {
         console.shift_flag = ON;
    }
    else
    {
         console.shift_flag = OFF;
    }
		
		
    if (console.rc->sw2 == REMOTE_SWITCH_VALUE_UP)
    {
				console.gimbal_other_cmd = GIMBAL_SENTRY_CMD;
        console.gimbal_cmd = GIMBAL_SENTRY_CMD;    
        console.chassis_cmd = CHASSIS_SENTRY_CMD; 
				
//        console.chassis.vx = console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
//        console.chassis.vy = console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y;
			  console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
    }
		else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_DOWN)
		{
				console.gimbal_other_cmd = GIMBAL_RELEASE_CMD;
				console.gimbal_cmd =GIMBAL_VISION_AIM_CMD;
        console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;

        console.chassis.vx = console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vw = console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_R;
        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;//-2000.0f;  // "-"?
		}
		
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_CENTRAL)
    {
				console.gimbal_other_cmd = GIMBAL_RELEASE_CMD;
				console.gimbal_cmd = GIMBAL_NORMAL_CMD;
        console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;

        console.chassis.vx = console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
				console.chassis.vy = console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y;
        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
    }
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_FALSE)
    {
        console.gimbal_cmd = GIMBAL_RELEASE_CMD;
        console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;
    }

		
    if (console.shoot_cmd == SHOOT_STOP_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_cmd = SHOOT_START_CMD;
        }
				else if(console.gimbal_cmd == GIMBAL_SENTRY_CMD)
				{
            console.shoot_cmd = SHOOT_START_CMD;
				}
    }
    else if (console.shoot_cmd == SHOOT_START_CMD)
    {

            if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
            {
             console.shoot_cmd = SHOOT_STOP_CMD;
            }
            else if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO2)
            {
             console.shoot.fire_cmd = ONE_FIRE_CMD;
            }
						else if (shoot_auto_test->distance && console.gimbal_cmd  ==  GIMBAL_VISION_AIM_CMD)
						{
						console.shoot.fire_cmd = ONE_FIRE_CMD;
						}
						else if (shoot_auto_test->distance && console.gimbal_cmd  ==  GIMBAL_SENTRY_CMD)
						{
						console.shoot.fire_cmd = ONE_FIRE_CMD;
						}
            else if (wheel_switch.switch_value_raw == REMOTE_SWITCH_VALUE_DOWN)
            {
							shoot_time++;
							if(shoot_time > 50)
									console.shoot.fire_cmd = ONE_FIRE_CMD ;
							else
								console.shoot.fire_cmd = STOP_FIRE_CMD;
						}
						else
						{
            console.shoot.fire_cmd = STOP_FIRE_CMD;
            shoot_time = 0;
						}
   }
}

static void Other_RemoteControl_Operation(void)
{
		static uint32_t shoot_time = 0;
		VisionDatabase_t* shoot_auto_test = VisionOtherData_Pointer();
	
    if((330 < console.rc->ch4 || 330 < console.rc->ch3) || (-330 > console.rc->ch4 || -330 > console.rc->ch3))
    {
         console.shift_flag = ON;
    }
    else
    {
         console.shift_flag = OFF;
    }
		
    if (console.rc->sw2 == REMOTE_SWITCH_VALUE_UP)
    {
				console.gimbal_other_cmd = GIMBAL_SENTRY_CMD;
        console.gimbal_cmd = GIMBAL_SENTRY_CMD;    
        console.chassis_cmd = CHASSIS_SENTRY_CMD; 
				
    }
		
		else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_DOWN)
		{
				console.gimbal_cmd =GIMBAL_RELEASE_CMD;
				console.gimbal_other_cmd = GIMBAL_VISION_AIM_CMD;
        console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;

        console.chassis.vx = console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vw = console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_R;
        console.gimbal_other.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal_other.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
		}
		
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_CENTRAL)
    {
				console.gimbal_cmd = GIMBAL_RELEASE_CMD;
				console.gimbal_other_cmd = GIMBAL_NORMAL_CMD;
        console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
        console.chassis.vx = console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vy = console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y;

        console.gimbal_other.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal_other.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
    }
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_FALSE)
    {
        console.gimbal_other_cmd = GIMBAL_RELEASE_CMD;
        console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;

        console.chassis.vx = console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vy = console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y;
        console.gimbal_other.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal_other.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
    }
		
		
    if (console.shoot_other_cmd == SHOOT_STOP_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_other_cmd = SHOOT_START_CMD;
        }
				else if(console.gimbal_other_cmd  ==  GIMBAL_SENTRY_CMD)
				{
						console.shoot_other_cmd = SHOOT_START_CMD;
				}
    }
    else if (console.shoot_other_cmd == SHOOT_START_CMD)
    {
            if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
            {
             console.shoot_other_cmd = SHOOT_STOP_CMD;
            }
            else if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO2)
            {
             console.shoot_other.fire_cmd = ONE_FIRE_CMD;
            }
						else if (shoot_auto_test->distance && console.gimbal_other_cmd  ==  GIMBAL_VISION_AIM_CMD)
						{
						console.shoot_other.fire_cmd = ONE_FIRE_CMD;
						}
						else if (shoot_auto_test->distance && console.gimbal_other_cmd  ==  GIMBAL_SENTRY_CMD)
						{
						console.shoot_other.fire_cmd = ONE_FIRE_CMD;
						}
            else if (wheel_switch.switch_value_raw == REMOTE_SWITCH_VALUE_DOWN)
            {
							shoot_time++;
							if (shoot_time > 50)
							{
                console.shoot_other.fire_cmd = ONE_FIRE_CMD;
							}
							else
							{
               console.shoot_other.fire_cmd = STOP_FIRE_CMD;
							}
            }
						else
						{
            console.shoot_other.fire_cmd = STOP_FIRE_CMD;
            shoot_time = 0;
						}
   }
}

static void Keyboard_Operation(void)    
{
    fp32 chassis_vx = 0;
    fp32 chassis_vy = 0;
	
   if(Console_mode==0)
	 {
		 
        if (console.rc->kb.bit.F)                                               // 恢复正常模式
          {
            console.magazine_cmd = MAGAZINE_OFF_CMD;
            console.gimbal_cmd = GIMBAL_NORMAL_CMD;
            console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
         }
/******************底盘模式*****************************/					
         if(console.rc->kb.bit.SHIFT)     //超电开关按下开
						{
	            console.supercap_cmd = SUPERCAP_ON_CMD;
              super_k=3.0;
						}  
						else
						{						
		          console.supercap_cmd = SUPERCAP_OFF_CMD;
							super_k=1;
						}
         if(console.rc->kb.bit.G && !last_rc.kb.bit.G)//切换朝向  正反                                                             
          {
            if(console.Reset_yaw != flag_positive)
            {
                console.Reset_yaw = flag_positive;
                console.Reset_pitch = flag_positive;
                
                console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
            }
            else
            {
                console.Reset_yaw = flag_reverse;
                console.Reset_pitch = flag_reverse;
                
                console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
            };
        };
				if (console.rc->kb.bit.R)                    //正常小陀螺
        {
            console.gimbal_cmd = GIMBAL_NORMAL_CMD;
            console.chassis_cmd = CHASSIS_SPIN_CMD;
        };
/**********************云台模式****************************/				
         if(!last_rc.kb.bit.V && console.rc->kb.bit.V )    //连射模式开启
            {             
              shoot5_flag++;
							if(shoot5_flag==3)
							{
							   shoot5_flag=0;
							}
                
            };		       
          if(!last_rc.kb.bit.E && console.rc->kb.bit.E )       //开关弹仓盖
            {
              if(console.magazine_cmd != MAGAZINE_OFF_CMD)
              {
                  console.magazine_cmd = MAGAZINE_OFF_CMD;
              }
              else
              {
                  console.magazine_cmd = MAGAZINE_ON_CMD;
              };
            };
           if(console.rc->kb.bit.CTRL)                                 //云台底盘分离
            {
            console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;
            };
						
////////        if(console.rc->mouse.r)                                    //开启自瞄
////////        {
////////					if(console.aim_table==1)
////////					{
////////            console.gimbal_cmd = GIMBAL_VISION_AIM2_CMD;
////////            console.shoot_cmd  = SHOOT_START_CMD;
////////					}
////////					else if(console.aim_table==0)
////////					{
////////					  console.gimbal_cmd = GIMBAL_VISION_AIM2_CMD;
////////            console.shoot_cmd  = SHOOT_START_CMD;
////////					}
////////        }
////////				else
////////				{
////////					console.gimbal_cmd = GIMBAL_NORMAL_CMD;
////////					if(vision_flag!=1)
////////					{
////////            console.shoot_cmd = SHOOT_STOP_CMD;	
////////					}						
////////				}; 
        if(console.rc->mouse.r)                                    //开启自瞄
        {
					if(Aim2==1)
					{
            console.gimbal_cmd = GIMBAL_VISION_AIM2_CMD;             //四号自瞄模式2
            console.shoot_cmd = SHOOT_START_CMD;
					}
					else
					{
					  console.gimbal_cmd = GIMBAL_VISION_AIM_CMD;
						console.shoot_cmd = SHOOT_START_CMD;
					}
        }
				else
				{
					  console.gimbal_cmd= GIMBAL_NORMAL_CMD;
					if(vision_flag!=1)
					{
            console.shoot_cmd = SHOOT_STOP_CMD;	
					}						
				};   				
        if (!last_rc.kb.bit.Q && console.rc->kb.bit.Q)              //开摩擦轮
        {
            if (console.shoot_cmd == SHOOT_STOP_CMD)
            {
                console.shoot_cmd = SHOOT_START_CMD;
						  	vision_flag=1;							  
            }
            else
            {
                console.shoot_cmd = SHOOT_STOP_CMD;
							  vision_flag=0;
            }
        }  
				if(!last_rc.kb.bit.C && console.rc->kb.bit.C)       //目标移动调整方向
				{ 
					console.spin90_flag = -5;
					if(console.gimbal_cmd == GIMBAL_VISION_AIM_CMD)
					{
					   if(console.gimbal_motor_direction_cmd==RIGHT)
					    {
			          console.gimbal_motor_direction_cmd=LEFT;
					    }
					    else
					    {
			          console.gimbal_motor_direction_cmd=RIGHT;
					    }
						}
					if(console.gimbal_cmd == GIMBAL_VISION_AIM2_CMD)
				   	{
								console.reset=1;	
					  }
				} 
				if(!last_rc.kb.bit.Z && console.rc->kb.bit.Z)   //切换机器人或风车
				{
				  if(Aim2==0)
					{
					  Aim2=1;
						console.aim_table = 1;
					}
					else
					{
					  console.aim_table =0;
					  Aim2=0;
					}
				}
       if(console.rc->kb.bit.X && !last_rc.kb.bit.X)
        {
           console.spin90_flag = 5;
        };
/**********************移动模式**************************/				
        ramp_v0_init(&shift_front_ramp,CHASSIS_SHIFT_ACCEL_TIME/CONSOLE_TASK_PERIOD);       //斜坡初始化
        ramp_v0_init(&shift_left_ramp,CHASSIS_SHIFT_ACCEL_TIME/CONSOLE_TASK_PERIOD);
        
        chassis_vx = KB_CHASSIS_MAX_SPEED_X * 0.272f; //0.408f                               
        chassis_vy = KB_CHASSIS_MAX_SPEED_Y * 0.272f; //0.408f   
   if (console.rc->kb.bit.S)                                       //移动   //3号反S
    {
        console.chassis.vx = chassis_vx*(2*console.Reset_yaw - 1) * ramp_v0_calculate(&front_back_ramp)*super_k;
    }
    else if (console.rc->kb.bit.W)
    {        
        console.chassis.vx = chassis_vx*(-2*console.Reset_yaw + 1) * ramp_v0_calculate(&front_back_ramp)*super_k;
    }
    else
    {
        console.chassis.vx = 0;
        ramp_v0_init(&front_back_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);
    };

    if (console.rc->kb.bit.D)
    {
        console.chassis.vy = chassis_vy*(-2*console.Reset_yaw +1) * ramp_v0_calculate(&left_right_ramp);
    }
    else if(console.rc->kb.bit.A)
    {
        console.chassis.vy = chassis_vy*(2*console.Reset_yaw - 1) * ramp_v0_calculate(&left_right_ramp);
    }
    else
    {
        console.chassis.vy = 0;
        ramp_v0_init(&left_right_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);
    };

    console.gimbal.yaw_v = -console.rc->mouse.x * KB_GIMBAL_MOVE_RATIO_YAW;
    console.gimbal.pitch_v = -console.rc->mouse.y * KB_GIMBAL_MOVE_RATIO_PIT;		
/*********************射击模式******************************/
	  if(console.shoot_cmd == SHOOT_START_CMD)
    {
        if(console.gimbal_cmd != GIMBAL_VISION_AIM_CMD)
          {
            if(shoot5_flag==0)
						{
						  if(!last_rc.mouse.l && console.rc->mouse.l)//   单点模式                                                             
                { 
									  console.shoot.fire_cmd = ONE_FIRE_CMD;
                }
              else
                {	
									  console.shoot.fire_cmd = STOP_FIRE_CMD;
						    }
            }
            if(shoot5_flag==1)
            {
					    if(!last_rc.mouse.l && console.rc->mouse.l)//   五连发模式                                                             
                { 
									  console.shoot.fire_cmd = RAPID_FIRE_CMD;
                }
              else
                {	
									  console.shoot.fire_cmd = STOP_FIRE_CMD;
						    }
			     	}	
             if(shoot5_flag==2)
					  {
					   if(console.rc->mouse.l)//   无限制发射模式                                                             
                 { 
									  console.shoot.fire_cmd = ONE_FIRE_CMD;
                 }
                else
                 {	
									  console.shoot.fire_cmd = STOP_FIRE_CMD;
						     }				
					    }						
          }
			  else
				  {
					 if(shoot5_flag==0)
						{
						  if(!last_rc.mouse.l && console.rc->mouse.l)//   单点模式                                                             
                { 
									  console.shoot.fire_cmd = ONE_FIRE_CMD;
                }
              else
                {	
									  console.shoot.fire_cmd = STOP_FIRE_CMD;
						    }
            }
           if(shoot5_flag==1)
            {
					    if(!last_rc.mouse.l && console.rc->mouse.l)//   五连发模式                                                             
                { 
									  console.shoot.fire_cmd = RAPID_FIRE_CMD;
                }
              else
                {	
									  console.shoot.fire_cmd = STOP_FIRE_CMD;
						    }
			      }	
             if(shoot5_flag==2)
						{
						    if(console.rc->mouse.l)//   无限制发射模式                                                             
                 { 
									  console.shoot.fire_cmd = ONE_FIRE_CMD;
                 }
                else
                 {	
									  console.shoot.fire_cmd = STOP_FIRE_CMD;
						     }
						}
/*********************自瞄模块（自射击）**************************/
						  if (aim_shoot_state == 0)
              { 
                  console.shoot.fire_cmd = STOP_FIRE_CMD;
              }
               else if (aim_shoot_state == 1)
              {
                  console.shoot.fire_cmd =TWO_FIRE_CMD;
              }
               else
              {
                   console.shoot.fire_cmd = STOP_FIRE_CMD;
              }  
			    }					
	      }
/******************切换第二模式***************************/		
			  if(!last_rc.kb.bit.B && console.rc->kb.bit.B)	 				  
				{
				Console_mode=1;				
				}
			}
/**********************第二模式***********************/	 
	 else
	 {
/************自瞄模式*******************/		 
	  if (!last_rc.kb.bit.Z && console.rc->kb.bit.Z)         //视觉补偿Y轴  补偿系数0.1     
        {
					if(console.Yaw_comps<3)         //Y轴自瞄补偿中心偏离度    
					{
           console.Yaw_comps=console.Yaw_comps+0.1; 
					}
        }
				if (!last_rc.kb.bit.C && console.rc->kb.bit.C)         //视觉补偿Y轴   补偿系数0.1
        {
					if(console.Yaw_comps>-3)         //Y轴自瞄补偿中心偏离度    
					{
           console.Yaw_comps=console.Yaw_comps-0.1; 
					}		
        } 
				if (!last_rc.kb.bit.F && console.rc->kb.bit.F)         //视觉补偿P轴  补偿系数0.1     
        {
					if(console.Pitch_comps<3)         //P轴自瞄补偿中心偏离度    
					{
           console.Pitch_comps=console.Pitch_comps+0.1; 
					}
        }
				if (!last_rc.kb.bit.G && console.rc->kb.bit.G)         //视觉补偿P轴  补偿系数0.1     
        {
					if(console.Pitch_comps>-3)         //P轴自瞄补偿中心偏离度    
					{
           console.Pitch_comps=console.Pitch_comps-0.1; 
					}
        }
/****************切换第一模式************************/
			if(!last_rc.kb.bit.B && console.rc->kb.bit.B)	 				  
				{
				Console_mode=0;				
				}
/*******************切换自瞄模式********************/
 /*
				if (!last_rc.kb.bit.V&& console.rc->kb.bit.V)              //自动射击模式切换    //普通自瞄，大风车1，大风车2，大风车3（四档）
        {
          if(gimbal_handle.vision_tx_data->aiming_mode<3)
					{
					   gimbal_handle.vision_tx_data->aiming_mode++;
					}
          if(gimbal_handle.vision_tx_data->aiming_mode==3)	
					{
					   gimbal_handle.vision_tx_data->aiming_mode=0;						
					}												
        }
*/				
//      			if(!last_rc.kb.bit.E && console.rc->kb.bit.E)	
//						{
//							if(gimbal_handle.vision_tx_data->open_flag==0)
//							{
//						  gimbal_handle.vision_tx_data->open_flag=1;
//              }
//              else
//							{
//							gimbal_handle.vision_tx_data->open_flag=0;
//							}								
//						}							
/**********************移动模式**************************/				
        ramp_v0_init(&shift_front_ramp,CHASSIS_SHIFT_ACCEL_TIME/CONSOLE_TASK_PERIOD);       //斜坡初始化
        ramp_v0_init(&shift_left_ramp,CHASSIS_SHIFT_ACCEL_TIME/CONSOLE_TASK_PERIOD);
        
        chassis_vx = KB_CHASSIS_MAX_SPEED_X * 0.408f;                                  
        chassis_vy = KB_CHASSIS_MAX_SPEED_Y * 0.408f;  
   if (console.rc->kb.bit.W)                                       //移动
    {
        console.chassis.vx = (chassis_vx*(2*console.Reset_yaw - 1) * ramp_v0_calculate(&front_back_ramp))*2;
    }
    else if (console.rc->kb.bit.S)
    {        
        console.chassis.vx = (chassis_vx*(-2*console.Reset_yaw + 1) * ramp_v0_calculate(&front_back_ramp))*2;
    }
    else
    {
        console.chassis.vx = 0;
        ramp_v0_init(&front_back_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);
    };

    if (console.rc->kb.bit.A)
    {
        console.chassis.vy = chassis_vy*(-2*console.Reset_yaw +1) * ramp_v0_calculate(&left_right_ramp);
    }
    else if(console.rc->kb.bit.D)
    {
        console.chassis.vy = chassis_vy*(2*console.Reset_yaw - 1) * ramp_v0_calculate(&left_right_ramp);
    }
    else
    {
        console.chassis.vy = 0;
        ramp_v0_init(&left_right_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);
    };

    console.gimbal.yaw_v = -console.rc->mouse.x * KB_GIMBAL_MOVE_RATIO_YAW;
    console.gimbal.pitch_v = -console.rc->mouse.y * KB_GIMBAL_MOVE_RATIO_PIT;		
/*********************射击模式******************************/
	  if(console.shoot_cmd == SHOOT_START_CMD)
    {
        if(console.gimbal_cmd != GIMBAL_VISION_AIM_CMD)
          {
            if(shoot5_flag==0)
						{
						  if(!last_rc.mouse.l && console.rc->mouse.l)//   单点模式                                                             
                { 
									  console.shoot.fire_cmd = ONE_FIRE_CMD;
                }
              else
                {	
									  console.shoot.fire_cmd = STOP_FIRE_CMD;
						    }
            }
            else
            {
					    if(!last_rc.mouse.l && console.rc->mouse.l)//   五连发模式                                                             
                { 
									  console.shoot.fire_cmd = RAPID_FIRE_CMD;
                }
              else
                {	
									  console.shoot.fire_cmd = STOP_FIRE_CMD;
						    }
			     	}				 
          }
			  else
				{
					 if (aim_shoot_state == 0)     //自射击
        {
            console.shoot.fire_cmd = STOP_FIRE_CMD;
        }
        else if (aim_shoot_state == 1)
        {
            console.shoot.fire_cmd =TWO_FIRE_CMD;
        }
					 if(shoot5_flag==0)
						{
						  if(!last_rc.mouse.l && console.rc->mouse.l)//   单点模式                                                             
                { 
									  console.shoot.fire_cmd = ONE_FIRE_CMD;
                }
              else
                {	
									  console.shoot.fire_cmd = STOP_FIRE_CMD;
						    }
            }
           if(shoot5_flag==1)
            {
					    if(!last_rc.mouse.l && console.rc->mouse.l)//   五连发模式                                                             
                { 
									  console.shoot.fire_cmd = RAPID_FIRE_CMD;
                }
              else
                {	
									  console.shoot.fire_cmd = STOP_FIRE_CMD;
						    }
			      }	
            if(shoot5_flag==2)
						{
						    if(console.rc->mouse.l)//   无限制发射模式                                                             
                 { 
									  console.shoot.fire_cmd = ONE_FIRE_CMD;
                 }
                else
                 {	
									  console.shoot.fire_cmd = STOP_FIRE_CMD;
						     }
						}												
			    };
	     }
		 }
    VAL_LIMIT(console.friction_wheel_speed_buff,-1000,1000);
};
static void	ConsoleCheckVariation(void)
      {
			unsigned char NOW_GIMBAL_CMD;
			unsigned char LAST_GIMBAL_CMD;
			NOW_GIMBAL_CMD= console.gimbal_cmd;
			if(NOW_GIMBAL_CMD!=LAST_GIMBAL_CMD){
			gimbal_handle.yaw_motor.gyro_flag=gimbal_handle.yaw_motor.sensor.gyro_angle;
			gimbal_handle.yaw_motor.back_flag=0;
			                                   }
			LAST_GIMBAL_CMD=console.gimbal_cmd;
			
			}

