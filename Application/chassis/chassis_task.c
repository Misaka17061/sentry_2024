/* 包含头文件 ----------------------------------------------------------------*/
#include "chassis_task.h"
#include "infantry_def.h"
#include "cmsis_os.h"
#include "chassis_function.h"

#include "user_protocol.h"

#include "bsp_init.h"
#include "referee_system.h"
/* 私有类型定义 --------------------------------------------------------------*/
 uint16_t super_power_state = 0;
         fp32 vw_vw=400;
/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
fp32 aaa = 0;

uint16_t total_current_g;
/* 任务 */
osThreadId ChassisTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t chassis_task_stack = 0;
#endif

/* 扩展变量 ------------------------------------------------------------------*/
extern ChassisHandle_t chassis_handle;
uint8_t cap_switch;
uint16_t starttime=0;
uint16_t counttime=0;
uint16_t startflag=0;
/* 私有函数原形 --------------------------------------------------------------*/
static void ChassisCtrlModeSwitch(void);
static void ChassisSensorUpdata(void);
static void ChassisMotorSendCurrent(int16_t motor1_cur, int16_t motor2_cur, int16_t motor3_cur, int16_t motor4_cur);
static void ChassisStopMode(void);
static void ChassisFollowGimbalMode(void);
static void ChassisSeparateGimbalMode(void);
static void ChassisSentryMode(void);
static void ChassisSpinMode(void);
static void ChassisSupSpinMode(void);
static void ChassisHalfMode(void);
static void ChassisHalfSPINMode(void);
static void SuperCap_SendMessage(CAN_Object_t* obj);
void SuperCap19_SendMessage(CAN_Object_t* obj);
static void ChassisSuperCapTest(void);

/*      run         */
int i = 0;
int j = 0;
uint8_t run_flag = 1;
static void fuck_run(fuck_run_struct* fuck_run);
extern fuck_run_struct fuck;

static float speed = 0.1f;
/* 函数体 --------------------------------------------------------------------*/
void ChassisTask(void const*argument)
{
    for(;;)
    { 
        ChassisSensorUpdata();
        ChassisCtrlModeSwitch();
			  /* */
			
        switch (chassis_handle.ctrl_mode)
        {
            case CHASSIS_STOP:
            {
                ChassisStopMode();
            }break;
            case CHASSIS_FOLLOW_GIMBAL:
            {
                ChassisFollowGimbalMode();
            }break;
            case CHASSIS_SEPARATE_GIMBAL:
            {
                ChassisSeparateGimbalMode();
            }break;
						case CHASSIS_SENTRY:
						{
								ChassisSentryMode();
						}break;
            case CHASSIS_SPIN:
            {
                ChassisSpinMode();
            }break;
            case CHASSIS_SUPSPIN:  
            {
                ChassisSupSpinMode();
            }break;
            case CHASSIS_HALF:  
            {
                ChassisHalfMode();
            }break;
            case CHASSIS_HALF_SPIN:
            {
                ChassisHalfSPINMode();
            }break;
            default:
                break;
        }
        
        Chassis_ControlCalc(&chassis_handle);          //底盘解算

        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_handle.chassis_motor[i].given_speed = chassis_handle.wheel_rpm[i];
            chassis_handle.chassis_motor[i].current_set = pid_calc(&chassis_handle.chassis_motor[i].pid,
                                                                   chassis_handle.chassis_motor[i].motor_info->speed_rpm,
                                                                   chassis_handle.chassis_motor[i].given_speed);
        }
 
  //        Chassis_LimitCap(&chassis_handle);                 //20超电电压限制
          Chassis_LimitPower(&chassis_handle);           //底盘功率限制

        if(chassis_handle.ctrl_mode == CHASSIS_RELAX)
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                chassis_handle.chassis_motor[i].current_set = 0;
            }
        }
        
          SuperCap_SendMessage(chassis_handle.chassis_can);       //超电控制
//        SuperCap19_SendMessage(chassis_handle.chassis_can);
        
        ChassisMotorSendCurrent(chassis_handle.chassis_motor[0].current_set,
                                    chassis_handle.chassis_motor[1].current_set,
                                    chassis_handle.chassis_motor[2].current_set,
                                    chassis_handle.chassis_motor[3].current_set);

        osDelay(CHASSIS_TASK_PERIOD);
#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
void ChassisTaskInit(void)
{
    osThreadDef(chassis_task, ChassisTask, osPriorityNormal, 0, 256);
    ChassisTaskHandle = osThreadCreate(osThread(chassis_task), NULL);
}
void SuperCap_SendMessage(CAN_Object_t* obj)
{
    static uint16_t chassis_power_limit=0;
    //static uint16_t chassis_power=0;
    static uint16_t chassis_buff=0;
    static uint8_t TxData[8] = {0};
		static uint16_t chassis_all=0;
    
    chassis_power_limit = RefereeSystem_RobotState_Pointer()->chassis_power_limit;
    for(uint8_t i = 0; i < 4; i++)
        {
            total_current_g += fabs(chassis_handle.chassis_motor[i].motor_info->given_current);
        }
        if(chassis_handle.Super_Power_ratio<18000)
        {
            //chassis_power=27000*total_current_g/1000000+10;
        }
    //chassis_power = RefereeSystem_PowerHeatData_Pointer()->chassis_power;
    chassis_buff = RefereeSystem_PowerHeatData_Pointer()->chassis_power_buffer;
		if(chassis_buff>70)           //飞坡增益控制
		{
		     chassis_buff=chassis_buff-160;		
		}
		chassis_all=fabs(chassis_handle.vx)+fabs(chassis_handle.vy)+fabs(chassis_handle.vw);         //速度值总和  
    TxData[0] = chassis_power_limit>>8;
    TxData[1] = chassis_power_limit;
    TxData[2] = super_power_state;
    TxData[3] = chassis_buff>>8;
    TxData[4] = chassis_buff;
    TxData[5] = chassis_all>>8;
    TxData[6] = chassis_all;
    TxData[7] = 0;
    BSP_CAN_TransmitData(obj, 0x100, TxData, 8);
}
void SuperCap19_SendMessage(CAN_Object_t* obj)
{
    uint16_t f1,f2;
	  f1=RefereeSystem_RobotState_Pointer()->chassis_power_limit;
    f2=RefereeSystem_PowerHeatData_Pointer()->chassis_power_buffer;
    uint8_t TxData[8] = {0};
    TxData[0] = cap_switch;
    TxData[1] = f1>>8;
    TxData[2] = f1;
    TxData[3] = f2>>8;
    TxData[4] = f2;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;
    BSP_CAN_TransmitData(obj, 0x100, TxData, 8);
}

static void ChassisSensorUpdata(void)
{
	 Comm_GimbalInfo_t* gimbal_info;
	 if (chassis_handle.console->rc->sw1 == REMOTE_SWITCH_VALUE_DOWN)
	 {
    gimbal_info = GimbalInfo_Pointer();
	 }
	 else
	 {
		gimbal_info = GimbalOtherInfo_Pointer();
	 }
	 
	 if(chassis_handle.console->chassis_cmd == CHASSIS_SENTRY_CMD)
	 {
		gimbal_info = GimbalInfo_Pointer();
	 }
	  chassis_handle.gimbal_yaw_ecd_angle = gimbal_info->yaw_ecd_angle;
    chassis_handle.chassis_pitch = chassis_handle.imu->attitude.pitch - gimbal_info->pitch_gyro_angle;
    chassis_handle.chassis_roll = chassis_handle.imu->attitude.roll;
    chassis_handle.chassis_yaw = chassis_handle.imu->attitude.yaw - gimbal_info->yaw_gyro_angle;
}

static void ChassisCtrlModeSwitch(void)
{
    if (chassis_handle.console->chassis_cmd == CHASSIS_RELEASE_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_RELAX;
    }
    else if (chassis_handle.console->chassis_cmd == CHASSIS_STOP_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_STOP;
    }
    else if (chassis_handle.console->chassis_cmd == CHASSIS_FOLLOW_GIMBAL_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
    }
    else if (chassis_handle.console->chassis_cmd == CHASSIS_SEPARATE_GIMBAL_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_SEPARATE_GIMBAL;
    }
    else if (chassis_handle.console->chassis_cmd == CHASSIS_SPIN_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_SPIN;
    }
    else if(chassis_handle.console->chassis_cmd == CHASSIS_SUPSPIN_CMD)
    {
     chassis_handle.ctrl_mode = CHASSIS_SUPSPIN;  
    }
    else if(chassis_handle.console->chassis_cmd == CHASSIS_HALF_CMD)
    {
     chassis_handle.ctrl_mode = CHASSIS_HALF;  
    }
    else if(chassis_handle.console->chassis_cmd == CHASSIS_HALFSPIN_CMD)
    {
     chassis_handle.ctrl_mode = CHASSIS_HALF_SPIN;  
    }
		else if(chassis_handle.console->chassis_cmd == CHASSIS_SENTRY_CMD)
		{
		chassis_handle.ctrl_mode = CHASSIS_SENTRY;
		}
}

static void ChassisMotorSendCurrent(int16_t motor1_cur, int16_t motor2_cur, int16_t motor3_cur, int16_t motor4_cur)
{
    Motor_SendMessage(chassis_handle.chassis_can, MOTOR_1TO4_CONTROL_STD_ID, motor1_cur, motor2_cur, motor3_cur, motor4_cur);
}

static void ChassisStopMode(void)
{
    chassis_handle.vx = 0;
    chassis_handle.vy = 0;
    chassis_handle.vw = 0;
}

static void ChassisFollowGimbalMode(void)
{
    run_flag = 1;
		i = 0;
	  j = 0;
    chassis_handle.vx = chassis_handle.console->chassis.vx;
    chassis_handle.vy = chassis_handle.console->chassis.vy;
    chassis_handle.vw = pid_calc(&chassis_handle.chassis_follow_pid,
                                 -chassis_handle.gimbal_yaw_ecd_angle,
                                 0);   
}

static void ChassisSeparateGimbalMode(void)
{
    chassis_handle.vx = chassis_handle.console->chassis.vx;
    chassis_handle.vw = chassis_handle.console->chassis.vw;
}

static void ChassisSentryMode(void)
{

 if(run_flag && RefereeSystem_Game_State_Pointer()->game_progress == 4)
 {
	fuck_run(&fuck);
 }
 
 else if (run_flag == 0 && RefereeSystem_Game_State_Pointer()->game_progress == 4)
 {
// 	static fp32 tick_sup_spin=0;    

//			chassis_handle.vx = chassis_handle.console->chassis.vx;
//      chassis_handle.vy = chassis_handle.console->chassis.vy;
//      if (chassis_handle.vw<15)
//      {
//        startflag = 0;
//        tick_sup_spin = 0;  
//      }
//      if (startflag == 0)
//      {
//      chassis_handle.vw = 50 + 200*arm_sin_f32((tick_sup_spin*CHASSIS_TASK_PERIOD*PI)/20000);      
//      tick_sup_spin++;
//       if(tick_sup_spin * CHASSIS_TASK_PERIOD> 10000)
//       {
//      starttime=BSP_GetTime_ms();
//      startflag=1;
//       }
//      }
//      else
//      {
//           counttime=BSP_GetTime_ms();     
//       if( counttime-starttime>30000)
//            {
//              tick_sup_spin = 0;
//              startflag=0;
//            }    
      chassis_handle.vw += speed;
			VAL_ROLLBACK(chassis_handle.vw,50,81,speed);
		}
 else if(RefereeSystem_Game_State_Pointer()->game_progress == 5)
 {
		chassis_handle.vw = 0;
 }
}

static void ChassisSpinMode(void)
{
      static fp32 tick_sup_spin=0;    

			chassis_handle.vx = chassis_handle.console->chassis.vx;
      chassis_handle.vy = chassis_handle.console->chassis.vy;
      if (chassis_handle.vw<10)
      {
        startflag = 0;
        tick_sup_spin = 0;  
      }
      if (startflag == 0)
      {
      chassis_handle.vw = 125 + 375*arm_sin_f32((tick_sup_spin*CHASSIS_TASK_PERIOD*PI)/20000);      
      tick_sup_spin++;
       if(tick_sup_spin * CHASSIS_TASK_PERIOD> 10000)
       {
      starttime=BSP_GetTime_ms();
      startflag=1;
       }
      }
      else
      {
           counttime=BSP_GetTime_ms();     
       if( counttime-starttime>30000)
            {
              tick_sup_spin = 0;
              startflag=0;
            }    
      }
// if(super_power_state==0)     //正常小陀螺
//	 {
//	  chassis_handle.vx = chassis_handle.console->chassis.vx*0.5;
//    chassis_handle.vy = chassis_handle.console->chassis.vy*0.5;
//	  chassis_handle.vw = 400;
//	 }
//	if(super_power_state==1)   //超电小陀螺
//	{
//	  chassis_handle.vx = chassis_handle.console->chassis.vx*0.5;
//    chassis_handle.vy = chassis_handle.console->chassis.vy*0.5;
//	  chassis_handle.vw =400;	
//	}
}

static void ChassisSupSpinMode(void)  //变速陀螺
{   

    
}

static void ChassisHalfMode(void)
{       
	chassis_handle.vx = chassis_handle.console->chassis.vx;
  chassis_handle.vy = chassis_handle.console->chassis.vy;
	chassis_handle.vw = pid_calc(&chassis_handle.chassis_follow_pid,
                                 -chassis_handle.gimbal_yaw_ecd_angle,
                                 40);   
}

static void ChassisHalfSPINMode(void)
{        
	static uint8_t spin_flag=0;
	chassis_handle.vx = chassis_handle.console->chassis.vx;
  chassis_handle.vy = chassis_handle.console->chassis.vy;
	
	if(chassis_handle.gimbal_yaw_ecd_angle < 40 && spin_flag == 1)
	{
		chassis_handle.vw = 200;
	}
	else if(chassis_handle.gimbal_yaw_ecd_angle > 40 && spin_flag == 1)
	{
		chassis_handle.vw = -200;
	}
	else if(chassis_handle.gimbal_yaw_ecd_angle > -30 && spin_flag == 0)
	{
		chassis_handle.vw = -200;
	} 
	else if(chassis_handle.gimbal_yaw_ecd_angle < -30 && spin_flag == 0)
	{
		chassis_handle.vw = 200;
	}  
	
	if(chassis_handle.gimbal_yaw_ecd_angle == 40 && spin_flag == 1)
	{
		spin_flag = 0;
	};
	if(chassis_handle.gimbal_yaw_ecd_angle == -30 && spin_flag == 0)
	{
		spin_flag = 1;
	};
}
static void ChassisSuperCapTest(void)       //超电检测
{ 
//	if(chassis_handle.super_flag==0){
//  if(chassis_handle.Super_Power_ratio>30){
//  if(chassis_handle.console->supercap_cmd == SUPERCAP_OFF_CMD)
//	{
//    	super_power_state=0;
//		  chassis_handle.super_flag=0;
//	}
//	 if(chassis_handle.console->supercap_cmd == SUPERCAP_ON_CMD)
//	{
//	   super_power_state=1;
//		 chassis_handle.super_flag=0;
//	}                                                             
//                                    
//                                          }
//	else{
//		  super_power_state=0;
//		chassis_handle.console->supercap_cmd == SUPERCAP_OFF_CMD;
//	    }
//	                                }
//	if(chassis_handle.super_flag!=0){
//  if(chassis_handle.Super_Power_ratio<50){
//		chassis_handle.console->supercap_cmd == SUPERCAP_OFF_CMD;
//      super_power_state=0;                  
//                                    
//                                          }
//	else{
//		  super_power_state=0;
//		  chassis_handle.super_flag=0;
//	    }
//	                                }

if(chassis_handle.console->supercap_cmd == SUPERCAP_OFF_CMD)
	{
		 chassis_handle.super_flag=0;
		super_power_state=0;
	}
	else if(chassis_handle.console->supercap_cmd == SUPERCAP_ON_CMD)
	{
		if(chassis_handle.super_low_flag==0)
		{
		 chassis_handle.super_flag=1;
			super_power_state=1;
		}
		else if(chassis_handle.super_low_flag==1)
		{
	 	 chassis_handle.super_flag=0;
			super_power_state=0;
		}
	}
	if(chassis_handle.Super_Power_ratio<80)   //超电最低电压限制
	{
	   chassis_handle.super_low_flag=1;
	}
	else if(chassis_handle.Super_Power_ratio>80)
	{
      chassis_handle.super_low_flag=0;
	}
	}


static void fuck_run(fuck_run_struct* fuck_run)
{			

					i++;
					chassis_handle.vx = fuck_run->v_x;
					chassis_handle.vy = fuck_run->v_y;
					if(i*CHASSIS_TASK_PERIOD>fuck_run->i_max)
					{
						chassis_handle.vx = fuck_run->v_x_2;
						chassis_handle.vy = 0;
						chassis_handle.vw = fuck_run->w;
						j++;
						if(j*CHASSIS_TASK_PERIOD>fuck_run->j_max)
						{
							chassis_handle.vx = 0;
							chassis_handle.vy = 0;
							run_flag = 0;
						}
					}
				

				
}
