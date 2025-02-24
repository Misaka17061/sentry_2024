/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_task.h"
#include "infantry_def.h"
#include "cmsis_os.h"
#include "vision_protocol.h"
#include "detect_task.h"
#include "referee_system.h"
#include "chassis/chassis_app.h"
#include "user_protocol.h"
/* 私有类型定义 --------------------------------------------------------------*/


/* 私有宏定义 ----------------------------------------------------------------*/
/* gimbal back center time (ms) */
#define BACK_CENTER_TIME 2500

#define KALMAN_FILTER_ANGLE 0
#define KALMAN_FILTER_SPEED 1
#define KALMAN_FILTER_ACCEL 2

/* 私有变量 ------------------------------------------------------------------*/
extern Console_t console;


/* work                  space*/
static fp32  speed_head= 0.15f;
static fp32	yaw_speed=-0.15f;


extern AutoAim_t other_yaw_aim;
/* work                  space*/



static extKalman_t Yaw_Kalman_error;
static extKalman_t Pitch_Kalman_error;

static extKalman_t Distance_Kalman_error;



static kalman_filter_init_t yaw_kalman_filter_para={ .P_data = {2,     0,    0,   2},
                                              .A_data = {1, 0.002,    0,   1},
                                              .H_data = {1,     0,    0,   1},
                                              .Q_data = {1,     0,    0,   1},
                                              .R_data = {200,   0,    0,   400}};

static kalman_filter_init_t pitch_kalman_filter_para={ .P_data = {2,     0,    0,   2},
                                                .A_data = {1, 0.002,    0,   1},
                                                .H_data = {1,     0,    0,   1},
                                                .Q_data = {1,     0,    0,   1},
                                                .R_data = {200,   0,    0,   400}};
static kalman_filter_t yaw_kalman_filter;
static kalman_filter_t pitch_kalman_filter; 
                                                

               
/* 任务 */
osThreadId GimbalOtherTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t gimbal_task_stack = 0;
#endif

/* control ramp parameter */
static ramp_v0_t yaw_ramp = RAMP_GEN_DAFAULT;
static ramp_v0_t pitch_ramp = RAMP_GEN_DAFAULT;
																							

/* 扩展变量 ------------------------------------------------------------------*/
extern GimbalHandle_t gimbal_other_handle;

extern ChassisHandle_t chassis_handle;
 VisionDatabase_t vision_other_data;
/* 私有函数原形 --------------------------------------------------------------*/
static void GimbalOtherSensorUpdata(void);
static void GimbalOtherCtrlModeSwitch(void);
static void GimbalOtherMotorSendCurrent(int16_t yaw, int16_t pitch);


static void GimbalOtherAssistMode(void);
static void GimbalOtherInitMode(void);
static void GimbalOtherGyroAngleMode(void);
static void GimbalOtherRelativeAngleMode(void);
static void GimbalOtherNormalMode(void);
static void GimbalOtherBackMode(void);

static void GimbalOtherVisionAimMode(void);
static void GimbalSentryMode(void);
static void GimbalOtherADDMode(void);
static void VisionDataUpdate(void);
VisionDatabase_t* VisionOtherData_Pointer(void);
static void pid_paramSA(AutoAim_t* Aim);
static void Aim_contorl(AutoAim_t* Aim);

uint8_t other_sentry_flag = 1;
/* 函数体 --------------------------------------------------------------------*/
void GimbalOtherTask(void const*argument)
{
    for(;;)
    {
        GimbalOtherSensorUpdata();//IMU、码盘数据读取
        GimbalOtherCtrlModeSwitch();//与控制台(Console)云台模式进行比较并切换
				VisionDataUpdate();
        switch (gimbal_other_handle.ctrl_mode)
        {
					  case GIMBAL_ASSIST:
						{
								GimbalOtherAssistMode();         //辅瞄模式ver0.1
						}
            case GIMBAL_INIT:
            {
                GimbalOtherInitMode();
            }break;
            case GIMBAL_GYRO:
            {
                GimbalOtherGyroAngleMode();
            }break;
            case GIMBAL_RELATIVE:
            {
                GimbalOtherRelativeAngleMode();
            }break;
            case GIMBAL_NORMAL:
            {
                GimbalOtherNormalMode();
            }break;
            case GIMBAL_BACK:
            {
                GimbalOtherBackMode();
            }break;                   
            case GIMBAL_VISION_AIM:
            {
                GimbalOtherVisionAimMode();              
            }break;           
						case GIMBAL_SENTRY:
						{
						    GimbalSentryMode();
						}break;
						case GIMBAL_ADD:
            {
                GimbalOtherADDMode();
            }break;
            default:
                break;
        }

        GimbalMotorControl(&gimbal_other_handle.yaw_motor);
        GimbalMotorControl(&gimbal_other_handle.pitch_motor);

        if (gimbal_other_handle.ctrl_mode == GIMBAL_RELAX)
        {
            pid_clear(&gimbal_other_handle.yaw_motor.pid.outer_pid);
            pid_clear(&gimbal_other_handle.yaw_motor.pid.inter_pid);
            pid_clear(&gimbal_other_handle.pitch_motor.pid.outer_pid);
            pid_clear(&gimbal_other_handle.pitch_motor.pid.inter_pid);
            gimbal_other_handle.yaw_motor.current_set = 0;
            gimbal_other_handle.pitch_motor.current_set = 0;
        }
        
        
        GimbalOtherMotorSendCurrent((int16_t)YAW_MOTO_POSITIVE_DIR * gimbal_other_handle.yaw_motor.current_set,
                               (int16_t)PITCH_MOTO_POSITIVE_DIR * gimbal_other_handle.pitch_motor.current_set);
        osDelay(GIMBAL_TASK_PERIOD);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void GimbalOtherTaskInit(void)
{
    ramp_v0_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_OTHER_TASK_PERIOD);
    ramp_v0_init(&pitch_ramp, BACK_CENTER_TIME/GIMBAL_OTHER_TASK_PERIOD); 
    
    KalmanCreate(&Yaw_Kalman_error, 1, 40);
    KalmanCreate(&Pitch_Kalman_error, 1, 40);
    
    KalmanCreate(&Distance_Kalman_error, 1, 2000);
    
    mat_init(&yaw_kalman_filter.Q, 2, 2,yaw_kalman_filter_para.Q_data);
    mat_init(&yaw_kalman_filter.R, 2, 2,yaw_kalman_filter_para.R_data);
    kalman_filter_init(&yaw_kalman_filter,&yaw_kalman_filter_para);
    
    mat_init(&pitch_kalman_filter.Q, 2, 2,pitch_kalman_filter_para.Q_data);
    mat_init(&pitch_kalman_filter.R, 2, 2,pitch_kalman_filter_para.R_data);
    kalman_filter_init(&pitch_kalman_filter,&pitch_kalman_filter_para);
    
    
		
    

		
    
    osThreadDef(gimbal_other_task, GimbalOtherTask, osPriorityNormal, 0, 256);
    GimbalOtherTaskHandle = osThreadCreate(osThread(gimbal_other_task), NULL);
}

static void GimbalOtherSensorUpdata(void)
{
   
    
    if(gimbal_other_handle.console->Reset_yaw == flag_positive)
    {
        gimbal_other_handle.yaw_motor.sensor.relative_angle =  gimbal_other_handle.yaw_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_other_handle.yaw_motor.motor_info->ecd,
                                                                                                                      gimbal_other_handle.yaw_motor.offset_ecd.positive_offset_ecd);
        gimbal_other_handle.pitch_motor.sensor.relative_angle =  gimbal_other_handle.pitch_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_other_handle.pitch_motor.motor_info->ecd,
                                                                                                                          gimbal_other_handle.pitch_motor.offset_ecd.positive_offset_ecd); 
    }
    else if(gimbal_other_handle.console->Reset_yaw == flag_reverse)
    {
        gimbal_other_handle.yaw_motor.sensor.relative_angle =  gimbal_other_handle.yaw_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_other_handle.yaw_motor.motor_info->ecd,
                                                                                                                      gimbal_other_handle.yaw_motor.offset_ecd.reverse_offset_ecd);
        gimbal_other_handle.pitch_motor.sensor.relative_angle =  gimbal_other_handle.pitch_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_other_handle.pitch_motor.motor_info->ecd,
                                                                                                                          gimbal_other_handle.pitch_motor.offset_ecd.reverse_offset_ecd);
    };

    gimbal_other_handle.yaw_motor.sensor.gyro_angle = gimbal_other_handle.imu->attitude.yaw;//姿态角
    gimbal_other_handle.pitch_motor.sensor.gyro_angle = gimbal_other_handle.imu->attitude.pitch;
    gimbal_other_handle.yaw_motor.sensor.palstance = gimbal_other_handle.imu->gyro[2] * RAD_TO_ANGLE;//角速度
    gimbal_other_handle.pitch_motor.sensor.palstance = gimbal_other_handle.imu->gyro[1] * RAD_TO_ANGLE;
}

static void GimbalOtherCtrlModeSwitch(void)
{
    gimbal_other_handle.last_ctrl_mode = gimbal_other_handle.ctrl_mode;
    if (gimbal_other_handle.console->gimbal_other_cmd == GIMBAL_RELEASE_CMD)
    {
        gimbal_other_handle.ctrl_mode = GIMBAL_RELAX;
    }
		
		else if (gimbal_other_handle.console->gimbal_other_cmd == GIMBAL_ASSIST_CMD)
		{
				gimbal_other_handle.ctrl_mode = GIMBAL_ASSIST;
		}
		
    else if (gimbal_other_handle.console->gimbal_other_cmd == GIMBAL_INIT_CMD)
    {
        gimbal_other_handle.ctrl_mode = GIMBAL_INIT;
    }
    else if (gimbal_other_handle.console->gimbal_other_cmd == GIMBAL_GYRO_CMD)
    {
        gimbal_other_handle.ctrl_mode = GIMBAL_GYRO;
    }
    else if (gimbal_other_handle.console->gimbal_other_cmd == GIMBAL_RELATIVE_CMD)
    {
        gimbal_other_handle.ctrl_mode = GIMBAL_RELATIVE;
    }
    else if (gimbal_other_handle.console->gimbal_other_cmd == GIMBAL_NORMAL_CMD)
    {
        gimbal_other_handle.ctrl_mode = GIMBAL_NORMAL;
    }
		 else if (gimbal_other_handle.console->gimbal_other_cmd == GIMBAL_BACK_CMD)
    {
        gimbal_other_handle.ctrl_mode = GIMBAL_BACK;
   
    }
    else if (gimbal_other_handle.console->gimbal_other_cmd == GIMBAL_VISION_AIM_CMD)
    {
        gimbal_other_handle.ctrl_mode = GIMBAL_VISION_AIM;
    }
    else if (gimbal_other_handle.console->gimbal_other_cmd == GIMBAL_ADD_CMD)
    {
        gimbal_other_handle.ctrl_mode = GIMBAL_ADD;
    }
    else if (gimbal_other_handle.console->gimbal_other_cmd == GIMBAL_SENTRY_CMD )	
		{
		    gimbal_other_handle.ctrl_mode = GIMBAL_SENTRY;
		}			
}

static void GimbalOtherMotorSendCurrent(int16_t yaw_cur, int16_t pitch_cur)
{
    Motor_SendMessage(gimbal_other_handle.gimbal_can, GIMBAL_OTHER_MOTOR_CONTROL_STD_ID,yaw_cur, pitch_cur, 0, 0);
}


static void GimbalOtherInitMode(void)
{
    if(gimbal_other_handle.last_ctrl_mode != GIMBAL_INIT)
    {
        ramp_v0_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
        ramp_v0_init(&pitch_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
    }

    gimbal_other_handle.yaw_motor.mode = ENCONDE_MODE;
    gimbal_other_handle.pitch_motor.mode = ENCONDE_MODE;

    gimbal_other_handle.yaw_motor.given_value = gimbal_other_handle.yaw_motor.sensor.relative_angle;
    gimbal_other_handle.pitch_motor.given_value = gimbal_other_handle.pitch_motor.sensor.relative_angle * (1 - ramp_v0_calculate(&pitch_ramp));
    if (fabsf(gimbal_other_handle.pitch_motor.sensor.relative_angle) <= 4.0f)
    {
        gimbal_other_handle.yaw_motor.given_value = gimbal_other_handle.yaw_motor.sensor.relative_angle * (1 - ramp_v0_calculate(&yaw_ramp));

        if (fabsf(gimbal_other_handle.yaw_motor.sensor.relative_angle) <= 4.0f )
        {
            gimbal_other_handle.ctrl_mode = GIMBAL_NORMAL;
        }
    }
}

static void GimbalOtherAssistMode(void)
{
//首先需要调用视觉组传过来的数据，1、变量如何传输；2、变量如何调用
//判断传过来的数据是1或0 1代表左 0代表右
    fp32 yaw_target=0;
    uint8_t spin90_flag = Console_Pointer()->spin90_flag;
    
    gimbal_other_handle.yaw_motor.mode = GYRO_MODE;
    gimbal_other_handle.pitch_motor.mode = ENCONDE_MODE;

    yaw_target = gimbal_other_handle.yaw_motor.given_value + gimbal_other_handle.console->gimbal.yaw_v + spin90_flag*90;

    gimbal_other_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_other_handle.yaw_motor.sensor.gyro_angle);
    gimbal_other_handle.pitch_motor.given_value += gimbal_other_handle.console->gimbal.pitch_v;
    
    Console_Pointer()->spin90_flag = 0;
	    

    VAL_LIMIT(gimbal_other_handle.pitch_motor.given_value, gimbal_other_handle.pitch_motor.min_relative_angle, gimbal_other_handle.pitch_motor.max_relative_angle);
}

static void GimbalOtherGyroAngleMode(void)
{
    gimbal_other_handle.yaw_motor.mode = GYRO_MODE;
    gimbal_other_handle.pitch_motor.mode = GYRO_MODE;

    fp32 yaw_target = 0, pitch_target = 0;

    yaw_target = gimbal_other_handle.yaw_motor.given_value + gimbal_other_handle.console->gimbal.yaw_v;
    pitch_target = gimbal_other_handle.pitch_motor.given_value + gimbal_other_handle.console->gimbal.pitch_v;

    gimbal_other_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_other_handle.yaw_motor.sensor.gyro_angle);
    gimbal_other_handle.pitch_motor.given_value = AngleTransform(pitch_target, gimbal_other_handle.pitch_motor.sensor.gyro_angle);

    VAL_LIMIT(gimbal_other_handle.pitch_motor.given_value, gimbal_other_handle.pitch_motor.min_relative_angle, gimbal_other_handle.pitch_motor.max_relative_angle);

}

static void GimbalOtherRelativeAngleMode(void)
{
    gimbal_other_handle.yaw_motor.mode = ENCONDE_MODE;
    gimbal_other_handle.pitch_motor.mode = ENCONDE_MODE;

    gimbal_other_handle.yaw_motor.given_value += gimbal_other_handle.console->gimbal_other.yaw_v;
    gimbal_other_handle.pitch_motor.given_value += gimbal_other_handle.console->gimbal_other.pitch_v;

    VAL_LIMIT(gimbal_other_handle.yaw_motor.given_value, gimbal_other_handle.yaw_motor.min_relative_angle, gimbal_other_handle.yaw_motor.max_relative_angle);
    VAL_LIMIT(gimbal_other_handle.pitch_motor.given_value, gimbal_other_handle.pitch_motor.min_relative_angle, gimbal_other_handle.pitch_motor.max_relative_angle);

}

static void GimbalOtherNormalMode(void)
{
	other_sentry_flag = 1;
    fp32 yaw_target=0;
    uint8_t spin90_flag = Console_Pointer()->spin90_flag;
    
    gimbal_other_handle.yaw_motor.mode = GYRO_MODE;
    gimbal_other_handle.pitch_motor.mode = ENCONDE_MODE;

    yaw_target = gimbal_other_handle.yaw_motor.given_value + gimbal_other_handle.console->gimbal_other.yaw_v + spin90_flag*90;

    gimbal_other_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_other_handle.yaw_motor.sensor.gyro_angle);
    gimbal_other_handle.pitch_motor.given_value += gimbal_other_handle.console->gimbal_other.pitch_v;
    
    Console_Pointer()->spin90_flag = 0;
	    

    VAL_LIMIT(gimbal_other_handle.pitch_motor.given_value, gimbal_other_handle.pitch_motor.min_relative_angle, gimbal_other_handle.pitch_motor.max_relative_angle);
}
static void GimbalOtherBackMode(void)
{
    fp32 yaw_target=0;
	 
    uint8_t spin90_flag = Console_Pointer()->spin90_flag;
    
    gimbal_other_handle.yaw_motor.mode = GYRO_MODE;
    gimbal_other_handle.pitch_motor.mode = ENCONDE_MODE;
 
	  switch(gimbal_other_handle.yaw_motor.back_flag){
		case 0:
		gimbal_other_handle.yaw_motor.given_value = AngleTransform(gimbal_other_handle.yaw_motor.gyro_flag,  gimbal_other_handle.yaw_motor.sensor.gyro_angle);
    gimbal_other_handle.pitch_motor.given_value += gimbal_other_handle.console->gimbal.pitch_v;
		gimbal_other_handle.yaw_motor.back_flag= 1;
		break;
		case 1:
		yaw_target = gimbal_other_handle.yaw_motor.given_value + gimbal_other_handle.console->gimbal.yaw_v + spin90_flag*90;
    gimbal_other_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_other_handle.yaw_motor.sensor.gyro_angle);
    gimbal_other_handle.pitch_motor.given_value += gimbal_other_handle.console->gimbal.pitch_v;
    break;
    
    Console_Pointer()->spin90_flag = 0;
	}
}
static void GimbalOtherADDMode(void)
{
    fp32 yaw_target = 0;
    gimbal_other_handle.yaw_motor.mode = GYRO_MODE;
    gimbal_other_handle.pitch_motor.mode = ENCONDE_MODE;

    yaw_target = gimbal_other_handle.yaw_motor.given_value + gimbal_other_handle.console->gimbal.yaw_v;

    gimbal_other_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_other_handle.yaw_motor.sensor.gyro_angle);
    gimbal_other_handle.pitch_motor.given_value = 0;

    VAL_LIMIT(gimbal_other_handle.pitch_motor.given_value, gimbal_other_handle.pitch_motor.min_relative_angle, gimbal_other_handle.pitch_motor.max_relative_angle);
}

static void GimbalOtherVisionAimMode(void)      //自瞄（自开火手移动）
{
	vision_other_data.pitch_ratio=-0.3;
	gimbal_other_handle.yaw_motor.mode = GYRO_MODE;                                       //使能云台yaw轴陀螺仪模式
  gimbal_other_handle.pitch_motor.mode = ENCONDE_MODE;                                  //使能云台pitch轴编码器模式																														
	VisionDatabase_t* info = VisionOtherData_Pointer();
	   Aim_contorl(&other_yaw_aim);
      pid_paramSA(&other_yaw_aim);//自适应PID
    
    gimbal_other_handle.pitch_motor.given_value += gimbal_other_handle.console->gimbal_other.pitch_v-((fp32)info->pitch)*VS_GIMBAL_MOVE_RATIO_PIT*vision_other_data.pitch_ratio;
    gimbal_other_handle.yaw_motor.given_value += gimbal_other_handle.console->gimbal_other.yaw_v+pid_calc(&(other_yaw_aim.Vision_PID),gimbal_other_handle.yaw_motor.given_value,gimbal_other_handle.yaw_motor.given_value+info->yaw);
	
    VAL_LIMIT(gimbal_other_handle.pitch_motor.given_value, gimbal_other_handle.pitch_motor.min_relative_angle, gimbal_other_handle.pitch_motor.max_relative_angle);
    gimbal_other_handle.yaw_motor.given_value  =  AngleTransform(gimbal_other_handle.yaw_motor.given_value, gimbal_other_handle.yaw_motor.sensor.gyro_angle);
	  
       
 }



static void GimbalSentryMode(void)
{  
	vision_other_data.pitch_ratio=-0.3;																													//处理视觉数据
	VisionDatabase_t* info = VisionOtherData_Pointer();
	

			if (info->state)
			{
				gimbal_other_handle.yaw_motor.mode = GYRO_MODE;                                       //使能云台yaw轴陀螺仪模式
				gimbal_other_handle.pitch_motor.mode = ENCONDE_MODE;
				Aim_contorl(&other_yaw_aim);
				pid_paramSA(&other_yaw_aim);//自适应PID
    
				gimbal_other_handle.pitch_motor.given_value += gimbal_other_handle.console->gimbal_other.pitch_v-((fp32)info->pitch)*VS_GIMBAL_MOVE_RATIO_PIT*vision_other_data.pitch_ratio;
				gimbal_other_handle.yaw_motor.given_value += gimbal_other_handle.console->gimbal.yaw_v+pid_calc(&(other_yaw_aim.Vision_PID),gimbal_other_handle.yaw_motor.given_value,gimbal_other_handle.yaw_motor.given_value+info->yaw);

				VAL_LIMIT(gimbal_other_handle.pitch_motor.given_value, gimbal_other_handle.pitch_motor.min_relative_angle, gimbal_other_handle.pitch_motor.max_relative_angle);
				gimbal_other_handle.yaw_motor.given_value  =  AngleTransform(gimbal_other_handle.yaw_motor.given_value, gimbal_other_handle.yaw_motor.sensor.gyro_angle);
			}
//			else if (GimbalInfo_Pointer()->state)
//			{
//				gimbal_other_handle.yaw_motor.mode = ENCONDE_MODE;                                       //使能云台yaw轴陀螺仪模式
//				gimbal_other_handle.pitch_motor.mode = ENCONDE_MODE;                                  //使能云台pitch轴编码器模式	
//				Comm_GimbalInfo_t* gimbal_info = GimbalInfo_Pointer();
//				uint8_t spin90_flag = Console_Pointer()->spin90_flag;

//				gimbal_other_handle.yaw_motor.given_value = AngleTransform(gimbal_info->yaw_ecd_angle , gimbal_other_handle.yaw_motor.sensor.relative_angle);
//		
//				gimbal_other_handle.pitch_motor.given_value += speed_head;		
//				VAL_ROLLBACK(gimbal_other_handle.pitch_motor.given_value,-16,0,speed_head);	
//			}
			else if (info->state == 0)
			{
				gimbal_other_handle.yaw_motor.mode = GYRO_MODE;                                       //使能云台yaw轴陀螺仪模式
				gimbal_other_handle.pitch_motor.mode = ENCONDE_MODE;
				//  yaw_target = gimbal_handle.yaw_motor.given_value + gimbal_handle.console->gimbal.yaw_v + spin90_flag*90;
				fp32 yaw_target=0;
				uint8_t spin90_flag = Console_Pointer()->spin90_flag;
       
				yaw_target = gimbal_other_handle.yaw_motor.given_value +yaw_speed + spin90_flag*90;

				gimbal_other_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_other_handle.yaw_motor.sensor.gyro_angle); 
				gimbal_other_handle.pitch_motor.given_value +=speed_head;		
				VAL_ROLLBACK(gimbal_other_handle.pitch_motor.given_value,-16,0,speed_head);	
			}
	
}



static void VisionDataUpdate(void)
{
    Comm_VisionInfo_t* info = VisionInfo_Pointer();
    vision_other_data.state = info->state;
		vision_other_data.distance = info->distance_1;
    vision_other_data.last_pitch = vision_other_data.pitch;
    vision_other_data.last_yaw= vision_other_data.yaw;
    vision_other_data.pitch = info->pitch_int + (info->pitch_dec)*0.01;
    if ((int)vision_other_data.pitch >=30){
    vision_other_data.pitch = 30.0f - vision_other_data.pitch;//
    }     
    vision_other_data.yaw = info->yaw_int + (info->yaw_dec)*0.01;
    if ((int)vision_other_data.yaw >=60){
    vision_other_data.yaw = 60.0f - vision_other_data.yaw;
		vision_other_data.pitch_ratio=-0.3;//pitch轴响应比例（影响Pitch轴跟踪）-0.7会抖
		other_yaw_aim.systeam_time = BSP_GetTime_ms();//获取代码总运行时间
    }
}

static void Aim_contorl(AutoAim_t* Aim)
{
   if (vision_other_data.state)//视觉NUC在线时，此条件一直成立
		{     
    if (ABS(vision_other_data.yaw)<Aim->tol_angle)//自瞄已经达到一定角度之内
			{  //thought to be aimed right认为自瞄正确
        if (vision_other_data.yaw_success == AIM_NO&&!Aim->first_aim)
				{ 
        /************记录未进入第一次自瞄的时间**********/  
        Aim->Sstart_time = Aim->systeam_time; 
        Aim->first_aim = 1, Aim->aiming_time = 0;//进入第一次自瞄
        Aim->Vision_PID.p = 0.011;//user set            
        }
        else if(vision_other_data.yaw_success == AIM_NO&&Aim->first_aim)
				{
            Aim->stay_time = Aim->systeam_time-Aim->Sstart_time;//comparing aimed right time
            if(Aim->stay_time>Aim->tol_time)//自瞄持续时间达到tol_time的标准
						{
							Aim->first_aim = 0,Aim->aim_flag = 0;//认为已完成瞄准，退出此次自瞄及消抖 							
              if(ABS(vision_other_data.yaw)<0.5f)vision_other_data.yaw_success = AIM_RIGHT;//if long enough ,thought to be aimed stable 
            }
        }      
      }
    else{  //自瞄开始，此时还未瞄准到规定角度差
         vision_other_data.yaw_success = AIM_NO,Aim->Vision_PID.p = 0.013;//user set
      if (Aim->aim_flag == 0) 
			{
            Aim->Astart_time = Aim->systeam_time,Aim->aim_flag = 1;//记录自瞄开始时间，并进入新的单次自瞄         
      } 
      else{
            Aim->aiming_time = Aim->systeam_time - Aim->Astart_time;//记录自瞄持续时间
      }
    }
   }
  else{//视觉NUC离线，无法被视觉控制自瞄
      vision_other_data.yaw_success = AIM_NO;
      Aim->aiming_time = 0 , Aim->stay_time = 0;
       Aim->Vision_PID.i=0.00f;		
  }
    VAL_LIMIT(vision_other_data.yaw,-15.0f,15.0f);  
}

static void pid_paramSA(AutoAim_t* Aim)
{
 if(Aim->enable_paramSA)
 {
  if(vision_other_data.yaw_success == AIM_NO)
	{
   /*时间豁度补偿系数P   TODO 增添I补偿及D补偿*/
  Aim->Vision_PID.p += ((float)(Aim->aiming_time*0.0003/Aim->Ap_parm));
//  vision_pid.i +=  ((float)aiming_time*0.00002/Ai_parm);
//vision_pid.p -= ((float)stay_time/Sp_parm); //Sp_parm = 5000
  if(Aim->first_aim)//如果进入第一次自瞄
	{
  VAL_LIMIT(Aim->stay_time,1,1000);
  Aim->Vision_PID.p -= (Aim->Sp_parm/(float)(Aim->stay_time));
//	vision_pid.i -=  (Si_parm/(float)stay_time);//等待视觉规定丢失目标后传的角度
  }
	
	if(vision_other_data.yaw==-90)//90位规定的角度
		Aim->Vision_PID.i=0;
  VAL_LIMIT(Aim->Vision_PID.p,0.002,0.05);
	VAL_LIMIT(Aim->Vision_PID.i, 0.00000, 0.000015);
  }
//  else{
//  pid_init(&vision_pid, POSITION_PID, 200.0f, 200.0f,vision_p, vision_i, vision_d);
//  }
 }
}

VisionDatabase_t* VisionOtherData_Pointer(void)
{

    return &vision_other_data;
}



