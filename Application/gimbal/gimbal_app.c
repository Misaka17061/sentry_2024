/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_app.h"
#include "infantry_def.h"
#include "app_init.h"

#include "comm_protocol.h"
#include "referee_system.h"
#include "user_protocol.h"
#include "vision_protocol.h"

#include "timer_task.h"
#include "detect_task.h"
#include "shoot_task.h"
#include "math.h"
#define ABS(x) (((x) > 0) ? (x) : (-(x)))
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
GimbalHandle_t gimbal_handle;
VisionComm_t visioncomm_handle;//STF NEW
AutoAim_t yaw_aim;
extern Console_t console;
extern ShootHandle_t shoot_handle;
extern VisionDatabase_t vision_data;

static TransmitHandle_t gimbal_tx_handle;
static uint8_t gimbal_tx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];
static ReceiveHandle_t gimbal_rx_handle_from_chassis;
static ReceiveHandle_t gimbal_rx_handle_from_gimbal_other;
static uint8_t gimbal_rx_fifo_buffer_from_chassis[GIMBAL_CHASSIS_DATA_FIFO_SIZE];
static uint8_t gimbal_rx_fifo_buffer_from_gimbal_other[GIMBAL_CHASSIS_DATA_FIFO_SIZE];

static ReceiveHandle_t referee_rx_handle;
static uint8_t referee_rx_fifo_buffer[REFEREE_SYSTEM_FIFO_SIZE];

static TransmitHandle_t vision_tx_handle;
static uint8_t vision_tx_fifo_buffer[VISION_DATA_FIFO_SIZE];
static ReceiveHandle_t vision_rx_handle;
static uint8_t vision_rx_fifo_buffer[VISION_DATA_FIFO_SIZE];
/* 扩展变量 ------------------------------------------------------------------*/
Rx_vision f1,f2,f3;
uint8_t rtt[256]; 
/* 私有函数原形 --------------------------------------------------------------*/
static void CAN1_UploadDataHook(uint8_t *data, uint16_t len);
static void Vision_UploadDataHook(uint8_t *data, uint16_t len);
static int32_t GimbalInfoUploadCallback(void *argc);
static int32_t Vision_RobotInfoUploadCallback(void *argc);
static int32_t Vision_RobotInfoUploadCallback(void *argc);
static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len);
static void COM1_ReceiveCallback(uint8_t* data, uint16_t len);
static void COM2_ReceiveCallback(uint8_t* data, uint16_t len);
static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);
static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);
static void Aim_init(AutoAim_t* Aim);
/* 函数体 --------------------------------------------------------------------*/
void GimbalAppConfig(void)
{
		visioncomm_handle.comm_state = VISIONCOMM_INITIAL;//STF NEW
	  gimbal_handle.ctrl_mode = GIMBAL_INIT;
		gimbal_handle.gimbal_can = &can1_obj;
	
    gimbal_handle.console = Console_Pointer();
    gimbal_handle.imu = IMU_GetDataPointer();
	  gimbal_handle.vision_tx_data = RobotInfo_Pointer();       //机器人指针
	  gimbal_handle.vision_rx_data = VisionInfo_Pointer();      //视觉指针
    gimbal_handle.pitch_motor.motor_info = GimbalMotorPitch_Pointer();
		gimbal_handle.yaw_motor.motor_info = GimbalMotorYaw_Pointer();

    gimbal_handle.pitch_motor.offset_ecd.positive_offset_ecd =60;//正向pitch   /*上下*///    //790        354    /834       //517                  3158             
    gimbal_handle.pitch_motor.offset_ecd.reverse_offset_ecd = 517 ;//屁股pitch
    gimbal_handle.pitch_motor.ecd_ratio = PITCH_MOTO_POSITIVE_DIR * PITCH_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
    gimbal_handle.pitch_motor.max_relative_angle = 35;
    gimbal_handle.pitch_motor.min_relative_angle = -35;
		gimbal_handle.yaw_motor.offset_ecd.positive_offset_ecd = 7595;//正向yaw  /*左右*///    //3249     三号7698   反头//7384    //3591  五号 //7364     1153  //老步兵8162
		gimbal_handle.yaw_motor.offset_ecd.reverse_offset_ecd =0;//屁股yaw
		gimbal_handle.yaw_motor.ecd_ratio = YAW_MOTO_POSITIVE_DIR * YAW_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
    gimbal_handle.yaw_motor.max_relative_angle = 90;//90
    gimbal_handle.yaw_motor.min_relative_angle = -90;//-90
	
    vision_data.yaw = 2.0f;
    vision_data.yaw_ang_vel = 0.8f;
    vision_data.pitch_ratio = -0.4;
    vision_data.yaw_ratio = 0.5;
		Aim_init(&yaw_aim);
	pid_init(&gimbal_handle.yaw_motor.pid.outer_pid, POSITION_PID, 2000.0f, 0.0f,
             50.0f, 0.5f, 15.0f);  //20
  pid_init(&gimbal_handle.yaw_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 3000.0f,
						 100.0f, 0.0f, 10.0f); //100

    pid_init(&gimbal_handle.pitch_motor.pid.outer_pid, POSITION_PID, 2000.0f, 0.0f,
             45.0f, 0.0f, 10.0f);//55，0，18
    pid_init(&gimbal_handle.pitch_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 6000.0f,
             55.0f, 0.01f, 0.0f); //60，0.1，0.0  ||65，0.2，15
	 pid_init(&(yaw_aim.Vision_PID), POSITION_PID, 200.0f, 200.0f,0.0235, 0, 0.007); 

    /*--------------------event------------------------|-------enable-------|-offline time-|-beep_times-*/
    OfflineHandle_Init(OFFLINE_GIMBAL_PITCH,            OFFLINE_ERROR_LEVEL,       100,         0);//5
    OfflineHandle_Init(OFFLINE_GIMBAL_YAW,              OFFLINE_ERROR_LEVEL,       100,         0);//6
    OfflineHandle_Init(OFFLINE_FRICTION_WHEEL_MOTOR1,   OFFLINE_ERROR_LEVEL,       100,         1);//1
    OfflineHandle_Init(OFFLINE_FRICTION_WHEEL_MOTOR2,   OFFLINE_ERROR_LEVEL,       100,         2);
    OfflineHandle_Init(OFFLINE_TRIGGER_MOTOR,           OFFLINE_ERROR_LEVEL,       100,         3);
    OfflineHandle_Init(OFFLINE_REFEREE_SYSTEM,          OFFLINE_WARNING_LEVEL,     100,         3);//1
		OfflineHandle_Init(OFFLINE_CHASSIS_INFO,            OFFLINE_WARNING_LEVEL,     100,         1);
	  OfflineHandle_Init(OFFLINE_GIMBAL_OTHER_INFO,     	OFFLINE_ERROR_LEVEL,     	 100,         0);
    OfflineHandle_Init(OFFLINE_DBUS,                    OFFLINE_WARNING_LEVEL,     100,         0);
  //OfflineHandle_Init(OFFLINE_VISION_INFO,             OFFLINE_WARNING_LEVEL,     1000,        0);

		//云台发送初始化	底盘、第二云台接收初始化
    Comm_TransmitInit(&gimbal_tx_handle, gimbal_tx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, CAN1_UploadDataHook);	
    Comm_ReceiveInit(&gimbal_rx_handle_from_chassis, USER_PROTOCOL_HEADER_SOF, gimbal_rx_fifo_buffer_from_chassis, GIMBAL_CHASSIS_DATA_FIFO_SIZE, UserProtocol_ParseHandler);
		Comm_ReceiveInit(&gimbal_rx_handle_from_gimbal_other, USER_PROTOCOL_HEADER_SOF, gimbal_rx_fifo_buffer_from_gimbal_other, GIMBAL_CHASSIS_DATA_FIFO_SIZE, UserProtocol_ParseHandler);
    SoftwareTimerRegister(GimbalInfoUploadCallback, (void*)NULL, GIMBAL_UPLOAD_TIMER_PERIOD);
		
		//裁判系统接收初始化
    Comm_ReceiveInit(&referee_rx_handle, REFEREE_SYSTEM_HEADER_SOF, referee_rx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, RefereeSystem_ParseHandler);
		
		//视觉发送接收初始化
    Comm_TransmitInit(&vision_tx_handle, vision_tx_fifo_buffer, VISION_DATA_FIFO_SIZE, Vision_UploadDataHook);
    Comm_ReceiveInit(&vision_rx_handle, VISION_PROTOCOL_HEADER_SOF, vision_rx_fifo_buffer, VISION_DATA_FIFO_SIZE, VisionProtocol_ParseHandler);
    SoftwareTimerRegister(Vision_RobotInfoUploadCallback, (void*)NULL, 5);
    /*VisionProtocol_ParseHandler：
		
		vision_rx_handle视觉结构体的 UnpackDataSolve(); 解包处理回调函数*/

    BSP_UART_SetRxCallback(&dbus_obj, DBUS_ReceiveCallback);
    BSP_UART_SetRxCallback(&com1_obj, COM1_ReceiveCallback);
    BSP_UART_SetRxCallback(&com2_obj, COM2_ReceiveCallback);
    BSP_CAN_SetRxCallback(&can1_obj, CAN1_ReceiveCallback);
    BSP_CAN_SetRxCallback(&can2_obj, CAN2_ReceiveCallback);
		
}

static void CAN1_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_CAN_TransmitData(&can1_obj, GIMBAL_TX_DATA_STD_ID, data, len);
}

static void Vision_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_UART_TransmitData(&com1_obj, data, len);
}

static int32_t GimbalInfoUploadCallback(void *argc)
{
    Comm_GimbalInfo_t* info = GimbalInfo_Pointer();
    info->mode = gimbal_handle.ctrl_mode;
		info->state = vision_data.state;
    info->pitch_ecd_angle   = gimbal_handle.pitch_motor.sensor.relative_angle;
    info->yaw_ecd_angle     = gimbal_handle.yaw_motor.sensor.relative_angle;
    info->pitch_gyro_angle  = gimbal_handle.pitch_motor.sensor.gyro_angle;
    info->yaw_gyro_angle    = gimbal_handle.yaw_motor.sensor.gyro_angle;
    Comm_TransmitData(&gimbal_tx_handle, USER_PROTOCOL_HEADER_SOF, GIMBAL_INFO_CMD_ID, (uint8_t*)info, sizeof(Comm_GimbalInfo_t));
    return 0;
}
fp32 max_yaw = 0.0f,max_pitch = 0.0f;
static int32_t Vision_RobotInfoUploadCallback(void *argc)
{
    Comm_RobotInfo_t* info = RobotInfo_Pointer();
    uint16_t robot_id = RefereeSystem_GetRobotID();
    Console_t* console_info = Console_Pointer();
    info->data_head = 0xAA;
    if (robot_id > 100)     //ID大于100是蓝方  应该打红方；
    {
       info->enemy_color = Red;
    }
    else if (robot_id > 1)
    {
        info->enemy_color = Blue;
    }
    else
    { 
       info->enemy_color = AllColor;
    }

      info->speed = gimbal_handle.imu->gyro[2] * ANGLE_TO_RAD;//YAW轴移动弧速度//sqrt(pow(console.chassis.vx,2) + pow(console.chassis.vy,2));
      info->yaw_relative_angle = (gimbal_handle.imu->attitude.yaw) * ANGLE_TO_RAD;
      info->pitch_relative_angle = (gimbal_handle.imu->attitude.pitch - gimbal_handle.imu->i_attitude.i_pitch) * ANGLE_TO_RAD;
      info->bullet_speed = shoot_handle.shooter_speed_limit; 
      info->data_tail = 0xA5;
     Comm_TransmitData(&vision_tx_handle, VISION_PROTOCOL_HEADER_SOF, VISION_DATA_CMD_ID, (uint8_t*)info, sizeof(Comm_RobotInfo_t));
    return 0;
}

static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len)
{
    RC_DataParser(RC_GetDataPointer(), data, len);
    Comm_TransmitData(&gimbal_tx_handle, USER_PROTOCOL_HEADER_SOF, RC_DATA_CMD_ID, data, len);
    OfflineHandle_TimeUpdate(OFFLINE_DBUS);
}

static void COM1_ReceiveCallback(uint8_t* data, uint16_t len)
{     
   Comm_ReceiveData(&vision_rx_handle, data, len);
	 OfflineHandle_TimeUpdate(OFFLINE_VISION_INFO);
   visioncomm_handle.comm_state = VISIONCOMM_SUCCESSED;//STF NEW
//   Comm_TransmitData(&vision_tx_handle, VISION_PROTOCOL_HEADER_SOF, VISION_DATA_CMD_ID, (uint8_t*)gimbal_handle.vision_tx_data, sizeof(Comm_RobotInfo_t));    
}

static void COM2_ReceiveCallback(uint8_t* data, uint16_t len)
{

}

static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case GIMBAL_MOTOR_YAW_MESSAGE_ID:
        {
            Motor_DataParse(gimbal_handle.yaw_motor.motor_info, data);
            OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_YAW);
        }break;
        case GIMBAL_MOTOR_PITCH_MESSAGE_ID:
        {
            Motor_DataParse(gimbal_handle.pitch_motor.motor_info, data);
            OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_PITCH);
        }break;
        case GIMBAL_RX_DATA_STD_ID_FROM_CHASSIS:
        {
            Comm_ReceiveData(&gimbal_rx_handle_from_chassis, data, dlc);
            OfflineHandle_TimeUpdate(OFFLINE_CHASSIS_INFO);
        }break;
				case GIMBAL_RX_DATA_STD_ID_FROM_GIMBAL_OTHER:
        {
            Comm_ReceiveData(&gimbal_rx_handle_from_gimbal_other, data, dlc);
            OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_OTHER_INFO);
        }break;
        case REFEREE_DATA_STD_ID:
        {
            Comm_ReceiveData(&referee_rx_handle, data, dlc);
            OfflineHandle_TimeUpdate(OFFLINE_REFEREE_SYSTEM);
        }break;
        default:
            break;
    }
}

static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case FRICTION_WHEEL_1_MESSAGE_ID:
        {
            Motor_DataParse(FrictionWheelMotor_1_Pointer(), data);
            OfflineHandle_TimeUpdate(OFFLINE_FRICTION_WHEEL_MOTOR1);
        }break;
        case FRICTION_WHEEL_2_MESSAGE_ID:
        {
            Motor_DataParse(FrictionWheelMotor_2_Pointer(), data);
            OfflineHandle_TimeUpdate(OFFLINE_FRICTION_WHEEL_MOTOR2);
        }break;
        case TRIGGER_MOTOR_MESSAGE_ID:
        {
            Motor_DataParse(TriggerMotor_Pointer(), data);
            OfflineHandle_TimeUpdate(OFFLINE_TRIGGER_MOTOR);
        }break;
        default:
            break;
    }
}

static void Aim_init(AutoAim_t* Aim)
{
	Aim->pid_SAtime=0;
  Aim->aiming_time=0;
  Aim->stay_time=0;
  Aim->systeam_time=0;
  Aim->tol_angle = 1.5f;  //人为规定消抖角度范围(-tol_angle,tol_angle)
  Aim->tol_time = 700;//the aiming-done continueous time that user set 人为规定
  Aim->Sstart_time=Aim->Astart_time=0;//消抖持续时间,单次自瞄持续时间
  Aim->first_aim = 0;//初入消抖范围标志
  Aim->aim_flag = 0; //新的单次自瞄标志
	Aim->Ap_parm = 3500;//Sp_parm = 5000;//user set
  Aim->Sp_parm = 0.0075;
//float Ai_parm = 8000;
//float Si_parm = 0.002;
  Aim->enable_paramSA= 1;
}