/* 包含头文件 ----------------------------------------------------------------*/
#include "chassis_app.h"
#include "infantry_def.h"
#include "app_init.h"

#include "comm_protocol.h"
#include "user_protocol.h"
#include "referee_system.h"
#include "timer_task.h"
#include "detect_task.h"
#include "client_ui_base.h"
#include "gimbal/shoot_task.h"
#include "gimbal/gimbal_task.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
ChassisHandle_t chassis_handle;
FormatTrans_t f2t;
extern Console_t console;
extern GimbalHandle_t gimbal_handle;
extern GimbalHandle_t gimbal_other_handle;
extern ShootHandle_t shoot_handle;
static TransmitHandle_t chassis_tx_handle;
static uint8_t chassis_tx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];
/*          work space                */
static ReceiveHandle_t chassis_rx_handle_from_gimbal;
static ReceiveHandle_t chassis_rx_handle_from_gimbal_other;
/*          work space                */
static uint8_t chassis_rx_fifo_buffer_from_gimbal[GIMBAL_CHASSIS_DATA_FIFO_SIZE];
static uint8_t chassis_rx_fifo_buffer_from_gimbal_other[GIMBAL_CHASSIS_DATA_FIFO_SIZE];

static TransmitHandle_t referee_tx_handle;
static uint8_t referee_tx_fifo_buffer[REFEREE_SYSTEM_FIFO_SIZE];
static ReceiveHandle_t referee_rx_handle;
static uint8_t referee_rx_fifo_buffer[REFEREE_SYSTEM_FIFO_SIZE];

static TransmitHandle_t client_ui_tx_handle;
static uint8_t client_ui_tx_fifo_buffer[REFEREE_SYSTEM_FIFO_SIZE];
/*        work space        */
extern ShootHandle_t shoot_other_handle;
static void fuck_run_init(fuck_run_struct* fuck_run);
/*        work space        */

fuck_run_struct fuck;
//ext_shoot_data_t* power;
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void ClientUI_UploadDataHook(uint8_t *data, uint16_t len);
static int32_t ClientUI_Data(void *argc);
static void CAN1_UploadDataHook(uint8_t *data, uint16_t len);
static void CAN1_RefereeDataHook(uint8_t *data, uint16_t len);
static void CAN2_UploadDataHook(uint8_t *data, uint16_t len);
static int32_t Transmit_RefereeData(void *argc);
static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len);
static void COM1_ReceiveCallback(uint8_t* data, uint16_t len);
static void COM2_ReceiveCallback(uint8_t* data, uint16_t len);
static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);
static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);
/* 函数体 --------------------------------------------------------------------*/
void ChassisAppConfig(void)
{
    chassis_handle.console      = Console_Pointer();
    chassis_handle.imu          = IMU_GetDataPointer();
    chassis_handle.chassis_can  = &can2_obj;
    chassis_handle.ctrl_mode  = CHASSIS_RELAX;
		chassis_handle.Super_Power_ratio = 0;
    chassis_handle.structure.wheel_perimeter = WHEEL_PERIMETER;
    chassis_handle.structure.wheeltrack = WHEELTRACK;
    chassis_handle.structure.wheelbase = WHEELBASE;
    chassis_handle.structure.rotate_x_offset = GIMBAL_X_OFFSET;
    chassis_handle.structure.rotate_y_offset = GIMBAL_Y_OFFSET;
	
		fuck_run_init(&fuck);
    for (uint8_t i=0; i<4; i++)
    {
        chassis_handle.chassis_motor[i].motor_info = ChassisMotor_Pointer(i);
        pid_init(&chassis_handle.chassis_motor[i].pid, POSITION_PID, 10000, 500.0f,
                 8.5f, 0.0f, 2.0f);
    }
    pid_init(&chassis_handle.chassis_follow_pid, POSITION_PID, 300.0f, 50.0f,
             5.0f, 0.0f, 5.0f);//16  0  2
    pid_init(&chassis_handle.super_power_limit_pid, POSITION_PID, 16000.0f, 0.0f,
             2.0f, 0.0f, 0.4f);                   //16000f

    /*--------------------event-----------------|---------enable-----—----|-offline time-|-beep_times-*/
    OfflineHandle_Init(OFFLINE_CHASSIS_MOTOR1,  		OFFLINE_ERROR_LEVEL,       100,         1);
    OfflineHandle_Init(OFFLINE_CHASSIS_MOTOR2,  		OFFLINE_ERROR_LEVEL,       100,         2);
    OfflineHandle_Init(OFFLINE_CHASSIS_MOTOR3,  		OFFLINE_ERROR_LEVEL,       100,         3);
    OfflineHandle_Init(OFFLINE_CHASSIS_MOTOR4,  		OFFLINE_ERROR_LEVEL,       100,         4);
    OfflineHandle_Init(OFFLINE_REFEREE_SYSTEM,  		OFFLINE_WARNING_LEVEL,     100,         3);//1
    OfflineHandle_Init(OFFLINE_GIMBAL_INFO,     		OFFLINE_ERROR_LEVEL,     	 100,         0);
		OfflineHandle_Init(OFFLINE_GIMBAL_OTHER_INFO,   OFFLINE_WARNING_LEVEL,     100,         1);
    OfflineHandle_Init(OFFLINE_DBUS,            		OFFLINE_WARNING_LEVEL,     100,         0);//2

		//底盘发送初始化	云台、第二云台接收初始化
    Comm_TransmitInit(&chassis_tx_handle, chassis_tx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, CAN1_UploadDataHook);
    Comm_ReceiveInit(&chassis_rx_handle_from_gimbal, USER_PROTOCOL_HEADER_SOF, chassis_rx_fifo_buffer_from_gimbal, GIMBAL_CHASSIS_DATA_FIFO_SIZE, UserProtocol_ParseHandler);
		Comm_ReceiveInit(&chassis_rx_handle_from_gimbal_other, USER_PROTOCOL_HEADER_SOF, chassis_rx_fifo_buffer_from_gimbal_other, GIMBAL_CHASSIS_DATA_FIFO_SIZE, UserProtocol_ParseHandler);

		//裁判系统发送接收初始化
    Comm_TransmitInit(&referee_tx_handle, referee_tx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, CAN1_RefereeDataHook);
    Comm_ReceiveInit(&referee_rx_handle, REFEREE_SYSTEM_HEADER_SOF, referee_rx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, RefereeSystem_ParseHandler);
    SoftwareTimerRegister(Transmit_RefereeData, (void*)NULL, 5);      //20=1s
	
		//UI发送初始化
    Comm_TransmitInit(&client_ui_tx_handle, client_ui_tx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, ClientUI_UploadDataHook);
    ClientUI_Init(&client_ui_tx_handle);
    SoftwareTimerRegister(ClientUI_Data, (void*)NULL, 100);
		
		/*临时*/
		//power = RefereeSystem_SpeedData_Pointer();
    
    BSP_UART_SetRxCallback(&dbus_obj, DBUS_ReceiveCallback);
    BSP_UART_SetRxCallback(&com1_obj, COM1_ReceiveCallback);
    BSP_UART_SetRxCallback(&com2_obj, COM2_ReceiveCallback);
    BSP_CAN_SetRxCallback(&can1_obj, CAN1_ReceiveCallback);
    BSP_CAN_SetRxCallback(&can2_obj, CAN2_ReceiveCallback);
}

static void ClientUI_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_UART_TransmitData(&com1_obj, data, len);
}

UiGraphicData_t xx7;
UiGraphicData_t cx1;
UiGraphicData_t cx2;
UiGraphicData_t cx3;
UiGraphicData_t cx4;
UiGraphicData_t cx5;//瞄准线
UiGraphicData_t cx6;
UiGraphicData_t cx7;
UiGraphicData_t cx8;
UiGraphicData_t cx9;
UiGraphicData_t cx10;
UiGraphicData_t cx11;
UiGraphicData_t cx12;
UiGraphicData_t yuan;//小陀螺
UiGraphicData_t hh;
UiGraphicData_t ch;
UiGraphicData_t zm;
UiGraphicData_t cz;
UiGraphicData_t bt;
UiStringData_t xtl;
UiStringData_t lxy;
UiStringData_t czk;
UiStringData_t chz;
UiStringData_t giz;
UiStringData_t dcg;
UiStringData_t dds;
UiStringData_t das;
UiGraphicData_t cd;
UiGraphicData_t dc;
UiNumberData_t num;//超电充能百分比
UiNumberData_t pit;
UiNumberData_t pyt;
UiNumberData_t lxh;
UiNumberData_t cyh;

static int32_t ClientUI_Data(void *argc)
{
    Comm_GimbalInfo_t * gimbal_info = GimbalInfo_Pointer();
    uint16_t robot_id = RefereeSystem_GetRobotID();
    ClientUI_SetHeaderSenderID(robot_id);
    ClientUI_SetHeaderReceiverID(0x100 + robot_id);
	
	
	    ClientUI_DrawString(&xtl, "xtl", 0,870, 35, ":spin", 15, 3, UI_COLOR_YELLOW);//小陀螺状态，如不需要自己去掉
	    ClientUI_DrawCircle(&hh, "hh", 0, 840, 25, 6, 12, UI_COLOR_MAIN);       //画圆
	    ClientUI_DrawString(&chz, "chz", 0, 870, 95, ":separate", 15, 3, UI_COLOR_YELLOW);//底盘状态，如不需要自己去掉
	    ClientUI_DrawCircle(&ch, "ch", 0, 840, 85, 6, 12, UI_COLOR_MAIN);
	    ClientUI_DrawString(&czk, "czk",0,870,-25,":magazine",15,3,UI_COLOR_YELLOW);//单仓盖状态
	    ClientUI_DrawCircle(&cz, "cz",0,840,-35,6,12,UI_COLOR_MAIN);
	    ClientUI_DrawString(&giz, "giz",0,870,155,":aimmode",15,3,UI_COLOR_YELLOW);//自瞄模式
	    ClientUI_DrawCircle(&bt, "bt",0,840,145,6,12,UI_COLOR_MAIN);
/************车状态UI**************/
	    if(chassis_handle.ctrl_mode == CHASSIS_SPIN)                //小陀螺
	    {
		    ClientUI_SetColor(&hh, UI_COLOR_GREEN);
	    }
			else
			{
				ClientUI_SetColor(&hh, UI_COLOR_MAIN);
			}
		  if(chassis_handle.ctrl_mode == CHASSIS_SEPARATE_GIMBAL)    //底盘状态
			{
				ClientUI_SetColor(&ch, UI_COLOR_GREEN);
			}
			else
      {
				ClientUI_SetColor(&ch, UI_COLOR_MAIN);
			} 
			if(console.magazine_cmd == MAGAZINE_ON_CMD)   //单仓盖状态
			{
        ClientUI_SetColor(&cz, UI_COLOR_GREEN);
			}
			else 
			{
			  ClientUI_SetColor(&cz, UI_COLOR_MAIN);
			}
      if(console.aim_table==0)                 //自瞄目标
			{
			ClientUI_SetColor(&bt, UI_COLOR_MAIN);		
			}
      else if(console.aim_table==1)
			{			
			ClientUI_SetColor(&bt,UI_COLOR_GREEN);
			}
/***************辅助瞄准线******************/
		if(console.rc->kb.bit.Q)
    {
        ClientUI_DrawLine(&cx1, "cx1", 5, -74, 0, 26, 0, 2, UI_COLOR_YELLOW);
		   	ClientUI_DrawIntNumber(&pit, "pit", 1, -650, 200,(float)gimbal_info->pitch_gyro_angle, 20, 3, UI_COLOR_YELLOW);//仰角		  	
		}
		ClientUI_SetNumber(&pit,(float)-gimbal_info->pitch_gyro_angle*1000);
/*******************超电************************/   //红放绿停
	  	if(chassis_handle.console->supercap_cmd == SUPERCAP_OFF_CMD)
			{
        ClientUI_DrawIntNumber(&num, "num", 1, -560, 140, (int)chassis_handle.Super_Power_ratio, 30, 3, UI_COLOR_GREEN);//超电充能百分比
			}
			if(chassis_handle.console->supercap_cmd == SUPERCAP_ON_CMD)
			{
	    	ClientUI_DrawIntNumber(&num, "num", 1, -560, 140, (int)chassis_handle.Super_Power_ratio, 30, 3, UI_COLOR_PINK);//超电充能百分比
			}
			ClientUI_SetNumber(&num,(int)chassis_handle.Super_Power_ratio);
/*******************自瞄补偿**************************/	 
		  
/*********************自瞄模式************************/
		
			ClientUI_Update(&num, &hh, &ch, &cz,&pyt,&pit, &bt);		
}
static void CAN1_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_CAN_TransmitData(&can1_obj, CHASSIS_TX_DATA_STD_ID, data, len);
}

static void CAN1_RefereeDataHook(uint8_t *data, uint16_t len)
{
    BSP_CAN_TransmitData(&can1_obj, REFEREE_DATA_STD_ID, data, len);
}

static int32_t Transmit_RefereeData(void *argc)
{
    if (CheckDeviceIsOffline(OFFLINE_REFEREE_SYSTEM))
        return 0;
		ext_game_robot_state_t* robot_state = RefereeSystem_RobotState_Pointer();
		Comm_TransmitData(&referee_tx_handle, REFEREE_SYSTEM_HEADER_SOF, GAME_ROBOT_STATE_CMD_ID, (uint8_t*)robot_state, sizeof(ext_game_robot_state_t));
		ext_power_heat_data_t* power_heat_data = RefereeSystem_PowerHeatData_Pointer();
		Comm_TransmitData(&referee_tx_handle, REFEREE_SYSTEM_HEADER_SOF, POWER_HEAT_DATA_CMD_ID, (uint8_t*)power_heat_data, sizeof(ext_power_heat_data_t));
    return 0;
}

static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len)
{
    RC_DataParser(RC_GetDataPointer(), data, len);
    Comm_TransmitData(&chassis_tx_handle, USER_PROTOCOL_HEADER_SOF, RC_DATA_CMD_ID, data, len);
    OfflineHandle_TimeUpdate(OFFLINE_DBUS);
}

static void COM1_ReceiveCallback(uint8_t* data, uint16_t len)
{
    Comm_ReceiveData(&referee_rx_handle, data, len);
    OfflineHandle_TimeUpdate(OFFLINE_REFEREE_SYSTEM);
}

static void COM2_ReceiveCallback(uint8_t* data, uint16_t len)
{
	
}

static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case CHASSIS_RX_DATA_STD_ID_FROM_GIMBAL:
        {
            Comm_ReceiveData(&chassis_rx_handle_from_gimbal, data, dlc);
					OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_INFO);
        }break;
        case CHASSIS_RX_DATA_STD_ID_FROM_GIMBAL_OTHER:
        {
            Comm_ReceiveData(&chassis_rx_handle_from_gimbal_other, data, dlc);
					OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_OTHER_INFO);
        }break;
        default:
            break;
    }
}
static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case CHASSIS_MOTOR_LF_MESSAGE_ID:
        case CHASSIS_MOTOR_RF_MESSAGE_ID:
        case CHASSIS_MOTOR_LB_MESSAGE_ID:
        case CHASSIS_MOTOR_RB_MESSAGE_ID:
        {
            uint8_t i = std_id - CHASSIS_MOTOR_LF_MESSAGE_ID;
            Motor_DataParse(chassis_handle.chassis_motor[i].motor_info, data);
            OfflineHandle_TimeUpdate((OfflineEvent_e)(OFFLINE_CHASSIS_MOTOR1+i));
        }break;
				
          case 0x210:           /*超电*/
        {
					f2t.u[0]=data[0];
					f2t.u[1]=data[1];
					f2t.u[2]=data[2];
					f2t.u[3]=data[3];
					f2t.u[4]=data[4];
					f2t.u[5]=data[5];
					f2t.u[6]=data[6];
					f2t.u[7]=data[7];
				  f2t.f=(uint16_t)(data[0] << 8 | data[1]);        
					f2t.super_state=data[2];  					//超电状态
          f2t.f2=(uint16_t)(data[3] << 8 | data[4]);       
          chassis_handle.Super_Power_ratio = f2t.f;//超电剩余     
          chassis_handle.Chassis_super_power=f2t.f2;  //底盘功率 
        default:
            break;
			 }  
    }
}

static void fuck_run_init(fuck_run_struct* fuck_run)
{
	fuck_run->i_max = 3090;
	fuck_run->j_max = 1968;
	fuck_run->v_x	= 168;
	fuck_run->v_y = 225;
	fuck_run->v_x_2 = 250;
	fuck_run->w = 40;
	
	
}
