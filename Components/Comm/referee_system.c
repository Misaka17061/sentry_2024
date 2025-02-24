/* 包含头文件 ----------------------------------------------------------------*/
#include "referee_system.h"
#include "string.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
ext_referee_warning_t referee_warning_t;

ext_game_robot_state_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_shoot_data_t speed_heat_data_t;
ext_game_robot_pos_t game_robot_pos_t;
ext_buff_musk_t buff_musk_t;
aerial_robot_energy_t robot_energy_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;

remote_control_t  remote_control;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: RefereeSystem_ParseHandler
 * Description: 裁判系统数据解析处理
 * Input: cmd_id 协议命令码
 *        data 数据指针
 *        len 数据长度
 * Return: 无
*************************************************/
void RefereeSystem_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len)
{
    switch(cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, data, sizeof(ext_game_state_t));
        }break;

        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, data, sizeof(ext_game_result_t));
        }break;

        case GAME_ROBOT_SURVIV_CMD_ID:
        {
            memcpy(&game_robot_HP_t, data, sizeof(ext_game_robot_HP_t));
        }break;

        case EVENT_DATA_CMD_ID:
        {
            memcpy(&field_event, data, sizeof(ext_event_data_t));
        }break;

        case SUPPLY_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, data, sizeof(ext_supply_projectile_action_t));
        }break;

        case SUPPLY_BOOKING_CMD_ID:
        {
            memcpy(&supply_projectile_booking_t, data, sizeof(ext_supply_projectile_booking_t));
        }break;

        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t, data, sizeof(ext_referee_warning_t));
        }break;

        case GAME_ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, data, sizeof(ext_game_robot_state_t));
        }break;

        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t, data, sizeof(ext_power_heat_data_t));
        }break;

        case GAME_ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, data, sizeof(ext_game_robot_pos_t));
        }break;

        case BUFF_MUCK_CMD_ID:
        {
            memcpy(&buff_musk_t, data, sizeof(ext_buff_musk_t));
        }break;

        case AERIAL_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy_t, data, sizeof(aerial_robot_energy_t));
        }break;

        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, data, sizeof(ext_robot_hurt_t));
        }break;

        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&speed_heat_data_t, data, sizeof(ext_shoot_data_t));
        }break;

        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, data, sizeof(ext_bullet_remaining_t));
        }break;
				
//				case CLIENT_CUSTOM_DATA_CMD_ID:
//        {
//            memcpy(&remote_control, data, sizeof(remote_control_t));
//        }break;
    }
}

/*************************************************
 * Function: RefereeSystem_GetRobotID
 * Description: 获取机器人ID
 * Input: 无
 * Return: 机器人ID robot_id_t
*************************************************/
uint8_t RefereeSystem_GetRobotID(void)
{
    return robot_state.robot_id;
}

/*************************************************
 * Function: RefereeSystem_RobotState_Pointer
 * Description: 获取机器人状态
 * Input: 无
 * Return: 机器人状态指针
*************************************************/
ext_game_robot_state_t* RefereeSystem_RobotState_Pointer(void)
{
    return &robot_state;
}

/*************************************************
 * Function: RefereeSystem_PowerHeatData_Pointer
 * Description: 获取机器人功率热量数据
 * Input: 无
 * Return: 机器人功率热量数据指针
*************************************************/
ext_power_heat_data_t* RefereeSystem_PowerHeatData_Pointer(void)
{
    return &power_heat_data_t;
}
/*************************************************
 * Function: RefereeSystem_PowerHeatData_Pointer
 * Description: 获取机器人射速数据
 * Input: 无
 * Return: 机器人射速数据指针
*************************************************/
ext_shoot_data_t* RefereeSystem_SpeedData_Pointer(void)
{
    return &speed_heat_data_t;
}

/*************************************************
 * Function: RefereeSystem_PowerHeatData_Pointer
 * Description: 获取机器人射速数据
 * Input: 无
 * Return: 机器人射速数据指针
*************************************************/
ext_game_state_t* RefereeSystem_Game_State_Pointer(void)
{
    return &game_state;
}


