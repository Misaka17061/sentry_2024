#ifndef REFEREE_SYSTEM_H
#define REFEREE_SYSTEM_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"
/* 类型定义 ------------------------------------------------------------------*/
/*
 * @brief command id
 */
typedef enum
{
    GAME_STATE_CMD_ID           = 0x0001,       //比赛状态数据 1HZ
    GAME_RESULT_CMD_ID          = 0x0002,       //比赛结果数据，比赛结束后发送
    GAME_ROBOT_SURVIV_CMD_ID    = 0x0003,       //比赛机器人血量数据 1HZ
    DART_STATUS_CMD_ID          = 0x0004,       //飞镖发射状态,飞镖发射后发送
    ICRA_BUFF_ZONE_STATUS_CMD_ID= 0x0005,       //人工智能挑战赛加成与惩罚区状态,1Hz
    EVENT_DATA_CMD_ID           = 0x0101,       //场地事件数据，事件改变后发送
    SUPPLY_ACTION_CMD_ID        = 0x0102,       //场地补给站动作标识数据，动作改变后发送
    SUPPLY_BOOKING_CMD_ID       = 0x0103,       //请求补给站补弹数据，上限10HZ
    REFEREE_WARNING_CMD_ID      = 0x0104,       //裁判警告数据，警告后发送
    DART_REMAINING_TIME_CMD_ID  = 0x0105,       //飞镖发射口倒计时,1Hz
    GAME_ROBOT_STATE_CMD_ID     = 0x0201,       //机器人性能体系数据，10HZ
    POWER_HEAT_DATA_CMD_ID      = 0x0202,       //实时底盘功率和枪口热量数据，50HZ
    GAME_ROBOT_POS_CMD_ID       = 0x0203,       //机器人位置数据,10HZ
    BUFF_MUCK_CMD_ID            = 0x0204,       //机器人增益数据
    AERIAL_ENERGY_CMD_ID        = 0x0205,       //空中机器人能量状态数据,10HZ
    ROBOT_HURT_CMD_ID           = 0x0206,       //伤害状态数据
    SHOOT_DATA_CMD_ID           = 0x0207,       //实时射击数据
    BULLET_REMAINING_CMD_ID     = 0x0208,       //弹丸剩余发射数，仅空中机器人，哨兵机器人以及 ICRA 机器人发送,1HZ
    RFID_STATUS_CMD_ID          = 0x0209,       //机器人 RFID 状态,1Hz
    DART_CLIENT_CMD_ID          = 0x020A,       //飞镖机器人客户端指令数据,10Hz
		SENTINEL_POS_CMD_ID         = 0x020B,       //地面机器人位置数据，固定以1Hz 频率发送
    RADAR_MARK_DATA_CMD_ID      = 0x020C,       //雷达标记进度数据，固定以 1Hz频率发送
		SENTINEL_DECISION_CMD_ID    = 0x020D,       //哨兵自主决策信息同步，固定以1Hz 频率发送
		RADAR_DECISION_CMD_ID       = 0x020E,       //雷达自主决策信息同步，固定以1Hz 频率发送
		CLIENT_CUSTOM_DATA_CMD_ID       = 0x0301,       //机器人交互数据，发送方触发发送，频率上限为 10Hz
		CUSTOM_CONTROLLER_CMD_ID    = 0x0302,       //自定义控制器→选手端图传连接的机器人
		CLIENT_MAP_INTERACT_CMD_ID  = 0x0303,       //选手端小地图交互数据，选手端触发发送
//		CLIENT_CUSTOM_DATA_CMD_ID   = 0x0304,       //机器人间交互数据，固定30HZ
		CLIENT_MAP_RADAR_CAD_ID     = 0x0305,       //选手端小地图接收雷达数据，频率上限为 10Hz
		CLIENT_MAP_SENTINEL_CMD_ID  = 0x0307,       //选手端小地图接收哨兵数据，频率上限为 1Hz
		CLIENT_MAP_ROBOT_CMD_ID     = 0x0308,       //选手端小地图接收机器人数据，频率上限为 3Hz
} cmd_id_e;

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 101,
    BLUE_ENGINEER   = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL     = 106,
    BLUE_SENTRY     = 107,
} robot_id_t;

#pragma pack(push,1)
typedef struct     //比赛状态数据:0x0001 发送频率:1Hz
{
    uint8_t game_type : 4;  //0-3 bit：比赛类型
                            //• 1：RoboMaster 机甲大师赛；
                            //• 2：RoboMaster 机甲大师单项赛；
                            //• 3：ICRA RoboMaster 人工智能挑战赛

    uint8_t game_progress : 4;  //4-7 bit：当前比赛阶段
                                //• 0：未开始比赛；
                                //• 1：准备阶段；
                                //• 2：自检阶段；
                                //• 3：5s倒计时；
                                //• 4：对战中；
                                //• 5：比赛结算中

    uint16_t stage_remain_time; //当前阶段剩余时间，单位s
    uint64_t SyncTimeStamp;     //机器人接收到该指令的精确 Unix 时间，当机载端收到有效的 NTP 服务器授时后生效
} ext_game_state_t;

typedef struct     //比赛结果数据：0x0002。发送频率：比赛结束后发送
{
    uint8_t winner;         //0 平局 1 红方胜利 2 蓝方胜利
} ext_game_result_t;

typedef struct     //机器人血量数据：0x0003。发送频率：1Hz
{
    uint16_t red_1_robot_HP;        //红1英雄机器人血量，未上场以及罚下血量为0
    uint16_t red_2_robot_HP;        //红2工程机器人血量
    uint16_t red_3_robot_HP;        //红3步兵机器人血量
    uint16_t red_4_robot_HP;        //红4步兵机器人血量
    uint16_t red_5_robot_HP;        //红5步兵机器人血量
    uint16_t red_7_robot_HP;        //红7哨兵机器人血量
    uint16_t red_outpost_HP;        //红方前哨战血量
    uint16_t red_base_HP;           //红方基地血量
    uint16_t blue_1_robot_HP;       //蓝1英雄机器人血量
    uint16_t blue_2_robot_HP;       //蓝2工程机器人血量
    uint16_t blue_3_robot_HP;       //蓝3步兵机器人血量
    uint16_t blue_4_robot_HP;       //蓝4步兵机器人血量
    uint16_t blue_5_robot_HP;       //蓝5步兵机器人血量
    uint16_t blue_7_robot_HP;       //蓝7哨兵机器人血量
    uint16_t blue_outpost_HP;       //蓝方前哨站血量
    uint16_t blue_base_HP;          //蓝方基地血量
} ext_game_robot_HP_t;

typedef struct     //飞镖发射状态：0x0004。发送频率：飞镖发射后发送
{
    uint8_t dart_belong;            //发射飞镖的队伍：1：红方飞镖    2：蓝方飞镖
    uint16_t stage_remaining_time;  //发射时的剩余比赛时间，单位 s
} ext_dart_status_t;

typedef struct     //人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz
{
/*
 * bit[0, 4, 8, 12, 16, 20]为 F1-F6 激活状态：，1 为已激活，
 * • 0 为未激活
 * • 1 为已激活
 * bit[1-3, 5-7, 9-11, 13-15, 17-19, 21-23]为 F1-F1 的状态信息：
 * • 1 为红方回血区；
 * • 2 为红方弹药补给区；
 * • 3 为蓝方回血区；
 * • 4 为蓝方弹药补给区；
 * • 5 为禁止射击区；
 * • 6 为禁止移动区；
 */
    uint8_t F1_zone_status:1;
    uint8_t F1_zone_buff_debuff_status:3;
    uint8_t F2_zone_status:1;
    uint8_t F2_zone_buff_debuff_status:3;
    uint8_t F3_zone_status:1;
    uint8_t F3_zone_buff_debuff_status:3;
    uint8_t F4_zone_status:1;
    uint8_t F4_zone_buff_debuff_status:3;
    uint8_t F5_zone_status:1;
    uint8_t F5_zone_buff_debuff_status:3;
    uint8_t F6_zone_status:1;
    uint8_t F6_zone_buff_debuff_status:3;
    uint16_t red1_bullet_left;              //红方 1 号剩余弹量
    uint16_t red2_bullet_left;              //红方 2 号剩余弹量
    uint16_t blue1_bullet_left;             //蓝方 1 号剩余弹量
    uint16_t blue2_bullet_left;             //蓝方 2 号剩余弹量
} ext_ICRA_buff_debuff_zone_status_t;

typedef struct     // 场地事件数据：0x0101。发送频率：事件改变后发送
{
/*
 * bit 0-1：己方停机坪占领状态
 * • 0为无机器人占领；
 * • 1为空中机器人已占领但未停桨；
 * • 2为空中机器人已占领并停桨
 * bit 2：己方补给站 1号补血点占领状态 1为已占领；
 * bit 3：己方补给站 2号补血点占领状态 1为已占领；
 * bit 4：己方补给站 3号补血点占领状态 1为已占领；
 * bit 5-7：己方能量机关状态：
 * • 5为打击点占领状态，1 为占领； 
 * • 6为小能量机关激活状态，1 为已激活；
 * • 7为大能量机关激活状态，1 为已激活；
 * bit 8：己方关口占领状态 1为已占领；
 * bit 9：己方碉堡占领状态 1为已占领；
 * bit 10：己方资源岛占领状态 1为已占领；

 * bit 11：己方基地护盾状态：
 * • 1为基地有虚拟护盾血量；
 * • 0为基地无虚拟护盾血量；
 * bit 12-27：保留
 * bit 28-29：ICRA红方防御加成
 * • 0：防御加成未激活；
 * • 1：防御加成5s触发激活中；
 * • 2：防御加成已激活
 * bit 30-31：ICRA蓝方防御加成
 * • 0：防御加成未激活；
 * • 1：防御加成5s触发激活中；
 * • 2：防御加成已激活
 */
    uint32_t event_type;
} ext_event_data_t;

typedef struct      //补给站动作标识：0x0102。发送频率：动作改变后发送
{
//    uint8_t supply_projectile_id;   //补给站口ID：1：1号补给口；2：2 号补给口
    uint8_t reserved;               //保留
		uint8_t supply_robot_id;        //补弹机器人ID：0为当前无机器人补弹，1为红方英雄机器人补弹，2为红方工程机器人补弹，3/4/5为红方步兵机器人补弹，11为蓝方英雄机器人补弹，12为蓝方工程机器人补弹，13/14/15为蓝方步兵机器人补弹
    uint8_t supply_projectile_step; //出弹口开闭状态：0为关闭，1为子弹准备中，2为子弹下落
    uint8_t supply_projectile_num;  //补弹数量：50：50颗子弹；100：100颗子弹；150：150颗子弹；200：200颗子弹。
} ext_supply_projectile_action_t;

typedef struct      //请求补给站补弹子弹：cmd_id (0x0103)。发送频率：上限10Hz。RM对抗赛尚未开放
{
    uint8_t supply_projectile_id;   //补给站补弹口ID：1：1号补给口
    uint8_t supply_robot_id;        //补弹机器人ID：1为红方英雄机器人补弹，2为红方工程机器人补弹，3/4/5为红方步兵机器人补弹，11为蓝方英雄机器人补弹，12为蓝方工程机器人补弹，13/14/15为蓝方步兵机器人补弹
    uint8_t supply_num;             //补弹数目：50 ：请求50颗子弹下落
} ext_supply_projectile_booking_t;

typedef struct      //裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送
{
    uint8_t level;                  //警告等级：
    uint8_t foul_robot_id;          //犯规机器人 ID：1级以及5级警告时，机器人ID为0二三四级警告时，机器人ID为犯规机器人ID
		uint8_t count;                  //己方最后一次受到判罚的违规机器人对应判罚等级的违规次数
} ext_referee_warning_t;

typedef struct      //飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz
{
    uint8_t dart_remaining_time;    //15s 倒计时
		uint16_t dart_info;
} ext_dart_remaining_time_t;

typedef struct     //比赛机器人状态：0x0201。发送频率：10Hz
{
    uint8_t robot_id;       //机器人ID：1：红方英雄机器人；2：红方工程机器人；3/4/5：红方步兵机器人；6：红方空中机器人；7：红方哨兵机器人；11：蓝方英雄机器人；12：蓝方工程机器人；13/14/15：蓝方步兵机器人；16：蓝方空中机器人；17：蓝方哨兵机器人。
    uint8_t robot_level;    //机器人等级：1：一级；2：二级；3：三级
    uint16_t remain_HP;     //机器人剩余血量
    uint16_t max_HP;        //机器人上限血量
		uint16_t shooter_barrel_cooling_value;        //机器人枪口热量每秒冷却值
		uint16_t shooter_barrel_heat_limit;           //机器人枪口热量上限
		uint16_t chassis_power_limit;                 //机器人底盘功率上限
		uint8_t power_management_gimbal_output : 1;   //云台电源管理模块的输出情况
		uint8_t power_management_chassis_output : 1;  //底盘电源管理模块的输出情况
		uint8_t power_management_shooter_output : 1;  //射击电源管理模块的输出情况
//    uint16_t shooter_id1_17mm_cooling_rate;     //机器人 1 号 17mm 枪口每秒冷却值
//    uint16_t shooter_id1_17mm_cooling_limit;    //机器人 1 号 17mm 枪口热量上限
//    uint16_t shooter_id1_17mm_speed_limit;      //机器人 1 号 17mm 枪口上限速度 单位 m/s
//    uint16_t shooter_id2_17mm_cooling_rate;     //机器人 2 号 17mm 枪口每秒冷却值
//    uint16_t shooter_id2_17mm_cooling_limit;    //机器人 2 号 17mm 枪口热量上限
//    uint16_t shooter_id2_17mm_speed_limit;      //机器人 2 号 17mm 枪口上限速度 单位 m/s
//    uint16_t shooter_id1_42mm_cooling_rate;     //机器人 42mm 枪口每秒冷却值
//    uint16_t shooter_id1_42mm_cooling_limit;    //机器人 42mm 枪口热量上限
//    uint16_t shooter_id1_42mm_speed_limit;      //机器人 42mm 枪口上限速度 单位 m/s
//    uint16_t chassis_power_limit;               //机器人最大底盘功率， 单位 w
//    /* 主控电源输出情况： */
//    uint8_t mains_power_gimbal_output : 1;  //0 bit：gimbal口输出： 1为有24V输出，0为无24v输出；
//    uint8_t mains_power_chassis_output : 1; //1 bit：chassis口输出：1为有24V输出，0为无24v输出；
//    uint8_t mains_power_shooter_output : 1; //2 bit：shooter口输出：1为有24V输出，0为无24v输出；
} ext_game_robot_state_t;

typedef struct     // 实时功率热量数据：0x0202。发送频率：50Hz
{
    uint16_t chassis_volt;          //底盘输出电压 单位 毫伏
    uint16_t chassis_current;       //底盘输出电流 单位 毫安
    float chassis_power;            //底盘输出功率 单位 W 瓦
    uint16_t chassis_power_buffer;  //底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
    uint16_t shooter_id1_17mm_cooling_heat;         //1 号 17mm 枪口热量
    uint16_t shooter_id2_17mm_cooling_heat;         //2 号 17mm 枪口热量
    uint16_t shooter_id1_42mm_cooling_heat;         //42mm 枪口热量
} ext_power_heat_data_t;

typedef struct     //机器人位置：0x0203。发送频率：10Hz
{
    float x;        //位置x坐标，单位m
    float y;        //位置y坐标，单位m
		float angle;    //本机器人测速模块的朝向，单位：度。正北为 0 度
//    float z;        //位置z坐标，单位m
//    float yaw;      //位置枪口，单位度
} ext_game_robot_pos_t;

typedef struct     //机器人增益：0x0204。发送频率：状态改变后发送
{

//    uint8_t power_rune_buff;    // bit 0：机器人血量补血状态
                                // bit 1：枪口热量冷却加速
                                // bit 2：机器人防御加成
                                // bit 3：机器人攻击加成
                                // 其他bit保留
	uint8_t recovery_buff;        //机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
	uint8_t cooling_buff;         //机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却）
	uint8_t defence_buff;         //机器人防御增益（百分比，值为 50 表示 50%防御增益）
	uint8_t vulnerability_buff;   //机器人负防御增益（百分比，值为 30 表示-30%防御增益）
	uint16_t attack_buff;         //机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
}ext_buff_musk_t;

typedef struct     //空中机器人能量状态：0x0205。发送频率：10Hz
{
//    uint8_t attack_time;    //可攻击时间 单位 s。30s 递减至0
		uint8_t airforce_status;  //空中机器人状态（0 为正在冷却，1 为冷却完毕，2 为正在空中支援）
		uint8_t time_remain;      //此状态的剩余时间
} aerial_robot_energy_t;

typedef struct     //伤害状态：0x0206。发送频率：伤害发生后发送
{
    uint8_t armor_id : 4;   // bit 0-3：当血量变化类型为装甲伤害，代表装甲ID，其中数值为0-4号代表机器人的五个装甲片，其他血量变化类型，该变量数值为0。
    uint8_t hurt_type : 4;  // bit 4-7：血量变化类型 0x0 装甲伤害扣血；0x1 模块掉线扣血；0x2 超射速扣血；0x3 超枪口热量扣血；0x4 超底盘功率扣血；0x5 装甲撞击扣血
} ext_robot_hurt_t;

typedef struct     //实时射击信息：0x0207。发送频率：射击后发送
{
    uint8_t bullet_type;    //子弹类型: 1：17mm弹丸 2：42mm弹丸
    uint8_t shooter_id;     //发射机构 ID：1：1 号 17mm 发射机构;  2：2 号 17mm 发射机构;  3：42mm 发射机构
    uint8_t launching_frequency;    //弹丸射速 单位 Hz
    float initial_speed;     //弹丸初速度 单位 m/s
} ext_shoot_data_t;

typedef struct     //子弹剩余发射数：0x0208。发送频率：1Hz 周期发送，空中机器人以及哨兵机器人主控发送
{
    uint16_t bullet_remaining_num_17mm;     //17mm 子弹剩余发射数目
    uint16_t bullet_remaining_num_42mm;     //42mm 子弹剩余发射数目
    uint16_t coin_remaining_num;            //剩余金币数量
} ext_bullet_remaining_t;

typedef struct      //机器人 RFID 状态：0x0209。发送频率：1Hz
{
/*
 * bit 0：基地增益点 RFID 状态；
 * bit 1：高地增益点 RFID 状态；
 * bit 2：能量机关激活点 RFID 状态；
 * bit 3：飞坡增益点 RFID 状态；
 * bit 4：前哨岗增益点 RFID 状态；
 * bit 5：资源岛增益点 RFID 状态；
 * bit 6：补血点增益点 RFID 状态；
 * bit 7：工程机器人补血卡 RFID 状态；
 * bit 8-25：保留 
 * bit 26-31：人工智能挑战赛 F1-F6 RFID 状态；
 * RFID 状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不
  *   能获取对应的增益效果。
 */
    uint32_t rfid_status;
} ext_rfid_status_t;

typedef struct      //飞镖机器人客户端指令数据：0x020A。发送频率：10Hz
{
    uint8_t dart_launch_opening_status;     //当前飞镖发射口的状态0：关闭；   1：正在开启或者关闭中;    2：已经开启
    uint8_t dart_attack_target;             //飞镖的打击目标，默认为前哨站；1：前哨站；     2：基地。
    uint16_t target_change_time;            //切换打击目标时的比赛剩余时间，单位秒，从未切换默认为 0。
//    uint8_t first_dart_speed;               //检测到的第一枚飞镖速度，单位 0.1m/s/LSB, 未检测是为 0。
//    uint8_t second_dart_speed;              //检测到的第二枚飞镖速度，单位 0.1m/s/LSB，未检测是为 0。
//    uint8_t third_dart_speed;               //检测到的第三枚飞镖速度，单位 0.1m/s/LSB，未检测是为 0。
//    uint8_t fourth_dart_speed;              //检测到的第四枚飞镖速度，单位 0.1m/s/LSB，未检测是为 0。
//    uint16_t last_dart_launch_time;         //最近一次的发射飞镖的比赛剩余时间，单位秒，初始值为 0。
    uint16_t operate_launch_cmd_time;       //最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0。
} ext_dart_client_cmd_t;

typedef struct    //地面机器人位置数据,0x20B
{
	float hero_x;          //己方英雄机器人位置 x 轴坐标，单位：m
	float hero_y;          //己方英雄机器人位置 y 轴坐标，单位：m
	float engineer_x;      //己方工程机器人位置 x 轴坐标，单位：m
	float engineer_y;      //己方工程机器人位置 y 轴坐标，单位：m
	float standard_3_x;    //己方 3 号步兵机器人位置 x 轴坐标，单位：m
	float standard_3_y;    //己方 3 号步兵机器人位置 y 轴坐标，单位：m
	float standard_4_x;    //己方 4 号步兵机器人位置 x 轴坐标，单位：m
	float standard_4_y;    //己方 4 号步兵机器人位置 y 轴坐标，单位：m
	float standard_5_x;    //己方 5 号步兵机器人位置 x 轴坐标，单位：m
	float standard_5_y;    //己方 5 号步兵机器人位置 y 轴坐标，单位：m
}ext_ground_robot_position_t;

///* 人机交互数据 */
typedef struct
{
    uint16_t data_cmd_id;       //数据段的内容 ID
    uint16_t sender_ID;         //发送者的 ID
    uint16_t receiver_ID;       //接收者的 ID
} ext_student_interactive_header_data_t;

typedef struct         //交互数据 机器人间通信：0x0301。内容 ID:0x0200~0x02FF.发送频率：上限 10Hz
{
  uint16_t data_cmd_id;
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t data[113];          //数据段 n 需要小于 113
} robot_interactive_data_t;

typedef struct         //键鼠遥控数据,0x0304
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
}remote_control_t;


typedef struct          //客户端删除图形。内容 ID:0x0100
{
    uint8_t operate_tpye;       //图形操作。0: 空操作； 1: 删除图层； 2: 删除所有；
    uint8_t layer;              //图层数：0~9
} ext_client_custom_graphic_delete_t;

typedef struct          //客户端添加图形。内容 ID:0x0101
{
    uint8_t graphic_name[3];    //在删除，修改等操作中，作为客户端的索引。
    uint32_t operate_tpye:3;    //图形操作：0：空操作；1：增加；2：修改；3：删除；
    uint32_t graphic_tpye:3;    //图形类型：0：直线；1：矩形；2：整圆；3：椭圆；4：圆弧；4：圆弧；6：整型数；7：字符；
    uint32_t layer:4;           //图层数，0~9
    uint32_t color:4;           //颜色：0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
    uint32_t start_angle:9;     //起始角度，单位：°，范围[0,360]；
    uint32_t end_angle:9;       //终止角度，单位：°，范围[0,360]。
    uint32_t width:10;          //线宽；
    uint32_t start_x:11;        //起点 x 坐标；
    uint32_t start_y:11;        //起点 y 坐标。
    uint32_t radius:10;         //字体大小或者半径；
    uint32_t end_x:11;          //终点 x 坐标；
    uint32_t end_y:11;          //终点 y 坐标
} graphic_data_struct_t;

#pragma pack(pop)
/* 宏定义 --------------------------------------------------------------------*/
#define REFEREE_SYSTEM_HEADER_SOF   0xA5
#define REFEREE_SYSTEM_FIFO_SIZE    (512u)

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void RefereeSystem_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len);
uint8_t RefereeSystem_GetRobotID(void);
ext_game_robot_state_t* RefereeSystem_RobotState_Pointer(void);
ext_power_heat_data_t* RefereeSystem_PowerHeatData_Pointer(void);
ext_shoot_data_t* RefereeSystem_SpeedData_Pointer(void);
remote_control_t* RefereeSystem_ClientData_Pointer(void);
ext_game_state_t* RefereeSystem_Game_State_Pointer(void);

#endif  // REFEREE_SYSTEM_H

