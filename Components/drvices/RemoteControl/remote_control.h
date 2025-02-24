#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"

/* 类型定义 ------------------------------------------------------------------*/
/**
  * @brief  remote control information
  */
//#pragma pack(push, 1)
typedef struct
{
    /* 摇杆 */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    /* 拨杆 */
    uint8_t sw1;
    uint8_t sw2;
    /* 鼠标 */
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t l;
        uint8_t r;
    } mouse;
    /* 键盘 */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        } bit;
    } kb;
    /* 拨轮 */
    int16_t wheel;
} RC_Info_t;
//#pragma pack(pop)

typedef struct
{
    uint8_t switch_value_raw;       // 当前开关值
    uint8_t last_switch_value_raw;  // 上次开关值
    uint8_t switch_state;           // 状态
} RC_Switch_t;

/* 宏定义 --------------------------------------------------------------------*/
#define RC_FRAME_LENGTH     (18u)
#define RC_STICK_OFFSET     (1024u)     // 拨杆中间值
#define RC_RESOLUTION       (660.0f)    // 遥控取值范围 做转化用
#define RC_VALUE_MIN        (364u)
#define RC_VALUE_MAX        (1684u)
#define RC_DEADBAND         (5)         // 遥控器死区，因为遥控器的拨杆在中位时不一定为0

#define REMOTE_SWITCH_VALUE_UP          0x01u
#define REMOTE_SWITCH_VALUE_DOWN        0x02u
#define REMOTE_SWITCH_VALUE_CENTRAL     0x03u
#define REMOTE_SWITCH_VALUE_FALSE       0x04u
#define REMOTE_SWITCH_CHANGE_1TO3      (uint8_t)((REMOTE_SWITCH_VALUE_UP << 2) | REMOTE_SWITCH_VALUE_CENTRAL)
#define REMOTE_SWITCH_CHANGE_2TO3      (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 2) | REMOTE_SWITCH_VALUE_CENTRAL)
#define REMOTE_SWITCH_CHANGE_3TO1      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_UP)
#define REMOTE_SWITCH_CHANGE_3TO2      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_DOWN)

#define REMOTE_SWITCH_CHANGE_1TO3TO2   (uint8_t)((REMOTE_SWITCH_VALUE_UP << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_DOWN))

#define REMOTE_SWITCH_CHANGE_2TO3TO1   (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_UP))

#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void RC_DataParser(RC_Info_t *rc, uint8_t *buf, uint16_t len);
void RC_SwitchAction(RC_Switch_t *sw, uint8_t value);
RC_Info_t* RC_GetDataPointer(void);

#endif  // REMOTE_CONTROL_H

