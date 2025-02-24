/* 包含头文件 ----------------------------------------------------------------*/
#include "remote_control.h"
#include "string.h"
#include "stdlib.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/\
RC_Info_t m_rc_info;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void RC_DataParser(RC_Info_t *rc, uint8_t *buf, uint16_t len)
{
    if(buf == NULL)
    {
        return;
    }
    rc->ch1 = (((int16_t)buf[0] | ((int16_t)buf[1] << 8)) & 0x07FF) - (int16_t)RC_STICK_OFFSET;
    rc->ch2 = ((((int16_t)buf[1] >> 3) | ((int16_t)buf[2] << 5)) & 0x07FF) - (int16_t)RC_STICK_OFFSET;
    rc->ch3 = ((((int16_t)buf[2] >> 6) | ((int16_t)buf[3] << 2) | ((int16_t)buf[4] << 10)) & 0x07FF) - (int16_t)RC_STICK_OFFSET;
    rc->ch4 = ((((int16_t)buf[4] >> 1) | ((int16_t)buf[5] << 7)) & 0x07FF) - (int16_t)RC_STICK_OFFSET;
    /* prevent remote control zero deviation */
    if(rc->ch1 <= RC_DEADBAND && rc->ch1 >= -RC_DEADBAND)
        rc->ch1 = 0;
    if(rc->ch2 <= RC_DEADBAND && rc->ch2 >= -RC_DEADBAND)
        rc->ch2 = 0;
    if(rc->ch3 <= RC_DEADBAND && rc->ch3 >= -RC_DEADBAND)
        rc->ch3 = 0;
    if(rc->ch4 <= RC_DEADBAND && rc->ch4 >= -RC_DEADBAND)
        rc->ch4 = 0;

    rc->sw1 = ((buf[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buf[5] >> 4) & 0x0003;

    if ((abs(rc->ch1) > 660) || \
            (abs(rc->ch2) > 660) || \
            (abs(rc->ch3) > 660) || \
            (abs(rc->ch4) > 660))
    {
        memset(rc, 0, sizeof(RC_Info_t));
        return ;
    }

    rc->mouse.x = buf[6] | (buf[7] << 8); // x axis
    rc->mouse.y = buf[8] | (buf[9] << 8);
    rc->mouse.z = buf[10] | (buf[11] << 8);

    rc->mouse.l = buf[12];
    rc->mouse.r = buf[13];

    rc->kb.key_code = buf[14] | buf[15] << 8; // key borad code
    rc->wheel = (buf[16] | buf[17] << 8) - 1024;
}

void RC_SwitchAction(RC_Switch_t *sw, uint8_t value)
{
    /* 最新状态值 */
    sw->switch_value_raw = value;

    /* 取最新值和上一次值 */
    sw->switch_state = sw->last_switch_value_raw << 2 | sw->switch_value_raw;

    /* 更新上一次值 */
    sw->last_switch_value_raw = sw->switch_value_raw;
}

RC_Info_t* RC_GetDataPointer(void)
{
    return &m_rc_info;
}
