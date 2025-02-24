#ifndef CLIENT_UI_BASE_H
#define CLIENT_UI_BASE_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "comm_protocol.h"
#include "referee_system.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum            //命令码
{
    DELETE_UI_DATA_CMD_ID = 0x100,              //删除命令
    DRAW_GRAPHIC_SINGLE_DATA_CMD_ID = 0x101,    //一个图形
    DRAW_GRAPHIC_DOUBLE_DATA_CMD_ID = 0x102,    //二个图形
    DRAW_GRAPHIC_FIVE_DATA_CMD_ID   = 0x103,    //五个图形
    DRAW_GRAPHIC_SEVEN_DATA_CMD_ID  = 0x104,    //七个图形
    DRAW_STRING_DATA_CMD_ID         = 0x110,    //字符串
} UI_CMD_ID_e;

typedef enum            //客户端ID
{
    RED_HERO_CLIENT        = 0x0101,
    RED_ENGINEER_CLIENT    = 0x0102,
    RED_INFANTRY_1_CLIENT  = 0x0103,
    RED_INFANTRY_2_CLIENT  = 0x0104,
    RED_INFANTRY_3_CLIENT  = 0x0105,
    RED_AERIAL_CLIENT      = 0x0106,
    BLUE_HERO_CLIENT       = 0x0165,
    BLUE_ENGINEER_CLIENT   = 0x0166,
    BLUE_INFANTRY_1_CLIENT = 0x0167,
    BLUE_INFANTRY_2_CLIENT = 0x0168,
    BLUE_INFANTRY_3_CLIENT = 0x0169,
    BLUE_AERIAL_CLIENT     = 0x016A,
} UI_ClientID_e;

typedef enum            //命令码：0x100，删除操作
{
    UI_NOTHING_DELETE_OPERATE   = 0,    //无操作
    UI_DELETE_LAYER_OPERATE     = 1,    //删除一个图层中的图形
    UI_DELETE_ALL_OPERATE       = 2     //删除所有图形
} UI_DeleteOperate_e;

typedef enum            //图形绘图时的操作
{
    UI_NOTHING_OPERATE = 0, //无操作
    UI_INSERT_OPERATE = 1,  //增加
    UI_UPDATE_OPERATE = 2,  //跟新
    UI_DELETE_OPERATE = 3,  //删除
} UI_OperateType_e;

typedef enum
{
    UI_GRAPHIC_LINE         = 0,    //直线
    UI_GRAPHIC_RECTANGLE    = 1,    //矩形
    UI_GRAPHIC_CIRCLE       = 2,    //园
    UI_GRAPHIC_ELLIPSE      = 3,    //椭圆
    UI_GRAPHIC_CIRCLE_ARC   = 4,    //圆弧
    UI_GRAPHIC_FLOAT        = 5,    //浮点数
    UI_GRAPHIC_INT          = 6,    //整数
    UI_GRAPHIC_STRING       = 7,    //字符
} UI_GraphicType_e;

typedef enum
{
    UI_COLOR_MAIN   = 0,    //红蓝主色
    UI_COLOR_YELLOW = 1,    //黄色
    UI_COLOR_GREEN  = 2,    //绿色
    UI_COLOR_ORANGE = 3,    //橙色
    UI_COLOR_PURPLISH_RED = 4,//紫红色
    UI_COLOR_PINK   = 5,    //粉色
    UI_COLOR_CYAN   = 6,    //青色
    UI_COLOR_BLACK  = 7,    //黑色
    UI_COLOR_WHITE  = 8,    //白色
} UI_Color_e;

typedef ext_student_interactive_header_data_t UiHeader_t;
typedef ext_client_custom_graphic_delete_t UiDelete_t;
typedef graphic_data_struct_t UiGraphicData_t;

#pragma pack(push,1)
typedef struct
{
    uint8_t graphic_name[3];    //在删除，修改等操作中，作为客户端的索引。
    uint32_t operate_tpye:3;    //图形操作：0：空操作；1：增加；2：修改；3：删除；
    uint32_t graphic_tpye:3;    //图形类型：0：直线；1：矩形；2：整圆；3：椭圆；4：圆弧；4：圆弧；6：整型数；7：字符；
    uint32_t layer:4;           //图层数，0~9
    uint32_t color:4;           //颜色：0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
    uint32_t font_size:9;       //字体大小
    uint32_t decimals:9;        //小数位数
    uint32_t width:10;          //线宽；
    uint32_t start_x:11;        //起点 x 坐标；
    uint32_t start_y:11;        //起点 y 坐标。
    int32_t number;             //显示的数值
} UiNumberData_t;

typedef struct
{
    uint8_t graphic_name[3];    //在删除，修改等操作中，作为客户端的索引。
    uint32_t operate_tpye:3;    //图形操作：0：空操作；1：增加；2：修改；3：删除；
    uint32_t graphic_tpye:3;    //图形类型：0：直线；1：矩形；2：整圆；3：椭圆；4：圆弧；4：圆弧；6：整型数；7：字符；
    uint32_t layer:4;           //图层数，0~9
    uint32_t color:4;           //颜色：0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
    uint32_t font_size:9;       //字体大小
    uint32_t len:9;             //字符串长度
    uint32_t width:10;          //线宽；
    uint32_t start_x:11;        //起点 x 坐标；
    uint32_t start_y:11;        //起点 y 坐标。
    int32_t reserved;           //占位，无用处
    uint8_t str[30];            //显示的字符串
} UiStringData_t;
#pragma pack(pop)
/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void ClientUI_Init(TransmitHandle_t* p_handle);
void ClientUI_SetHeaderCmdID(uint16_t cmd_id);
void ClientUI_SetHeaderSenderID(uint16_t sender_ID);
void ClientUI_SetHeaderReceiverID(uint16_t receiver_ID);
void ClientUI_Delete(UI_DeleteOperate_e operate, uint8_t layer);
void ClientUI_DrawLine(UiGraphicData_t* graphic_data, char* graphic_name, uint8_t layer,
                       int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y, uint16_t width, uint8_t color);
void ClientUI_DrawRectangle(UiGraphicData_t* graphic_data, char* graphic_name, uint8_t layer,
                            int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y, uint16_t width, uint8_t color);
void ClientUI_DrawCircle(UiGraphicData_t* graphic_data, char* graphic_name, uint8_t layer,
                         int16_t center_x, int16_t center_y, uint16_t radius, uint16_t width, uint8_t color);
void ClientUI_DrawCircleArc(UiGraphicData_t* graphic_data, char* graphic_name, uint8_t layer,
                            int16_t center_x, int16_t center_y, uint16_t radius_x, uint16_t radius_y,
                            uint16_t start_angle, uint16_t end_angle, uint16_t width, uint8_t color);
void ClientUI_DrawEllipse(UiGraphicData_t* graphic_data, char* graphic_name, uint8_t layer,
                          int16_t center_x, int16_t center_y, uint16_t radius_x, uint16_t radius_y, uint16_t width, uint8_t color);
void ClientUI_DrawFloatNumber(UiNumberData_t* number_data, char* graphic_name, uint8_t layer,
                              int16_t start_x, int16_t start_y, fp32 num, uint16_t font_size, uint16_t width, uint8_t color);
void ClientUI_DrawIntNumber(UiNumberData_t* number_data, char* graphic_name, uint8_t layer,
                            int16_t start_x, int16_t start_y, int32_t num, uint16_t font_size, uint16_t width, uint8_t color);
void ClientUI_DrawString(UiStringData_t* str_data, char* graphic_name, uint8_t layer,
                         int16_t start_x, int16_t start_y, char* string, uint16_t font_size, uint16_t width, uint8_t color);
void ClientUI_UpdateString(UiStringData_t* str_data, char* string);
void ClientUI_SetPosition(void* graphic_data, int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y);
void ClientUI_SetLineWidth(void* graphic_data, uint16_t width);
void ClientUI_SetColor(void* graphic_data, uint8_t color);
void ClientUI_SetNumber(UiNumberData_t* number_data, int32_t num);
void ClientUI_SetFloatNumber(UiNumberData_t* number_data, fp32 num);
void ClientUI_Update(void* ui1, void* ui2, void* ui3,void* ui4, void* ui5, void* ui6, void* ui7);


#endif  // CLIENT_UI_BASE_H

