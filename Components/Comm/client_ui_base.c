/* 包含头文件 ----------------------------------------------------------------*/
#include "client_ui_base.h"
#include "string.h"
#include "fifo.h"
#include "stdarg.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define CLIENT_UI_SCREEN_PIXELS_WIDTH   (1920)     //客户端显示器分辨率
#define CLIENT_UI_SCREEN_PIXELS_HEIGHT  (1080)
#define CLIENT_UI_DATA_MAX_SIZE 120

/* 私有变量 ------------------------------------------------------------------*/
UiHeader_t header;
TransmitHandle_t* transmit_handle;
uint8_t client_ui_data_packet[CLIENT_UI_DATA_MAX_SIZE] = {0};

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void ClientUI_TransmitData(void* p_data, uint16_t len);

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: ClientUI_Init
 * Description: UI初始化
 * Input:
 * Return:
*************************************************/
void ClientUI_Init(TransmitHandle_t* p_handle)
{
    if (p_handle == NULL)
        return;

    transmit_handle = p_handle;
}

/*************************************************
 * Function: ClientUI_SetHeaderReceiverID
 * Description: 设置命令码
 * Input:
 * Return:
*************************************************/
void ClientUI_SetHeaderCmdID(uint16_t cmd_id)
{
    header.data_cmd_id = cmd_id;
}

/*************************************************
 * Function: ClientUI_SetHeaderReceiverID
 * Description: 设置发送方ID
 * Input:
 * Return:
*************************************************/
void ClientUI_SetHeaderSenderID(uint16_t sender_ID)
{
    header.sender_ID = sender_ID;
}

/*************************************************
 * Function: ClientUI_SetHeaderReceiverID
 * Description: 设置接收方ID
 * Input:
 * Return:
*************************************************/
void ClientUI_SetHeaderReceiverID(uint16_t receiver_ID)
{
    header.receiver_ID = receiver_ID;
}

/*************************************************
 * Function: ClientUI_Delete
 * Description: 删除图层
 * Input:
 * Return:
*************************************************/
void ClientUI_Delete(UI_DeleteOperate_e operate, uint8_t layer)
{
    UiDelete_t ui_delete;
    ClientUI_SetHeaderCmdID(DELETE_UI_DATA_CMD_ID);
    ui_delete.operate_tpye = operate;
    ui_delete.layer = layer;
    ClientUI_TransmitData(&ui_delete, sizeof(UiDelete_t));
}

/*************************************************
 * Function: ClientUI_DrawRectangle
 * Description: 绘制线
 * Input:
 * Return:
*************************************************/
void ClientUI_DrawLine(UiGraphicData_t* graphic_data, char* graphic_name, uint8_t layer,
                       int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y, uint16_t width, uint8_t color)
{
    if (graphic_data == NULL)
        return;

    memset(graphic_data, 0, sizeof(UiGraphicData_t));
    if (strlen(graphic_name) > 3)
        return;
    strcpy((char*)graphic_data->graphic_name, graphic_name);
    graphic_data->operate_tpye = UI_INSERT_OPERATE;
    graphic_data->graphic_tpye = UI_GRAPHIC_LINE;
    graphic_data->layer = layer;

    graphic_data->start_x = CLIENT_UI_SCREEN_PIXELS_WIDTH/2+start_x;
    graphic_data->start_y = CLIENT_UI_SCREEN_PIXELS_HEIGHT/2+start_y;
    graphic_data->end_x = CLIENT_UI_SCREEN_PIXELS_WIDTH/2+end_x;
    graphic_data->end_y = CLIENT_UI_SCREEN_PIXELS_HEIGHT/2+end_y;
    graphic_data->width = width;
    graphic_data->color = color;

    ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_SINGLE_DATA_CMD_ID);
    ClientUI_TransmitData(graphic_data, sizeof(UiGraphicData_t));
}

/*************************************************
 * Function: ClientUI_DrawRectangle
 * Description: 绘制矩形
 * Input:
 * Return:
*************************************************/
void ClientUI_DrawRectangle(UiGraphicData_t* graphic_data, char* graphic_name, uint8_t layer,
                            int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y, uint16_t width, uint8_t color)
{
    if (graphic_data == NULL)
        return;

    memset(graphic_data, 0, sizeof(UiGraphicData_t));
    if (strlen(graphic_name) > 3)
        return;
    strcpy((char*)graphic_data->graphic_name, graphic_name);
    graphic_data->operate_tpye = UI_INSERT_OPERATE;
    graphic_data->graphic_tpye = UI_GRAPHIC_RECTANGLE;
    graphic_data->layer = layer;

    graphic_data->start_x = CLIENT_UI_SCREEN_PIXELS_WIDTH/2+start_x;
    graphic_data->start_y = CLIENT_UI_SCREEN_PIXELS_HEIGHT/2+start_y;
    graphic_data->end_x = CLIENT_UI_SCREEN_PIXELS_WIDTH/2+end_x;
    graphic_data->end_y = CLIENT_UI_SCREEN_PIXELS_HEIGHT/2+end_y;
    graphic_data->width = width;
    graphic_data->color = color;

    ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_SINGLE_DATA_CMD_ID);
    ClientUI_TransmitData(graphic_data, sizeof(UiGraphicData_t));
}

/*************************************************
 * Function: ClientUI_DrawCircle
 * Description: 绘制圆
 * Input:
 * Return:
*************************************************/
void ClientUI_DrawCircle(UiGraphicData_t* graphic_data, char* graphic_name, uint8_t layer,
                         int16_t center_x, int16_t center_y, uint16_t radius, uint16_t width, uint8_t color)
{
    if (graphic_data == NULL)
        return;

    memset(graphic_data, 0, sizeof(UiGraphicData_t));
    if (strlen(graphic_name) > 3)
        return;
    strcpy((char*)graphic_data->graphic_name, graphic_name);
    graphic_data->operate_tpye = UI_INSERT_OPERATE;
    graphic_data->graphic_tpye = UI_GRAPHIC_CIRCLE;
    graphic_data->layer = layer;

    graphic_data->start_x = CLIENT_UI_SCREEN_PIXELS_WIDTH/2+center_x;
    graphic_data->start_y = CLIENT_UI_SCREEN_PIXELS_HEIGHT/2+center_y;
    graphic_data->radius = radius;
    graphic_data->width = width;
    graphic_data->color = color;

    ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_SINGLE_DATA_CMD_ID);
    ClientUI_TransmitData(graphic_data, sizeof(UiGraphicData_t));
}

/*************************************************
 * Function: ClientUI_DrawCircleArc
 * Description: 绘制圆弧
 * Input:
 * Return:
*************************************************/
void ClientUI_DrawCircleArc(UiGraphicData_t* graphic_data, char* graphic_name, uint8_t layer,
                            int16_t center_x, int16_t center_y, uint16_t radius_x, uint16_t radius_y,
                            uint16_t start_angle, uint16_t end_angle, uint16_t width, uint8_t color)
{
    if (graphic_data == NULL)
        return;

    memset(graphic_data, 0, sizeof(UiGraphicData_t));
    if (strlen(graphic_name) > 3)
        return;
    strcpy((char*)graphic_data->graphic_name, graphic_name);
    graphic_data->operate_tpye = UI_INSERT_OPERATE;
    graphic_data->graphic_tpye = UI_GRAPHIC_CIRCLE_ARC;
    graphic_data->layer = layer;

    graphic_data->start_x = CLIENT_UI_SCREEN_PIXELS_WIDTH/2+center_x;
    graphic_data->start_y = CLIENT_UI_SCREEN_PIXELS_HEIGHT/2+center_y;
    graphic_data->end_x = radius_x;
    graphic_data->end_y = radius_y;
    graphic_data->start_angle = start_angle;
    graphic_data->end_angle = end_angle;
    graphic_data->width = width;
    graphic_data->color = color;

    ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_SINGLE_DATA_CMD_ID);
    ClientUI_TransmitData(graphic_data, sizeof(UiGraphicData_t));
}

/*************************************************
 * Function: ClientUI_DrawEllipse
 * Description: 绘制椭圆
 * Input:
 * Return:
*************************************************/
void ClientUI_DrawEllipse(UiGraphicData_t* graphic_data, char* graphic_name, uint8_t layer,
                          int16_t center_x, int16_t center_y, uint16_t radius_x, uint16_t radius_y, uint16_t width, uint8_t color)
{
    if (graphic_data == NULL)
        return;

    memset(graphic_data, 0, sizeof(UiGraphicData_t));
    if (strlen(graphic_name) > 3)
        return;
    strcpy((char*)graphic_data->graphic_name, graphic_name);
    graphic_data->operate_tpye = UI_INSERT_OPERATE;
    graphic_data->graphic_tpye = UI_GRAPHIC_ELLIPSE;
    graphic_data->layer = layer;

    graphic_data->start_x = CLIENT_UI_SCREEN_PIXELS_WIDTH/2+center_x;
    graphic_data->start_y = CLIENT_UI_SCREEN_PIXELS_HEIGHT/2+center_y;
    graphic_data->end_x = radius_x;
    graphic_data->end_y = radius_y;
    graphic_data->width = width;
    graphic_data->color = color;

    ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_SINGLE_DATA_CMD_ID);
    ClientUI_TransmitData(graphic_data, sizeof(UiGraphicData_t));
}

/*************************************************
 * Function: ClientUI_DrawFloatNumber
 * Description: 绘制浮点型数据
 * Input:
 * Return:
*************************************************/
void ClientUI_DrawFloatNumber(UiNumberData_t* number_data, char* graphic_name, uint8_t layer,
                              int16_t start_x, int16_t start_y, fp32 num, uint16_t font_size, uint16_t width, uint8_t color)
{
    if (number_data == NULL)
        return;

    memset(number_data, 0, sizeof(UiNumberData_t));
    if (strlen(graphic_name) > 3)
        return;
    strcpy((char*)number_data->graphic_name, graphic_name);
    number_data->operate_tpye = UI_INSERT_OPERATE;
    number_data->graphic_tpye = UI_GRAPHIC_FLOAT;
    number_data->layer = layer;

    number_data->start_x = CLIENT_UI_SCREEN_PIXELS_WIDTH/2+start_x;
    number_data->start_y = CLIENT_UI_SCREEN_PIXELS_HEIGHT/2+start_y;
    number_data->font_size = font_size;
    number_data->decimals = 3;
    number_data->width = width;
    number_data->color = color;
    number_data->number = num * 1000;

    ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_SINGLE_DATA_CMD_ID);
    ClientUI_TransmitData(number_data, sizeof(UiGraphicData_t));
}

/*************************************************
 * Function: ClientUI_DrawIntNumber
 * Description: 绘制整型数据
 * Input:
 * Return:
*************************************************/
void ClientUI_DrawIntNumber(UiNumberData_t* number_data, char* graphic_name, uint8_t layer,
                            int16_t start_x, int16_t start_y, int32_t num, uint16_t font_size, uint16_t width, uint8_t color)
{
    if (number_data == NULL)
        return;

    memset(number_data, 0, sizeof(UiNumberData_t));
    if (strlen(graphic_name) > 3)
        return;
    strcpy((char*)number_data->graphic_name, graphic_name);
    number_data->operate_tpye = UI_INSERT_OPERATE;
    number_data->graphic_tpye = UI_GRAPHIC_INT;
    number_data->layer = layer;

    number_data->start_x = CLIENT_UI_SCREEN_PIXELS_WIDTH/2+start_x;
    number_data->start_y = CLIENT_UI_SCREEN_PIXELS_HEIGHT/2+start_y;
    number_data->font_size = font_size;
    number_data->decimals = 3;
    number_data->width = width;
    number_data->color = color;
    number_data->number = num;

    ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_SINGLE_DATA_CMD_ID);
    ClientUI_TransmitData(number_data, sizeof(UiNumberData_t));
}

/*************************************************
 * Function: ClientUI_DrawString
 * Description: 绘制字符串
 * Input:
 * Return:
*************************************************/
void ClientUI_DrawString(UiStringData_t* str_data, char* graphic_name, uint8_t layer,
                         int16_t start_x, int16_t start_y, char* string, uint16_t font_size, uint16_t width, uint8_t color)
{
    if (str_data == NULL)
        return;

    memset(str_data, 0, sizeof(UiStringData_t));
    if (strlen(graphic_name) > 3 || strlen(string) > 30)
        return;
    strcpy((char*)str_data->graphic_name, graphic_name);
    str_data->operate_tpye = UI_INSERT_OPERATE;
    str_data->graphic_tpye = UI_GRAPHIC_STRING;
    str_data->layer = layer;

    str_data->start_x = CLIENT_UI_SCREEN_PIXELS_WIDTH/2+start_x;
    str_data->start_y = CLIENT_UI_SCREEN_PIXELS_HEIGHT/2+start_y;
    str_data->font_size = font_size;
    str_data->len = strlen(string);
    str_data->width = width;
    str_data->color = color;

    strcpy((char*)str_data->str, string);

    ClientUI_SetHeaderCmdID(DRAW_STRING_DATA_CMD_ID);
    ClientUI_TransmitData(str_data, sizeof(UiGraphicData_t) + strlen(string));
}

/*************************************************
 * Function: ClientUI_UpdateString
 * Description: 跟新字符串
 * Input:
 * Return:
*************************************************/
void ClientUI_UpdateString(UiStringData_t* str_data, char* string)
{
    if (str_data == NULL || strlen(string) > 30 || str_data->graphic_tpye != UI_GRAPHIC_STRING)
        return;

    strcpy((char*)str_data->str, string);

    ClientUI_SetHeaderCmdID(DRAW_STRING_DATA_CMD_ID);
    ClientUI_TransmitData(str_data, sizeof(UiGraphicData_t) + strlen(string));
}

/*************************************************
 * Function: ClientUI_SetPosition
 * Description: 修改显示的位置，使用是注意数据含义
 * Input:
 * Return:
*************************************************/
void ClientUI_SetPosition(void* graphic_data, int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y)
{
    if (graphic_data == NULL)
        return;
    ((UiGraphicData_t*)graphic_data)->start_x = CLIENT_UI_SCREEN_PIXELS_WIDTH/2+start_x;
    ((UiGraphicData_t*)graphic_data)->start_y = CLIENT_UI_SCREEN_PIXELS_HEIGHT/2+start_y;
    ((UiGraphicData_t*)graphic_data)->end_x = CLIENT_UI_SCREEN_PIXELS_WIDTH/2+end_x;
    ((UiGraphicData_t*)graphic_data)->end_y = CLIENT_UI_SCREEN_PIXELS_HEIGHT/2+end_y;
}

/*************************************************
 * Function: ClientUI_SetLineWidth
 * Description: 修改显示的线宽
 * Input:
 * Return:
*************************************************/
void ClientUI_SetLineWidth(void* graphic_data, uint16_t width)
{
    if (graphic_data == NULL)
        return;
    ((UiGraphicData_t*)graphic_data)->width = width;
}

/*************************************************
 * Function: ClientUI_SetColor
 * Description: 修改显示的颜色
 * Input:
 * Return:
*************************************************/
void ClientUI_SetColor(void* graphic_data, uint8_t color)
{
    if (graphic_data == NULL || color > UI_COLOR_WHITE)
        return;
    ((UiGraphicData_t*)graphic_data)->color = color;
}

/*************************************************
 * Function: ClientUI_SetNumber
 * Description: 修改整型显示数据
 * Input:
 * Return:
*************************************************/
void ClientUI_SetNumber(UiNumberData_t* number_data, int32_t num)
{
    if (number_data == NULL || number_data->graphic_tpye != UI_GRAPHIC_INT)
        return;
    number_data->number = num;
}

/*************************************************
 * Function: ClientUI_SetFloatNumber
 * Description: 修改浮点型显示数据
 * Input:
 * Return:
*************************************************/
void ClientUI_SetFloatNumber(UiNumberData_t* number_data, fp32 num)
{
    if (number_data == NULL || number_data->graphic_tpye != UI_GRAPHIC_FLOAT)
        return;
    number_data->number = num * 1000;
}

/*************************************************
 * Function: ClientUI_Update
 * Description: UI刷新，修改数据后调用
 * Input:
 * Return:
*************************************************/
void ClientUI_Update(void* ui1, void* ui2, void* ui3,void* ui4, void* ui5, void* ui6, void* ui7)
{
    uint8_t data_cnt = 0;
    uint8_t use_len = 0;
    UiGraphicData_t* p_graphic[7] = {ui1, ui2, ui3, ui4, ui5, ui6, ui7};
    uint8_t* p_data_packet = client_ui_data_packet;

    for (uint8_t i = 0; i < 7; i++)
    {
        if (p_graphic[i] != NULL)
            data_cnt++;
    }

    if (data_cnt == 1)
    {
        use_len = 1;
        ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_SINGLE_DATA_CMD_ID);
    }
    else if (data_cnt == 2)
    {
        use_len = 2;
        ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_DOUBLE_DATA_CMD_ID);
    }
    else if (data_cnt <= 5)
    {
        use_len = 5;
        ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_FIVE_DATA_CMD_ID);
    }
    else if (data_cnt <= 7)
    {
        use_len = 7;
        ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_SEVEN_DATA_CMD_ID);
    }
    else
        return;

    memcpy(p_data_packet, &header, sizeof(UiHeader_t));
    p_data_packet += sizeof(UiHeader_t);

    for (uint8_t i = 0; i < 7; i++)
    {
        if (p_graphic[i] != NULL)
        {
            p_graphic[i]->operate_tpye = UI_UPDATE_OPERATE;
            memcpy(p_data_packet, p_graphic[i], sizeof(UiGraphicData_t));
            p_data_packet += sizeof(UiGraphicData_t);
        }
    }

    if (data_cnt < use_len)
    {
        UiGraphicData_t none_data = {0};
        for (; data_cnt < use_len; data_cnt++)
        {
            memcpy(p_data_packet, &none_data, sizeof(UiGraphicData_t));
            p_data_packet += sizeof(UiGraphicData_t);
        }
    }

    Comm_TransmitData(transmit_handle, REFEREE_SYSTEM_HEADER_SOF, CLIENT_CUSTOM_DATA_CMD_ID,
                      client_ui_data_packet, sizeof(UiHeader_t) + sizeof(UiGraphicData_t) * data_cnt);
}

//void ClientUI_Update(uint8_t data_cnt, ...)
//{
//    va_list args;
//    UiGraphicData_t* p_graphic;
//    uint8_t* p_data_packet = client_ui_data_packet;
//
//    if (data_cnt == 1)
//        ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_SINGLE_DATA_CMD_ID);
//    else if (data_cnt == 2)
//        ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_DOUBLE_DATA_CMD_ID);
//    else if (data_cnt == 5)
//        ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_FIVE_DATA_CMD_ID);
//    else if (data_cnt == 7)
//        ClientUI_SetHeaderCmdID(DRAW_GRAPHIC_SEVEN_DATA_CMD_ID);
//    else
//        return;
//
//    memcpy(p_data_packet, &header, sizeof(UiHeader_t));
//    p_data_packet += sizeof(UiHeader_t);
//    va_start(args, data_cnt);
//
//    for (uint8_t i = 0; i<data_cnt; i++)
//    {
//        p_graphic = va_arg(args, UiGraphicData_t*);
//        p_graphic->operate_tpye = UI_UPDATE_OPERATE;
//        memcpy(p_data_packet, p_graphic, sizeof(UiGraphicData_t));
//        p_data_packet += sizeof(UiGraphicData_t);
//    }
//    va_end(args);
//    Comm_TransmitData(transmit_handle, REFEREE_SYSTEM_HEADER_SOF, CLIENT_CUSTOM_DATA_CMD_ID,
//                      client_ui_data_packet, sizeof(UiHeader_t) + sizeof(UiGraphicData_t) * data_cnt);
//}

/*************************************************
 * Function: ClientUI_TransmitData
 * Description: 发送底层函数
 * Input:
 * Return:
*************************************************/
static void ClientUI_TransmitData(void* p_data, uint16_t len)
{
    if (transmit_handle == NULL)
        return;
    memcpy(client_ui_data_packet, &header, sizeof(UiHeader_t));
    memcpy(client_ui_data_packet + sizeof(UiHeader_t), p_data, len);
    Comm_TransmitData(transmit_handle, REFEREE_SYSTEM_HEADER_SOF, CLIENT_CUSTOM_DATA_CMD_ID,
                      client_ui_data_packet, sizeof(UiHeader_t) + len);
}
