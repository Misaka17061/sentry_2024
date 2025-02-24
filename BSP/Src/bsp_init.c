/*******************************************************************************
 * Robofuture RM Team
 * File name: bsp_init.c
 * Author: Zhb        Version: 1.0        Date: 2021/3/12
 * Description: 针对开发板初始化BSP各个模块
 * Function List:
 *   1. BSP_Init 初始化BSP各模块
 * History:
 *      <author> <time>  <version > <desc>
 *        Zhb   21/03/12  1.0       首次提交
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_init.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define COM1_TX_BUFFER_SIZE (512)       //512    //100      //底盘512档    //云台100档     //信息缓存影响速率
#define COM1_TX_FIFO_SIZE   (1024)       //1024   //300    
#define COM1_RX_BUFFER_SIZE (512)        //512    //38

#define COM2_TX_BUFFER_SIZE (512)
#define COM2_TX_FIFO_SIZE   (1024)
#define COM2_RX_BUFFER_SIZE (512)

#define DBUS_RX_BUFFER_SIZE (30)
/* 私有变量 ------------------------------------------------------------------*/
uint8_t com1_tx_buffer[COM1_TX_BUFFER_SIZE];
uint8_t com1_tx_fifo_buffer[COM1_TX_FIFO_SIZE];
uint8_t com1_rx_buffer[COM1_RX_BUFFER_SIZE];

uint8_t com2_tx_buffer[COM2_TX_BUFFER_SIZE];
uint8_t com2_tx_fifo_buffer[COM2_TX_FIFO_SIZE];
uint8_t com2_rx_buffer[COM2_RX_BUFFER_SIZE];

uint8_t dbus_rx_buffer[DBUS_RX_BUFFER_SIZE];

uint8_t can1_tx_fifo_buff[CAN_TX_FIFO_SIZE];
uint8_t can2_tx_fifo_buff[CAN_TX_FIFO_SIZE];
/* 全局变量 ------------------------------------------------------------------*/
/* IO对象结构 */
GPIO_Object_t app_gpio;
GPIO_Object_t led_r_gpio;
GPIO_Object_t led_g_gpio;
GPIO_Object_t led_b_gpio;
GPIO_Object_t temp_pwm_gpio;
GPIO_Object_t buzzer_gpio;
GPIO_Object_t laser_gpio;

/* 串口对象结构 */
UART_Object_t com1_obj;  //这里名字使用板子上丝印
UART_Object_t com2_obj;
UART_Object_t dbus_obj;
/* CAN对象结构 */
CAN_Object_t can1_obj;
CAN_Object_t can2_obj;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/


/*************************************************
 * Function: BSP_Init
 * Description: BSP初始化
 * Input: 无
 * Return: 无
*************************************************/
void BSP_Init(void)
{
    BSP_DelayInit();

    BSP_GPIO_NewObject(&app_gpio, GPIO_INPUT_DEV, APP_CONFIG_GPIO_Port, APP_CONFIG_Pin);
    BSP_GPIO_NewObject_PWM(&led_r_gpio, &htim5, TIM_CHANNEL_3);
    BSP_GPIO_NewObject_PWM(&led_g_gpio, &htim5, TIM_CHANNEL_2);
    BSP_GPIO_NewObject_PWM(&led_b_gpio, &htim5, TIM_CHANNEL_1);
    BSP_GPIO_NewObject_PWM(&temp_pwm_gpio, &htim10, TIM_CHANNEL_1);
    BSP_GPIO_NewObject_PWM(&buzzer_gpio, &htim4, TIM_CHANNEL_3);
    BSP_GPIO_NewObject_PWM(&laser_gpio, &htim3, TIM_CHANNEL_3);

    BSP_UART_NewObject(&com1_obj, &huart6);
    BSP_UART_TransmitConfig(&com1_obj, com1_tx_buffer, COM1_TX_BUFFER_SIZE, com1_tx_fifo_buffer, COM1_TX_FIFO_SIZE);
    BSP_UART_ReceiveConfig(&com1_obj, com1_rx_buffer, COM1_RX_BUFFER_SIZE, NULL);
    BSP_UART_NewObject(&com2_obj, &huart1);
    BSP_UART_TransmitConfig(&com2_obj, com2_tx_buffer, COM2_TX_BUFFER_SIZE, com2_tx_fifo_buffer, COM2_TX_FIFO_SIZE);
    BSP_UART_ReceiveConfig(&com2_obj, com2_rx_buffer, COM2_RX_BUFFER_SIZE, NULL);
    BSP_UART_NewObject(&dbus_obj, &huart3);
    BSP_UART_ReceiveConfig(&dbus_obj, dbus_rx_buffer, DBUS_RX_BUFFER_SIZE, NULL);

    BSP_CAN_NewObject(&can1_obj, &hcan1, can1_tx_fifo_buff, NULL);
    BSP_CAN_NewObject(&can2_obj, &hcan2, can2_tx_fifo_buff, NULL);
}
