#ifndef BSP_INIT_H
#define BSP_INIT_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "bsp_def.h"
#include "bsp_delay.h"
#include "bsp_gpio.h"
#include "bsp_i2c.h"
#include "bsp_can.h"
#include "bsp_uart.h"

/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
/* IO对象结构 */
extern GPIO_Object_t app_gpio;
extern GPIO_Object_t led_r_gpio;
extern GPIO_Object_t led_g_gpio;
extern GPIO_Object_t led_b_gpio;
extern GPIO_Object_t temp_pwm_gpio;
extern GPIO_Object_t buzzer_gpio;
extern GPIO_Object_t laser_gpio;
/* 串口对象结构 */
extern UART_Object_t com1_obj;
extern UART_Object_t com2_obj;
extern UART_Object_t dbus_obj;
/* CAN对象结构 */
extern CAN_Object_t can1_obj;
extern CAN_Object_t can2_obj;

/* 函数声明 ------------------------------------------------------------------*/
void BSP_Init(void);

#endif  // BSP_INIT_H

