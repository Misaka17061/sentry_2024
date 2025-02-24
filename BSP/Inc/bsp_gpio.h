#ifndef BSP_GPIO_H
#define BSP_GPIO_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "bsp_def.h"
#include "gpio.h"
#ifdef HAL_TIM_MODULE_ENABLED
#include "tim.h"
#endif

/* 类型定义 ------------------------------------------------------------------*/
typedef void (*GPIO_EXIT_Callback_t)(void);

typedef enum
{
    GPIO_EMPTY_DEV = 0,
    GPIO_OUTPUT_PP_DEV,
    GPIO_OUTPUT_OD_DEV,
    GPIO_INPUT_DEV,
    GPIO_PWM_DEV,
    GPIO_EXTI_DEV,
} GPIO_Device_e;

typedef struct
{
    void* handle;
    uint32_t value;
    GPIO_Device_e device;
    GPIO_EXIT_Callback_t callback;
} GPIO_Object_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
BSP_Status_e BSP_GPIO_NewObject(GPIO_Object_t* obj, GPIO_Device_e dev, GPIO_TypeDef* GPIOx, uint16_t pin);
GPIO_PinState BSP_GPIO_ReadPin(GPIO_Object_t* obj);
void BSP_GPIO_WritePin(GPIO_Object_t* obj, GPIO_PinState pin_state);
void BSP_GPIO_TogglePin(GPIO_Object_t* obj);
BSP_Status_e BSP_GPIO_NewObject_EXTI(GPIO_Object_t* obj, GPIO_TypeDef* GPIOx, uint16_t pin, GPIO_EXIT_Callback_t fun);


#ifdef HAL_TIM_MODULE_ENABLED
BSP_Status_e BSP_GPIO_NewObject_PWM(GPIO_Object_t* obj, TIM_HandleTypeDef* tim, uint32_t channel);
void BSP_GPIO_SetPwmValue(GPIO_Object_t* obj, uint16_t value);
void BSP_GPIO_SetPwmPrescaler(GPIO_Object_t* obj, uint16_t psc);
#endif

#endif /* BSP_GPIO_H */

