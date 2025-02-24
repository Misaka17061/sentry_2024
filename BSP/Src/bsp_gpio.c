/*******************************************************************************
 * Robofuture RM Team
 * File name: bsp_gpio.c
 * Author: Zhb        Version: 1.0        Date: 2021/3/12
 * Description: 提供对IO的快速操作对象
 * Function List:
 *   1. BSP_GPIO_NewObject 用于创建普通IO的读写对象
 *   2. BSP_GPIO_ReadPin 读取IO电平
 *   3. BSP_GPIO_WritePin 写入IO电平
 *   4. BSP_GPIO_TogglePin 反转IO电平
 *   5. BSP_GPIO_NewObject_EXTI 创建外部中断IO对象
 *   6. HAL_GPIO_EXTI_Callback 设置外部中断IO回调
 *   7. BSP_GPIO_NewObject_PWM 创建PWM对象
 *   8. BSP_GPIO_SetPwmValue 设置PWM占空比
 *   9. BSP_GPIO_SetPwmPrescaler 设置PWM分频值
 * History:
 *      <author> <time>  <version > <desc>
 *        Zhb   21/03/12  1.0       首次提交
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_gpio.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define GPIO_DEVICE  14 // GPIO设备总数

/* 私有变量 ------------------------------------------------------------------*/
/*
  * 保存创建的GPIO对象指针
 */
static GPIO_Object_t* m_objects[GPIO_DEVICE];

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: BSP_GPIO_NewObject
 * Description: 创建IO设备对象
 * Input: obj 对象指针
 * Input: dev 设备类型，类型如下：
 *                      GPIO_EMPTY_DEV 未定义类型
 *                      GPIO_OUTPUT_PP_DEV 推挽输出IO
 *                      GPIO_OUTPUT_OD_DEV 开漏输出IO
 *                      GPIO_INPUT_DEV 输入设备IO
 * Input: GPIOx IO地址
 * Input: pin 引脚号
 * Return: BSP_OK 对象创建成功
 *         BSP_ERROR 对象创建失败
*************************************************/
BSP_Status_e BSP_GPIO_NewObject(GPIO_Object_t* obj, GPIO_Device_e dev, GPIO_TypeDef* GPIOx, uint16_t pin)
{
    if(obj == NULL)
        return BSP_ERROR;

    assert_param(obj->device == GPIO_INPUT_DEV
            || obj->device == GPIO_OUTPUT_OD_DEV
            || obj->device == GPIO_OUTPUT_PP_DEV);
    assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
    assert_param(IS_GPIO_PIN(pin));

    obj->handle = GPIOx;
    obj->value = pin;
    obj->device = dev;
    return BSP_OK;
}

/*************************************************
 * Function: BSP_GPIO_ReadPin
 * Description: 读取GPIO引脚电平，只有在类型是
 *              GPIO_INPUT_DEV或GPIO_OUTPUT_OD_DEV
  *                             时才可读取
 * Input: obj 对象指针
 * Return: GPIO_PIN_RESET 低电平
 *         GPIO_PIN_SET 高电平
*************************************************/
GPIO_PinState BSP_GPIO_ReadPin(GPIO_Object_t* obj)
{
    assert_param(obj->device == GPIO_INPUT_DEV
            || obj->device == GPIO_OUTPUT_OD_DEV);

    return HAL_GPIO_ReadPin((GPIO_TypeDef *)obj->handle, obj->value);
}

/*************************************************
 * Function: BSP_GPIO_WritePin
 * Description: 写入IO电平，只有在类型是
 *              GPIO_OUTPUT_PP_DEV或GPIO_OUTPUT_OD_DEV
  *                             时才可写入
 * Input: obj 对象指针
 * Input: pin_state 写入高低电平，选择如下
 *                              GPIO_PIN_RESET 低电平
 *                              GPIO_PIN_SET 高电平
 * Return: 无
*************************************************/
void BSP_GPIO_WritePin(GPIO_Object_t* obj, GPIO_PinState pin_state)
{
    assert_param(obj->device == GPIO_OUTPUT_PP_DEV
             || obj->device == GPIO_OUTPUT_OD_DEV);

    HAL_GPIO_WritePin((GPIO_TypeDef *)obj->handle, obj->value, pin_state);
}

/*************************************************
 * Function: BSP_GPIO_TogglePin
 * Description: 反转IO电平，只有在类型GPIO_OUTPUT_PP_DEV
  *                             或GPIO_OUTPUT_OD_DEV时有效
 * Input: obj 对象指针
 * Return: 无
*************************************************/
void BSP_GPIO_TogglePin(GPIO_Object_t* obj)
{
	assert_param(obj->device == GPIO_OUTPUT_PP_DEV
             || obj->device == GPIO_OUTPUT_OD_DEV);

	HAL_GPIO_TogglePin((GPIO_TypeDef *)obj->handle, obj->value);
}

/*************************************************
 * Function: BSP_GPIO_NewObject_EXTI
 * Description: 创建外部中断对象
 * Input: obj 对象指针
 * Input: GPIOx IO地址
 * Input: pin 触发引脚号
 * Input: fun 外部自动触发回调
 * Return: BSP_OK 对象创建成功
 *         BSP_ERROR 对象创建失败
*************************************************/
BSP_Status_e BSP_GPIO_NewObject_EXTI(GPIO_Object_t* obj, GPIO_TypeDef* GPIOx, uint16_t pin, GPIO_EXIT_Callback_t fun)
{
    if(obj == NULL)
        return BSP_ERROR;

    assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
    assert_param(IS_GPIO_PIN(pin));

    obj->handle = GPIOx;
    obj->value = pin;
    obj->device = GPIO_EXTI_DEV;
    obj->callback = fun;

    for (uint8_t i=0; i < GPIO_DEVICE; i++)
    {
        if (m_objects[i] == NULL)
        {
            m_objects[i] = obj;
            return BSP_OK;
        }
    }
    return BSP_ERROR;
}

/*************************************************
 * Function: HAL_GPIO_EXTI_Callback
 * Description: 外部中断回调
 * Input: GPIO_Pin 触发引脚号
 * Return: 无
 * Others: 中断函数
*************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    for (uint8_t i=0; i < GPIO_DEVICE; i++)
    {
        if (m_objects[i]->value == GPIO_Pin)
        {
            if (m_objects[i]->device == GPIO_EXTI_DEV)
                m_objects[i]->callback();
        }
    }
}

/* GPIO PWM CODE BEGIN */
#ifdef HAL_TIM_MODULE_ENABLED
/*************************************************
 * Function: BSP_GPIO_NewObject_PWM
 * Description: 创建PWM控制对象
 * Input: obj 对象指针
 * Input: tim TIM指针
 * Input: channel PWM通道
 * Return: BSP_OK 对象创建成功
 *         BSP_ERROR 对象创建失败
*************************************************/
BSP_Status_e BSP_GPIO_NewObject_PWM(GPIO_Object_t* obj, TIM_HandleTypeDef* tim, uint32_t channel)
{
    obj->handle = tim;
    obj->value = channel;
    obj->device = GPIO_PWM_DEV;
    HAL_TIM_PWM_Start(obj->handle, obj->value);
    return BSP_OK;
}

/*************************************************
 * Function: BSP_GPIO_SetPwmValue
 * Description: 设置PWM占空比值
 * Input: obj 对象指针
 * Input: value 占空比值
 * Return: BSP_OK 对象创建成功
 *         BSP_ERROR 对象创建失败
*************************************************/
void BSP_GPIO_SetPwmValue(GPIO_Object_t* obj, uint16_t value)
{
    assert_param(obj->device == GPIO_PWM_DEV);

    __HAL_TIM_SetCompare((TIM_HandleTypeDef *)obj->handle, obj->value, value);
}

/*************************************************
 * Function: BSP_GPIO_SetPwmPrescaler
 * Description: 设置PWM分频值
 * Input: obj 对象指针
 * Input: value 占空比值
 * Return: BSP_OK 对象创建成功
 *         BSP_ERROR 对象创建失败
*************************************************/
void BSP_GPIO_SetPwmPrescaler(GPIO_Object_t* obj, uint16_t psc)
{
    assert_param(obj->device == GPIO_PWM_DEV);

    __HAL_TIM_PRESCALER((TIM_HandleTypeDef *)obj->handle, psc);
}
#endif
/* GPIO PWM CODE END */


