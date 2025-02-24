#ifndef BSP_I2C_H
#define BSP_I2C_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "bsp_def.h"
#include "bsp_gpio.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
    GPIO_Object_t SDA;
    GPIO_Object_t SCL;
} I2C_Object_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
BSP_Status_e BSP_I2C_NewObject(I2C_Object_t* p_i2c, GPIO_TypeDef* SDA_Port, uint16_t SDA_Pin, GPIO_TypeDef* SCL_Port, uint16_t SCL_Pin);
BSP_Status_e BSP_I2C_Read_Len(I2C_Object_t* p_i2c, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);
BSP_Status_e BSP_I2C_Write_Len(I2C_Object_t* p_i2c, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);
BSP_Status_e BSP_I2C_Read_Data(I2C_Object_t* p_i2c, uint8_t addr, uint8_t reg, uint8_t* data);
BSP_Status_e BSP_I2C_Write_Data(I2C_Object_t* p_i2c, uint8_t dev, uint8_t reg, uint8_t data);


#endif /* BSP_I2C_H */

