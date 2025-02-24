/*******************************************************************************
 * Robofuture RM Team
 * File name: bsp_i2c.c
 * Author: Zhb        Version: 1.0        Date: 2021/3/12
 * Description: 使用IO模拟I2C，快速使用I2C读写
 * Function List:
 *   1. BSP_I2C_NewObject 创建I2C管理对象
 *   2. BSP_I2C_Read_Len 读取I2C设备指定寄存器值
 *   3. BSP_I2C_Write_Len 写入I2C设备指定寄存器值
 *   4. BSP_I2C_Read_Data 读取I2C设备指定寄存器的一个值
 *   5. BSP_I2C_Write_Data 写入I2C设备指定寄存器的一个值
 * History:
 *      <author> <time>  <version > <desc>
 *        Zhb   21/03/12  1.0       首次提交
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_i2c.h"
#include "bsp_delay.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define HIGH GPIO_PIN_SET
#define LOW  GPIO_PIN_RESET

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: I2C_Delay
 * Description: I2C使用延时函数，us级延时
 * Input: 无
 * Return: 无
*************************************************/
static void I2C_Delay(uint16_t nus)
{
    BSP_DelayUs(nus);
}

/*************************************************
 * Function: BSP_I2C_NewObject
 * Description: 创建I2C对象
 * Input: i2c I2C对象指针
 *        SDA_Port I2C SDA端口
 *        SDA_Pin  I2C SDA引脚
 *        SCL_Port I2C SCL引脚
 *        SCL_Pin  I2C SCL引脚
 * Return: 无
*************************************************/
BSP_Status_e BSP_I2C_NewObject(I2C_Object_t* i2c, GPIO_TypeDef* SDA_Port, uint16_t SDA_Pin, GPIO_TypeDef* SCL_Port, uint16_t SCL_Pin)
{
    BSP_GPIO_NewObject(&i2c->SDA, GPIO_OUTPUT_OD_DEV, SDA_Port, SDA_Pin);
    BSP_GPIO_NewObject(&i2c->SCL, GPIO_INPUT_DEV, SCL_Port, SCL_Pin);
    return BSP_OK;
}

/*************************************************
 * Function: I2C_SDA_Write
 * Description: SDA控制引脚输出电平
 * Input: i2c I2C对象指针
 *        state 写入高低电平，选择如下
 *                           GPIO_PIN_RESET 低电平
 *                           GPIO_PIN_SET 高电平
 * Return: 无
*************************************************/
static void I2C_SDA_Write(I2C_Object_t* i2c, GPIO_PinState state)
{
    HAL_GPIO_WritePin(i2c->SDA.handle, i2c->SDA.value, state);
}

/*************************************************
 * Function: I2C_SDA_Read
 * Description: SDA引脚读取电平
 * Input: i2c I2C对象指针
 * Return: 无
*************************************************/
static GPIO_PinState I2C_SDA_Read(I2C_Object_t* i2c)
{
    return HAL_GPIO_ReadPin(i2c->SDA.handle, i2c->SDA.value);
}

/*************************************************
 * Function: I2C_SCL_Write
 * Description: SCL控制引脚输出电平
 * Input: i2c I2C对象指针
 *        state 写入高低电平，选择如下
 *                           GPIO_PIN_RESET 低电平
 *                           GPIO_PIN_SET 高电平
 * Return: 无
*************************************************/
static void I2C_SCL_Write(I2C_Object_t* i2c, GPIO_PinState state)
{
    HAL_GPIO_WritePin(i2c->SCL.handle, i2c->SCL.value, state);
}

/*************************************************
 * Function: I2C_Start
 * Description: I2C起始信号
 * Input: i2c I2C对象指针
 * Return: 无
*************************************************/
static void I2C_Start(I2C_Object_t* i2c)
{
    I2C_SDA_Write(i2c, HIGH);
    I2C_SCL_Write(i2c, HIGH);
    I2C_Delay(5);
    I2C_SDA_Write(i2c, LOW);    // START:when CLK is high,DATA change form high to low
    I2C_Delay(5);
    I2C_SCL_Write(i2c, LOW);    // 钳住I2C总线，准备发送或接收数据
    I2C_Delay(5);
}

/*************************************************
 * Function: I2C_Stop
 * Description: I2C停止信号
 * Input: i2c I2C对象指针
 * Return: 无
*************************************************/
static void I2C_Stop(I2C_Object_t* i2c)
{
    I2C_SDA_Write(i2c, LOW);    // STOP:when CLK is high DATA change form low to high
    I2C_SCL_Write(i2c, HIGH);
    I2C_Delay(5);
    I2C_SDA_Write(i2c, HIGH);   // 发送I2C总线结束信号
}

/*************************************************
 * Function: I2C_WaitAck
 * Description: 等待应答信号到来
 * Input: i2c I2C对象指针
 * Return: 无
*************************************************/
static uint8_t I2C_WaitAck(I2C_Object_t* i2c)
{
    uint32_t re;

    I2C_SDA_Write(i2c, HIGH);
    I2C_Delay(5);
    I2C_SCL_Write(i2c, HIGH);
    I2C_Delay(5);
    if (I2C_SDA_Read(i2c)) {
        re = 1;
    } else {
        re = 0;
    }
    I2C_SCL_Write(i2c, LOW);
    I2C_Delay(5);
    return re;
}

/*************************************************
 * Function: I2C_Ack
 * Description: 产生ACK应答
 * Input: i2c I2C对象指针
 * Return: 无
*************************************************/
static void I2C_Ack(I2C_Object_t* i2c)
{
    I2C_SDA_Write(i2c, LOW);
    I2C_Delay(5);
    I2C_SCL_Write(i2c, HIGH);
    I2C_Delay(5);
    I2C_SCL_Write(i2c, LOW);
    I2C_Delay(5);
    I2C_SDA_Write(i2c, HIGH);
}

/*************************************************
 * Function: I2C_NAck
 * Description: 产生NACK应答
 * Input: i2c I2C对象指针
 * Return: 无
*************************************************/
static void I2C_NAck(I2C_Object_t* i2c)
{
    I2C_SDA_Write(i2c, HIGH);
    I2C_Delay(5);
    I2C_SCL_Write(i2c, HIGH);
    I2C_Delay(5);
    I2C_SCL_Write(i2c, LOW);
    I2C_Delay(5);
}

/*************************************************
 * Function: I2C_SendByte
 * Description: 发送1个字节
 * Input: i2c I2C对象指针
 *        ack 1,发送ACK 0,发送nACK
 * Return: 无
*************************************************/
static void I2C_SendByte(I2C_Object_t* i2c, uint8_t byte)
{
    uint8_t i;
    for (i=0; i<8; i++) {
        if (byte & 0x80) {
            I2C_SDA_Write(i2c, HIGH);
        } else {
            I2C_SDA_Write(i2c, LOW);
        }
        I2C_Delay(5);
        I2C_SCL_Write(i2c, HIGH);
        I2C_Delay(5);
        I2C_SCL_Write(i2c, LOW);
        if (i == 7) {
            I2C_SDA_Write(i2c, HIGH);
        }
        byte <<= 1;
        I2C_Delay(5);
    }
}

/*************************************************
 * Function: I2C_ReadByte
 * Description: 接收1个字节
 * Input: i2c I2C对象指针
 *        ack 1,发送ACK 0,发送nACK
 * Return: 返回接收值
*************************************************/
static uint8_t I2C_ReadByte(I2C_Object_t* i2c, uint8_t ack)
{
    uint8_t i, value=0;
//    SDA_Input(p_i2c);//sda输入
    for (i=0; i<8; i++ ) {
        value <<= 1;
        I2C_SCL_Write(i2c, HIGH);
        I2C_Delay(5);
        if(I2C_SDA_Read(i2c))value++;
        I2C_SCL_Write(i2c, LOW);
        I2C_Delay(5);
    }
    if (ack) {
        I2C_Ack(i2c); //发送ACK
    } else {
        I2C_NAck(i2c);//发送nACK
    }
    return value;
}

/*************************************************
 * Function: BSP_I2C_Read_Len
 * Description: 读取指定设备 指定寄存器的 len个值
 * Input: i2c I2C对象指针
 *        addr 设备地址
 *        reg 读取的寄存器
 *        len 读取字节长度
 *        buf 数据接收缓存
 * Return: BSP_OK 对象创建成功
 *         BSP_ERROR 对象创建失败
*************************************************/
BSP_Status_e BSP_I2C_Read_Len(I2C_Object_t* i2c, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    I2C_Start(i2c);
    I2C_SendByte(i2c, (addr<<1)|0);      //发送写命令
    if (I2C_WaitAck(i2c)) {
        I2C_Stop(i2c);
        return BSP_ERROR;
    }

    I2C_SendByte(i2c, reg);   //发送地址
    I2C_WaitAck(i2c);
    I2C_Start(i2c);
    I2C_SendByte(i2c, (addr<<1)|1);  //进入接收模式
    I2C_WaitAck(i2c);

    while (len) {
        if(len==1)*buf=I2C_ReadByte(i2c, 0);
        else *buf=I2C_ReadByte(i2c, 1);
        len--;
        buf++;
    }
    I2C_Stop(i2c);//产生一个停止条件
    return BSP_OK;
}

/*************************************************
 * Function: BSP_I2C_Write_Len
 * Description: 将多个字节写入指定设备 指定寄存器
 * Input: i2c I2C对象指针
 *        addr 设备地址
 *        reg 写入的寄存器
 *        len 写入字节长度
 *        buf 数据缓存
 * Return: BSP_OK 对象创建成功
 *         BSP_ERROR 对象创建失败
*************************************************/
BSP_Status_e BSP_I2C_Write_Len(I2C_Object_t* i2c, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    uint8_t i;
    I2C_Start(i2c);
    I2C_SendByte(i2c, (addr<<1)|0);
    if (I2C_WaitAck(i2c)) {
        I2C_Stop(i2c);
        return BSP_ERROR;
    }

    I2C_SendByte(i2c, reg);   //发送地址
    I2C_WaitAck(i2c);
    for (i=0; i<len; i++) {
        I2C_SendByte(i2c, buf[i]);
        if(I2C_WaitAck(i2c))
        {
            I2C_Stop(i2c);
            return BSP_ERROR;
        }
    }
    I2C_Stop(i2c);
    return BSP_OK;
}

/*************************************************
 * Function: BSP_I2C_Read_Data
 * Description: 读取指定设备 指定寄存器的一个值
 * Input: i2c I2C对象指针
 *        addr 设备地址
 *        reg 读取的寄存器
 *        data 读取到的数据
 * Return: BSP_OK 对象创建成功
 *         BSP_ERROR 对象创建失败
*************************************************/
BSP_Status_e BSP_I2C_Read_Data(I2C_Object_t* i2c, uint8_t addr, uint8_t reg, uint8_t* data)
{
    return BSP_I2C_Read_Len(i2c, addr, reg, 1, data);
}

/*************************************************
 * Function: BSP_I2C_Read_Data
 * Description: 写入指定设备 指定寄存器一个字节
 * Input: i2c I2C对象指针
 *        addr 设备地址
 *        reg 写入的寄存器
 *        data 写入的数据
 * Return: BSP_OK 对象创建成功
 *         BSP_ERROR 对象创建失败
*************************************************/
BSP_Status_e BSP_I2C_Write_Data(I2C_Object_t* i2c, uint8_t dev, uint8_t reg, uint8_t data)
{
    return BSP_I2C_Write_Len(i2c, dev, reg, 1, &data);
}
