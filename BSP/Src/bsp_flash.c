/*******************************************************************************
 * Robofuture RM Team
 * File name: bsp_flash.c
 * Author: Zhb        Version: 1.0        Date: 2021/3/12
 * Description: 提供快速访问flash的函数
 * Function List:
 *   1. BSP_Flash_EraseAddress 擦除一页flash
 *   2. BSP_Flash_WriteSingleAddress 往一页flash写数据
 *   3. BSP_Flash_WriteMuliAddress 往几页flash写数据
 *   4. BSP_Flash_Read 从flash读数据
 *   5. BSP_Flash_GerSector 获取flash的扇区号
 *   6. BSP_Flash_GetNextFlashAddress 获取下一页flash地址
 * History:
 *      <author> <time>  <version > <desc>
 *        Zhb   21/03/12  1.0       首次提交
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_flash.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: BSP_Flash_EraseAddress
 * Description: 擦除整页flash
 *        address flash地址
 * Input: len 要擦除的页数
 * Return: 无
*************************************************/
void BSP_Flash_EraseAddress(uint32_t address, uint16_t len)
{
    FLASH_EraseInitTypeDef flash_erase;
    uint32_t error;

    flash_erase.Sector = BSP_Flash_GerSector(address);
    flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    flash_erase.NbSectors = len;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&flash_erase, &error);
    HAL_FLASH_Lock();
}

/*************************************************
 * Function: BSP_Flash_WriteSingleAddress
 * Description: 往一页flash中写入数据
 * Input: start_address flash地址
 *        buf 数据指针
 *        len 数据长度
 * Return: BSP_OK 写入成功
 *         BSP_ERROR 写入失败
*************************************************/
BSP_Status_e BSP_Flash_WriteSingleAddress(uint32_t start_address, uint32_t* buf, uint32_t len)
{
    static uint32_t uw_address;
    static uint32_t end_address;
    static uint32_t* data_buf;
    static uint32_t data_len;

    HAL_FLASH_Unlock();

    uw_address = start_address;
    end_address = BSP_Flash_GetNextFlashAddress(start_address);
    data_buf = buf;
    data_len = 0;

    while (uw_address <= end_address)
    {

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,uw_address, *data_buf) == HAL_OK)
        {
            uw_address += 4;
            data_buf++;
            data_len++;
            if (data_len == len)
            {
                break;
            }
        }
        else
        {
            HAL_FLASH_Lock();
            return BSP_ERROR;
        }
    }

    HAL_FLASH_Lock();
    return BSP_OK;
}

/*************************************************
 * Function: BSP_Flash_WriteMuliAddress
 * Description: 往几页flash中写入数据
 * Input: start_address flash起始地址
 *        end_address flash结束地址
 *        buf 数据指针
 *        len 数据长度
 * Return: BSP_OK 写入成功
 *         BSP_ERROR 写入失败
*************************************************/
BSP_Status_e BSP_Flash_WriteMuliAddress(uint32_t start_address, uint32_t end_address, uint32_t* buf, uint32_t len)
{
    uint32_t uw_address = 0;
    uint32_t* data_buf;
    uint32_t data_len;

    HAL_FLASH_Unlock();

    uw_address = start_address;
    data_buf = buf;
    data_len = 0;
    while (uw_address <= end_address)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,uw_address, *data_buf) == HAL_OK)
        {
            uw_address += 4;
            data_buf++;
            data_len++;
            if (data_len == len)
            {
                break;
            }
        }
        else
        {
            HAL_FLASH_Lock();
            return BSP_ERROR;
        }
    }

    HAL_FLASH_Lock(); 
    return BSP_OK;
}

/*************************************************
 * Function: BSP_Flash_Read
 * Description: 从flash中读取数据
 * Input: address flash地址
 *        buf 数据指针
 *        len 数据长度
 * Return: 无
*************************************************/
void BSP_Flash_Read(uint32_t address, uint32_t* buf, uint32_t len)
{
    memcpy(buf, (void*)address, len*4);
}

/*************************************************
 * Function: BSP_Flash_GerSector
 * Description: 获取flash的扇区号
 * Input: address flash地址
 * Return: 扇区号
*************************************************/
uint32_t BSP_Flash_GerSector(uint32_t address)
{
    uint32_t sector = 0;
    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_SECTOR_7;
    }
    else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
    {
        sector = FLASH_SECTOR_8;
    }
    else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
    {
        sector = FLASH_SECTOR_9;
    }
    else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
    {
        sector = FLASH_SECTOR_10;
    }
    else if ((address < ADDR_FLASH_SECTOR_12) && (address >= ADDR_FLASH_SECTOR_11))
    {
        sector = FLASH_SECTOR_11;
    }
    else
    {
        sector = FLASH_SECTOR_11;
    }

    return sector;
}

/*************************************************
 * Function: BSP_Flash_GetNextFlashAddress
 * Description: 获取下一页flash地址
 * Input: address flash地址
 * Return: 下一页flash地址
*************************************************/
uint32_t BSP_Flash_GetNextFlashAddress(uint32_t address)
{
    uint32_t sector = 0;

    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = ADDR_FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = ADDR_FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = ADDR_FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = ADDR_FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = ADDR_FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = ADDR_FLASH_SECTOR_6;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = ADDR_FLASH_SECTOR_7;
    }
    else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
    {
        sector = ADDR_FLASH_SECTOR_8;
    }
    else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
    {
        sector = ADDR_FLASH_SECTOR_9;
    }
    else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
    {
        sector = ADDR_FLASH_SECTOR_10;
    }
    else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
    {
        sector = ADDR_FLASH_SECTOR_11;
    }
    else /*(address < FLASH_END_ADDR) && (address >= ADDR_FLASH_SECTOR_23))*/
    {
        sector = FLASH_END_ADDR;
    }
    return sector;
}
