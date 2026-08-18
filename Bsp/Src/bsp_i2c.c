//
// Created by kun on 25-7-17.
//

#include "stm32f4xx_hal.h"
#include "../Inc/bsp_i2c.h"

#include "../Inc/bsp_tick.h"
#include "i2c.h"
/**
  * @brief          通过I2C读取ist8310的一个字节
  * @param[in]      寄存器地址
  * @retval         寄存器值
  */
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    //static const uint16_t IIC_time = 2000;
    uint8_t reg_data;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS << 1,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &reg_data,
        1,
        1000);
    //ist8310_delay_us(IIC_time);
    return reg_data;
}
/**
  * @brief          通过I2C写入一个字节到ist8310的寄存器中
  * @param[in]      寄存器地址
  * @retval         写入值
  */
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write( &hi2c3, IST8310_IIC_ADDRESS << 1,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        1000);
}
/**
  * @brief          通过I2C读取IST8310的多个字节
  * @param[in]      寄存器开始地址
  * @param[out]     存取缓冲区
  * @param[in]      读取字节数
  * @retval         none
  */
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    while (len)
    {
        (*buf) = ist8310_IIC_read_single_reg(reg);
        reg++;
        buf++;
        len--;
    }
}
/**
  * @brief          通过I2C写入多个字节到IST8310的寄存器中
  * @param[in]      寄存器开始地址
  * @param[in]      存取缓冲区
  * @param[in]      写入字节数
  * @retval         none
  */
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    static const uint16_t IIC_time = 2000;
    while (len)
    {
        ist8310_IIC_write_single_reg(reg, (*data));
        reg++;
        data++;
        len--;
        Delay_us(IIC_time);
    }
}
