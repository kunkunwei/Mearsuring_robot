//
// Created by kun on 25-7-17.
//

#include "stm32f4xx_hal.h"
#include "../Inc/bsp_i2c.h"

#include "../Inc/bsp_tick.h"
#include "i2c.h"

HAL_StatusTypeDef BSP_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint32_t timeout_ms)
{
    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr_7bit << 1), 1, timeout_ms);
}

HAL_StatusTypeDef BSP_I2C_MasterTransmit(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (hi2c == NULL || data == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Master_Transmit(hi2c, (uint16_t)(addr_7bit << 1), (uint8_t *)data, len, timeout_ms);
}

HAL_StatusTypeDef BSP_I2C_MasterReceive(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (hi2c == NULL || data == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Master_Receive(hi2c, (uint16_t)(addr_7bit << 1), data, len, timeout_ms);
}

HAL_StatusTypeDef BSP_I2C_MemRead(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint8_t reg, uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (hi2c == NULL || data == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr_7bit << 1), reg, I2C_MEMADD_SIZE_8BIT, data, len, timeout_ms);
}

HAL_StatusTypeDef BSP_I2C_MemWrite(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint8_t reg, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (hi2c == NULL || data == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(hi2c, (uint16_t)(addr_7bit << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, timeout_ms);
}

uint8_t BSP_I2C_ScanBus(I2C_HandleTypeDef *hi2c, uint8_t *addr_buf, uint8_t max_addr_num, uint32_t timeout_ms)
{
    uint8_t found_num = 0U;

    if (hi2c == NULL || addr_buf == NULL || max_addr_num == 0U)
    {
        return 0U;
    }

    for (uint8_t addr = 0x03U; addr <= 0x7FU && found_num < max_addr_num; addr++)
    {
        if (BSP_I2C_IsDeviceReady(hi2c, addr, timeout_ms) == HAL_OK)
        {
            addr_buf[found_num++] = addr;
        }
    }

    return found_num;
}

void BSP_I2C_RecoverBus(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hi2c == NULL || hi2c->Instance != I2C2)
    {
        return;
    }

    HAL_I2C_DeInit(hi2c);
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
    Delay_us(10U);

    for (uint8_t i = 0U; i < 9U; i++)
    {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
        Delay_us(10U);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
        Delay_us(10U);
    }

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
    Delay_us(10U);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
    Delay_us(10U);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
    Delay_us(10U);

    MX_I2C2_Init();
}

/**
  * @brief          通过I2C读取ist8310的一个字节
  * @param[in]      寄存器地址
  * @retval         寄存器值
  */
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t reg_data = 0U;
    (void)BSP_I2C_MemRead(&hi2c3, IST8310_IIC_ADDRESS, reg, &reg_data, 1U, 2U);
    return reg_data;
}
/**
  * @brief          通过I2C写入一个字节到ist8310的寄存器中
  * @param[in]      寄存器地址
  * @retval         写入值
  */
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    (void)BSP_I2C_MemWrite(&hi2c3, IST8310_IIC_ADDRESS, reg, &data, 1U, 2U);
}

HAL_StatusTypeDef ist8310_IIC_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    return BSP_I2C_MemRead(&hi2c3, IST8310_IIC_ADDRESS, reg, buf, len, 2U);
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
    (void)ist8310_IIC_read_regs(reg, buf, len);
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
