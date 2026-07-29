//
// Created by kun on 25-7-17.
//

#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define IST8310_IIC_ADDRESS 0x0E  //IST8310的IIC地址


HAL_StatusTypeDef BSP_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint32_t timeout_ms);
HAL_StatusTypeDef BSP_I2C_MasterTransmit(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, const uint8_t *data, uint16_t len, uint32_t timeout_ms);
HAL_StatusTypeDef BSP_I2C_MasterReceive(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint8_t *data, uint16_t len, uint32_t timeout_ms);
HAL_StatusTypeDef BSP_I2C_MemRead(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint8_t reg, uint8_t *data, uint16_t len, uint32_t timeout_ms);
HAL_StatusTypeDef BSP_I2C_MemWrite(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint8_t reg, const uint8_t *data, uint16_t len, uint32_t timeout_ms);
uint8_t BSP_I2C_ScanBus(I2C_HandleTypeDef *hi2c, uint8_t *addr_buf, uint8_t max_addr_num, uint32_t timeout_ms);
void BSP_I2C_RecoverBus(I2C_HandleTypeDef *hi2c);

extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern void ist8310_RST_H(void); //复位IO 置高
extern void ist8310_RST_L(void); //复位IO 置地 置地会引起ist8310重启

#endif //BSP_I2C_H
