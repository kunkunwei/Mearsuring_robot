//
// Created by kun on 25-7-17.
//

#ifndef BSP_I2C_H
#define BSP_I2C_H

#define IST8310_IIC_ADDRESS 0x0E  //IST8310的IIC地址


extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern void ist8310_RST_H(void); //复位IO 置高
extern void ist8310_RST_L(void); //复位IO 置地 置地会引起ist8310重启

#endif //BSP_I2C_H
