//
// Created by kun on 25-7-17.
//
/**
****************************(C) COPYRIGHT 2019 DJI****************************
* @file       IST8310driver.c/h
* @brief      ist8310 is a 3-axis digital magnetometer, the file includes initialization function,
*             read magnetic field strength data function.
*             IST8310是一款三轴数字磁力计，本文件包括初始化函数，读取磁场数据函数。
* @note       IST8310 only support I2C. IST8310只支持I2C。
* @history
*  Version    Date            Author          Modification
*  V1.0.0     Dec-26-2018     RM              1. done
*
@verbatim
==============================================================================

==============================================================================
@endverbatim
****************************(C) COPYRIGHT 2019 DJI****************************
*/
#include "stm32f4xx_hal.h"
#include "ist8310.h"
#include "bsp_i2c.h"
#include "bsp_tick.h"
#define MAG_SEN 0.3f //raw int16 data change to uT unit. 原始整型数据变成 单位ut

#define IST8310_WHO_AM_I 0x00       //ist8310 "who am I "
#define IST8310_WHO_AM_I_VALUE 0x10 //device ID

#define IST8310_WRITE_REG_NUM 4

ist8310_real_data_t ist8310_Info={
    .status=0,
    .raw_mag={0.0f,0.0f,0.0f},
    .calibrated_mag={0.0f,0.0f,0.0f},
    .mag_max={mag_max_x,mag_max_y,mag_max_z},
    .mag_min={mag_min_x,mag_min_y,mag_min_z},
    .mag_bias={(mag_max_x+mag_min_x)/2.0f,(mag_max_y+mag_min_y)/2.0f,(mag_max_z+mag_min_z)/2.0f},
    .mag_scale={2.0f/(mag_max_x-mag_min_x),2.0f/(mag_max_y-mag_min_y),2.0f/(mag_max_z-mag_min_z)}
};
//第一列:IST8310的寄存器
//第二列:需要写入的寄存器值
//第三列:返回的错误码
static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] ={
        {0x0B, 0x08, 0x01},     //开启中断，并且设置低电平
        {0x41, 0x09, 0x02},     //平均采样四次
        {0x42, 0xC0, 0x03},     //必须是0xC0
        {0x0A, 0x0B, 0x04}};    //200Hz输出频率


/**
  * @brief          初始化IST8310
  * @param[in]      none
  * @retval         error value
  */
uint8_t ist8310_init(void)
{
    static const uint8_t wait_time = 150;
    static const uint8_t sleepTime = 50;
    uint8_t res = 0;
    uint8_t writeNum = 0;

    ist8310_RST_L();
    Delay_ms(sleepTime);
    ist8310_RST_H();
    Delay_ms(sleepTime);

    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }

    //set ist8310 sonsor config and check
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
        Delay_us(wait_time);
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
        Delay_us(wait_time);
        if (res != ist8310_write_reg_data_error[writeNum][1])
        {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }
    return IST8310_NO_ERROR;
}

/**
  * @brief          如果已经通过I2C的DMA方式读取到了从STAT1到DATAZL的数据，可以使用这个函数进行处理
  * @param[in]      status_buf:数据指针,从STAT1(0x02) 寄存器到 DATAZL(0x08)寄存器
  * @param[out]     ist8310_real_data:ist8310的数据结构
  * @retval         none
  */
void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data)
{

    if (status_buf[0] & 0x01)
    {
        int16_t temp_ist8310_data = 0;
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;

        temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        ist8310_real_data->raw_mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        ist8310_real_data->raw_mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        ist8310_real_data->raw_mag[2] = MAG_SEN * temp_ist8310_data;
    }
    else
    {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

/**
  * @brief          通过I2C读取磁场数据
  * @param[out]     磁场数组
  * @retval         none
  */
void ist8310_read_mag(fp32 mag[3])
{
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
    //read the "DATAXL" register (0x03)
    ist8310_IIC_read_muli_reg(0x03, buf, 6);

    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2] = MAG_SEN * temp_ist8310_data;
}

void mag_calibration(ist8310_real_data_t *ist8310_Info)
{
    ist8310_Info->calibrated_mag[0] = (ist8310_Info->raw_mag[0] - ist8310_Info->mag_bias[0]) * ist8310_Info->mag_scale[0];
    ist8310_Info->calibrated_mag[1] = (ist8310_Info->raw_mag[1] - ist8310_Info->mag_bias[1]) * ist8310_Info->mag_scale[1];
    ist8310_Info->calibrated_mag[2] = (ist8310_Info->raw_mag[2] - ist8310_Info->mag_bias[2]) * ist8310_Info->mag_scale[2];
}
// 机器人缓慢旋转一圈，记录最大最小值
void simple_mag_calibration_messure(ist8310_real_data_t *ist8310_Info)
{
    /*  到新的场地就放到循环任务里面调用，记录磁场数据最值*/
    // 记录各轴的最大最小值
    // bias = (max + min) / 2
    // scale = (max - min) / 2
    static float mag_max[3] = {mag_max_x,mag_max_y,mag_max_z};
    static float mag_min[3] = {mag_min_x,mag_min_y,mag_min_z};
    for (int i = 0; i < 3; i++)
    {
        if (ist8310_Info->raw_mag[i] > mag_max[i])
        {
            mag_max[i] = ist8310_Info->raw_mag[i];
            ist8310_Info->mag_max[i]=mag_max[i];
        }
        if (ist8310_Info->raw_mag[i] < mag_min[i])
        {
            mag_min[i] = ist8310_Info->raw_mag[i];
            ist8310_Info->mag_min[i]=mag_min[i];
        }
    }
    for (int i = 0; i < 3; i++)
    {
        ist8310_Info->mag_bias[i] = (ist8310_Info->mag_max[i] + ist8310_Info->mag_min[i]) / 2.0f;
        float scale = (ist8310_Info->mag_max[i] - ist8310_Info->mag_min[i]) / 2.0f;
        ist8310_Info->mag_scale[i] = 1.0f / scale;
    }
}
void IST8310_Info_Update(ist8310_real_data_t *ist8310_Info)
{
    ist8310_read_mag(ist8310_Info->raw_mag);
    mag_calibration(ist8310_Info);
}
