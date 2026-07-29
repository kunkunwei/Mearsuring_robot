#include "ultrasonic_i2c.h"
#include "bsp_i2c.h"
#include "bsp_tick.h"

static void UltrasonicI2C_RecoverIfBusy(UltrasonicI2C_t *sensor)
{
    if (sensor == NULL || sensor->hi2c == NULL)
    {
        return;
    }

    if (HAL_I2C_GetState(sensor->hi2c) != HAL_I2C_STATE_READY)
    {
        BSP_I2C_RecoverBus(sensor->hi2c);
    }
}

void UltrasonicI2C_Init(UltrasonicI2C_t *sensor, I2C_HandleTypeDef *hi2c, uint8_t addr_7bit)
{
    if (sensor == NULL)
    {
        return;
    }

    sensor->hi2c = hi2c;
    sensor->addr_7bit = addr_7bit;
    sensor->distance_mm = 0U;
    sensor->online = 0U;
    sensor->raw_high = 0U;
    sensor->raw_low = 0U;
    sensor->last_status = HAL_ERROR;
    sensor->write_status = HAL_ERROR;
    sensor->read_status = HAL_ERROR;

    if (hi2c != NULL)
    {
        sensor->last_status = BSP_I2C_IsDeviceReady(hi2c, addr_7bit, ULTRASONIC_I2C_TIMEOUT_MS);
        sensor->online = (sensor->last_status == HAL_OK) ? 1U : 0U;
    }
}

HAL_StatusTypeDef UltrasonicI2C_WriteCommand(UltrasonicI2C_t *sensor, uint8_t cmd)
{
    uint8_t data[2] = {SR09_COMMAND_REG, cmd};

    if (sensor == NULL || sensor->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    UltrasonicI2C_RecoverIfBusy(sensor);

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(sensor->hi2c,
                                                       (uint16_t)(sensor->addr_7bit << 1),
                                                       data,
                                                       sizeof(data),
                                                       ULTRASONIC_I2C_TIMEOUT_MS);
    if (status == HAL_BUSY)
    {
        BSP_I2C_RecoverBus(sensor->hi2c);
        status = HAL_I2C_Master_Transmit(sensor->hi2c,
                                         (uint16_t)(sensor->addr_7bit << 1),
                                         data,
                                         sizeof(data),
                                         ULTRASONIC_I2C_TIMEOUT_MS);
    }

    return status;
}

HAL_StatusTypeDef UltrasonicI2C_StartMeasure(UltrasonicI2C_t *sensor)
{
    if (sensor == NULL || sensor->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    sensor->write_status = UltrasonicI2C_WriteCommand(sensor, SR09_MEASURE_US_CMD);
    sensor->last_status = sensor->write_status;
    sensor->online = (sensor->write_status == HAL_OK) ? 1U : 0U;

    return sensor->write_status;
}

HAL_StatusTypeDef UltrasonicI2C_ReadDistance(UltrasonicI2C_t *sensor)
{
    uint8_t data[2] = {0U};

    if (sensor == NULL || sensor->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    UltrasonicI2C_RecoverIfBusy(sensor);

    sensor->read_status = HAL_I2C_Master_Receive(sensor->hi2c,
                                                 (uint16_t)(sensor->addr_7bit << 1),
                                                 data,
                                                 sizeof(data),
                                                 ULTRASONIC_I2C_TIMEOUT_MS);
    if (sensor->read_status == HAL_BUSY)
    {
        BSP_I2C_RecoverBus(sensor->hi2c);
        sensor->read_status = HAL_I2C_Master_Receive(sensor->hi2c,
                                                     (uint16_t)(sensor->addr_7bit << 1),
                                                     data,
                                                     sizeof(data),
                                                     ULTRASONIC_I2C_TIMEOUT_MS);
    }
    sensor->last_status = sensor->read_status;
    sensor->online = (sensor->read_status == HAL_OK) ? 1U : 0U;

    if (sensor->read_status != HAL_OK)
    {
        return sensor->read_status;
    }

    sensor->raw_high = data[0];
    sensor->raw_low = data[1];

    uint16_t echo_us = (uint16_t)(((uint16_t)sensor->raw_high << 8) | (uint16_t)sensor->raw_low);
    uint32_t distance_mm = ((uint32_t)echo_us * 17U) / 100U;
    if (distance_mm > ULTRASONIC_MAX_DISTANCE_MM)
    {
        distance_mm = ULTRASONIC_MAX_DISTANCE_MM;
    }

    sensor->distance_mm = (uint16_t)distance_mm;

    return HAL_OK;
}

HAL_StatusTypeDef UltrasonicI2C_ChangeAddress(I2C_HandleTypeDef *hi2c, uint8_t old_addr_7bit, uint8_t new_addr_7bit)
{
    UltrasonicI2C_t sensor;

    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    UltrasonicI2C_Init(&sensor, hi2c, old_addr_7bit);
    if (sensor.online == 0U)
    {
        return HAL_ERROR;
    }

    if (UltrasonicI2C_WriteCommand(&sensor, SR09_ADDR_UNLOCK_1_CMD) != HAL_OK)
    {
        return HAL_ERROR;
    }
    Delay_us(1000U);

    if (UltrasonicI2C_WriteCommand(&sensor, SR09_ADDR_UNLOCK_2_CMD) != HAL_OK)
    {
        return HAL_ERROR;
    }
    Delay_us(1000U);

    if (UltrasonicI2C_WriteCommand(&sensor, SR09_ADDR_UNLOCK_3_CMD) != HAL_OK)
    {
        return HAL_ERROR;
    }
    Delay_us(1000U);

    if (UltrasonicI2C_WriteCommand(&sensor, (uint8_t)(new_addr_7bit << 1)) != HAL_OK)
    {
        return HAL_ERROR;
    }
    Delay_us(100000U);

    return HAL_OK;
}

uint8_t UltrasonicI2C_Scan(I2C_HandleTypeDef *hi2c, uint8_t *addr_buf, uint8_t max_addr_num)
{
    return BSP_I2C_ScanBus(hi2c, addr_buf, max_addr_num, ULTRASONIC_SCAN_TIMEOUT_MS);
}
