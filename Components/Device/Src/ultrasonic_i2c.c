#include "ultrasonic_i2c.h"
#include "bsp_i2c.h"

static uint8_t ultrasonic_elapsed(uint32_t now_ms, uint32_t last_ms, uint32_t interval_ms)
{
    return (uint32_t)(now_ms - last_ms) >= interval_ms;
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
    sensor->measure_pending = 0U;
    sensor->request_tick = 0U;
    sensor->update_tick = 0U;
    sensor->last_status = HAL_ERROR;

    if (hi2c != NULL)
    {
        sensor->last_status = BSP_I2C_IsDeviceReady(hi2c, addr_7bit, ULTRASONIC_I2C_TIMEOUT_MS);
        sensor->online = (sensor->last_status == HAL_OK) ? 1U : 0U;
    }
}

HAL_StatusTypeDef UltrasonicI2C_StartMeasure(UltrasonicI2C_t *sensor)
{
    uint8_t cmd = ULTRASONIC_MEASURE_CMD;

    if (sensor == NULL || sensor->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    sensor->last_status = BSP_I2C_MasterTransmit(sensor->hi2c,
                                                 sensor->addr_7bit,
                                                 &cmd,
                                                 1U,
                                                 ULTRASONIC_I2C_TIMEOUT_MS);
    sensor->online = (sensor->last_status == HAL_OK) ? 1U : 0U;
    sensor->measure_pending = (sensor->last_status == HAL_OK) ? 1U : 0U;
    sensor->request_tick = HAL_GetTick();

    return sensor->last_status;
}

HAL_StatusTypeDef UltrasonicI2C_ReadDistance(UltrasonicI2C_t *sensor)
{
    uint8_t data[3] = {0U};
    uint32_t raw_distance = 0U;

    if (sensor == NULL || sensor->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    sensor->last_status = BSP_I2C_MemRead(sensor->hi2c,
                                          sensor->addr_7bit,
                                          ULTRASONIC_DISTANCE_REG,
                                          data,
                                          sizeof(data),
                                          ULTRASONIC_I2C_TIMEOUT_MS);
    sensor->online = (sensor->last_status == HAL_OK) ? 1U : 0U;
    sensor->measure_pending = 0U;

    if (sensor->last_status != HAL_OK)
    {
        return sensor->last_status;
    }

    raw_distance = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[2];
    sensor->distance_mm = (uint16_t)(raw_distance / 1000U);
    if (sensor->distance_mm > ULTRASONIC_MAX_DISTANCE_MM)
    {
        sensor->distance_mm = ULTRASONIC_MAX_DISTANCE_MM;
    }
    sensor->update_tick = HAL_GetTick();

    return HAL_OK;
}

void UltrasonicI2C_Update(UltrasonicI2C_t *sensor, uint32_t now_ms)
{
    if (sensor == NULL || sensor->hi2c == NULL)
    {
        return;
    }

    if (sensor->measure_pending != 0U)
    {
        if (ultrasonic_elapsed(now_ms, sensor->request_tick, ULTRASONIC_READ_DELAY_MS))
        {
            (void)UltrasonicI2C_ReadDistance(sensor);
        }
        return;
    }

    if (sensor->request_tick == 0U || ultrasonic_elapsed(now_ms, sensor->request_tick, ULTRASONIC_MEASURE_INTERVAL_MS))
    {
        (void)UltrasonicI2C_StartMeasure(sensor);
    }
}

uint8_t UltrasonicI2C_Scan(I2C_HandleTypeDef *hi2c, uint8_t *addr_buf, uint8_t max_addr_num)
{
    return BSP_I2C_ScanBus(hi2c, addr_buf, max_addr_num, ULTRASONIC_SCAN_TIMEOUT_MS);
}
