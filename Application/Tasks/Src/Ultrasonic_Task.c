//
// Created by kun on 2026/7/29.
//

#include "main.h"
#include "Ultrasonic_Task.h"
#include "cmsis_os.h"
#include "i2c.h"

#define ULTRASONIC_CONVERT_TIME_MS 80U
#define ULTRASONIC_IDLE_TIME_MS 20U
#define ULTRASONIC_RIGHT_ADDR_CONFIG_ENABLE 0U

static UltrasonicI2C_t ultrasonic_left;
static UltrasonicI2C_t ultrasonic_right;

static HAL_StatusTypeDef Ultrasonic_Task_SetRightAddress(void)
{
    HAL_StatusTypeDef status;

    UltrasonicI2C_Init(&ultrasonic_right, &hi2c2, SR09_DEFAULT_ADDR);
    osDelay(100U);

    status = UltrasonicI2C_WriteCommand(&ultrasonic_right, SR09_ADDR_UNLOCK_1_CMD);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(10U);

    status = UltrasonicI2C_WriteCommand(&ultrasonic_right, SR09_ADDR_UNLOCK_2_CMD);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(10U);

    status = UltrasonicI2C_WriteCommand(&ultrasonic_right, SR09_ADDR_UNLOCK_3_CMD);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(10U);

    status = UltrasonicI2C_WriteCommand(&ultrasonic_right, (uint8_t)(ULTRASONIC_RIGHT_ADDR << 1));
    ultrasonic_right.write_status = status;
    ultrasonic_right.last_status = status;
    ultrasonic_right.online = (status == HAL_OK) ? 1U : 0U;
    osDelay(100U);

    return status;
}

const UltrasonicI2C_t *Ultrasonic_Task_GetLeft(void)
{
    return &ultrasonic_left;
}

const UltrasonicI2C_t *Ultrasonic_Task_GetRight(void)
{
    return &ultrasonic_right;
}

void Ultrasonic_Task(void const *argument)
{
    (void)argument;

#if ULTRASONIC_RIGHT_ADDR_CONFIG_ENABLE
    UltrasonicI2C_Init(&ultrasonic_left, NULL, ULTRASONIC_LEFT_ADDR);
    (void)Ultrasonic_Task_SetRightAddress();
    UltrasonicI2C_Init(&ultrasonic_right, &hi2c2, ULTRASONIC_RIGHT_ADDR);

    for (;;)
    {
        (void)UltrasonicI2C_StartMeasure(&ultrasonic_right);
        osDelay(ULTRASONIC_CONVERT_TIME_MS);
        (void)UltrasonicI2C_ReadDistance(&ultrasonic_right);
        osDelay(ULTRASONIC_IDLE_TIME_MS);
    }
#else
    UltrasonicI2C_Init(&ultrasonic_left, &hi2c2, ULTRASONIC_LEFT_ADDR);
    UltrasonicI2C_Init(&ultrasonic_right, &hi2c2, ULTRASONIC_RIGHT_ADDR);

    for (;;)
    {
        (void)UltrasonicI2C_StartMeasure(&ultrasonic_left);
        osDelay(ULTRASONIC_CONVERT_TIME_MS);
        (void)UltrasonicI2C_ReadDistance(&ultrasonic_left);
        osDelay(ULTRASONIC_IDLE_TIME_MS);

        (void)UltrasonicI2C_StartMeasure(&ultrasonic_right);
        osDelay(ULTRASONIC_CONVERT_TIME_MS);
        (void)UltrasonicI2C_ReadDistance(&ultrasonic_right);
        osDelay(ULTRASONIC_IDLE_TIME_MS);
    }
#endif
}
