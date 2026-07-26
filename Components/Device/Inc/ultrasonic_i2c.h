#ifndef ULTRASONIC_I2C_H
#define ULTRASONIC_I2C_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define ULTRASONIC_LEFT_ADDR             0x57U
#define ULTRASONIC_RIGHT_ADDR            0x58U

#define ULTRASONIC_MEASURE_CMD           0x01U
#define ULTRASONIC_DISTANCE_REG          0xAFU
#define ULTRASONIC_MEASURE_INTERVAL_MS   200U
#define ULTRASONIC_READ_DELAY_MS         200U
#define ULTRASONIC_I2C_TIMEOUT_MS        5U
#define ULTRASONIC_SCAN_TIMEOUT_MS       1U
#define ULTRASONIC_MAX_DISTANCE_MM       6000U

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t addr_7bit;
    uint16_t distance_mm;
    uint8_t online;
    uint8_t measure_pending;
    uint32_t request_tick;
    uint32_t update_tick;
    HAL_StatusTypeDef last_status;
} UltrasonicI2C_t;

void UltrasonicI2C_Init(UltrasonicI2C_t *sensor, I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);
HAL_StatusTypeDef UltrasonicI2C_StartMeasure(UltrasonicI2C_t *sensor);
HAL_StatusTypeDef UltrasonicI2C_ReadDistance(UltrasonicI2C_t *sensor);
void UltrasonicI2C_Update(UltrasonicI2C_t *sensor, uint32_t now_ms);
uint8_t UltrasonicI2C_Scan(I2C_HandleTypeDef *hi2c, uint8_t *addr_buf, uint8_t max_addr_num);

#endif
