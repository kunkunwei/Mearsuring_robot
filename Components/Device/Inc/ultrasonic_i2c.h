#ifndef ULTRASONIC_I2C_H
#define ULTRASONIC_I2C_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define SR09_DEFAULT_ADDR                0x74U  /* manual 8-bit address: 0xE8 */
#define ULTRASONIC_LEFT_ADDR             SR09_DEFAULT_ADDR
#define ULTRASONIC_RIGHT_ADDR            0x69U  /* manual 8-bit address: 0xD2 */

#define SR09_COMMAND_REG                 0x02U
#define SR09_MEASURE_MM_CMD              0xB4U
#define SR09_MEASURE_US_CMD              0xB2U
#define SR09_NOISE_BATTERY_CMD           0x70U
#define SR09_NOISE_USB_CMD               0x71U
#define SR09_NOISE_SWITCH_SUPPLY_CMD     0x73U
#define SR09_NOISE_HIGH_NOISE_CMD        0x74U
#define SR09_LED_ENABLE_CMD              0xC0U
#define SR09_LED_DISABLE_CMD             0xC1U
#define SR09_ADDR_UNLOCK_1_CMD           0x9AU
#define SR09_ADDR_UNLOCK_2_CMD           0x92U
#define SR09_ADDR_UNLOCK_3_CMD           0x9EU

#define ULTRASONIC_MEASURE_INTERVAL_MS   100U
#define ULTRASONIC_I2C_TIMEOUT_MS        50U
#define ULTRASONIC_SCAN_TIMEOUT_MS       1U
#define ULTRASONIC_MAX_DISTANCE_MM       5000U

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t addr_7bit;
    uint16_t distance_mm;
    uint8_t online;
    uint8_t raw_high;
    uint8_t raw_low;
    HAL_StatusTypeDef last_status;
    HAL_StatusTypeDef write_status;
    HAL_StatusTypeDef read_status;
} UltrasonicI2C_t;

void UltrasonicI2C_Init(UltrasonicI2C_t *sensor, I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);
HAL_StatusTypeDef UltrasonicI2C_StartMeasure(UltrasonicI2C_t *sensor);
HAL_StatusTypeDef UltrasonicI2C_ReadDistance(UltrasonicI2C_t *sensor);
HAL_StatusTypeDef UltrasonicI2C_WriteCommand(UltrasonicI2C_t *sensor, uint8_t cmd);
HAL_StatusTypeDef UltrasonicI2C_ChangeAddress(I2C_HandleTypeDef *hi2c, uint8_t old_addr_7bit, uint8_t new_addr_7bit);
uint8_t UltrasonicI2C_Scan(I2C_HandleTypeDef *hi2c, uint8_t *addr_buf, uint8_t max_addr_num);

#endif
