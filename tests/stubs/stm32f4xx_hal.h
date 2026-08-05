#ifndef TEST_STM32F4XX_HAL_H
#define TEST_STM32F4XX_HAL_H

#include <stdint.h>

typedef struct
{
    uint32_t unused;
} UART_HandleTypeDef;

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR = 1,
} HAL_StatusTypeDef;

uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t *data,
                                    uint16_t length,
                                    uint32_t timeout);

#endif
