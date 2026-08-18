//
// Created by kun on 25-7-9.
//

#ifndef VOFA_H
#define VOFA_H

#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "ist8310.h"
#include "sbus_remote.h"
#include "Chassis_Task.h"

#define VOFA_CHANNELS 12
#define VOFA_TAIL {0x00, 0x00, 0x80, 0x7F}
#define VOFA_RX_CMD_MAX_LEN 64U

typedef struct
{
    float data[VOFA_CHANNELS];
    uint8_t tail[4];
} Vofa_Frame_t;

void uart_printf(UART_HandleTypeDef *huart, const char *fmt, ...);
HAL_StatusTypeDef Vofa_Send_chassis_Info(UART_HandleTypeDef *huart, const chassis_move_t *chassis);
HAL_StatusTypeDef Vofa_Send_Observe_Info(UART_HandleTypeDef *huart, const chassis_move_t *chassis);
HAL_StatusTypeDef Vofa_Send_Odom_Info(UART_HandleTypeDef *huart);
HAL_StatusTypeDef Vofa_Send_Motor_Info(UART_HandleTypeDef *huart, const chassis_move_t *chassis);
HAL_StatusTypeDef Vofa_Send_Speed_Control_Info(UART_HandleTypeDef *huart, const chassis_move_t *chassis);
bool Vofa_TryStorePidCommand(const uint8_t *data, uint16_t len);
void Vofa_Process_RxCommand(void);

extern Vofa_Frame_t vofa_tx;

#endif // VOFA_H
