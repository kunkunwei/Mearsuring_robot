#ifndef DEVICE_MINIPC_H
#define DEVICE_MINIPC_H

#include "stdbool.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"

#define MINIPC_FRAME_HEADER                    0x42U
#define MINIPC_ADDR_CHASSIS_CMD                0x31U
#define MINIPC_ADDR_CHASSIS_ODOM               0x32U
#define MINIPC_CHASSIS_CMD_FRAME_LENGTH        12U
#define MINIPC_CHASSIS_ODOM_FRAME_LENGTH       36U
#define MINIPC_CHASSIS_CMD_TIMEOUT_MS          500U

#pragma pack(push, 1)
typedef struct
{
    uint8_t header;
    uint8_t address;
    uint8_t length;
    float vx;
    float wz;
    uint8_t checksum;
} MiniPC_ChassisCmdFrame_Typedef;

typedef struct
{
    uint8_t header;
    uint8_t address;
    uint8_t length;
    float x;
    float y;
    float yaw;
    float distance;
    float vx;
    float wz;
    uint16_t motor_ecd[4];
    uint8_t checksum;
} MiniPC_ChassisOdomFrame_Typedef;
#pragma pack(pop)

typedef union
{
    float value;
    uint8_t bytes[4];
} MiniPC_FloatUnion_t;

typedef struct
{
    float vx;
    float wz;
    uint32_t update_tick;
    uint8_t online;
} MiniPC_ChassisCmd_Typedef;

typedef struct
{
    float x;
    float y;
    float yaw;
    float distance;
    float vx;
    float wz;
    uint16_t motor_ecd[4];
} MiniPC_ChassisOdom_Typedef;

bool MiniPC_DecodeChassisCmdFrame(const uint8_t *buf, uint32_t len, MiniPC_ChassisCmdFrame_Typedef *frame);
bool MiniPC_UpdateChassisCmdFromBuffer(const uint8_t *buf, uint32_t len);
const MiniPC_ChassisCmd_Typedef *MiniPC_GetChassisCmdPoint(void);
bool MiniPC_IsChassisCmdOnline(uint32_t timeout_ms);
bool MiniPC_SendChassisOdomUSB(const MiniPC_ChassisOdom_Typedef *odom);
bool MiniPC_SendChassisOdomUART(UART_HandleTypeDef *huart, const MiniPC_ChassisOdom_Typedef *odom);

#endif
