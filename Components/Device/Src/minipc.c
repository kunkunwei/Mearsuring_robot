#include "minipc.h"
#include "usbd_cdc_if.h"
#include <string.h>

static MiniPC_ChassisCmd_Typedef chassis_cmd;

static uint8_t MiniPC_FrameChecksum(const uint8_t *buf, uint8_t len)
{
    uint8_t checksum = 0U;

    if (buf == NULL || len < 2U)
    {
        return 0U;
    }

    for (uint8_t i = 0U; i < (uint8_t)(len - 1U); i++)
    {
        checksum += buf[i];
    }

    return checksum;
}

static void MiniPC_PackFloat(uint8_t *dst, float value)
{
    MiniPC_FloatUnion_t data = {.value = value};
    memcpy(dst, data.bytes, sizeof(data.bytes));
}

static void MiniPC_PackU16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static float MiniPC_UnpackFloat(const uint8_t *src)
{
    MiniPC_FloatUnion_t data = {0};
    memcpy(data.bytes, src, sizeof(data.bytes));
    return data.value;
}

bool MiniPC_DecodeChassisCmdFrame(const uint8_t *buf, uint32_t len, MiniPC_ChassisCmdFrame_Typedef *frame)
{
    if (buf == NULL || frame == NULL || len != MINIPC_CHASSIS_CMD_FRAME_LENGTH)
    {
        return false;
    }

    if (buf[0] != MINIPC_FRAME_HEADER ||
        buf[1] != MINIPC_ADDR_CHASSIS_CMD ||
        buf[2] != MINIPC_CHASSIS_CMD_FRAME_LENGTH)
    {
        return false;
    }

    if (MiniPC_FrameChecksum(buf, MINIPC_CHASSIS_CMD_FRAME_LENGTH) != buf[MINIPC_CHASSIS_CMD_FRAME_LENGTH - 1U])
    {
        return false;
    }

    frame->header = buf[0];
    frame->address = buf[1];
    frame->length = buf[2];
    frame->vx = MiniPC_UnpackFloat(&buf[3]);
    frame->wz = MiniPC_UnpackFloat(&buf[7]);
    frame->checksum = buf[11];

    return true;
}

bool MiniPC_UpdateChassisCmdFromBuffer(const uint8_t *buf, uint32_t len)
{
    MiniPC_ChassisCmdFrame_Typedef frame;

    if (!MiniPC_DecodeChassisCmdFrame(buf, len, &frame))
    {
        return false;
    }

    chassis_cmd.vx = frame.vx;
    chassis_cmd.wz = frame.wz;
    chassis_cmd.update_tick = HAL_GetTick();
    chassis_cmd.online = 1U;

    return true;
}

bool MiniPC_UpdateChassisCmdFromStream(const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len < MINIPC_CHASSIS_CMD_FRAME_LENGTH)
    {
        return false;
    }

    bool updated = false;
    for (uint32_t i = 0U; i <= len - MINIPC_CHASSIS_CMD_FRAME_LENGTH; i++)
    {
        if (buf[i] != MINIPC_FRAME_HEADER)
        {
            continue;
        }

        if (MiniPC_UpdateChassisCmdFromBuffer(&buf[i], MINIPC_CHASSIS_CMD_FRAME_LENGTH))
        {
            updated = true;
            i += MINIPC_CHASSIS_CMD_FRAME_LENGTH - 1U;
        }
    }

    return updated;
}

const MiniPC_ChassisCmd_Typedef *MiniPC_GetChassisCmdPoint(void)
{
    return &chassis_cmd;
}

bool MiniPC_IsChassisCmdOnline(uint32_t timeout_ms)
{
    if (chassis_cmd.online == 0U)
    {
        return false;
    }

    return (HAL_GetTick() - chassis_cmd.update_tick) <= timeout_ms;
}

static void MiniPC_BuildChassisOdomFrame(uint8_t *tx_buf, const MiniPC_ChassisOdom_Typedef *odom)
{
    tx_buf[0] = MINIPC_FRAME_HEADER;
    tx_buf[1] = MINIPC_ADDR_CHASSIS_ODOM;
    tx_buf[2] = MINIPC_CHASSIS_ODOM_FRAME_LENGTH;

    MiniPC_PackFloat(&tx_buf[3], odom->x);
    MiniPC_PackFloat(&tx_buf[7], odom->y);
    MiniPC_PackFloat(&tx_buf[11], odom->yaw);
    MiniPC_PackFloat(&tx_buf[15], odom->distance);
    MiniPC_PackFloat(&tx_buf[19], odom->vx);
    MiniPC_PackFloat(&tx_buf[23], odom->wz);
    MiniPC_PackU16(&tx_buf[27], odom->motor_ecd[0]);
    MiniPC_PackU16(&tx_buf[29], odom->motor_ecd[1]);
    MiniPC_PackU16(&tx_buf[31], odom->motor_ecd[2]);
    MiniPC_PackU16(&tx_buf[33], odom->motor_ecd[3]);
    MiniPC_PackU16(&tx_buf[35], odom->left_mm);
    MiniPC_PackU16(&tx_buf[37], odom->right_mm);
    tx_buf[39] = odom->left_online;
    tx_buf[40] = odom->right_online;
    tx_buf[41] = MiniPC_FrameChecksum(tx_buf, MINIPC_CHASSIS_ODOM_FRAME_LENGTH);
}

bool MiniPC_SendChassisOdomUSB(const MiniPC_ChassisOdom_Typedef *odom)
{
    if (odom == NULL)
    {
        return false;
    }

    uint8_t tx_buf[MINIPC_CHASSIS_ODOM_FRAME_LENGTH] = {0};
    MiniPC_BuildChassisOdomFrame(tx_buf, odom);

    return CDC_Transmit_FS(tx_buf, MINIPC_CHASSIS_ODOM_FRAME_LENGTH) == USBD_OK;
}

bool MiniPC_SendChassisOdomUART(UART_HandleTypeDef *huart, const MiniPC_ChassisOdom_Typedef *odom)
{
    if (huart == NULL || odom == NULL)
    {
        return false;
    }

    uint8_t tx_buf[MINIPC_CHASSIS_ODOM_FRAME_LENGTH] = {0};
    MiniPC_BuildChassisOdomFrame(tx_buf, odom);

    return HAL_UART_Transmit(huart, tx_buf, MINIPC_CHASSIS_ODOM_FRAME_LENGTH, 10U) == HAL_OK;
}
