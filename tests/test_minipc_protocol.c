#include "minipc.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static uint8_t uart_tx[128];
static uint16_t uart_tx_length;

uint32_t HAL_GetTick(void)
{
    return 1234U;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t *data,
                                    uint16_t length,
                                    uint32_t timeout)
{
    (void)huart;
    (void)timeout;
    memcpy(uart_tx, data, length);
    uart_tx_length = length;
    return HAL_OK;
}

uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length)
{
    (void)data;
    (void)length;
    return 0U;
}

static uint8_t checksum(const uint8_t *data, uint8_t length)
{
    uint8_t value = 0U;
    for (uint8_t i = 0U; i < length; i++)
    {
        value += data[i];
    }
    return value;
}

static void build_reset_frame(uint8_t segment_id, uint8_t frame[MINIPC_ODOM_RESET_FRAME_LENGTH])
{
    frame[0] = MINIPC_FRAME_HEADER;
    frame[1] = MINIPC_ADDR_ODOM_RESET;
    frame[2] = MINIPC_ODOM_RESET_FRAME_LENGTH;
    frame[3] = MINIPC_ODOM_RESET_COMMAND;
    frame[4] = segment_id;
    frame[5] = checksum(frame, 5U);
}

static void test_reset_frame_is_latched_once(void)
{
    uint8_t frame[MINIPC_ODOM_RESET_FRAME_LENGTH];
    uint8_t segment_id = 0U;
    build_reset_frame(7U, frame);

    assert(MiniPC_UpdateChassisCmdFromStream(frame, sizeof(frame)));
    assert(MiniPC_TakeOdomResetRequest(&segment_id));
    assert(segment_id == 7U);
    assert(!MiniPC_TakeOdomResetRequest(&segment_id));

    assert(MiniPC_UpdateChassisCmdFromStream(frame, sizeof(frame)));
    assert(!MiniPC_TakeOdomResetRequest(&segment_id));

    build_reset_frame(8U, frame);
    assert(MiniPC_UpdateChassisCmdFromStream(frame, sizeof(frame)));
    assert(MiniPC_TakeOdomResetRequest(&segment_id));
    assert(segment_id == 8U);
}

static void test_reset_frame_can_follow_noise_and_velocity_frame(void)
{
    uint8_t stream[1U + MINIPC_CHASSIS_CMD_FRAME_LENGTH + MINIPC_ODOM_RESET_FRAME_LENGTH] = {0};
    const float vx = 0.25f;
    const float wz = -0.5f;
    uint8_t segment_id = 0U;

    stream[0] = 0xAAU;
    stream[1] = MINIPC_FRAME_HEADER;
    stream[2] = MINIPC_ADDR_CHASSIS_CMD;
    stream[3] = MINIPC_CHASSIS_CMD_FRAME_LENGTH;
    memcpy(&stream[4], &vx, sizeof(vx));
    memcpy(&stream[8], &wz, sizeof(wz));
    stream[12] = checksum(&stream[1], MINIPC_CHASSIS_CMD_FRAME_LENGTH - 1U);
    build_reset_frame(9U, &stream[13]);

    assert(MiniPC_UpdateChassisCmdFromStream(stream, sizeof(stream)));
    assert(fabsf(MiniPC_GetChassisCmdPoint()->vx - vx) < 0.0001f);
    assert(fabsf(MiniPC_GetChassisCmdPoint()->wz - wz) < 0.0001f);
    assert(MiniPC_TakeOdomResetRequest(&segment_id));
    assert(segment_id == 9U);
}

static void test_odom_frame_contains_applied_segment_id(void)
{
    UART_HandleTypeDef huart = {0};
    MiniPC_ChassisOdom_Typedef odom = {0};
    odom.x = 1.0f;
    odom.segment_id = 12U;

    assert(MINIPC_CHASSIS_ODOM_FRAME_LENGTH == 51U);
    assert(MiniPC_SendChassisOdomUART(&huart, &odom));
    assert(uart_tx_length == MINIPC_CHASSIS_ODOM_FRAME_LENGTH);
    assert(uart_tx[0] == MINIPC_FRAME_HEADER);
    assert(uart_tx[1] == MINIPC_ADDR_CHASSIS_ODOM);
    assert(uart_tx[2] == MINIPC_CHASSIS_ODOM_FRAME_LENGTH);
    assert(uart_tx[49] == 12U);
    assert(uart_tx[50] == checksum(uart_tx, 50U));
}

int main(void)
{
    test_reset_frame_is_latched_once();
    test_reset_frame_can_follow_noise_and_velocity_frame();
    test_odom_frame_contains_applied_segment_id();
    return 0;
}
