/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : sbus_remote.c
 * @brief          : SBUS remote control interface implementation
 * @author         : Generated for SBUS Protocol
 * @date           : 2025/12/15
 * @version        : v1.0
 ******************************************************************************
 * @attention      : SBUS Protocol for FlySky i6X transmitter with FS-IA6B receiver
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "sbus_remote.h"
#include <string.h>

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief SBUS remote control structure variable
 * @note  static: 只允许本 TU 内 SBUS_Frame_Parse 写入，外部通过
 *        SBUS_Process_Buffer() 触发更新，通过 get_sbus_remote_control_point() 只读访问
 */
static SBUS_Remote_Info_Typedef sbus_remote_ctrl = {
    .rc_lost    = true,
    .online_cnt = 0xFAU,
};

/**
 * @brief SBUS usart RxDMA MultiBuffer
 */
uint8_t SBUS_Remote_MultiRx_Buf[2][SBUS_RX_BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief  Verify SBUS frame integrity
 * @param  sbus_buf: pointer to array containing SBUS frame data
 * @retval true if frame is valid, false otherwise
 */
bool SBUS_Frame_Verify(volatile const uint8_t *sbus_buf)
{
    if (sbus_buf == NULL)
        return false;

    /* Check frame header and footer */
    if (sbus_buf[0] != SBUS_FRAME_HEADER)
        return false;

    if (sbus_buf[24] != SBUS_FRAME_FOOTER)
        return false;

    return true;
}

/**
 * @brief  Parse SBUS frame and update remote control data
 * @param  sbus_buf: pointer to array containing SBUS frame data
 * @param  sbus_remote: pointer to SBUS_Remote_Info_Typedef structure
 * @retval none
 */
void SBUS_Frame_Parse(volatile const uint8_t *sbus_buf, SBUS_Remote_Info_Typedef *sbus_remote)
{
    if (sbus_buf == NULL || sbus_remote == NULL)
        return;

    /* Verify frame integrity first */
    if (!SBUS_Frame_Verify(sbus_buf))
        return;

    /* Parse 16 channels (11 bits each) from bytes 1-22
     * SBUS protocol packs channels in little-endian bit order
     */
    int16_t ch_raw[16];
    ch_raw[0]  = ((sbus_buf[1] | sbus_buf[2] << 8) & 0x07FF);
    ch_raw[1]  = ((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF);
    ch_raw[2]  = ((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF);
    ch_raw[3]  = ((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF);
    ch_raw[4]  = ((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF);
    ch_raw[5]  = ((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF);
    ch_raw[6]  = ((sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF);
    ch_raw[7]  = ((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF);
    ch_raw[8]  = ((sbus_buf[12] | sbus_buf[13] << 8) & 0x07FF);
    ch_raw[9]  = ((sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF);
    ch_raw[10] = ((sbus_buf[14] >> 6 | sbus_buf[15] << 2 | sbus_buf[16] << 10) & 0x07FF);
    ch_raw[11] = ((sbus_buf[16] >> 1 | sbus_buf[17] << 7) & 0x07FF);
    ch_raw[12] = ((sbus_buf[17] >> 4 | sbus_buf[18] << 4) & 0x07FF);
    ch_raw[13] = ((sbus_buf[18] >> 7 | sbus_buf[19] << 1 | sbus_buf[20] << 9) & 0x07FF);
    ch_raw[14] = ((sbus_buf[20] >> 2 | sbus_buf[21] << 6) & 0x07FF);
    ch_raw[15] = ((sbus_buf[21] >> 5 | sbus_buf[22] << 3) & 0x07FF);

    /* Center channels at 0 */
    for (int i = 0; i < 16; i++) {
        ch_raw[i] -= SBUS_CH_VALUE_OFFSET;
    }

    /* Map to your 10-channel layout */
    /* Analog: ch[0..5] */
    for (int i = 0; i < 6; i++) {
        sbus_remote->rc.ch[i] = ch_raw[i];
    }

    /* Switches mapped to {-1,0,+1}
     * s[0] <- CH6 (index 6) two-position
     * s[1] <- CH7 (index 7) two-position
     * s[2] <- CH8 (index 8) three-position
     * s[3] <- CH9 (index 9) two-position
     */
    const int16_t THRESH = 400; /* dead-zone threshold */
    /* CH6 */
    sbus_remote->rc.s[0] = (ch_raw[6] > THRESH) ? +1 : (ch_raw[6] < -THRESH ? -1 : 0);
    /* CH7 */
    sbus_remote->rc.s[1] = (ch_raw[7] > THRESH) ? +1 : (ch_raw[7] < -THRESH ? -1 : 0);
    /* CH8 (three-position: -1/0/+1) */
    if (ch_raw[8] > THRESH) sbus_remote->rc.s[2] = +1;
    else if (ch_raw[8] < -THRESH) sbus_remote->rc.s[2] = -1;
    else sbus_remote->rc.s[2] = 0;
    /* CH9 */
    sbus_remote->rc.s[3] = (ch_raw[9] > THRESH) ? +1 : (ch_raw[9] < -THRESH ? -1 : 0);

    /* Parse status flags from byte 23 (may be 0 on some receivers) */
    sbus_remote->status.frame_lost = (sbus_buf[23] & SBUS_FLAG_FRAME_LOST) ? 1 : 0;
    sbus_remote->status.failsafe   = (sbus_buf[23] & SBUS_FLAG_FAILSAFE) ? 1 : 0;

    /* Reset the online count */
    sbus_remote->online_cnt = 0xFAU;

    /* Reset the lost flag */
    sbus_remote->rc_lost = false;
}

/**
 * @brief  Monitor SBUS connection status and clear data if offline
 * @note   This function should be called periodically (e.g., every 10ms) from a task
 *         If no valid SBUS frame is received within the timeout period,
 *         all channel data will be cleared to prevent loss of control.
 * @param  sbus_remote: pointer to SBUS_Remote_Info_Typedef structure
 * @retval none
 */
void SBUS_Connection_Monitor(SBUS_Remote_Info_Typedef *sbus_remote)
{
    if (sbus_remote == NULL)
        return;

    /* Judge the device status
     * Timeout threshold: counter decrements to 0
     * At 10ms per call, timeout = 0xFAU * 10 = ~2.5 seconds
     */
    if (sbus_remote->online_cnt == 0)
    {
        /* Connection timeout - no data received for extended period */

        /* Clear all channel data to prevent loss of control */
        for (int i = 0; i < 6; i++)
        {
            sbus_remote->rc.ch[i] = 0;
        }
        for (int i = 0; i < 4; i++)
        {
            sbus_remote->rc.s[i] = 0;
        }
        sbus_remote->status.frame_lost = 0;
        sbus_remote->status.failsafe = 0;

        /* Set the lost flag */
        sbus_remote->rc_lost = true;
    }
    else if (sbus_remote->online_cnt > 0)
    {
        /* Online count is decrementing (counter is active) */
        sbus_remote->online_cnt--;
    }
}

/**
 * @brief  Get pointer to SBUS remote control structure
 * @retval pointer to SBUS_Remote_Info_Typedef structure
 */
const SBUS_Remote_Info_Typedef *get_sbus_remote_control_point(void)
{
    return &sbus_remote_ctrl;
}

/**
 * @brief  处理一帧 SBUS 数据（由 BSP UART 回调调用）
 * @param  buf  DMA 接收缓冲区指针
 * @note   封装了 SBUS_Frame_Parse(&sbus_remote_ctrl) 调用，避免外部直接
 *         操作内部 static 变量
 */
void SBUS_Process_Buffer(volatile const uint8_t *buf)
{
    SBUS_Frame_Parse(buf, &sbus_remote_ctrl);
}
