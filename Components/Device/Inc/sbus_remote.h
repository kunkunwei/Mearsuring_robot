/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : sbus_remote.h
 * @brief          : SBUS remote control interface for FlySky i6X - FS-IA6B
 * @author         : Generated for SBUS Protocol
 * @date           : 2025/12/15
 * @version        : v1.0
 ******************************************************************************
 * @attention      : SBUS Protocol for FlySky i6X transmitter with FS-IA6B receiver
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SBUS_REMOTE_H
#define SBUS_REMOTE_H

/* Includes ------------------------------------------------------------------*/
#include "config.h"

/* Exported defines -----------------------------------------------------------*/
/**
 * @brief Length of SBUS frame
 */
#define SBUS_FRAME_LENGTH 25u

/**
 * @brief SBUS frame header
 */
#define SBUS_FRAME_HEADER 0x0F

/**
 * @brief SBUS frame footer
 */
#define SBUS_FRAME_FOOTER 0x00

/**
 * @brief Number of SBUS channels
 */
#define SBUS_CHANNEL_NUM 16u

/**
 * @brief SBUS channel value range (0-2047, 11-bit)
 */
#define SBUS_CH_VALUE_MIN 0u
#define SBUS_CH_VALUE_MAX 2047u
#define SBUS_CH_VALUE_MID 1024u

/**
 * @brief SBUS channel value offset (to make center at 0)
 */
#define SBUS_CH_VALUE_OFFSET 1024

/**
 * @brief SBUS digital channel positions
 */
#define SBUS_DIGI_CH1 16u
#define SBUS_DIGI_CH2 17u

/**
 * @brief SBUS flags in byte 23
 */
#define SBUS_FLAG_CH17       0x80  // bit7 - Digital channel 17
#define SBUS_FLAG_CH18       0x40  // bit6 - Digital channel 18
#define SBUS_FLAG_FRAME_LOST 0x20  // bit5 - Frame lost flag
#define SBUS_FLAG_FAILSAFE   0x10  // bit4 - Failsafe activated

/**
 * @brief Length of SBUS receive buffer
 */
#define SBUS_RX_BUFFER_SIZE 50u

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for SBUS remote control.
 */
typedef struct
{
    /**
     * @brief structure that contains the channel information
     *        For your FlySky i6X (10ch active):
     *        - ch[0..5]: 6 analog axes, value around [-784, +784]
     *        - s[0]: CH6  two-position switch  (-1 / +1)
     *        - s[1]: CH7  two-position switch  (-1 / +1)
     *        - s[2]: CH8  three-position switch (-1 / 0 / +1)
     *        - s[3]: CH9  two-position switch  (-1 / +1)
     */
    struct
    {
        int16_t ch[6]; /*!< 6 analog channels, centered around 0 */
        int8_t  s[4];  /*!< 4 switch channels mapped to {-1,0,+1} */
    } rc;

    /**
     * @brief SBUS status flags
     */
    struct
    {
        uint8_t frame_lost : 1; /*!< Frame lost flag */
        uint8_t failsafe : 1;   /*!< Failsafe activated flag */
        uint8_t reserved : 6;   /*!< Reserved bits */
    } status;

    bool rc_lost;       /*!< Remote control lost flag */
    uint8_t online_cnt; /*!< Online count for connection monitoring */
} SBUS_Remote_Info_Typedef;

/* Exported variables ---------------------------------------------------------*/
/* sbus_remote_ctrl 已在 sbus_remote.c 中声明为 static，外部通过下方 getter 只读访问 */

/**
 * @brief SBUS usart RxDMA MultiBuffer
 */
extern uint8_t SBUS_Remote_MultiRx_Buf[2][SBUS_RX_BUFFER_SIZE];

/* ----------------------- SBUS Channel Mapping Definition ------------------- */
/* FlySky i6X standard channel mapping:
 * Channel 0 (1): Right stick horizontal (Roll)
 * Channel 1 (2): Right stick vertical (Pitch)
 * Channel 2 (3): Left stick vertical (Throttle)
 * Channel 3 (4): Left stick horizontal (Yaw)
 * Channel 4 (5): SWA (3-position switch)
 * Channel 5 (6): SWB (3-position switch)
 * Channel 6 (7): SWC (2-position switch)
 * Channel 7 (8): SWD (2-position switch)
 * Channel 8 (9): VRA (variable pot)
 * Channel 9 (10): VRB (variable pot)
 */

/* ----------------------- SBUS Switch Value Definition --------------------- */
/* For 3-position switches (SWA, SWB):
 * UP position: ~2000
 * MID position: ~1500
 * DOWN position: ~1000
 * For 2-position switches (SWC, SWD):
 * UP position: ~2000
 * DOWN position: ~1000
 */

#define SBUS_SW_UP_THRESHOLD       1700
#define SBUS_SW_MID_THRESHOLD_LOW  1200
#define SBUS_SW_MID_THRESHOLD_HIGH 1700
#define SBUS_SW_DOWN_THRESHOLD     1200

#define sbus_switch_is_up(val)     ((val) > SBUS_SW_UP_THRESHOLD)
#define sbus_switch_is_mid(val)    (((val) >= SBUS_SW_MID_THRESHOLD_LOW) && ((val) <= SBUS_SW_MID_THRESHOLD_HIGH))
#define sbus_switch_is_down(val)   ((val) < SBUS_SW_DOWN_THRESHOLD)

/* 针对 rc.s[] 归一化值 {-1, 0, +1} 的判断宏（与 DBUS 的 switch_is_xxx 对应） */
#define SBUS_SW_UP   ( -1)
#define SBUS_SW_MID  ( 0)
#define SBUS_SW_DOWN (1)
#define sbus_sw_is_up(s)   ((s) == SBUS_SW_UP)
#define sbus_sw_is_mid(s)  ((s) == SBUS_SW_MID)
#define sbus_sw_is_down(s) ((s) == SBUS_SW_DOWN)

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief  Parse SBUS frame and update remote control data
 * @param  sbus_buf: pointer to array containing SBUS frame data
 * @param  sbus_remote: pointer to SBUS_Remote_Info_Typedef structure
 * @retval none
 */
extern void SBUS_Frame_Parse(volatile const uint8_t *sbus_buf, SBUS_Remote_Info_Typedef *sbus_remote);

/**
 * @brief  Monitor SBUS connection status and clear data if offline
 * @param  sbus_remote: pointer to SBUS_Remote_Info_Typedef structure
 * @retval none
 */
extern void SBUS_Connection_Monitor(SBUS_Remote_Info_Typedef *sbus_remote);

/**
 * @brief  Get pointer to SBUS remote control structure
 * @retval pointer to SBUS_Remote_Info_Typedef structure
 */
extern const SBUS_Remote_Info_Typedef *get_sbus_remote_control_point(void);

/**
 * @brief  处理一帧 SBUS 数据（由 BSP UART 回调调用）
 * @param  buf DMA 接收缓冲区指针
 * @note   封装了对内部 static sbus_remote_ctrl 的写操作
 */
void SBUS_Process_Buffer(volatile const uint8_t *buf);

/**
 * @brief  Verify SBUS frame integrity
 * @param  sbus_buf: pointer to array containing SBUS frame data
 * @retval true if frame is valid, false otherwise
 */
extern bool SBUS_Frame_Verify(volatile const uint8_t *sbus_buf);

#endif // SBUS_REMOTE_H
