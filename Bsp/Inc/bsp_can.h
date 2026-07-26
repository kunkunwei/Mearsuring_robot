/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the can filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f4xx.h"
/* Exported types ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
typedef struct {
		CAN_HandleTypeDef *hcan;
    CAN_RxHeaderTypeDef header;
    uint8_t 			Data[8];
} CAN_RxFrameTypeDef;

typedef struct {
		CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef header;
    uint8_t				Data[8];
}CAN_TxFrameTypeDef;


/**
 * @brief CAN接收回调函数类型
 * @note  BSP层不知道具体的电机类型，通过注册回调将数据上传给Device层解析
 */
typedef void (*BSP_CAN_RxCallback_t)(const CAN_RxHeaderTypeDef *header, uint8_t data[8]);

typedef struct
{
    uint8_t bus;
    uint8_t ide;
    uint32_t id;
    uint32_t count;
} BSP_CAN_DebugRxId_t;

typedef struct
{
    uint8_t bus;
    uint8_t ide;
    uint8_t rtr;
    uint8_t dlc;
    uint32_t id;
    uint8_t data[8];
    uint32_t dropped;
} BSP_CAN_DebugRxFrame_t;


extern CAN_TxFrameTypeDef Shoot_Pull_TxFrame;
extern CAN_TxFrameTypeDef ALLFricTxFrame;


extern CAN_TxFrameTypeDef YAW_MOTOR_FRAME;
extern CAN_TxFrameTypeDef PITCH_MOTOR_FRAME;

extern CAN_TxFrameTypeDef ChassisTxFrame;

/**
  * @brief  Configures the CAN Filter.
  */
extern void BSP_CAN_Init(void);
/**
  * @brief  USER function to transmit the Specifies message.
  */
extern void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader);
/**
  * @brief  Register CAN1 receive callback (called in CAN1 Rx FIFO0 ISR).
  */
void BSP_CAN1_RegisterRxCallback(BSP_CAN_RxCallback_t cb);
/**
  * @brief  Register CAN2 receive callback (called in CAN2 Rx FIFO0 ISR).
  */
void BSP_CAN2_RegisterRxCallback(BSP_CAN_RxCallback_t cb);
/**
  * @brief  Pop one newly observed CAN ID for debug print.
  */
uint8_t BSP_CAN_DebugPopNewRxId(BSP_CAN_DebugRxId_t *out);
/**
  * @brief  Pop one raw CAN RX frame for debug print.
  */
uint8_t BSP_CAN_DebugPopRxFrame(BSP_CAN_DebugRxFrame_t *out);
/**
  * @brief  get
  */

#endif //BSP_CAN_H
