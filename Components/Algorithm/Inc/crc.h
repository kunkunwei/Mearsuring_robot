/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : crc.h
  * @brief          : CRC 校验
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 待测试
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CRC_H
#define CRC_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

  /* Exported functions prototypes ---------------------------------------------*/
  /**
    * @brief          计算 CRC8 校验值
    */
  extern uint8_t get_CRC8_check_sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
  /**
    * @brief          CRC8 校验验证函数
    */
  extern uint32_t verify_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);
  /**
    * @brief          在数据末尾附加 CRC8 校验字节
    */
  extern void append_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);
  /**
    * @brief          计算 CRC16 校验值
    */
  extern uint16_t get_CRC16_check_sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
  /**
    * @brief          CRC16 校验验证函数
    */
  extern uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);
  /**
    * @brief          在数据末尾附加 CRC16 校验字节
    */
  extern void append_CRC16_check_sum(uint8_t * pchMessage,uint32_t dwLength);

  extern uint16_t CRC16_INIT;

#endif
