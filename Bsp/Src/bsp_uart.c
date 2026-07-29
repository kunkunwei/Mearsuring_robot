/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.c
  * @brief          : bsp uart functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"


#include "usart.h"
#include "minipc.h"
#ifdef USE_SBUS_PROTOCOL
#include "sbus_remote.h"
#else
#include "remote_control.h"
#endif

/* Private variables ---------------------------------------------------------*/

/* DMA双缓冲 */
__attribute__ ((section (".AXI_SRAM"))) uint8_t Mode_Buf[2][1];
__attribute__ ((section (".AXI_SRAM"))) uint8_t USART6_MiniPC_Buf[2][MINIPC_UART_RX_BUFFER_SIZE];
uint8_t Usart_Mode=0;
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Starts the multi_buffer DMA Transfer with interrupt enabled.
  */
static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t *, uint32_t);
static void USART6_RxDMA_MultiBufferStart(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t *, uint32_t);



/**
  * @brief  Configures the USART.
  * @param  None
  * @retval None
  */
void BSP_USART_Init(void)
{
  /* Starts the remote control multi_buffer DMA Transfer with interrupt enabled. */
#ifdef USE_SBUS_PROTOCOL
	/* Starts the SBUS remote control multi_buffer DMA Transfer with interrupt enabled. */
	USART_RxDMA_MultiBufferStart(&huart3,(uint32_t *)&(huart3.Instance->DR),(uint32_t *)SBUS_Remote_MultiRx_Buf[0],(uint32_t *)SBUS_Remote_MultiRx_Buf[1],SBUS_RX_BUFFER_SIZE);
#else
	USART_RxDMA_MultiBufferStart(&huart3,(uint32_t *)&(huart3.Instance->DR),(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],SBUS_RX_BUF_NUM);
#endif
	  /* Starts the Referee multi_buffer DMA Transfer with interrupt enabled. */
	// USART_RxDMA_MultiBufferStart(&huart1,(uint32_t *)&(huart1.Instance->DR),(uint32_t *)Mode_Buf[0],(uint32_t *)Mode_Buf[1],1);

	USART6_RxDMA_MultiBufferStart(&huart6,
								  (uint32_t *)&(huart6.Instance->DR),
								  (uint32_t *)USART6_MiniPC_Buf[0],
								  (uint32_t *)USART6_MiniPC_Buf[1],
								  MINIPC_UART_RX_BUFFER_SIZE);

}
//------------------------------------------------------------------------------

/**
  * @brief  Starts the multi_buffer DMA Transfer with interrupt enabled.
  * @param  huart       pointer to a UART_HandleTypeDef structure that contains
  *                     the configuration information for the specified USART Stream.  
  * @param  SrcAddress pointer to The source memory Buffer address
  * @param  DstAddress pointer to The destination memory Buffer address
  * @param  SecondMemAddress pointer to The second memory Buffer address in case of multi buffer Transfer  
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval none
  */
static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{	
  /* configuare the huart Reception Type TOIDLE */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

  /* configuare the huart Receicve Size */
	huart->RxXferSize = SBUS_RX_BUF_NUM;
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  /* Disable DMA */
  do{
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(huart->hdmarx->Instance->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
  huart->hdmarx->Instance->PAR = (uint32_t)SrcAddress;

  /* Configure the destination memory Buffer address */
  huart->hdmarx->Instance->M0AR = (uint32_t)DstAddress;

  /* Configure DMA Stream destination address */
  huart->hdmarx->Instance->M1AR = (uint32_t)SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  huart->hdmarx->Instance->NDTR = DataLength;

  /* Enable double memory buffer */
  SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}
//------------------------------------------------------------------------------

/**
  * @brief  Starts USART6 fixed-length frame double-buffer DMA reception.
  * @param  huart            UART handle (USART6)
  * @param  SrcAddress       UART DR address
  * @param  DstAddress       DMA Memory0 address
  * @param  SecondMemAddress DMA Memory1 address
  * @param  DataLength       Frame length (FRAME_SIZE)
  * @retval None
  */
static void USART6_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
	/* configuare the huart Reception Type TOIDLE */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

	/* USART6 fixed frame size should match DataLength */
	huart->RxXferSize = (uint16_t)DataLength;

	/* Enable the DMA transfer for the receiver request */
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

	/* Enalbe IDLE interrupt */
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

	/* Disable DMA */
	do{
		__HAL_DMA_DISABLE(huart->hdmarx);
	}while(huart->hdmarx->Instance->CR & DMA_SxCR_EN);

	/* Configure the source memory Buffer address */
	huart->hdmarx->Instance->PAR = (uint32_t)SrcAddress;

	/* Configure the destination memory Buffer address */
	huart->hdmarx->Instance->M0AR = (uint32_t)DstAddress;

	/* Configure DMA Stream destination address */
	huart->hdmarx->Instance->M1AR = (uint32_t)SecondMemAddress;

	/* Configure the length of data to be transferred from source to destination */
	huart->hdmarx->Instance->NDTR = DataLength;

	/* Enable double memory buffer */
	SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);

	/* Enable DMA */
	__HAL_DMA_ENABLE(huart->hdmarx);
}
//------------------------------------------------------------------------------

/**
  * @brief  USER USART3 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
#ifdef USE_SBUS_PROTOCOL
	/* SBUS Protocol Handler */
	/* Current memory buffer used is Memory 0 */
	if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
	{
		//Disable DMA
		__HAL_DMA_DISABLE(huart->hdmarx);

		huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
		/* reset the receive count */
		__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUFFER_SIZE);

		// if(Size >= SBUS_FRAME_LENGTH)
		// {
		// 	SBUS_Frame_Parse(SBUS_Remote_MultiRx_Buf[0],&sbus_remote_ctrl);
		// }
		/* Parse SBUS frame, Frame_Verify will check validity */
		SBUS_Process_Buffer(SBUS_Remote_MultiRx_Buf[0]);
	}
	/* Current memory buffer used is Memory 1 */
	else
	{
		//Disable DMA
		__HAL_DMA_DISABLE(huart->hdmarx);

		huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);

		/* reset the receive count */
		__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUFFER_SIZE);

		// if(Size >= SBUS_FRAME_LENGTH)
		// {
		// 	SBUS_Frame_Parse(SBUS_Remote_MultiRx_Buf[1],&sbus_remote_ctrl);
		// }
		/* Parse SBUS frame, Frame_Verify will check validity */
		SBUS_Process_Buffer(SBUS_Remote_MultiRx_Buf[1]);
	}
#else
	/* DBUS Protocol Handler */
  /* Current memory buffer used is Memory 0 */
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
  {
			//Disable DMA 
			__HAL_DMA_DISABLE(huart->hdmarx);

			huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
      /* reset the receive count */
      __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

      if(Size == RC_FRAME_LENGTH)
      {
        RC_Process_Buffer(SBUS_MultiRx_Buf[0], Size);
      }
  }
  /* Current memory buffer used is Memory 1 */
  else
  {
			//Disable DMA 
			__HAL_DMA_DISABLE(huart->hdmarx);

			huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
		
      /* reset the receive count */
      __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

      if(Size == RC_FRAME_LENGTH)
      {
        RC_Process_Buffer(SBUS_MultiRx_Buf[1], Size);
      }
  }
#endif
}
////------------------------------------------------------------------------------

/**
  * @brief  USER USART1 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
// static void USER_USART1_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
// {
//   //printf("%d,%d\r\n",Mode_Buf[0][0],Mode_Buf[1][0]);
//   //Usart_Mode = Mode_Buf[0][0];
//
//   /* Current memory buffer used is Memory 0 */
//   if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
//   {
// 			//Disable DMA
// 			__HAL_DMA_DISABLE(huart->hdmarx);
//
// 			huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
//
//       if(Size == 1)
//       {
//         //Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[0]);
// 				// memset(Referee_System_Info_MultiRx_Buf[0],0,REFEREE_RXFRAME_LENGTH);
//         // printf("Mode:%d\r\n",Mode_Buf[0][0]);
//         Usart_Mode = Mode_Buf[0][0];
//       /* reset the receive count */
//       __HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH);
//       }
//   }
//   /* Current memory buffer used is Memory 1 */
//   else
//   {
// 			//Disable DMA
// 			__HAL_DMA_DISABLE(huart->hdmarx);
//
// 			huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
//
//       if(Size == 1)
//       {
//         // Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[1]);
//         // printf("Mode:%d\r\n",Mode_Buf[1][0]);
//         Usart_Mode = Mode_Buf[1][0];
// 				// memset(Referee_System_Info_MultiRx_Buf[1],0,REFEREE_RXFRAME_LENGTH);
//       /* reset the receive count */
//       __HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH);
//       }
//   }
// }
/**
 * @brief UART6接收回调（在HAL_UARTEx_RxEventCallback中调用）解析裁判系统信息
 */
void USER_USART6_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint8_t *rx_buf = NULL;

	if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
	{
		__HAL_DMA_DISABLE(huart->hdmarx);
		rx_buf = USART6_MiniPC_Buf[1];
		huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
	}
	else
	{
		__HAL_DMA_DISABLE(huart->hdmarx);
		rx_buf = USART6_MiniPC_Buf[0];
		huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
	}

	if (Size > MINIPC_UART_RX_BUFFER_SIZE)
	{
		Size = MINIPC_UART_RX_BUFFER_SIZE;
	}

	(void)MiniPC_UpdateChassisCmdFromStream(rx_buf, Size);

	__HAL_DMA_SET_COUNTER(huart->hdmarx, MINIPC_UART_RX_BUFFER_SIZE);
}
/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
  //printf("USART1,%d\r\n",Size);
	if(huart->Instance == USART3)
	{
		USER_USART3_RxHandler(huart,Size);
	}else if(huart->Instance == USART1)
	{
		// USER_USART1_RxHandler(huart,Size);
		// PC_Ctrl_RxHandler(huart,Size);
    
	}
	else if(huart->Instance == USART6)
	{
		USER_USART6_RxHandler(huart,Size);
	}

  /* reset the Reception Type */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}
//------------------------------------------------------------------------------
