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

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "can.h"
#include "main.h"

/* Private variables ---------------------------------------------------------*/
/**
 * @brief CAN接收回调函数存储，Device层通过注册接口设置
 */
static BSP_CAN_RxCallback_t s_can1_rx_cb = NULL;
static BSP_CAN_RxCallback_t s_can2_rx_cb = NULL;

#define BSP_CAN_DEBUG_ID_MAX 32U
#define BSP_CAN_DEBUG_FRAME_FIFO_SIZE 64U
#define BSP_CAN_TX_QUEUE_SIZE 32U

typedef struct
{
  uint8_t used;
  uint8_t reported;
  BSP_CAN_DebugRxId_t info;
} BSP_CAN_DebugIdSlot_t;

typedef struct
{
  CAN_HandleTypeDef *hcan;
  CAN_TxHeaderTypeDef header;
  uint8_t data[8];
} BSP_CAN_TxQueueItem_t;

static volatile BSP_CAN_DebugIdSlot_t s_debug_rx_ids[BSP_CAN_DEBUG_ID_MAX];
static volatile BSP_CAN_DebugRxFrame_t s_debug_rx_frame_fifo[BSP_CAN_DEBUG_FRAME_FIFO_SIZE];
static volatile uint8_t s_debug_rx_frame_head = 0U;
static volatile uint8_t s_debug_rx_frame_tail = 0U;
static volatile uint32_t s_debug_rx_frame_dropped = 0U;
static BSP_CAN_TxQueueItem_t s_can_tx_queue[BSP_CAN_TX_QUEUE_SIZE];
static volatile uint8_t s_can_tx_queue_head = 0U;
static volatile uint8_t s_can_tx_queue_tail = 0U;
static volatile uint32_t s_can_tx_queue_dropped = 0U;
static volatile uint8_t s_can_tx_queue_high_watermark = 0U;
static volatile uint8_t s_can_tx_queue_pumping = 0U;

static uint8_t BSP_CAN_TxQueueDepth(void)
{
  if (s_can_tx_queue_head >= s_can_tx_queue_tail)
  {
    return (uint8_t)(s_can_tx_queue_head - s_can_tx_queue_tail);
  }
  return (uint8_t)(BSP_CAN_TX_QUEUE_SIZE - s_can_tx_queue_tail + s_can_tx_queue_head);
}

static void BSP_CAN_TxQueuePush(const CAN_TxFrameTypeDef *frame)
{
  __disable_irq();
  const uint8_t next_head = (uint8_t)((s_can_tx_queue_head + 1U) % BSP_CAN_TX_QUEUE_SIZE);
  if (next_head == s_can_tx_queue_tail)
  {
    s_can_tx_queue_tail = (uint8_t)((s_can_tx_queue_tail + 1U) % BSP_CAN_TX_QUEUE_SIZE);
    s_can_tx_queue_dropped++;
  }

  BSP_CAN_TxQueueItem_t *slot = &s_can_tx_queue[s_can_tx_queue_head];
  slot->hcan = frame->hcan;
  slot->header = frame->header;
  for (uint8_t i = 0U; i < 8U; i++)
  {
    slot->data[i] = frame->Data[i];
  }

  s_can_tx_queue_head = next_head;
  const uint8_t queue_depth = BSP_CAN_TxQueueDepth();
  if (queue_depth > s_can_tx_queue_high_watermark)
  {
    s_can_tx_queue_high_watermark = queue_depth;
  }
  __enable_irq();
}

static void BSP_CAN_TxQueuePump(void)
{
  uint32_t tx_mailbox = 0U;

  __disable_irq();
  if (s_can_tx_queue_pumping != 0U)
  {
    __enable_irq();
    return;
  }
  s_can_tx_queue_pumping = 1U;
  __enable_irq();

  for (;;)
  {
    BSP_CAN_TxQueueItem_t item;

    __disable_irq();
    if (s_can_tx_queue_tail == s_can_tx_queue_head)
    {
      s_can_tx_queue_pumping = 0U;
      __enable_irq();
      return;
    }
    item = s_can_tx_queue[s_can_tx_queue_tail];
    __enable_irq();

    if (HAL_CAN_GetTxMailboxesFreeLevel(item.hcan) == 0U ||
        HAL_CAN_AddTxMessage(item.hcan, &item.header, item.data, &tx_mailbox) != HAL_OK)
    {
      __disable_irq();
      s_can_tx_queue_pumping = 0U;
      __enable_irq();
      return;
    }

    __disable_irq();
    s_can_tx_queue_tail = (uint8_t)((s_can_tx_queue_tail + 1U) % BSP_CAN_TX_QUEUE_SIZE);
    __enable_irq();
  }
}

static void BSP_CAN_DebugPushRxFrame(uint8_t bus, const CAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
  if (header == NULL || data == NULL)
  {
    return;
  }

  const uint8_t next_head = (uint8_t)((s_debug_rx_frame_head + 1U) % BSP_CAN_DEBUG_FRAME_FIFO_SIZE);
  if (next_head == s_debug_rx_frame_tail)
  {
    s_debug_rx_frame_tail = (uint8_t)((s_debug_rx_frame_tail + 1U) % BSP_CAN_DEBUG_FRAME_FIFO_SIZE);
    s_debug_rx_frame_dropped++;
  }

  BSP_CAN_DebugRxFrame_t *slot = (BSP_CAN_DebugRxFrame_t *)&s_debug_rx_frame_fifo[s_debug_rx_frame_head];
  slot->bus = bus;
  slot->ide = (uint8_t)header->IDE;
  slot->rtr = (uint8_t)header->RTR;
  slot->dlc = (uint8_t)header->DLC;
  slot->id = (header->IDE == CAN_ID_EXT) ? header->ExtId : header->StdId;
  slot->dropped = s_debug_rx_frame_dropped;
  for (uint8_t i = 0U; i < 8U; i++)
  {
    slot->data[i] = data[i];
  }

  s_debug_rx_frame_head = next_head;
}

static void BSP_CAN_DebugRecordRxId(uint8_t bus, const CAN_RxHeaderTypeDef *header)
{
  if (header == NULL)
  {
    return;
  }

  const uint8_t ide = (uint8_t)header->IDE;
  const uint32_t id = (header->IDE == CAN_ID_EXT) ? header->ExtId : header->StdId;

  for (uint8_t i = 0U; i < BSP_CAN_DEBUG_ID_MAX; i++)
  {
    if (s_debug_rx_ids[i].used != 0U &&
        s_debug_rx_ids[i].info.bus == bus &&
        s_debug_rx_ids[i].info.ide == ide &&
        s_debug_rx_ids[i].info.id == id)
    {
      s_debug_rx_ids[i].info.count++;
      return;
    }
  }

  for (uint8_t i = 0U; i < BSP_CAN_DEBUG_ID_MAX; i++)
  {
    if (s_debug_rx_ids[i].used == 0U)
    {
      s_debug_rx_ids[i].used = 1U;
      s_debug_rx_ids[i].reported = 0U;
      s_debug_rx_ids[i].info.bus = bus;
      s_debug_rx_ids[i].info.ide = ide;
      s_debug_rx_ids[i].info.id = id;
      s_debug_rx_ids[i].info.count = 1U;
      return;
    }
  }
}

uint8_t BSP_CAN_DebugPopNewRxId(BSP_CAN_DebugRxId_t *out)
{
  if (out == NULL)
  {
    return 0U;
  }

  __disable_irq();
  for (uint8_t i = 0U; i < BSP_CAN_DEBUG_ID_MAX; i++)
  {
    if (s_debug_rx_ids[i].used != 0U && s_debug_rx_ids[i].reported == 0U)
    {
      *out = s_debug_rx_ids[i].info;
      s_debug_rx_ids[i].reported = 1U;
      __enable_irq();
      return 1U;
    }
  }
  __enable_irq();

  return 0U;
}

uint8_t BSP_CAN_DebugPopRxFrame(BSP_CAN_DebugRxFrame_t *out)
{
  if (out == NULL)
  {
    return 0U;
  }

  __disable_irq();
  if (s_debug_rx_frame_tail == s_debug_rx_frame_head)
  {
    __enable_irq();
    return 0U;
  }

  *out = s_debug_rx_frame_fifo[s_debug_rx_frame_tail];
  s_debug_rx_frame_tail = (uint8_t)((s_debug_rx_frame_tail + 1U) % BSP_CAN_DEBUG_FRAME_FIFO_SIZE);
  __enable_irq();

  return 1U;
}

/**
 * @brief 注册 CAN1 接收回调函数
 */
void BSP_CAN1_RegisterRxCallback(BSP_CAN_RxCallback_t cb)
{
  s_can1_rx_cb = cb;
}

/**
 * @brief 注册 CAN2 接收回调函数
 */
void BSP_CAN2_RegisterRxCallback(BSP_CAN_RxCallback_t cb)
{
  s_can2_rx_cb = cb;
}

/**
 * @brief the structure that contains the Information of CAN Receive.
 */
CAN_RxHeaderTypeDef USER_CAN_RxInstance;
/**
 * @brief the array that contains the Information of CAN Receive data.
 */
uint8_t USER_CAN_RxFrameData[8];
/**
 * @brief the structure that contains the Information of CAN Transmit.
 */

CAN_TxFrameTypeDef ChassisTxFrame = {
  .hcan = &hcan1,
  .header.StdId = CAN1_CMD_ALL_ID,
  .header.IDE = CAN_ID_STD,
  .header.RTR = CAN_RTR_DATA,
  .header.DLC = 8,
};
/**
 * @brief  Configures the CAN Filter.
 * @param  None
 * @retval None
 */
void BSP_CAN_Init(void)
{
  CAN_FilterTypeDef CAN_FilterConfig = {0};

  /* Update the CAN1 filter Conifguration */
  CAN_FilterConfig.FilterActivation = ENABLE;
  CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterConfig.FilterIdHigh = 0x0000;
  CAN_FilterConfig.FilterIdLow = 0x0000;
  CAN_FilterConfig.FilterMaskIdHigh = 0x0000;
  CAN_FilterConfig.FilterMaskIdLow = 0x0000;
  CAN_FilterConfig.FilterBank = 0;
  CAN_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN_FilterConfig.SlaveStartFilterBank = 0;

  /* configures the CAN1 filter */
  if (HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the CAN1 module. */
  HAL_CAN_Start(&hcan1);

  /* Enable CAN1 FIFO0 RX and TX mailbox empty interrupts */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);

  /* Update the CAN2 filter Conifguration */
  CAN_FilterConfig.FilterBank = 14;
  CAN_FilterConfig.SlaveStartFilterBank = 14;

  /* configures the CAN2 filter */
  // if (HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfig) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  /* Start the CAN2 module. */
  // HAL_CAN_Start(&hcan2);

  /* Enable CAN2 FIFO0 interrupts */
  // HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader)
{
  if (TxHeader == NULL || TxHeader->hcan == NULL)
  {
    return;
  }

  uint32_t tx_mailbox = 0U;

  BSP_CAN_TxQueuePump();
  if (s_can_tx_queue_tail != s_can_tx_queue_head)
  {
    BSP_CAN_TxQueuePush(TxHeader);
    BSP_CAN_TxQueuePump();
    return;
  }

  if (HAL_CAN_GetTxMailboxesFreeLevel(TxHeader->hcan) != 0U &&
      HAL_CAN_AddTxMessage(TxHeader->hcan, &TxHeader->header, TxHeader->Data, &tx_mailbox) == HAL_OK)
  {
    return;
  }

  BSP_CAN_TxQueuePush(TxHeader);
  BSP_CAN_TxQueuePump();
}

uint32_t BSP_CAN_GetTxQueueDropped(void)
{
  return s_can_tx_queue_dropped;
}

uint8_t BSP_CAN_GetTxQueueHighWatermark(void)
{
  return s_can_tx_queue_high_watermark;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  (void)hcan;
  BSP_CAN_TxQueuePump();
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  (void)hcan;
  BSP_CAN_TxQueuePump();
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  (void)hcan;
  BSP_CAN_TxQueuePump();
}

/**
 * @brief  USER function to converting the CAN1 received message.
 * @param  Instance: pointer to the CAN Register base address
 * @param  StdId: Specifies the standard identifier.
 * @param  data: array that contains the received massage.
 * @retval None
 */

static void CAN1_RxFifo0RxHandler(const CAN_RxHeaderTypeDef *header, uint8_t data[8])
{
  if (s_can1_rx_cb != NULL)
  {
    s_can1_rx_cb(header, data);
  }
}

//------------------------------------------------------------------------------

/**
 * @brief  USER function to converting the CAN2 received message.
 * @param  Instance: pointer to the CAN Register base address
 * @param  StdId: Specifies the standard identifier.
 * @param  data: array that contains the received massage.
 * @retval None
 */
static void CAN2_RxFifo0RxHandler(const CAN_RxHeaderTypeDef *header, uint8_t data[8])
{
  if (s_can2_rx_cb != NULL)
  {
    s_can2_rx_cb(header, data);
  }
}
//------------------------------------------------------------------------------

/**
 * @brief  Rx FIFO 0 message pending callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get an CAN frame from the Rx FIFO zone into the message RAM. */
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &USER_CAN_RxInstance, USER_CAN_RxFrameData);

  /* judge the instance of receive frame data */
  if (hcan->Instance == CAN1)
  {
    BSP_CAN_DebugPushRxFrame(1U, &USER_CAN_RxInstance, USER_CAN_RxFrameData);
    BSP_CAN_DebugRecordRxId(1U, &USER_CAN_RxInstance);
    CAN1_RxFifo0RxHandler(&USER_CAN_RxInstance, USER_CAN_RxFrameData);
  }
  else if (hcan->Instance == CAN2)
  {
    BSP_CAN_DebugRecordRxId(2U, &USER_CAN_RxInstance);
    CAN2_RxFifo0RxHandler(&USER_CAN_RxInstance, USER_CAN_RxFrameData);
  }
}
//------------------------------------------------------------------------------
