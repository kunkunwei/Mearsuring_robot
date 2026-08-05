/**
  ******************************************************************************
  * @file           : mymotor.c  
  * @brief          : 自定义电机接口实现文件
  * @author         : [作者名]
  * @date           : 2025-09-23
  ******************************************************************************
  * @attention      : 实现DJI电机、达妙电机、廉科电机的数据解析和控制接口
  *                  提供统一的电机数据访问方式，便于上层应用调用
  ******************************************************************************
  */

#include "mymotor.h"
#include "bsp_can.h"
#include "can.h"
#include "vofa.h"
#include "vesc_can_protocol.h"

#define VESC_CAN_PACKET_SET_CURRENT 1U
#define VESC_CAN_PACKET_STATUS 9U
#define VESC_CAN_PACKET_STATUS_4 16U
#define VESC_CAN_PACKET_STATUS_5 27U
#define VESC_TACHOMETER_SCALE 6
#define VESC_PID_POS_SCALE 50.0f
#define MOTOR_TWO_PI 6.28318530717958647692f

/* DJI电机实例定义（static：只有本文件的 CAN 回调可写；外部通过 getter 只读） */
static dji_motor_measure_t chassis_motor[4];  // 底盘四个电机
/* 私有函数声明 */

static uint8_t VESC_IdToIndex(uint8_t vesc_id);
static int32_t Int32FromCan(const uint8_t *data);
static void Int32ToCan(int32_t value, uint8_t *data);


///////////////////////////DJI电机数据解析////////////////////////////////////
/**
 * @brief  DJI电机通用数据解析函数
 * @param  ptr        DJI电机数据结构体指针
 * @param  rx_message CAN接收数据缓冲区(8字节)  
 * @retval None
 */
void get_dji_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message)
{
    /* 解析DJI电机标准反馈数据 */
    (ptr)->last_ecd = (ptr)->ecd;
    (ptr)->ecd = (uint16_t) (rx_message[0] << 8 | rx_message[1]);       // 编码器位置(0-8191)
    (ptr)->rpm = (int16_t)(rx_message[2] << 8 | rx_message[3]);        // 转速(rpm)
    (ptr)->current = (int16_t)(rx_message[4] << 8 | rx_message[5]);    // 电流(mA)
    (ptr)->temp = rx_message[6];                                        // 温度(°C)
    (ptr)->status1_tick = HAL_GetTick();
}

void get_vesc_motor_measure(const CAN_RxHeaderTypeDef *header, uint8_t *rx_message)
{
    if (header == NULL || rx_message == NULL || header->IDE != CAN_ID_EXT)
    {
        return;
    }

    const uint8_t vesc_id = (uint8_t)(header->ExtId & 0xFFU);
    const uint8_t cmd_id = (uint8_t)((header->ExtId >> 8U) & 0xFFU);
    const uint8_t motor_index = VESC_IdToIndex(vesc_id);
    if (motor_index >= 4U)
    {
        return;
    }

    dji_motor_measure_t *motor = &chassis_motor[motor_index];
    const uint32_t now_tick = HAL_GetTick();
    if (cmd_id == VESC_CAN_PACKET_STATUS)
    {
        const int32_t erpm = Int32FromCan(&rx_message[0]);
        const int16_t current_deci_amp = (int16_t)((rx_message[4] << 8) | rx_message[5]);

        motor->erpm = erpm;
        motor->rpm = (int16_t)((float)erpm / VESC_M3508_POLE_PAIRS);
        motor->real_w = ((float)motor->rpm) * MOTOR_TWO_PI / 60.0f;
        motor->current = current_deci_amp;
        motor->status1_tick = now_tick;
    }
    else if (cmd_id == VESC_CAN_PACKET_STATUS_4)
    {
        const int16_t pid_pos_raw = (int16_t)((rx_message[6] << 8) | rx_message[7]);

        motor->pid_pos_deg = (float)pid_pos_raw / VESC_PID_POS_SCALE;
        motor->status4_tick = now_tick;
    }
    else if (cmd_id == VESC_CAN_PACKET_STATUS_5)
    {
        const int32_t tachometer = Int32FromCan(&rx_message[0]);
        int32_t ecd = (int32_t)(((int64_t)tachometer * 8192LL) /
                                ((int64_t)VESC_TACHOMETER_SCALE * (int64_t)VESC_M3508_POLE_PAIRS));
        ecd %= 8192;
        if (ecd < 0)
        {
            ecd += 8192;
        }

        motor->tachometer = tachometer;
        motor->real_pos = ((float)tachometer) * MOTOR_TWO_PI /
                          ((float)VESC_TACHOMETER_SCALE * VESC_M3508_POLE_PAIRS);
        motor->last_ecd = motor->ecd;
        motor->ecd = (uint16_t)ecd;
    }
}

////////////////////////////////电机实例访问接口////////////////////////////////

/**
 * @brief  获取底盘电机数据指针
 * @param  i 电机编号(0-3)
 * @retval
 *
 * */

dji_motor_measure_t* get_chassis_motor(uint8_t i){
    return &chassis_motor[(i & 0x03)];
}

void VESC_Chassis_SetCurrent(uint8_t vesc_id, float current_a)
{
    CAN_TxFrameTypeDef tx = {
        .hcan = &hcan1,
        .header.ExtId = ((uint32_t)VESC_CAN_PACKET_SET_CURRENT << 8U) | vesc_id,
        .header.IDE = CAN_ID_EXT,
        .header.RTR = CAN_RTR_DATA,
        .header.DLC = 4,
    };

    const int32_t current_milli_amp = (int32_t)(current_a * 1000.0f);
    Int32ToCan(current_milli_amp, tx.Data);
    USER_CAN_TxMessage(&tx);
}

void VESC_Chassis_SetMechanicalRpm(uint8_t vesc_id, float mechanical_rpm)
{
    CAN_TxFrameTypeDef tx = {
        .hcan = &hcan1,
        .header.IDE = CAN_ID_EXT,
        .header.RTR = CAN_RTR_DATA,
        .header.DLC = 4,
    };

    const int32_t erpm = VescCan_MechanicalRpmToErpm(mechanical_rpm,
                                                      VESC_M3508_POLE_PAIRS);
    if (!VescCan_BuildSetErpmFrame(vesc_id, erpm, &tx.header.ExtId, tx.Data))
    {
        return;
    }
    USER_CAN_TxMessage(&tx);
}


////////////////////////////////CAN回调注册////////////////////////////////
/**
 * @brief CAN1 接收中断分发回调
 * @note  将 CAN 原始帧路由到对应的电机解析函数，由 BSP 层调用
 */
static void CAN1_MotorRxDispatch(const CAN_RxHeaderTypeDef *header, uint8_t data[8])
{
#if CHASSIS_ESC_PROTOCOL == CHASSIS_ESC_PROTOCOL_VESC
    get_vesc_motor_measure(header, data);
#else
    if (header == NULL || header->IDE != CAN_ID_STD)
    {
        return;
    }

    switch (header->StdId)
    {
    case CAN1_CHASSIS_MOTOR_1_ID:
    case CAN1_CHASSIS_MOTOR_2_ID:
    case CAN1_CHASSIS_MOTOR_3_ID:
    case CAN1_CHASSIS_MOTOR_4_ID:
        get_dji_motor_measure(&chassis_motor[header->StdId - 0x201], data);
        break;
    default:
        break;
    }
#endif
}

/**
 * @brief CAN2 接收中断分发回调
 */
static void CAN2_MotorRxDispatch(const CAN_RxHeaderTypeDef *header, uint8_t data[8])
{
    (void)header;
    (void)data;
}

/**
 * @brief 将电机 CAN 接收回调注册到 BSP 层
 * @note  应在 BSP_CAN_Init() 之后、任务启动之前调用
 */
void mymotor_register_can_callbacks(void)
{
    BSP_CAN1_RegisterRxCallback(CAN1_MotorRxDispatch);
    BSP_CAN2_RegisterRxCallback(CAN2_MotorRxDispatch);
}

static uint8_t VESC_IdToIndex(uint8_t vesc_id)
{
    switch (vesc_id)
    {
    case VESC_MOTOR_1_ID:
        return 0U;
    case VESC_MOTOR_2_ID:
        return 1U;
    case VESC_MOTOR_3_ID:
        return 2U;
    case VESC_MOTOR_4_ID:
        return 3U;
    default:
        return 0xFFU;
    }
}

static int32_t Int32FromCan(const uint8_t *data)
{
    return (int32_t)(((uint32_t)data[0] << 24U) |
                     ((uint32_t)data[1] << 16U) |
                     ((uint32_t)data[2] << 8U) |
                     ((uint32_t)data[3]));
}

static void Int32ToCan(int32_t value, uint8_t *data)
{
    data[0] = (uint8_t)((uint32_t)value >> 24U);
    data[1] = (uint8_t)((uint32_t)value >> 16U);
    data[2] = (uint8_t)((uint32_t)value >> 8U);
    data[3] = (uint8_t)value;
}

bool vesc_motor_status_is_online(const dji_motor_measure_t *motor, uint32_t timeout_ms)
{
    if (motor == NULL || motor->status1_tick == 0U)
    {
        return false;
    }

    return (HAL_GetTick() - motor->status1_tick) <= timeout_ms;
}

bool vesc_motor_status4_is_online(const dji_motor_measure_t *motor, uint32_t timeout_ms)
{
    if (motor == NULL || motor->status4_tick == 0U)
    {
        return false;
    }

    return (HAL_GetTick() - motor->status4_tick) <= timeout_ms;
}
