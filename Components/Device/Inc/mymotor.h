/**
  ******************************************************************************
  * @file           : mymotor.h
  * @brief          : 自定义电机接口头文件
  * @author         : [作者名]
  * @date           : 2025-09-23
  ******************************************************************************
  * @attention      : 包含DJI电机、达妙电机、廉科电机的统一接口定义
  *                  支持发射机构、云台、底盘等多种电机控制需求
  ******************************************************************************
  */

#ifndef __MYMOTOR_H__
#define __MYMOTOR_H__

#include "main.h"

#define CHASSIS_ESC_PROTOCOL_DJI  0
#define CHASSIS_ESC_PROTOCOL_VESC 1
/* Change to CHASSIS_ESC_PROTOCOL_DJI when switching back to the old DJI ESCs. */
#define CHASSIS_ESC_PROTOCOL CHASSIS_ESC_PROTOCOL_VESC

#define VESC_M3508_POLE_PAIRS 7.0f
#define VESC_MOTOR_1_ID 1U
#define VESC_MOTOR_2_ID 2U
#define VESC_MOTOR_3_ID 3U
#define VESC_MOTOR_4_ID 4U
/* CAN通信ID定义 */
/**
 * @brief CAN消息ID枚举
 * @note GM6020电流给定值范围：-16384~0~16384, 对应最大转矩电流范围 -3A~0~3A
 *       转矩常数 741 mN·m/A
 */
typedef enum
{
    /* CAN1总线电机ID定义 */
    CAN1_CHASSIS_MOTOR_1_ID = 0x201,   // 底盘电机ID
    CAN1_CHASSIS_MOTOR_2_ID = 0x202,   // 底盘电机ID
    CAN1_CHASSIS_MOTOR_3_ID = 0x203,   // 底盘电机ID
    CAN1_CHASSIS_MOTOR_4_ID = 0x204,   // 底盘电机ID
    /*控制ID*/
    CAN1_CMD_ALL_ID=0x200,                 // CAN1发送命令帧ID,控制底盘电机
} can_msg_id_e;

/**
 * @brief DJI电机测量数据结构体
 * @details 用于存储DJI电机的反馈数据和计算得到的物理量
 */
typedef struct
{
    uint16_t last_ecd;       //上次转子机械角度 (0-8191)，用于计算位置变化量
    uint16_t ecd;          // 编码器位置原始值(0-8191)
    int16_t rpm;           // 转速原始值(rpm)
    int16_t current;       // DJI raw current; VESC Status1 current in 0.1 A.
    uint8_t temp;          // 温度(°C)
    int32_t erpm;          // VESC electrical RPM
    int32_t tachometer;    // VESC tachometer
    float pid_pos_deg;     // VESC PID position now, degree
    uint32_t status1_tick;
    uint32_t status4_tick;

    float real_w;          // 实际角速度(rad/s)
    float real_pos;        // 实际位置(rad)

    // float speed;           // 目标速度
    // float accel;           // 加速度
    
    int16_t target_current; // 目标电流值
}dji_motor_measure_t;

/* DJI电机数据解析函数声明 */
void get_dji_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message);
void get_chassis_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message);
void get_vesc_motor_measure(const CAN_RxHeaderTypeDef *header, uint8_t *rx_message);
void VESC_Chassis_SetCurrent(uint8_t vesc_id, float current_a);
void VESC_Chassis_SetMechanicalRpm(uint8_t vesc_id, float mechanical_rpm);
bool vesc_motor_status_is_online(const dji_motor_measure_t *motor, uint32_t timeout_ms);
bool vesc_motor_status4_is_online(const dji_motor_measure_t *motor, uint32_t timeout_ms);

/* DJI电机实例已在 mymotor.c 中声明为 static，外部通过下方 getter 访问 */
/* DJI电机访问函数声明 */

dji_motor_measure_t* get_chassis_motor(uint8_t i);  // 获取底盘电机指针，i=0~3分别对应4个底盘电机

/**
 * @brief 将电机 CAN 接收回调注册到 BSP 层
 * @note  应在 BSP_CAN_Init() 之后、任务启动之前调用
 */
void mymotor_register_can_callbacks(void);
/////////////////////////////////////////////////////////////////////////////////


#endif // !__MYMOTOR_H__
