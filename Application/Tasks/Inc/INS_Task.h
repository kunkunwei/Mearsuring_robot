/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : INS_Task.h
 * @brief          : 惯性导航系统任务头文件
 * @details        : 实现基于BMI088 IMU的惯性导航功能，包括姿态解算、角度滤波、
 *                   温度控制等功能，使用扩展卡尔曼滤波器进行四元数姿态估计
 * @author         : Yan Yuanbin
 * @date           : 2023/04/27
 * @version        : v1.0
 ******************************************************************************
 * @attention      : 该模块是机器人控制系统的核心部分，提供精确的姿态信息
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INS_TASK_H
#define INS_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 惯性导航系统信息结构体
 * @details 包含IMU传感器数据处理后的姿态角、角速度、加速度等信息
 */
typedef struct
{
    float pit_angle;    // PITCH轴角度（俯仰角，弧度）
    float yaw_angle;    // YAW轴角度（偏航角，弧度）
    float yaw_tolangle; // YAW轴累积总角度（包含多圈旋转，弧度）
    float rol_angle;    // ROLL轴角度（横滚角，弧度）

    float pit_gyro; // PITCH轴角速度（弧度/秒）
    float yaw_gyro; // YAW轴角速度（弧度/秒）
    float rol_gyro; // ROLL轴角速度（弧度/秒）

    float angle[3]; // 三轴欧拉角数组 [PITCH, ROLL, YAW]
    float gyro[3];  // 三轴角速度数组 [PITCH, ROLL, YAW]
    float accel[3]; // 三轴加速度数组（经过滤波处理，m/s²）

    float last_yawangle;   // 上一次YAW角度值（用于多圈计算）
    int16_t YawRoundCount; // YAW轴旋转圈数计数器
} INS_Info_Typedef;

/* Exported variables ---------------------------------------------------------*/
/* INS_Info 已在 INS_Task.c 中声明为 static，外部通过下方 getter 只读访问 */

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief 获取INS姿态角数据指针
 * @return 指向三轴欧拉角数组的指针
 */
const float *get_INS_angle_point();

/**
 * @brief 获取陀螺仪角速度数据指针
 * @return 指向三轴角速度数组的指针
 */
const float *get_gyro_data_point();

/**
 * @brief 获取加速度计数据指针
 * @return 指向三轴加速度数组的指针
 */
const float *get_accel_data_point();

/**
 * @brief 获取INS完整信息结构体指针
 * @return 指向INS_Info_Typedef结构体的常量指针
 * @details 其他模块通过此函数获取惯导系统的完整状态信息
 */
const INS_Info_Typedef *get_ins_info_point();
#endif // INS_TASK_H
