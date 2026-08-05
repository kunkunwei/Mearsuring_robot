//
// Created by kun on 2026/7/31.
//

#ifndef __MAHONY_H
#define __MAHONY_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= */
/*  内存对齐宏定义           */
/* ========================= */
#define ALIGN_4BYTE __attribute__((aligned(4)))
#define ALIGN_8BYTE __attribute__((aligned(8)))
#define ALIGN_16BYTE __attribute__((aligned(16)))
#define ALIGN_32BYTE __attribute__((aligned(32)))

/* ========================= */
/*  姿态数据结构体（对齐）   */
/* ========================= */
typedef struct ALIGN_8BYTE {
    float q0, q1, q2, q3;      // 单位四元数 (对齐到8字节)
    float roll;                // 欧拉角 - 滚转 (弧度)
    float pitch;               // 欧拉角 - 俯仰 (弧度)
    float yaw;                 // 欧拉角 - 偏航 (弧度)
    float q_deriv[4];          // 四元数导数 (用于调试)
    float omega_corrected[3];  // 修正后的角速度 (rad/s)
} Mahony_Attitude_t;

/* ========================= */
/*  传感器数据结构体         */
/* ========================= */
typedef struct ALIGN_8BYTE {
    float gyro[3];             // 角速度 (rad/s)  [x, y, z]
    float accel[3];            // 加速度 (m/s^2)  [x, y, z]
    float mag[3];              // 磁场 (uT)       [x, y, z]
    float dt;                  // 采样周期 (s)
    uint32_t timestamp;        // 时间戳 (ms)
    uint8_t valid;             // 数据有效标志
} Mahony_ImuData_t;

/* ========================= */
/*  算法参数结构体           */
/* ========================= */
typedef struct ALIGN_8BYTE {
    float Kp_accel;            // 加速度计比例增益
    float Ki_accel;            // 加速度计积分增益
    float Kp_mag;              // 磁力计比例增益
    float Ki_mag;              // 磁力计积分增益
    float mag_weight;          // 磁场权重 (0.0 ~ 1.0)
    float integral_limit;      // 积分限幅 (rad/s)
    float dt_min;              // 最小采样周期 (s)
    float dt_max;              // 最大采样周期 (s)
} Mahony_Params_t;

/* ========================= */
/*  算法实例结构体           */
/* ========================= */
typedef struct ALIGN_8BYTE {
    Mahony_Params_t params;    // 参数
    Mahony_Attitude_t att;     // 姿态
    float integralFB_accel[3]; // 加速度计积分反馈项
    float integralFB_mag[3];   // 磁力计积分反馈项
    float mag_ref[3];          // 参考磁场方向（世界系）
    bool is_initialized;       // 初始化标志
    bool mag_initialized;      // 磁力计参考方向已初始化
    uint32_t update_count;     // 更新次数计数
    float dt_filtered;         // 滤波后的dt (用于平滑)
} Mahony_Instance_t;

/* ========================= */
/*  API函数声明              */
/* ========================= */

/**
 * @brief 初始化Mahony算法实例
 * @param inst  算法实例指针
 * @param Kp_accel 加速度计比例增益 (推荐: 0.5~2.0)
 * @param Ki_accel 加速度计积分增益 (推荐: 0.01~0.1)
 * @param Kp_mag   磁力计比例增益 (推荐: 0.3~1.0)
 * @param Ki_mag   磁力计积分增益 (推荐: 0.005~0.05)
 * @param mag_weight 磁场权重 (推荐: 0.3~0.7)
 */
void Mahony_Init(Mahony_Instance_t *inst,
                 float Kp_accel, float Ki_accel,
                 float Kp_mag, float Ki_mag,
                 float mag_weight);

/**
 * @brief 重置算法姿态（保持参数不变）
 * @param inst 算法实例指针
 */
void Mahony_Reset(Mahony_Instance_t *inst);

/**
 * @brief 设置磁力计参考方向（世界坐标系）
 * @param inst 算法实例指针
 * @param ref_x 世界系X轴磁场分量 (归一化后)
 * @param ref_y 世界系Y轴磁场分量 (归一化后)
 * @param ref_z 世界系Z轴磁场分量 (归一化后)
 */
void Mahony_SetMagReference(Mahony_Instance_t *inst, float ref_x, float ref_y, float ref_z);

/**
 * @brief 核心更新函数 (使用CMSIS-DSP加速)
 * @param inst 算法实例指针
 * @param data 传感器数据指针
 */
void Mahony_Update(Mahony_Instance_t *inst, const Mahony_ImuData_t *data);

/**
 * @brief 计算欧拉角 (从四元数转换)
 * @param inst 算法实例指针
 */
void Mahony_UpdateEuler(Mahony_Instance_t *inst);

/**
 * @brief 获取欧拉角 (弧度)
 */
static inline float Mahony_GetRoll(const Mahony_Instance_t *inst)  { return inst->att.roll; }
static inline float Mahony_GetPitch(const Mahony_Instance_t *inst) { return inst->att.pitch; }
static inline float Mahony_GetYaw(const Mahony_Instance_t *inst)   { return inst->att.yaw; }

/**
 * @brief 获取四元数
 * @param inst 算法实例指针
 * @param q 输出数组 [q0, q1, q2, q3]
 */
static inline void Mahony_GetQuaternion(const Mahony_Instance_t *inst, float q[4]) {
    q[0] = inst->att.q0;
    q[1] = inst->att.q1;
    q[2] = inst->att.q2;
    q[3] = inst->att.q3;
}

/**
 * @brief 获取修正后的角速度
 */
static inline void Mahony_GetCorrectedOmega(const Mahony_Instance_t *inst, float omega[3]) {
    omega[0] = inst->att.omega_corrected[0];
    omega[1] = inst->att.omega_corrected[1];
    omega[2] = inst->att.omega_corrected[2];
}

#ifdef __cplusplus
}
#endif


#endif //__MAHONY_H
