#ifndef __OBSERVE_TASK_H
#define __OBSERVE_TASK_H

#include "main.h"
#include "stdio.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "INS_Task.h"
#include "remote_control.h"
#include "kalman.h"

typedef struct
{
    float x;        // Odom X in startup frame, m.
    float y;        // Odom Y in startup frame, m.
    float yaw;      // Heading in startup frame, rad.
    float distance; // Signed forward distance, m.
    float vx;       // Forward speed used by odom, m/s.
    float wz;       // Yaw rate used by odom, rad/s.
    float slip;     // Encoder yaw rate minus IMU yaw rate, rad/s.
    float front_distance;
    float rear_distance;
    float left_distance;
    float right_distance;
    float wheel_weight[4];
    uint8_t motion_mode;
    uint8_t valid;  // 1 when slip is small enough for odom to be trusted.
} Chassis_Odom_t;

extern void ObserveTask(void const *argument);

extern void xvEstimateKF_Init(KalmanFilter_Info_TypeDef *EstimateKF);
extern void xvEstimateKF_Update(KalmanFilter_Info_TypeDef *EstimateKF, float acc, float vel);

extern fp32 get_KF_Spd(void);
extern fp32 get_diff_Spd(void);
extern const Chassis_Odom_t *get_chassis_odom_point(void);
extern void chassis_odom_reset(void);

#endif
