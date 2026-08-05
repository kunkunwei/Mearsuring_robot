#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

typedef enum
{
    ODOM_MOTION_STILL = 0,
    ODOM_MOTION_STRAIGHT = 1,
    ODOM_MOTION_TURN = 2,
    ODOM_MOTION_SLOPE = 3,
} OdomMotionMode_t;

typedef enum
{
    ODOM_STRAIGHT_PAIR_REAR = 0,
    ODOM_STRAIGHT_PAIR_FRONT = 1,
} OdomStraightPair_t;

typedef struct
{
    float wheel_radius_m;
    float wheel_track_m;
    float straight_wz_threshold;
    float still_vx_threshold;
    float still_wz_threshold;
    float slope_pitch_threshold;
    float wheel_speed_outlier_ratio;
    float wheel_speed_outlier_offset;
    float max_wheel_delta_m;
    float imu_yaw_turn_weight;
    float imu_yaw_straight_weight;
    OdomStraightPair_t straight_pair;
} OdomConfig_t;

typedef struct
{
    float pos_deg;
    float speed_mps;
    float speed_rpm;
    uint8_t pos_ready;
    uint8_t online;
} OdomWheelSample_t;

typedef struct
{
    float yaw_rad;
    float gyro_z_rad_s;
    float pitch_rad;
    uint8_t yaw_ready;
} OdomImuSample_t;

typedef struct
{
    OdomWheelSample_t wheel[4];
    OdomImuSample_t imu;
    float command_vx_mps;
    float command_wz_rad_s;
    float dt_s;
} OdomInput_t;

typedef struct
{
    float ds_m;
    float dtheta_rad;
    float heading_rad;
    float vx_mps;
    float wz_rad_s;
    float slip_wz_rad_s;
    float front_distance_m;
    float rear_distance_m;
    float left_distance_m;
    float right_distance_m;
    float wheel_weight[4];
    OdomMotionMode_t motion_mode;
    uint8_t valid;
} OdomOutput_t;

typedef struct
{
    OdomConfig_t config;
    float last_pos_deg[4];
    float origin_pos_deg[4];
    float last_yaw_rad;
    float heading_rad;
    uint8_t wheel_ready[4];
    uint8_t origin_ready[4];
    uint8_t yaw_ready;
} OdomEstimator_t;

void OdomEstimator_Init(OdomEstimator_t *estimator, const OdomConfig_t *config);
void OdomEstimator_Reset(OdomEstimator_t *estimator);
void OdomEstimator_ResetDistanceOrigin(OdomEstimator_t *estimator);
void OdomEstimator_SetHeading(OdomEstimator_t *estimator, float heading_rad);
void OdomEstimator_Update(OdomEstimator_t *estimator, const OdomInput_t *input, OdomOutput_t *output);

#endif
