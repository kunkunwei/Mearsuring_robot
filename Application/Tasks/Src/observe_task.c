#include "observe_task.h"
#include "Chassis_Task.h"
#include "mymotor.h"
#include "ist8310.h"
#include "magnetic_heading.h"
#include "minipc.h"
#include "odometry.h"
#include <math.h>
#include <string.h>

#define OBSERVE_TASK_PERIOD_MS 5
#define OBSERVE_TASK_PERIOD_S  (OBSERVE_TASK_PERIOD_MS / 1000.0f)
#define ODOM_YAW_PI 3.14159265358979323846f
#define ODOM_STILL_VX_THRESHOLD 0.006f
#define ODOM_STILL_WZ_THRESHOLD 0.020f
#define ODOM_STILL_ACCEL_THRESHOLD 0.120f
#define MAGNETOMETER_UPDATE_PERIOD_MS 10U
#define MAGNETOMETER_STALE_TIMEOUT_MS 50U

static KalmanFilter_Info_TypeDef vaEstimateKF;
static uint8_t vaEstimateKF_ready = 0U;

static float vaEstimateKF_F[4] = {
    1.0f, OBSERVE_TASK_PERIOD_S,
    0.0f, 1.0f,
};

static float vaEstimateKF_P[4] = {
    1.0f, 0.0f,
    0.0f, 1.0f,
};

static float vaEstimateKF_Q[4] = {
    10.0f, 0.0f,
    0.0f, 15.0f,
};

static float vaEstimateKF_R[4] = {
    100.0f, 0.0f,
    0.0f, 300.0f,
};

static const float vaEstimateKF_H[4] = {
    1.0f, 0.0f,
    0.0f, 1.0f,
};

static float vel_acc[2]; // vel_acc[0] = vx, vel_acc[1] = ax.
static fp32 v_real;
static fp32 aver_v;
static fp32 diff_v;
static fp32 imu_bias_estimate = -0.4f;
static Chassis_Odom_t chassis_odom;
static const chassis_move_t *local_chassis_move;
static OdomEstimator_t odom_estimator;
static MagneticHeadingEstimator_t magnetic_heading_estimator;
static MagneticHeadingOutput_t magnetic_heading_output;

static const MagneticHeadingConfig_t magnetic_heading_config = {
    .calibration = {
        .bias_ut = {0.0f, 0.0f, 0.0f},
        .soft_iron = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
        },
        .axis_source = {0U, 1U, 2U},
        .axis_sign = {1, 1, 1},
    },
    .field_norm_min_ut = 15.0f,
    .field_norm_max_ut = 100.0f,
    .field_norm_tolerance_ratio = 0.30f,
    .heading_innovation_limit_rad = 0.7853981634f,
    .correction_gain = 0.25f,
    .max_correction_rate_rad_s = 0.0349065850f,
    .reference_update_gain = 0.01f,
};

static void CalculateWheelLinearSpeed(const chassis_move_t *chassis, float *left_speed, float *right_speed);
static void UpdateIMUBias(fp32 raw_imu_accel, float aver_v);
static void BuildOdomInput(const chassis_move_t *chassis, OdomInput_t *input);
static void UpdateOdometry(const OdomOutput_t *output, uint8_t magnetic_sample_updated);
static uint8_t IsChassisStill(float vx, float wz, float accel);
static void ZeroVelocityEstimate(void);
static void ResetSegmentOdometry(uint8_t segment_id);
static float WrapPi(float angle);

void ObserveTask(void const *argument)
{
    (void)argument;

    while (!is_chassis_init_done())
    {
        osDelay(1);
    }

    local_chassis_move = get_chassis_control_point();
    xvEstimateKF_Init(&vaEstimateKF);
    OdomEstimator_Init(&odom_estimator, NULL);
    MagneticHeading_Init(&magnetic_heading_estimator, &magnetic_heading_config);

    float vrb = 0.0f;
    float vlb = 0.0f;
    fp32 raw_imu_accel = 0.0f;
    fp32 raw_imu_gyro_z = 0.0f;
    fp32 compensated_accel = 0.0f;
    OdomInput_t odom_input;
    OdomOutput_t odom_output;
    TickType_t systick = 0;
    TickType_t last_magnetic_update_tick = 0;

    for (;;)
    {
        systick = osKernelSysTick();

        uint8_t requested_segment_id = 0U;
        if (MiniPC_TakeOdomResetRequest(&requested_segment_id))
        {
            ResetSegmentOdometry(requested_segment_id);
        }

        uint8_t magnetic_sample_updated = 0U;
        if ((TickType_t)(systick - last_magnetic_update_tick) >= MAGNETOMETER_UPDATE_PERIOD_MS)
        {
            last_magnetic_update_tick = systick;
            magnetic_sample_updated = IST8310_Info_Update(&ist8310_Info) ? 1U : 0U;
        }

        CalculateWheelLinearSpeed(local_chassis_move, &vlb, &vrb);

        aver_v = (vrb + vlb) / 2.0f;
        raw_imu_gyro_z = *(local_chassis_move->chassis_imu_gyro + INS_GYRO_Z_ADDRESS_OFFSET);
        const fp32 wheel_wz = (vrb - vlb) / (2.0f * MOTOR_DISTANCE_TO_CENTER);
        diff_v = wheel_wz - raw_imu_gyro_z;

        raw_imu_accel = *(local_chassis_move->chassis_imu_accel + INS_ACCEL_X_ADDRESS_OFFSET);
        UpdateIMUBias(raw_imu_accel, aver_v);
        compensated_accel = raw_imu_accel + imu_bias_estimate;

        xvEstimateKF_Update(&vaEstimateKF, compensated_accel, aver_v);

        uint8_t chassis_still = IsChassisStill(aver_v, raw_imu_gyro_z, compensated_accel) &&
                                fabsf(local_chassis_move->state_set.vx) < ODOM_STILL_VX_THRESHOLD &&
                                fabsf(local_chassis_move->state_set.wz) < ODOM_STILL_WZ_THRESHOLD;
        if (chassis_still)
        {
            ZeroVelocityEstimate();
        }

        v_real = vel_acc[0];
        BuildOdomInput(local_chassis_move, &odom_input);
        OdomEstimator_Update(&odom_estimator, &odom_input, &odom_output);
        UpdateOdometry(&odom_output, magnetic_sample_updated);

        osDelayUntil(&systick, OBSERVE_TASK_PERIOD_MS);
    }
}

void xvEstimateKF_Init(KalmanFilter_Info_TypeDef *EstimateKF)
{
    if (EstimateKF == NULL)
    {
        return;
    }

    Kalman_Filter_Init(EstimateKF, 2, 0, 2);

    memcpy(EstimateKF->Data.A, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->Data.P, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Data.Q, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->Data.R, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->Data.H, vaEstimateKF_H, sizeof(vaEstimateKF_H));

    vaEstimateKF_ready = 1U;
}

static void CalculateWheelLinearSpeed(const chassis_move_t *chassis, float *left_speed, float *right_speed)
{
    if (chassis == NULL || left_speed == NULL || right_speed == NULL)
    {
        return;
    }

    *left_speed = (chassis->chassis_motor[0].speed + chassis->chassis_motor[3].speed) * 0.5f;
    *right_speed = (chassis->chassis_motor[1].speed + chassis->chassis_motor[2].speed) * 0.5f;
}

static void UpdateIMUBias(fp32 raw_imu_accel, float aver_v_current)
{
    if (fabsf(aver_v_current) < 0.01f && fabsf(raw_imu_accel + imu_bias_estimate) < 0.5f)
    {
        imu_bias_estimate = imu_bias_estimate * 0.999f + (-raw_imu_accel) * 0.001f;
    }
}

static void BuildOdomInput(const chassis_move_t *chassis, OdomInput_t *input)
{
    if (chassis == NULL || input == NULL)
    {
        return;
    }

    memset(input, 0, sizeof(*input));
    input->dt_s = OBSERVE_TASK_PERIOD_S;
    input->command_vx_mps = chassis->state_set.vx;
    input->command_wz_rad_s = chassis->state_set.wz;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        input->wheel[i].pos_deg = chassis->chassis_motor[i].pos_deg;
        input->wheel[i].speed_mps = chassis->chassis_motor[i].speed;
        input->wheel[i].speed_rpm = chassis->chassis_motor[i].speed_rpm;
        input->wheel[i].pos_ready = chassis->chassis_motor[i].pos_ready;
        input->wheel[i].online =
            vesc_motor_status4_is_online(chassis->chassis_motor[i].chassis_motor_measure,
                                         CHASSIS_MOTOR_STATUS4_TIMEOUT_MS);
    }

    if (chassis->chassis_INS_angle != NULL)
    {
        input->imu.yaw_rad = *(chassis->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET);
        // 里程计坡度判断使用与底盘控制一致的修正后真实Pitch
        input->imu.pitch_rad = Chassis_Hold_CorrectPitch(
            &chassis->control_manager.config.hold,
            *(chassis->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET));
        input->imu.yaw_ready = 1U;
    }

    if (chassis->chassis_imu_gyro != NULL)
    {
        input->imu.gyro_z_rad_s = *(chassis->chassis_imu_gyro + INS_GYRO_Z_ADDRESS_OFFSET);
    }
}

static void UpdateOdometry(const OdomOutput_t *output, uint8_t magnetic_sample_updated)
{
    if (output == NULL)
    {
        return;
    }

    if (output->valid == 0U)
    {
        chassis_odom.valid = 0U;
        return;
    }

    diff_v = output->slip_wz_rad_s;
    chassis_odom.wz = output->wz_rad_s;
    chassis_odom.vx = output->vx_mps;
    chassis_odom.slip = output->slip_wz_rad_s;
    chassis_odom.front_distance = output->front_distance_m;
    chassis_odom.rear_distance = output->rear_distance_m;
    chassis_odom.left_distance = output->left_distance_m;
    chassis_odom.right_distance = output->right_distance_m;
    for (uint8_t i = 0U; i < 4U; i++)
    {
        chassis_odom.wheel_weight[i] = output->wheel_weight[i];
    }
    chassis_odom.motion_mode = (uint8_t)output->motion_mode;
    chassis_odom.valid = output->valid;

    float corrected_heading = output->heading_rad;
    if (magnetic_sample_updated != 0U &&
        local_chassis_move != NULL &&
        local_chassis_move->chassis_INS_angle != NULL)
    {
        const MagneticHeadingInput_t magnetic_input = {
            .raw_mag_ut = {
                ist8310_Info.raw_mag[0],
                ist8310_Info.raw_mag[1],
                ist8310_Info.raw_mag[2],
            },
            // 代码中的Pitch对应实车Roll，供磁航向执行横滚倾斜补偿
            .roll_rad = *(local_chassis_move->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET),
            // 代码中的Roll对应实车Pitch，并使用上电零点修正纵向倾角
            .pitch_rad = Chassis_Hold_CorrectPitch(
                &local_chassis_move->control_manager.config.hold,
                *(local_chassis_move->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET)),
            .predicted_yaw_rad = output->heading_rad,
            .dt_s = MAGNETOMETER_UPDATE_PERIOD_MS / 1000.0f,
            .sample_valid = 1U,
        };
        MagneticHeading_Update(&magnetic_heading_estimator,
                               &magnetic_input,
                               &magnetic_heading_output);
        if (magnetic_heading_output.trusted != 0U)
        {
            corrected_heading = magnetic_heading_output.corrected_yaw_rad;
            OdomEstimator_SetHeading(&odom_estimator, corrected_heading);
        }
    }

    if (ist8310_Info.update_tick == 0U ||
        (uint32_t)(HAL_GetTick() - ist8310_Info.update_tick) > MAGNETOMETER_STALE_TIMEOUT_MS)
    {
        magnetic_heading_output.trusted = 0U;
    }

    chassis_odom.yaw = WrapPi(corrected_heading);
    chassis_odom.mag_body_ut[0] = magnetic_heading_output.body_mag_ut[0];
    chassis_odom.mag_body_ut[1] = magnetic_heading_output.body_mag_ut[1];
    chassis_odom.mag_body_ut[2] = magnetic_heading_output.body_mag_ut[2];
    chassis_odom.mag_yaw = magnetic_heading_output.magnetic_yaw_rad;
    chassis_odom.mag_field_norm_ut = magnetic_heading_output.field_norm_ut;
    chassis_odom.mag_innovation = magnetic_heading_output.heading_innovation_rad;
    chassis_odom.mag_quality = magnetic_heading_output.quality;
    chassis_odom.mag_online = ist8310_Info.online;
    chassis_odom.mag_trusted = magnetic_heading_output.trusted;
    chassis_odom.x += output->ds_m * cosf(chassis_odom.yaw);
    chassis_odom.y += output->ds_m * sinf(chassis_odom.yaw);
    chassis_odom.distance += output->ds_m;
}

static uint8_t IsChassisStill(float vx, float wz, float accel)
{
    return (fabsf(vx) < ODOM_STILL_VX_THRESHOLD &&
            fabsf(wz) < ODOM_STILL_WZ_THRESHOLD &&
            fabsf(accel) < ODOM_STILL_ACCEL_THRESHOLD) ? 1U : 0U;
}

static void ZeroVelocityEstimate(void)
{
    vel_acc[0] = 0.0f;
    vel_acc[1] = 0.0f;
    v_real = 0.0f;

    if (vaEstimateKF_ready != 0U)
    {
        if (vaEstimateKF.Data.xhat != NULL)
        {
            vaEstimateKF.Data.xhat[0] = 0.0f;
            vaEstimateKF.Data.xhat[1] = 0.0f;
        }
        if (vaEstimateKF.Data.xhatminus != NULL)
        {
            vaEstimateKF.Data.xhatminus[0] = 0.0f;
            vaEstimateKF.Data.xhatminus[1] = 0.0f;
        }
        if (vaEstimateKF.Output != NULL)
        {
            vaEstimateKF.Output[0] = 0.0f;
            vaEstimateKF.Output[1] = 0.0f;
        }
    }
}

static float WrapPi(float angle)
{
    while (angle > ODOM_YAW_PI)
    {
        angle -= 2.0f * ODOM_YAW_PI;
    }
    while (angle < -ODOM_YAW_PI)
    {
        angle += 2.0f * ODOM_YAW_PI;
    }
    return angle;
}

void xvEstimateKF_Update(KalmanFilter_Info_TypeDef *EstimateKF, float acc, float vel)
{
    if (EstimateKF == NULL)
    {
        return;
    }

    EstimateKF->MeasuredVector[0] = vel;
    EstimateKF->MeasuredVector[1] = acc;

    Kalman_Filter_Update(EstimateKF);

    for (uint8_t i = 0; i < 2; i++)
    {
        vel_acc[i] = EstimateKF->Output[i];
    }
}

fp32 get_KF_Spd(void)
{
    return v_real;
}

fp32 get_diff_Spd(void)
{
    return diff_v;
}

const Chassis_Odom_t *get_chassis_odom_point(void)
{
    return &chassis_odom;
}

void chassis_odom_reset(void)
{
    memset(&chassis_odom, 0, sizeof(chassis_odom));
    OdomEstimator_Reset(&odom_estimator);
    MagneticHeading_Reset(&magnetic_heading_estimator);
    memset(&magnetic_heading_output, 0, sizeof(magnetic_heading_output));
    ZeroVelocityEstimate();
}

static void ResetSegmentOdometry(uint8_t segment_id)
{
    chassis_odom.x = 0.0f;
    chassis_odom.y = 0.0f;
    chassis_odom.distance = 0.0f;
    chassis_odom.vx = 0.0f;
    chassis_odom.wz = 0.0f;
    chassis_odom.slip = 0.0f;
    chassis_odom.front_distance = 0.0f;
    chassis_odom.rear_distance = 0.0f;
    chassis_odom.left_distance = 0.0f;
    chassis_odom.right_distance = 0.0f;
    memset(chassis_odom.wheel_weight, 0, sizeof(chassis_odom.wheel_weight));
    chassis_odom.motion_mode = (uint8_t)ODOM_MOTION_STILL;
    chassis_odom.valid = 0U;
    chassis_odom.segment_id = segment_id;
    OdomEstimator_ResetDistanceOrigin(&odom_estimator);
}
