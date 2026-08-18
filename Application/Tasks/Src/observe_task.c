#include "observe_task.h"
#include "Chassis_Task.h"
#include "mymotor.h"
#include <math.h>
#include <string.h>

#define OBSERVE_TASK_PERIOD_MS 5
#define OBSERVE_TASK_PERIOD_S  (OBSERVE_TASK_PERIOD_MS / 1000.0f)
#define ODOM_SLIP_WZ_VALID_THRESHOLD 0.35f
#define ODOM_YAW_PI 3.14159265358979323846f
#define ODOM_VX_DEADBAND 0.003f
#define ODOM_WZ_DEADBAND 0.010f
#define ODOM_STILL_VX_THRESHOLD 0.006f
#define ODOM_STILL_WZ_THRESHOLD 0.020f
#define ODOM_STILL_ACCEL_THRESHOLD 0.120f
#define ODOM_STILL_DS_THRESHOLD (ODOM_STILL_VX_THRESHOLD * OBSERVE_TASK_PERIOD_S)
#define ODOM_ENCODER_RANGE 8192
#define ODOM_WHEEL_CIRCUMFERENCE (2.0f * ODOM_YAW_PI * WHEEL_R)
#define ODOM_ENCODER_TO_METER (ODOM_WHEEL_CIRCUMFERENCE / (float)ODOM_ENCODER_RANGE)
#define ODOM_VESC_TACHOMETER_SCALE 6.0f
#define ODOM_VESC_TACHOMETER_TO_METER (ODOM_WHEEL_CIRCUMFERENCE / (ODOM_VESC_TACHOMETER_SCALE * VESC_M3508_POLE_PAIRS))

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
static uint16_t odom_last_ecd[4];
static int32_t odom_last_tachometer[4];
static uint8_t odom_encoder_ready = 0U;

static void CalculateWheelLinearSpeed(const chassis_move_t *chassis, float *left_speed, float *right_speed);
static float CalculateEncoderDeltaDistance(const chassis_move_t *chassis);
static void UpdateIMUBias(fp32 raw_imu_accel, float aver_v);
static void UpdateOdometry(const chassis_move_t *chassis, float vlb, float vrb, float ds);
static uint8_t IsChassisStill(float vx, float wz, float accel);
static void ZeroVelocityEstimate(void);
static float WrapPi(float angle);
static int16_t EncoderDelta(uint16_t now, uint16_t last);
static float RobustWheelDistance(float wheel_ds[4]);

static const float odom_wheel_direction[4] = {
    CHASSIS_MOTOR_1_FORWARD_SIGN,
    CHASSIS_MOTOR_2_FORWARD_SIGN,
    CHASSIS_MOTOR_3_FORWARD_SIGN,
    CHASSIS_MOTOR_4_FORWARD_SIGN,
};

void ObserveTask(void const *argument)
{
    (void)argument;

    while (!is_chassis_init_done())
    {
        osDelay(1);
    }

    local_chassis_move = get_chassis_control_point();
    xvEstimateKF_Init(&vaEstimateKF);

    float vrb = 0.0f;
    float vlb = 0.0f;
    fp32 raw_imu_accel = 0.0f;
    fp32 raw_imu_gyro_z = 0.0f;
    fp32 compensated_accel = 0.0f;
    TickType_t systick = 0;

    for (;;)
    {
        systick = osKernelSysTick();

        CalculateWheelLinearSpeed(local_chassis_move, &vlb, &vrb);

        aver_v = (vrb + vlb) / 2.0f;
        float encoder_ds = CalculateEncoderDeltaDistance(local_chassis_move);
        raw_imu_gyro_z = *(local_chassis_move->chassis_imu_gyro + INS_GYRO_Z_ADDRESS_OFFSET);
        const fp32 wheel_wz = (vrb - vlb) / (2.0f * MOTOR_DISTANCE_TO_CENTER);
        diff_v = wheel_wz - raw_imu_gyro_z;

        if (fabsf(diff_v) > ODOM_SLIP_WZ_VALID_THRESHOLD)
        {
            vaEstimateKF.Data.R[0] = 5000.0f;
        }
        else
        {
            vaEstimateKF.Data.R[0] = 250.0f;
        }

        raw_imu_accel = *(local_chassis_move->chassis_imu_accel + INS_ACCEL_X_ADDRESS_OFFSET);
        UpdateIMUBias(raw_imu_accel, aver_v);
        compensated_accel = raw_imu_accel + imu_bias_estimate;

        xvEstimateKF_Update(&vaEstimateKF, compensated_accel, aver_v);

        uint8_t chassis_still = IsChassisStill(aver_v, raw_imu_gyro_z, compensated_accel) &&
                                (fabsf(encoder_ds) < ODOM_STILL_DS_THRESHOLD);
        if (chassis_still)
        {
            ZeroVelocityEstimate();
        }

        v_real = vel_acc[0];
        if (chassis_still)
        {
            encoder_ds = 0.0f;
        }
        UpdateOdometry(local_chassis_move, vlb, vrb, encoder_ds);

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

static float CalculateEncoderDeltaDistance(const chassis_move_t *chassis)
{
    if (chassis == NULL)
    {
        return 0.0f;
    }

    if (odom_encoder_ready == 0U)
    {
        for (uint8_t i = 0; i < 4U; i++)
        {
#if CHASSIS_ESC_PROTOCOL == CHASSIS_ESC_PROTOCOL_VESC
            odom_last_tachometer[i] = chassis->chassis_motor[i].chassis_motor_measure->tachometer;
#else
            odom_last_ecd[i] = chassis->chassis_motor[i].chassis_motor_measure->ecd;
#endif
        }
        odom_encoder_ready = 1U;
        return 0.0f;
    }

    float wheel_ds[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (uint8_t i = 0; i < 4U; i++)
    {
#if CHASSIS_ESC_PROTOCOL == CHASSIS_ESC_PROTOCOL_VESC
        const int32_t now_tachometer = chassis->chassis_motor[i].chassis_motor_measure->tachometer;
        const int32_t delta_tachometer = now_tachometer - odom_last_tachometer[i];
        odom_last_tachometer[i] = now_tachometer;
        wheel_ds[i] = (float)delta_tachometer * ODOM_VESC_TACHOMETER_TO_METER * odom_wheel_direction[i];
#else
        const uint16_t now_ecd = chassis->chassis_motor[i].chassis_motor_measure->ecd;
        const int16_t delta_ecd = EncoderDelta(now_ecd, odom_last_ecd[i]);
        odom_last_ecd[i] = now_ecd;
        wheel_ds[i] = (float)delta_ecd * ODOM_ENCODER_TO_METER * odom_wheel_direction[i];
#endif
    }

    return RobustWheelDistance(wheel_ds);
}

static void UpdateIMUBias(fp32 raw_imu_accel, float aver_v_current)
{
    if (fabsf(aver_v_current) < 0.01f && fabsf(raw_imu_accel + imu_bias_estimate) < 0.5f)
    {
        imu_bias_estimate = imu_bias_estimate * 0.999f + (-raw_imu_accel) * 0.001f;
    }
}

static void UpdateOdometry(const chassis_move_t *chassis, float vlb, float vrb, float ds)
{
    if (chassis == NULL || chassis->chassis_imu_gyro == NULL)
    {
        return;
    }

    const float wz_imu = *(chassis->chassis_imu_gyro + INS_GYRO_Z_ADDRESS_OFFSET);
    const float wheel_wz = (vrb - vlb) / (2.0f * MOTOR_DISTANCE_TO_CENTER);
    const float raw_vx = ds / OBSERVE_TASK_PERIOD_S;
    const float odom_vx = (fabsf(raw_vx) < ODOM_VX_DEADBAND) ? 0.0f : raw_vx;
    const float odom_wz = (fabsf(wz_imu) < ODOM_WZ_DEADBAND) ? 0.0f : wz_imu;
    const float odom_ds = (fabsf(raw_vx) < ODOM_VX_DEADBAND) ? 0.0f : ds;

    diff_v = wheel_wz - wz_imu;

    chassis_odom.wz = odom_wz;
    chassis_odom.vx = odom_vx;
    chassis_odom.slip = diff_v;
    chassis_odom.valid = (fabsf(diff_v) < ODOM_SLIP_WZ_VALID_THRESHOLD) ? 1U : 0U;

    chassis_odom.yaw = WrapPi(chassis_odom.yaw + odom_wz * OBSERVE_TASK_PERIOD_S);
    chassis_odom.x += odom_ds * cosf(chassis_odom.yaw);
    chassis_odom.y += odom_ds * sinf(chassis_odom.yaw);
    chassis_odom.distance += odom_ds;
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

static int16_t EncoderDelta(uint16_t now, uint16_t last)
{
    int16_t delta = (int16_t)(now - last);
    if (delta > (ODOM_ENCODER_RANGE / 2))
    {
        delta -= ODOM_ENCODER_RANGE;
    }
    else if (delta < -(ODOM_ENCODER_RANGE / 2))
    {
        delta += ODOM_ENCODER_RANGE;
    }
    return delta;
}

static float RobustWheelDistance(float wheel_ds[4])
{
    float sorted[4] = {
        wheel_ds[0],
        wheel_ds[1],
        wheel_ds[2],
        wheel_ds[3],
    };

    for (uint8_t i = 0U; i < 3U; i++)
    {
        for (uint8_t j = (uint8_t)(i + 1U); j < 4U; j++)
        {
            if (sorted[j] < sorted[i])
            {
                const float tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }

    return (sorted[1] + sorted[2]) / 2.0f;
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
    odom_encoder_ready = 0U;
    ZeroVelocityEstimate();
}
