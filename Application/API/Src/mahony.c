//
// Created by kun on 2026/7/31.
//

#include "mahony.h"
#include <math.h>
#include <string.h>

#define MAHONY_DEFAULT_DT_S           0.001f
#define MAHONY_ACCEL_NORM_MIN         7.0f
#define MAHONY_ACCEL_NORM_MAX         12.5f
#define MAHONY_EPSILON                1e-6f
#define MAHONY_HALF_PI                1.57079632679f

static inline float mahony_limit(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

static void quaternionNormalize(float *q0, float *q1, float *q2, float *q3)
{
    const float norm2 = (*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3);
    if (norm2 < MAHONY_EPSILON)
    {
        *q0 = 1.0f;
        *q1 = 0.0f;
        *q2 = 0.0f;
        *q3 = 0.0f;
        return;
    }

    const float inv_norm = 1.0f / sqrtf(norm2);
    *q0 *= inv_norm;
    *q1 *= inv_norm;
    *q2 *= inv_norm;
    *q3 *= inv_norm;
}

void Mahony_Init(Mahony_Instance_t *inst,
                 float Kp_accel, float Ki_accel,
                 float Kp_mag, float Ki_mag,
                 float mag_weight)
{
    if (inst == NULL)
    {
        return;
    }

    memset(inst, 0, sizeof(Mahony_Instance_t));

    inst->params.Kp_accel = (Kp_accel > 0.0f) ? Kp_accel : 1.0f;
    inst->params.Ki_accel = (Ki_accel > 0.0f) ? Ki_accel : 0.05f;
    inst->params.Kp_mag = (Kp_mag > 0.0f) ? Kp_mag : 0.5f;
    inst->params.Ki_mag = (Ki_mag > 0.0f) ? Ki_mag : 0.01f;
    inst->params.mag_weight = (mag_weight >= 0.0f && mag_weight <= 1.0f) ? mag_weight : 0.5f;
    inst->params.integral_limit = 2.0f;
    inst->params.dt_min = MAHONY_DEFAULT_DT_S;
    inst->params.dt_max = 0.05f;
    inst->dt_filtered = MAHONY_DEFAULT_DT_S;

    inst->att.q0 = 1.0f;
    inst->att.q1 = 0.0f;
    inst->att.q2 = 0.0f;
    inst->att.q3 = 0.0f;

    inst->mag_ref[0] = 1.0f;
    inst->mag_ref[1] = 0.0f;
    inst->mag_ref[2] = 0.0f;
    inst->is_initialized = false;
    inst->mag_initialized = false;
    inst->update_count = 0U;
}

void Mahony_Reset(Mahony_Instance_t *inst)
{
    if (inst == NULL)
    {
        return;
    }

    inst->att.q0 = 1.0f;
    inst->att.q1 = 0.0f;
    inst->att.q2 = 0.0f;
    inst->att.q3 = 0.0f;
    inst->att.roll = 0.0f;
    inst->att.pitch = 0.0f;
    inst->att.yaw = 0.0f;

    memset(inst->att.q_deriv, 0, sizeof(inst->att.q_deriv));
    memset(inst->att.omega_corrected, 0, sizeof(inst->att.omega_corrected));
    memset(inst->integralFB_accel, 0, sizeof(inst->integralFB_accel));
    memset(inst->integralFB_mag, 0, sizeof(inst->integralFB_mag));

    inst->is_initialized = false;
    inst->update_count = 0U;
}

void Mahony_SetMagReference(Mahony_Instance_t *inst, float ref_x, float ref_y, float ref_z)
{
    if (inst == NULL)
    {
        return;
    }

    const float norm = sqrtf(ref_x * ref_x + ref_y * ref_y + ref_z * ref_z);
    if (norm <= MAHONY_EPSILON)
    {
        inst->mag_initialized = false;
        return;
    }

    inst->mag_ref[0] = ref_x / norm;
    inst->mag_ref[1] = ref_y / norm;
    inst->mag_ref[2] = ref_z / norm;
    inst->mag_initialized = true;
}

void Mahony_Update(Mahony_Instance_t *inst, const Mahony_ImuData_t *data)
{
    if (inst == NULL || data == NULL || data->valid == 0U)
    {
        return;
    }

    float dt = data->dt;
    dt = mahony_limit(dt, inst->params.dt_min, inst->params.dt_max);
    inst->dt_filtered = dt;

    float q0 = inst->att.q0;
    float q1 = inst->att.q1;
    float q2 = inst->att.q2;
    float q3 = inst->att.q3;

    float gx = data->gyro[0];
    float gy = data->gyro[1];
    float gz = data->gyro[2];
    float ax = data->accel[0];
    float ay = data->accel[1];
    float az = data->accel[2];
    float mx = data->mag[0];
    float my = data->mag[1];
    float mz = data->mag[2];

    float ex_accel = 0.0f;
    float ey_accel = 0.0f;
    float ez_accel = 0.0f;

    const float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (accel_norm >= MAHONY_ACCEL_NORM_MIN && accel_norm <= MAHONY_ACCEL_NORM_MAX)
    {
        const float inv_accel_norm = 1.0f / accel_norm;
        ax *= inv_accel_norm;
        ay *= inv_accel_norm;
        az *= inv_accel_norm;

        const float vx = 2.0f * (q1 * q3 - q0 * q2);
        const float vy = 2.0f * (q0 * q1 + q2 * q3);
        const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        ex_accel = ay * vz - az * vy;
        ey_accel = az * vx - ax * vz;
        ez_accel = ax * vy - ay * vx;

        inst->integralFB_accel[0] += ex_accel * inst->params.Ki_accel * dt;
        inst->integralFB_accel[1] += ey_accel * inst->params.Ki_accel * dt;
        inst->integralFB_accel[2] += ez_accel * inst->params.Ki_accel * dt;
    }

    float ex_mag = 0.0f;
    float ey_mag = 0.0f;
    float ez_mag = 0.0f;

    const float mag_norm = sqrtf(mx * mx + my * my + mz * mz);
    if (mag_norm > MAHONY_EPSILON && inst->mag_initialized)
    {
        const float inv_mag_norm = 1.0f / mag_norm;
        mx *= inv_mag_norm;
        my *= inv_mag_norm;
        mz *= inv_mag_norm;

        const float bx = inst->mag_ref[0];
        const float by = inst->mag_ref[1];
        const float bz = inst->mag_ref[2];

        const float wx = 2.0f * (bx * (0.5f - q2 * q2 - q3 * q3) +
                                 by * (q1 * q2 - q0 * q3) +
                                 bz * (q1 * q3 + q0 * q2));
        const float wy = 2.0f * (bx * (q1 * q2 + q0 * q3) +
                                 by * (0.5f - q1 * q1 - q3 * q3) +
                                 bz * (q2 * q3 - q0 * q1));
        const float wz = 2.0f * (bx * (q1 * q3 - q0 * q2) +
                                 by * (q2 * q3 + q0 * q1) +
                                 bz * (0.5f - q1 * q1 - q2 * q2));

        ex_mag = inst->params.mag_weight * (my * wz - mz * wy);
        ey_mag = inst->params.mag_weight * (mz * wx - mx * wz);
        ez_mag = inst->params.mag_weight * (mx * wy - my * wx);

        inst->integralFB_mag[0] += ex_mag * inst->params.Ki_mag * dt;
        inst->integralFB_mag[1] += ey_mag * inst->params.Ki_mag * dt;
        inst->integralFB_mag[2] += ez_mag * inst->params.Ki_mag * dt;
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        inst->integralFB_accel[i] = mahony_limit(inst->integralFB_accel[i],
                                                 -inst->params.integral_limit,
                                                 inst->params.integral_limit);
        inst->integralFB_mag[i] = mahony_limit(inst->integralFB_mag[i],
                                               -inst->params.integral_limit,
                                               inst->params.integral_limit);
    }

    const float omega_corr_x = inst->params.Kp_accel * ex_accel + inst->integralFB_accel[0] +
                               inst->params.Kp_mag * ex_mag + inst->integralFB_mag[0];
    const float omega_corr_y = inst->params.Kp_accel * ey_accel + inst->integralFB_accel[1] +
                               inst->params.Kp_mag * ey_mag + inst->integralFB_mag[1];
    const float omega_corr_z = inst->params.Kp_accel * ez_accel + inst->integralFB_accel[2] +
                               inst->params.Kp_mag * ez_mag + inst->integralFB_mag[2];

    gx += omega_corr_x;
    gy += omega_corr_y;
    gz += omega_corr_z;

    inst->att.omega_corrected[0] = gx;
    inst->att.omega_corrected[1] = gy;
    inst->att.omega_corrected[2] = gz;

    const float q0_dot = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    const float q1_dot = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    const float q2_dot = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    const float q3_dot = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    inst->att.q_deriv[0] = q0_dot;
    inst->att.q_deriv[1] = q1_dot;
    inst->att.q_deriv[2] = q2_dot;
    inst->att.q_deriv[3] = q3_dot;

    q0 += q0_dot * dt;
    q1 += q1_dot * dt;
    q2 += q2_dot * dt;
    q3 += q3_dot * dt;

    quaternionNormalize(&q0, &q1, &q2, &q3);

    inst->att.q0 = q0;
    inst->att.q1 = q1;
    inst->att.q2 = q2;
    inst->att.q3 = q3;

    inst->is_initialized = true;
    inst->update_count++;
}

void Mahony_UpdateEuler(Mahony_Instance_t *inst)
{
    if (inst == NULL)
    {
        return;
    }

    const float q0 = inst->att.q0;
    const float q1 = inst->att.q1;
    const float q2 = inst->att.q2;
    const float q3 = inst->att.q3;

    inst->att.roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                            q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

    const float sinp = 2.0f * (q0 * q2 - q1 * q3);
    if (sinp >= 1.0f)
    {
        inst->att.pitch = MAHONY_HALF_PI;
    }
    else if (sinp <= -1.0f)
    {
        inst->att.pitch = -MAHONY_HALF_PI;
    }
    else
    {
        inst->att.pitch = asinf(sinp);
    }

    inst->att.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                           q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
}
