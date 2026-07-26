#include "odometry.h"
#include <math.h>
#include <string.h>

static const OdomConfig_t odom_default_config = {
    .wheel_radius_m = 0.025f,
    .wheel_track_m = 0.081f,
    .straight_wz_threshold = 0.08f,
    .still_vx_threshold = 0.004f,
    .still_wz_threshold = 0.015f,
    .slope_pitch_threshold = 0.10f,
    .wheel_speed_outlier_ratio = 2.0f,
    .wheel_speed_outlier_offset = 0.08f,
    .max_wheel_delta_m = 0.08f,
    .imu_yaw_turn_weight = 0.75f,
    .imu_yaw_straight_weight = 0.20f,
    .straight_pair = ODOM_STRAIGHT_PAIR_REAR,
};

static float odom_abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float odom_constrain(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float odom_wrap_pi(float angle)
{
    const float pi = 3.14159265358979323846f;

    while (angle > pi)
    {
        angle -= 2.0f * pi;
    }
    while (angle < -pi)
    {
        angle += 2.0f * pi;
    }
    return angle;
}

static float odom_weighted_average(float a, float wa, float b, float wb)
{
    const float weight_sum = wa + wb;
    if (weight_sum <= 0.0001f)
    {
        return 0.0f;
    }
    return (a * wa + b * wb) / weight_sum;
}

static uint8_t odom_axle_distance(const float wheel_ds[4],
                                  const float weight[4],
                                  uint8_t left_index,
                                  uint8_t right_index,
                                  float *distance)
{
    if (weight[left_index] <= 0.0001f || weight[right_index] <= 0.0001f || distance == NULL)
    {
        return 0U;
    }

    *distance = (wheel_ds[left_index] + wheel_ds[right_index]) * 0.5f;
    return 1U;
}

static float odom_choose_straight_distance(const OdomEstimator_t *estimator,
                                           const float wheel_ds[4],
                                           const float weight[4],
                                           float fallback_distance)
{
    float front_distance = 0.0f;
    float rear_distance = 0.0f;
    const uint8_t front_valid = odom_axle_distance(wheel_ds, weight, 0U, 1U, &front_distance);
    const uint8_t rear_valid = odom_axle_distance(wheel_ds, weight, 3U, 2U, &rear_distance);

    if (front_valid == 0U && rear_valid == 0U)
    {
        return fallback_distance;
    }
    if (front_valid == 0U)
    {
        return rear_distance;
    }
    if (rear_valid == 0U)
    {
        return front_distance;
    }

    return (estimator->config.straight_pair == ODOM_STRAIGHT_PAIR_FRONT) ? front_distance : rear_distance;
}

static OdomMotionMode_t odom_classify_motion(const OdomEstimator_t *estimator, const OdomInput_t *input)
{
    float mean_abs_wheel_speed = 0.0f;
    uint8_t online_count = 0U;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        if (input->wheel[i].online != 0U)
        {
            mean_abs_wheel_speed += odom_abs(input->wheel[i].speed_mps);
            online_count++;
        }
    }

    if (online_count > 0U)
    {
        mean_abs_wheel_speed /= (float)online_count;
    }

    if (odom_abs(input->command_vx_mps) < estimator->config.still_vx_threshold &&
        odom_abs(input->command_wz_rad_s) < estimator->config.still_wz_threshold &&
        mean_abs_wheel_speed < estimator->config.still_vx_threshold &&
        odom_abs(input->imu.gyro_z_rad_s) < estimator->config.still_wz_threshold)
    {
        return ODOM_MOTION_STILL;
    }

    if (odom_abs(input->imu.pitch_rad) > estimator->config.slope_pitch_threshold &&
        odom_abs(input->command_wz_rad_s) < estimator->config.straight_wz_threshold)
    {
        return ODOM_MOTION_SLOPE;
    }

    if (odom_abs(input->command_wz_rad_s) >= estimator->config.straight_wz_threshold)
    {
        return ODOM_MOTION_TURN;
    }

    return ODOM_MOTION_STRAIGHT;
}

static void odom_init_output(OdomOutput_t *output)
{
    memset(output, 0, sizeof(*output));
    output->valid = 0U;
}

static void odom_update_wheel_delta(OdomEstimator_t *estimator,
                                    const OdomInput_t *input,
                                    float wheel_ds[4],
                                    uint8_t delta_valid[4])
{
    const float meter_per_degree = 2.0f * 3.14159265358979323846f *
                                   estimator->config.wheel_radius_m / 360.0f;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        wheel_ds[i] = 0.0f;
        delta_valid[i] = 0U;

        if (input->wheel[i].pos_ready == 0U)
        {
            estimator->wheel_ready[i] = 0U;
            continue;
        }

        if (estimator->wheel_ready[i] == 0U)
        {
            estimator->last_pos_deg[i] = input->wheel[i].pos_deg;
            estimator->wheel_ready[i] = 1U;
            continue;
        }

        const float delta_deg = input->wheel[i].pos_deg - estimator->last_pos_deg[i];
        wheel_ds[i] = delta_deg * meter_per_degree;
        delta_valid[i] = 1U;
    }
}

static void odom_commit_wheel_position(OdomEstimator_t *estimator, const OdomInput_t *input)
{
    for (uint8_t i = 0U; i < 4U; i++)
    {
        if (input->wheel[i].pos_ready != 0U && estimator->wheel_ready[i] != 0U)
        {
            estimator->last_pos_deg[i] = input->wheel[i].pos_deg;
        }
    }
}

static void odom_update_origin_distance(OdomEstimator_t *estimator, const OdomInput_t *input, OdomOutput_t *output)
{
    const float meter_per_degree = 2.0f * 3.14159265358979323846f *
                                   estimator->config.wheel_radius_m / 360.0f;
    float pos_from_origin[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t ready[4] = {0U, 0U, 0U, 0U};

    for (uint8_t i = 0U; i < 4U; i++)
    {
        if (input->wheel[i].pos_ready == 0U)
        {
            estimator->origin_ready[i] = 0U;
            continue;
        }

        if (estimator->origin_ready[i] == 0U)
        {
            estimator->origin_pos_deg[i] = input->wheel[i].pos_deg;
            estimator->origin_ready[i] = 1U;
        }

        pos_from_origin[i] = (input->wheel[i].pos_deg - estimator->origin_pos_deg[i]) * meter_per_degree;
        ready[i] = 1U;
    }

    if (ready[0] != 0U && ready[1] != 0U)
    {
        output->front_distance_m = (pos_from_origin[0] + pos_from_origin[1]) * 0.5f;
    }
    if (ready[3] != 0U && ready[2] != 0U)
    {
        output->rear_distance_m = (pos_from_origin[3] + pos_from_origin[2]) * 0.5f;
    }
    if (ready[0] != 0U && ready[3] != 0U)
    {
        output->left_distance_m = (pos_from_origin[0] + pos_from_origin[3]) * 0.5f;
    }
    if (ready[1] != 0U && ready[2] != 0U)
    {
        output->right_distance_m = (pos_from_origin[1] + pos_from_origin[2]) * 0.5f;
    }
}

static void odom_update_wheel_weights(const OdomEstimator_t *estimator,
                                      const OdomInput_t *input,
                                      const float wheel_ds[4],
                                      const uint8_t delta_valid[4],
                                      OdomMotionMode_t motion_mode,
                                      float weight[4])
{
    float mean_abs_speed = 0.0f;
    uint8_t online_count = 0U;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        if (input->wheel[i].online != 0U)
        {
            mean_abs_speed += odom_abs(input->wheel[i].speed_mps);
            online_count++;
        }
    }

    if (online_count > 0U)
    {
        mean_abs_speed /= (float)online_count;
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        weight[i] = (delta_valid[i] != 0U) ? 1.0f : 0.0f;

        if (weight[i] <= 0.0f)
        {
            continue;
        }

        const float abs_speed = odom_abs(input->wheel[i].speed_mps);
        const float speed_limit = mean_abs_speed * estimator->config.wheel_speed_outlier_ratio +
                                  estimator->config.wheel_speed_outlier_offset;
        if (abs_speed > speed_limit)
        {
            weight[i] *= 0.20f;
        }

        if (odom_abs(wheel_ds[i]) > estimator->config.max_wheel_delta_m)
        {
            weight[i] = 0.0f;
        }
    }

    if (motion_mode == ODOM_MOTION_SLOPE)
    {
        weight[0] *= 0.35f;
        weight[1] *= 0.35f;
        weight[2] *= 1.20f;
        weight[3] *= 1.20f;
    }
}

void OdomEstimator_Init(OdomEstimator_t *estimator, const OdomConfig_t *config)
{
    if (estimator == NULL)
    {
        return;
    }

    memset(estimator, 0, sizeof(*estimator));
    estimator->config = (config != NULL) ? *config : odom_default_config;
}

void OdomEstimator_Reset(OdomEstimator_t *estimator)
{
    if (estimator == NULL)
    {
        return;
    }

    memset(estimator->last_pos_deg, 0, sizeof(estimator->last_pos_deg));
    memset(estimator->origin_pos_deg, 0, sizeof(estimator->origin_pos_deg));
    memset(estimator->wheel_ready, 0, sizeof(estimator->wheel_ready));
    memset(estimator->origin_ready, 0, sizeof(estimator->origin_ready));
    estimator->last_yaw_rad = 0.0f;
    estimator->yaw_ready = 0U;
}

void OdomEstimator_Update(OdomEstimator_t *estimator, const OdomInput_t *input, OdomOutput_t *output)
{
    if (estimator == NULL || input == NULL || output == NULL || input->dt_s <= 0.0f)
    {
        return;
    }

    odom_init_output(output);

    float wheel_ds[4];
    uint8_t delta_valid[4];
    odom_update_wheel_delta(estimator, input, wheel_ds, delta_valid);
    odom_update_origin_distance(estimator, input, output);

    output->motion_mode = odom_classify_motion(estimator, input);
    if (output->motion_mode == ODOM_MOTION_STILL)
    {
        float max_abs_ds = 0.0f;
        for (uint8_t i = 0U; i < 4U; i++)
        {
            if (delta_valid[i] != 0U && odom_abs(wheel_ds[i]) > max_abs_ds)
            {
                max_abs_ds = odom_abs(wheel_ds[i]);
            }
        }

        if (max_abs_ds > estimator->config.still_vx_threshold * input->dt_s)
        {
            output->motion_mode = (odom_abs(input->command_wz_rad_s) >= estimator->config.straight_wz_threshold ||
                                   odom_abs(input->imu.gyro_z_rad_s) >= estimator->config.straight_wz_threshold) ?
                                  ODOM_MOTION_TURN :
                                  ODOM_MOTION_STRAIGHT;
        }
    }
    odom_update_wheel_weights(estimator, input, wheel_ds, delta_valid, output->motion_mode, output->wheel_weight);

    const float left_ds = odom_weighted_average(wheel_ds[0], output->wheel_weight[0],
                                                wheel_ds[3], output->wheel_weight[3]);
    const float right_ds = odom_weighted_average(wheel_ds[1], output->wheel_weight[1],
                                                 wheel_ds[2], output->wheel_weight[2]);
    const float left_weight = output->wheel_weight[0] + output->wheel_weight[3];
    const float right_weight = output->wheel_weight[1] + output->wheel_weight[2];

    if (left_weight <= 0.0001f || right_weight <= 0.0001f)
    {
        output->valid = 0U;
        return;
    }

    float wheel_ds_body = (left_ds + right_ds) * 0.5f;
    const float wheel_dtheta = (right_ds - left_ds) / estimator->config.wheel_track_m;

    if (output->motion_mode == ODOM_MOTION_STRAIGHT)
    {
        wheel_ds_body = odom_choose_straight_distance(estimator, wheel_ds, output->wheel_weight, wheel_ds_body);
    }
    else if (output->motion_mode == ODOM_MOTION_SLOPE)
    {
        float rear_distance = 0.0f;
        if (odom_axle_distance(wheel_ds, output->wheel_weight, 3U, 2U, &rear_distance) != 0U)
        {
            wheel_ds_body = rear_distance;
        }
    }

    float imu_dtheta = 0.0f;
    uint8_t imu_delta_valid = 0U;
    if (input->imu.yaw_ready != 0U)
    {
        if (estimator->yaw_ready == 0U)
        {
            estimator->last_yaw_rad = input->imu.yaw_rad;
            estimator->yaw_ready = 1U;
        }
        else
        {
            imu_dtheta = odom_wrap_pi(input->imu.yaw_rad - estimator->last_yaw_rad);
            estimator->last_yaw_rad = input->imu.yaw_rad;
            imu_delta_valid = 1U;
        }
    }
    else
    {
        estimator->yaw_ready = 0U;
    }

    float yaw_weight = (output->motion_mode == ODOM_MOTION_TURN) ?
                       estimator->config.imu_yaw_turn_weight :
                       estimator->config.imu_yaw_straight_weight;
    yaw_weight = odom_constrain(yaw_weight, 0.0f, 1.0f);

    if (imu_delta_valid != 0U)
    {
        output->dtheta_rad = wheel_dtheta * (1.0f - yaw_weight) + imu_dtheta * yaw_weight;
        output->slip_wz_rad_s = (wheel_dtheta - imu_dtheta) / input->dt_s;
    }
    else
    {
        output->dtheta_rad = wheel_dtheta;
        output->slip_wz_rad_s = wheel_dtheta / input->dt_s - input->imu.gyro_z_rad_s;
    }

    if (output->motion_mode == ODOM_MOTION_TURN && odom_abs(input->command_vx_mps) < estimator->config.still_vx_threshold)
    {
        output->ds_m = 0.0f;
    }
    else
    {
        output->ds_m = wheel_ds_body;
    }

    if (output->motion_mode == ODOM_MOTION_STILL)
    {
        output->ds_m = 0.0f;
        output->dtheta_rad = 0.0f;
    }

    odom_commit_wheel_position(estimator, input);

    output->vx_mps = output->ds_m / input->dt_s;
    output->wz_rad_s = output->dtheta_rad / input->dt_s;
    output->valid = 1U;
}
