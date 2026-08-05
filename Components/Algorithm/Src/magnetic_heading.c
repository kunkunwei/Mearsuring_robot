#include "magnetic_heading.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define MAG_HEADING_PI 3.14159265358979323846f
#define MAG_HEADING_EPSILON 1e-6f

static float mag_abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float mag_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float mag_wrap_pi(float angle)
{
    while (angle > MAG_HEADING_PI)
    {
        angle -= 2.0f * MAG_HEADING_PI;
    }
    while (angle < -MAG_HEADING_PI)
    {
        angle += 2.0f * MAG_HEADING_PI;
    }
    return angle;
}

static uint8_t mag_is_finite(float value)
{
    return isfinite(value) ? 1U : 0U;
}

static float mag_median3(float a, float b, float c)
{
    if (a > b)
    {
        const float temp = a;
        a = b;
        b = temp;
    }
    if (b > c)
    {
        const float temp = b;
        b = c;
        c = temp;
    }
    if (a > b)
    {
        b = a;
    }
    return b;
}

static void mag_apply_calibration(const MagneticCalibration_t *calibration,
                                  const float raw_mag_ut[3],
                                  float body_mag_ut[3])
{
    float centered[3];
    float corrected_sensor[3] = {0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0U; i < 3U; i++)
    {
        centered[i] = raw_mag_ut[i] - calibration->bias_ut[i];
    }

    for (uint8_t row = 0U; row < 3U; row++)
    {
        for (uint8_t column = 0U; column < 3U; column++)
        {
            corrected_sensor[row] += calibration->soft_iron[row][column] * centered[column];
        }
    }

    for (uint8_t body_axis = 0U; body_axis < 3U; body_axis++)
    {
        const uint8_t source = calibration->axis_source[body_axis];
        const int8_t sign = calibration->axis_sign[body_axis];
        if (source > 2U || (sign != 1 && sign != -1))
        {
            body_mag_ut[body_axis] = 0.0f;
        }
        else
        {
            body_mag_ut[body_axis] = corrected_sensor[source] * (float)sign;
        }
    }
}

static float mag_tilt_compensated_yaw(const float body_mag_ut[3], float roll, float pitch)
{
    const float sin_roll = sinf(roll);
    const float cos_roll = cosf(roll);
    const float sin_pitch = sinf(pitch);
    const float cos_pitch = cosf(pitch);
    const float horizontal_x = body_mag_ut[0] * cos_pitch + body_mag_ut[2] * sin_pitch;
    const float horizontal_y = body_mag_ut[0] * sin_roll * sin_pitch +
                               body_mag_ut[1] * cos_roll -
                               body_mag_ut[2] * sin_roll * cos_pitch;
    return atan2f(-horizontal_y, horizontal_x);
}

void MagneticHeading_Init(MagneticHeadingEstimator_t *estimator,
                          const MagneticHeadingConfig_t *config)
{
    if (estimator == NULL || config == NULL)
    {
        return;
    }

    memset(estimator, 0, sizeof(*estimator));
    estimator->config = *config;
}

void MagneticHeading_Reset(MagneticHeadingEstimator_t *estimator)
{
    if (estimator == NULL)
    {
        return;
    }

    estimator->reference_norm_ut = 0.0f;
    estimator->yaw_offset_rad = 0.0f;
    estimator->norm_ready = 0U;
    estimator->alignment_ready = 0U;
}

void MagneticHeading_Update(MagneticHeadingEstimator_t *estimator,
                            const MagneticHeadingInput_t *input,
                            MagneticHeadingOutput_t *output)
{
    if (estimator == NULL || input == NULL || output == NULL)
    {
        return;
    }

    memset(output, 0, sizeof(*output));
    output->corrected_yaw_rad = mag_wrap_pi(input->predicted_yaw_rad);

    if (input->sample_valid == 0U || input->dt_s <= 0.0f ||
        mag_is_finite(input->predicted_yaw_rad) == 0U ||
        mag_is_finite(input->roll_rad) == 0U ||
        mag_is_finite(input->pitch_rad) == 0U)
    {
        return;
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (mag_is_finite(input->raw_mag_ut[i]) == 0U)
        {
            return;
        }
    }

    mag_apply_calibration(&estimator->config.calibration, input->raw_mag_ut, output->body_mag_ut);
    output->field_norm_ut = sqrtf(output->body_mag_ut[0] * output->body_mag_ut[0] +
                                  output->body_mag_ut[1] * output->body_mag_ut[1] +
                                  output->body_mag_ut[2] * output->body_mag_ut[2]);

    if (output->field_norm_ut < estimator->config.field_norm_min_ut ||
        output->field_norm_ut > estimator->config.field_norm_max_ut)
    {
        return;
    }

    if (estimator->norm_ready == 0U)
    {
        estimator->reference_norm_ut = output->field_norm_ut;
        estimator->norm_ready = 1U;
    }
    else
    {
        const float norm_error_ratio = mag_abs(output->field_norm_ut - estimator->reference_norm_ut) /
                                       estimator->reference_norm_ut;
        if (norm_error_ratio > estimator->config.field_norm_tolerance_ratio)
        {
            return;
        }
    }

    output->magnetic_yaw_rad = mag_tilt_compensated_yaw(output->body_mag_ut,
                                                        input->roll_rad,
                                                        input->pitch_rad);
    if (estimator->alignment_ready == 0U)
    {
        estimator->yaw_offset_rad = mag_wrap_pi(input->predicted_yaw_rad - output->magnetic_yaw_rad);
        estimator->alignment_ready = 1U;
    }

    output->aligned_yaw_rad = mag_wrap_pi(output->magnetic_yaw_rad + estimator->yaw_offset_rad);
    output->heading_innovation_rad = mag_wrap_pi(output->aligned_yaw_rad - input->predicted_yaw_rad);
    if (mag_abs(output->heading_innovation_rad) > estimator->config.heading_innovation_limit_rad)
    {
        return;
    }

    const float correction_limit = estimator->config.max_correction_rate_rad_s * input->dt_s;
    const float correction = mag_clamp(estimator->config.correction_gain *
                                       output->heading_innovation_rad * input->dt_s,
                                       -correction_limit,
                                       correction_limit);
    output->corrected_yaw_rad = mag_wrap_pi(input->predicted_yaw_rad + correction);

    const float norm_ratio = mag_abs(output->field_norm_ut - estimator->reference_norm_ut) /
                             estimator->reference_norm_ut;
    const float norm_quality = 1.0f - mag_clamp(norm_ratio /
                                               estimator->config.field_norm_tolerance_ratio,
                                               0.0f,
                                               1.0f);
    const float heading_quality = 1.0f - mag_clamp(mag_abs(output->heading_innovation_rad) /
                                                  estimator->config.heading_innovation_limit_rad,
                                                  0.0f,
                                                  1.0f);
    output->quality = (norm_quality < heading_quality) ? norm_quality : heading_quality;
    output->trusted = 1U;

    const float reference_gain = mag_clamp(estimator->config.reference_update_gain, 0.0f, 1.0f);
    estimator->reference_norm_ut += reference_gain *
                                    (output->field_norm_ut - estimator->reference_norm_ut);
}

void MagneticCalibrationCapture_Begin(MagneticCalibrationCapture_t *capture)
{
    if (capture == NULL)
    {
        return;
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        capture->min_ut[i] = FLT_MAX;
        capture->max_ut[i] = -FLT_MAX;
    }
    capture->sample_count = 0U;
}

void MagneticCalibrationCapture_Add(MagneticCalibrationCapture_t *capture,
                                    const float raw_mag_ut[3])
{
    if (capture == NULL || raw_mag_ut == NULL)
    {
        return;
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (mag_is_finite(raw_mag_ut[i]) == 0U)
        {
            return;
        }
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (raw_mag_ut[i] < capture->min_ut[i])
        {
            capture->min_ut[i] = raw_mag_ut[i];
        }
        if (raw_mag_ut[i] > capture->max_ut[i])
        {
            capture->max_ut[i] = raw_mag_ut[i];
        }
    }
    capture->sample_count++;
}

uint8_t MagneticCalibrationCapture_Finish(const MagneticCalibrationCapture_t *capture,
                                          MagneticCalibration_t *calibration)
{
    if (capture == NULL || calibration == NULL || capture->sample_count < 6U)
    {
        return 0U;
    }

    float radius[3];
    for (uint8_t i = 0U; i < 3U; i++)
    {
        radius[i] = 0.5f * (capture->max_ut[i] - capture->min_ut[i]);
        if (radius[i] <= MAG_HEADING_EPSILON)
        {
            return 0U;
        }
    }

    memset(calibration, 0, sizeof(*calibration));
    const float target_radius = mag_median3(radius[0], radius[1], radius[2]);
    for (uint8_t i = 0U; i < 3U; i++)
    {
        calibration->bias_ut[i] = 0.5f * (capture->max_ut[i] + capture->min_ut[i]);
        calibration->soft_iron[i][i] = target_radius / radius[i];
        calibration->axis_source[i] = i;
        calibration->axis_sign[i] = 1;
    }
    return 1U;
}
