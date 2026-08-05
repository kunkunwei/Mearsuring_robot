#ifndef MAGNETIC_HEADING_H
#define MAGNETIC_HEADING_H

#include <stdint.h>

typedef struct
{
    float bias_ut[3];
    float soft_iron[3][3];
    uint8_t axis_source[3];
    int8_t axis_sign[3];
} MagneticCalibration_t;

typedef struct
{
    MagneticCalibration_t calibration;
    float field_norm_min_ut;
    float field_norm_max_ut;
    float field_norm_tolerance_ratio;
    float heading_innovation_limit_rad;
    float correction_gain;
    float max_correction_rate_rad_s;
    float reference_update_gain;
} MagneticHeadingConfig_t;

typedef struct
{
    MagneticHeadingConfig_t config;
    float reference_norm_ut;
    float yaw_offset_rad;
    uint8_t norm_ready;
    uint8_t alignment_ready;
} MagneticHeadingEstimator_t;

typedef struct
{
    float raw_mag_ut[3];
    float roll_rad;
    float pitch_rad;
    float predicted_yaw_rad;
    float dt_s;
    uint8_t sample_valid;
} MagneticHeadingInput_t;

typedef struct
{
    float body_mag_ut[3];
    float field_norm_ut;
    float magnetic_yaw_rad;
    float aligned_yaw_rad;
    float heading_innovation_rad;
    float corrected_yaw_rad;
    float quality;
    uint8_t trusted;
} MagneticHeadingOutput_t;

typedef struct
{
    float min_ut[3];
    float max_ut[3];
    uint32_t sample_count;
} MagneticCalibrationCapture_t;

void MagneticHeading_Init(MagneticHeadingEstimator_t *estimator,
                          const MagneticHeadingConfig_t *config);
void MagneticHeading_Reset(MagneticHeadingEstimator_t *estimator);
void MagneticHeading_Update(MagneticHeadingEstimator_t *estimator,
                            const MagneticHeadingInput_t *input,
                            MagneticHeadingOutput_t *output);

void MagneticCalibrationCapture_Begin(MagneticCalibrationCapture_t *capture);
void MagneticCalibrationCapture_Add(MagneticCalibrationCapture_t *capture,
                                    const float raw_mag_ut[3]);
uint8_t MagneticCalibrationCapture_Finish(const MagneticCalibrationCapture_t *capture,
                                          MagneticCalibration_t *calibration);

#endif
