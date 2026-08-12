#ifndef CHASSIS_HOLD_CTRL_H
#define CHASSIS_HOLD_CTRL_H

#include <stdint.h>

#define CHASSIS_HOLD_MOTOR_COUNT 4U

typedef struct
{
    float position_kp_a_per_deg;
    float speed_kd_a_per_rpm;
    float pitch_feedforward_a;
    float pitch_zero_offset_rad;
    // 修正Pitch不超过该角度时，关闭Pitch前馈
    float pitch_feedforward_off_pitch_rad;
    // 修正Pitch达到该角度时，完全启用Pitch前馈
    float pitch_feedforward_full_pitch_rad;
    float current_limit_a;
} Chassis_Hold_Config_t;

typedef struct
{
    float position_ref_deg;
    uint8_t initialized;
} Chassis_Hold_State_t;

typedef struct
{
    float position_deg[CHASSIS_HOLD_MOTOR_COUNT];
    float speed_rpm[CHASSIS_HOLD_MOTOR_COUNT];
    float pitch_rad;
} Chassis_Hold_Input_t;

typedef struct
{
    float mean_position_deg;
    float mean_speed_rpm;
    float position_current_a;
    float speed_current_a;
    float corrected_pitch_rad;
    float pitch_current_a;
    float raw_current_a;
} Chassis_Hold_Output_t;

void Chassis_Hold_Reset(Chassis_Hold_State_t *state);
void Chassis_Hold_Capture(Chassis_Hold_State_t *state,
                          const float position_deg[CHASSIS_HOLD_MOTOR_COUNT]);
float Chassis_Hold_CorrectPitch(const Chassis_Hold_Config_t *config,
                                float raw_pitch_rad);
float Chassis_Hold_ComputePitchFeedforward(const Chassis_Hold_Config_t *config,
                                           float raw_pitch_rad,
                                           float *corrected_pitch_rad);
float Chassis_Hold_Update(Chassis_Hold_State_t *state,
                          const Chassis_Hold_Config_t *config,
                          const Chassis_Hold_Input_t *input,
                          Chassis_Hold_Output_t *output);

#endif
