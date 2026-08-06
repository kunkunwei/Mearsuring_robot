#ifndef CHASSIS_MIT_CTRL_H
#define CHASSIS_MIT_CTRL_H

#include <stdint.h>

typedef struct
{
    float position_kp_a_per_deg;
    float speed_kd_a_per_rpm;
    float speed_ki_a_per_rpm_s;
    float speed_integral_limit_a;
    float friction_current_a;
    float friction_rpm_scale;
    float position_error_limit_deg;
    float current_limit_a;
} Chassis_Mit_Config_t;

typedef struct
{
    float position_ref_deg;
    float speed_integral_current_a;
    float last_target_rpm;
    uint8_t initialized;
} Chassis_Mit_State_t;

typedef struct
{
    float target_rpm;
    float speed_rpm;
    float position_deg;
} Chassis_Mit_Input_t;

typedef struct
{
    float position_error_deg;
    float position_current_a;
    float speed_current_a;
    float speed_integral_current_a;
    float friction_current_a;
    float raw_current_a;
} Chassis_Mit_Output_t;

void Chassis_Mit_Reset(Chassis_Mit_State_t *state, float position_deg);
float Chassis_Mit_Update(Chassis_Mit_State_t *state,
                         const Chassis_Mit_Config_t *config,
                         const Chassis_Mit_Input_t *input,
                         float dt_s,
                         Chassis_Mit_Output_t *output);

#endif
