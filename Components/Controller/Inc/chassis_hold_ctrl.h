#ifndef CHASSIS_HOLD_CTRL_H
#define CHASSIS_HOLD_CTRL_H

#include <stdint.h>

#define CHASSIS_HOLD_MOTOR_COUNT 4U

typedef struct
{
    float position_kp_a_per_deg;
    float speed_kd_a_per_rpm;
    float pitch_feedforward_a;
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
    float pitch_current_a;
    float raw_current_a;
} Chassis_Hold_Output_t;

void Chassis_Hold_Reset(Chassis_Hold_State_t *state);
void Chassis_Hold_Capture(Chassis_Hold_State_t *state,
                          const float position_deg[CHASSIS_HOLD_MOTOR_COUNT]);
float Chassis_Hold_Update(Chassis_Hold_State_t *state,
                          const Chassis_Hold_Config_t *config,
                          const Chassis_Hold_Input_t *input,
                          Chassis_Hold_Output_t *output);

#endif
