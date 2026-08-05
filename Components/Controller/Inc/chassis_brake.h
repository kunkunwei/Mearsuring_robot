#ifndef CHASSIS_BRAKE_H
#define CHASSIS_BRAKE_H

typedef struct
{
    float speed_gain_a_per_rpm;
    float current_limit_a;
} Chassis_Brake_Config_t;

typedef struct
{
    float speed_current_a;
    float raw_current_a;
} Chassis_Brake_Output_t;

float Chassis_Brake_Update(const Chassis_Brake_Config_t *config,
                           float speed_rpm,
                           Chassis_Brake_Output_t *output);

#endif
