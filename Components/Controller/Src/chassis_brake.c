#include "chassis_brake.h"

#include <stddef.h>

static float brake_limit(float value, float min_value, float max_value)
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

float Chassis_Brake_Update(const Chassis_Brake_Config_t *config,
                           float speed_rpm,
                           Chassis_Brake_Output_t *output)
{
    if (config == NULL || output == NULL)
    {
        return 0.0f;
    }

    output->speed_current_a = -config->speed_gain_a_per_rpm * speed_rpm;
    output->raw_current_a = brake_limit(output->speed_current_a,
                                        -config->current_limit_a,
                                        config->current_limit_a);
    return output->raw_current_a;
}
