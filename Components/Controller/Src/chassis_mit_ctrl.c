#include "chassis_mit_ctrl.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float mit_limit(float value, float min_value, float max_value)
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

void Chassis_Mit_Reset(Chassis_Mit_State_t *state, float position_deg)
{
    if (state == NULL)
    {
        return;
    }

    state->position_ref_deg = position_deg;
    state->initialized = 1U;
}

float Chassis_Mit_Update(Chassis_Mit_State_t *state,
                         const Chassis_Mit_Config_t *config,
                         const Chassis_Mit_Input_t *input,
                         float dt_s,
                         Chassis_Mit_Output_t *output)
{
    if (state == NULL || config == NULL || input == NULL || output == NULL || dt_s <= 0.0f)
    {
        return 0.0f;
    }

    memset(output, 0, sizeof(*output));
    if (state->initialized == 0U)
    {
        Chassis_Mit_Reset(state, input->position_deg);
    }

    state->position_ref_deg += input->target_rpm * 6.0f * dt_s;
    output->position_error_deg = mit_limit(state->position_ref_deg - input->position_deg,
                                           -config->position_error_limit_deg,
                                           config->position_error_limit_deg);
    state->position_ref_deg = input->position_deg + output->position_error_deg;

    output->position_current_a = config->position_kp_a_per_deg * output->position_error_deg;
    output->speed_current_a = config->speed_kd_a_per_rpm *
                              (input->target_rpm - input->speed_rpm);
    if (config->friction_rpm_scale > 0.0f)
    {
        output->friction_current_a = config->friction_current_a *
                                     tanhf(input->target_rpm / config->friction_rpm_scale);
    }

    output->raw_current_a = mit_limit(output->position_current_a +
                                      output->speed_current_a +
                                      output->friction_current_a,
                                      -config->current_limit_a,
                                      config->current_limit_a);
    return output->raw_current_a;
}
