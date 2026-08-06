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
    state->speed_integral_current_a = 0.0f;
    state->last_target_rpm = 0.0f;
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

    if (input->slip_limited != 0U)
    {
        state->speed_integral_current_a = 0.0f;
        return 0.0f;
    }

    state->position_ref_deg += input->target_rpm * 6.0f * dt_s;
    output->position_error_deg = mit_limit(state->position_ref_deg - input->position_deg,
                                           -config->position_error_limit_deg,
                                           config->position_error_limit_deg);
    state->position_ref_deg = input->position_deg + output->position_error_deg;

    output->position_current_a = config->position_kp_a_per_deg * output->position_error_deg;
    const float speed_error_rpm = input->target_rpm - input->speed_rpm;
    output->speed_current_a = config->speed_kd_a_per_rpm * speed_error_rpm;
    const float speed_channel_limit_a = (config->speed_error_current_limit_a > 0.0f) ?
                                        config->speed_error_current_limit_a :
                                        config->current_limit_a;
    if (config->friction_rpm_scale > 0.0f)
    {
        output->friction_current_a = config->friction_current_a *
                                     tanhf(input->target_rpm / config->friction_rpm_scale);
    }

    const uint8_t target_active = (fabsf(input->target_rpm) > 0.001f) ? 1U : 0U;
    if (target_active == 0U)
    {
        state->speed_integral_current_a = 0.0f;
        state->last_target_rpm = 0.0f;
    }
    else if (input->target_rpm * state->last_target_rpm < 0.0f)
    {
        state->speed_integral_current_a = 0.0f;
    }
    if (target_active != 0U)
    {
        state->last_target_rpm = input->target_rpm;
    }

    const float base_current_a = output->position_current_a + output->friction_current_a;
    if (target_active != 0U &&
        config->speed_ki_a_per_rpm_s > 0.0f &&
        config->speed_integral_limit_a > 0.0f)
    {
        const float integral_candidate_a = mit_limit(
            state->speed_integral_current_a +
                config->speed_ki_a_per_rpm_s * speed_error_rpm * dt_s,
            -config->speed_integral_limit_a,
            config->speed_integral_limit_a);
        const float candidate_speed_channel_a = output->speed_current_a + integral_candidate_a;
        const uint8_t drives_out_of_saturation =
            (candidate_speed_channel_a > speed_channel_limit_a && speed_error_rpm < 0.0f) ||
            (candidate_speed_channel_a < -speed_channel_limit_a && speed_error_rpm > 0.0f);
        if (fabsf(candidate_speed_channel_a) <= speed_channel_limit_a ||
            drives_out_of_saturation != 0U)
        {
            state->speed_integral_current_a = integral_candidate_a;
        }
    }
    else
    {
        state->speed_integral_current_a = 0.0f;
    }
    output->speed_integral_current_a = state->speed_integral_current_a;

    const float speed_channel_a = mit_limit(output->speed_current_a +
                                            output->speed_integral_current_a,
                                            -speed_channel_limit_a,
                                            speed_channel_limit_a);
    output->speed_channel_current_a = speed_channel_a;
    output->raw_current_a = mit_limit(base_current_a + speed_channel_a,
                                      -config->current_limit_a,
                                      config->current_limit_a);
    return output->raw_current_a;
}
