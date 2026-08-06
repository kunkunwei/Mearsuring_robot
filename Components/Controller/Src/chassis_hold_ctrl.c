#include "chassis_hold_ctrl.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float hold_limit(float value, float min_value, float max_value)
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

static float hold_robust_mean(const float values[CHASSIS_HOLD_MOTOR_COUNT])
{
    float sorted[CHASSIS_HOLD_MOTOR_COUNT];
    memcpy(sorted, values, sizeof(sorted));

    for (uint8_t i = 1U; i < CHASSIS_HOLD_MOTOR_COUNT; i++)
    {
        const float value = sorted[i];
        uint8_t j = i;
        while (j > 0U && sorted[j - 1U] > value)
        {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = value;
    }

    return 0.5f * (sorted[1] + sorted[2]);
}

void Chassis_Hold_Reset(Chassis_Hold_State_t *state)
{
    if (state == NULL)
    {
        return;
    }
    memset(state, 0, sizeof(*state));
}

void Chassis_Hold_Capture(Chassis_Hold_State_t *state,
                          const float position_deg[CHASSIS_HOLD_MOTOR_COUNT])
{
    if (state == NULL || position_deg == NULL)
    {
        return;
    }

    state->position_ref_deg = hold_robust_mean(position_deg);
    state->initialized = 1U;
}

float Chassis_Hold_CorrectPitch(const Chassis_Hold_Config_t *config,
                                float raw_pitch_rad)
{
    if (config == NULL)
    {
        return raw_pitch_rad;
    }
    return raw_pitch_rad - config->pitch_zero_offset_rad;
}

float Chassis_Hold_Update(Chassis_Hold_State_t *state,
                          const Chassis_Hold_Config_t *config,
                          const Chassis_Hold_Input_t *input,
                          Chassis_Hold_Output_t *output)
{
    if (state == NULL || config == NULL || input == NULL || output == NULL)
    {
        return 0.0f;
    }

    memset(output, 0, sizeof(*output));
    output->mean_position_deg = hold_robust_mean(input->position_deg);
    output->mean_speed_rpm = hold_robust_mean(input->speed_rpm);
    if (state->initialized == 0U)
    {
        Chassis_Hold_Capture(state, input->position_deg);
    }

    output->position_current_a = config->position_kp_a_per_deg *
                                 (state->position_ref_deg - output->mean_position_deg);
    output->speed_current_a = -config->speed_kd_a_per_rpm * output->mean_speed_rpm;
    output->corrected_pitch_rad = Chassis_Hold_CorrectPitch(config, input->pitch_rad);
    output->pitch_current_a = config->pitch_feedforward_a * sinf(output->corrected_pitch_rad);
    output->raw_current_a = hold_limit(output->position_current_a +
                                       output->speed_current_a +
                                       output->pitch_current_a,
                                       -config->current_limit_a,
                                       config->current_limit_a);
    return output->raw_current_a;
}
