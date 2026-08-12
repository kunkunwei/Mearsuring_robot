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

float Chassis_Hold_ComputePitchFeedforward(const Chassis_Hold_Config_t *config,
                                           float raw_pitch_rad,
                                           float *corrected_pitch_rad)
{
    // 如果配置无效，则无法计算Pitch前馈电流
    if (config == NULL)
    {
        // 如果调用者需要修正角度，则保留原始Pitch便于诊断
        if (corrected_pitch_rad != NULL)
        {
            *corrected_pitch_rad = raw_pitch_rad;
        }
        // 返回零电流，避免无效配置产生不可控输出
        return 0.0f;
    }

    // 先减去上电标定得到的Pitch零点
    const float corrected_pitch = Chassis_Hold_CorrectPitch(config, raw_pitch_rad);
    // 如果调用者需要修正后的Pitch，则同步输出该数值
    if (corrected_pitch_rad != NULL)
    {
        *corrected_pitch_rad = corrected_pitch;
    }

    const float abs_pitch = fabsf(corrected_pitch);
    const float off_pitch = config->pitch_feedforward_off_pitch_rad;
    const float full_pitch = config->pitch_feedforward_full_pitch_rad;
    float feedforward_scale = 1.0f;

    // 如果死区参数有效，并且当前Pitch仍在死区内，则关闭Pitch前馈
    if (off_pitch > 0.0f && abs_pitch <= off_pitch)
    {
        feedforward_scale = 0.0f;
    }
    // 如果过渡区参数有效，并且当前Pitch位于过渡区，则线性增加前馈比例
    else if (full_pitch > off_pitch && abs_pitch < full_pitch)
    {
        feedforward_scale = (abs_pitch - off_pitch) / (full_pitch - off_pitch);
    }

    // 使用原始修正Pitch计算重力方向，并通过比例平滑启用前馈
    return config->pitch_feedforward_a * sinf(corrected_pitch) * feedforward_scale;
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
    // 统一使用带3°到5°平滑死区的Pitch前馈计算
    output->pitch_current_a = Chassis_Hold_ComputePitchFeedforward(
        config,
        input->pitch_rad,
        &output->corrected_pitch_rad);
    output->raw_current_a = hold_limit(output->position_current_a +
                                       output->speed_current_a +
                                       output->pitch_current_a,
                                       -config->current_limit_a,
                                       config->current_limit_a);
    return output->raw_current_a;
}
