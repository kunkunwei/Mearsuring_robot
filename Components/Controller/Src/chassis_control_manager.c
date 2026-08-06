#include "chassis_control_manager.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float control_abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float control_limit(float value, float min_value, float max_value)
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

static int8_t control_sign(float value)
{
    if (value > 0.001f)
    {
        return 1;
    }
    if (value < -0.001f)
    {
        return -1;
    }
    return 0;
}

static float control_slew(float target, float current, float max_step)
{
    return current + control_limit(target - current, -max_step, max_step);
}

static float control_slew_asymmetric(float target,
                                     float current,
                                     float rise_step,
                                     float release_step)
{
    const int8_t target_sign = control_sign(target);
    const int8_t current_sign = control_sign(current);

    if (target_sign != 0 && current_sign != 0 && target_sign != current_sign)
    {
        return control_slew(0.0f, current, release_step);
    }

    const uint8_t increasing_magnitude =
        (target_sign != 0) &&
        (current_sign == 0 || control_abs(target) > control_abs(current));
    const float max_step = increasing_magnitude ? rise_step : release_step;
    return control_slew(target, current, max_step);
}

static void control_clear_output(Chassis_Control_Output_t *output,
                                 Chassis_Control_State_e state,
                                 Chassis_Control_Fault_e fault)
{
    memset(output, 0, sizeof(*output));
    output->state = state;
    output->fault = fault;
}

static uint8_t control_feedback_valid(const Chassis_Control_Input_t *input)
{
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        if (input->speed_valid[i] == 0U || input->position_valid[i] == 0U)
        {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t control_command_active(const Chassis_Control_Manager_t *manager,
                                      const Chassis_Control_Input_t *input)
{
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        if (control_abs(input->target_rpm[i]) > manager->config.command_deadband_rpm)
        {
            return 1U;
        }
    }
    return 0U;
}

static uint8_t control_is_pure_turn(const Chassis_Control_Manager_t *manager,
                                    const Chassis_Control_Input_t *input)
{
    const float left_target_rpm = 0.5f * (input->target_rpm[0] + input->target_rpm[3]);
    const float right_target_rpm = 0.5f * (input->target_rpm[1] + input->target_rpm[2]);
    const float longitudinal_target_rpm = 0.5f * (left_target_rpm + right_target_rpm);
    const float turn_target_rpm = 0.5f * (right_target_rpm - left_target_rpm);

    return (control_abs(longitudinal_target_rpm) <= manager->config.command_deadband_rpm &&
            control_abs(turn_target_rpm) > manager->config.command_deadband_rpm) ? 1U : 0U;
}

static float control_pitch_feedforward(const Chassis_Control_Manager_t *manager,
                                       float raw_pitch_rad)
{
    const float corrected_pitch_rad = Chassis_Hold_CorrectPitch(&manager->config.hold,
                                                                 raw_pitch_rad);
    return manager->config.hold.pitch_feedforward_a * sinf(corrected_pitch_rad);
}

static float control_robust_abs_speed(const Chassis_Control_Input_t *input)
{
    float speed[CHASSIS_CONTROL_MOTOR_COUNT];
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        speed[i] = control_abs(input->speed_rpm[i]);
    }

    for (uint8_t i = 1U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        const float value = speed[i];
        uint8_t j = i;
        while (j > 0U && speed[j - 1U] > value)
        {
            speed[j] = speed[j - 1U];
            j--;
        }
        speed[j] = value;
    }

    return 0.5f * (speed[1] + speed[2]);
}

static void control_enter_drive(Chassis_Control_Manager_t *manager,
                                const Chassis_Control_Input_t *input)
{
    manager->state = CHASSIS_CTRL_DRIVE;
    manager->hold_still_time_s = 0.0f;
    manager->turn_sync_ratio = 1.0f;
    memset(manager->breakaway_engaged, 0, sizeof(manager->breakaway_engaged));
    memset(manager->breakaway_engaged_time_s, 0, sizeof(manager->breakaway_engaged_time_s));
    memset(manager->drive_slip_elapsed_s, 0, sizeof(manager->drive_slip_elapsed_s));
    memset(manager->drive_slip_limited, 0, sizeof(manager->drive_slip_limited));
    Chassis_Hold_Reset(&manager->hold_state);
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        Chassis_Mit_Reset(&manager->drive_state[i], input->position_deg[i]);
    }
}

static float control_brake_position_comp_scale(const Chassis_Control_Manager_t *manager,
                                               float raw_pitch_rad)
{
    const float pitch_rad = control_abs(Chassis_Hold_CorrectPitch(&manager->config.hold,
                                                                  raw_pitch_rad));
    const float off_pitch = manager->config.brake_position_comp_off_pitch_rad;
    const float full_pitch = manager->config.brake_position_comp_full_pitch_rad;

    if (pitch_rad <= off_pitch)
    {
        return 0.0f;
    }
    if (pitch_rad >= full_pitch || full_pitch <= off_pitch)
    {
        return 1.0f;
    }
    return (pitch_rad - off_pitch) / (full_pitch - off_pitch);
}

static void control_enter_brake(Chassis_Control_Manager_t *manager,
                                const Chassis_Control_Input_t *input)
{
    manager->state = CHASSIS_CTRL_BRAKE;
    manager->hold_still_time_s = 0.0f;
    manager->brake_position_comp_scale = control_brake_position_comp_scale(manager,
                                                                            input->pitch_rad);
    Chassis_Hold_Capture(&manager->hold_state, input->position_deg);
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        manager->hold_wheel_position_ref_deg[i] = input->position_deg[i];
    }
}

static void control_enter_hold(Chassis_Control_Manager_t *manager,
                               const Chassis_Control_Input_t *input)
{
    Chassis_Hold_State_t stop_state;
    Chassis_Hold_Reset(&stop_state);
    Chassis_Hold_Capture(&stop_state, input->position_deg);

    manager->hold_state.position_ref_deg =
        manager->brake_position_comp_scale * manager->hold_state.position_ref_deg +
        (1.0f - manager->brake_position_comp_scale) * stop_state.position_ref_deg;
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        manager->hold_wheel_position_ref_deg[i] =
            manager->brake_position_comp_scale * manager->hold_wheel_position_ref_deg[i] +
            (1.0f - manager->brake_position_comp_scale) * input->position_deg[i];
    }
    manager->hold_state.initialized = 1U;
    manager->state = CHASSIS_CTRL_HOLD;
}

static void control_latch_fault(Chassis_Control_Manager_t *manager,
                                Chassis_Control_Fault_e fault,
                                Chassis_Control_Output_t *output)
{
    manager->state = CHASSIS_CTRL_FAULT;
    manager->fault = fault;
    memset(manager->last_current_a, 0, sizeof(manager->last_current_a));
    control_clear_output(output, manager->state, manager->fault);
}

void Chassis_ControlManager_DefaultConfig(Chassis_Control_Config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    *config = (Chassis_Control_Config_t){
        .drive = {
            .position_kp_a_per_deg = 0.0050f,
            .speed_kd_a_per_rpm = 0.008f,
            .speed_ki_a_per_rpm_s = 0.01f,
            .speed_integral_limit_a = 1.0f,
            .speed_error_current_limit_a = 1.5f,
            .friction_current_a = 1.00f,
            .friction_rpm_scale = 100.0f,
            .position_error_limit_deg = 5.0f,
            .current_limit_a = 3.5f,
        },
        .brake = {
            .speed_gain_a_per_rpm = 0.010f,
            .current_limit_a = 3.0f,
        },
        .hold = {
            .position_kp_a_per_deg = 0.010f,
            .speed_kd_a_per_rpm = 0.010f,
            .pitch_feedforward_a = -8.0f,
            .pitch_zero_offset_rad = 3.7f / 57.29577951308232f,
            .current_limit_a = 6.5f,
        },
        .brake_position_comp_off_pitch_rad = 3.0f / 57.29577951308232f,
        .brake_position_comp_full_pitch_rad = 5.0f / 57.29577951308232f,
        .command_deadband_rpm = 3.0f,
        .turn_breakaway_current_a = 2.50f,
        .turn_breakaway_target_rpm = 30.0f,
        .turn_breakaway_enter_rpm = 15.0f,
        .turn_breakaway_release_ratio = 0.90f,
        .turn_breakaway_hold_time_s = 0.500f,
        .turn_breakaway_taper_time_s = 0.250f,
        .turn_sync_enabled = 1U,
        .turn_sync_min_ratio = 0.25f,
        .turn_sync_time_s = 0.100f,
        .drive_slip_error_ratio = 0.40f,
        .drive_slip_recover_ratio = 0.20f,
        .drive_slip_confirm_time_s = 0.500f,
        .hold_enter_speed_rpm = 5.0f,
        .hold_enter_time_s = 0.100f,
        .current_rise_a_per_s = 30.0f,
        .current_release_a_per_s = 60.0f,
        .hold_overspeed_rpm = 200.0f,
        .hold_overspeed_time_s = 0.500f,
        .oscillation_window_s = 0.100f,
        .oscillation_min_current_a = 0.30f,
        .oscillation_reversal_limit = 4U,
        .saturation_time_s = 0.100f,
        .dt_min_s = 0.003f,
        .dt_max_s = 0.008f,
        .timing_fault_count_limit = 3U,
    };
}

void Chassis_ControlManager_Init(Chassis_Control_Manager_t *manager,
                                 const Chassis_Control_Config_t *config)
{
    if (manager == NULL)
    {
        return;
    }

    memset(manager, 0, sizeof(*manager));
    if (config == NULL)
    {
        Chassis_ControlManager_DefaultConfig(&manager->config);
    }
    else
    {
        manager->config = *config;
    }
    manager->turn_sync_ratio = 1.0f;
    manager->state = CHASSIS_CTRL_DISABLED;
}

void Chassis_ControlManager_Disable(Chassis_Control_Manager_t *manager,
                                    Chassis_Control_Output_t *output)
{
    if (manager == NULL || output == NULL)
    {
        return;
    }

    const Chassis_Control_Config_t config = manager->config;
    memset(manager, 0, sizeof(*manager));
    manager->config = config;
    manager->state = CHASSIS_CTRL_DISABLED;
    control_clear_output(output, CHASSIS_CTRL_DISABLED, CHASSIS_CTRL_FAULT_NONE);
}

void Chassis_ControlManager_SetDriveGains(Chassis_Control_Manager_t *manager,
                                          float position_kp_a_per_deg,
                                          float speed_kd_a_per_rpm,
                                          float friction_current_a)
{
    if (manager == NULL)
    {
        return;
    }

    manager->config.drive.position_kp_a_per_deg = control_limit(position_kp_a_per_deg, 0.0f, 0.05f);
    manager->config.drive.speed_kd_a_per_rpm = control_limit(speed_kd_a_per_rpm, 0.0f, 0.05f);
    manager->config.drive.friction_current_a = control_limit(friction_current_a,
                                                              0.0f,
                                                              manager->config.drive.current_limit_a);
}

static void control_breakaway_reset_wheel(Chassis_Control_Manager_t *manager, uint8_t wheel)
{
    manager->breakaway_engaged[wheel] = 0U;
    manager->breakaway_engaged_time_s[wheel] = 0.0f;
}

static float control_turn_breakaway_correction(Chassis_Control_Manager_t *manager,
                                               uint8_t wheel,
                                               float target_rpm,
                                               float speed_rpm,
                                               float rolling_friction_a,
                                               float dt_s)
{
    const Chassis_Control_Config_t *config = &manager->config;
    const int8_t target_sign = control_sign(target_rpm);
    const float abs_target = control_abs(target_rpm);

    if (config->turn_breakaway_current_a <= 0.0f ||
        config->turn_breakaway_target_rpm <= 0.0f ||
        target_sign == 0 ||
        abs_target <= config->command_deadband_rpm)
    {
        control_breakaway_reset_wheel(manager, wheel);
        return 0.0f;
    }

    /* 滞回：转速达到目标的 release_ratio 后释放，重新介入要求低于 enter_rpm，
     * 避免补偿阈值贴着工作点造成极限环。 */
    const float release_rpm = (abs_target * config->turn_breakaway_release_ratio >
                               config->turn_breakaway_enter_rpm * 2.0f) ?
                              abs_target * config->turn_breakaway_release_ratio :
                              config->turn_breakaway_enter_rpm * 2.0f;
    if (speed_rpm * (float)target_sign >= release_rpm)
    {
        control_breakaway_reset_wheel(manager, wheel);
        return 0.0f;
    }

    if (manager->breakaway_engaged[wheel] == 0U)
    {
        if (control_abs(speed_rpm) > config->turn_breakaway_enter_rpm)
        {
            return 0.0f;
        }
        manager->breakaway_engaged[wheel] = 1U;
        manager->breakaway_engaged_time_s[wheel] = 0.0f;
    }
    manager->breakaway_engaged_time_s[wheel] += dt_s;

    /* 超时保护：持续 hold_time 仍没跟上目标，则线性衰减到 0，
     * 防止拖滞轮长期被破静摩擦电流持续加压。 */
    const float elapsed_s = manager->breakaway_engaged_time_s[wheel];
    const float hold_s = config->turn_breakaway_hold_time_s;
    const float taper_s = config->turn_breakaway_taper_time_s;
    float time_scale = 1.0f;
    if (elapsed_s > hold_s)
    {
        time_scale = (taper_s > 0.0f) ?
                     control_limit(1.0f - (elapsed_s - hold_s) / taper_s, 0.0f, 1.0f) :
                     0.0f;
        if (time_scale <= 0.0f)
        {
            control_breakaway_reset_wheel(manager, wheel);
            return 0.0f;
        }
    }

    const float target_scale = control_limit(abs_target / config->turn_breakaway_target_rpm,
                                             0.0f,
                                             1.0f);
    const float full_breakaway_a = (float)target_sign * config->turn_breakaway_current_a;
    return (full_breakaway_a - rolling_friction_a) * target_scale * time_scale;
}

static float control_update_turn_sync(Chassis_Control_Manager_t *manager,
                                      const Chassis_Control_Input_t *input,
                                      uint8_t pure_turn,
                                      float dt_s)
{
    if (pure_turn == 0U || manager->config.turn_sync_enabled == 0U)
    {
        manager->turn_sync_ratio = 1.0f;
        return 1.0f;
    }

    float min_follow_ratio = 1.0f;
    uint8_t active_count = 0U;
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        const float abs_target = control_abs(input->target_rpm[i]);
        if (abs_target > manager->config.command_deadband_rpm)
        {
            const float follow_ratio = control_limit(
                control_abs(input->speed_rpm[i]) / fmaxf(abs_target, 10.0f),
                0.0f,
                1.0f);
            if (follow_ratio < min_follow_ratio)
            {
                min_follow_ratio = follow_ratio;
            }
            active_count++;
        }
    }
    if (active_count == 0U)
    {
        manager->turn_sync_ratio = 1.0f;
        return 1.0f;
    }

    const float response = (manager->config.turn_sync_time_s > 0.0f) ?
                           control_limit(dt_s / manager->config.turn_sync_time_s, 0.0f, 1.0f) :
                           1.0f;
    manager->turn_sync_ratio += (min_follow_ratio - manager->turn_sync_ratio) * response;
    manager->turn_sync_ratio = control_limit(manager->turn_sync_ratio,
                                             manager->config.turn_sync_min_ratio,
                                             1.0f);
    return manager->turn_sync_ratio;
}

static uint8_t control_update_slip_limit(Chassis_Control_Manager_t *manager,
                                         uint8_t wheel,
                                         float target_rpm,
                                         float speed_rpm,
                                         float dt_s)
{
    const Chassis_Control_Config_t *config = &manager->config;
    const float target_mag = control_abs(target_rpm);
    if (target_mag <= config->command_deadband_rpm)
    {
        manager->drive_slip_elapsed_s[wheel] = 0.0f;
        manager->drive_slip_limited[wheel] = 0U;
        return 0U;
    }

    const float follow_ratio = control_abs(speed_rpm) / fmaxf(target_mag, 10.0f);
    if (follow_ratio >= 1.0f - config->drive_slip_recover_ratio)
    {
        manager->drive_slip_elapsed_s[wheel] = 0.0f;
        manager->drive_slip_limited[wheel] = 0U;
    }
    else if (follow_ratio <= 1.0f - config->drive_slip_error_ratio)
    {
        manager->drive_slip_elapsed_s[wheel] += dt_s;
    }

    if (manager->drive_slip_elapsed_s[wheel] >= config->drive_slip_confirm_time_s)
    {
        manager->drive_slip_limited[wheel] = 1U;
    }
    return manager->drive_slip_limited[wheel];
}

static void control_compute_drive(Chassis_Control_Manager_t *manager,
                                  const Chassis_Control_Input_t *input,
                                  float dt_s,
                                  Chassis_Control_Output_t *output)
{
    const uint8_t pure_turn = control_is_pure_turn(manager, input);
    const float pitch_current_a = control_pitch_feedforward(manager, input->pitch_rad);
    const float sync_ratio = control_update_turn_sync(manager, input, pure_turn, dt_s);

    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        const float effective_target_rpm = input->target_rpm[i] * sync_ratio;
        const uint8_t slip_limited = control_update_slip_limit(manager,
                                                               (uint8_t)i,
                                                               effective_target_rpm,
                                                               input->speed_rpm[i],
                                                               dt_s);
        const Chassis_Mit_Input_t motor_input = {
            .target_rpm = effective_target_rpm,
            .speed_rpm = input->speed_rpm[i],
            .position_deg = input->position_deg[i],
            .slip_limited = slip_limited,
        };
        Chassis_Mit_Output_t motor_output;
        float drive_current_a = Chassis_Mit_Update(&manager->drive_state[i],
                                                    &manager->config.drive,
                                                    &motor_input,
                                                    dt_s,
                                                    &motor_output);
        if (slip_limited == 0U && pure_turn != 0U &&
            manager->config.turn_breakaway_current_a > 0.0f)
        {
            drive_current_a += control_turn_breakaway_correction(manager,
                                                                 (uint8_t)i,
                                                                 effective_target_rpm,
                                                                 input->speed_rpm[i],
                                                                 motor_output.friction_current_a,
                                                                 dt_s);
        }

        output->raw_current_a[i] = control_limit(drive_current_a + pitch_current_a,
                                                 -manager->config.drive.current_limit_a,
                                                 manager->config.drive.current_limit_a);
        output->position_current_a[i] = motor_output.position_current_a;
        output->speed_current_a[i] = motor_output.speed_current_a;
        output->feedforward_current_a[i] = motor_output.friction_current_a + pitch_current_a;
    }
}

static void control_build_hold_input(const Chassis_Control_Input_t *input,
                                     Chassis_Hold_Input_t *hold_input)
{
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        hold_input->position_deg[i] = input->position_deg[i];
        hold_input->speed_rpm[i] = input->speed_rpm[i];
    }
    hold_input->pitch_rad = input->pitch_rad;
}

static void control_compute_brake(Chassis_Control_Manager_t *manager,
                                  const Chassis_Control_Input_t *input,
                                  Chassis_Control_Output_t *output)
{
    Chassis_Hold_Input_t hold_input;
    Chassis_Hold_Output_t hold_output;
    control_build_hold_input(input, &hold_input);
    Chassis_Hold_Update(&manager->hold_state,
                        &manager->config.hold,
                        &hold_input,
                        &hold_output);
    const float position_current_a = hold_output.position_current_a *
                                     manager->brake_position_comp_scale;

    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        Chassis_Brake_Output_t motor_output;
        const float speed_current_a = Chassis_Brake_Update(&manager->config.brake,
                                                            input->speed_rpm[i],
                                                            &motor_output);
        output->position_current_a[i] = position_current_a;
        output->speed_current_a[i] = motor_output.speed_current_a;
        output->feedforward_current_a[i] = hold_output.pitch_current_a;
        output->raw_current_a[i] = control_limit(position_current_a +
                                                 speed_current_a +
                                                 hold_output.pitch_current_a,
                                                 -manager->config.brake.current_limit_a,
                                                 manager->config.brake.current_limit_a);
    }
}

static void control_compute_hold(Chassis_Control_Manager_t *manager,
                                 const Chassis_Control_Input_t *input,
                                 Chassis_Control_Output_t *output)
{
    Chassis_Hold_Input_t hold_input;
    control_build_hold_input(input, &hold_input);

    Chassis_Hold_Output_t hold_output;
    Chassis_Hold_Update(&manager->hold_state,
                        &manager->config.hold,
                        &hold_input,
                        &hold_output);
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        const float position_current_a = manager->config.hold.position_kp_a_per_deg *
                                         (manager->hold_wheel_position_ref_deg[i] -
                                          input->position_deg[i]);
        const float speed_current_a = -manager->config.hold.speed_kd_a_per_rpm *
                                      input->speed_rpm[i];
        output->raw_current_a[i] = control_limit(position_current_a +
                                                 speed_current_a +
                                                 hold_output.pitch_current_a,
                                                 -manager->config.hold.current_limit_a,
                                                 manager->config.hold.current_limit_a);
        output->position_current_a[i] = position_current_a;
        output->speed_current_a[i] = speed_current_a;
        output->feedforward_current_a[i] = hold_output.pitch_current_a;
    }
}

static void control_apply_slew(Chassis_Control_Manager_t *manager,
                               float dt_s,
                               Chassis_Control_Output_t *output)
{
    const float rise_step = manager->config.current_rise_a_per_s * dt_s;
    const float release_step = manager->config.current_release_a_per_s * dt_s;
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        manager->last_current_a[i] = control_slew_asymmetric(output->raw_current_a[i],
                                                             manager->last_current_a[i],
                                                             rise_step,
                                                             release_step);
        output->current_a[i] = manager->last_current_a[i];
    }
}

static uint8_t control_detect_reversal(Chassis_Control_Manager_t *manager,
                                       float dt_s,
                                       const Chassis_Control_Output_t *output)
{
    manager->reversal_window_elapsed_s += dt_s;
    if (manager->reversal_window_elapsed_s > manager->config.oscillation_window_s)
    {
        manager->reversal_window_elapsed_s = 0.0f;
        memset(manager->reversal_count, 0, sizeof(manager->reversal_count));
    }

    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        const int8_t sign = (control_abs(output->current_a[i]) >=
                             manager->config.oscillation_min_current_a) ?
                            control_sign(output->current_a[i]) : 0;
        if (sign != 0 && manager->last_current_sign[i] != 0 && sign != manager->last_current_sign[i])
        {
            manager->reversal_count[i]++;
        }
        if (sign != 0)
        {
            manager->last_current_sign[i] = sign;
        }
        if (manager->reversal_count[i] >= manager->config.oscillation_reversal_limit)
        {
            return 1U;
        }
    }
    return 0U;
}

static uint8_t control_detect_saturation(Chassis_Control_Manager_t *manager,
                                         float dt_s,
                                         const Chassis_Control_Output_t *output)
{
    const float limit = (manager->state == CHASSIS_CTRL_HOLD) ?
                        manager->config.hold.current_limit_a :
                        manager->config.brake.current_limit_a;
    uint8_t saturated = 0U;
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        const float monitored_current = (manager->state == CHASSIS_CTRL_HOLD) ?
                                        output->position_current_a[i] + output->speed_current_a[i] :
                                        output->raw_current_a[i];
        if (control_abs(monitored_current) >= limit - 0.001f)
        {
            saturated = 1U;
            break;
        }
    }

    manager->saturation_elapsed_s = saturated ?
                                    manager->saturation_elapsed_s + dt_s :
                                    0.0f;
    return (manager->saturation_elapsed_s >= manager->config.saturation_time_s) ? 1U : 0U;
}

static uint8_t control_detect_hold_overspeed(Chassis_Control_Manager_t *manager,
                                             const Chassis_Control_Input_t *input,
                                             uint8_t command_active,
                                             float dt_s)
{
    if (command_active != 0U || manager->state != CHASSIS_CTRL_HOLD ||
        control_robust_abs_speed(input) <= manager->config.hold_overspeed_rpm)
    {
        manager->hold_overspeed_elapsed_s = 0.0f;
        return 0U;
    }

    manager->hold_overspeed_elapsed_s += dt_s;
    return (manager->hold_overspeed_elapsed_s >= manager->config.hold_overspeed_time_s) ? 1U : 0U;
}

void Chassis_ControlManager_Update(Chassis_Control_Manager_t *manager,
                                   const Chassis_Control_Input_t *input,
                                   float dt_s,
                                   Chassis_Control_Output_t *output)
{
    if (manager == NULL || input == NULL || output == NULL)
    {
        return;
    }

    if (input->enabled == 0U)
    {
        Chassis_ControlManager_Disable(manager, output);
        return;
    }
    if (manager->state == CHASSIS_CTRL_FAULT)
    {
        control_clear_output(output, manager->state, manager->fault);
        return;
    }

    if (!isfinite(dt_s) || dt_s < manager->config.dt_min_s || dt_s > manager->config.dt_max_s)
    {
        manager->timing_fault_count++;
        dt_s = 0.005f;
        if (manager->timing_fault_count >= manager->config.timing_fault_count_limit)
        {
            control_latch_fault(manager, CHASSIS_CTRL_FAULT_TIMING, output);
            return;
        }
    }
    else
    {
        manager->timing_fault_count = 0U;
    }

    if (control_feedback_valid(input) == 0U)
    {
        control_latch_fault(manager, CHASSIS_CTRL_FAULT_FEEDBACK, output);
        return;
    }

    const uint8_t command_active = control_command_active(manager, input);
    if (control_detect_hold_overspeed(manager, input, command_active, dt_s) != 0U)
    {
        control_latch_fault(manager, CHASSIS_CTRL_FAULT_OVERSPEED, output);
        return;
    }

    if (command_active != 0U)
    {
        if (manager->state != CHASSIS_CTRL_DRIVE)
        {
            control_enter_drive(manager, input);
        }
    }
    else if (manager->state == CHASSIS_CTRL_DRIVE || manager->state == CHASSIS_CTRL_DISABLED)
    {
        control_enter_brake(manager, input);
    }

    if (manager->state == CHASSIS_CTRL_BRAKE)
    {
        if (control_robust_abs_speed(input) <= manager->config.hold_enter_speed_rpm)
        {
            manager->hold_still_time_s += dt_s;
            if (manager->hold_still_time_s >= manager->config.hold_enter_time_s - 1.0e-6f)
            {
                control_enter_hold(manager, input);
            }
        }
        else
        {
            manager->hold_still_time_s = 0.0f;
        }
    }

    control_clear_output(output, manager->state, manager->fault);
    if (manager->state == CHASSIS_CTRL_DRIVE)
    {
        control_compute_drive(manager, input, dt_s, output);
    }
    else if (manager->state == CHASSIS_CTRL_BRAKE)
    {
        control_compute_brake(manager, input, output);
    }
    else if (manager->state == CHASSIS_CTRL_HOLD)
    {
        control_compute_hold(manager, input, output);
    }

    control_apply_slew(manager, dt_s, output);
    if (command_active == 0U)
    {
        if (manager->state == CHASSIS_CTRL_HOLD)
        {
            if (control_detect_reversal(manager, dt_s, output) != 0U)
            {
                control_latch_fault(manager, CHASSIS_CTRL_FAULT_OSCILLATION, output);
                return;
            }
            if (control_detect_saturation(manager, dt_s, output) != 0U)
            {
                control_latch_fault(manager, CHASSIS_CTRL_FAULT_SATURATION, output);
            }
        }
        else
        {
            manager->reversal_window_elapsed_s = 0.0f;
            manager->saturation_elapsed_s = 0.0f;
            memset(manager->reversal_count, 0, sizeof(manager->reversal_count));
            memset(manager->last_current_sign, 0, sizeof(manager->last_current_sign));
        }
    }
    else
    {
        manager->reversal_window_elapsed_s = 0.0f;
        manager->saturation_elapsed_s = 0.0f;
        memset(manager->reversal_count, 0, sizeof(manager->reversal_count));
        memset(manager->last_current_sign, 0, sizeof(manager->last_current_sign));
    }
}
