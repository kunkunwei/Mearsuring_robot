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

static float control_max_abs_speed(const Chassis_Control_Input_t *input)
{
    float max_speed = 0.0f;
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        const float speed = control_abs(input->speed_rpm[i]);
        if (speed > max_speed)
        {
            max_speed = speed;
        }
    }
    return max_speed;
}

static void control_enter_drive(Chassis_Control_Manager_t *manager,
                                const Chassis_Control_Input_t *input)
{
    manager->state = CHASSIS_CTRL_DRIVE;
    manager->hold_still_time_s = 0.0f;
    Chassis_Hold_Reset(&manager->hold_state);
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        Chassis_Mit_Reset(&manager->drive_state[i], input->position_deg[i]);
    }
}

static void control_enter_brake(Chassis_Control_Manager_t *manager)
{
    manager->state = CHASSIS_CTRL_BRAKE;
    manager->hold_still_time_s = 0.0f;
    Chassis_Hold_Reset(&manager->hold_state);
}

static void control_enter_hold(Chassis_Control_Manager_t *manager,
                               const Chassis_Control_Input_t *input)
{
    manager->state = CHASSIS_CTRL_HOLD;
    Chassis_Hold_Capture(&manager->hold_state, input->position_deg);
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
            .position_kp_a_per_deg = 0.0105f,
            .speed_kd_a_per_rpm = 0.0105f,
            .friction_current_a = 0.25f,
            .friction_rpm_scale = 30.0f,
            .position_error_limit_deg = 5.0f,
            .current_limit_a = 0.5f,
        },
        .brake = {
            .speed_gain_a_per_rpm = 0.010f,
            .current_limit_a = 0.5f,
        },
        .hold = {
            .position_kp_a_per_deg = 0.010f,
            .speed_kd_a_per_rpm = 0.010f,
            .pitch_feedforward_a = 0.0f,
            .current_limit_a = 0.5f,
        },
        .command_deadband_rpm = 3.0f,
        .hold_enter_speed_rpm = 5.0f,
        .hold_enter_time_s = 0.200f,
        .current_slew_a_per_s = 10.0f,
        .hold_overspeed_rpm = 50.0f,
        .oscillation_window_s = 0.100f,
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

static void control_compute_drive(Chassis_Control_Manager_t *manager,
                                  const Chassis_Control_Input_t *input,
                                  float dt_s,
                                  Chassis_Control_Output_t *output)
{
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        const Chassis_Mit_Input_t motor_input = {
            .target_rpm = input->target_rpm[i],
            .speed_rpm = input->speed_rpm[i],
            .position_deg = input->position_deg[i],
        };
        Chassis_Mit_Output_t motor_output;
        output->raw_current_a[i] = Chassis_Mit_Update(&manager->drive_state[i],
                                                       &manager->config.drive,
                                                       &motor_input,
                                                       dt_s,
                                                       &motor_output);
        output->position_current_a[i] = motor_output.position_current_a;
        output->speed_current_a[i] = motor_output.speed_current_a;
        output->feedforward_current_a[i] = motor_output.friction_current_a;
    }
}

static void control_compute_brake(Chassis_Control_Manager_t *manager,
                                  const Chassis_Control_Input_t *input,
                                  Chassis_Control_Output_t *output)
{
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        Chassis_Brake_Output_t motor_output;
        output->raw_current_a[i] = Chassis_Brake_Update(&manager->config.brake,
                                                        input->speed_rpm[i],
                                                        &motor_output);
        output->speed_current_a[i] = motor_output.speed_current_a;
    }
}

static void control_compute_hold(Chassis_Control_Manager_t *manager,
                                 const Chassis_Control_Input_t *input,
                                 Chassis_Control_Output_t *output)
{
    Chassis_Hold_Input_t hold_input = {.pitch_rad = input->pitch_rad};
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        hold_input.position_deg[i] = input->position_deg[i];
        hold_input.speed_rpm[i] = input->speed_rpm[i];
    }

    Chassis_Hold_Output_t hold_output;
    const float current_a = Chassis_Hold_Update(&manager->hold_state,
                                                 &manager->config.hold,
                                                 &hold_input,
                                                 &hold_output);
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        output->raw_current_a[i] = current_a;
        output->position_current_a[i] = hold_output.position_current_a;
        output->speed_current_a[i] = hold_output.speed_current_a;
        output->feedforward_current_a[i] = hold_output.pitch_current_a;
    }
}

static void control_apply_slew(Chassis_Control_Manager_t *manager,
                               float dt_s,
                               Chassis_Control_Output_t *output)
{
    const float max_step = manager->config.current_slew_a_per_s * dt_s;
    for (uint8_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        manager->last_current_a[i] = control_slew(output->raw_current_a[i],
                                                  manager->last_current_a[i],
                                                  max_step);
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
        const int8_t sign = control_sign(output->current_a[i]);
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
        if (control_abs(output->raw_current_a[i]) >= limit - 0.001f)
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
    if (command_active == 0U && manager->state == CHASSIS_CTRL_HOLD &&
        control_max_abs_speed(input) > manager->config.hold_overspeed_rpm)
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
        control_enter_brake(manager);
    }

    if (manager->state == CHASSIS_CTRL_BRAKE)
    {
        if (control_max_abs_speed(input) <= manager->config.hold_enter_speed_rpm)
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
        if (control_detect_reversal(manager, dt_s, output) != 0U)
        {
            control_latch_fault(manager, CHASSIS_CTRL_FAULT_OSCILLATION, output);
            return;
        }
        if (manager->state == CHASSIS_CTRL_HOLD &&
            control_detect_saturation(manager, dt_s, output) != 0U)
        {
            control_latch_fault(manager, CHASSIS_CTRL_FAULT_SATURATION, output);
        }
        else if (manager->state != CHASSIS_CTRL_HOLD)
        {
            manager->saturation_elapsed_s = 0.0f;
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
