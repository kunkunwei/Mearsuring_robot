#include "chassis_control_manager.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

static Chassis_Control_Input_t valid_input(void)
{
    Chassis_Control_Input_t input = {
        .pitch_rad = 3.7f / 57.29577951308232f,
    };
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.speed_valid[i] = 1U;
        input.position_valid[i] = 1U;
    }
    return input;
}

static void assert_currents_are_zero(const Chassis_Control_Output_t *output)
{
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        assert(output->current_a[i] == 0.0f);
    }
}

static Chassis_Mit_Config_t integral_only_mit_config(void)
{
    const Chassis_Mit_Config_t config = {
        .speed_ki_a_per_rpm_s = 0.010f,
        .speed_integral_limit_a = 1.0f,
        .current_limit_a = 10.0f,
    };
    return config;
}

static void test_mit_speed_integral_accumulates_persistent_error(void)
{
    Chassis_Mit_Config_t config = integral_only_mit_config();
    Chassis_Mit_State_t state;
    Chassis_Mit_Output_t output;
    const Chassis_Mit_Input_t input = {.target_rpm = 100.0f};

    Chassis_Mit_Reset(&state, 0.0f);
    Chassis_Mit_Update(&state, &config, &input, 0.100f, &output);
    const float first_current = output.speed_integral_current_a;
    Chassis_Mit_Update(&state, &config, &input, 0.100f, &output);

    assert(first_current > 0.0f);
    assert(output.speed_integral_current_a > first_current);
}

static void test_mit_speed_integral_is_bounded(void)
{
    Chassis_Mit_Config_t config = integral_only_mit_config();
    Chassis_Mit_State_t state;
    Chassis_Mit_Output_t output;
    const Chassis_Mit_Input_t input = {.target_rpm = 100.0f};

    Chassis_Mit_Reset(&state, 0.0f);
    for (size_t step = 0U; step < 30U; step++)
    {
        Chassis_Mit_Update(&state, &config, &input, 0.100f, &output);
    }

    assert(fabsf(output.speed_integral_current_a - config.speed_integral_limit_a) <= 1.0e-6f);
}

static void test_mit_speed_integral_stops_at_output_saturation(void)
{
    Chassis_Mit_Config_t config = integral_only_mit_config();
    Chassis_Mit_State_t state;
    Chassis_Mit_Output_t output;
    const Chassis_Mit_Input_t input = {.target_rpm = 100.0f};

    config.speed_kd_a_per_rpm = 0.1f;
    config.current_limit_a = 1.0f;
    Chassis_Mit_Reset(&state, 0.0f);
    Chassis_Mit_Update(&state, &config, &input, 0.100f, &output);

    assert(fabsf(output.raw_current_a - config.current_limit_a) <= 1.0e-6f);
    assert(fabsf(output.speed_integral_current_a) <= 1.0e-6f);
}

static void test_mit_speed_integral_resets_on_target_reversal(void)
{
    Chassis_Mit_Config_t config = integral_only_mit_config();
    Chassis_Mit_State_t state;
    Chassis_Mit_Output_t output;
    Chassis_Mit_Input_t input = {.target_rpm = 100.0f};

    Chassis_Mit_Reset(&state, 0.0f);
    Chassis_Mit_Update(&state, &config, &input, 0.100f, &output);
    assert(output.speed_integral_current_a > 0.0f);

    input.target_rpm = -100.0f;
    Chassis_Mit_Update(&state, &config, &input, 0.100f, &output);
    assert(output.speed_integral_current_a < 0.0f);
}

static void test_mit_speed_integral_clears_at_zero_target(void)
{
    Chassis_Mit_Config_t config = integral_only_mit_config();
    Chassis_Mit_State_t state;
    Chassis_Mit_Output_t output;
    Chassis_Mit_Input_t input = {.target_rpm = 100.0f};

    Chassis_Mit_Reset(&state, 0.0f);
    Chassis_Mit_Update(&state, &config, &input, 0.100f, &output);
    assert(output.speed_integral_current_a > 0.0f);

    input.target_rpm = 0.0f;
    Chassis_Mit_Update(&state, &config, &input, 0.100f, &output);
    assert(fabsf(output.speed_integral_current_a) <= 1.0e-6f);
}

static void test_mit_speed_error_channel_has_independent_limit(void)
{
    const Chassis_Mit_Config_t config = {
        .speed_kd_a_per_rpm = 0.10f,
        .speed_ki_a_per_rpm_s = 0.10f,
        .speed_integral_limit_a = 3.0f,
        .speed_error_current_limit_a = 1.5f,
        .current_limit_a = 10.0f,
    };
    Chassis_Mit_State_t state;
    Chassis_Mit_Output_t output;
    const Chassis_Mit_Input_t input = {.target_rpm = 100.0f};

    Chassis_Mit_Reset(&state, 0.0f);
    for (size_t step = 0U; step < 30U; step++)
    {
        Chassis_Mit_Update(&state, &config, &input, 0.100f, &output);
        assert(fabsf(output.speed_channel_current_a) <= 1.5f + 1.0e-6f);
    }
    assert(fabsf(output.raw_current_a - 1.5f) <= 1.0e-5f);
}

static void test_mit_slip_limited_wheel_drops_tracking_current(void)
{
    const Chassis_Mit_Config_t config = {
        .speed_kd_a_per_rpm = 0.10f,
        .speed_integral_limit_a = 1.0f,
        .current_limit_a = 10.0f,
    };
    Chassis_Mit_State_t state;
    Chassis_Mit_Output_t output;
    const Chassis_Mit_Input_t input = {
        .target_rpm = 100.0f,
        .slip_limited = 1U,
    };

    Chassis_Mit_Reset(&state, 0.0f);
    Chassis_Mit_Update(&state, &config, &input, 0.100f, &output);

    assert(output.raw_current_a == 0.0f);
    assert(output.speed_integral_current_a == 0.0f);
}

static void test_disabled_output_is_zero(void)
{
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_Init(&manager, NULL);
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_DISABLED);
    assert(output.fault == CHASSIS_CTRL_FAULT_NONE);
    assert_currents_are_zero(&output);
}

static void test_nonzero_command_enters_bounded_drive(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    Chassis_ControlManager_Init(&manager, &config);
    input.enabled = 1U;
    input.target_rpm[0] = 20.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_DRIVE);
    assert(output.current_a[0] > 0.0f);
    assert(output.current_a[0] <= config.current_rise_a_per_s * 0.005f + 1.0e-6f);
    assert(fabsf(output.current_a[0]) <= config.drive.current_limit_a + 1.0e-6f);
}

static void test_pure_turn_breakaway_current_is_separate_from_rolling_friction(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.drive.position_kp_a_per_deg = 0.0f;
    config.drive.speed_kd_a_per_rpm = 0.0f;
    config.drive.speed_ki_a_per_rpm_s = 0.0f;
    config.drive.friction_current_a = 0.5f;
    config.drive.friction_rpm_scale = 100.0f;
    config.turn_breakaway_current_a = 2.5f;
    config.turn_breakaway_target_rpm = 30.0f;
    config.turn_breakaway_enter_rpm = 15.0f;
    config.turn_breakaway_release_ratio = 0.90f;
    config.turn_breakaway_hold_time_s = 10.0f;
    config.turn_breakaway_taper_time_s = 1.0f;
    config.turn_sync_enabled = 0U;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    input.target_rpm[0] = 60.0f;
    input.target_rpm[1] = -60.0f;
    input.target_rpm[2] = -60.0f;
    input.target_rpm[3] = 60.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_DRIVE);
    assert(output.raw_current_a[0] >= 2.49f);
    assert(output.raw_current_a[1] <= -2.49f);

    input.speed_rpm[0] = 80.0f;
    input.speed_rpm[1] = -80.0f;
    input.speed_rpm[2] = -80.0f;
    input.speed_rpm[3] = 80.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(fabsf(output.raw_current_a[0]) < 1.0f);
    assert(fabsf(output.raw_current_a[1]) < 1.0f);

    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.speed_rpm[i] = 0.0f;
        input.target_rpm[i] = (i == 0U || i == 3U) ? 5.0f : -5.0f;
    }
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.raw_current_a[0] > 0.3f);
    assert(output.raw_current_a[0] < 0.6f);
    assert(output.raw_current_a[1] < -0.3f);
    assert(output.raw_current_a[1] > -0.6f);
}

static void test_pure_turn_breakaway_does_not_reengage_above_enter_speed(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.drive.position_kp_a_per_deg = 0.0f;
    config.drive.speed_kd_a_per_rpm = 0.0f;
    config.drive.speed_ki_a_per_rpm_s = 0.0f;
    config.drive.friction_current_a = 0.0f;
    config.drive.friction_rpm_scale = 100.0f;
    config.turn_sync_enabled = 0U;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    config.turn_breakaway_current_a = 2.5f;
    config.turn_breakaway_target_rpm = 30.0f;
    config.turn_breakaway_enter_rpm = 15.0f;
    config.turn_breakaway_release_ratio = 0.90f;
    config.turn_breakaway_hold_time_s = 10.0f;
    config.turn_breakaway_taper_time_s = 1.0f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.target_rpm[i] = (i == 0U || i == 3U) ? 77.0f : -77.0f;
    }
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.raw_current_a[0] >= 2.4f);

    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.speed_rpm[i] = (i == 0U || i == 3U) ? 70.0f : -70.0f;
    }
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(fabsf(output.raw_current_a[0]) <= 1.0e-4f);

    for (size_t step = 0U; step < 20U; step++)
    {
        for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
        {
            input.speed_rpm[i] = (i == 0U || i == 3U) ? 50.0f : -50.0f;
        }
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
        assert(fabsf(output.raw_current_a[0]) <= 1.0e-4f);
    }
}

static void test_pure_turn_breakaway_taper_releases_dragging_wheel(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.drive.position_kp_a_per_deg = 0.0f;
    config.drive.speed_kd_a_per_rpm = 0.0f;
    config.drive.speed_ki_a_per_rpm_s = 0.0f;
    config.drive.friction_current_a = 0.0f;
    config.drive.friction_rpm_scale = 100.0f;
    config.turn_sync_enabled = 0U;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    config.turn_breakaway_current_a = 2.5f;
    config.turn_breakaway_target_rpm = 30.0f;
    config.turn_breakaway_enter_rpm = 15.0f;
    config.turn_breakaway_release_ratio = 0.90f;
    config.turn_breakaway_hold_time_s = 0.050f;
    config.turn_breakaway_taper_time_s = 0.050f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.target_rpm[i] = (i == 0U || i == 3U) ? 77.0f : -77.0f;
        input.speed_rpm[i] = 0.0f;
    }
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.raw_current_a[0] >= 2.4f);

    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.speed_rpm[i] = (i == 0U || i == 3U) ? 30.0f : -30.0f;
    }
    for (size_t step = 0U; step < 25U; step++)
    {
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }
    assert(fabsf(output.raw_current_a[0]) <= 1.0e-4f);

    for (size_t step = 0U; step < 10U; step++)
    {
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
        assert(fabsf(output.raw_current_a[0]) <= 1.0e-4f);
    }
}

static void test_pure_turn_sync_scales_all_wheels_to_slowest(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.drive.position_kp_a_per_deg = 0.0f;
    config.drive.speed_kd_a_per_rpm = 0.0f;
    config.drive.speed_ki_a_per_rpm_s = 0.0f;
    config.drive.friction_current_a = 0.5f;
    config.drive.friction_rpm_scale = 100.0f;
    config.turn_breakaway_current_a = 0.0f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    config.turn_sync_enabled = 1U;
    config.turn_sync_min_ratio = 0.25f;
    config.turn_sync_time_s = 0.100f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.target_rpm[i] = (i == 0U || i == 3U) ? 77.0f : -77.0f;
    }
    input.speed_rpm[0] = -77.0f;
    input.speed_rpm[1] = 70.0f;
    input.speed_rpm[2] = 30.0f;
    input.speed_rpm[3] = -77.0f;

    for (size_t step = 0U; step < 120U; step++)
    {
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }

    const float expected_slow_current_a = 0.5f * tanhf(30.0f / 100.0f);
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        const float expected = (i == 0U || i == 3U) ?
                               expected_slow_current_a :
                               -expected_slow_current_a;
        assert(fabsf(output.raw_current_a[i] - expected) <= 0.01f);
    }

    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.speed_rpm[i] = (i == 0U || i == 3U) ? -77.0f : 77.0f;
    }
    for (size_t step = 0U; step < 120U; step++)
    {
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }
    const float expected_full_current_a = 0.5f * tanhf(77.0f / 100.0f);
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        const float expected = (i == 0U || i == 3U) ?
                               expected_full_current_a :
                               -expected_full_current_a;
        assert(fabsf(output.raw_current_a[i] - expected) <= 0.01f);
    }
}

static void test_drive_slip_limit_disables_dragging_wheel_tracking(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.drive.position_kp_a_per_deg = 0.0f;
    config.drive.speed_kd_a_per_rpm = 0.01f;
    config.drive.speed_ki_a_per_rpm_s = 0.0f;
    config.drive.friction_current_a = 0.0f;
    config.turn_sync_enabled = 0U;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.target_rpm[i] = 77.0f;
    }
    input.speed_rpm[0] = 70.0f;
    input.speed_rpm[1] = 30.0f;
    input.speed_rpm[2] = 70.0f;
    input.speed_rpm[3] = 70.0f;

    for (size_t step = 0U; step < 50U; step++)
    {
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }
    assert(fabsf(output.raw_current_a[1] - 0.47f) <= 1.0e-4f);
    assert(fabsf(output.raw_current_a[0] - 0.07f) <= 1.0e-4f);

    for (size_t step = 0U; step < 60U; step++)
    {
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }
    assert(output.raw_current_a[1] == 0.0f);
    assert(fabsf(output.raw_current_a[0] - 0.07f) <= 1.0e-4f);

    input.speed_rpm[1] = 70.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(fabsf(output.raw_current_a[1] - 0.07f) <= 1.0e-4f);
}

static void test_drive_keeps_pitch_feedforward_on_slope(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.drive.position_kp_a_per_deg = 0.0f;
    config.drive.speed_kd_a_per_rpm = 0.0f;
    config.drive.speed_ki_a_per_rpm_s = 0.0f;
    config.drive.friction_current_a = 0.0f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    input.pitch_rad = config.hold.pitch_zero_offset_rad +
                      14.0f / 57.29577951308232f;
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.target_rpm[i] = 5.0f;
    }
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_DRIVE);
    assert(output.raw_current_a[0] > 1.8f);
    assert(output.raw_current_a[1] > 1.8f);
}

static void test_zero_command_brakes_before_hold(void)
{
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_Init(&manager, NULL);
    input.enabled = 1U;
    input.target_rpm[0] = 20.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    input.target_rpm[0] = 0.0f;
    input.speed_rpm[0] = 20.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_BRAKE);
    assert(output.current_a[0] <= 0.0f);
}

static void test_release_captures_position_and_compensates_pitch_during_brake(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 1.0f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.target_rpm[i] = 100.0f;
        input.speed_rpm[i] = 100.0f;
        input.position_deg[i] = 10.0f;
    }
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.target_rpm[i] = 0.0f;
        input.speed_rpm[i] = 20.0f;
        input.position_deg[i] = 12.0f;
    }
    input.pitch_rad = config.hold.pitch_zero_offset_rad +
                      8.0f / 57.29577951308232f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_BRAKE);
    assert(manager.hold_state.initialized != 0U);
    assert(fabsf(manager.hold_state.position_ref_deg - 12.0f) <= 1.0e-6f);
    assert(output.position_current_a[0] == 0.0f);
    assert(output.speed_current_a[0] < 0.0f);
    assert(output.feedforward_current_a[0] > 0.0f);
    assert(output.raw_current_a[0] > 0.0f);
}

static float brake_position_current_for_pitch(float corrected_pitch_deg)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 1.0f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    input.target_rpm[0] = 100.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    input.target_rpm[0] = 0.0f;
    input.speed_rpm[0] = 20.0f;
    input.pitch_rad = config.hold.pitch_zero_offset_rad +
                      corrected_pitch_deg / 57.29577951308232f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.position_deg[i] = 20.0f;
    }
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_BRAKE);
    return output.position_current_a[0];
}

static void test_flat_brake_disables_position_compensation(void)
{
    assert(fabsf(brake_position_current_for_pitch(0.0f)) <= 1.0e-6f);
}

static void test_uphill_and_downhill_enable_brake_position_compensation(void)
{
    const float uphill_current = brake_position_current_for_pitch(6.0f);
    const float downhill_current = brake_position_current_for_pitch(-6.0f);

    assert(uphill_current < 0.0f);
    assert(downhill_current < 0.0f);
    assert(fabsf(uphill_current - downhill_current) <= 1.0e-6f);
}

static void test_brake_position_compensation_blends_between_three_and_five_degrees(void)
{
    const float transition_current = brake_position_current_for_pitch(4.0f);
    const float full_current = brake_position_current_for_pitch(6.0f);

    assert(fabsf(transition_current - 0.5f * full_current) <= 1.0e-6f);
}

static float hold_reference_after_brake(float corrected_pitch_deg)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.010f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    input.target_rpm[0] = 100.0f;
    input.pitch_rad = config.hold.pitch_zero_offset_rad +
                      corrected_pitch_deg / 57.29577951308232f;
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.position_deg[i] = 10.0f;
    }
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    input.target_rpm[0] = 0.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_BRAKE);

    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.position_deg[i] = 20.0f;
    }
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_HOLD);
    return manager.hold_state.position_ref_deg;
}

static void test_flat_brake_to_hold_uses_stop_position(void)
{
    assert(fabsf(hold_reference_after_brake(0.0f) - 20.0f) <= 1.0e-6f);
}

static void test_steep_slope_brake_to_hold_preserves_release_position(void)
{
    assert(fabsf(hold_reference_after_brake(-6.0f) - 10.0f) <= 1.0e-6f);
    assert(fabsf(hold_reference_after_brake(6.0f) - 10.0f) <= 1.0e-6f);
}

static void test_brake_to_hold_blends_reference_in_pitch_transition(void)
{
    assert(fabsf(hold_reference_after_brake(4.0f) - 15.0f) <= 1.0e-6f);
}

static void test_one_speed_outlier_does_not_block_hold_entry(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.010f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    input.speed_rpm[3] = 20.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_HOLD);
}

static void test_high_speed_release_enters_brake_instead_of_fault(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 1.0f;
    Chassis_ControlManager_Init(&manager, &config);
    input.enabled = 1U;
    input.target_rpm[0] = 100.0f;
    input.speed_rpm[0] = 100.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_DRIVE);

    input.target_rpm[0] = 0.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_BRAKE);
    assert(output.fault == CHASSIS_CTRL_FAULT_NONE);
    assert(output.current_a[0] <= 0.0f);
    for (size_t step = 0U; step < 25U; step++)
    {
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }
    assert(output.state == CHASSIS_CTRL_BRAKE);
    assert(output.fault == CHASSIS_CTRL_FAULT_NONE);
    assert(output.current_a[0] < 0.0f);
}

static void test_feedback_loss_latches_fault(void)
{
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_Init(&manager, NULL);
    input.enabled = 1U;
    input.target_rpm[0] = 20.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    input.speed_valid[0] = 0U;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_FAULT);
    assert(output.fault == CHASSIS_CTRL_FAULT_FEEDBACK);
    assert_currents_are_zero(&output);

    input.speed_valid[0] = 1U;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_FAULT);

    Chassis_ControlManager_Disable(&manager, &output);
    assert(output.state == CHASSIS_CTRL_DISABLED);
    assert(output.fault == CHASSIS_CTRL_FAULT_NONE);
    assert_currents_are_zero(&output);
}

static void test_drive_current_reversal_releases_fast_then_ramps_opposite(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.current_rise_a_per_s = 10.0f;
    config.current_release_a_per_s = 60.0f;
    Chassis_ControlManager_Init(&manager, &config);
    input.enabled = 1U;
    for (size_t step = 0U; step < 5U; step++)
    {
        input.target_rpm[0] = 1000.0f;
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }
    assert(fabsf(output.current_a[0] - 0.25f) <= 1.0e-6f);

    input.target_rpm[0] = -1000.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(fabsf(output.current_a[0]) <= 1.0e-6f);

    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(fabsf(output.current_a[0] + 0.05f) <= 1.0e-6f);
}

static void test_hold_closes_each_wheel_position_independently(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.005f;
    config.hold.speed_kd_a_per_rpm = 0.0f;
    config.hold.pitch_feedforward_a = 0.0f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);
    input.enabled = 1U;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_HOLD);

    input.position_deg[1] = 100.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_HOLD);
    assert(fabsf(output.position_current_a[0]) <= 1.0e-6f);
    assert(output.position_current_a[1] < -0.9f);
    assert(fabsf(output.position_current_a[2]) <= 1.0e-6f);
    assert(fabsf(output.position_current_a[3]) <= 1.0e-6f);
}

static void test_hold_damps_a_slipping_wheel_independently(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.005f;
    config.hold.position_kp_a_per_deg = 0.0f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    input.pitch_rad = config.hold.pitch_zero_offset_rad +
                      15.0f / 57.29577951308232f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_HOLD);

    input.speed_rpm[1] = 150.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_HOLD);
    assert(output.speed_current_a[1] < 0.0f);
    assert(fabsf(output.speed_current_a[0]) <= 1.0e-6f);
    assert(output.raw_current_a[1] < output.raw_current_a[0]);
    assert(fabsf(output.raw_current_a[0] - output.raw_current_a[2]) <= 1.0e-6f);
}

static void test_pitch_offset_removes_flat_table_bias(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.005f;
    config.hold.position_kp_a_per_deg = 0.0f;
    config.hold.speed_kd_a_per_rpm = 0.0f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    input.pitch_rad = config.hold.pitch_zero_offset_rad;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_HOLD);
    assert(fabsf(output.feedforward_current_a[0]) <= 1.0e-6f);
    assert(fabsf(output.current_a[0]) <= 1.0e-6f);
}

static void test_pitch_feedforward_uses_three_to_five_degree_smooth_deadband(void)
{
    Chassis_Control_Config_t config;
    float corrected_pitch_rad = 0.0f;

    Chassis_ControlManager_DefaultConfig(&config);

    // 2°位于死区内，Pitch前馈必须完全关闭
    const float deadband_current = Chassis_Hold_ComputePitchFeedforward(
        &config.hold,
        config.hold.pitch_zero_offset_rad + 2.0f / 57.29577951308232f,
        &corrected_pitch_rad);
    assert(fabsf(corrected_pitch_rad - 2.0f / 57.29577951308232f) <= 1.0e-6f);
    assert(fabsf(deadband_current) <= 1.0e-6f);

    // 4°位于3°到5°过渡区中点，Pitch前馈应为完整值的一半
    const float transition_current = Chassis_Hold_ComputePitchFeedforward(
        &config.hold,
        config.hold.pitch_zero_offset_rad + 4.0f / 57.29577951308232f,
        NULL);
    const float transition_full_current =
        config.hold.pitch_feedforward_a * sinf(4.0f / 57.29577951308232f);
    assert(fabsf(transition_current - 0.5f * transition_full_current) <= 1.0e-6f);

    // 5°达到完整启用阈值，Pitch前馈不得继续缩小
    const float full_current = Chassis_Hold_ComputePitchFeedforward(
        &config.hold,
        config.hold.pitch_zero_offset_rad + 5.0f / 57.29577951308232f,
        NULL);
    const float expected_full_current =
        config.hold.pitch_feedforward_a * sinf(5.0f / 57.29577951308232f);
    assert(fabsf(full_current - expected_full_current) <= 1.0e-6f);
}

static void test_pitch_feedforward_deadband_applies_to_drive_brake_and_hold(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.drive.position_kp_a_per_deg = 0.0f;
    config.drive.speed_kd_a_per_rpm = 0.0f;
    config.drive.speed_ki_a_per_rpm_s = 0.0f;
    config.drive.friction_current_a = 0.0f;
    config.brake.speed_gain_a_per_rpm = 0.0f;
    config.hold.position_kp_a_per_deg = 0.0f;
    config.hold.speed_kd_a_per_rpm = 0.0f;
    config.hold_enter_time_s = 0.005f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    input.pitch_rad = config.hold.pitch_zero_offset_rad +
                      2.0f / 57.29577951308232f;
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.target_rpm[i] = 5.0f;
    }
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_DRIVE);
    assert(fabsf(output.feedforward_current_a[0]) <= 1.0e-6f);

    // 摇杆回中且轮速仍较高时进入BRAKE，死区内仍不得产生Pitch前馈
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.target_rpm[i] = 0.0f;
        input.speed_rpm[i] = 20.0f;
    }
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_BRAKE);
    assert(fabsf(output.feedforward_current_a[0]) <= 1.0e-6f);

    // 轮速降为零后进入HOLD，死区内同样不得产生Pitch前馈
    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.speed_rpm[i] = 0.0f;
    }
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_HOLD);
    assert(fabsf(output.feedforward_current_a[0]) <= 1.0e-6f);
}

static void test_pitch_offset_preserves_real_uphill_angle(void)
{
    Chassis_Hold_Config_t config = {
        .pitch_feedforward_a = 10.0f,
        .pitch_zero_offset_rad = 3.7f / 57.29577951308232f,
        .pitch_feedforward_off_pitch_rad = 3.0f / 57.29577951308232f,
        .pitch_feedforward_full_pitch_rad = 5.0f / 57.29577951308232f,
        .current_limit_a = 10.0f,
    };
    Chassis_Hold_State_t state = {0};
    Chassis_Hold_Input_t input = {
        .pitch_rad = (3.7f + 13.1f) / 57.29577951308232f,
    };
    Chassis_Hold_Output_t output;

    Chassis_Hold_Update(&state, &config, &input, &output);

    assert(fabsf(output.corrected_pitch_rad -
                 (13.1f / 57.29577951308232f)) <= 1.0e-6f);
    assert(output.pitch_current_a > 0.0f);
}

static void test_brief_hold_overspeed_does_not_latch_fault(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.005f;
    config.hold_overspeed_rpm = 50.0f;
    config.hold_overspeed_time_s = 0.100f;
    Chassis_ControlManager_Init(&manager, &config);
    input.enabled = 1U;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_HOLD);

    input.speed_rpm[2] = 51.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_HOLD);
    assert(output.fault == CHASSIS_CTRL_FAULT_NONE);

    input.speed_rpm[2] = 0.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_HOLD);
    assert(output.fault == CHASSIS_CTRL_FAULT_NONE);
}

static void test_sustained_hold_overspeed_latches_fault(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.005f;
    config.hold_overspeed_rpm = 50.0f;
    config.hold_overspeed_time_s = 0.100f;
    Chassis_ControlManager_Init(&manager, &config);
    input.enabled = 1U;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_HOLD);

    input.speed_rpm[0] = 51.0f;
    input.speed_rpm[1] = 51.0f;
    input.speed_rpm[2] = 51.0f;
    for (size_t step = 0U; step < 21U; step++)
    {
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }

    assert(output.state == CHASSIS_CTRL_FAULT);
    assert(output.fault == CHASSIS_CTRL_FAULT_OVERSPEED);
    assert_currents_are_zero(&output);
}

static void test_two_slipping_hold_wheels_do_not_drop_all_hold_current(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.005f;
    config.hold_overspeed_rpm = 50.0f;
    config.hold_overspeed_time_s = 0.100f;
    Chassis_ControlManager_Init(&manager, &config);
    input.enabled = 1U;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_HOLD);

    input.speed_rpm[1] = 60.0f;
    input.speed_rpm[2] = 60.0f;
    for (size_t step = 0U; step < 25U; step++)
    {
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }

    assert(output.state == CHASSIS_CTRL_HOLD);
    assert(output.fault == CHASSIS_CTRL_FAULT_NONE);
}

static void test_repeated_zero_command_reversals_latch_fault(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.005f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);
    input.enabled = 1U;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_HOLD);

    for (size_t step = 0U; step < 5U; step++)
    {
        const float position = ((step & 1U) == 0U) ? 100.0f : -100.0f;
        for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
        {
            input.position_deg[i] = position;
        }
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }

    assert(output.state == CHASSIS_CTRL_FAULT);
    assert(output.fault == CHASSIS_CTRL_FAULT_OSCILLATION);
    assert_currents_are_zero(&output);
}

static void test_small_hold_current_reversals_do_not_latch_fault(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.005f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);
    input.enabled = 1U;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_HOLD);

    for (size_t step = 0U; step < 8U; step++)
    {
        const float position = ((step & 1U) == 0U) ? 10.0f : -10.0f;
        for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
        {
            input.position_deg[i] = position;
        }
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }

    assert(output.state == CHASSIS_CTRL_HOLD);
    assert(output.fault == CHASSIS_CTRL_FAULT_NONE);
}

static void test_brake_reversals_do_not_latch_oscillation_fault(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 1.0f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    input.target_rpm[0] = 100.0f;
    input.speed_rpm[0] = 100.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_DRIVE);

    input.target_rpm[0] = 0.0f;
    for (size_t step = 0U; step < 8U; step++)
    {
        input.speed_rpm[0] = ((step & 1U) == 0U) ? 100.0f : -100.0f;
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }

    assert(output.state == CHASSIS_CTRL_BRAKE);
    assert(output.fault == CHASSIS_CTRL_FAULT_NONE);
}

static void test_pitch_feedforward_saturation_does_not_latch_fault(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.005f;
    config.hold.position_kp_a_per_deg = 0.0f;
    config.hold.speed_kd_a_per_rpm = 0.0f;
    config.hold.pitch_feedforward_a = 10.0f;
    config.hold.current_limit_a = 0.5f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);

    input.enabled = 1U;
    input.pitch_rad = config.hold.pitch_zero_offset_rad + 1.57079632679f;
    for (size_t step = 0U; step < 30U; step++)
    {
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }

    assert(output.state == CHASSIS_CTRL_HOLD);
    assert(output.fault == CHASSIS_CTRL_FAULT_NONE);
    assert(output.current_a[0] > 0.49f);
}

static void test_sustained_hold_saturation_latches_fault(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.005f;
    config.current_rise_a_per_s = 1000.0f;
    config.current_release_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);
    input.enabled = 1U;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.position_deg[i] = -(config.hold.current_limit_a /
                                  config.hold.position_kp_a_per_deg + 1.0f);
    }
    for (size_t step = 0U; step < 21U; step++)
    {
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }

    assert(output.state == CHASSIS_CTRL_FAULT);
    assert(output.fault == CHASSIS_CTRL_FAULT_SATURATION);
    assert_currents_are_zero(&output);
}

static void test_repeated_bad_control_period_latches_fault(void)
{
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_Init(&manager, NULL);
    input.enabled = 1U;
    input.target_rpm[0] = 20.0f;
    for (size_t step = 0U; step < 3U; step++)
    {
        Chassis_ControlManager_Update(&manager, &input, 0.020f, &output);
    }

    assert(output.state == CHASSIS_CTRL_FAULT);
    assert(output.fault == CHASSIS_CTRL_FAULT_TIMING);
    assert_currents_are_zero(&output);
}

int main(void)
{
    test_mit_speed_integral_accumulates_persistent_error();
    test_mit_speed_integral_is_bounded();
    test_mit_speed_integral_stops_at_output_saturation();
    test_mit_speed_integral_resets_on_target_reversal();
    test_mit_speed_integral_clears_at_zero_target();
    test_mit_speed_error_channel_has_independent_limit();
    test_mit_slip_limited_wheel_drops_tracking_current();
    test_disabled_output_is_zero();
    test_nonzero_command_enters_bounded_drive();
    test_pure_turn_breakaway_current_is_separate_from_rolling_friction();
    test_pure_turn_breakaway_does_not_reengage_above_enter_speed();
    test_pure_turn_breakaway_taper_releases_dragging_wheel();
    test_pure_turn_sync_scales_all_wheels_to_slowest();
    test_drive_slip_limit_disables_dragging_wheel_tracking();
    test_drive_keeps_pitch_feedforward_on_slope();
    test_zero_command_brakes_before_hold();
    test_release_captures_position_and_compensates_pitch_during_brake();
    test_flat_brake_disables_position_compensation();
    test_uphill_and_downhill_enable_brake_position_compensation();
    test_brake_position_compensation_blends_between_three_and_five_degrees();
    test_flat_brake_to_hold_uses_stop_position();
    test_steep_slope_brake_to_hold_preserves_release_position();
    test_brake_to_hold_blends_reference_in_pitch_transition();
    test_one_speed_outlier_does_not_block_hold_entry();
    test_high_speed_release_enters_brake_instead_of_fault();
    test_feedback_loss_latches_fault();
    test_drive_current_reversal_releases_fast_then_ramps_opposite();
    test_hold_closes_each_wheel_position_independently();
    test_hold_damps_a_slipping_wheel_independently();
    test_pitch_offset_removes_flat_table_bias();
    test_pitch_feedforward_uses_three_to_five_degree_smooth_deadband();
    test_pitch_feedforward_deadband_applies_to_drive_brake_and_hold();
    test_pitch_offset_preserves_real_uphill_angle();
    test_brief_hold_overspeed_does_not_latch_fault();
    test_sustained_hold_overspeed_latches_fault();
    test_two_slipping_hold_wheels_do_not_drop_all_hold_current();
    test_repeated_zero_command_reversals_latch_fault();
    test_small_hold_current_reversals_do_not_latch_fault();
    test_brake_reversals_do_not_latch_oscillation_fault();
    test_pitch_feedforward_saturation_does_not_latch_fault();
    test_sustained_hold_saturation_latches_fault();
    test_repeated_bad_control_period_latches_fault();
    return 0;
}
