#include "chassis_control_manager.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

static Chassis_Control_Input_t valid_input(void)
{
    Chassis_Control_Input_t input = {0};
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
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_Init(&manager, NULL);
    input.enabled = 1U;
    input.target_rpm[0] = 20.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_DRIVE);
    assert(output.current_a[0] > 0.0f);
    assert(output.current_a[0] <= 0.05f + 1.0e-6f);
    assert(fabsf(output.current_a[0]) <= 0.5f + 1.0e-6f);
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

static void test_high_speed_release_enters_brake_instead_of_fault(void)
{
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_Init(&manager, NULL);
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

static void test_drive_current_reversal_passes_through_zero(void)
{
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();
    float previous_current = 0.0f;
    uint8_t crossed_zero = 0U;

    Chassis_ControlManager_Init(&manager, NULL);
    input.enabled = 1U;
    for (size_t step = 0U; step < 12U; step++)
    {
        input.target_rpm[0] = 20.0f;
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }
    assert(output.current_a[0] > 0.0f);

    previous_current = output.current_a[0];
    for (size_t step = 0U; step < 30U; step++)
    {
        input.target_rpm[0] = -20.0f;
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
        assert(fabsf(output.current_a[0] - previous_current) <= 0.05f + 1.0e-6f);
        if (previous_current >= 0.0f && output.current_a[0] <= 0.0f)
        {
            assert(fabsf(previous_current) <= 0.05f + 1.0e-6f);
            crossed_zero = 1U;
        }
        previous_current = output.current_a[0];
    }
    assert(crossed_zero != 0U);
    assert(output.current_a[0] < 0.0f);
}

static void enter_hold(Chassis_Control_Manager_t *manager,
                       Chassis_Control_Input_t *input,
                       Chassis_Control_Output_t *output)
{
    input->enabled = 1U;
    for (size_t step = 0U; step < 40U; step++)
    {
        Chassis_ControlManager_Update(manager, input, 0.005f, output);
    }
    assert(output->state == CHASSIS_CTRL_HOLD);
}

static void test_hold_uses_one_common_longitudinal_current(void)
{
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_Init(&manager, NULL);
    enter_hold(&manager, &input, &output);

    input.position_deg[0] = 1.0f;
    input.position_deg[1] = 2.0f;
    input.position_deg[2] = 100.0f;
    input.position_deg[3] = 3.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_HOLD);
    assert(output.raw_current_a[0] < 0.0f);
    for (size_t i = 1U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        assert(fabsf(output.raw_current_a[i] - output.raw_current_a[0]) < 1.0e-6f);
    }
}

static void test_hold_overspeed_latches_fault(void)
{
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_Init(&manager, NULL);
    enter_hold(&manager, &input, &output);
    input.enabled = 1U;
    input.speed_rpm[2] = 51.0f;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    assert(output.state == CHASSIS_CTRL_FAULT);
    assert(output.fault == CHASSIS_CTRL_FAULT_OVERSPEED);
    assert_currents_are_zero(&output);
}

static void test_repeated_zero_command_reversals_latch_fault(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.005f;
    config.current_slew_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);
    input.enabled = 1U;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    assert(output.state == CHASSIS_CTRL_HOLD);

    for (size_t step = 0U; step < 5U; step++)
    {
        const float position = ((step & 1U) == 0U) ? 10.0f : -10.0f;
        for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
        {
            input.position_deg[i] = position;
        }
        Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);
    }

    assert(output.state == CHASSIS_CTRL_FAULT);
    assert(output.fault == CHASSIS_CTRL_FAULT_OSCILLATION);
    assert_currents_are_zero(&output);
}

static void test_sustained_hold_saturation_latches_fault(void)
{
    Chassis_Control_Config_t config;
    Chassis_Control_Manager_t manager;
    Chassis_Control_Output_t output;
    Chassis_Control_Input_t input = valid_input();

    Chassis_ControlManager_DefaultConfig(&config);
    config.hold_enter_time_s = 0.005f;
    config.current_slew_a_per_s = 1000.0f;
    Chassis_ControlManager_Init(&manager, &config);
    input.enabled = 1U;
    Chassis_ControlManager_Update(&manager, &input, 0.005f, &output);

    for (size_t i = 0U; i < CHASSIS_CONTROL_MOTOR_COUNT; i++)
    {
        input.position_deg[i] = -100.0f;
    }
    for (size_t step = 0U; step < 20U; step++)
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
    test_disabled_output_is_zero();
    test_nonzero_command_enters_bounded_drive();
    test_zero_command_brakes_before_hold();
    test_high_speed_release_enters_brake_instead_of_fault();
    test_feedback_loss_latches_fault();
    test_drive_current_reversal_passes_through_zero();
    test_hold_uses_one_common_longitudinal_current();
    test_hold_overspeed_latches_fault();
    test_repeated_zero_command_reversals_latch_fault();
    test_sustained_hold_saturation_latches_fault();
    test_repeated_bad_control_period_latches_fault();
    return 0;
}
