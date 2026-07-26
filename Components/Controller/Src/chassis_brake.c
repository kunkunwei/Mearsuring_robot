#include "chassis_brake.h"

typedef struct
{
    fp32 done_rpm;
    fp32 max_current;
    fp32 min_current;
    fp32 current_per_rpm;
    fp32 speed_ref_decel_rpm_per_s;
    fp32 current_step;
    fp32 release_step;
    fp32 abs_ref_rpm;
    fp32 abs_ratio;
    fp32 abs_drop_rpm;
    fp32 abs_release_ratio;
} Chassis_Brake_Config_t;

static const Chassis_Brake_Config_t brake_cfg = {
    .done_rpm = 5.0f,
    .max_current = 1200.0f,
    .min_current = 150.0f,
    .current_per_rpm = 9.0f,
    .speed_ref_decel_rpm_per_s = 2500.0f,
    .current_step = 22.0f,
    .release_step = 240.0f,
    .abs_ref_rpm = 35.0f,
    .abs_ratio = 0.45f,
    .abs_drop_rpm = 20.0f,
    .abs_release_ratio = 0.20f,
};

static fp32 brake_abs(fp32 value)
{
    return (value >= 0.0f) ? value : -value;
}

static fp32 brake_sign(fp32 value)
{
    if (value > 0.0f)
    {
        return 1.0f;
    }
    if (value < 0.0f)
    {
        return -1.0f;
    }
    return 0.0f;
}

static fp32 brake_limit(fp32 value, fp32 min_value, fp32 max_value)
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

static fp32 brake_slew_rate_limit(fp32 target, fp32 current, fp32 step)
{
    if (target > current + step)
    {
        return current + step;
    }
    if (target < current - step)
    {
        return current - step;
    }
    return target;
}

static fp32 brake_speed_ref_update(Chassis_Motor_t *motor)
{
    const fp32 speed_rpm = motor->speed_rpm;
    const fp32 decel_step = brake_cfg.speed_ref_decel_rpm_per_s * CHASSIS_CONTROL_TIME;

    if (brake_abs(motor->brake_speed_ref_rpm) <= brake_cfg.done_rpm ||
        brake_sign(motor->brake_speed_ref_rpm) != brake_sign(speed_rpm))
    {
        motor->brake_speed_ref_rpm = speed_rpm;
    }

    motor->brake_speed_ref_rpm = brake_slew_rate_limit(0.0f,
                                                       motor->brake_speed_ref_rpm,
                                                       decel_step);
    return motor->brake_speed_ref_rpm;
}

static fp32 brake_current_limit(fp32 abs_speed_rpm)
{
    fp32 current_limit = abs_speed_rpm * brake_cfg.current_per_rpm;

    if (abs_speed_rpm > brake_cfg.done_rpm && current_limit < brake_cfg.min_current)
    {
        current_limit = brake_cfg.min_current;
    }

    return brake_limit(current_limit, 0.0f, brake_cfg.max_current);
}

static uint8_t brake_should_release(const Chassis_Motor_t *motor, fp32 ref_abs_speed_rpm)
{
    if (motor == NULL || ref_abs_speed_rpm < brake_cfg.abs_ref_rpm)
    {
        return 0U;
    }

    const fp32 abs_speed_rpm = brake_abs(motor->speed_rpm);
    const fp32 last_abs_speed_rpm = brake_abs(motor->last_speed_rpm);
    const uint8_t slower_than_reference =
        (abs_speed_rpm < ref_abs_speed_rpm * brake_cfg.abs_ratio) ? 1U : 0U;
    const uint8_t speed_dropped_too_fast =
        (last_abs_speed_rpm > abs_speed_rpm + brake_cfg.abs_drop_rpm) ? 1U : 0U;

    return (slower_than_reference != 0U || speed_dropped_too_fast != 0U) ? 1U : 0U;
}

void Chassis_Brake_Reset(Chassis_Motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->brake_current_cmd = 0.0f;
    motor->brake_speed_ref_rpm = 0.0f;
}

void Chassis_Brake_Update(Chassis_Motor_t *motor, PidTypeDef *pid, fp32 ref_abs_speed_rpm)
{
    if (motor == NULL || pid == NULL)
    {
        return;
    }

    if (brake_abs(motor->speed_rpm) <= brake_cfg.done_rpm)
    {
        motor->target_current = 0;
        Chassis_Brake_Reset(motor);
        old_PID_clear(pid);
        return;
    }

    const fp32 brake_speed_ref_rpm = brake_speed_ref_update(motor);
    const fp32 speed_error_rpm = brake_speed_ref_rpm - motor->speed_rpm;
    const fp32 brake_cmd_sign = brake_sign(speed_error_rpm);
    const fp32 current_limit = brake_current_limit(brake_abs(motor->speed_rpm));
    fp32 brake_current = brake_limit(brake_cfg.current_per_rpm * speed_error_rpm,
                                     -current_limit,
                                     current_limit);

    const uint8_t abs_release = brake_should_release(motor, ref_abs_speed_rpm);
    if (abs_release != 0U)
    {
        brake_current *= brake_cfg.abs_release_ratio;
    }

    if (brake_cmd_sign != 0.0f &&
        brake_abs(brake_current) < brake_cfg.min_current &&
        brake_abs(motor->speed_rpm) > brake_cfg.done_rpm)
    {
        brake_current = brake_cmd_sign * brake_cfg.min_current;
    }

    const fp32 current_step = (abs_release != 0U &&
                              brake_abs(brake_current) < brake_abs(motor->brake_current_cmd)) ?
                              brake_cfg.release_step :
                              brake_cfg.current_step;

    motor->brake_current_cmd = brake_slew_rate_limit(brake_current,
                                                     motor->brake_current_cmd,
                                                     current_step);

    pid->set = brake_speed_ref_rpm;
    pid->fdb = motor->speed_rpm;
    pid->error[0] = speed_error_rpm;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = motor->brake_current_cmd;
    pid->out = motor->brake_current_cmd;
    motor->target_current = (int16_t)pid->out;
}
