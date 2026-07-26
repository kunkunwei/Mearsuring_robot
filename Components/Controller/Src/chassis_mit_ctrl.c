#include "chassis_mit_ctrl.h"
#include "chassis_brake.h"
#include "mymotor.h"

static fp32 mit_abs(fp32 value)
{
    return (value >= 0.0f) ? value : -value;
}

static fp32 mit_sign(fp32 value)
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

static fp32 mit_limit(fp32 value, fp32 min_value, fp32 max_value)
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

static fp32 mit_feedforward_current(Chassis_Motor_t *motor, const PidTypeDef *pid)
{
    if (motor == NULL || pid == NULL)
    {
        return 0.0f;
    }

    const int8_t target_sign = (int8_t)mit_sign(motor->speed_set_rpm);
    if (target_sign == 0)
    {
        motor->ff_breakaway_active = 1U;
        motor->ff_last_sign = 0;
        return 0.0f;
    }

    if (target_sign != motor->ff_last_sign)
    {
        motor->ff_breakaway_active = 1U;
        motor->ff_last_sign = target_sign;
    }

    const fp32 abs_feedback_rpm = mit_abs(motor->speed_rpm);
    if (abs_feedback_rpm <= CHASSIS_MIT_BREAKAWAY_ENTER_RPM)
    {
        motor->ff_breakaway_active = 1U;
    }
    else if (abs_feedback_rpm >= CHASSIS_MIT_BREAKAWAY_EXIT_RPM)
    {
        motor->ff_breakaway_active = 0U;
    }

    const fp32 ff_current = (motor->ff_breakaway_active != 0U) ?
                            pid->Kd :
                            (pid->Kd * CHASSIS_MIT_RUNNING_FF_RATIO);
    return ff_current * (fp32)target_sign;
}

void Chassis_Mit_ResetMotor(Chassis_Motor_t *motor, PidTypeDef *pid)
{
    if (motor == NULL || pid == NULL)
    {
        return;
    }

    motor->pos_set_deg = motor->pos_deg;
    motor->target_current = 0;
    motor->ff_breakaway_active = 1U;
    motor->ff_last_sign = 0;
    Chassis_Brake_Reset(motor);
    old_PID_clear(pid);
}

void Chassis_Mit_CurrentControl(Chassis_Motor_t *motor, PidTypeDef *pid, fp32 brake_ref_abs_speed_rpm)
{
    if (motor == NULL || pid == NULL)
    {
        return;
    }

    if (!vesc_motor_status_is_online(motor->chassis_motor_measure, CHASSIS_MOTOR_STATUS1_TIMEOUT_MS))
    {
        Chassis_Mit_ResetMotor(motor, pid);
        return;
    }

    if (mit_abs(motor->speed_set_rpm) <= CHASSIS_MIT_ACTIVE_RPM_THRESHOLD)
    {
        motor->pos_set_deg = motor->pos_deg;
        motor->ff_breakaway_active = 1U;
        motor->ff_last_sign = 0;
        Chassis_Brake_Update(motor, pid, brake_ref_abs_speed_rpm);
        return;
    }

    Chassis_Brake_Reset(motor);
    motor->pos_set_deg += motor->speed_set_rpm * 6.0f * CHASSIS_CONTROL_TIME;

    fp32 pos_error_deg = motor->pos_set_deg - motor->pos_deg;
    if (!vesc_motor_status4_is_online(motor->chassis_motor_measure, CHASSIS_MOTOR_STATUS4_TIMEOUT_MS))
    {
        pos_error_deg = 0.0f;
        motor->pos_set_deg = motor->pos_deg;
    }
    else
    {
        pos_error_deg = mit_limit(pos_error_deg,
                                  -CHASSIS_MIT_POS_ERROR_MAX_DEG,
                                  CHASSIS_MIT_POS_ERROR_MAX_DEG);
        motor->pos_set_deg = motor->pos_deg + pos_error_deg;
    }

    const fp32 speed_error_rpm = motor->speed_set_rpm - motor->speed_rpm;

    pid->set = motor->speed_set_rpm;
    pid->fdb = motor->speed_rpm;
    pid->error[0] = speed_error_rpm;
    pid->Pout = pid->Kp * pos_error_deg;
    pid->Dout = pid->Ki * speed_error_rpm;
    pid->Iout = mit_feedforward_current(motor, pid);

    const fp32 current_set = pid->Pout + pid->Dout + pid->Iout;
    pid->out = mit_limit(current_set, -M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_OUT);
    motor->target_current = (int16_t)pid->out;
}
