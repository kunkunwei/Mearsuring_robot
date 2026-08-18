#include "Chassis_task.h"
#include "main.h"
#include "remote_control.h"
#include "observe_task.h"
#include "minipc.h"
// ============================================================
#include "bsp_dwt.h"
#include "monitor.h"
#include "mymotor.h"
#include "usart.h"
#include "vofa.h"

// ============================================================

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
// 闄愬箙鍑芥暟
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

static fp32 chassis_abs(fp32 value)
{
    return (value >= 0.0f) ? value : -value;
}

static fp32 chassis_sign(fp32 value)
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

static fp32 chassis_wrap_deg(fp32 angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static void chassis_mit_control_reset(Chassis_Motor_t *motor, PidTypeDef *mit_param)
{
    if (motor == NULL || mit_param == NULL)
    {
        return;
    }

    motor->pos_set_deg = motor->pos_deg;
    motor->target_current = 0;
    motor->ff_breakaway_active = 1U;
    motor->ff_last_sign = 0;
    old_PID_clear(mit_param);
}

static void chassis_mit_control_reset_all(chassis_move_t *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        chassis->chassis_motor[i].speed_set = 0.0f;
        chassis->chassis_motor[i].speed_set_rpm = 0.0f;
        chassis_mit_control_reset(&chassis->chassis_motor[i],
                                  &chassis->chassis_pid.motor_speed_pid[i]);
    }
    old_PID_clear(&chassis->chassis_pid.chassis_yaw_gyro_pid);
}

static fp32 chassis_mit_feedforward_current(Chassis_Motor_t *motor, const PidTypeDef *mit_param)
{
    if (motor == NULL || mit_param == NULL)
    {
        return 0.0f;
    }

    const int8_t target_sign = (int8_t)chassis_sign(motor->speed_set_rpm);
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

    const fp32 abs_feedback_rpm = chassis_abs(motor->speed_rpm);
    if (abs_feedback_rpm <= CHASSIS_MIT_BREAKAWAY_ENTER_RPM)
    {
        motor->ff_breakaway_active = 1U;
    }
    else if (abs_feedback_rpm >= CHASSIS_MIT_BREAKAWAY_EXIT_RPM)
    {
        motor->ff_breakaway_active = 0U;
    }

    const fp32 ff_current = (motor->ff_breakaway_active != 0U) ?
                            mit_param->Kd :
                            (mit_param->Kd * CHASSIS_MIT_RUNNING_FF_RATIO);
    return ff_current * (fp32)target_sign;
}

static void chassis_mit_stop_brake_control(Chassis_Motor_t *motor, PidTypeDef *mit_param)
{
    if (motor == NULL || mit_param == NULL)
    {
        return;
    }

    if (chassis_abs(motor->speed_rpm) <= CHASSIS_STOP_BRAKE_DONE_RPM)
    {
        chassis_mit_control_reset(motor, mit_param);
        return;
    }

    motor->pos_set_deg = motor->pos_deg;
    motor->ff_breakaway_active = 1U;
    motor->ff_last_sign = 0;

    const fp32 speed_error_rpm = -motor->speed_rpm;

    mit_param->set = 0.0f;
    mit_param->fdb = motor->speed_rpm;
    mit_param->error[0] = speed_error_rpm;
    mit_param->Pout = 0.0f;
    mit_param->Dout = mit_param->Ki * speed_error_rpm;
    mit_param->Iout = 0.0f;
    mit_param->out = fp32_constrain(mit_param->Dout,
                                    -CHASSIS_STOP_BRAKE_MAX_CURRENT,
                                    CHASSIS_STOP_BRAKE_MAX_CURRENT);
    motor->target_current = (int16_t)mit_param->out;
}

static void chassis_mit_current_control(Chassis_Motor_t *motor, PidTypeDef *mit_param)
{
    if (motor == NULL || mit_param == NULL)
    {
        return;
    }

    if (!vesc_motor_status_is_online(motor->chassis_motor_measure, CHASSIS_MOTOR_STATUS1_TIMEOUT_MS))
    {
        chassis_mit_control_reset(motor, mit_param);
        return;
    }

    if (chassis_abs(motor->speed_set_rpm) <= CHASSIS_MIT_ACTIVE_RPM_THRESHOLD)
    {
        chassis_mit_stop_brake_control(motor, mit_param);
        return;
    }

    motor->pos_set_deg += motor->speed_set_rpm * 6.0f * CHASSIS_CONTROL_TIME;

    fp32 pos_error_deg = motor->pos_set_deg - motor->pos_deg;
    if (!vesc_motor_status4_is_online(motor->chassis_motor_measure, CHASSIS_MOTOR_STATUS4_TIMEOUT_MS))
    {
        pos_error_deg = 0.0f;
        motor->pos_set_deg = motor->pos_deg;
    }
    else
    {
        pos_error_deg = fp32_constrain(pos_error_deg,
                                       -CHASSIS_MIT_POS_ERROR_MAX_DEG,
                                       CHASSIS_MIT_POS_ERROR_MAX_DEG);
        motor->pos_set_deg = motor->pos_deg + pos_error_deg;
    }
    const fp32 speed_error_rpm = motor->speed_set_rpm - motor->speed_rpm;

    mit_param->set = motor->speed_set_rpm;
    mit_param->fdb = motor->speed_rpm;
    mit_param->error[0] = speed_error_rpm;
    mit_param->Pout = mit_param->Kp * pos_error_deg;
    mit_param->Dout = mit_param->Ki * speed_error_rpm;
    mit_param->Iout = chassis_mit_feedforward_current(motor, mit_param);

    const fp32 current_set = mit_param->Pout + mit_param->Dout + mit_param->Iout;
    mit_param->out = fp32_constrain(current_set, -M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_OUT);
    motor->target_current = (int16_t)mit_param->out;
}

static void chassis_hold_test_reset(chassis_move_t *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        chassis->chassis_motor[i].hold_pos_ready = 0U;
        chassis->chassis_motor[i].hold_pos_error_deg = 0.0f;
        old_PID_clear(&chassis->chassis_pid.motor_speed_pid[i]);
    }
}

static void chassis_hold_position_control(Chassis_Motor_t *motor, PidTypeDef *hold_param)
{
    if (motor == NULL || hold_param == NULL)
    {
        return;
    }

    motor->speed_set = 0.0f;
    motor->speed_set_rpm = 0.0f;

    if (!vesc_motor_status_is_online(motor->chassis_motor_measure, CHASSIS_MOTOR_STATUS1_TIMEOUT_MS) ||
        !vesc_motor_status4_is_online(motor->chassis_motor_measure, CHASSIS_MOTOR_STATUS4_TIMEOUT_MS))
    {
        chassis_mit_control_reset(motor, hold_param);
        motor->hold_pos_ready = 0U;
        motor->hold_pos_error_deg = 0.0f;
        return;
    }

    if (motor->hold_pos_ready == 0U)
    {
        motor->hold_pos_ready = 1U;
        motor->hold_pos_ref_deg = motor->pos_deg;
        motor->hold_pos_error_deg = 0.0f;
        motor->target_current = 0;
        old_PID_clear(hold_param);
        return;
    }

    fp32 pos_error_deg = motor->hold_pos_ref_deg - motor->pos_deg;
    pos_error_deg = fp32_constrain(pos_error_deg,
                                   -CHASSIS_HOLD_POS_ERROR_MAX_DEG,
                                   CHASSIS_HOLD_POS_ERROR_MAX_DEG);
    motor->hold_pos_error_deg = pos_error_deg;

    const fp32 speed_damping = -hold_param->Ki * motor->speed_rpm;
    const fp32 static_ff = (chassis_abs(pos_error_deg) > CHASSIS_HOLD_STATIC_DEADBAND_DEG) ?
                           (hold_param->Kd * chassis_sign(pos_error_deg)) :
                           0.0f;

    hold_param->set = 0.0f;
    hold_param->fdb = motor->pos_deg - motor->hold_pos_ref_deg;
    hold_param->error[0] = pos_error_deg;
    hold_param->Pout = hold_param->Kp * pos_error_deg;
    hold_param->Dout = speed_damping;
    hold_param->Iout = static_ff;
    hold_param->out = fp32_constrain(hold_param->Pout + hold_param->Dout + hold_param->Iout,
                                     -M3505_MOTOR_SPEED_PID_MAX_OUT,
                                     M3505_MOTOR_SPEED_PID_MAX_OUT);
    motor->target_current = (int16_t)hold_param->out;
}

static void chassis_hold_test_control(chassis_move_t *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    chassis->state_set.vx = 0.0f;
    chassis->state_set.wz = 0.0f;
    old_PID_clear(&chassis->chassis_pid.chassis_yaw_gyro_pid);

    for (uint8_t i = 0U; i < 4U; i++)
    {
        chassis_hold_position_control(&chassis->chassis_motor[i],
                                      &chassis->chassis_pid.motor_speed_pid[i]);
    }
}

static void chassis_limit_wheel_speed_keep_ratio(fp32 wheel_speed[4])
{
    fp32 max_abs_speed = 0.0f;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        const fp32 abs_speed = chassis_abs(wheel_speed[i]);
        if (abs_speed > max_abs_speed)
        {
            max_abs_speed = abs_speed;
        }
    }

    if (max_abs_speed <= MAX_WHEEL_SPEED)
    {
        return;
    }

    const fp32 scale = MAX_WHEEL_SPEED / max_abs_speed;
    for (uint8_t i = 0U; i < 4U; i++)
    {
        wheel_speed[i] *= scale;
    }
}

static void chassis_apply_turn_min_rpm(chassis_move_t *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    if (chassis_abs(chassis->state_set.vx) > CHASSIS_TURN_MIN_VX_THRESHOLD ||
        chassis_abs(chassis->state_set.wz) < CHASSIS_TURN_MIN_WZ_THRESHOLD)
    {
        return;
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        Chassis_Motor_t *motor = &chassis->chassis_motor[i];
        const fp32 target_sign = chassis_sign(motor->speed_set_rpm);
        if (target_sign == 0.0f)
        {
            continue;
        }

        if (chassis_abs(motor->speed_set_rpm) < CHASSIS_TURN_MIN_RPM)
        {
            motor->speed_set_rpm = target_sign * CHASSIS_TURN_MIN_RPM;
            motor->speed_set = motor->speed_set_rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
        }
    }
}
// 搴曠洏杩愬姩鏁版嵁
static chassis_move_t chassis_move;
static volatile bool chassis_init_done = false; // 鍒濆鍖栧畬鎴愭爣蹇楋紝缃綅鍚庡叾浠栦换鍔℃墠鍙畨鍏ㄨ闂?chassis 鍐呮寚閽?// 浠庝簯鍙伴€氫俊鑾峰彇鎺у埗鍛戒护

// ============================================================

static void chassis_init(chassis_move_t *chassis_move_init);
void chassis_set_mode(chassis_move_t *chassis_move_mode);
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
void chassis_set_contorl(chassis_move_t *chassis_move_control);
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
// ============================================================


void Chassis_Task(void const *argument)
{
    /* USER CODE BEGIN Chassis_Task */
    /* Infinite loop */
    // osDelay(100);
    TickType_t systick = 0;

    chassis_init(&chassis_move);
    chassis_init_done=true;

    for (;;)
    {
        systick = osKernelSysTick();
        chassis_set_mode(&chassis_move);
        chassis_mode_change_control_transit(&chassis_move);
        chassis_feedback_update(&chassis_move);
        chassis_set_contorl(&chassis_move);
        chassis_control_loop(&chassis_move);
        SystemMonitor_Process();
        osDelayUntil(&systick, CHASSIS_CONTROL_TIME_MS);
    }
    /* USER CODE END Chassis_Task */
}

/**
 * @brief          搴曠洏鍒濆鍖? * @author         pxx
 * @param          chassis_move_init   搴曠洏缁撴瀯浣撴寚閽? * @retval         void
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    uint8_t i;
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    const static fp32 chassis_yaw_gyro_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    //搴曠洏閫熷害鐜痯id鍊?    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    // 搴曠洏鏃嬭浆鐜痯id鍊?    const static fp32 chassis_yaw_gyro_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    // 涓€闃朵綆閫氭护娉㈠垵濮嬪寲
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};

    // 搴曠洏寮€鏈虹姸鎬佷负鏃犲姏銆佷笉鎺ヨЕ鍦伴潰
    chassis_move_init->mode.chassis_mode = CHASSIS_FORCE_RAW;
    chassis_move_init->mode.last_chassis_mode = CHASSIS_FORCE_RAW;
    chassis_move_init->mode.last_normol_channel = 0;

    // 鑾峰彇閬ユ帶鍣ㄦ寚閽?    chassis_move_init->chassis_RC = get_remote_control_point();
    chassis_move_init->chassis_RC = get_remote_control_point();
    // 鑾峰彇闄€铻轰华濮挎€佽鎸囬拡
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    chassis_move_init->chassis_imu_gyro = get_gyro_data_point();
    chassis_move_init->chassis_imu_accel = get_accel_data_point();

    //鍒濆鍖朠ID 杩愬姩
    for (i = 0; i < 4; i++)
    {   // 鑾峰彇搴曠洏椹卞姩杞數鏈烘寚閽?        chassis_move_init->chassis_motor[i].chassis_motor_measure = get_chassis_motor(i);
        chassis_move_init->chassis_motor[i].chassis_motor_measure = get_chassis_motor(i);
        old_PID_Init(&chassis_move_init->chassis_pid.motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    // 鍒濆鍖栨棆杞琍ID
    old_PID_Init(&chassis_move_init->chassis_pid.chassis_yaw_gyro_pid, PID_POSITION, chassis_yaw_gyro_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);

    //鐢ㄤ竴闃舵护娉唬鏇挎枩娉㈠嚱鏁扮敓鎴?    first_order_filter_init(&chassis_move_init->state_set.chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    // 鍒濆鍖朩Z婊ゆ尝鍣?    first_order_filter_init(&chassis_move_init->state_set.chassis_cmd_slow_set_wz, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->state_set.chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->state_set.chassis_cmd_slow_set_wz, CHASSIS_CONTROL_TIME, chassis_x_order_filter);


    // ============================================================
    // 鏇存柊涓€涓嬫暟鎹?    chassis_feedback_update(chassis_move_init);
}
/**
 * @brief          搴曠洏鏁版嵁鏇存柊
 * @author         pxx
 * @param          chassis_move_update 搴曠洏缁撴瀯浣撴寚閽? * @retval         void
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
    const fp32 motor_forward_sign[4] = {
        CHASSIS_MOTOR_1_FORWARD_SIGN,
        CHASSIS_MOTOR_2_FORWARD_SIGN,
        CHASSIS_MOTOR_3_FORWARD_SIGN,
        CHASSIS_MOTOR_4_FORWARD_SIGN,
    };
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //鏇存柊鐢垫満閫熷害锛堝皢RPM杞崲涓簃/s锛?        // M3508鐢垫満锛歊PM杞崲涓簉ad/s鐨勭郴鏁版槸 2蟺/60 = 0.10471975512
        chassis_move_update->chassis_motor[i].speed_rpm = chassis_move_update->chassis_motor[i].chassis_motor_measure->rpm *
                                                          motor_forward_sign[i];
        chassis_move_update->chassis_motor[i].speed = chassis_move_update->chassis_motor[i].speed_rpm *
                                                      CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
        chassis_move_update->chassis_motor[i].accel = chassis_move_update->chassis_pid.motor_speed_pid[i].Dbuf[0] *
                                                      CHASSIS_CONTROL_FREQUENCE *
                                                      CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
        if (chassis_move_update->chassis_motor[i].chassis_motor_measure->status4_tick !=
            chassis_move_update->chassis_motor[i].last_status4_tick)
        {
            const fp32 pos_raw_deg = chassis_move_update->chassis_motor[i].chassis_motor_measure->pid_pos_deg *
                                     motor_forward_sign[i];
            chassis_move_update->chassis_motor[i].last_status4_tick =
                chassis_move_update->chassis_motor[i].chassis_motor_measure->status4_tick;
            if (chassis_move_update->chassis_motor[i].pos_ready == 0U)
            {
                chassis_move_update->chassis_motor[i].pos_ready = 1U;
                chassis_move_update->chassis_motor[i].last_pos_raw_deg = pos_raw_deg;
                chassis_move_update->chassis_motor[i].pos_deg = 0.0f;
                chassis_move_update->chassis_motor[i].pos_set_deg = 0.0f;
            }
            else
            {
                const fp32 pos_delta_deg = chassis_wrap_deg(pos_raw_deg - chassis_move_update->chassis_motor[i].last_pos_raw_deg);
                chassis_move_update->chassis_motor[i].pos_deg += pos_delta_deg;
                chassis_move_update->chassis_motor[i].last_pos_raw_deg = pos_raw_deg;
            }
        }
    }

    // 4WD differential drive, body frame: +X forward, +Y left, +WZ counter-clockwise.
    const fp32 left_speed = (chassis_move_update->chassis_motor[0].speed + chassis_move_update->chassis_motor[3].speed) / 2.0f;
    const fp32 right_speed = (chassis_move_update->chassis_motor[1].speed + chassis_move_update->chassis_motor[2].speed) / 2.0f;

    chassis_move_update->state_ref.vx = (left_speed + right_speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->state_ref.wz = (right_speed - left_speed) / (2.0f * MOTOR_DISTANCE_TO_CENTER);

}
/**
 * @brief          璁剧疆搴曠洏妯″紡锛堟敮鎸佷簯鍙版帶鍒跺拰閬ユ帶鍣ㄦ帶鍒跺弻妯″紡锛? * @author         pxx
 * @param          chassis_move_mode   搴曠洏缁撴瀯浣撴寚閽? * @retval         void
 */
void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[FUNCTION_CHANNEL]) &&
        !switch_is_down(chassis_move_mode->mode.last_normol_channel))
    {
        chassis_odom_reset();
    }

    if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_move_mode->mode.chassis_mode = CHASSIS_FORCE_RAW;
    }
    else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_move_mode->mode.chassis_mode = CHASSIS_MANL_CTRL ;
    }
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_move_mode->mode.chassis_mode = CHASSIS_HOLD_TEST;
    }
    else
    {
        chassis_move_mode->mode.chassis_mode = CHASSIS_FORCE_RAW;
    }

    // 鏇存柊涓婃閬ユ帶鍣ㄦā寮忕姸鎬?   chassis_move_mode->mode.last_chassis_mode = chassis_move_mode->mode.chassis_mode;
    chassis_move_mode->mode.last_normol_channel = chassis_move_mode->chassis_RC->rc.s[FUNCTION_CHANNEL];

}
/**
 * @brief          閬ユ帶鍣ㄧ姸鎬佸垏鎹㈡暟鎹繚瀛? * @author         pxx
 * @param          chassis_move_transit    搴曠洏缁撴瀯浣撴寚閽? * @retval         void
 */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->mode.last_chassis_mode == chassis_move_transit->mode.chassis_mode)
    {
        return;
    }
    // 鍒囧叆鎵嬪姩鎺у埗妯″紡
    if ((chassis_move_transit->mode.last_chassis_mode != CHASSIS_MANL_CTRL) &&
         chassis_move_transit->mode.chassis_mode      == CHASSIS_MANL_CTRL)
    {
        // 搴曠洏鎸夌収杞姩鏈€灏忕殑瑙掑害杞埌浜戝彴YAW瑙掑害涓?锛屽仛鍒板弻鏂圭殑姝ｆ柟鍚戦噸鍚?        // 璁＄畻鏈€灏忚浆鍔ㄨ搴︼細灏嗗簳鐩樺綋鍓峺aw瑙掑害璋冩暣鍒?搴︼紙浜戝彴姝ｆ柟鍚戯級
        // chassis_move_transit->chassis_yaw_set = 0.0f;
        // chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
        // chassis_move_transit->state_set.chassis_yaw_set = chassis_move_transit->state_ref.chassis_yaw_absolute ;

    }
    // 鍒囧叆ROS鎺у埗妯″紡
    else if ((chassis_move_transit->mode.last_chassis_mode != CHASSIS_ROS_CTRL) &&
              chassis_move_transit->mode.chassis_mode      == CHASSIS_ROS_CTRL)
    {
        // 浜戝彴闇€瑕佹寜鐓ц浆鍔ㄦ渶灏忕殑瑙掑害杞埌搴曠洏褰撳墠YAW瑙掑害锛屽仛鍒板弻鏂圭殑姝ｆ柟鍚戦噸鍚?        // 搴曠洏淇濇寔褰撳墠鏈濆悜锛屼簯鍙颁細璺熼殢搴曠洏
        // chassis_move_transit->state_set.chassis_yaw_set =chassis_move_transit->state_ref.chassis_yaw_absolute;
    }

    if ((chassis_move_transit->mode.last_chassis_mode != CHASSIS_HOLD_TEST) &&
         chassis_move_transit->mode.chassis_mode      == CHASSIS_HOLD_TEST)
    {
        chassis_move_transit->state_set.vx = 0.0f;
        chassis_move_transit->state_set.wz = 0.0f;
        chassis_hold_test_reset(chassis_move_transit);
    }

    // 鍒囧叆鏃犲姏妯″紡锛屾竻绌洪噷绋嬭
    if ((chassis_move_transit->mode.last_chassis_mode != CHASSIS_FORCE_RAW) &&
              chassis_move_transit->mode.chassis_mode == CHASSIS_FORCE_RAW)
    {
        chassis_move_transit->state_set.vx = 0.0f;
        chassis_move_transit->state_set.wz = 0.0f;
    }

    chassis_move_transit->mode.last_chassis_mode = chassis_move_transit->mode.chassis_mode;
}

/**
 * @brief          閬ユ帶鍣ㄧ殑鏁版嵁澶勭悊鎴愬簳鐩樼殑鍓嶈繘vx閫熷害锛寁y閫熷害
 * @author         pxx
 * @param          vx_set  x杞村墠杩涢€熷害璁剧疆锛宮/s
 * @param          w_set  z杞磋閫熷害璁剧疆锛宮/s
 * @param          chassis_move_rc_to_vector   搴曠洏缁撴瀯浣撴寚閽? * @retval         void
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *w_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || w_set == NULL)
    {
        return;
    }
    int16_t vx_channel = 0;
    int16_t w_channel = 0;
    fp32 vx_set_channel = 0.0f;
    fp32 w_set_channel = 0.0f;

    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[RC_LEFT_Y_CH], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[RC_LEFT_X_CH], w_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    w_set_channel = -w_channel * CHASSIS_ANGLE_Z_RC_SEN;

    if (vx_channel == 0)
    {
        chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_vx.input = 0.0f;
        chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_vx.out = 0.0f;
        *vx_set = 0.0f;
    }
    else
    {
        first_order_filter_cali(&chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_vx, vx_set_channel);
        *vx_set = chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_vx.out;
    }

    if (w_channel == 0)
    {
        chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_wz.input = 0.0f;
        chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_wz.out = 0.0f;
        *w_set = 0.0f;
    }
    else
    {
        first_order_filter_cali(&chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_wz, w_set_channel);
        *w_set = chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_wz.out;
    }
}
/**
  * @brief          宸€熻疆杩愬姩鍒嗚В锛堥€嗚В锛?  * @author         pxx
  * @param          vx_set  x杞村墠杩涢€熷害璁剧疆锛宮/s
  * @param          wz_set  z杞存棆杞€熷害璁剧疆锛宺ad/s
  * @param          wheel_speed 鍥涗釜杞瓙鐨勮浆閫?  * @retval         void
  */
void chassis_vector_to_wheel_speed(const fp32 vx_set,  const fp32 wz_set, fp32 wheel_speed[4])
{
    // 0/3 are left wheels, 1/2 are right wheels. Positive wz rotates around the chassis center.
    wheel_speed[0] = vx_set -  MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set +  MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set +  MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = vx_set -  MOTOR_DISTANCE_TO_CENTER * wz_set;
}

/**
 * @brief          璁剧疆閬ユ帶鍣ㄨ緭鍏ユ帶鍒堕噺
 * @author         pxx
 * @param          chassis_move_control    搴曠洏缁撴瀯浣撴寚閽? * @retval         void
 */
void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set_clean = 0.0f;
    fp32 w_set_clean = 0.0f;

    if (chassis_move_control->mode.chassis_mode == CHASSIS_FORCE_RAW)
    {
        chassis_move_control->state_set.vx = 0.0f;
        chassis_move_control->state_set.wz = 0.0f;
        return;
    }

    if (chassis_move_control->mode.chassis_mode == CHASSIS_MANL_CTRL)
    {
        chassis_rc_to_control_vector(&vx_set_clean, &w_set_clean, chassis_move_control);
    }
    else if (chassis_move_control->mode.chassis_mode == CHASSIS_ROS_CTRL)
    {
        const MiniPC_ChassisCmd_Typedef *cmd = MiniPC_GetChassisCmdPoint();
        if (cmd != NULL && MiniPC_IsChassisCmdOnline(MINIPC_CHASSIS_CMD_TIMEOUT_MS))
        {
            vx_set_clean = cmd->vx;
            w_set_clean = cmd->wz;
        }
    }

    const fp32 target_vx = fp32_constrain(vx_set_clean, -NORMAL_MAX_CHASSIS_SPEED_X, NORMAL_MAX_CHASSIS_SPEED_X);
    const fp32 target_wz = fp32_constrain(w_set_clean, -NORMAL_MAX_CHASSIS_SPEED_WZ, NORMAL_MAX_CHASSIS_SPEED_WZ);

    chassis_move_control->state_set.vx = target_vx;
    chassis_move_control->state_set.wz = target_wz;


}

/**
 * @brief          搴曠洏鎺у埗PID璁＄畻
 * @author         pxx
 * @param          chassis_move_control_loop   搴曠洏缁撴瀯浣撴寚閽? * @retval         void
 */
void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    if (chassis_move_control_loop->mode.chassis_mode == CHASSIS_FORCE_RAW )
    {
        chassis_mit_control_reset_all(chassis_move_control_loop);
        return;
    }

    if (chassis_move_control_loop->mode.chassis_mode == CHASSIS_HOLD_TEST)
    {
        chassis_hold_test_control(chassis_move_control_loop);
        return;
    }


    float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //杩愬姩鍒嗚В
    chassis_vector_to_wheel_speed(chassis_move_control_loop->state_set.vx,
                                  chassis_move_control_loop->state_set.wz,
                                  wheel_speed);
    chassis_limit_wheel_speed_keep_ratio(wheel_speed);
    // Do not mix IMU yaw-rate PID into wheel speed while tuning the motor speed loop.
    old_PID_clear(&chassis_move_control_loop->chassis_pid.chassis_yaw_gyro_pid);
    const fp32 yaw_pid_force = 0.0f;

    //璁＄畻pid
    for (i = 0; i < 4; i++)
    {
        if (i==1||i==2)
        {
            chassis_move_control_loop->chassis_motor[i].speed_set = wheel_speed[i] + yaw_pid_force;
        }
        else
        {
            chassis_move_control_loop->chassis_motor[i].speed_set = wheel_speed[i] - yaw_pid_force;
        }

    }

    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->chassis_motor[i].speed_set_rpm =
            chassis_move_control_loop->chassis_motor[i].speed_set * CHASSIS_MOTOR_VECTOR_TO_RPM_SEN;
    }
    chassis_apply_turn_min_rpm(chassis_move_control_loop);

    for (i = 0; i < 4; i++)
    {
        chassis_mit_current_control(&chassis_move_control_loop->chassis_motor[i],
                                    &chassis_move_control_loop->chassis_pid.motor_speed_pid[i]);
    }

}


const chassis_move_t *get_chassis_control_point(void)
{
    return &chassis_move;
}
const Chassis_ref_t *get_chassis_ref_point(void)
{
    return &chassis_move.state_ref;
}

void chassis_set_motor_speed_pid(fp32 kp, fp32 ki, fp32 kd)
{
    const fp32 motor_speed_pid[3] = {kp, ki, kd};

    for (uint8_t i = 0U; i < 4U; i++)
    {
        old_PID_Init(&chassis_move.chassis_pid.motor_speed_pid[i],
                     PID_POSITION,
                     motor_speed_pid,
                     M3505_MOTOR_SPEED_PID_MAX_OUT,
                     M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
}

bool is_chassis_init_done(void)
{
    return chassis_init_done;
}
