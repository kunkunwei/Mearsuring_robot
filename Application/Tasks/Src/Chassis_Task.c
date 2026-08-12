#include "Chassis_task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "observe_task.h"
#include "minipc.h"
#include <math.h>
// ============================================================
#include "monitor.h"
#include "mymotor.h"

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

// 上电后先等待INS姿态解算收敛，单位ms
#define CHASSIS_PITCH_ZERO_SETTLE_TIME_MS 1000U
// 在平地静止状态下采集Pitch平均值的时长，单位ms
#define CHASSIS_PITCH_ZERO_SAMPLE_TIME_MS 1000U
// Pitch零点采样周期，单位ms
#define CHASSIS_PITCH_ZERO_SAMPLE_PERIOD_MS 2U

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

// 搴曠洏杩愬姩鏁版嵁
static chassis_move_t chassis_move;
static Chassis_Current_Command_t chassis_current_command;
static volatile bool chassis_init_done = false; // 鍒濆鍖栧畬鎴愭爣蹇楋紝缃綅鍚庡叾浠栦换鍔℃墠鍙畨鍏ㄨ闂?chassis 鍐呮寚閽?// 浠庝簯鍙伴€氫俊鑾峰彇鎺у埗鍛戒护

// ============================================================

static void chassis_publish_current_command(const float current_a[4], uint32_t now_tick)
{
    if (current_a == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < 4U; i++)
    {
        chassis_current_command.current_a[i] = current_a[i];
    }
    chassis_current_command.tick = now_tick;
    chassis_current_command.sequence++;
    taskEXIT_CRITICAL();
}

bool chassis_get_current_command(Chassis_Current_Command_t *command, uint32_t now_tick)
{
    if (command == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    *command = chassis_current_command;
    taskEXIT_CRITICAL();

    return command->sequence != 0U &&
           (uint32_t)(now_tick - command->tick) <= CHASSIS_CURRENT_COMMAND_TIMEOUT_MS;
}

static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_calibrate_pitch_zero(chassis_move_t *chassis_move_calibrate);
void chassis_set_mode(chassis_move_t *chassis_move_mode);
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
void chassis_set_contorl(chassis_move_t *chassis_move_control);
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
// ============================================================


void Chassis_Task(void const *argument)
{
    (void)argument;
    /* USER CODE BEGIN Chassis_Task */
    /* Infinite loop */
    // osDelay(100);
    TickType_t systick = 0;

    chassis_init(&chassis_move);

    // 在允许CAN控制任务运行前完成本次上电的平地Pitch零点采样
    chassis_calibrate_pitch_zero(&chassis_move);

    // Pitch零点采样结束后，才允许其他任务访问并控制底盘
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
 * @brief 上电后在平地静止状态下采集Pitch平均值，作为本次运行的零点
 * @param chassis_move_calibrate 底盘控制结构体指针
 */
static void chassis_calibrate_pitch_zero(chassis_move_t *chassis_move_calibrate)
{
    // 如果底盘结构体或INS角度指针无效，则保留默认的固定Pitch零偏
    if (chassis_move_calibrate == NULL || chassis_move_calibrate->chassis_INS_angle == NULL)
    {
        return;
    }

    // 校准期间持续保持控制管理器关闭，确保输出电流为零
    Chassis_ControlManager_Disable(&chassis_move_calibrate->control_manager,
                                   &chassis_move_calibrate->control_output);

    // 等待INS滤波器和姿态解算先完成初始收敛
    osDelay(CHASSIS_PITCH_ZERO_SETTLE_TIME_MS);

    float pitch_sum_rad = 0.0f;
    uint32_t valid_sample_count = 0U;
    const uint32_t sample_start_tick = HAL_GetTick();

    // 当前代码中的Roll轴对应实车Pitch，因此从Roll索引采集真实Pitch零点
    while ((uint32_t)(HAL_GetTick() - sample_start_tick) < CHASSIS_PITCH_ZERO_SAMPLE_TIME_MS)
    {
        const float pitch_sample_rad =
            *(chassis_move_calibrate->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);

        // 只累加有效浮点数，避免异常姿态数据污染平均值
        if (isfinite(pitch_sample_rad))
        {
            pitch_sum_rad += pitch_sample_rad;
            valid_sample_count++;
        }

        // 等待下一个采样周期，同时让INS任务继续更新姿态
        osDelay(CHASSIS_PITCH_ZERO_SAMPLE_PERIOD_MS);
    }

    // 如果没有取得有效样本，则保留配置中的默认固定Pitch零偏
    if (valid_sample_count == 0U)
    {
        return;
    }

    // 使用本次上电采样的Pitch平均值覆盖本次运行的零点
    chassis_move_calibrate->control_manager.config.hold.pitch_zero_offset_rad =
        pitch_sum_rad / (float)valid_sample_count;
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
    // 涓€闃朵綆閫氭护娉㈠垵濮嬪寲
    static const fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};

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
        chassis_move_init->chassis_motor[i].last_speed_rpm = 0.0f;
        chassis_move_init->chassis_motor[i].current_cmd_a = 0.0f;
        chassis_move_init->chassis_motor[i].target_current = 0;
    }
    Chassis_ControlManager_Init(&chassis_move_init->control_manager, NULL);
    Chassis_ControlManager_Disable(&chassis_move_init->control_manager,
                                   &chassis_move_init->control_output);
    chassis_move_init->last_torque_control_tick = HAL_GetTick();
    chassis_move_init->next_torque_control_tick =
        chassis_move_init->last_torque_control_tick + CHASSIS_TORQUE_CONTROL_PERIOD_MS;
    chassis_publish_current_command(chassis_move_init->control_output.current_a,
                                    chassis_move_init->last_torque_control_tick);

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
        const fp32 last_speed_rpm = chassis_move_update->chassis_motor[i].last_speed_rpm;
        chassis_move_update->chassis_motor[i].speed_rpm = chassis_move_update->chassis_motor[i].chassis_motor_measure->rpm *
                                                          motor_forward_sign[i];
        chassis_move_update->chassis_motor[i].speed = chassis_move_update->chassis_motor[i].speed_rpm *
                                                      CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
        chassis_move_update->chassis_motor[i].accel =
                                                      (chassis_move_update->chassis_motor[i].speed_rpm - last_speed_rpm) *
                                                      CHASSIS_CONTROL_FREQUENCE *
                                                      CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
        chassis_move_update->chassis_motor[i].last_speed_rpm =
            chassis_move_update->chassis_motor[i].speed_rpm;
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

    if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[FUNCTION_CHANNEL]))
    {
        chassis_move_mode->mode.chassis_mode = CHASSIS_HOLD_TEST;
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_move_mode->mode.chassis_mode = CHASSIS_FORCE_RAW;
    }
    else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_move_mode->mode.chassis_mode = CHASSIS_MANL_CTRL ;
    }
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_move_mode->mode.chassis_mode = CHASSIS_ROS_CTRL;
    }
    else
    {
        chassis_move_mode->mode.chassis_mode = CHASSIS_FORCE_RAW;
    }

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
    // chassis_move_control->state_set.wz = 0;
    chassis_move_control->state_set.wz = target_wz;
}

/**
 * @brief          搴曠洏鎺у埗PID璁＄畻
 * @author         pxx
 * @param          chassis_move_control_loop   搴曠洏缁撴瀯浣撴寚閽? * @retval         void
 */
void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    if (chassis_move_control_loop == NULL)
    {
        return;
    }

    const uint32_t now_tick = HAL_GetTick();
    if (chassis_move_control_loop->mode.chassis_mode == CHASSIS_FORCE_RAW )
    {
        for (uint8_t i = 0U; i < 4U; i++)
        {
            chassis_move_control_loop->chassis_motor[i].speed_set = 0.0f;
            chassis_move_control_loop->chassis_motor[i].speed_set_rpm = 0.0f;
            chassis_move_control_loop->chassis_motor[i].current_cmd_a = 0.0f;
            chassis_move_control_loop->chassis_motor[i].target_current = 0;
        }
        Chassis_ControlManager_Disable(&chassis_move_control_loop->control_manager,
                                       &chassis_move_control_loop->control_output);
        chassis_publish_current_command(chassis_move_control_loop->control_output.current_a,
                                        now_tick);
        chassis_move_control_loop->last_torque_control_tick = now_tick;
        chassis_move_control_loop->next_torque_control_tick =
            now_tick + CHASSIS_TORQUE_CONTROL_PERIOD_MS;
        return;
    }

    if ((int32_t)(now_tick - chassis_move_control_loop->next_torque_control_tick) < 0)
    {
        return;
    }

    const fp32 control_dt_s =
        (fp32)(uint32_t)(now_tick - chassis_move_control_loop->last_torque_control_tick) * 0.001f;
    chassis_move_control_loop->last_torque_control_tick = now_tick;
    do
    {
        chassis_move_control_loop->next_torque_control_tick += CHASSIS_TORQUE_CONTROL_PERIOD_MS;
    }
    while ((int32_t)(now_tick - chassis_move_control_loop->next_torque_control_tick) >= 0);


    float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //杩愬姩鍒嗚В
    chassis_vector_to_wheel_speed(chassis_move_control_loop->state_set.vx,
                                  chassis_move_control_loop->state_set.wz,
                                  wheel_speed);
    chassis_limit_wheel_speed_keep_ratio(wheel_speed);
    // Store the chassis-frame wheel targets used by the current controller.
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->chassis_motor[i].speed_set = wheel_speed[i];
    }

    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->chassis_motor[i].speed_set_rpm =
            chassis_move_control_loop->chassis_motor[i].speed_set * CHASSIS_MOTOR_VECTOR_TO_RPM_SEN;
    }
    Chassis_Control_Input_t control_input = {
        .enabled = 1U,
        // 当前代码中的Roll轴对应实车Pitch，因此控制补偿读取Roll索引
        .pitch_rad = (chassis_move_control_loop->chassis_INS_angle != NULL) ?
                     *(chassis_move_control_loop->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET) :
                     0.0f,
    };

    for (i = 0; i < 4; i++)
    {
        Chassis_Motor_t *motor = &chassis_move_control_loop->chassis_motor[i];
        control_input.target_rpm[i] = motor->speed_set_rpm;
        control_input.speed_rpm[i] = motor->speed_rpm;
        control_input.position_deg[i] = motor->pos_deg;
        control_input.speed_valid[i] =
            vesc_motor_status_is_online(motor->chassis_motor_measure,
                                        CHASSIS_MOTOR_STATUS1_TIMEOUT_MS) ? 1U : 0U;
        control_input.position_valid[i] =
            (motor->pos_ready != 0U &&
             vesc_motor_status4_is_online(motor->chassis_motor_measure,
                                          CHASSIS_MOTOR_STATUS4_TIMEOUT_MS)) ? 1U : 0U;
    }

    Chassis_ControlManager_Update(&chassis_move_control_loop->control_manager,
                                  &control_input,
                                  control_dt_s,
                                  &chassis_move_control_loop->control_output);

    for (i = 0; i < 4; i++)
    {
        Chassis_Motor_t *motor = &chassis_move_control_loop->chassis_motor[i];
        motor->current_cmd_a = chassis_move_control_loop->control_output.current_a[i];
        motor->target_current = (int16_t)fp32_constrain(motor->current_cmd_a * 1000.0f,
                                                        -32768.0f,
                                                        32767.0f);
    }
    chassis_publish_current_command(chassis_move_control_loop->control_output.current_a,
                                    now_tick);

}


const chassis_move_t *get_chassis_control_point(void)
{
    return &chassis_move;
}
const Chassis_ref_t *get_chassis_ref_point(void)
{
    return &chassis_move.state_ref;
}

void chassis_set_mit_gains(fp32 position_kp_ma_per_deg,
                           fp32 speed_kd_ma_per_rpm,
                           fp32 friction_ma)
{
    Chassis_ControlManager_SetDriveGains(&chassis_move.control_manager,
                                         position_kp_ma_per_deg * 0.001f,
                                         speed_kd_ma_per_rpm * 0.001f,
                                         friction_ma * 0.001f);
}

bool is_chassis_init_done(void)
{
    return chassis_init_done;
}
