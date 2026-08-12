//
// Created by kun on 25-7-9.
//

#include "../Inc/vofa.h"
#include "bsp_can.h"
#include "observe_task.h"
#include "mymotor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VOFA_RAD_TO_DEG 57.29577951308232f
#define VOFA_CHASSIS_PIPELINE_CHANNELS 20U

typedef struct
{
    float data[VOFA_CHASSIS_PIPELINE_CHANNELS];
    uint8_t tail[4];
} Vofa_ChassisPipelineFrame_t;

static volatile uint8_t vofa_pid_cmd_pending = 0U;
static volatile uint16_t vofa_pid_cmd_len = 0U;
static uint8_t vofa_pid_cmd_buf[VOFA_RX_CMD_MAX_LEN];
static fp32 vofa_motor_pid_kp = 10.5f;  // MIT position gain, mA/deg
static fp32 vofa_motor_pid_ki = 10.5f;  // MIT speed gain, mA/rpm
static fp32 vofa_motor_pid_kd = 250.0f; // friction compensation, mA

static char *Vofa_SkipPidSeparators(char *ptr)
{
    while (*ptr == ' ' || *ptr == '\t' || *ptr == ',' || *ptr == ':' || *ptr == '=')
    {
        ptr++;
    }
    return ptr;
}

static bool Vofa_ParsePidItemCommand(char *cmd, fp32 *kp, fp32 *ki, fp32 *kd)
{
    char *ptr = cmd;
    char *end = NULL;
    char pid_item = 0;
    fp32 value = 0.0f;

    if (cmd == NULL || kp == NULL || ki == NULL || kd == NULL)
    {
        return false;
    }

    if (!((ptr[0] == 'K' || ptr[0] == 'k') &&
          (ptr[1] == 'P' || ptr[1] == 'p' ||
           ptr[1] == 'I' || ptr[1] == 'i' ||
           ptr[1] == 'D' || ptr[1] == 'd')))
    {
        return false;
    }

    pid_item = ptr[1];
    ptr += 2;
    ptr = Vofa_SkipPidSeparators(ptr);
    value = strtof(ptr, &end);
    if (end == ptr)
    {
        return false;
    }

    if (pid_item == 'P' || pid_item == 'p')
    {
        if (value < 0.0f || value > 50.0f)
        {
            return false;
        }
        *kp = value;
    }
    else if (pid_item == 'I' || pid_item == 'i')
    {
        if (value < 0.0f || value > 50.0f)
        {
            return false;
        }
        *ki = value;
    }
    else
    {
        if (value < 0.0f || value > 500.0f)
        {
            return false;
        }
        *kd = value;
    }

    return true;
}

bool Vofa_TryStorePidCommand(const uint8_t *data, uint16_t len)
{
    uint16_t start = 0U;
    uint16_t copy_len = 0U;

    if (data == NULL || len < 4U)
    {
        return false;
    }

    while (start + 2U < len)
    {
        if ((data[start] == 'K' || data[start] == 'k') &&
            (data[start + 1U] == 'P' || data[start + 1U] == 'p' ||
             data[start + 1U] == 'I' || data[start + 1U] == 'i' ||
             data[start + 1U] == 'D' || data[start + 1U] == 'd'))
        {
            break;
        }
        start++;
    }

    if (start + 2U >= len)
    {
        return false;
    }

    copy_len = (uint16_t)(len - start);
    if (copy_len >= VOFA_RX_CMD_MAX_LEN)
    {
        copy_len = VOFA_RX_CMD_MAX_LEN - 1U;
    }

    for (uint16_t i = 0U; i < copy_len; i++)
    {
        if (data[start + i] == '\r' || data[start + i] == '\n' || data[start + i] == '\0')
        {
            copy_len = i;
            break;
        }
    }

    if (copy_len < 4U)
    {
        return false;
    }

    __disable_irq();
    memcpy(vofa_pid_cmd_buf, &data[start], copy_len);
    vofa_pid_cmd_buf[copy_len] = '\0';
    vofa_pid_cmd_len = copy_len;
    vofa_pid_cmd_pending = 1U;
    __enable_irq();

    return true;
}

void Vofa_Process_RxCommand(void)
{
    uint8_t local_buf[VOFA_RX_CMD_MAX_LEN] = {0};
    uint16_t len = 0U;
    fp32 kp = vofa_motor_pid_kp;
    fp32 ki = vofa_motor_pid_ki;
    fp32 kd = vofa_motor_pid_kd;

    if (vofa_pid_cmd_pending == 0U)
    {
        return;
    }

    __disable_irq();
    len = vofa_pid_cmd_len;
    if (len >= VOFA_RX_CMD_MAX_LEN)
    {
        len = VOFA_RX_CMD_MAX_LEN - 1U;
    }
    memcpy(local_buf, vofa_pid_cmd_buf, len);
    local_buf[len] = '\0';
    vofa_pid_cmd_pending = 0U;
    __enable_irq();

    if (Vofa_ParsePidItemCommand((char *)local_buf, &kp, &ki, &kd))
    {
        vofa_motor_pid_kp = kp;
        vofa_motor_pid_ki = ki;
        vofa_motor_pid_kd = kd;
        chassis_set_mit_gains(kp, ki, kd);
    }
}




/* JustFloat协议发送遥控器通道值
 * @param huart: 串口句柄
 * @param rc_channels: 遥控器通道值数组（浮点数格式）
 * @param num_channels: 通道数量（建议不超过VOFA_CHANNELS）
 * @return HAL_StatusTypeDef: HAL_OK表示成功，其他表示失败
 */
/* INS_Info 已迁移为 static，通过 get_ins_info_point() 访问，此处无需 extern 声明 */

void uart_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    char buffer[128]; // 根据需要可增大
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

HAL_StatusTypeDef Vofa_Send_chassis_Info(UART_HandleTypeDef *huart, const chassis_move_t *chassis)
{


    Vofa_Frame_t frame={

        .data = {
            chassis->mode.chassis_mode,
            // chassis->chassis_motor[1].chassis_motor_measure->current,
            // chassis->chassis_motor[2].chassis_motor_measure->current,
            // chassis->chassis_motor[3].chassis_motor_measure->current,
            // chassis->chassis_motor[3].chassis_motor_measure->target_current,
            chassis->state_set.vx,
            chassis->state_set.wz,
            // chassis->chassis_RC->rc.s[0],
            // chassis->state_ref.vx,
            // chassis->state_ref.wz,
            chassis->chassis_motor[0].speed_set,
            chassis->chassis_motor[0].target_current,
            chassis->chassis_motor[0].chassis_motor_measure->current,
            chassis->control_output.speed_current_a[0],
            chassis->chassis_motor[0].speed,
            (float)chassis->control_output.state,
            (float)chassis->control_output.fault,
            chassis->control_output.raw_current_a[0],
            chassis->chassis_motor[0].current_cmd_a,
        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}

HAL_StatusTypeDef Vofa_Send_Observe_Info(UART_HandleTypeDef *huart, const chassis_move_t *chassis)
{
    if (huart == NULL || chassis == NULL || chassis->chassis_imu_accel == NULL || chassis->chassis_imu_gyro == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * VOFA+ JustFloat channels:
     * 0 acc_x_body, 1 acc_y_body, 2 acc_z_body, 3 gyro_z_body,
     * 4 wheel_vx, 5 wheel_wz, 6 kf_vx, 7 slip_wz.
     */
    Vofa_Frame_t frame = {
        .data = {
            *(chassis->chassis_imu_accel + INS_ACCEL_X_ADDRESS_OFFSET),
            *(chassis->chassis_imu_accel + INS_ACCEL_Y_ADDRESS_OFFSET),
            *(chassis->chassis_imu_accel + INS_ACCEL_Z_ADDRESS_OFFSET),
            *(chassis->chassis_imu_gyro + INS_GYRO_Z_ADDRESS_OFFSET),
            chassis->state_ref.vx,
            chassis->state_ref.wz,
            get_KF_Spd(),
            get_diff_Spd(),
        },
        .tail = VOFA_TAIL,
    };

    return HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
}

HAL_StatusTypeDef Vofa_Send_Odom_Info(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    const Chassis_Odom_t *odom = get_chassis_odom_point();
    if (odom == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * VOFA+ JustFloat channels:
     * 0 odom_x, 1 odom_y, 2 odom_yaw, 3 signed_distance,
     * 4 odom_vx, 5 odom_wz, 6 slip_wz, 7 valid.
     */
    Vofa_Frame_t frame = {
        .data = {
            odom->x,
            odom->y,
            odom->yaw,
            odom->distance,
            odom->vx,
            odom->wz,
            odom->slip,
            (float)odom->valid,
        },
        .tail = VOFA_TAIL,
    };

    return HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
}

HAL_StatusTypeDef Vofa_Send_Odom_Debug_Info(UART_HandleTypeDef *huart, const chassis_move_t *chassis)
{
    if (huart == NULL || chassis == NULL || chassis->chassis_INS_angle == NULL || chassis->chassis_imu_gyro == NULL)
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        if (chassis->chassis_motor[i].chassis_motor_measure == NULL)
        {
            return HAL_ERROR;
        }
    }

    const Chassis_Odom_t *odom = get_chassis_odom_point();
    if (odom == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * VOFA+ JustFloat channels:
     * 0 odom_distance(m), 1 odom_yaw(deg), 2 INS_yaw(deg), 3 INS_gyro_z(deg/s),
     * 4-7 wheel encoder continuous angle(deg), 8 valid, 9 motion_mode,
     * 10 left_distance(m), 11 right_distance(m).
     */
    Vofa_Frame_t frame = {
        .data = {
            odom->distance,
            odom->yaw * VOFA_RAD_TO_DEG,
            *(chassis->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) * VOFA_RAD_TO_DEG,
            *(chassis->chassis_imu_gyro + INS_GYRO_Z_ADDRESS_OFFSET) * VOFA_RAD_TO_DEG,
            chassis->chassis_motor[0].pos_deg,
            chassis->chassis_motor[1].pos_deg,
            chassis->chassis_motor[2].pos_deg,
            chassis->chassis_motor[3].pos_deg,
            (float)odom->valid,
            (float)odom->motion_mode,
            odom->left_distance,
            odom->right_distance,
        },
        .tail = VOFA_TAIL,
    };

    return HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
}

HAL_StatusTypeDef Vofa_Send_Motor_Info(UART_HandleTypeDef *huart, const chassis_move_t *chassis)
{
    if (huart == NULL || chassis == NULL)
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        if (chassis->chassis_motor[i].chassis_motor_measure == NULL)
        {
            return HAL_ERROR;
        }
    }

#if CHASSIS_ESC_PROTOCOL == CHASSIS_ESC_PROTOCOL_VESC
    const float motor_pos[4] = {
        chassis->chassis_motor[0].pos_deg,
        chassis->chassis_motor[1].pos_deg,
        chassis->chassis_motor[2].pos_deg,
        chassis->chassis_motor[3].pos_deg,
    };
#else
    const float motor_pos[4] = {
        chassis->chassis_motor[0].chassis_motor_measure->real_pos * VOFA_RAD_TO_DEG,
        chassis->chassis_motor[1].chassis_motor_measure->real_pos * VOFA_RAD_TO_DEG,
        chassis->chassis_motor[2].chassis_motor_measure->real_pos * VOFA_RAD_TO_DEG,
        chassis->chassis_motor[3].chassis_motor_measure->real_pos * VOFA_RAD_TO_DEG,
    };
#endif

    /*
     * VOFA+ JustFloat channels:
     * 0-3 motor angle in degree, 4-7 motor mechanical rpm.
     * VESC mode unwraps Status 4 PID-position Now to continuous wheel angle.
     */
    Vofa_Frame_t frame = {
        .data = {
            motor_pos[0],
            motor_pos[1],
            motor_pos[2],
            motor_pos[3],
            (float)chassis->chassis_motor[0].chassis_motor_measure->rpm,
            (float)chassis->chassis_motor[1].chassis_motor_measure->rpm,
            (float)chassis->chassis_motor[2].chassis_motor_measure->rpm,
            (float)chassis->chassis_motor[3].chassis_motor_measure->rpm,
        },
        .tail = VOFA_TAIL,
    };

    return HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
}

HAL_StatusTypeDef Vofa_Send_Speed_Control_Info(UART_HandleTypeDef *huart, const chassis_move_t *chassis)
{
    if (huart == NULL || chassis == NULL)
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        if (chassis->chassis_motor[i].chassis_motor_measure == NULL)
        {
            return HAL_ERROR;
        }
    }

    /*
     * VOFA+ JustFloat channels:
     * 0-3 wheel speed set rpm, 4-7 wheel speed feedback rpm,
     * 8-11 final wheel current command in A.
     */
    Vofa_Frame_t frame = {
        .data = {
            chassis->chassis_motor[0].speed_set_rpm,
            chassis->chassis_motor[1].speed_set_rpm,
            chassis->chassis_motor[2].speed_set_rpm,
            chassis->chassis_motor[3].speed_set_rpm,
            chassis->chassis_motor[0].speed_rpm,
            chassis->chassis_motor[1].speed_rpm,
            chassis->chassis_motor[2].speed_rpm,
            chassis->chassis_motor[3].speed_rpm,
            chassis->chassis_motor[0].current_cmd_a,
            chassis->chassis_motor[1].current_cmd_a,
            chassis->chassis_motor[2].current_cmd_a,
            chassis->chassis_motor[3].current_cmd_a,
        },
        .tail = VOFA_TAIL,
    };

    return HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
}

HAL_StatusTypeDef Vofa_Send_ChassisPipeline_Debug(UART_HandleTypeDef *huart,
                                                   const chassis_move_t *chassis)
{
    if (huart == NULL || chassis == NULL)
    {
        return HAL_ERROR;
    }
    for (uint8_t i = 0U; i < 4U; i++)
    {
        if (chassis->chassis_motor[i].chassis_motor_measure == NULL)
        {
            return HAL_ERROR;
        }
    }

    Chassis_Current_Command_t current_command = {0};
    const bool snapshot_valid = chassis_get_current_command(&current_command, HAL_GetTick());
    const float diagnostic_code = (float)((uint32_t)chassis->control_output.state * 100U +
                                          (uint32_t)chassis->control_output.fault * 10U +
                                          (snapshot_valid ? 1U : 0U));

    /* target rpm[0..3], feedback rpm[4..7], CAN snapshot current[8..11],
       ESC feedback current[12..15], state/fault/valid code[16],
       CAN TX dropped count[17], queue high watermark[18], corrected real pitch deg[19]. */
    Vofa_ChassisPipelineFrame_t frame = {
        .data = {
            chassis->chassis_motor[0].speed_set_rpm,
            chassis->chassis_motor[1].speed_set_rpm,
            chassis->chassis_motor[2].speed_set_rpm,
            chassis->chassis_motor[3].speed_set_rpm,
            chassis->chassis_motor[0].speed_rpm,
            chassis->chassis_motor[1].speed_rpm,
            chassis->chassis_motor[2].speed_rpm,
            chassis->chassis_motor[3].speed_rpm,
            snapshot_valid ? current_command.current_a[0] : 0.0f,
            snapshot_valid ? current_command.current_a[1] : 0.0f,
            snapshot_valid ? current_command.current_a[2] : 0.0f,
            snapshot_valid ? current_command.current_a[3] : 0.0f,
            (float)chassis->chassis_motor[0].chassis_motor_measure->current *
                CHASSIS_MOTOR_1_FORWARD_SIGN * 0.1f,
            (float)chassis->chassis_motor[1].chassis_motor_measure->current *
                CHASSIS_MOTOR_2_FORWARD_SIGN * 0.1f,
            (float)chassis->chassis_motor[2].chassis_motor_measure->current *
                CHASSIS_MOTOR_3_FORWARD_SIGN * 0.1f,
            (float)chassis->chassis_motor[3].chassis_motor_measure->current *
                CHASSIS_MOTOR_4_FORWARD_SIGN * 0.1f,
            diagnostic_code,
            (float)BSP_CAN_GetTxQueueDropped(),
            (float)BSP_CAN_GetTxQueueHighWatermark(),
            // 代码中的Roll对应实车Pitch，减去上电零点后发送控制器使用的修正Pitch
            (chassis->chassis_INS_angle != NULL) ?
                Chassis_Hold_CorrectPitch(
                    &chassis->control_manager.config.hold,
                    *(chassis->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET)) * VOFA_RAD_TO_DEG :
                0.0f,
        },
        .tail = VOFA_TAIL,
    };

    return HAL_UART_Transmit(huart,
                             (uint8_t *)&frame,
                             sizeof(Vofa_ChassisPipelineFrame_t),
                             100U);
}

HAL_StatusTypeDef Vofa_Send_Ultrasonic_Info(UART_HandleTypeDef *huart,
                                            const UltrasonicI2C_t *left,
                                            const UltrasonicI2C_t *right,
                                            uint8_t scan_count,
                                            const uint8_t *scan_addr)
{
    if (huart == NULL || left == NULL || right == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * VOFA+ JustFloat channels:
     * 0 left_distance_mm, 1 right_distance_mm, 2 left_online, 3 right_online,
     * 4 left_hal_status, 5 right_hal_status, 6 left_addr, 7 right_addr,
     * 8 i2c_scan_count, 9 scan_addr_0, 10 scan_addr_1, 11 scan_addr_2.
     */
    Vofa_Frame_t frame = {
        .data = {
            (float)left->distance_mm,
            (float)right->distance_mm,
            (float)left->online,
            (float)right->online,
            (float)left->last_status,
            (float)right->last_status,
            (float)left->addr_7bit,
            (float)right->addr_7bit,
            (float)scan_count,
            (scan_addr != NULL && scan_count > 0U) ? (float)scan_addr[0] : 0.0f,
            (scan_addr != NULL && scan_count > 1U) ? (float)scan_addr[1] : 0.0f,
            (scan_addr != NULL && scan_count > 2U) ? (float)scan_addr[2] : 0.0f,
        },
        .tail = VOFA_TAIL,
    };

    return HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
}

HAL_StatusTypeDef Vofa_Send_Brake_Debug_Info(UART_HandleTypeDef *huart, const chassis_move_t *chassis)
{
    if (huart == NULL || chassis == NULL)
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        if (chassis->chassis_motor[i].chassis_motor_measure == NULL)
        {
            return HAL_ERROR;
        }
    }

    /*
     * VOFA+ JustFloat channels:
     * 0-3 raw controller current in A,
     * 4-7 wheel speed_rpm motor1..4,
     * 8-11 slew-limited current in A.
     */
    Vofa_Frame_t frame = {
        .data = {
            chassis->control_output.raw_current_a[0],
            chassis->control_output.raw_current_a[1],
            chassis->control_output.raw_current_a[2],
            chassis->control_output.raw_current_a[3],
            chassis->chassis_motor[0].speed_rpm,
            chassis->chassis_motor[1].speed_rpm,
            chassis->chassis_motor[2].speed_rpm,
            chassis->chassis_motor[3].speed_rpm,
            chassis->chassis_motor[0].current_cmd_a,
            chassis->chassis_motor[1].current_cmd_a,
            chassis->chassis_motor[2].current_cmd_a,
            chassis->chassis_motor[3].current_cmd_a,
        },
        .tail = VOFA_TAIL,
    };

    return HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
}
