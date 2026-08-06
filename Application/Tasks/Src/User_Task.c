#include "main.h"
#include "user_task.h"

#include "observe_task.h"
#include "arm_math.h"
#include "bsp_tim.h"
#include "bsp_can.h"
#include "Can_Task.h"
#include "usart.h"
#include "vofa.h"
#include "i2c.h"
#include "ultrasonic_i2c.h"
#include "Ultrasonic_Task.h"
#include "Chassis_Task.h"
#include "buzzer_music.h"
#include "mymotor.h"

#define CAN_RAW_DEBUG_ENABLE 0
#define CAN_DEBUG_NO_FRAME_PRINT_MS 500U
#define ULTRASONIC_DEBUG_PRINT_INTERVAL_MS 200U
#define BUZZER_MOTOR_OFFLINE_TIMEOUT_MS 200U

static uint32_t ultrasonic_last_debug_print_tick = 0U;

static uint8_t User_GetMotorOfflineMask(void)
{
    if (!is_chassis_init_done())
    {
        return 0U;
    }

    const chassis_move_t *chassis = get_chassis_control_point();
    uint8_t offline_mask = 0U;
    for (uint8_t i = 0U; i < 4U; i++)
    {
        if (!vesc_motor_status_is_online(chassis->chassis_motor[i].chassis_motor_measure,
                                         BUZZER_MOTOR_OFFLINE_TIMEOUT_MS))
        {
            offline_mask |= (uint8_t)(1U << i);
        }
    }
    return offline_mask;
}

static void User_Ultrasonic_DebugPrint(void)
{
    uint32_t now_ms = HAL_GetTick();
    const UltrasonicI2C_t *ultrasonic_left = Ultrasonic_Task_GetLeft();
    const UltrasonicI2C_t *ultrasonic_right = Ultrasonic_Task_GetRight();

    if (ultrasonic_last_debug_print_tick != 0U &&
        (uint32_t)(now_ms - ultrasonic_last_debug_print_tick) < ULTRASONIC_DEBUG_PRINT_INTERVAL_MS)
    {
        return;
    }
    ultrasonic_last_debug_print_tick = now_ms;

    uart_printf(&huart1,
                "SR09 L_ON=%u L_MM=%u LW=%d LR=%d R_ON=%u R_MM=%u RW=%d RR=%d\r\n",
                ultrasonic_left->online,
                ultrasonic_left->distance_mm,
                (int)ultrasonic_left->write_status,
                (int)ultrasonic_left->read_status,
                ultrasonic_right->online,
                ultrasonic_right->distance_mm,
                (int)ultrasonic_right->write_status,
                (int)ultrasonic_right->read_status);
}

const UltrasonicI2C_t *User_GetUltrasonicLeft(void)
{
    return Ultrasonic_Task_GetLeft();
}

const UltrasonicI2C_t *User_GetUltrasonicRight(void)
{
    return Ultrasonic_Task_GetRight();
}

void User_Task(void const *argument)
{
    TickType_t systick = 0;
    (void)argument;
    Buzzer_AlertInit(HAL_GetTick());
#if CAN_RAW_DEBUG_ENABLE
    BSP_CAN_DebugRxFrame_t can_debug_frame;
    uint32_t last_no_frame_print = 0U;
#endif
    for (;;)
    {
        systick = xTaskGetTickCount();
        Buzzer_AlertUpdate(HAL_GetTick(), User_GetMotorOfflineMask());
        // uart_printf(&huart1, "CAN\r\n");
        // HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        // Vofa_Send_Odom_Info(&huart1);
        // Vofa_Send_chassis_Info(&huart1,local_chassis);
        // Vofa_Send_Observe_Info(&huart1, local_chassis);
        // Vofa_Send_PC_Ctrl_Info(&huart6, local_pc_ctrl_info);
        // Vofa_Send_shoot_Info(&huart6, localgimbal);
        // Vofa_Send_chassis_Info(&huart6, local_chassis);
        // Vofa_Send_Motor_Info(&huart6, local_chassis);
        // Vofa_Process_RxCommand();
        // User_Ultrasonic_DebugPrint();
        // Vofa_Send_Brake_Debug_Info(&huart6, get_chassis_control_point());
        // (void)Vofa_Send_ChassisPipeline_Debug(&huart6, get_chassis_control_point());
        // Vofa_Send_Ultrasonic_Info(&huart6,
                                  // &ultrasonic_left,
                                  // &ultrasonic_right,
                                  // ultrasonic_scan_count,
                                  // ultrasonic_scan_addr);
#ifdef USE_SBUS_PROTOCOL
        if ((get_sbus_remote_control_point()->rc.s[LEFT_1_SWITCH] == -1))
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if ((get_sbus_remote_control_point()->rc.s[LEFT_1_SWITCH] == 1) && (get_remote_control_point()->rc.s[LEFT_2_SWITCH] == -1))
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if ((get_remote_control_point()->rc.s[LEFT_1_SWITCH] == 1))
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        }
#else
        if (switch_is_down(get_remote_control_point()->rc.s[0]))
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if (switch_is_up(get_remote_control_point()->rc.s[0]))
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if (switch_is_mid(get_remote_control_point()->rc.s[0]))
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        }
#endif


        // osDelay(20);
        osDelayUntil(&systick, 30); // 30ms周期控制
    }
}
