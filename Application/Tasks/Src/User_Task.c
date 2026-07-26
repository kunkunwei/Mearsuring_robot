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

#define CAN_RAW_DEBUG_ENABLE 0
#define CAN_DEBUG_NO_FRAME_PRINT_MS 500U
#define ULTRASONIC_SCAN_INTERVAL_MS 1000U
#define ULTRASONIC_SCAN_ADDR_MAX 8U

static UltrasonicI2C_t ultrasonic_left;
static UltrasonicI2C_t ultrasonic_right;
static uint8_t ultrasonic_init_done = 0U;
static uint8_t ultrasonic_scan_addr[ULTRASONIC_SCAN_ADDR_MAX];
static uint8_t ultrasonic_scan_count = 0U;
static uint32_t ultrasonic_last_scan_tick = 0U;

static void User_Ultrasonic_Init(void)
{
    if (ultrasonic_init_done != 0U)
    {
        return;
    }

    UltrasonicI2C_Init(&ultrasonic_left, &hi2c2, ULTRASONIC_LEFT_ADDR);
    UltrasonicI2C_Init(&ultrasonic_right, &hi2c2, ULTRASONIC_RIGHT_ADDR);
    ultrasonic_init_done = 1U;
}

static void User_Ultrasonic_Update(void)
{
    uint32_t now_ms = HAL_GetTick();

    UltrasonicI2C_Update(&ultrasonic_left, now_ms);
    UltrasonicI2C_Update(&ultrasonic_right, now_ms);

    if (ultrasonic_last_scan_tick == 0U ||
        (uint32_t)(now_ms - ultrasonic_last_scan_tick) >= ULTRASONIC_SCAN_INTERVAL_MS)
    {
        ultrasonic_scan_count = UltrasonicI2C_Scan(&hi2c2, ultrasonic_scan_addr, ULTRASONIC_SCAN_ADDR_MAX);
        ultrasonic_last_scan_tick = now_ms;
    }
}

const UltrasonicI2C_t *User_GetUltrasonicLeft(void)
{
    return &ultrasonic_left;
}

const UltrasonicI2C_t *User_GetUltrasonicRight(void)
{
    return &ultrasonic_right;
}

void User_Task(void const *argument)
{
    TickType_t systick = 0;
    (void)argument;
    // User_Ultrasonic_Init();

#if CAN_RAW_DEBUG_ENABLE
    BSP_CAN_DebugRxFrame_t can_debug_frame;
    uint32_t last_no_frame_print = 0U;
#endif
    for (;;)
    {
        systick = xTaskGetTickCount();
        // uart_printf(&huart1, "CAN\r\n");
        // HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        // Vofa_Send_Odom_Info(&huart1);
        // Vofa_Send_chassis_Info(&huart1,local_chassis);
        // Vofa_Send_Observe_Info(&huart1, local_chassis);
        // Vofa_Send_PC_Ctrl_Info(&huart6, local_pc_ctrl_info);
        // Vofa_Send_shoot_Info(&huart6, localgimbal);
        // Vofa_Send_chassis_Info(&huart6, local_chassis);
        // Vofa_Send_Motor_Info(&huart6, local_chassis);
        Vofa_Process_RxCommand();
        // User_Ultrasonic_Update();
        Vofa_Send_Brake_Debug_Info(&huart6, get_chassis_control_point());
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
            buzzer_off();
            // buzzer = 1;
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
