#include "main.h"
#include "user_task.h"

#include "observe_task.h"
#include "arm_math.h"
#include "bsp_tim.h"
#include "bsp_can.h"
#include "Can_Task.h"
#include "usart.h"
#include "vofa.h"

#define CAN_RAW_DEBUG_ENABLE 0
#define CAN_DEBUG_NO_FRAME_PRINT_MS 500U

void User_Task(void const *argument)
{
    TickType_t systick = 0;
    (void)argument;
    const chassis_move_t *local_chassis = get_chassis_control_point();

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
        
        Vofa_Send_Odom_Debug_Info(&huart6, local_chassis);
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
