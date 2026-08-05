#include "main.h"
#include "can_task.h"

#include "usb.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "can.h"
#include "Chassis_Task.h"
#include "User_Task.h"
#include "mymotor.h" /* 显式引入，不再依�?bsp_can.h 间接引入 */
#include "vofa.h"
// #define DEBUG

/* CAN Manager Instance */

static void Dji_Motor_chassis_Can_Send(int16_t current_1, int16_t current_2, int16_t current_3,int16_t current_4);
static void VESC_Motor_chassis_Can_Send(float current_1_a, float current_2_a,
                                        float current_3_a, float current_4_a);

#define VESC_CONTROL_SEND_PERIOD_MS 5U



void Can_Task(void const *argument)
{
    /* USER CODE BEGIN Can_Task */
    /* Infinite loop */
    TickType_t systick = 0;

    /* 等待底盘初始化完成后再注册回调，避免访问空指�?*/
    while (!is_chassis_init_done()) osDelay(1);


    /* 将电�?CAN 接收回调注册�?BSP �?*/
    mymotor_register_can_callbacks();

    // const gimbal_t *local_gimbal = get_gimbal_point(); /* 地址固定，循环外初始化一次即�?*/
    const chassis_move_t *local_chassis = get_chassis_control_point();

    // save_pos_zero(CAN1_YAW_MOTOR_ID);
    for (;;)
    {
        systick = xTaskGetTickCount();

            // CAN1

            // Dji_Motor_chassis_Can_Send(0,
                                       // 0,
                                       // 0,
                                       // 0);
#if CHASSIS_ESC_PROTOCOL == CHASSIS_ESC_PROTOCOL_VESC
            Chassis_Current_Command_t current_command = {0};
            if (chassis_get_current_command(&current_command, HAL_GetTick()))
            {
                VESC_Motor_chassis_Can_Send(-current_command.current_a[0],
                                             current_command.current_a[1],
                                             current_command.current_a[2],
                                            -current_command.current_a[3]);
            }
            else
            {
                VESC_Motor_chassis_Can_Send(0.0f, 0.0f, 0.0f, 0.0f);
            }
#else
            Dji_Motor_chassis_Can_Send(-local_chassis->chassis_motor[0].target_current,
                                       local_chassis->chassis_motor[1].target_current,
                                       local_chassis->chassis_motor[2].target_current,
                                       -local_chassis->chassis_motor[3].target_current);
#endif

#if CHASSIS_ESC_PROTOCOL == CHASSIS_ESC_PROTOCOL_VESC
        osDelayUntil(&systick, VESC_CONTROL_SEND_PERIOD_MS);
#else
        osDelayUntil(&systick, 2);
#endif
    }
    /* USER CODE END Can_Task */
}


static void Dji_Motor_chassis_Can_Send(int16_t current_1, int16_t current_2, int16_t current_3,int16_t current_4)
{
    ChassisTxFrame.Data[0] = current_1 >> 8;
    ChassisTxFrame.Data[1] = current_1;
    ChassisTxFrame.Data[2] = current_2 >> 8;
    ChassisTxFrame.Data[3] = current_2;
    ChassisTxFrame.Data[4] = current_3 >> 8;
    ChassisTxFrame.Data[5] = current_3;
    ChassisTxFrame.Data[6] = current_4>>8;
    ChassisTxFrame.Data[7] = current_4;
    USER_CAN_TxMessage(&ChassisTxFrame);
}

static void VESC_Motor_chassis_Can_Send(float current_1_a, float current_2_a,
                                        float current_3_a, float current_4_a)
{
    VESC_Chassis_SetCurrent(VESC_MOTOR_1_ID, current_1_a);
    VESC_Chassis_SetCurrent(VESC_MOTOR_2_ID, current_2_a);
    VESC_Chassis_SetCurrent(VESC_MOTOR_3_ID, current_3_a);
    VESC_Chassis_SetCurrent(VESC_MOTOR_4_ID, current_4_a);
}
