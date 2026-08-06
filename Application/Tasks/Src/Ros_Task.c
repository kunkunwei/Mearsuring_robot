#include "Ros_Task.h"
#include "Chassis_Task.h"
#include "minipc.h"
#include "observe_task.h"
#include "usart.h"
#include "Ultrasonic_Task.h"
#include "chassis_hold_ctrl.h"

#define ROS_TASK_PERIOD_MS 10

void Ros_Task(void const *argument)
{
    (void)argument;

    while (!is_chassis_init_done())
    {
        osDelay(1);
    }

    TickType_t systick = 0;
    const chassis_move_t *local_chassis = get_chassis_control_point();

    for (;;)
    {
        systick = osKernelSysTick();

        const Chassis_Odom_t *odom = get_chassis_odom_point();
        if (odom != NULL && local_chassis != NULL)
        {
            const UltrasonicI2C_t *left_ultrasonic = Ultrasonic_Task_GetLeft();
            const UltrasonicI2C_t *right_ultrasonic = Ultrasonic_Task_GetRight();

            MiniPC_ChassisOdom_Typedef tx_odom = {
                .x = odom->x,
                .y = odom->y,
                .yaw = odom->yaw,
                .distance = odom->distance,
                .vx = odom->vx,
                .wz = odom->wz,
                .pitch_rad = (local_chassis->chassis_INS_angle != NULL) ?
                             Chassis_Hold_CorrectPitch(
                                 &local_chassis->control_manager.config.hold,
                                 *(local_chassis->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET)) :
                             0.0f,
                .roll_rad = (local_chassis->chassis_INS_angle != NULL) ?
                            *(local_chassis->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET) :
                            0.0f,
                .motor_pos_deg = {
                    local_chassis->chassis_motor[0].pos_deg,
                    local_chassis->chassis_motor[1].pos_deg,
                    local_chassis->chassis_motor[2].pos_deg,
                    local_chassis->chassis_motor[3].pos_deg,
                },
                .left_mm = (left_ultrasonic != NULL) ? left_ultrasonic->distance_mm : 0U,
                .right_mm = (right_ultrasonic != NULL) ? right_ultrasonic->distance_mm : 0U,
                .left_online = (left_ultrasonic != NULL) ? left_ultrasonic->online : 0U,
                .right_online = (right_ultrasonic != NULL) ? right_ultrasonic->online : 0U,
                .segment_id = odom->segment_id,
            };

            (void)MiniPC_SendChassisOdomUART(&huart6, &tx_odom);
        }

        osDelayUntil(&systick, ROS_TASK_PERIOD_MS);
    }
}
