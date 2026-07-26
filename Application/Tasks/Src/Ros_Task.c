#include "Ros_Task.h"
#include "Chassis_Task.h"
#include "minipc.h"
#include "observe_task.h"
#include "usart.h"

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
            MiniPC_ChassisOdom_Typedef tx_odom = {
                .x = odom->x,
                .y = odom->y,
                .yaw = odom->yaw,
                .distance = odom->distance,
                .vx = odom->vx,
                .wz = odom->wz,
                .motor_ecd = {
                    local_chassis->chassis_motor[0].chassis_motor_measure->ecd,
                    local_chassis->chassis_motor[1].chassis_motor_measure->ecd,
                    local_chassis->chassis_motor[2].chassis_motor_measure->ecd,
                    local_chassis->chassis_motor[3].chassis_motor_measure->ecd,
                },
            };

            (void)MiniPC_SendChassisOdomUSB(&tx_odom);
            // Later UART6 transport uses the same protocol:
            // (void)MiniPC_SendChassisOdomUART(&huart6, &tx_odom);
        }

        osDelayUntil(&systick, ROS_TASK_PERIOD_MS);
    }
}
