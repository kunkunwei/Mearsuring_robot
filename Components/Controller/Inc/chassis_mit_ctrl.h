#ifndef CHASSIS_MIT_CTRL_H
#define CHASSIS_MIT_CTRL_H

#include "Chassis_Task.h"

void Chassis_Mit_ResetMotor(Chassis_Motor_t *motor, PidTypeDef *pid);
void Chassis_Mit_CurrentControl(Chassis_Motor_t *motor, PidTypeDef *pid, fp32 brake_ref_abs_speed_rpm);

#endif
