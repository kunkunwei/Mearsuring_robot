#ifndef CHASSIS_BRAKE_H
#define CHASSIS_BRAKE_H

#include "Chassis_Task.h"

void Chassis_Brake_Reset(Chassis_Motor_t *motor);
void Chassis_Brake_Update(Chassis_Motor_t *motor, PidTypeDef *pid, fp32 ref_abs_speed_rpm);

#endif
