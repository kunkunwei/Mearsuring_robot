//
// Created by kun on 2026/7/29.
//

#ifndef MEARSURING_ROBOT_ULTRASONIC_TASK_H
#define MEARSURING_ROBOT_ULTRASONIC_TASK_H

#include "ultrasonic_i2c.h"

void Ultrasonic_Task(void const *argument);
const UltrasonicI2C_t *Ultrasonic_Task_GetLeft(void);
const UltrasonicI2C_t *Ultrasonic_Task_GetRight(void);

#endif //MEARSURING_ROBOT_ULTRASONIC_TASK_H
