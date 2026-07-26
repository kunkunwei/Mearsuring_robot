#ifndef __USER_TASK__
#define __USER_TASK__

#include "ultrasonic_i2c.h"



void User_Task(void const * argument);
const UltrasonicI2C_t *User_GetUltrasonicLeft(void);
const UltrasonicI2C_t *User_GetUltrasonicRight(void);


#endif 
