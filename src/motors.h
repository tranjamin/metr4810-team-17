#ifndef MOTORS_H
#define MOTORS_H

#include "FreeRTOSConfig.h"

#define MOTORS_TASK_NAME "Motor Control Task"
#define MOTORS_TASK_PRIORITY 3
#define MOTORS_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define MOTORS_TASK_COREMASK 0x01

void vMotorsTask();
void vMotorsInit();

#endif