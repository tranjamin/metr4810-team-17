#ifndef MOTORS_H
#define MOTORS_H

#include "FreeRTOSConfig.h"

#define MOTORS_TASK_NAME "Motor Control Task"
#define MOTORS_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define MOTORS_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define MOTORS_TASK_COREMASK 0x01

// set the duty cycles of the pwms (as a percent)
void setTraversalDuty_LHS(float);
void setTraversalDuty_RHS(float);

void vMotorsTask();
void vMotorsInit();

#endif