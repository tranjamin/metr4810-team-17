#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include "FreeRTOSConfig.h"

#define PATHPLANNING_TASK_NAME "Pathplanning Task"
#define PATHPLANNING_TASK_PRIORITY 3
#define PATHPLANNING_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define PATHPLANNING_TASK_COREMASK 0x01

void vPathplanningTask();
void vPathplanningInit();

#endif