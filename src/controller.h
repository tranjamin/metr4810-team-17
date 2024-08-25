#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "FreeRTOSConfig.h"

#define CONTROLLER_TASK_NAME "Main Control Task"
#define CONTROLLER_TASK_PRIORITY 3
#define CONTROLLER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define CONTROLLER_TASK_COREMASK 0x01

void vControllerTask();
void vControllerInit();

#endif