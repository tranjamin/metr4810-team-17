#ifndef DETECTION_H
#define DETECTION_H

#include "FreeRTOSConfig.h"

#define DETECTION_TASK_NAME "Bean Detection Task"
#define DETECTION_TASK_PRIORITY 3
#define DETECTION_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define DETECTION_TASK_COREMASK 0x01

void vDetectionTask();
void vDetectionInit();

#endif