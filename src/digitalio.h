#ifndef DIGITALIO_H
#define DIGITALIO_H

#include "FreeRTOSConfig.h"

#define DIGITALIO_TASK_NAME "Switch/Pushbutton Control Task"
#define DIGITALIO_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define DIGITALIO_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define DIGITALIO_TASK_COREMASK 0x01

void vDigitalIOTask();
void vDigitalIOInit();

#endif