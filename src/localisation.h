#ifndef LOCALISATION_H
#define LOCALISATION_H

#include "FreeRTOSConfig.h"

#define LOCALISATION_TASK_NAME "Localisation Task"
#define LOCALISATION_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define LOCALISATION_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define LOCALISATION_TASK_COREMASK 0x01

void vLocalisationTask();
void vLocalisationInit();

#endif