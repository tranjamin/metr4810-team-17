#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "FreeRTOSConfig.h"

#define DIAGNOSTICS_TASK_NAME "Diagnostics Task"
#define DIAGNOSTICS_TASK_PRIORITY 3
#define DIAGNOSTICS_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define DIAGNOSTICS_TASK_COREMASK 0x01

void vDiagnosticsTask();
void vDiagnosticsInit();

#endif