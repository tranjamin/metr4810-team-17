#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "FreeRTOSConfig.h"

#define DIAGNOSTICS_TASK_NAME "Diagnostics Task"
#define DIAGNOSTICS_TASK_PRIORITY 2
#define DIAGNOSTICS_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 10)
#define DIAGNOSTICS_TASK_COREMASK 0x01

#define DIAGNOSTICS_MAX_SIZE 800
typedef struct {
    char message[DIAGNOSTICS_MAX_SIZE];
} DiagnosticMessage;

BaseType_t xGetDiagnosticMessage(DiagnosticMessage*);
void vDiagnosticsTask();
void vDiagnosticsInit();

#endif