#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "FreeRTOSConfig.h"

#define DIAGNOSTICS_TASK_NAME "Diagnostics Task"
#define DIAGNOSTICS_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define DIAGNOSTICS_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 12)
#define DIAGNOSTICS_TASK_COREMASK 0x01

#define LOG_MAX_LENGTH 80 // Maximum length of a debug line
#define DIAGNOSTICS_MAX_SIZE 1000 // Maximum length of a diagnostic message

// Structure which holds a diagnostics message
typedef struct {
    char message[DIAGNOSTICS_MAX_SIZE];
} DiagnosticMessage;

// Pop off from respective queues
BaseType_t xGetDiagnosticMessage(DiagnosticMessage*);
BaseType_t xGetDebugLog(char buffer[LOG_MAX_LENGTH]);

// Send a debug message
void vDebugLog(char*, ...);

void vDiagnosticsTask();
void vDiagnosticsInit();

#endif