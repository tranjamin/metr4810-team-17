/**
The driver and RTOS tasks for diagnostic messages. Displays information about the system state to the user and logs messages.

Functions:
    xGetDiagnosticMessage(): gets a diagnostic message
    xGetDebugLog(): gets a debug message
    vDebugLog(): sends a debug message
    vDiagnosticsInit(): initialises firmware for the diagnostics task
    vDiagnosticsTask(): the main loop of this task

Definitions:
    DIAGNOSTICS_TASK_NAME: the RTOS name of this task
    DIAGNOSTICS_TASK_PRIORITY: the RTOS priority of this task
    DIAGNOSTICS_TASK_STACK_SIZE: the RTOS stack size of this task
    DIAGNOSTICS_TASK_COREMASK: the RTOS coremask of this task
    LOG_MAX_LENGTH: the maximum length of the debug message
    DIAGNOSTICS_MAX_SIZE: the maximum length of the diagnostics message

Structures:
    typedef struct DiagnosticMessage: a structure to represent a diagnostic message.

 */

#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "FreeRTOSConfig.h"

// RTOS Task Settings
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

// Functions
BaseType_t xGetDiagnosticMessage(DiagnosticMessage*);
BaseType_t xGetDebugLog(char buffer[LOG_MAX_LENGTH]);
void vDebugLog(char*, ...);
void vDiagnosticsTask();
void vDiagnosticsInit();

#endif