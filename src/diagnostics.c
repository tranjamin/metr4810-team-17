#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "queue.h"
#include "diagnostics.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

// Define these two macros to enable UART logging
#define DEBUG_vprintf vprintf
#define DEBUG_printf printf

#define VDELAY 1000

#define DIAGNOSTICS_QUEUE_LENGTH 1 // how long the diagnostics queue
#define LOG_QUEUE_LENGTH 10 // how long is the (wifi) log queue
#define QUEUE_WAIT_TICKS 0 // how long to wait when trying to add to the log queue

// function prototypes
void vDiagnosticsTask();
void vDiagnosticsInit();

// Queues to hold data
QueueHandle_t xDiagnosticQueue;
QueueHandle_t xLogQueue;

// Get a diagnostic message off the queue
BaseType_t xGetDiagnosticMessage(DiagnosticMessage* buffer) {
    if (xDiagnosticQueue == NULL) {
        return pdFALSE;
    }
    return xQueueReceiveFromISR(xDiagnosticQueue, (void*) buffer, NULL);
}

// Get diagnostics on the current tasks and store them in the queue
void vTaskDiagnostics() {
    DiagnosticMessage msg;
    memset(msg.message, 0, DIAGNOSTICS_MAX_SIZE);

    TaskStatus_t* tasks;
    volatile UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    tasks = pvPortMalloc(num_tasks * sizeof(TaskStatus_t));
    num_tasks = uxTaskGetSystemState(tasks, num_tasks, NULL);
    
    int task_no = 0;
    
    char helper_string[30];
    snprintf(helper_string, 30, "Number of Tasks: %d <br/>", num_tasks);
    strncat(msg.message, helper_string, 30);

    HeapStats_t stats;
    vPortGetHeapStats(&stats);
    char buf[60];
    snprintf(buf, 60, "Heap Remaining: %d/%dKB (%d percent) <br/>", stats.xMinimumEverFreeBytesRemaining/1024, configTOTAL_HEAP_SIZE/1024, stats.xMinimumEverFreeBytesRemaining*100/configTOTAL_HEAP_SIZE);
    strncat(msg.message, buf, DIAGNOSTICS_MAX_SIZE - 1 - strlen(msg.message));

    while (task_no < num_tasks) {
        TaskStatus_t task = tasks[task_no];

        char task_string[100];
        snprintf(task_string, 100, "Task %d: %s | Memory Remaining: %iB | Priority: %d<br/>", task.xTaskNumber, task.pcTaskName, task.usStackHighWaterMark, task.uxCurrentPriority);
        strncat(msg.message, task_string, DIAGNOSTICS_MAX_SIZE - 1 - strlen(msg.message));
        task_no++;
    }

    if (xDiagnosticQueue != NULL) {
        xQueueOverwrite(xDiagnosticQueue, &msg);
    }

    vPortFree(tasks);
}

// Get a debug message off te queue
BaseType_t xGetDebugLog(char buffer[LOG_MAX_LENGTH]) {
    if (xLogQueue == NULL) {
        return pdFALSE;
    }
    return xQueueReceiveFromISR(xLogQueue, (void*) buffer, NULL);
}

// Send a debug message into the queue
void vDebugLog(char* format, ...) {
    va_list str_args;
    va_start(str_args, format);
    #ifdef DEBUG_vprintf
        DEBUG_printf(format, str_args);
    #endif

    char str[LOG_MAX_LENGTH];
    vsnprintf(str, LOG_MAX_LENGTH, format, str_args);

    if (str == NULL) return;
    if (xLogQueue != NULL) xQueueSendToBackFromISR(xLogQueue, str, QUEUE_WAIT_TICKS);

}

void vDiagnosticsInit() {
    xDiagnosticQueue = xQueueCreate(DIAGNOSTICS_QUEUE_LENGTH, sizeof(DiagnosticMessage));
    xLogQueue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(char) * LOG_MAX_LENGTH);
}

void vDiagnosticsTask() {
    for (;;) {
        vTaskDiagnostics();
        vTaskDelay(VDELAY);
    }
}