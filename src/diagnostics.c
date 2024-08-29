#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "queue.h"
#include "diagnostics.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define DEBUG_vprintf vprintf
#define DEBUG_printf printf

#define VDELAY 1000
#define QUEUE_LENGTH 1
#define QUEUE_WAIT_TICKS 0

#define LOG_QUEUE_LENGTH 10

// function prototypes
void vDiagnosticsTask();
void vDiagnosticsInit();

QueueHandle_t xDiagnosticQueue;
QueueHandle_t xLogQueue;

BaseType_t xGetDiagnosticMessage(DiagnosticMessage* buffer) {
    if (xDiagnosticQueue == NULL) {
        return pdFALSE;
    }
    return xQueueReceiveFromISR(xDiagnosticQueue, (void*) buffer, NULL);
}

void vDiagnosticsInit() {
    xDiagnosticQueue = xQueueCreate(QUEUE_LENGTH, sizeof(DiagnosticMessage));
    xLogQueue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(char) * LOG_MAX_LENGTH);
}

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

BaseType_t xGetDebugLog(char buffer[LOG_MAX_LENGTH]) {
    if (xLogQueue == NULL) {
        return pdFALSE;
    }
    return xQueueReceiveFromISR(xLogQueue, (void*) buffer, NULL);
}

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

void vDiagnosticsTask() {
    for (;;) {
        vTaskDiagnostics();

        // block for some time
        vTaskDelay(VDELAY);
    }
}