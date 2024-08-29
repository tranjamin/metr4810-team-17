#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "queue.h"
#include "diagnostics.h"
#include <string.h>
#include <stdio.h>

#define VDELAY 1000
#define QUEUE_LENGTH 1
#define QUEUE_WAIT_TICKS 100

// function prototypes
void vDiagnosticsTask();
void vDiagnosticsInit();

QueueHandle_t xDiagnosticQueue;

BaseType_t xGetDiagnosticMessage(DiagnosticMessage* buffer) {
    if (xDiagnosticQueue == NULL) {
        return pdFALSE;
    }
    return xQueueReceiveFromISR(xDiagnosticQueue, (void*) buffer, NULL);
}

void vDiagnosticsInit() {
    xDiagnosticQueue = xQueueCreate(QUEUE_LENGTH, sizeof(DiagnosticMessage));
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

void vDiagnosticsTask() {
    for (;;) {
        vTaskDiagnostics();

        // block for some time
        vTaskDelay(VDELAY);
    }
}