#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "queue.h"
#include "diagnostics.h"
#include <string.h>
#include <stdio.h>

#define VDELAY 10000
#define QUEUE_LENGTH 10
#define QUEUE_WAIT_TICKS 0

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

void vDiagnosticsTask() {
    for (;;) {
        DiagnosticMessage msg;
        memset(msg.message, 0, MESSAGE_MAX_SIZE);

        TaskStatus_t* tasks;
        volatile UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
        tasks = pvPortMalloc(num_tasks * sizeof(TaskStatus_t));
        num_tasks = uxTaskGetSystemState(tasks, num_tasks, NULL);
        
        int task_no = 0;
        while (task_no < num_tasks) {
            TaskStatus_t task = tasks[task_no];
            char* task_name = task.pcTaskName;
            configSTACK_DEPTH_TYPE task_overflow = task.usStackHighWaterMark;
            char task_string[50];
            snprintf(task_string, 50, "Task %d: %s Memory Remaining: %i <br/>", task_no, task_name, task_overflow);
            strncat(msg.message, task_string, MESSAGE_MAX_SIZE - 1 - strlen(msg.message));
            task_no++;
        }

        if (xDiagnosticQueue != NULL) {
            xQueueSend(xDiagnosticQueue, &msg, QUEUE_WAIT_TICKS);
        }

        // block for some time
        vTaskDelay(VDELAY);
    }
}