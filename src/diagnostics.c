// RTOS INCLUDES
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// PICO INCLUDES
#include "pico/stdlib.h"
#include "diagnostics.h"

// STD INCLUDES
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define VDELAY 1000 // RTOS blocking tasks

#define DIAGNOSTICS_QUEUE_LENGTH 1 // how long the diagnostics queue os
#define LOG_QUEUE_LENGTH 10 // how long is the (wifi) log queue
#define QUEUE_WAIT_TICKS 0 // how long to wait when trying to add to the log queue

// Queues to hold data
QueueHandle_t xDiagnosticQueue;
QueueHandle_t xLogQueue;

/**
Gets a diagnostic message from the queue. Can be called from interrupts.

Parameters:
    buffer: the buffer to store the message in

Returns:
    pdTRUE if successfully popped from the queue, pdFALSE otherwise
 */
BaseType_t xGetDiagnosticMessage(DiagnosticMessage* buffer) {
    if (xDiagnosticQueue == NULL) {
        return pdFALSE;
    }
    return xQueueReceiveFromISR(xDiagnosticQueue, (void*) buffer, NULL);
}

/**
Get diagnostics on the current tasks and store them in the queue.
 */
void vTaskDiagnostics() {
    // allocate memory for the message
    DiagnosticMessage msg;
    memset(msg.message, 0, DIAGNOSTICS_MAX_SIZE);

    // get the number of tasks and allocate a buffer for them
    TaskStatus_t* tasks;
    volatile UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    tasks = pvPortMalloc(num_tasks * sizeof(TaskStatus_t));
    num_tasks = uxTaskGetSystemState(tasks, num_tasks, NULL);
    
    // add a log about the number of tasks to the message
    char preface_string[30];
    snprintf(preface_string, 30, "Number of Tasks: %d <br/>", num_tasks);
    strncat(msg.message, preface_string, 30);

    // add a log about the heap room remaining to the message
    HeapStats_t stats;
    vPortGetHeapStats(&stats);
    char buf[60];

    snprintf(buf, 60, "Heap Remaining: %d/%dKB (%d percent) <br/>", 
        stats.xMinimumEverFreeBytesRemaining/1024, configTOTAL_HEAP_SIZE/1024, 
        stats.xMinimumEverFreeBytesRemaining*100/configTOTAL_HEAP_SIZE
    );

    strncat(msg.message, buf, DIAGNOSTICS_MAX_SIZE - 1 - strlen(msg.message));

    // iterate through tasks and log information about them to the message
    int task_no = 0;
    while (task_no < num_tasks) {
        TaskStatus_t task = tasks[task_no];
        char task_string[100];

        snprintf(task_string, 100, "Task %d: %s | Memory Remaining: %iB | Priority: %d<br/>", 
            task.xTaskNumber, 
            task.pcTaskName, 
            task.usStackHighWaterMark, 
            task.uxCurrentPriority);

        strncat(msg.message, task_string, DIAGNOSTICS_MAX_SIZE - 1 - strlen(msg.message));
        task_no++;
    }

    // add the new diagnostic data to the queue, overwriting the previous
    if (xDiagnosticQueue != NULL) {
        xQueueOverwrite(xDiagnosticQueue, &msg);
    }

    // free memory allocated
    vPortFree(tasks);
}

/**
Gets the next debug message.

Parameters:
    buffer: the buffer to store the message in
 */
BaseType_t xGetDebugLog(char buffer[LOG_MAX_LENGTH]) {
    if (xLogQueue == NULL) {
        return pdFALSE;
    }
    return xQueueReceiveFromISR(xLogQueue, (void*) buffer, NULL);
}

/**
Sends a debug message.

Parameters:
    format: the string formatter
    ... the varargs for the format
 */
void vDebugLog(char* format, ...) {
    // get the input varargs
    va_list str_args;
    va_start(str_args, format);

    // print to the UART if enabled
    #ifdef DEBUG_vprintf
        DEBUG_printf(format, str_args);
    #endif

    // format the string
    char str[LOG_MAX_LENGTH];
    vsnprintf(str, LOG_MAX_LENGTH, format, str_args);
    if (str == NULL) return;

    // add the string to the log queue
    if (xLogQueue != NULL) xQueueSendToBackFromISR(xLogQueue, str, QUEUE_WAIT_TICKS);

}

/**
Initialiser hardware drivers for the diagnostics tasks.
 */
void vDiagnosticsInit() {
    // create the queues
    xDiagnosticQueue = xQueueCreate(DIAGNOSTICS_QUEUE_LENGTH, sizeof(DiagnosticMessage));
    xLogQueue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(char) * LOG_MAX_LENGTH);
}

/**
Main loop for the diagnostics task. Calculates new diagnostics every period and saves them.
 */
void vDiagnosticsTask() {
    for (;;) {
        // update the diagnostics
        vTaskDiagnostics();

        // wait for some time
        vTaskDelay(VDELAY);
    }
}