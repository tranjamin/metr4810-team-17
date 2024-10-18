/**
Drivers and RTOS tasks for the watchdog.

Functions:
    vWatchdogInit(): initialises firmware for the watchdog
    vWatchdogTask(): the main loop of this task

Definitions:
    WATCHDOG_TASK_NAME: the RTOS name of this task
    WATCHDOG_TASK_PRIORITY: the RTOS priority of this task
    WATCHDOG_TASK_STACK_SIZE: the RTOS stack size of this task
    WATCHDOG_TASK_COREMASK: the RTOS coremask of this task
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

// RTOS Task Settings
#define WATCHDOG_TASK_NAME "Watchdog Task"
#define WATCHDOG_TASK_PRIORITY (tskIDLE_PRIORITY + 15)
#define WATCHDOG_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define WATCHDOG_TASK_COREMASK 0x01

// Functions
void vWatchdogTask();
void vWatchdogInit();

#endif