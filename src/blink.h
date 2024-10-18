/**
Drivers and RTOS tasks for the heartbeat (blink signal), connected to the onbaord LED

Functions:
    vBlinkInit(): initialises firmware for the onboard LED
    vBlinkTask(): runs a blinking loop to the onboard LED.

Definitions:
    BLINK_TASK_NAME: the RTOS name of this task
    BLINK_TASK_PRIORITY: the RTOS priority of this task
    BLINK_TASK_STACK_SIZE: the RTOS stack size of this task
    BLINK_TASK_COREMASK: the RTOS coremask of this task
 */

#ifndef BLINK_H
#define BLINK_H

#include "FreeRTOSConfig.h"

#define BLINK_TASK_NAME "Blink Task"
#define BLINK_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define BLINK_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define BLINK_TASK_COREMASK 0x01

void vBlinkTask();
void vBlinkInit();

#endif