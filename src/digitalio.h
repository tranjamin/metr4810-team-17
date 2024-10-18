/**
Drivers for the GPIO inputs, including the optical sensor, switches and pushbuttons.

Functions:
    vDigitalIOInit(): initialise firmware for the GPIO inputs.

Definitions:
    DIGITALIO_TASK_NAME: the RTOS name of this task
    DIGITALIO_TASK_PRIORITY: the RTOS priority of this task
    DIGITALIO_TASK_STACK_SIZE: the RTOS stack size of this task
    DIGITALIO_TASK_COREMASK: the RTOS coremask of this task
 */
#ifndef DIGITALIO_H
#define DIGITALIO_H

#include "FreeRTOSConfig.h"

// RTOS Task Settings
#define DIGITALIO_TASK_NAME "Switch/Pushbutton Control Task"
#define DIGITALIO_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define DIGITALIO_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define DIGITALIO_TASK_COREMASK 0x01

// Functions
void vDigitalIOInit();

#endif