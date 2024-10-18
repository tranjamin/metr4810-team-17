/**
Drivers and RTOS tasks for the bean delivery.

Functions:
    vDeliveryInit(): initialises firmware for the delivery task
    vDeliveryTask(): the main loop of this task
    vStartDelivery(): starts the delivery sequence.

Definitions:
    DELIVERY_TASK_NAME: the RTOS name of this task
    DELIVERY_TASK_PRIORITY: the RTOS priority of this task
    DELIVERY_TASK_STACK_SIZE: the RTOS stack size of this task
    DELIVERY_TASK_COREMASK: the RTOS coremask of this task
 */

#ifndef DELIVERY_H
#define DELIVERY_H

#include "FreeRTOSConfig.h"

// RTOS Task Settings
#define DELIVERY_TASK_NAME "Bean Delivery Task"
#define DELIVERY_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define DELIVERY_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define DELIVERY_TASK_COREMASK 0x01

// Functions
void vDeliveryTask();
void vDeliveryInit();
void vStartDelivery();

#endif