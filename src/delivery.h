#ifndef DELIVERY_H
#define DELIVERY_H

#include "FreeRTOSConfig.h"

#define DELIVERY_TASK_NAME "Bean Delivery Task"
#define DELIVERY_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define DELIVERY_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define DELIVERY_TASK_COREMASK 0x01

void vDeliveryTask();
void vDeliveryInit();

void vStartDelivery();

#endif