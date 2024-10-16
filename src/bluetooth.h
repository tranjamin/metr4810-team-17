#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "FreeRTOSConfig.h"

#define BLUETOOTH_TASK_NAME "Bluetooth Task"
#define BLUETOOTH_TASK_PRIORITY (tskIDLE_PRIORITY + 10)
#define BLUETOOTH_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 50)
#define BLUETOOTH_TASK_COREMASK 0x02

void vBluetoothTask();
// void vBluetoothInit();

#endif