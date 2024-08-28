#ifndef WIFI_H
#define WIFI_H

#include "FreeRTOSConfig.h"

#define WIFI_TASK_NAME "WiFi Task"
#define WIFI_TASK_PRIORITY 10
#define WIFI_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 100)
#define WIFI_TASK_COREMASK 0x02

void vWifiTask();
void vWifiInit();

#endif