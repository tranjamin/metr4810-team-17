#ifndef WIFI_H
#define WIFI_H

#include "FreeRTOSConfig.h"

#define WIFI_TASK_NAME "WiFi Task"
<<<<<<< Updated upstream
#define WIFI_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define WIFI_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 12)
=======
#define WIFI_TASK_PRIORITY (tskIDLE_PRIORITY + 10)
#define WIFI_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 20)
>>>>>>> Stashed changes
#define WIFI_TASK_COREMASK 0x02

void vWifiTask();
void vWifiInit();

#endif