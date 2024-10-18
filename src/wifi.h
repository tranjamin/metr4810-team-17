/**
The driver and RTOS tasks for the wifi. Sets up both TCP and UDP servers.

Functions:
    vWifiTask(): the main loop for TCP
    vWifiUDPTask(): the main loop for UDP
    vWifiInit(): initialises firmware for both servvers
    enableUDP(): enabled UDP
    disableUDP(): disables UDP
 */

#ifndef WIFI_H
#define WIFI_H

#include "FreeRTOSConfig.h"

// RTOS Task Settings (TCP)
#define WIFI_TASK_NAME "WiFi TCP Task"
#define WIFI_TASK_PRIORITY (tskIDLE_PRIORITY + 10)
#define WIFI_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 50)
#define WIFI_TASK_COREMASK 0x02

// RTOS Task Settings (UDP)
#define WIFIUDP_TASK_NAME "WiFi UDP Task"
#define WIFIUDP_TASK_PRIORITY (tskIDLE_PRIORITY + 10)
#define WIFIUDP_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 50)
#define WIFIUDP_TASK_COREMASK 0x02

// Functions
void vWifiTask();
void vWifiUDPTask();
void vWifiInit();
void enableUDP();
void disableUDP();

#endif