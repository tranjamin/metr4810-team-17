// RTOS + PICO INCLUDES
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// DRIVER INCLUDES
#include "delivery.h"
#include "diagnostics.h"
#include "extraction.h"
#include "blink.h"
#include "motors.h"
#include "wifi.h"
#include "rgb.h"
#include "digitalio.h"
#include "watchdog.h"

// include this to prevent UDP from turning off
// #define UDP_ALWAYS_ON

// Define these two macros to enable UART logging
// #define DEBUG_vprintf vprintf
// #define DEBUG_printf printf

void main() {
    // set up all hardware
    vDeliveryInit();
    vDiagnosticsInit();
    vExtractionInit();
    vBlinkInit();
    vWifiInit();
    vDigitalIOInit();
    vWatchdogInit();

    // store references to each task for diagnostics
    TaskHandle_t xDeliveryHandle;
    TaskHandle_t xDiagnosticsHandle;
    TaskHandle_t xExtractionHandle;
    TaskHandle_t xBlinkHandle;
    TaskHandle_t xWifiHandle;
    TaskHandle_t xWifiUDPHandle;
    TaskHandle_t xDigitalIOHandle;
    TaskHandle_t xWatchdogHandle;

    // create all tasks
    xTaskCreate(vDeliveryTask, DELIVERY_TASK_NAME, DELIVERY_TASK_STACK_SIZE, NULL, DELIVERY_TASK_PRIORITY, &xDeliveryHandle);
    xTaskCreate(vDiagnosticsTask, DIAGNOSTICS_TASK_NAME, DIAGNOSTICS_TASK_STACK_SIZE, NULL, DIAGNOSTICS_TASK_PRIORITY, &xDiagnosticsHandle);
    xTaskCreate(vBlinkTask, BLINK_TASK_NAME, BLINK_TASK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, &xBlinkHandle);
    xTaskCreate(vWifiTask, WIFI_TASK_NAME, WIFI_TASK_STACK_SIZE, NULL, WIFI_TASK_PRIORITY, &xWifiHandle);
    xTaskCreate(vWifiUDPTask, WIFIUDP_TASK_NAME, WIFIUDP_TASK_STACK_SIZE, NULL, WIFIUDP_TASK_PRIORITY, &xWifiUDPHandle);
    xTaskCreate(vWatchdogTask, WATCHDOG_TASK_NAME, WATCHDOG_TASK_STACK_SIZE, NULL, WATCHDOG_TASK_PRIORITY, &xWatchdogHandle);
    xTaskCreate(vExtractionTask, EXTRACTION_TASK_NAME, EXTRACTION_TASK_STACK_SIZE, NULL, EXTRACTION_TASK_PRIORITY, &xExtractionHandle);

    // set core affinities
    vTaskCoreAffinitySet(xDeliveryHandle, (UBaseType_t) DELIVERY_TASK_COREMASK);
    vTaskCoreAffinitySet(xDiagnosticsHandle, (UBaseType_t) DIAGNOSTICS_TASK_COREMASK);
    vTaskCoreAffinitySet(xBlinkHandle, (UBaseType_t) BLINK_TASK_COREMASK);
    vTaskCoreAffinitySet(xWifiHandle, (UBaseType_t) WIFI_TASK_COREMASK);    
    vTaskCoreAffinitySet(xWifiUDPHandle, (UBaseType_t) WIFI_TASK_COREMASK);
    vTaskCoreAffinitySet(xWatchdogHandle, (UBaseType_t) WATCHDOG_TASK_COREMASK);
    vTaskCoreAffinitySet(xExtractionHandle, (UBaseType_t) EXTRACTION_TASK_COREMASK);

    // start the scheduling of tasks
    vTaskStartScheduler();
}
