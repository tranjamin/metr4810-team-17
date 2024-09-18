#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// individual tasks
#include "controller.h"
#include "delivery.h"
#include "detection.h"
#include "diagnostics.h"
#include "extraction.h"
#include "blink.h"
#include "localisation.h"
#include "motors.h"
#include "pathplanning.h"
#include "wifi.h"
#include "rgb.h"
#include "digitalio.h"
#include "watchdog.h"

void main() {
    // set up all hardware
    vControllerInit();
    vDeliveryInit();
    vDetectionInit();
    vDiagnosticsInit();
    vExtractionInit();
    vBlinkInit();
    vLocalisationInit();
    vMotorsInit();
    vPathplanningInit();
    vWifiInit();
    vRGBInit();
    vDigitalIOInit();
    vWatchdogInit();

    // store references to each task for diagnostics
    TaskHandle_t xControllerHandle;
    TaskHandle_t xDeliveryHandle;
    TaskHandle_t xDetectionHandle;
    TaskHandle_t xDiagnosticsHandle;
    TaskHandle_t xExtractionHandle;
    TaskHandle_t xBlinkHandle;
    TaskHandle_t xLocalisationHandle;
    TaskHandle_t xMotorsHandle;
    TaskHandle_t xPathplanningHandle;
    TaskHandle_t xWifiHandle;
    TaskHandle_t xRGBHandle;
    TaskHandle_t xDigitalIOHandle;
    TaskHandle_t xWatchdogHandle;

    // create all tasks
    // xTaskCreate(vControllerTask, CONTROLLER_TASK_NAME, CONTROLLER_TASK_STACK_SIZE, NULL, CONTROLLER_TASK_PRIORITY, &xControllerHandle);
    xTaskCreate(vDeliveryTask, DELIVERY_TASK_NAME, DELIVERY_TASK_STACK_SIZE, NULL, DELIVERY_TASK_PRIORITY, &xDeliveryHandle);
    // xTaskCreate(vDetectionTask, DETECTION_TASK_NAME, DETECTION_TASK_STACK_SIZE, NULL, DETECTION_TASK_PRIORITY, &xDetectionHandle);
    xTaskCreate(vDiagnosticsTask, DIAGNOSTICS_TASK_NAME, DIAGNOSTICS_TASK_STACK_SIZE, NULL, DIAGNOSTICS_TASK_PRIORITY, &xDiagnosticsHandle);
    xTaskCreate(vExtractionTask, EXTRACTION_TASK_NAME, EXTRACTION_TASK_STACK_SIZE, NULL, EXTRACTION_TASK_PRIORITY, &xExtractionHandle);
    xTaskCreate(vBlinkTask, BLINK_TASK_NAME, BLINK_TASK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, &xBlinkHandle);
    // xTaskCreate(vLocalisationTask, LOCALISATION_TASK_NAME, LOCALISATION_TASK_STACK_SIZE, NULL, LOCALISATION_TASK_PRIORITY, &xLocalisationHandle);
    xTaskCreate(vMotorsTask, MOTORS_TASK_NAME, MOTORS_TASK_STACK_SIZE, NULL, MOTORS_TASK_PRIORITY, &xMotorsHandle);
    // xTaskCreate(vPathplanningTask, PATHPLANNING_TASK_NAME, PATHPLANNING_TASK_STACK_SIZE, NULL, PATHPLANNING_TASK_PRIORITY, &xPathplanningHandle);
    xTaskCreate(vWifiTask, WIFI_TASK_NAME, WIFI_TASK_STACK_SIZE, NULL, WIFI_TASK_PRIORITY, &xWifiHandle);
    xTaskCreate(vRGBTask, RGB_TASK_NAME, RGB_TASK_STACK_SIZE, NULL, RGB_TASK_PRIORITY, &xRGBHandle);
    // xTaskCreate(vDigitalIOTask, DIGITALIO_TASK_NAME, DIGITALIO_TASK_STACK_SIZE, NULL, DIGITALIO_TASK_PRIORITY, &xDigitalIOHandle);
    xTaskCreate(vWatchdogTask, WATCHDOG_TASK_NAME, WATCHDOG_TASK_STACK_SIZE, NULL, WATCHDOG_TASK_PRIORITY, &xWatchdogHandle);

    // set core affinities
    vTaskCoreAffinitySet(xControllerHandle, (UBaseType_t) CONTROLLER_TASK_COREMASK);
    vTaskCoreAffinitySet(xDeliveryHandle, (UBaseType_t) DELIVERY_TASK_COREMASK);
    vTaskCoreAffinitySet(xDetectionHandle, (UBaseType_t) DETECTION_TASK_COREMASK);
    vTaskCoreAffinitySet(xDiagnosticsHandle, (UBaseType_t) DIAGNOSTICS_TASK_COREMASK);
    vTaskCoreAffinitySet(xExtractionHandle, (UBaseType_t) EXTRACTION_TASK_COREMASK);
    vTaskCoreAffinitySet(xBlinkHandle, (UBaseType_t) BLINK_TASK_COREMASK);
    vTaskCoreAffinitySet(xLocalisationHandle, (UBaseType_t) LOCALISATION_TASK_COREMASK);
    vTaskCoreAffinitySet(xMotorsHandle, (UBaseType_t) MOTORS_TASK_COREMASK);
    vTaskCoreAffinitySet(xPathplanningHandle, (UBaseType_t) PATHPLANNING_TASK_COREMASK);
    vTaskCoreAffinitySet(xWifiHandle, (UBaseType_t) WIFI_TASK_COREMASK);
    vTaskCoreAffinitySet(xRGBHandle, (UBaseType_t) RGB_TASK_COREMASK);
    // vTaskCoreAffinitySet(xDigitalIOHandle, (UBaseType_t) DIGITALIO_TASK_COREMASK);
    vTaskCoreAffinitySet(xWatchdogHandle, (UBaseType_t) WATCHDOG_TASK_COREMASK);

    // start the scheduling of tasks
    vTaskStartScheduler();
}
