#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "wifi.h"


void main() {
    vBlinkInit();

    // xTaskCreate(vBlinkTask, "Blink Task", 128, NULL, 2, NULL);
    xTaskCreate(vWifiTask, WIFI_TASK_NAME, WIFI_TASK_STACK_SIZE, NULL, WIFI_TASK_PRIORITY, NULL);
    vTaskStartScheduler();
}
