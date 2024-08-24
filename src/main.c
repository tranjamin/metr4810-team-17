#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "wifi.h"


void main() {
    vWifiTask();
    // xTaskCreate(vBlinkTask, "Blink Task", 128, NULL, 2, NULL);
    // xTaskCreate(vWifiTask, "WiFi Task", 128, NULL, 1, NULL);
    // vTaskStartScheduler();
}
