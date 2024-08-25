#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

#define VDELAY 3

// function prototypes
void vDeliveryTask();
void vDeliveryInit();

void vDeliveryInit() {
    // setup any hardware
}

void vDeliveryTask() {
    for (;;) {
        // do stuff

        // block for some time
        vTaskDelay(VDELAY);
    }
}