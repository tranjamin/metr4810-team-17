#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

#define VDELAY 3

// function prototypes
void vControllerTask();
void vControllerInit();

void vControllerInit() {
    // setup any hardware
}

void vControllerTask() {
    for (;;) {
        // do stuff

        // block for some time
        vTaskDelay(VDELAY);
    }
}