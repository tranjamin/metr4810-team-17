#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

#define VDELAY 3

// function prototypes
void vDetectionTask();
void vDetectionInit();

void vDetectionInit() {
    // setup any hardware
}

void vDetectionTask() {
    for (;;) {
        // do stuff

        // block for some time
        vTaskDelay(VDELAY);
    }
}