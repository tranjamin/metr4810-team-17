#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

#define VDELAY 3

// function prototypes
void vPathplanningTask();
void vPathplanningInit();

void vPathplanningInit() {
    // setup any hardware
}

void vPathplanningTask() {
    for (;;) {
        // do stuff

        // block for some time
        vTaskDelay(VDELAY);
    }
}