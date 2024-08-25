#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

#define VDELAY 3

// function prototypes
void vExtractionTask();
void vExtractionInit();

void vExtractionInit() {
    // setup any hardware
}

void vExtractionTask() {
    for (;;) {
        // do stuff

        // block for some time
        vTaskDelay(VDELAY);
    }
}