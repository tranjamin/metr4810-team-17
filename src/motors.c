#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

#define VDELAY 3

// function prototypes
void vMotorsTask();
void vMotorsInit();

void vMotorsInit() {
    // setup any hardware
}

void vMotorsTask() {
    for (;;) {
        // do stuff

        // block for some time
        vTaskDelay(VDELAY);
    }
}