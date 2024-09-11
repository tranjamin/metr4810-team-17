#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

#define VDELAY 3

// function prototypes
void vLocalisationTask();
void vLocalisationInit();

void vLocalisationInit() {
    // setup any hardware
}

void vLocalisationTask() {
    for (;;) {
        // do stuff

        // block for some time
        vTaskDelay(VDELAY);
    }
}