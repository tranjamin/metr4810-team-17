#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

#define VDELAY 3

// function prototypes
void vDiagnosticsTask();
void vDiagnosticsInit();

void vDiagnosticsInit() {
    // setup any hardware
}

void vDiagnosticsTask() {
    for (;;) {
        // do stuff

        // block for some time
        vTaskDelay(VDELAY);
    }
}