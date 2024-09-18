#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "diagnostics.h"
#include "rgb.h"

#define VDELAY 100
#define WATCHDOG_PERIOD 1000

// function prototypes
void vWatchdogTask();
void vWatchdogInit();

void vWatchdogInit() {
    vRGBInit();

    // setup any hardware
    if (watchdog_caused_reboot()) {
        setRGB_COLOUR_YELLOW();
        // vDebugLog("Watchdog Rebooted");
    } else {
        setRGB_COLOUR_GREEN();
    }

    watchdog_enable(WATCHDOG_PERIOD, 1);
}

void vWatchdogTask() {
    for (;;) {
        // do stuff
        watchdog_update();

        // block for some time
        vTaskDelay(VDELAY);
    }
}