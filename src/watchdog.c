// RTOS INCLUDES
#include "FreeRTOS.h"
#include "task.h"

// PICO INCLUDES
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

// DRIVER INCLUDES
#include "diagnostics.h"
#include "rgb.h"

#define VDELAY 100 // RTOS blocking time
#define WATCHDOG_PERIOD 1000 // RTOS timeout time

// Function Prototypes
void vWatchdogTask();
void vWatchdogInit();

/**
Sets up the hardware drivers for the watchdog task. Sets the RGB LED as yellow if the watchdog caused a reboot or green otherwise.
 */
void vWatchdogInit() {

    // initialise the RGB driver
    vRGBInit();

    // detect whether the system rebooted
    if (watchdog_caused_reboot()) {
        setRGB_COLOUR_YELLOW();
    } else {
        setRGB_COLOUR_GREEN();
    }

    watchdog_enable(WATCHDOG_PERIOD, 1);
}

void vWatchdogTask() {
    for (;;) {
        watchdog_update(); // reset the watchdog
        vTaskDelay(VDELAY); // block for some time
    }
}