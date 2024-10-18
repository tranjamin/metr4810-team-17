// RTOS INCLUDES
#include "FreeRTOS.h"
#include "task.h"

// Pico INCLUDES
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#define VDELAY 250 // the delay between blinks

// Function Prototypes
static void pico_set_led(bool);
void vBlinkTask();
void vBlinkInit();

/**
Sets the onboard LED as on/off

Parameters:
    led_on: true to set the LED on, false otherwise
 */
static void pico_set_led(bool led_on) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
}

/**
Main task loop for the heartbeat (blink) signal. Continuously flashes the LED.
Does not return.
 */
void vBlinkTask() {
   for (;;) {
        pico_set_led(true);
        vTaskDelay(VDELAY);
        pico_set_led(false);
        vTaskDelay(VDELAY);
    }
}

/**
Initialises the CYW43 firmware for the LED.
 */
void vBlinkInit() {
    hard_assert(cyw43_arch_init() == PICO_OK);
}