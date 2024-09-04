#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

#include "diagnostics.h"

#define PUSHBUTTON_PIN  17
#define SWITCH_PIN 16

void vDigitalIOInit();
void vDigitalIOTask();

void gpioCallback();

void vDigitalIOInit() {
    gpio_init(PUSHBUTTON_PIN);
    gpio_init(SWITCH_PIN);
    
    gpio_set_dir(PUSHBUTTON_PIN, GPIO_IN);
    gpio_set_dir(SWITCH_PIN, GPIO_IN);

    gpio_pull_down(PUSHBUTTON_PIN);
    gpio_pull_down(SWITCH_PIN);

    gpio_set_input_enabled(PUSHBUTTON_PIN, true);
    gpio_set_input_enabled(SWITCH_PIN, true);

    gpio_set_irq_enabled(PUSHBUTTON_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(SWITCH_PIN, GPIO_IRQ_EDGE_RISE, true);

    gpio_set_irq_callback(&gpioCallback);
}

void gpioCallback(uint gpio_number, uint32_t events) {
    switch (gpio_number) {
        case PUSHBUTTON_PIN:
            vDebugLog("Received Rising Edge on Pushbutton\n");
            break;
        case SWITCH_PIN:
            vDebugLog("Received Rising Edge on Switch\n");
    }
}

void vDigitalIOTask() {

}