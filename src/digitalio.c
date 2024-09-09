#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "diagnostics.h"
#include "rgb.h"

#define PUSHBUTTON_PIN  17
#define SWITCH_PIN 16
#define DEBOUNCE_INTERVAL_MS 50

void vDigitalIOInit();
void vDigitalIOTask();

void gpioCallback();

volatile uint32_t pushbutton_debounce_timer = 0;
volatile uint32_t switch_debounce_timer = 0;

volatile bool pushbutton_is_blue = false;

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
    gpio_set_irq_enabled(SWITCH_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_set_irq_callback(&gpioCallback);
}


void gpioCallback(uint gpio_number, uint32_t events) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    switch (gpio_number) {
        case PUSHBUTTON_PIN:
            if (current_time - pushbutton_debounce_timer > DEBOUNCE_INTERVAL_MS) {
                vDebugLog("Received Rising Edge on Pushbutton\n");
                pushbutton_debounce_timer = current_time;
                pushbutton_is_blue = !pushbutton_is_blue;
                setRGB_BLUE(pushbutton_is_blue ? 0 : 100);
            }
            break;
        case SWITCH_PIN:
            if (current_time - switch_debounce_timer > DEBOUNCE_INTERVAL_MS) {
                if (events & GPIO_IRQ_EDGE_RISE) {
                    vDebugLog("Received Rising Edge on Switch\n");
                    switch_debounce_timer = current_time;
                    setRGB_RED(100);
                } else if (events & GPIO_IRQ_EDGE_FALL) {
                    vDebugLog("Received Falling Edge on Switch\n");
                    switch_debounce_timer = current_time;
                    setRGB_RED(0);
                }
            }
            break;
    }
}

void vDigitalIOTask() {

}