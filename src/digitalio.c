#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "diagnostics.h"
#include "rgb.h"
#include "extraction.h"

#define PUSHBUTTON_PIN  17
#define SWITCH_PIN 16
#define DEBOUNCE_INTERVAL_MS 50

#define CSENSE_LHS 26
#define CSENSE_RHS 27
#define CSENSE_EXTRACTION 28
#define MANUAL_EXTRACTION_SPEED 50

#define OPTICAL_SENSOR 18

void vDigitalIOInit();

void gpioCallback();

volatile uint32_t pushbutton_debounce_timer = 0;
volatile uint32_t switch_debounce_timer = 0;
volatile uint32_t csense_lhs_debounce_timer = 0;
volatile uint32_t csense_rhs_debounce_timer = 0;
volatile uint32_t csense_extraction_debounce_timer = 0;

volatile bool pushbutton_is_blue = false;

void vDigitalIOInit() {
    gpio_init(PUSHBUTTON_PIN);
    gpio_init(SWITCH_PIN);
    gpio_init(CSENSE_LHS);
    gpio_init(CSENSE_RHS);
    gpio_init(CSENSE_EXTRACTION);
    gpio_init(OPTICAL_SENSOR);
    
    gpio_set_dir(PUSHBUTTON_PIN, GPIO_IN);
    gpio_set_dir(SWITCH_PIN, GPIO_IN);
    gpio_set_dir(CSENSE_LHS, GPIO_IN);
    gpio_set_dir(CSENSE_RHS, GPIO_IN);
    gpio_set_dir(CSENSE_EXTRACTION, GPIO_IN);
    gpio_set_dir(OPTICAL_SENSOR, GPIO_IN);

    gpio_pull_down(PUSHBUTTON_PIN);
    gpio_pull_down(SWITCH_PIN);
    gpio_pull_down(CSENSE_LHS);
    gpio_pull_down(CSENSE_RHS);
    gpio_pull_down(CSENSE_EXTRACTION);
    gpio_pull_down(OPTICAL_SENSOR);

    gpio_set_input_enabled(PUSHBUTTON_PIN, true);
    gpio_set_input_enabled(SWITCH_PIN, true);
    gpio_set_input_enabled(CSENSE_LHS, true);
    gpio_set_input_enabled(CSENSE_RHS, true);
    gpio_set_input_enabled(CSENSE_EXTRACTION, true);
    gpio_set_input_enabled(OPTICAL_SENSOR, true);

    gpio_set_irq_enabled(PUSHBUTTON_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(SWITCH_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(CSENSE_LHS, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(CSENSE_RHS, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(CSENSE_EXTRACTION, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(OPTICAL_SENSOR, GPIO_IRQ_EDGE_RISE, true);

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
                setRGB_COLOUR_BLUE();
            }
            break;
        case SWITCH_PIN:
            if (current_time - switch_debounce_timer > DEBOUNCE_INTERVAL_MS) {
                if (events & GPIO_IRQ_EDGE_RISE) {
                    vDebugLog("Received Rising Edge on Switch\n");
                    switch_debounce_timer = current_time;
                    setRGB_COLOUR_RED();
                } else if (events & GPIO_IRQ_EDGE_FALL) {
                    vDebugLog("Received Falling Edge on Switch\n");
                    switch_debounce_timer = current_time;
                    setRGB_COLOUR_BLACK();
                }
            }
            break;
        case CSENSE_LHS:
            if (current_time - csense_lhs_debounce_timer > DEBOUNCE_INTERVAL_MS) {
                if (events & GPIO_IRQ_EDGE_RISE) {
                    vDebugLog("Received Rising Edge on Current Sense 1\n");
                    csense_lhs_debounce_timer = current_time;
                    SET_EXTRACTION_FORWARD();
                    setRGB_COLOUR_PURPLE();
                    setExtractionPWM(MANUAL_EXTRACTION_SPEED);
                } else if (events & GPIO_IRQ_EDGE_FALL) {
                    vDebugLog("Received Falling Edge on Current Sense 1\n");
                    csense_lhs_debounce_timer = current_time;
                    SET_EXTRACTION_STOPPED();
                    setRGB_COLOUR_BLACK();
                }
            }
            break;
        case CSENSE_RHS:
            if (current_time - csense_rhs_debounce_timer > DEBOUNCE_INTERVAL_MS) {
                if (events & GPIO_IRQ_EDGE_RISE) {
                    vDebugLog("Received Rising Edge on Current Sense 2\n");
                    csense_rhs_debounce_timer = current_time;
                    SET_EXTRACTION_BACKWARD();
                    setExtractionPWM(MANUAL_EXTRACTION_SPEED);
                    setRGB_COLOUR_CYAN();
                } else if (events & GPIO_IRQ_EDGE_FALL) {
                    vDebugLog("Received Falling Edge on Current Sense 2\n");
                    csense_rhs_debounce_timer = current_time;
                    SET_EXTRACTION_STOPPED();
                    setRGB_COLOUR_BLACK();
                }
            }
            break;
        case CSENSE_EXTRACTION:
            if (current_time - csense_extraction_debounce_timer > DEBOUNCE_INTERVAL_MS) {
                if (events & GPIO_IRQ_EDGE_RISE) {
                    vDebugLog("Received Rising Edge on Current Sense 3\n");
                    csense_extraction_debounce_timer = current_time;
                    setRGB_COLOUR_YELLOW();
                } else if (events & GPIO_IRQ_EDGE_FALL) {
                    vDebugLog("Received Falling Edge on Current Sense 3\n");
                    csense_extraction_debounce_timer = current_time;
                    setRGB_COLOUR_BLACK();
                }
            }
            break;
        case OPTICAL_SENSOR:
            if (events & GPIO_IRQ_EDGE_RISE) {
                vDebugLog("Sensed on Optical Sensor\n");
                setRGB_COLOUR_DARK_BLUE();
                extractionProcedureSignalStop();
            }
            break;
    }
}