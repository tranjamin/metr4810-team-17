// RTOS INCLUDES
#include "FreeRTOS.h"
#include "task.h"

// PICO INCLUDES
#include "pico/stdlib.h"
#include "pico/time.h"

// DRIVER INCLUDES
#include "diagnostics.h"
#include "rgb.h"
#include "extraction.h"

// PIN CONNECTIONS
#define OPTICAL_SENSOR 15
#define PUSHBUTTON_PIN  17
#define SWITCH_PIN 16
#define CSENSE_LHS 26
#define CSENSE_RHS 27
#define CSENSE_EXTRACTION 28

// DEBOUNCE INTERVALS
#define DEBOUNCE_INTERVAL_MS 50
#define OPTICAL_DEBOUNCE_INTERVAL_MS 250

// Function Prototypes
void vDigitalIOInit();
void gpioCallback();

// Debounce timers
volatile uint32_t pushbutton_debounce_timer = 0;
volatile uint32_t switch_debounce_timer = 0;
volatile uint32_t csense_lhs_debounce_timer = 0;
volatile uint32_t csense_rhs_debounce_timer = 0;
volatile uint32_t csense_extraction_debounce_timer = 0;
volatile uint32_t optical_sensor_debounce_timer = 0;

volatile bool pushbutton_is_blue = false;

/**
Sets up the hardware drivers.
 */
void vDigitalIOInit() {
    // initialise all gpio pins
    gpio_init(PUSHBUTTON_PIN);
    gpio_init(SWITCH_PIN);
    gpio_init(CSENSE_LHS);
    gpio_init(CSENSE_RHS);
    gpio_init(CSENSE_EXTRACTION);
    gpio_init(OPTICAL_SENSOR);
    
    // set gpio pins as inputs
    gpio_set_dir(PUSHBUTTON_PIN, GPIO_IN);
    gpio_set_dir(SWITCH_PIN, GPIO_IN);
    gpio_set_dir(CSENSE_LHS, GPIO_IN);
    gpio_set_dir(CSENSE_RHS, GPIO_IN);
    gpio_set_dir(CSENSE_EXTRACTION, GPIO_IN);
    gpio_set_dir(OPTICAL_SENSOR, GPIO_IN);

    // pull down gpio pins
    gpio_pull_down(PUSHBUTTON_PIN);
    gpio_pull_down(SWITCH_PIN);
    gpio_pull_down(CSENSE_LHS);
    gpio_pull_down(CSENSE_RHS);
    gpio_pull_down(CSENSE_EXTRACTION);
    gpio_pull_down(OPTICAL_SENSOR);

    // enable gpio pin inputs
    gpio_set_input_enabled(PUSHBUTTON_PIN, true);
    gpio_set_input_enabled(SWITCH_PIN, true);
    gpio_set_input_enabled(CSENSE_LHS, true);
    gpio_set_input_enabled(CSENSE_RHS, true);
    gpio_set_input_enabled(CSENSE_EXTRACTION, true);
    gpio_set_input_enabled(OPTICAL_SENSOR, true);

    // register the IRQs for each input
    gpio_set_irq_enabled(PUSHBUTTON_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(SWITCH_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(CSENSE_LHS, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(CSENSE_RHS, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(CSENSE_EXTRACTION, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(OPTICAL_SENSOR, GPIO_IRQ_EDGE_RISE, true);

    gpio_set_irq_callback(&gpioCallback);
}

/**
The monolithic callback to call when an interrupt fires

Parameters:
    gpio_number: the gpio pin the callback is associated with
    events: the bitmask of events which have occurred
 */
void gpioCallback(uint gpio_number, uint32_t events) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    switch (gpio_number) {
        case PUSHBUTTON_PIN:
            if (current_time - pushbutton_debounce_timer > DEBOUNCE_INTERVAL_MS) {
                // received rising edge outside of the debounced interval
                pushbutton_debounce_timer = current_time;
                pushbutton_is_blue = !pushbutton_is_blue;
                setRGB_COLOUR_BLUE();
            }
            break;
        case SWITCH_PIN:
            if (current_time - switch_debounce_timer > DEBOUNCE_INTERVAL_MS) {
                if (events & GPIO_IRQ_EDGE_RISE) {
                    // received rising edge outside of the debounced interval
                    switch_debounce_timer = current_time;
                    setRGB_COLOUR_RED();
                } else if (events & GPIO_IRQ_EDGE_FALL) {
                    // received falling edge outside of the debounced interval
                    switch_debounce_timer = current_time;
                    setRGB_COLOUR_BLACK();
                }
            }
            break;
        case CSENSE_LHS:
            if (current_time - csense_lhs_debounce_timer > DEBOUNCE_INTERVAL_MS) {
                if (events & GPIO_IRQ_EDGE_RISE) {
                    // received rising edge outside of the debounced interval
                    csense_lhs_debounce_timer = current_time;
                    setRGB_COLOUR_PURPLE();
                } else if (events & GPIO_IRQ_EDGE_FALL) {
                    // received falling edge outside of the debounced interval
                    csense_lhs_debounce_timer = current_time;
                    setRGB_COLOUR_BLACK();
                }
            }
            break;
        case CSENSE_RHS:
            if (current_time - csense_rhs_debounce_timer > DEBOUNCE_INTERVAL_MS) {
                if (events & GPIO_IRQ_EDGE_RISE) {
                    // received rising edge outside of the debounced interval
                    csense_rhs_debounce_timer = current_time;
                    setRGB_COLOUR_CYAN();
                } else if (events & GPIO_IRQ_EDGE_FALL) {
                    // received falling edge outside of the debounced interval
                    csense_rhs_debounce_timer = current_time;
                    setRGB_COLOUR_BLACK();
                }
            }
            break;
        case CSENSE_EXTRACTION:
            if (current_time - csense_extraction_debounce_timer > DEBOUNCE_INTERVAL_MS) {
                if (events & GPIO_IRQ_EDGE_RISE) {
                    // received rising edge outside of the debounced interval
                    csense_extraction_debounce_timer = current_time;
                    setRGB_COLOUR_YELLOW();
                } else if (events & GPIO_IRQ_EDGE_FALL) {
                    // received falling edge outside of the debounced interval
                    csense_extraction_debounce_timer = current_time;
                    setRGB_COLOUR_BLACK();
                }
            }
            break;
        case OPTICAL_SENSOR:
            if (current_time - optical_sensor_debounce_timer > OPTICAL_DEBOUNCE_INTERVAL_MS) {
                if (events & GPIO_IRQ_EDGE_RISE) {
                    // received rising edge outside of the debounced interval
                    vDebugLog("Sensed on Optical Sensor\n");
                    optical_sensor_debounce_timer = current_time;
                    setRGB_COLOUR_DARK_BLUE();

                    // stop the extraction procedure
                    extractionProcedureSignalStop();
                }
            }
            break;
    }
}