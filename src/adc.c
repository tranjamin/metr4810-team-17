#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

#include "hardware/adc.h"

#include "diagnostics.h"

#define CSENSE_LHS 26
#define CSENSE_RHS 27
#define CSENSE_EXTRACTION 28

#define VDELAY 1000

// function prototypes
void vADCTask();
void vADCInit();

void vADCInit() {
    // setup any hardware
    adc_init();
    adc_gpio_init(CSENSE_LHS);
    adc_gpio_init(CSENSE_RHS);
    adc_gpio_init(CSENSE_EXTRACTION);
    
    // adc_fifo_setup(
    //     true,    // Write each completed conversion to the sample FIFO
    //     true,    // Enable DMA data request (DREQ)
    //     1,       // DREQ (and IRQ) asserted when at least 1 sample present
    //     false,   // We won't see the ERR bit because of 8 bit reads; disable.
    //     true     // Shift each sample to 8 bits when pushing to FIFO
    // );

    adc_set_clkdiv(0);
    adc_set_round_robin(0b00111);
}

void vADCTask() {
    for (;;) {
        // do stuff

        uint16_t analog_value = adc_read();

        switch(adc_get_selected_input()) {
            case CSENSE_LHS - 26:
                vDebugLog("Current Sense on LHS %d", analog_value);
            case CSENSE_RHS - 26:
                vDebugLog("Current Sense on RHS %d", analog_value);
                break;
            case CSENSE_EXTRACTION - 26:
                vDebugLog("Current Sense on EX %d", analog_value);
                break;
        }

        // block for some time
        vTaskDelay(VDELAY);
    }
}