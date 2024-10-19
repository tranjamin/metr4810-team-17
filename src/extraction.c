// RTOS INCLUDES
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/pwm.h"
#include "semphr.h"

// PICO INCLUDES
#include "pico/stdlib.h"

// DRIVER INCLUDES
#include "wifi.h"
#include "extraction.h"
#include "motors.h"
#include "diagnostics.h"

// PWM DEFINITIONS
#define EXTRACTION_PWM_CHAN pwm_gpio_to_channel(EXTRACTION_PWM) // the PWM channel operating on
#define EXTRACTION_PWM_SLICE pwm_gpio_to_slice_num(EXTRACTION_PWM) // the PWM slice operating on

// RTOS BLOCKING TIMES
#define VDELAY 3
#define SEMPH_TICKS 1000
#define EXRACTION_TIMEOUT 35000 // time to wait if extraction is stuck

// PWM CONFIGURATION OPTIONS
#define CLK_DIVIDER 128
#define PWM_TOP 8192
#define EXTRACTION_PWM_SPEED 100 // PWM speed to run the extraction motor

// STATE MACHINE STATES
#define EXTRACTION_IDLE 0
#define EXTRACTION_RUNNING 1

volatile int extraction_state; // the current state of the FSM
SemaphoreHandle_t extractionSemaphoreStart; // controls when to start the extraction procedure
SemaphoreHandle_t extractionSemaphoreStop; // controls when to stop the extraction procedure

// Function Prototyeps
void extractionProcedureSignalStart();
void extractionProcedureSignalStop();
void waitExtractionStopOnSense();

void vExtractionTask();
void vExtractionInit();

/**
Stop the extraction procedure.
 */
void extractionProcedureSignalStop() {
    xSemaphoreGiveFromISR(extractionSemaphoreStop, NULL);
}

/**
Start the extraction procedure.
 */
void extractionProcedureSignalStart() {
    xSemaphoreGiveFromISR(extractionSemaphoreStart, NULL);
}

/**
Stops the extraction motor when the optical sensor detects
 */
void waitExtractionStopOnSense() {
    extraction_state = EXTRACTION_RUNNING;
}

/**
Sets the pwm of the extraction motor

Parameters:
    percent: the percent pwm, from 0 to 100.
 */
void setExtractionPWM(float percent) {
    pwm_set_chan_level(EXTRACTION_PWM_SLICE, EXTRACTION_PWM_CHAN, (uint16_t) PWM_TOP * percent / 100);
}

/**
Sets up the hardware drivers for the extraction task.
 */
void vExtractionInit() {
    // initialise all gpio pins
    gpio_init(EXTRACTION_INA);
    gpio_init(EXTRACTION_INB);
    gpio_init(EXTRACTION_PWM);

    // set gpio pins as outputs
    gpio_set_dir(EXTRACTION_INA, GPIO_OUT);
    gpio_set_dir(EXTRACTION_INB, GPIO_OUT);
    gpio_set_dir(EXTRACTION_PWM, GPIO_OUT);
    
    gpio_set_function(EXTRACTION_PWM, GPIO_FUNC_PWM); // set gpio function as pwm
    pwm_set_phase_correct(EXTRACTION_PWM_SLICE, true); // set phase correction for better efficiency
    pwm_set_wrap(EXTRACTION_PWM_SLICE, PWM_TOP); // set the TOP of the pwm timer

    pwm_set_enabled(EXTRACTION_PWM_SLICE, true); // enable pwm
    setExtractionPWM(EXTRACTION_PWM_SPEED); // set pwm for the delivery motor
    SET_EXTRACTION_STOPPED(); // intially stopped

    // set up semaphores
    extractionSemaphoreStart = xSemaphoreCreateBinary();
    extractionSemaphoreStop = xSemaphoreCreateBinary();

    extraction_state = EXTRACTION_IDLE; // set initial state to IDLE
}

/**
The main loop for this task 
 */
void vExtractionTask() {
    for (;;) {
        switch (extraction_state) {
            case EXTRACTION_IDLE:
                if (xSemaphoreTake(extractionSemaphoreStart, SEMPH_TICKS) == pdTRUE) { // if delivery has been started
                    vDebugLog("Disabling UDP and Starting Extraction");

                    // suspend other tasks
                    disableUDP();
                    SET_TRAVERSAL_LHS_STOPPED();
                    SET_TRAVERSAL_RHS_STOPPED();

                    // set the extraction as forward
                    SET_EXTRACTION_FORWARD();
                    extraction_state = EXTRACTION_RUNNING;
                }
                break;
            case EXTRACTION_RUNNING:
                if (xSemaphoreTake(extractionSemaphoreStop, EXRACTION_TIMEOUT) == pdTRUE) { // if delivery has been stopped
                    vDebugLog("Re-enabling UDP");

                    // set the extraction as stopped
                    SET_EXTRACTION_STOPPED();
                    extraction_state = EXTRACTION_IDLE; 

                    // enable other tasks
                    enableUDP();
                } else { // if a timeout has occurred
                    // move backwards to recover
                    SET_EXTRACTION_BACKWARD();

                    // re-enable UDP to prevent deadlock
                    vDebugLog("Re-enabling UDP");
                    enableUDP();
                }
                break;
        }
    }
}


