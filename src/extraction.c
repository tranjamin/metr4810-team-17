#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "semphr.h"

#include "wifi.h"
#include "extraction.h"
#include "motors.h"
#include "diagnostics.h"

#define EXTRACTION_PWM_CHAN pwm_gpio_to_channel(EXTRACTION_PWM)
#define EXTRACTION_PWM_SLICE pwm_gpio_to_slice_num(EXTRACTION_PWM)

#define VDELAY 3
#define SEMPH_TICKS 1000
#define EXRACTION_TIMEOUT 3500

#define CLK_DIVIDER 128
#define PWM_TOP 8192

#define EXTRACTION_IDLE 0
#define EXTRACTION_RUNNING 1

volatile int extraction_state;
SemaphoreHandle_t extractionSemaphoreStart;
SemaphoreHandle_t extractionSemaphoreStop;

void extractionProcedureSignalStop() {
    xSemaphoreGiveFromISR(extractionSemaphoreStop, NULL);
}

void extractionProcedureSignalStart() {
    xSemaphoreGiveFromISR(extractionSemaphoreStart, NULL);
}

// function prototypes
void vExtractionTask();
void vExtractionInit();

void vExtractionInit() {
    gpio_init(EXTRACTION_INA);
    gpio_init(EXTRACTION_INB);
    gpio_init(EXTRACTION_PWM);

    gpio_set_dir(EXTRACTION_INA, GPIO_OUT);
    gpio_set_dir(EXTRACTION_INB, GPIO_OUT);
    gpio_set_dir(EXTRACTION_PWM, GPIO_OUT);
    
    // set gpio function
    gpio_set_function(EXTRACTION_PWM, GPIO_FUNC_PWM);

    // set phase correction for better efficiency
    pwm_set_phase_correct(EXTRACTION_PWM_SLICE, true);

    // set the TOP
    pwm_set_wrap(EXTRACTION_PWM_SLICE, PWM_TOP);

    // set PWM and DIR
    setExtractionPWM(90);
    SET_EXTRACTION_STOPPED();

    // enable pwm
    pwm_set_enabled(EXTRACTION_PWM_SLICE, true);    

    extractionSemaphoreStart = xSemaphoreCreateBinary();
    extractionSemaphoreStop = xSemaphoreCreateBinary();
    extraction_state = EXTRACTION_IDLE;
}

void vExtractionTask() {
    for (;;) {
        switch (extraction_state) {
            case EXTRACTION_IDLE:
                if (xSemaphoreTake(extractionSemaphoreStart, SEMPH_TICKS) == pdTRUE) {
                    vDebugLog("Disabling UDP and Starting Extraction");
                    disableUDP();
                    SET_EXTRACTION_FORWARD();
                    SET_TRAVERSAL_LHS_STOPPED();
                    SET_TRAVERSAL_RHS_STOPPED();
                    extraction_state = EXTRACTION_RUNNING;
                }
                break;
            case EXTRACTION_RUNNING:
                if (xSemaphoreTake(extractionSemaphoreStop, EXRACTION_TIMEOUT) == pdTRUE) {  
                    SET_EXTRACTION_STOPPED();
                    extraction_state = EXTRACTION_IDLE; 
                    vDebugLog("Re-enabling UDP");
                    enableUDP();
                } else { // we have timed out
                    SET_EXTRACTION_BACKWARD();
                    vDebugLog("Re-enabling UDP");
                    enableUDP();
                }
                break;
        }
    }
}

void setExtractionPWM(float percent) {
    pwm_set_chan_level(EXTRACTION_PWM_SLICE, EXTRACTION_PWM_CHAN, (uint16_t) PWM_TOP * percent / 100);
}

