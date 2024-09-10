#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "semphr.h"

#define EXTRACTION_INA 7
#define EXTRACTION_INB 6
#define EXTRACTION_PWM 8

#define EXTRACTION_PWM_CHAN pwm_gpio_to_channel(EXTRACTION_PWM)
#define EXTRACTION_PWM_SLICE pwm_gpio_to_slice_num(EXTRACTION_PWM)

#define SET_EXTRACTION_STOPPED() (gpio_put(EXTRACTION_INA, 0), gpio_put(EXTRACTION_INB, 0))
#define SET_EXTRACTION_FORWARD() (gpio_put(EXTRACTION_INA, 1), gpio_put(EXTRACTION_INB, 0))
#define SET_EXTRACTION_BACKWARD() (gpio_put(EXTRACTION_INA, 0), gpio_put(EXTRACTION_INB, 1))
#define SET_EXTRACTION_BRAKED() (gpio_put(EXTRACTION_INA, 1), gpio_put(EXTRACTION_INB, 1))

#define VDELAY 3
#define SEMPH_TICKS 10

#define CLK_DIVIDER 128
#define PWM_TOP 8192

SemaphoreHandle_t extractionSemaphore;

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

    // enable pwm
    pwm_set_enabled(EXTRACTION_PWM_SLICE, true);

    extractionSemaphore = xSemaphoreCreateBinary();
}

void setExtractionPWM(float percent) {
    pwm_set_chan_level(EXTRACTION_PWM_SLICE, EXTRACTION_PWM_CHAN, (uint16_t) PWM_TOP * percent / 100);
}

void vEnableExtraction() {
    xSemaphoreGiveFromISR(extractionSemaphore, NULL);
}

void vDisableExtraction() {
    xSemaphoreTakeFromISR(extractionSemaphore, NULL);
}

void vExtractionTask() {
    for (;;) {
        // wait to take semaphore
        if (xSemaphoreTake(extractionSemaphore, SEMPH_TICKS) == pdTRUE) {


        }
        // block for some time
        vTaskDelay(VDELAY);
    }
}