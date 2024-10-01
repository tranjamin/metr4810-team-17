#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "semphr.h"

#include "extraction.h"

#define EXTRACTION_PWM_CHAN pwm_gpio_to_channel(EXTRACTION_PWM)
#define EXTRACTION_PWM_SLICE pwm_gpio_to_slice_num(EXTRACTION_PWM)

#define VDELAY 3

#define CLK_DIVIDER 128
#define PWM_TOP 8192

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
    setExtractionPWM(50);
    SET_EXTRACTION_STOPPED();

    // enable pwm
    pwm_set_enabled(EXTRACTION_PWM_SLICE, true);
    
}

void setExtractionPWM(float percent) {
    pwm_set_chan_level(EXTRACTION_PWM_SLICE, EXTRACTION_PWM_CHAN, (uint16_t) PWM_TOP * percent / 100);
}