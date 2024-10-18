// RTOS INCLUDES
#include "FreeRTOS.h"
#include "task.h"

// PICO INCLUDES
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// DRIVER INCLUDES
#include "motors.h"

// PWM DEFINITIONS
#define TRAVERSAL_LHS_PWM_SLICE pwm_gpio_to_slice_num(TRAVERSAL_LHS_PWM)
#define TRAVERSAL_LHS_PWM_CHAN pwm_gpio_to_channel(TRAVERSAL_LHS_PWM)
#define TRAVERSAL_RHS_PWM_SLICE pwm_gpio_to_slice_num(TRAVERSAL_RHS_PWM)
#define TRAVERSAL_RHS_PWM_CHAN pwm_gpio_to_channel(TRAVERSAL_RHS_PWM)

// PWM CONFIGURATION OPTIONS
#define CLK_DIVIDER 128
#define PWM_TOP 8192

// RTOS BLOCKING TIMES
#define VDELAY 100

// Function Prototypes
void vMotorsTask();
void vMotorsInit();
void setTraversalDuty_LHS(float);
void setTraversalDuty_RHS(float);

/**
Sets the duty cycle of the LHS motor

Parameters:
    percent: the percent PWM drive the LHS motor to
 */
void setTraversalDuty_LHS(float percent) {
    pwm_set_chan_level(TRAVERSAL_LHS_PWM_SLICE, TRAVERSAL_LHS_PWM_CHAN, (uint16_t) PWM_TOP * percent / 100);
}

/**
Sets the duty cycle of the RHS motor

Parameters:
    percent: the percent PWM drive the RHS motor to
 */
void setTraversalDuty_RHS(float percent) {
    pwm_set_chan_level(TRAVERSAL_RHS_PWM_SLICE, TRAVERSAL_RHS_PWM_CHAN, (uint16_t) PWM_TOP * percent / 100);
}

/**
Sets up the hardware drivers for the motor task.
 */
void vMotorsInit() {
    // intiailise all gpio pins
    gpio_init(TRAVERSAL_LHS_INA);
    gpio_init(TRAVERSAL_LHS_INB);
    gpio_init(TRAVERSAL_LHS_PWM);
    gpio_init(TRAVERSAL_RHS_INA);
    gpio_init(TRAVERSAL_RHS_INB);
    gpio_init(TRAVERSAL_RHS_PWM);

    // set gpio pins as outputs
    gpio_set_dir(TRAVERSAL_LHS_INA, GPIO_OUT);
    gpio_set_dir(TRAVERSAL_LHS_INB, GPIO_OUT);
    gpio_set_dir(TRAVERSAL_LHS_PWM, GPIO_OUT);
    gpio_set_dir(TRAVERSAL_RHS_INA, GPIO_OUT);
    gpio_set_dir(TRAVERSAL_RHS_INB, GPIO_OUT);
    gpio_set_dir(TRAVERSAL_RHS_PWM, GPIO_OUT);

    // set gpio function as pwm
    gpio_set_function(TRAVERSAL_LHS_PWM, GPIO_FUNC_PWM);
    gpio_set_function(TRAVERSAL_RHS_PWM, GPIO_FUNC_PWM);

    // set phase correction for better efficiency
    pwm_set_phase_correct(TRAVERSAL_LHS_PWM_SLICE, true);
    pwm_set_phase_correct(TRAVERSAL_RHS_PWM_SLICE, true);

    // set the clock divider
    pwm_set_clkdiv(TRAVERSAL_LHS_PWM_SLICE, CLK_DIVIDER);
    pwm_set_clkdiv(TRAVERSAL_RHS_PWM_SLICE, CLK_DIVIDER);

    // set the TOP
    pwm_set_wrap(TRAVERSAL_RHS_PWM_SLICE, PWM_TOP);
    pwm_set_wrap(TRAVERSAL_LHS_PWM_SLICE, PWM_TOP);

    // enable pwm
    pwm_set_enabled(TRAVERSAL_LHS_PWM_SLICE, true);
    pwm_set_enabled(TRAVERSAL_RHS_PWM_SLICE, true);

    // set initial pwm to 0%
    setTraversalDuty_LHS(0);
    setTraversalDuty_RHS(0);

    // set motors to be intially off
    SET_TRAVERSAL_LHS_STOPPED();
    SET_TRAVERSAL_RHS_STOPPED();
}