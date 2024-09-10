#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define RGB_RED 13
#define RGB_GREEN 14
#define RGB_BLUE 15

#define RGB_RED_SLICE pwm_gpio_to_slice_num(RGB_RED)
#define RGB_RED_CHAN pwm_gpio_to_channel(RGB_RED)
#define RGB_GREEN_SLICE pwm_gpio_to_slice_num(RGB_GREEN)
#define RGB_GREEN_CHAN pwm_gpio_to_channel(RGB_GREEN)
#define RGB_BLUE_SLICE pwm_gpio_to_slice_num(RGB_BLUE)
#define RGB_BLUE_CHAN pwm_gpio_to_channel(RGB_BLUE)

#define CLK_DIVIDER 8
#define PWM_TOP 8192

#define VDELAY 100

// function prototypes
void vRGBTask();
void vRGBInit();
void setRGB_RED(float);
void setRGB_GREEN(float);
void setRGB_BLUE(float);

void setRGB_RED(float percent) {
    pwm_set_chan_level(RGB_RED_SLICE, RGB_RED_CHAN, (uint16_t) PWM_TOP * percent / 100);
}

void setRGB_GREEN(float percent) {
    pwm_set_chan_level(RGB_GREEN_SLICE, RGB_GREEN_CHAN, (uint16_t) PWM_TOP * percent / 100);
}

void setRGB_BLUE(float percent) {
    pwm_set_chan_level(RGB_BLUE_SLICE, RGB_BLUE_CHAN, (uint16_t) PWM_TOP * percent / 100);
}

void vRGBInit() {
    // initialise gpio pins
    gpio_init(RGB_RED);
    gpio_init(RGB_GREEN);
    gpio_init(RGB_BLUE);

    // set direction as output
    gpio_set_dir(RGB_RED, GPIO_OUT);
    gpio_set_dir(RGB_GREEN, GPIO_OUT);
    gpio_set_dir(RGB_BLUE, GPIO_OUT);

    // set gpio to be pulled up
    gpio_pull_up(RGB_RED);
    gpio_pull_up(RGB_GREEN);
    gpio_pull_up(RGB_BLUE);

    // set gpio function to be pwm
    gpio_set_function(RGB_RED, GPIO_FUNC_PWM);
    gpio_set_function(RGB_GREEN, GPIO_FUNC_PWM);
    gpio_set_function(RGB_BLUE, GPIO_FUNC_PWM);

    // set the clock divider
    pwm_set_clkdiv(RGB_RED_SLICE, CLK_DIVIDER);
    pwm_set_clkdiv(RGB_GREEN_SLICE, CLK_DIVIDER);
    pwm_set_clkdiv(RGB_BLUE_SLICE, CLK_DIVIDER);

    // set the TOP
    pwm_set_wrap(RGB_RED_SLICE, PWM_TOP);
    pwm_set_wrap(RGB_GREEN_SLICE, PWM_TOP);
    pwm_set_wrap(RGB_BLUE_SLICE, PWM_TOP);

    // enable pwm
    pwm_set_enabled(RGB_RED_SLICE, true);
    pwm_set_enabled(RGB_GREEN_SLICE, true);
    pwm_set_enabled(RGB_BLUE_SLICE, true);

    // set initial pwm to 50%
    setRGB_RED(50.0);
    setRGB_GREEN(50.0);
    setRGB_BLUE(50.0);
}

void vRGBTask() {
    float i = 0;
    for (;;) {
        // do stuff
        taskENTER_CRITICAL();
        setRGB_RED(i);
        setRGB_BLUE(i);
        setRGB_GREEN(i);
        taskEXIT_CRITICAL();
        i++;
        if (i >= 100) i = 0;

        // block for some time
        vTaskDelay(VDELAY);
    }
}