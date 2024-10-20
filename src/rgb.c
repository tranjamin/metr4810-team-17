// RTOS INCLUDES
#include "FreeRTOS.h"
#include "task.h"

// PICO INCLUDES
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// PIN CONNECTIONS
#define RGB_RED 12
#define RGB_GREEN 13
#define RGB_BLUE 14

// PWM DEFINITIONS (SLICES AND CHANNELS)
#define RGB_RED_SLICE pwm_gpio_to_slice_num(RGB_RED)
#define RGB_RED_CHAN pwm_gpio_to_channel(RGB_RED)
#define RGB_GREEN_SLICE pwm_gpio_to_slice_num(RGB_GREEN)
#define RGB_GREEN_CHAN pwm_gpio_to_channel(RGB_GREEN)
#define RGB_BLUE_SLICE pwm_gpio_to_slice_num(RGB_BLUE)
#define RGB_BLUE_CHAN pwm_gpio_to_channel(RGB_BLUE)

// PWM CONFIGURATION OPTIONS
#define CLK_DIVIDER 8 // clock divider for PWM timer
#define PWM_TOP 8192 // clock saturation level for PWM timer

// RTOS BLOCKING TIME
#define VDELAY 100

// Function Prototypes
void vRGBInit();
void setRGB_RED(float);
void setRGB_GREEN(float);
void setRGB_BLUE(float);
void setRGB_COLOUR_BLACK();
void setRGB_COLOUR_BLUE();
void setRGB_COLOUR_CYAN();
void setRGB_COLOUR_DARK_BLUE();
void setRGB_COLOUR_DARK_GREEN();
void setRGB_COLOUR_DARK_RED();
void setRGB_COLOUR_GRAY();
void setRGB_COLOUR_GREEN();
void setRGB_COLOUR_PURPLE();
void setRGB_COLOUR_RED();
void setRGB_COLOUR_WHITE();
void setRGB_COLOUR_YELLOW();

#define SET_RGB_COLOUR(R, G, B) (setRGB_RED(R), setRGB_BLUE(B), setRGB_GREEN(G)) // sets RGB colour of motor

/**
Sets the RGB to red
 */
void setRGB_COLOUR_RED() {
    SET_RGB_COLOUR(100, 0, 0);
}

/**
Sets the RGB to dark red
 */
void setRGB_COLOUR_DARK_RED() {
    SET_RGB_COLOUR(1, 0, 0);
}

/**
Sets the RGB to green
 */
void setRGB_COLOUR_GREEN() {
    SET_RGB_COLOUR(0, 100, 0);
}

/**
Sets the RGB to dark green
 */
void setRGB_COLOUR_DARK_GREEN() {
    SET_RGB_COLOUR(0, 1, 0);
}

/**
Sets the RGB to blue
 */
void setRGB_COLOUR_BLUE() {
    SET_RGB_COLOUR(0, 0, 100);
}

/**
Sets the RGB to dark blue
 */
void setRGB_COLOUR_DARK_BLUE() {
    SET_RGB_COLOUR(0, 0, 1);
}

/**
Sets the RGB to white
 */
void setRGB_COLOUR_WHITE() {
    SET_RGB_COLOUR(50, 70, 100);
}

/**
Sets the RGB to purple
 */
void setRGB_COLOUR_PURPLE() {
    SET_RGB_COLOUR(50, 0, 100);
}

/**
Sets the RGB to cyan
 */
void setRGB_COLOUR_CYAN() {
    SET_RGB_COLOUR(0, 70, 100);
}

/**
Sets the RGB to yellow
 */
void setRGB_COLOUR_YELLOW() {
    SET_RGB_COLOUR(50, 70, 0);
}

/**
Sets the RGB to black
 */
void setRGB_COLOUR_BLACK() {
    SET_RGB_COLOUR(0, 0, 0);
}

/**
Sets the RGB to gray
 */
void setRGB_COLOUR_GRAY() {
    SET_RGB_COLOUR(5, 7, 10);
}

/**
Sets the red channel of the RGB to an intensity

Parameters:
    percent: the percent intensity of red (0 - 100)
 */
void setRGB_RED(float percent) {
    pwm_set_chan_level(RGB_RED_SLICE, RGB_RED_CHAN, (uint16_t) PWM_TOP * (100 - percent) / 100);
}

/**
Sets the green channel of the RGB to an intensity

Parameters:
    percent: the percent intensity of red (0 - 100)
 */
void setRGB_GREEN(float percent) {
    pwm_set_chan_level(RGB_GREEN_SLICE, RGB_GREEN_CHAN, (uint16_t) PWM_TOP * (100 - percent) / 100);
}

/**
Sets the blue channel of the RGB to an intensity

Parameters:
    percent: the percent intensity of red (0 - 100)
 */
void setRGB_BLUE(float percent) {
    pwm_set_chan_level(RGB_BLUE_SLICE, RGB_BLUE_CHAN, (uint16_t) PWM_TOP * (100 - percent) / 100);
}

/**
Sets up the hardware drivers for the rgb task.
 */
void vRGBInit() {
    // initialise all gpio pins
    gpio_init(RGB_RED);
    gpio_init(RGB_GREEN);
    gpio_init(RGB_BLUE);

    // set gpio pins as outputs
    gpio_set_dir(RGB_RED, GPIO_OUT);
    gpio_set_dir(RGB_GREEN, GPIO_OUT);
    gpio_set_dir(RGB_BLUE, GPIO_OUT);

    // set gpio pins to be pulled up
    gpio_pull_up(RGB_RED);
    gpio_pull_up(RGB_GREEN);
    gpio_pull_up(RGB_BLUE);

    // set gpio functions to be pwm
    gpio_set_function(RGB_RED, GPIO_FUNC_PWM);
    gpio_set_function(RGB_GREEN, GPIO_FUNC_PWM);
    gpio_set_function(RGB_BLUE, GPIO_FUNC_PWM);

    // set the clock dividers
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

    // set initial pwm to 0
    setRGB_RED(0.0);
    setRGB_GREEN(0.0);
    setRGB_BLUE(0.0);
}