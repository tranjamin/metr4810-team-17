/**
Drivers and RTOS tasks for the RGB LED.

Functions:
    setRGB_RED(): sets the red intensity
    setRGB_GREEN(): sets the green intensity
    setRGB_BLUE(): sets the blue intensity

    Colour presets:
        setRGB_COLOUR_RED()
        setRGB_COLOUR_GREEN()
        setRGB_COLOUR_BLUE()
        setRGB_COLOUR_DARK_RED()
        setRGB_COLOUR_DARK_GREEN()
        setRGB_COLOUR_DARK_BLUE()
        setRGB_COLOUR_PURPLE()
        setRGB_COLOUR_CYAN()
        setRGB_COLOUR_YELLOW()
        setRGB_COLOUR_WHITE()
        setRGB_COLOUR_GRAY()
        setRGB_COLOUR_BLACK()

Definitions:
    RGB_TASK_NAME: the RTOS name of this task
    RGB_TASK_PRIORITY: the RTOS priority of this task
    RGB_TASK_STACK_SIZE: the RTOS stack size of this task
    RGB_TASK_COREMASK: the RTOS coremask of this task
 */

#ifndef RGB_H
#define RGB_H

#include "FreeRTOSConfig.h"

// RTOS Task Settings
#define RGB_TASK_NAME "RGB LED Control Task"
#define RGB_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define RGB_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define RGB_TASK_COREMASK 0x01

// Sets the intensity of colour as a percent (i.e. 0-100)
void setRGB_RED(float);
void setRGB_GREEN(float);
void setRGB_BLUE(float);

// Presets for different colours
void setRGB_COLOUR_RED();
void setRGB_COLOUR_GREEN();
void setRGB_COLOUR_BLUE();
void setRGB_COLOUR_DARK_RED();
void setRGB_COLOUR_DARK_GREEN();
void setRGB_COLOUR_DARK_BLUE();
void setRGB_COLOUR_PURPLE();
void setRGB_COLOUR_CYAN();
void setRGB_COLOUR_YELLOW();
void setRGB_COLOUR_WHITE();
void setRGB_COLOUR_GRAY();
void setRGB_COLOUR_BLACK();

void vRGBInit();

#endif