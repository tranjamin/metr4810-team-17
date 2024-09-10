#ifndef RGB_H
#define RGB_H

#include "FreeRTOSConfig.h"

#define RGB_TASK_NAME "RGB LED Control Task"
#define RGB_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define RGB_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define RGB_TASK_COREMASK 0x01

// Sets the intensity of colour as a percent (i.e. 0-100)
void setRGB_RED(float);
void setRGB_GREEN(float);
void setRGB_BLUE(float);

void setRGB_COLOUR_RED();
void setRGB_COLOUR_GREEN();
void setRGB_COLOUR_BLUE();

void vRGBTask();
void vRGBInit();

#endif