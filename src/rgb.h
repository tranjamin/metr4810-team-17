#ifndef RGB_H
#define RGB_H

#include "FreeRTOSConfig.h"

#define RGB_TASK_NAME "RGB LED Control Task"
#define RGB_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define RGB_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define RGB_TASK_COREMASK 0x01

void setRGB_Red(float);
void setRGB_Green(float);
void setRGB_Blue(float);

void vRGBTask();
void vRGBInit();

#endif