#ifndef EXTRACTION_H
#define EXTRACTION_H

#include "FreeRTOSConfig.h"

#define EXTRACTION_TASK_NAME "Bean Delivery Task"
#define EXTRACTION_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define EXTRACTION_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define EXTRACTION_TASK_COREMASK 0x01

#define EXTRACTION_INA 6
#define EXTRACTION_INB 7
#define EXTRACTION_PWM 8

#define SET_EXTRACTION_STOPPED() (gpio_put(EXTRACTION_INA, 0), gpio_put(EXTRACTION_INB, 0))
#define SET_EXTRACTION_FORWARD() (gpio_put(EXTRACTION_INA, 1), gpio_put(EXTRACTION_INB, 0))
#define SET_EXTRACTION_BACKWARD() (gpio_put(EXTRACTION_INA, 0), gpio_put(EXTRACTION_INB, 1))
#define SET_EXTRACTION_BRAKED() (gpio_put(EXTRACTION_INA, 1), gpio_put(EXTRACTION_INB, 1))

void vExtractionInit();
void setExtractionPWM(float);

#endif