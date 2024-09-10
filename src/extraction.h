#ifndef EXTRACTION_H
#define EXTRACTION_H

#include "FreeRTOSConfig.h"

#define EXTRACTION_TASK_NAME "Bean Delivery Task"
#define EXTRACTION_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define EXTRACTION_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define EXTRACTION_TASK_COREMASK 0x01

void vExtractionTask();
void vExtractionInit();
void setExtractionPWM(float);

#endif