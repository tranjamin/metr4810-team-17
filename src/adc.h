#ifndef ADC_H
#define ADC_H

#include "FreeRTOSConfig.h"

#define ADC_TASK_NAME "ADC Task"
#define ADC_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define ADC_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define ADC_TASK_COREMASK 0x01

void vADCTask();
void vADCInit();

#endif