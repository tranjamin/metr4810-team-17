#ifndef BLINK_H
#define BLINK_H

#define BLINK_TASK_NAME "Blink Task"
#define BLINK_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define BLINK_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define BLINK_TASK_COREMASK 0x01

void vBlinkTask();
void vBlinkInit();

#endif