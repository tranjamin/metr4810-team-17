#ifndef WATCHDOG_H
#define WATCHDOG_H

#define WATCHDOG_TASK_NAME "Watchdog Task"
#define WATCHDOG_TASK_PRIORITY (tskIDLE_PRIORITY + 15)
#define WATCHDOG_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define WATCHDOG_TASK_COREMASK 0x01

void vWatchdogTask();
void vWatchdogInit();

#endif