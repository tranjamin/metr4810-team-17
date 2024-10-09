#ifndef MOTORS_H
#define MOTORS_H

#include "FreeRTOSConfig.h"

#define MOTORS_TASK_NAME "Motor Control Task"
#define MOTORS_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define MOTORS_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define MOTORS_TASK_COREMASK 0x01

#define TRAVERSAL_LHS_INA 4
#define TRAVERSAL_LHS_INB 3
#define TRAVERSAL_LHS_PWM 5

#define TRAVERSAL_RHS_INA 1
#define TRAVERSAL_RHS_INB 0
#define TRAVERSAL_RHS_PWM 2

#define SET_TRAVERSAL_LHS_STOPPED() (gpio_put(TRAVERSAL_LHS_INA, 0), gpio_put(TRAVERSAL_LHS_INB, 0))
#define SET_TRAVERSAL_RHS_STOPPED() (gpio_put(TRAVERSAL_RHS_INA, 0), gpio_put(TRAVERSAL_RHS_INB, 0))
#define SET_TRAVERSAL_LHS_BACKWARD() (gpio_put(TRAVERSAL_LHS_INA, 0), gpio_put(TRAVERSAL_LHS_INB, 1))
#define SET_TRAVERSAL_RHS_BACKWARD() (gpio_put(TRAVERSAL_RHS_INA, 0), gpio_put(TRAVERSAL_RHS_INB, 1))
#define SET_TRAVERSAL_LHS_FORWARD() (gpio_put(TRAVERSAL_LHS_INA, 1), gpio_put(TRAVERSAL_LHS_INB, 0))
#define SET_TRAVERSAL_RHS_FORWARD() (gpio_put(TRAVERSAL_RHS_INA, 1), gpio_put(TRAVERSAL_RHS_INB, 0))
#define SET_TRAVERSAL_LHS_BRAKED() (gpio_put(TRAVERSAL_LHS_INA, 1), gpio_put(TRAVERSAL_LHS_INB, 1))
#define SET_TRAVERSAL_RHS_BRAKED() (gpio_put(TRAVERSAL_RHS_INA, 1), gpio_put(TRAVERSAL_RHS_INB, 1))

// set the duty cycles of the pwms (as a percent)
void setTraversalDuty_LHS(float);
void setTraversalDuty_RHS(float);

void vMotorsTask();
void vMotorsInit();

#endif