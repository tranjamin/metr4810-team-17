/**
Drivers for the motors.

Functions:
    vMotorsInit(): initialises firmware for the motor task
    setTraversalDuty_LHS(): sets the LHS pwm
    setTraversalDuty_RHS(): sets the RHS pwm
Definitions:
    TRAVERSAL_LHS_INA: The pico GPIO pin corresponding to the motor driver INA for LHS
    TRAVERSAL_LHS_INB: The pico GPIO pin corresponding to the motor driver INB for LHS
    TRAVERSAL_LHS_PWM: The pico GPIO pin corresponding to the motor driver PWM for LHS
    TRAVERSAL_RHS_INA: The pico GPIO pin corresponding to the motor driver INA for RHS
    TRAVERSAL_RHS_INB: The pico GPIO pin corresponding to the motor driver INB for RHS
    TRAVERSAL_RHS_PWM: The pico GPIO pin corresponding to the motor driver PWM for RHS

    SET_LHS_STOPPED(): stops the LHS motor
    SET_LHS_FORWARD(): moves the LHS motor forward
    SET_LHS_BACKWARD(): moves the LHS motor backward
    SET_LHS_BRAKED(): brakes the LHS motor
    SET_RHS_STOPPED(): stops the RHS motor
    SET_RHS_FORWARD(): moves the RHS motor forward
    SET_RHS_BACKWARD(): moves the RHS motor backward
    SET_RHS_BRAKED(): brakes the RHS motor
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "FreeRTOSConfig.h"

// Pin Mappings
#define TRAVERSAL_LHS_INA 4
#define TRAVERSAL_LHS_INB 3
#define TRAVERSAL_LHS_PWM 5
#define TRAVERSAL_RHS_INA 1
#define TRAVERSAL_RHS_INB 0
#define TRAVERSAL_RHS_PWM 2

// Motor Direction Macros
#define SET_TRAVERSAL_LHS_STOPPED() (gpio_put(TRAVERSAL_LHS_INA, 0), gpio_put(TRAVERSAL_LHS_INB, 0))
#define SET_TRAVERSAL_RHS_STOPPED() (gpio_put(TRAVERSAL_RHS_INA, 0), gpio_put(TRAVERSAL_RHS_INB, 0))
#define SET_TRAVERSAL_LHS_BACKWARD() (gpio_put(TRAVERSAL_LHS_INA, 0), gpio_put(TRAVERSAL_LHS_INB, 1))
#define SET_TRAVERSAL_RHS_BACKWARD() (gpio_put(TRAVERSAL_RHS_INA, 0), gpio_put(TRAVERSAL_RHS_INB, 1))
#define SET_TRAVERSAL_LHS_FORWARD() (gpio_put(TRAVERSAL_LHS_INA, 1), gpio_put(TRAVERSAL_LHS_INB, 0))
#define SET_TRAVERSAL_RHS_FORWARD() (gpio_put(TRAVERSAL_RHS_INA, 1), gpio_put(TRAVERSAL_RHS_INB, 0))
#define SET_TRAVERSAL_LHS_BRAKED() (gpio_put(TRAVERSAL_LHS_INA, 1), gpio_put(TRAVERSAL_LHS_INB, 1))
#define SET_TRAVERSAL_RHS_BRAKED() (gpio_put(TRAVERSAL_RHS_INA, 1), gpio_put(TRAVERSAL_RHS_INB, 1))

// Duty cycle commands
void setTraversalDuty_LHS(float);
void setTraversalDuty_RHS(float);

void vMotorsInit();

#endif