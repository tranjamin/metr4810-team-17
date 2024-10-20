/**
Drivers and RTOS tasks for the bean extraction.

Functions:
    vExtractionInit(): initialises firmware for the delivery task
    vExtractionTask(): the main loop of this task
    extractionProcedureStart(): starts the extraction sequence.
    extractionProcedureStop(): stops the extraction sequence.
    extractionManualStart(): manually starts the extraction.
    extractionManualStop(): stops the extraction in the right configuration.

Definitions:
    EXTRACTION_INA: The pico GPIO pin corresponding to motor driver INA
    EXTRACTION_INB: The pico GPIO pin corresponding to motor driver INB
    EXTRACTION_WM: The pico GPIO pin corresponding to motor driver PWM

    EXTRACTION_TASK_NAME: the RTOS name of this task
    EXTRACTION_TASK_PRIORITY: the RTOS priority of this task
    EXTRACTION_TASK_STACK_SIZE: the RTOS stack size of this task
    EXTRACTION_TASK_COREMASK: the RTOS coremask of this task

    SET_EXTRACTION_STOPPED(): stops the extraction motor
    SET_EXTRACTION_FORWARD(): moves the extraction motor forward
    SET_EXTRACTION_BACKWARD(): moves the extraction motor backward
    SET_EXTRACTION_BRAKED(): brakes the extraction motor
 */

#ifndef EXTRACTION_H
#define EXTRACTION_H

#include "FreeRTOSConfig.h"

// RTOS Task Settings
#define EXTRACTION_TASK_NAME "Bean Extraction Task"
#define EXTRACTION_TASK_PRIORITY (tskIDLE_PRIORITY + 8)
#define EXTRACTION_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define EXTRACTION_TASK_COREMASK 0x01

// Pin Mappings
#define EXTRACTION_INA 6
#define EXTRACTION_INB 7
#define EXTRACTION_PWM 8

// Motor Direction Macros
#define SET_EXTRACTION_STOPPED() (gpio_put(EXTRACTION_INA, 0), gpio_put(EXTRACTION_INB, 0))
#define SET_EXTRACTION_FORWARD() (gpio_put(EXTRACTION_INA, 1), gpio_put(EXTRACTION_INB, 0))
#define SET_EXTRACTION_BACKWARD() (gpio_put(EXTRACTION_INA, 0), gpio_put(EXTRACTION_INB, 1))
#define SET_EXTRACTION_BRAKED() (gpio_put(EXTRACTION_INA, 1), gpio_put(EXTRACTION_INB, 1))

// Functions
void vExtractionInit();
void vExtractionTask();
void extractionProcedureSignalStop();
void extractionProcedureSignalStart();
void extractionManualStart();
void extractionManualStop();

#endif