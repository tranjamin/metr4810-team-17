// RTOS INCLUDES
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// PICO INCLUDES
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// DRIVER INCLUDES
#include "motors.h"
#include "extraction.h"
#include "diagnostics.h"
#include "wifi.h"

// PIN CONNECTIONS
#define DELIVERY_INA 10 // the pico GPIO pin corresponding to motor driver INA
#define DELIVERY_INB 9 // the pico GPIO pin corresponding to motor driver INB
#define DELIVERY_PWM 11 // the pico GPIO pin corresponding to motor driver PWM

// PWM DEFINITIONS
#define DELIVERY_PWM_CHAN pwm_gpio_to_channel(DELIVERY_PWM) // the PWM channel operating on
#define DELIVERY_PWM_SLICE pwm_gpio_to_slice_num(DELIVERY_PWM) // the PWM slice operating on

// MOTOR DIRECTION MACROS
#define SET_DELIVERY_STOPPED() (gpio_put(DELIVERY_INA, 0), gpio_put(DELIVERY_INB, 0)) // set motor stopped
#define SET_DELIVERY_FORWARD() (gpio_put(DELIVERY_INA, 1), gpio_put(DELIVERY_INB, 0)) // set motor forward
#define SET_DELIVERY_BACKWARD() (gpio_put(DELIVERY_INA, 0), gpio_put(DELIVERY_INB, 1)) // set motor backward
#define SET_DELIVERY_BRAKED() (gpio_put(DELIVERY_INA, 1), gpio_put(DELIVERY_INB, 1)) // set motor braked

// RTOS BLOCKING TIMES
#define VDELAY 3 // time to block between loops
#define SEMPH_TICKS 10 // time to block to wait for delivery semaphore

// PWM CONFIGURATION OPTIONS
#define CLK_DIVIDER 128 // clock divider for PWM timer
#define PWM_TOP 8192 // clock saturation level for PWM timer
#define DELIVERY_PWM_SPEED 70 // PWM speed to run the delivery motor at

// STATE MACHINE STATES
#define IDLE 0 // not delivering
#define FORWARD 1 // pushing beans out
#define BACKWARD 2 // pulling mechanism back

// DELIVERY SEQUENCE TIMINGS
#define FORWARD_TIME_MS 11000 // time to push beans out
#define BACKWARD_TIME_MS 11000 // time to retract back

SemaphoreHandle_t deliverySemaphore; // the semaphore which controls when to start delivery
volatile int delivery_state; // the current state of the FSM
alarm_pool_t* delivery_alarm_pool; // the alarm pool holding the timers for the sequence
volatile alarm_id_t current_alarm; // the current alarm that is active

// Function Prototypes
void setDeliveryPWM(float percent);
void vRunDelivery();

void vDeliveryInit();
void vDeliveryTask();

void transition_to_idle();
void transition_to_backward();

/**
Sets the PWM of the delivery motor

Parameters:
    percent: the percent pwm, from 0 to 100.
 */
void setDeliveryPWM(float percent) {
    pwm_set_chan_level(DELIVERY_PWM_SLICE, DELIVERY_PWM_CHAN, (uint16_t) PWM_TOP * percent / 100);
}

/**
Starts the delivery sequence.
 */
void vRunDelivery() {
    xSemaphoreGiveFromISR(deliverySemaphore, NULL);
}

/**
Sets up the hardware drivers for the delivery task.
 */
void vDeliveryInit() {
    // initialise all gpio pins
    gpio_init(DELIVERY_INA);
    gpio_init(DELIVERY_INB);
    gpio_init(DELIVERY_PWM);

    // set gpio pins as outputs
    gpio_set_dir(DELIVERY_INA, GPIO_OUT);
    gpio_set_dir(DELIVERY_INB, GPIO_OUT);
    gpio_set_dir(DELIVERY_PWM, GPIO_OUT);
    
    gpio_set_function(DELIVERY_PWM, GPIO_FUNC_PWM); // set gpio function as pwm
    pwm_set_phase_correct(DELIVERY_PWM_SLICE, true); // set phase correction for better efficiency
    pwm_set_wrap(DELIVERY_PWM_SLICE, PWM_TOP); // set the TOP of the pwm timer

    pwm_set_enabled(DELIVERY_PWM_SLICE, true); // enable pwm
    setDeliveryPWM(DELIVERY_PWM_SPEED); // set pwm for delivery motor
    SET_DELIVERY_STOPPED(); // initially stopped

    deliverySemaphore = xSemaphoreCreateBinary(); // set up semaphore to start delivery process
    delivery_state = IDLE; // set initial state to IDLE
    
    // initialise alarm pool
    alarm_pool_init_default();
    delivery_alarm_pool = alarm_pool_create(0, 2);
}

/**
Handles the transition between moving backwards and idling
 */
void transition_to_idle() {
    alarm_pool_cancel_alarm(delivery_alarm_pool, current_alarm); // cancel the backwards alarm
    delivery_state = IDLE; // set FSM state
    SET_DELIVERY_STOPPED(); // stop the delivery motor
    enableUDP(); // re enable handling of UDP messages
}

/**
Handles the transition between moving forwards and backwards
 */
void transition_to_backward() {
    alarm_pool_cancel_alarm(delivery_alarm_pool, current_alarm); // cancel the forward alarm
    delivery_state = BACKWARD; // set FSM state
    SET_DELIVERY_BACKWARD(); // move backwards

    // create new alarm for backwards movement
    current_alarm = alarm_pool_add_alarm_in_ms(
        delivery_alarm_pool, // alarm pool reference
        BACKWARD_TIME_MS, // timeout time
        (alarm_callback_t) transition_to_idle, // callback to execture on timeout
        NULL, // callback takes no parameters
        false // don't execute the callback if it has already expired
        );
}

/**
Starts the delivery sequence
 */
void vStartDelivery() {
    xSemaphoreGiveFromISR(deliverySemaphore, NULL);
}

/**
The main loop for this task
 */
void vDeliveryTask() {
    for (;;) {
        switch (delivery_state) {
            case IDLE:
                if (xSemaphoreTake(deliverySemaphore, SEMPH_TICKS) == pdTRUE) { // if delivery has been started
                    delivery_state = FORWARD;
                    
                    // suspend other tasks
                    disableUDP();
                    SET_TRAVERSAL_RHS_STOPPED();
                    SET_TRAVERSAL_LHS_STOPPED();
                    SET_EXTRACTION_STOPPED();

                    // set up alarm timer
                    current_alarm = alarm_pool_add_alarm_in_ms(
                        delivery_alarm_pool, // alarm pool to reference
                        FORWARD_TIME_MS, // callback expiry time
                        (alarm_callback_t) transition_to_backward, // callback to fire
                        NULL, // callback takes no arguments
                        false // don't execute the callback if already expired
                    );
                    
                    // run delivery motor
                    SET_DELIVERY_FORWARD();
                }
                break;
            case FORWARD:
                break;
            case BACKWARD:
                break;
        }

        // block for some time
        vTaskDelay(VDELAY);
    }
}