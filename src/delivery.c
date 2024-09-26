#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "semphr.h"
#include "hardware/pwm.h"

#include "motors.h"
#include "extraction.h"
#include "diagnostics.h"

#define DELIVERY_INA 10
#define DELIVERY_INB 9
#define DELIVERY_PWM 11

#define DELIVERY_PWM_CHAN pwm_gpio_to_channel(DELIVERY_PWM)
#define DELIVERY_PWM_SLICE pwm_gpio_to_slice_num(DELIVERY_PWM)

#define SET_DELIVERY_STOPPED() (gpio_put(DELIVERY_INA, 0), gpio_put(DELIVERY_INB, 0))
#define SET_DELIVERY_FORWARD() (gpio_put(DELIVERY_INA, 1), gpio_put(DELIVERY_INB, 0))
#define SET_DELIVERY_BACKWARD() (gpio_put(DELIVERY_INA, 0), gpio_put(DELIVERY_INB, 1))
#define SET_DELIVERY_BRAKED() (gpio_put(DELIVERY_INA, 1), gpio_put(DELIVERY_INB, 1))

#define VDELAY 3
#define SEMPH_TICKS 10

#define CLK_DIVIDER 128
#define PWM_TOP 8192

#define IDLE 0
#define FORWARD 1
#define BACKWARD 2

#define FORWARD_TIME_MS 5000
#define BACKWARD_TIME_MS 5000

SemaphoreHandle_t deliverySemaphore;
volatile int delivery_state;
alarm_pool_t* delivery_alarm_pool;
volatile alarm_id_t current_alarm;


void setDeliveryPWM(float percent) {
    pwm_set_chan_level(DELIVERY_PWM_SLICE, DELIVERY_PWM_CHAN, (uint16_t) PWM_TOP * percent / 100);
}

void vRunDelivery() {
    xSemaphoreGiveFromISR(deliverySemaphore, NULL);
}

// function prototypes
void vDeliveryTask();
void vDeliveryInit();

void vDeliveryInit() {
    gpio_init(DELIVERY_INA);
    gpio_init(DELIVERY_INB);
    gpio_init(DELIVERY_PWM);

    gpio_set_dir(DELIVERY_INA, GPIO_OUT);
    gpio_set_dir(DELIVERY_INB, GPIO_OUT);
    gpio_set_dir(DELIVERY_PWM, GPIO_OUT);
    
    // set gpio function
    gpio_set_function(DELIVERY_PWM, GPIO_FUNC_PWM);

    // set phase correction for better efficiency
    pwm_set_phase_correct(DELIVERY_PWM_SLICE, true);

    // set the TOP
    pwm_set_wrap(DELIVERY_PWM_SLICE, PWM_TOP);

    // enable pwm
    pwm_set_enabled(DELIVERY_PWM_SLICE, true);
    setDeliveryPWM(30);

    // set up semaphore
    deliverySemaphore = xSemaphoreCreateBinary();
    delivery_state = IDLE;
    

    // set up alarm
    alarm_pool_init_default();
    delivery_alarm_pool = alarm_pool_create(0, 2);
}

void transition_to_idle() {
    alarm_pool_cancel_alarm(delivery_alarm_pool, current_alarm);
    delivery_state = IDLE;
    SET_DELIVERY_STOPPED();
}

void transition_to_backward() {
    alarm_pool_cancel_alarm(delivery_alarm_pool, current_alarm);
    delivery_state = BACKWARD;
    SET_DELIVERY_BACKWARD();
    current_alarm = alarm_pool_add_alarm_in_ms(delivery_alarm_pool, BACKWARD_TIME_MS, (alarm_callback_t) transition_to_idle, NULL, false);
}

void vStartDelivery() {
    xSemaphoreGiveFromISR(deliverySemaphore, NULL);
}

void vDeliveryTask() {
    for (;;) {
        switch (delivery_state) {
            case IDLE:
                if (xSemaphoreTake(deliverySemaphore, SEMPH_TICKS) == pdTRUE) {
                    delivery_state = FORWARD;
                    
                    // suspend other tasks
                    SET_TRAVERSAL_RHS_STOPPED();
                    SET_TRAVERSAL_LHS_STOPPED();
                    SET_EXTRACTION_STOPPED();

                    // set up alarm timer
                    current_alarm = alarm_pool_add_alarm_in_ms(delivery_alarm_pool, FORWARD_TIME_MS, (alarm_callback_t) transition_to_backward, NULL, false);
                    
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