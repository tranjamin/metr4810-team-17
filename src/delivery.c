#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "semphr.h"

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

SemaphoreHandle_t deliverySemaphore;

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

    deliverySemaphore = xSemaphoreCreateBinary();
}

void vDeliveryTask() {
    for (;;) {
        // wait to take semaphore
        if (xSemaphoreTake(deliverySemaphore, SEMPH_TICKS) == pdTRUE) {
            // suspend other tasks

            // run delivery motor
            SET_DELIVERY_FORWARD();

            SET_DELIVERY_BACKWARD();

            SET_DELIVERY_STOPPED();

            xSemaphoreGive(deliverySemaphore);
        }
        // block for some time
        vTaskDelay(VDELAY);
    }
}