#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

#define TRAVERSAL_LHS_INA 1
#define TRAVERSAL_LHS_INB 0
#define TRAVERSAL_LHS_PWM 2

#define TRAVERSAL_RHS_INA 3
#define TRAVERSAL_RHS_INB 4
#define TRAVERSAL_RHS_PWM 5

#define SET_TRAVERSAL_LHS_STOPPED() (
    gpio_put(TRAVERSAL_LHS_INA, 0);
    gpio_put(TRAVERSAL_LHS_INB, 0);
)

#define SET_TRAVERSAL_RHS_STOPPED() (
    gpio_put(TRAVERSAL_RHS_INA, 0);
    gpio_put(TRAVERSAL_RHS_INB, 0);
)

#define SET_TRAVERSAL_LHS_BACKWARD() (
    gpio_put(TRAVERSAL_LHS_INA, 0);
    gpio_put(TRAVERSAL_LHS_INB, 1);
)

#define SET_TRAVERSAL_RHS_BACKWARD() (
    gpio_put(TRAVERSAL_RHS_INA, 0);
    gpio_put(TRAVERSAL_RHS_INB, 1);
)

#define SET_TRAVERSAL_LHS_FORWARD() (
    gpio_put(TRAVERSAL_LHS_INA, 1);
    gpio_put(TRAVERSAL_LHS_INB, 0);
)

#define SET_TRAVERSAL_RHS_FORWARD() (
    gpio_put(TRAVERSAL_RHS_INA, 1);
    gpio_put(TRAVERSAL_RHS_INB, 0);
)

#define SET_TRAVERSAL_LHS_BRAKED() (
    gpio_put(TRAVERSAL_LHS_INA, 1);
    gpio_put(TRAVERSAL_LHS_INB, 1);
)

#define SET_TRAVERSAL_RHS_BRAKED() (
    gpio_put(TRAVERSAL_RHS_INA, 1);
    gpio_put(TRAVERSAL_RHS_INB, 1);
)

#define VDELAY 3

// function prototypes
void vMotorsTask();
void vMotorsInit();

void vMotorsInit() {
    gpio_init(TRAVERSAL_LHS_INA);
    gpio_init(TRAVERSAL_LHS_INB);
    gpio_init(TRAVERSAL_LHS_PWM);
    gpio_init(TRAVERSAL_RHS_INA);
    gpio_init(TRAVERSAL_RHS_INB);
    gpio_init(TRAVERSAL_RHS_PWM);

    gpio_set_dir(TRAVERSAL_LHS_INA, GPIO_OUT);
    gpio_set_dir(TRAVERSAL_LHS_INB, GPIO_OUT);
    gpio_set_dir(TRAVERSAL_LHS_PWM, GPIO_OUT);
    gpio_set_dir(TRAVERSAL_RHS_INA, GPIO_OUT);
    gpio_set_dir(TRAVERSAL_RHS_INB, GPIO_OUT);
    gpio_set_dir(TRAVERSAL_RHS_PWM, GPIO_OUT);
}

void vMotorsTask() {
    for (;;) {
        // do stuff

        // block for some time
        vTaskDelay(VDELAY);
    }
}