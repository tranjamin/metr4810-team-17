#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#ifdef CYW43_WL_GPIO_LED_PIN
    #include "pico/cyw43_arch.h"
#endif

#define VDELAY 250

int pico_led_init(void) {
    #if defined(LED_PIN)
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        return PICO_OK;
    #elif defined(CYW43_WL_GPIO_LED_PIN)
        if (cyw43_arch_init()) return 1;
        cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
    #endif
}

void pico_set_led(bool led_on) {
    #if defined(PICO_DEFAULT_LED_PIN)
        gpio_put(PICO_DEFAULT_LED_PIN, led_on);
    #elif defined(CYW43_WL_GPIO_LED_PIN)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
    #endif
}

void vBlinkTask() {
   for (;;) {
        pico_set_led(true);
        vTaskDelay(VDELAY);
        pico_set_led(false);
        vTaskDelay(VDELAY);
    }
}

void vBlinkInit() {
    hard_assert(pico_led_init() == PICO_OK);
}