#pragma once
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LED_DEFAULT_ON  1
#define LED_DEFAULT_OFF 0

#define LED_GPIO           GPIO_NUM_1
#define LED_ACTIVE_LEVEL   0 
#define LED_INACTIVE_LEVEL (!LED_ACTIVE_LEVEL)

void led_driver_init(bool power);
void set_user_led_state(bool power);

#ifdef __cplusplus
}
#endif
