#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct led_driver led_driver_t;

typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_ON,
    LED_MODE_BLINK,       // 1000 ms
    LED_MODE_BLINK_FAST,  // 250 ms
} led_mode_t;

typedef enum {
    LED_COLOR_RED = 0,
    LED_COLOR_GREEN,
    LED_COLOR_BLUE,
} led_color_t;

led_driver_t* led_driver_create(void);
void led_driver_destroy(led_driver_t* d);

void led_driver_set(led_driver_t* d, led_mode_t mode, led_color_t color);

void led_driver_sleep(led_driver_t* d, uint32_t ms);

void led_driver_blink_once(led_driver_t* d, led_color_t color);

#ifdef __cplusplus
}
#endif
