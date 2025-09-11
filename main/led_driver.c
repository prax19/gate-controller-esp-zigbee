#include "led_driver.h"

static bool s_power = false;

void set_user_led_state(bool power)
{
    s_power = power;
    gpio_set_level(LED_GPIO, power ? LED_ACTIVE_LEVEL : LED_INACTIVE_LEVEL);
}

void led_driver_init(bool power)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    set_user_led_state(power);
}
