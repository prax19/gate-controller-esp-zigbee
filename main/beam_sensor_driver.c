#include "beam_sensor_driver.h"

void beam_sensor_driver_init() {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BEAM_SENSOR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = BEAM_SENSOR_DEFAULT_LOW ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = (!BEAM_SENSOR_DEFAULT_LOW) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}

bool beam_read(void) {
    bool lvl = gpio_get_level(BEAM_SENSOR_GPIO);
    return BEAM_SENSOR_DEFAULT_LOW ? !lvl : lvl;
}