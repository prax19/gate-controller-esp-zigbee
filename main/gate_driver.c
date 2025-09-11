#include "gate_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void gate_pin_init(gpio_num_t gpio, bool active_high) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(gpio, active_high ? 0 : 1);
}

void gate_driver_init(gate_state initial_state) {
    gate_pin_init(SIGNAL_PEDESTRIAN_PIN, true);
    gate_pin_init(SIGNAL_SEQUENCE_PIN, true);
    gate_pin_init(SIGNAL_OPEN_PIN, true);
    gate_pin_init(SIGNAL_CLOSE_PIN, true);
}

void short_signal(gpio_num_t gpio) {
    gpio_set_level(gpio, true);
    vTaskDelay(pdMS_TO_TICKS(SIGNAL_DURATION));
    gpio_set_level(gpio, false);
}

void set_gate_state(gate_state state) {
        switch (state) {
        case GATE_PEDESTRIAN:
            short_signal(SIGNAL_PEDESTRIAN_PIN);
            break;
        case GATE_SEQ_OPEN:
            short_signal(SIGNAL_SEQUENCE_PIN);
            break;
        case GATE_OPEN:
            short_signal(SIGNAL_OPEN_PIN);
            break;
        case GATE_CLOSE:
            short_signal(SIGNAL_CLOSE_PIN);
            break;
        default:
            break;
    }
}