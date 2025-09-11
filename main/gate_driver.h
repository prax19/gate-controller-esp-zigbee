#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIGNAL_PEDESTRIAN_PIN     GPIO_NUM_0
#define SIGNAL_SEQUENCE_PIN       GPIO_NUM_2
#define SIGNAL_OPEN_PIN           GPIO_NUM_22
#define SIGNAL_CLOSE_PIN          GPIO_NUM_16

#define SIGNAL_DURATION           200

typedef enum {
    GATE_OPEN,
    GATE_CLOSE,
    GATE_PEDESTRIAN,
    GATE_SEQ_OPEN,
} gate_state;

void gate_driver_init(gate_state initial_state);
void short_signal(gpio_num_t gpio);
void set_gate_state(gate_state state);

#ifdef __cplusplus
}
#endif