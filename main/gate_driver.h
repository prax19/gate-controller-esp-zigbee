#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIGNAL_PEDESTRIAN_PIN     GPIO_NUM_18
#define SIGNAL_SEQUENCE_PIN       GPIO_NUM_19
#define SIGNAL_OPEN_PIN           GPIO_NUM_20
#define SIGNAL_CLOSE_PIN          GPIO_NUM_21

#define SIGNAL_DURATION           2000

typedef enum {
    GATE_OPEN,
    GATE_CLOSE,
    GATE_PEDESTRIAN,
    GATE_SEQ_OPEN,
} gate_cmd;

void gate_driver_init();
void call_gate_cmd(gate_cmd state);

#ifdef __cplusplus
}
#endif