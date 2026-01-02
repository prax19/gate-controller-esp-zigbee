#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gate_driver gate_driver_t;

typedef enum {
    GATE_OPEN = 0,
    GATE_CLOSE,
    GATE_PEDESTRIAN,
    GATE_STOP,
} gate_cmd_t;

gate_driver_t* gate_driver_create(void);

void gate_driver_destroy(gate_driver_t* d);

void gate_driver_command(gate_driver_t* d, gate_cmd_t cmd);

#ifdef __cplusplus
}
#endif
