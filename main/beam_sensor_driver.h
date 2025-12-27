#include "driver/gpio.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BEAM_SENSOR_DEFAULT_LOW 1

#define BEAM_SENSOR_GPIO           GPIO_NUM_22

void beam_sensor_driver_init();
bool beam_read();

#ifdef __cplusplus
}
#endif