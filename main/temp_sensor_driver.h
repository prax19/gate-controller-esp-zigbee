#include <stdbool.h>
#include "driver/gpio.h"

#include "onewire_bus.h"
#include "ds18b20.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_PIN     GPIO_NUM_11

void temp_driver_init();
float read_temperature();

#ifdef __cplusplus
}
#endif