#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct beam_sensor_driver beam_sensor_driver_t;

typedef void (*beam_sensor_cb_t)(bool occupied, void *user_ctx);

beam_sensor_driver_t* beam_sensor_driver_create(beam_sensor_cb_t cb, void *user_ctx);

void beam_sensor_driver_destroy(beam_sensor_driver_t *d);

bool beam_sensor_driver_get_occupied(beam_sensor_driver_t *d);

void beam_sensor_driver_set_filter_ms(beam_sensor_driver_t *handle, uint32_t filter_ms);


#ifdef __cplusplus
}
#endif
