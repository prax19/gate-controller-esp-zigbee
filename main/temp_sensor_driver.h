#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct temp_sensor_driver temp_sensor_driver_t;

typedef void (*temp_sensor_cb_t)(float temp_c, void *user_ctx);

typedef struct {
    gpio_num_t gpio_num;     // default: GPIO_NUM_11
    uint32_t period_ms;      // default: 10000
    bool enable_pullup;      // default: true (typowe dla 1-Wire)
} temp_sensor_config_t;

#define TEMP_SENSOR_DRIVER_DEFAULT_CONFIG() \
    (temp_sensor_config_t){                \
        .gpio_num = GPIO_NUM_11,           \
        .period_ms = 10000,                \
        .enable_pullup = true,             \
    }

temp_sensor_driver_t *temp_sensor_driver_create(const temp_sensor_config_t *cfg,
                                                temp_sensor_cb_t cb,
                                                void *user_ctx);

void temp_sensor_driver_destroy(temp_sensor_driver_t *handle);

bool temp_sensor_driver_get_last(temp_sensor_driver_t *handle,
                                 float *out_temp_c,
                                 int64_t *out_timestamp_us);

void temp_sensor_driver_trigger(temp_sensor_driver_t *handle);

bool temp_sensor_driver_is_ready(temp_sensor_driver_t *handle);

#ifdef __cplusplus
}
#endif
