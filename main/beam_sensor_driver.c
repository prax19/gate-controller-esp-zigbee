#include "beam_sensor_driver.h"

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#define BEAM_SENSOR_GPIO            GPIO_NUM_22

#define BEAM_SENSOR_DEFAULT_LOW     1

#define BEAM_SENSOR_FILTER_DEFAULT_MS  200

#define BEAM_SENSOR_POLL_MS         20

#define TASK_STACK                  2048
#define TASK_PRIO                   5

#define NTFY_STOP                   (1u << 31)

struct beam_sensor_driver {
    TaskHandle_t task;
    SemaphoreHandle_t done;

    beam_sensor_cb_t cb;
    void *cb_ctx;

    volatile bool occupied;

    bool stable;
    bool pending_active;
    bool pending_value;
    TickType_t pending_since;

    volatile TickType_t filter_ticks;
};

static inline bool raw_to_occupied(int raw_level)
{
    return raw_level ? false : true;
}

static void gpio_init_input(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BEAM_SENSOR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = BEAM_SENSOR_DEFAULT_LOW ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = (!BEAM_SENSOR_DEFAULT_LOW) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}

static void maybe_publish(struct beam_sensor_driver *d, bool new_occupied)
{
    d->occupied = new_occupied;
    if (d->cb) {
        d->cb(new_occupied, d->cb_ctx);
    }
}

static void beam_task(void *arg)
{
    struct beam_sensor_driver *d = (struct beam_sensor_driver*)arg;

    gpio_init_input();

    d->stable = BEAM_SENSOR_DEFAULT_LOW ? false : true;
    d->occupied = raw_to_occupied(d->stable ? 1 : 0);

    d->pending_active = false;
    d->pending_since = 0;

    for (;;) {
        uint32_t bits = 0;
        xTaskNotifyWait(0, 0xFFFFFFFFu, &bits, pdMS_TO_TICKS(BEAM_SENSOR_POLL_MS));
        if (bits & NTFY_STOP) break;

        int raw = gpio_get_level(BEAM_SENSOR_GPIO);
        bool raw_occ = raw_to_occupied(raw);
        bool stable_occ = raw_to_occupied(d->stable);

        if (raw_occ == stable_occ) {
            d->pending_active = false;
            continue;
        }

        TickType_t now = xTaskGetTickCount();
        if (!d->pending_active || d->pending_value != raw_occ) {
            d->pending_active = true;
            d->pending_value = raw_occ;
            d->pending_since = now;
            continue;
        }

        TickType_t filter = d->filter_ticks;
        if ((now - d->pending_since) >= filter) {
            d->pending_active = false;
            d->stable = raw;
            maybe_publish(d, raw_occ);
        }
    }

    xSemaphoreGive(d->done);
    vTaskDelete(NULL);
}

beam_sensor_driver_t* beam_sensor_driver_create(beam_sensor_cb_t cb, void *user_ctx)
{
    struct beam_sensor_driver *d = (struct beam_sensor_driver*)calloc(1, sizeof(*d));
    if (!d) return NULL;

    d->cb = cb;
    d->cb_ctx = user_ctx;

    d->filter_ticks = pdMS_TO_TICKS(BEAM_SENSOR_FILTER_DEFAULT_MS);
    if (d->filter_ticks == 0) d->filter_ticks = 1;

    d->done = xSemaphoreCreateBinary();
    if (!d->done) {
        free(d);
        return NULL;
    }

    if (xTaskCreate(beam_task, "beam_sns", TASK_STACK, d, TASK_PRIO, &d->task) != pdPASS) {
        vSemaphoreDelete(d->done);
        free(d);
        return NULL;
    }

    return (beam_sensor_driver_t*)d;
}

void beam_sensor_driver_destroy(beam_sensor_driver_t *handle)
{
    struct beam_sensor_driver *d = (struct beam_sensor_driver*)handle;
    if (!d) return;

    xTaskNotify(d->task, NTFY_STOP, eSetBits);
    (void)xSemaphoreTake(d->done, pdMS_TO_TICKS(1000));

    vSemaphoreDelete(d->done);
    free(d);
}

bool beam_sensor_driver_get_occupied(beam_sensor_driver_t *handle)
{
    struct beam_sensor_driver *d = (struct beam_sensor_driver*)handle;
    if (!d) return false;
    return d->occupied;
}

void beam_sensor_driver_set_filter_ms(beam_sensor_driver_t *handle, uint32_t filter_ms)
{
    struct beam_sensor_driver *d = (struct beam_sensor_driver*)handle;
    if (!d) return;

    TickType_t t = pdMS_TO_TICKS(filter_ms);
    if (t == 0) t = 1;
    d->filter_ticks = t;
}
