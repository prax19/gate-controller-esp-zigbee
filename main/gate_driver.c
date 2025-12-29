#include "gate_driver.h"

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define SIGNAL_OPEN_PIN        GPIO_NUM_20
#define SIGNAL_CLOSE_PIN       GPIO_NUM_21
#define SIGNAL_PEDESTRIAN_PIN  GPIO_NUM_18
#define SIGNAL_SEQUENCE_PIN    GPIO_NUM_19

#define SIGNAL_DURATION_MS     200

#define SIGNAL_ACTIVE_HIGH     true

#define TASK_STACK             2048
#define TASK_PRIO              5
#define NOTIFY_STOP_VALUE      0xFFFFFFFFu

struct gate_driver {
    TaskHandle_t task;
    SemaphoreHandle_t done;

    gpio_num_t pin_open;
    gpio_num_t pin_close;
    gpio_num_t pin_ped;
    gpio_num_t pin_seq;

    bool active_high;
};

static void gate_pin_init(gpio_num_t gpio, bool active_high)
{
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

static inline int level_idle(bool active_high)  { return active_high ? 0 : 1; }
static inline int level_pulse(bool active_high) { return active_high ? 1 : 0; }

static void all_idle(struct gate_driver* d)
{
    int idle = level_idle(d->active_high);
    gpio_set_level(d->pin_ped,  idle);
    gpio_set_level(d->pin_seq,  idle);
    gpio_set_level(d->pin_open, idle);
    gpio_set_level(d->pin_close,idle);
}

static void short_signal(struct gate_driver* d, gpio_num_t gpio)
{
    gpio_set_level(gpio, level_pulse(d->active_high));
    vTaskDelay(pdMS_TO_TICKS(SIGNAL_DURATION_MS));
    gpio_set_level(gpio, level_idle(d->active_high));
}

static gpio_num_t cmd_to_pin(const struct gate_driver* d, gate_cmd_t cmd)
{
    (void)d;
    switch (cmd) {
        case GATE_PEDESTRIAN: return SIGNAL_PEDESTRIAN_PIN;
        case GATE_SEQ_OPEN:   return SIGNAL_SEQUENCE_PIN;
        case GATE_OPEN:       return SIGNAL_OPEN_PIN;
        case GATE_CLOSE:      return SIGNAL_CLOSE_PIN;
        default:              return SIGNAL_OPEN_PIN;
    }
}

static void gate_task(void* arg)
{
    struct gate_driver* d = (struct gate_driver*)arg;

    all_idle(d);

    for (;;) {
        uint32_t v = 0;

        xTaskNotifyWait(0, 0xFFFFFFFFu, &v, portMAX_DELAY);

        if (v == NOTIFY_STOP_VALUE) break;

        all_idle(d);
        short_signal(d, cmd_to_pin(d, (gate_cmd_t)v));
    }

    all_idle(d);
    xSemaphoreGive(d->done);
    vTaskDelete(NULL);
}

gate_driver_t* gate_driver_create(void)
{
    struct gate_driver* d = (struct gate_driver*)calloc(1, sizeof(*d));
    if (!d) return NULL;

    d->pin_ped  = SIGNAL_PEDESTRIAN_PIN;
    d->pin_seq  = SIGNAL_SEQUENCE_PIN;
    d->pin_open = SIGNAL_OPEN_PIN;
    d->pin_close= SIGNAL_CLOSE_PIN;

    d->active_high = SIGNAL_ACTIVE_HIGH;

    d->done = xSemaphoreCreateBinary();
    if (!d->done) {
        free(d);
        return NULL;
    }

    gate_pin_init(d->pin_ped,  d->active_high);
    gate_pin_init(d->pin_seq,  d->active_high);
    gate_pin_init(d->pin_open, d->active_high);
    gate_pin_init(d->pin_close,d->active_high);

    if (xTaskCreate(gate_task, "gate_drv", TASK_STACK, d, TASK_PRIO, &d->task) != pdPASS) {
        vSemaphoreDelete(d->done);
        free(d);
        return NULL;
    }

    return (gate_driver_t*)d;
}

void gate_driver_destroy(gate_driver_t* handle)
{
    struct gate_driver* d = (struct gate_driver*)handle;
    if (!d) return;

    xTaskNotify(d->task, NOTIFY_STOP_VALUE, eSetValueWithOverwrite);
    (void)xSemaphoreTake(d->done, pdMS_TO_TICKS(1000));

    vSemaphoreDelete(d->done);
    free(d);
}

void gate_driver_command(gate_driver_t* handle, gate_cmd_t cmd)
{
    struct gate_driver* d = (struct gate_driver*)handle;
    if (!d) return;

    xTaskNotify(d->task, (uint32_t)cmd, eSetValueWithOverwrite);
}
