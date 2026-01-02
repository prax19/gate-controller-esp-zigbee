#include "gate_driver.h"

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#define SIGNAL_OPEN_PIN        GPIO_NUM_20
#define SIGNAL_CLOSE_PIN       GPIO_NUM_21
#define SIGNAL_STOP_PIN        GPIO_NUM_19
#define SIGNAL_PEDESTRIAN_PIN  GPIO_NUM_18

#define SIGNAL_DURATION_MS     200
#define SIGNAL_ACTIVE_HIGH     true

#define TASK_STACK             2048
#define TASK_PRIO              5
#define NOTIFY_STOP_VALUE      0xFFFFFFFFu

typedef enum {
    SIG_OPEN = 0,
    SIG_CLOSE,
    SIG_STOP,
    SIG_PED,
    SIG_COUNT
} gate_sig_t;

struct gate_sig_cfg {
    gpio_num_t pin;
    bool invert;
};

struct gate_driver {
    TaskHandle_t task;
    SemaphoreHandle_t done;

    bool active_high;
    TickType_t pulse_ticks;

    struct gate_sig_cfg sig[SIG_COUNT];
};

static inline int base_idle(bool active_high)  { return active_high ? 0 : 1; }
static inline int base_pulse(bool active_high) { return active_high ? 1 : 0; }

static inline int sig_level(const struct gate_driver* d, gate_sig_t s, bool pulse)
{
    int idle  = base_idle(d->active_high);
    int pl    = base_pulse(d->active_high);

    if (d->sig[s].invert) {
        return pulse ? idle : pl;
    }
    return pulse ? pl : idle;
}

static void gpio_out_init(gpio_num_t gpio, int initial_level)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(gpio, initial_level);
}

static void all_idle(struct gate_driver* d)
{
    for (int s = 0; s < SIG_COUNT; ++s) {
        gpio_set_level(d->sig[s].pin, sig_level(d, (gate_sig_t)s, false));
    }
}

static void pulse_sig(struct gate_driver* d, gate_sig_t s)
{
    gpio_set_level(d->sig[s].pin, sig_level(d, s, true));
    vTaskDelay(d->pulse_ticks);
    gpio_set_level(d->sig[s].pin, sig_level(d, s, false));
}

static gate_sig_t cmd_to_sig(gate_cmd_t cmd)
{
    switch (cmd) {
        case GATE_OPEN:       return SIG_OPEN;
        case GATE_CLOSE:      return SIG_CLOSE;
        case GATE_STOP:       return SIG_STOP;
        case GATE_PEDESTRIAN: return SIG_PED;
        default:              return SIG_OPEN;
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
        pulse_sig(d, cmd_to_sig((gate_cmd_t)v));
    }

    all_idle(d);
    xSemaphoreGive(d->done);
    vTaskDelete(NULL);
}

gate_driver_t* gate_driver_create(void)
{
    struct gate_driver* d = (struct gate_driver*)calloc(1, sizeof(*d));
    if (!d) return NULL;

    d->active_high = SIGNAL_ACTIVE_HIGH;
    d->pulse_ticks = pdMS_TO_TICKS(SIGNAL_DURATION_MS);

    d->sig[SIG_OPEN]  = (struct gate_sig_cfg){ .pin = SIGNAL_OPEN_PIN,       .invert = false };
    d->sig[SIG_CLOSE] = (struct gate_sig_cfg){ .pin = SIGNAL_CLOSE_PIN,      .invert = false };
    d->sig[SIG_PED]   = (struct gate_sig_cfg){ .pin = SIGNAL_PEDESTRIAN_PIN, .invert = false };

    d->sig[SIG_STOP]  = (struct gate_sig_cfg){ .pin = SIGNAL_STOP_PIN,       .invert = true  };

    d->done = xSemaphoreCreateBinary();
    if (!d->done) {
        free(d);
        return NULL;
    }

    for (int s = 0; s < SIG_COUNT; ++s) {
        gpio_out_init(d->sig[s].pin, sig_level(d, (gate_sig_t)s, false));
    }

    all_idle(d);

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

void gate_set_standby_state(void)
{
    const int idle_norm = SIGNAL_ACTIVE_HIGH ? 0 : 1;
    const int idle_stop = SIGNAL_ACTIVE_HIGH ? 1 : 0;

    gpio_out_init(SIGNAL_STOP_PIN, idle_stop);

    gpio_out_init(SIGNAL_OPEN_PIN, idle_norm);
    gpio_out_init(SIGNAL_CLOSE_PIN, idle_norm);
    gpio_out_init(SIGNAL_PEDESTRIAN_PIN, idle_norm);
}
