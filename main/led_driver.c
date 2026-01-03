#include "led_driver.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

// GPIO pins
#define LED_R_GPIO          GPIO_NUM_0
#define LED_G_GPIO          GPIO_NUM_1
#define LED_B_GPIO          GPIO_NUM_2

// ACTIVE-LOW: LOW = ON, HIGH = OFF
#define LED_ACTIVE_LEVEL    0

#define TASK_STACK          2048
#define TASK_PRIO           5

#define NTFY_DIRTY_STATE    (1u << 0)
#define NTFY_DIRTY_SLEEP    (1u << 1)
#define NTFY_PULSE          (1u << 2)
#define NTFY_STOP           (1u << 31)

struct led_driver {
    TaskHandle_t task;

    gpio_num_t pin_r, pin_g, pin_b;

    led_mode_t  desired_mode;
    led_color_t desired_color;

    bool        desired_sleep_armed;
    TickType_t  desired_sleep_until;

    bool        desired_pulse_armed;
    TickType_t  desired_pulse_until;
    led_color_t desired_pulse_color;

    led_mode_t  mode;
    led_color_t color;
    bool        phase_on;

    bool        sleep_armed;
    TickType_t  sleep_until;

    bool        pulse_armed;
    TickType_t  pulse_until;
    led_color_t pulse_color;

    portMUX_TYPE mux;
    SemaphoreHandle_t done;
};

static inline bool tick_reached(TickType_t now, TickType_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static inline void gpio_write(gpio_num_t pin, bool on)
{
    int level = on ? LED_ACTIVE_LEVEL : !LED_ACTIVE_LEVEL;
    gpio_set_level(pin, level);
}

static void all_off(struct led_driver* d)
{
    gpio_write(d->pin_r, false);
    gpio_write(d->pin_g, false);
    gpio_write(d->pin_b, false);
}

static void color_on(struct led_driver* d, led_color_t c)
{
    all_off(d);
    switch (c) {
        case LED_COLOR_RED:   gpio_write(d->pin_r, true); break;
        case LED_COLOR_GREEN: gpio_write(d->pin_g, true); break;
        case LED_COLOR_BLUE:  gpio_write(d->pin_b, true); break;
        case LED_COLOR_YELLOW:
            gpio_write(d->pin_r, true);
            gpio_write(d->pin_g, true);
        break;
        default: break;
    }
}

static TickType_t blink_period(led_mode_t mode)
{
    switch (mode) {
        case LED_MODE_BLINK:      return pdMS_TO_TICKS(1000);
        case LED_MODE_BLINK_FAST: return pdMS_TO_TICKS(250); // FAST = 250ms
        default:                  return portMAX_DELAY;
    }
}

static void apply_state(struct led_driver* d)
{
    if (d->pulse_armed) {
        TickType_t now = xTaskGetTickCount();
        if (!tick_reached(now, d->pulse_until)) {
            color_on(d, d->pulse_color);
            return;
        }
        d->pulse_armed = false;
    }

    switch (d->mode) {
        case LED_MODE_OFF:
            all_off(d);
            break;

        case LED_MODE_ON:
            color_on(d, d->color);
            break;

        case LED_MODE_BLINK:
        case LED_MODE_BLINK_FAST:
            if (d->phase_on) color_on(d, d->color);
            else all_off(d);
            break;

        default:
            all_off(d);
            break;
    }
}

static void handle_sleep_if_due(struct led_driver* d)
{
    if (!d->sleep_armed) return;

    TickType_t now = xTaskGetTickCount();
    if (tick_reached(now, d->sleep_until)) {
        d->sleep_armed = false;

        d->mode = LED_MODE_OFF;
        d->phase_on = false;

        d->pulse_armed = false;

        apply_state(d);
    }
}

static TickType_t compute_wait_timeout(struct led_driver* d)
{
    TickType_t now = xTaskGetTickCount();

    TickType_t t_blink = blink_period(d->mode);

    TickType_t t_sleep = portMAX_DELAY;
    if (d->sleep_armed) {
        if (tick_reached(now, d->sleep_until)) return 0;
        t_sleep = d->sleep_until - now;
    }

    TickType_t t_pulse = portMAX_DELAY;
    if (d->pulse_armed) {
        if (tick_reached(now, d->pulse_until)) return 0;
        t_pulse = d->pulse_until - now;
    }

    TickType_t t = t_blink;
    if (t_sleep < t) t = t_sleep;
    if (t_pulse < t) t = t_pulse;
    return t;
}

static void led_task(void* arg)
{
    struct led_driver* d = (struct led_driver*)arg;

    // start
    d->mode = LED_MODE_OFF;
    d->color = LED_COLOR_RED;
    d->phase_on = false;

    d->sleep_armed = false;
    d->pulse_armed = false;

    apply_state(d);

    for (;;) {
        handle_sleep_if_due(d);

        uint32_t bits = 0;
        TickType_t to = compute_wait_timeout(d);

        BaseType_t got = xTaskNotifyWait(
            0,
            0xFFFFFFFFu,
            &bits,
            to
        );

        if (got == pdTRUE) {
            if (bits & NTFY_STOP) break;

            if (bits & (NTFY_DIRTY_STATE | NTFY_DIRTY_SLEEP | NTFY_PULSE)) {
                bool mode_changed = false;

                portENTER_CRITICAL(&d->mux);

                if (bits & NTFY_DIRTY_STATE) {
                    mode_changed = (d->mode != d->desired_mode);
                    d->mode  = d->desired_mode;
                    d->color = d->desired_color;
                }

                if (bits & NTFY_DIRTY_SLEEP) {
                    d->sleep_armed = d->desired_sleep_armed;
                    d->sleep_until = d->desired_sleep_until;
                }

                if (bits & NTFY_PULSE) {
                    d->pulse_armed  = d->desired_pulse_armed;
                    d->pulse_until  = d->desired_pulse_until;
                    d->pulse_color  = d->desired_pulse_color;

                    d->desired_pulse_armed = false;
                }

                portEXIT_CRITICAL(&d->mux);

                if (mode_changed) {
                    if (d->mode == LED_MODE_BLINK || d->mode == LED_MODE_BLINK_FAST) {
                        d->phase_on = true;
                    } else {
                        d->phase_on = (d->mode == LED_MODE_ON);
                    }
                }

                apply_state(d);
            }
        } else {
            handle_sleep_if_due(d);

            if (d->mode == LED_MODE_BLINK || d->mode == LED_MODE_BLINK_FAST) {
                d->phase_on = !d->phase_on;
            }

            apply_state(d);
        }
    }

    all_off(d);
    xSemaphoreGive(d->done);
    vTaskDelete(NULL);
}

led_driver_t* led_driver_create(void)
{
    struct led_driver* d = (struct led_driver*)calloc(1, sizeof(*d));
    if (!d) return NULL;

    d->pin_r = LED_R_GPIO;
    d->pin_g = LED_G_GPIO;
    d->pin_b = LED_B_GPIO;

    d->desired_mode  = LED_MODE_OFF;
    d->desired_color = LED_COLOR_RED;

    d->desired_sleep_armed = false;
    d->desired_sleep_until = 0;

    d->desired_pulse_armed = false;
    d->desired_pulse_until = 0;
    d->desired_pulse_color = LED_COLOR_RED;

    d->mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

    d->done = xSemaphoreCreateBinary();
    if (!d->done) {
        free(d);
        return NULL;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << d->pin_r) | (1ULL << d->pin_g) | (1ULL << d->pin_b),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    all_off(d);

    if (xTaskCreate(led_task, "led_drv", TASK_STACK, d, TASK_PRIO, &d->task) != pdPASS) {
        vSemaphoreDelete(d->done);
        free(d);
        return NULL;
    }

    return (led_driver_t*)d;
}

void led_driver_destroy(led_driver_t* handle)
{
    struct led_driver* d = (struct led_driver*)handle;
    if (!d) return;

    xTaskNotify(d->task, NTFY_STOP, eSetBits);
    (void)xSemaphoreTake(d->done, pdMS_TO_TICKS(1000));

    vSemaphoreDelete(d->done);
    free(d);
}

void led_driver_set(led_driver_t* handle, led_mode_t mode, led_color_t color)
{
    struct led_driver* d = (struct led_driver*)handle;
    if (!d) return;

    portENTER_CRITICAL(&d->mux);
    d->desired_mode  = mode;
    d->desired_color = color;
    portEXIT_CRITICAL(&d->mux);

    xTaskNotify(d->task, NTFY_DIRTY_STATE, eSetBits);
}

void led_driver_sleep(led_driver_t* handle, uint32_t ms)
{
    struct led_driver* d = (struct led_driver*)handle;
    if (!d) return;

    portENTER_CRITICAL(&d->mux);

    if (ms == 0) {
        d->desired_sleep_armed = false;
        d->desired_sleep_until = 0;
    } else {
        TickType_t now = xTaskGetTickCount();
        d->desired_sleep_armed = true;
        d->desired_sleep_until = now + pdMS_TO_TICKS(ms);
    }

    portEXIT_CRITICAL(&d->mux);

    xTaskNotify(d->task, NTFY_DIRTY_SLEEP, eSetBits);
}

void led_driver_blink_once(led_driver_t* handle, led_color_t color)
{
    struct led_driver* d = (struct led_driver*)handle;
    if (!d) return;

    TickType_t now = xTaskGetTickCount();
    TickType_t until = now + pdMS_TO_TICKS(250);

    portENTER_CRITICAL(&d->mux);
    d->desired_pulse_armed = true;
    d->desired_pulse_until = until;
    d->desired_pulse_color = color;
    portEXIT_CRITICAL(&d->mux);

    xTaskNotify(d->task, NTFY_PULSE, eSetBits);
}
