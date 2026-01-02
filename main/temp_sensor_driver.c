#include "temp_sensor_driver.h"

#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "onewire_bus.h"
#include "ds18b20.h"

#define TASK_STACK          3072
#define TASK_PRIO           5

#define CONV_DELAY_MS       800

#define NTFY_STOP           (1u << 31)
#define NTFY_FORCE          (1u << 30)

static const char *TAG = "temp_drv";

struct temp_sensor_driver {
    TaskHandle_t task;
    SemaphoreHandle_t done;
    SemaphoreHandle_t lock;

    temp_sensor_config_t cfg;

    temp_sensor_cb_t cb;
    void *cb_ctx;

    onewire_bus_handle_t bus;
    ds18b20_device_handle_t sensor;
    bool ready;

    float last_temp;
    bool last_valid;
    int64_t last_ts_us;
};

static void cleanup_bus_and_sensor(struct temp_sensor_driver *d)
{
    if (d->sensor) {
        ds18b20_del_device(d->sensor);
        d->sensor = NULL;
    }
    if (d->bus) {
        onewire_bus_del(d->bus);
        d->bus = NULL;
    }
    d->ready = false;
}

static esp_err_t init_bus_and_find_sensor(struct temp_sensor_driver *d)
{
    cleanup_bus_and_sensor(d);

    onewire_bus_config_t bus_config = {
        .bus_gpio_num = d->cfg.gpio_num,
        .flags = {
            .en_pull_up = d->cfg.enable_pullup,
        },
    };

    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10,
    };

    esp_err_t err = onewire_new_bus_rmt(&bus_config, &rmt_config, &d->bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "onewire_new_bus_rmt failed: %s", esp_err_to_name(err));
        return err;
    }

    onewire_device_iter_handle_t iter = NULL;
    err = onewire_new_device_iter(d->bus, &iter);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "onewire_new_device_iter failed: %s", esp_err_to_name(err));
        cleanup_bus_and_sensor(d);
        return err;
    }

    onewire_device_t dev;
    while (onewire_device_iter_get_next(iter, &dev) == ESP_OK) {
        ds18b20_config_t cfg = {};
        if (ds18b20_new_device_from_enumeration(&dev, &cfg, &d->sensor) == ESP_OK) {
            onewire_device_address_t addr = 0;
            ds18b20_get_device_address(d->sensor, &addr);
            ESP_LOGI(TAG, "DS18B20 found on GPIO%d, addr: %016llX",
                     (int)d->cfg.gpio_num,
                     (unsigned long long)addr);
            break;
        }
    }

    onewire_del_device_iter(iter);

    if (!d->sensor) {
        ESP_LOGW(TAG, "No DS18B20 found on GPIO%d", (int)d->cfg.gpio_num);
        cleanup_bus_and_sensor(d);
        return ESP_ERR_NOT_FOUND;
    }

    d->ready = true;
    return ESP_OK;
}

static bool do_measure(struct temp_sensor_driver *d, float *out_t)
{
    if (!d->ready || !d->bus || !d->sensor) {
        return false;
    }

    esp_err_t err = ds18b20_trigger_temperature_conversion_for_all(d->bus);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "trigger conversion failed: %s", esp_err_to_name(err));
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(CONV_DELAY_MS));

    float t = NAN;
    err = ds18b20_get_temperature(d->sensor, &t);
    if (err != ESP_OK || isnan(t)) {
        ESP_LOGW(TAG, "read temperature failed: %s", esp_err_to_name(err));
        return false;
    }

    *out_t = t;
    return true;
}

static void temp_task(void *arg)
{
    struct temp_sensor_driver *d = (struct temp_sensor_driver *)arg;

    for (;;) {
        uint32_t bits = 0;
        (void)xTaskNotifyWait(0, 0xFFFFFFFFu, &bits, 0);
        if (bits & NTFY_STOP) goto out;

        esp_err_t err = init_bus_and_find_sensor(d);
        if (err == ESP_OK) break;

        xTaskNotifyWait(0, 0xFFFFFFFFu, &bits, pdMS_TO_TICKS(1000));
        if (bits & NTFY_STOP) goto out;
    }

    for (;;) {
        float t = NAN;
        bool ok = do_measure(d, &t);

        if (xSemaphoreTake(d->lock, pdMS_TO_TICKS(200)) == pdTRUE) {
            d->last_valid = ok;
            if (ok) {
                d->last_temp = t;
                d->last_ts_us = esp_timer_get_time();
            }
            xSemaphoreGive(d->lock);
        }

        if (ok && d->cb) {
            d->cb(t, d->cb_ctx);
        }

        uint32_t bits = 0;
        xTaskNotifyWait(0, 0xFFFFFFFFu, &bits, pdMS_TO_TICKS(d->cfg.period_ms));
        if (bits & NTFY_STOP) break;
        if (bits & NTFY_FORCE) continue;
    }

out:
    cleanup_bus_and_sensor(d);
    xSemaphoreGive(d->done);
    vTaskDelete(NULL);
}

temp_sensor_driver_t *temp_sensor_driver_create(const temp_sensor_config_t *cfg,
                                                temp_sensor_cb_t cb,
                                                void *user_ctx)
{
    struct temp_sensor_driver *d = (struct temp_sensor_driver *)calloc(1, sizeof(*d));
    if (!d) return NULL;

    d->cfg = cfg ? *cfg : TEMP_SENSOR_DRIVER_DEFAULT_CONFIG();
    d->cb = cb;
    d->cb_ctx = user_ctx;

    d->last_temp = NAN;
    d->last_valid = false;
    d->last_ts_us = 0;

    d->done = xSemaphoreCreateBinary();
    if (!d->done) {
        free(d);
        return NULL;
    }

    d->lock = xSemaphoreCreateMutex();
    if (!d->lock) {
        vSemaphoreDelete(d->done);
        free(d);
        return NULL;
    }

    if (xTaskCreate(temp_task, "temp_sns", TASK_STACK, d, TASK_PRIO, &d->task) != pdPASS) {
        vSemaphoreDelete(d->lock);
        vSemaphoreDelete(d->done);
        free(d);
        return NULL;
    }

    return (temp_sensor_driver_t *)d;
}

void temp_sensor_driver_destroy(temp_sensor_driver_t *handle)
{
    struct temp_sensor_driver *d = (struct temp_sensor_driver *)handle;
    if (!d) return;

    xTaskNotify(d->task, NTFY_STOP, eSetBits);
    (void)xSemaphoreTake(d->done, pdMS_TO_TICKS(1500));

    vSemaphoreDelete(d->lock);
    vSemaphoreDelete(d->done);
    free(d);
}

bool temp_sensor_driver_get_last(temp_sensor_driver_t *handle,
                                 float *out_temp_c,
                                 int64_t *out_timestamp_us)
{
    struct temp_sensor_driver *d = (struct temp_sensor_driver *)handle;
    if (!d || !out_temp_c) return false;

    bool ok = false;

    if (xSemaphoreTake(d->lock, pdMS_TO_TICKS(50)) == pdTRUE) {
        ok = d->last_valid;
        if (ok) {
            *out_temp_c = d->last_temp;
            if (out_timestamp_us) {
                *out_timestamp_us = d->last_ts_us;
            }
        }
        xSemaphoreGive(d->lock);
    }

    return ok;
}

void temp_sensor_driver_trigger(temp_sensor_driver_t *handle)
{
    struct temp_sensor_driver *d = (struct temp_sensor_driver *)handle;
    if (!d) return;
    xTaskNotify(d->task, NTFY_FORCE, eSetBits);
}

bool temp_sensor_driver_is_ready(temp_sensor_driver_t *handle)
{
    struct temp_sensor_driver *d = (struct temp_sensor_driver *)handle;
    if (!d) return false;
    return d->ready;
}

bool temp_sensor_driver_read_once(const temp_sensor_config_t *cfg, int16_t *out_temp_c_x100)
{
    if (!out_temp_c_x100) return false;

    float temp_c = 0.0f;

    struct temp_sensor_driver d = {0};
    d.cfg = cfg ? *cfg : TEMP_SENSOR_DRIVER_DEFAULT_CONFIG();

    if (init_bus_and_find_sensor(&d) != ESP_OK) {
        cleanup_bus_and_sensor(&d);
        return false;
    }

    bool ok = do_measure(&d, &temp_c);
    cleanup_bus_and_sensor(&d);

    if (!ok) return false;

    if (!isfinite(temp_c)) return false;

    int32_t scaled = (int32_t)lroundf(temp_c * 100.0f);

    if (scaled > 32767)  scaled = 32767;
    if (scaled < -27315) scaled = -27315;

    *out_temp_c_x100 = (int16_t)scaled;
    return true;
}

