#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "temp_sensor_driver.h"

static const char *TAG = "temp_drv";

static onewire_bus_handle_t s_bus = NULL;
static ds18b20_device_handle_t s_sensor = NULL;
static bool s_inited = false;

void temp_driver_init()
{
    if (s_inited) {
        return;
    }

    onewire_bus_config_t bus_config = {
        .bus_gpio_num = SENSOR_PIN,
        .flags = {
            .en_pull_up = true,
        },
    };

    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10,
    };

    esp_err_t err = onewire_new_bus_rmt(&bus_config, &rmt_config, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "onewire_new_bus_rmt failed: %s", esp_err_to_name(err));
        return;
    }

    onewire_device_iter_handle_t iter = NULL;
    err = onewire_new_device_iter(s_bus, &iter);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "onewire_new_device_iter failed: %s", esp_err_to_name(err));
        return;
    }

    onewire_device_t dev;
    while (onewire_device_iter_get_next(iter, &dev) == ESP_OK) {
        ds18b20_config_t cfg = {};
        if (ds18b20_new_device_from_enumeration(&dev, &cfg, &s_sensor) == ESP_OK) {
            onewire_device_address_t addr = 0;
            ds18b20_get_device_address(s_sensor, &addr);
            ESP_LOGI(TAG, "DS18B20 found, addr: %016llX", (unsigned long long)addr);
            break;
        }
    }

    onewire_del_device_iter(iter);

    if (!s_sensor) {
        ESP_LOGE(TAG, "No DS18B20 found on GPIO%d", (int)SENSOR_PIN);
        return;
    }

    s_inited = true;
}

float read_temperature()
{
    if (!s_inited || !s_bus || !s_sensor) {
        return NAN;
    }

    esp_err_t err = ds18b20_trigger_temperature_conversion_for_all(s_bus);
    if (err != ESP_OK) {
        return NAN;
    }

    // DS18B20 (domyślnie 12-bit) potrzebuje do ~750ms na konwersję
    vTaskDelay(pdMS_TO_TICKS(800));

    float t = NAN;
    err = ds18b20_get_temperature(s_sensor, &t);
    if (err != ESP_OK) {
        return NAN;
    }

    return t;
}
