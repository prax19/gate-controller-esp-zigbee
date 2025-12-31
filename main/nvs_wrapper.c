#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

#define NVS_NS "gatecfg"

uint8_t cfg_nvs_get_u8_def(const char *key, uint8_t def)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return def;

    uint8_t v = def;
    esp_err_t err = nvs_get_u8(h, key, &v);
    nvs_close(h);

    return (err == ESP_OK) ? v : def;
}

void cfg_nvs_set_u8(const char *key, uint8_t v)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;

    (void)nvs_set_u8(h, key, v);
    (void)nvs_commit(h);
    nvs_close(h);
}

uint16_t cfg_nvs_get_u16_def(const char *key, uint16_t def)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW("CFG", "nvs_open ro failed: %s", esp_err_to_name(err));
        return def;
    }

    uint16_t v = def;
    err = nvs_get_u16(h, key, &v);
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGW("CFG", "nvs_get_u16(%s) failed: %s", key, esp_err_to_name(err));
        return def;
    }
    return v;
}

void cfg_nvs_set_u16(const char *key, uint16_t v)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW("CFG", "nvs_open rw failed: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_u16(h, key, v);
    if (err != ESP_OK) ESP_LOGW("CFG", "nvs_set_u16(%s) failed: %s", key, esp_err_to_name(err));

    err = nvs_commit(h);
    if (err != ESP_OK) ESP_LOGW("CFG", "nvs_commit failed: %s", esp_err_to_name(err));

    nvs_close(h);
}
