#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "main.h"
#include "driver/gpio.h"
#include "temp_sensor_driver.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_GATE_CONTROLLER";
/********************* Define functions **************************/


led_driver_t *g_led = NULL; // LED handler

void schedule_network_steering() {
    ESP_LOGI(TAG, "Start secondary network steering");
    // esp_zb_secur_network_min_join_lqi_set(0);
    esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning,
            ESP_ZB_BDB_MODE_NETWORK_STEERING, ED_NETWORK_STEERING_RETRY_TIME);
    led_driver_set(g_led, LED_MODE_BLINK_FAST, LED_COLOR_BLUE);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        led_driver_set(g_led, LED_MODE_BLINK_FAST, LED_COLOR_BLUE);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                led_driver_set(g_led, LED_MODE_BLINK_FAST, LED_COLOR_BLUE);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
            led_driver_set(g_led, LED_MODE_ON, LED_COLOR_RED);
            schedule_network_steering();
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                    extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                    extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                    esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
                    led_driver_set(g_led, LED_MODE_ON, LED_COLOR_GREEN);
                    led_driver_sleep(g_led, 5000);
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            led_driver_set(g_led, LED_MODE_ON, LED_COLOR_RED);
            schedule_network_steering();
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        led_driver_set(g_led, LED_MODE_BLINK, LED_COLOR_BLUE);
        break;
    case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        led_driver_set(g_led, LED_MODE_BLINK, LED_COLOR_RED);
        schedule_network_steering();
        break;
    default:
        if (err_status != ESP_OK) {
            led_driver_set(g_led, LED_MODE_BLINK_FAST, LED_COLOR_RED);
            ESP_LOGW(TAG, "Signal %s(0x%x) fail %s – retry steering…",
                     esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
            schedule_network_steering();
        } else {
            ESP_LOGI(TAG, "Signal %s(0x%x) OK", esp_zb_zdo_signal_to_string(sig_type), sig_type);
        }
        break;
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = false;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_ESP_GATE_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
                call_gate_cmd(light_state ? GATE_OPEN: GATE_CLOSE);
                led_driver_blink_once(g_led, LED_COLOR_GREEN);
            }
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    // esp_zb_set_tx_power(8);

    /* Basic config variables */
    uint8_t zcl_ver = 3;
    uint8_t power_src = 0x01;
    uint16_t identify_time = 0;
    uint8_t name_support = 0;

    esp_zb_attribute_list_t *basic = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_ver);
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_src);
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,   &ESP_MODEL_IDENTIFIER[0]);
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,  &ESP_MANUFACTURER_NAME[0]);

    // IDENTIFY (server)
    esp_zb_attribute_list_t *identify_srv = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(identify_srv, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_time);

    // GROUPS (server)
    esp_zb_attribute_list_t *groups_srv = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_GROUPS);
    esp_zb_groups_cluster_add_attr(groups_srv, ESP_ZB_ZCL_ATTR_GROUPS_NAME_SUPPORT_ID, &name_support);

    // SCENES (server)
    esp_zb_attribute_list_t *scenes_srv = esp_zb_scenes_cluster_create(NULL);

    // ON/OFF (server)
    esp_zb_on_off_cluster_cfg_t onoff_cfg = {
        .on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE
    };
    esp_zb_attribute_list_t *onoff_srv = esp_zb_on_off_cluster_create(&onoff_cfg);

    // Occupancy sensor (server)
    esp_zb_occupancy_sensing_cluster_cfg_t occupancy_cfg = {
        .occupancy=0x00,
        .sensor_type=0x03,
        .sensor_type_bitmap=0x04
    };
    esp_zb_attribute_list_t *occupancy_sensor_srv = esp_zb_occupancy_sensing_cluster_create(&occupancy_cfg);

    /* Joining cluster lists */
    esp_zb_cluster_list_t *clist = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster( clist, basic,        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(clist, identify_srv,ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_groups_cluster( clist, groups_srv,  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_scenes_cluster( clist, scenes_srv,  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster( clist, onoff_srv,   ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_occupancy_sensing_cluster( clist, occupancy_sensor_srv,   ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Device and endpoint registering */
    esp_zb_ep_list_t *eplist = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t epcfg = {
        .endpoint          = HA_ESP_GATE_ENDPOINT,
        .app_profile_id    = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id     = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID,
        .app_device_version= 1,
    };

    ESP_ERROR_CHECK( esp_zb_ep_list_add_ep(eplist, clist, epcfg) );

    esp_zb_device_register(eplist);

    ESP_ERROR_CHECK(esp_zb_start(true));
    esp_zb_stack_main_loop();
}

static void zb_set_occupancy(bool beam_broken) {
    uint8_t v = beam_broken ? 1 : 0;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ESP_GATE_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
        &v, false);
    esp_zb_lock_release();
}

static void beam_task(void *arg) {
    bool last = beam_read();
    zb_set_occupancy(last);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        bool cur = beam_read();
        if (cur != last) {
            cur = beam_read();
            if (cur != last) {
                last = cur;
                zb_set_occupancy(cur);
            }
        }
    }
}

static void temp_task(void *arg) {
    for (;;) {
        float t = read_temperature();
        if(!isnan(t)) {
            ESP_LOGI(TAG, "Temperature: %.2f C", t);
        } else {
            ESP_LOGW(TAG, "Failed to read temperature");
        }
        vTaskDelay(pdMS_TO_TICKS(8000));
    }
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    g_led = led_driver_create();
    led_driver_set(g_led, LED_MODE_OFF, LED_COLOR_GREEN);

    gate_driver_init(GATE_CLOSE);
    beam_sensor_driver_init();
    temp_driver_init();
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    xTaskCreate(beam_task, "beam", 3072, NULL, 6, NULL);
    xTaskCreate(temp_task, "temp", 2048, NULL, 2, NULL);
}
