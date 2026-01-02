#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "main.h"
#include "driver/gpio.h"
#include "temp_sensor_driver.h"
#include "zcl/esp_zigbee_zcl_common.h" 
#include "zcl/esp_zigbee_zcl_command.h"
#include "zcl/esp_zigbee_zcl_window_covering.h"
#include "nvs_wrapper.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_GATE_CONTROLLER";

// Zigbee parameters
#define GATE_CFG_CLUSTER_ID              0xFF00
// #define GATE_CFG_ATTR_TRANSITION_DS      0x0000 
#define GATE_CFG_ATTR_BEAM_DEBOUNCE_MS   0x0001

// static uint16_t s_transition_ds;
static uint16_t s_beam_debounce_ms;
static uint8_t s_lift_pct = 100; 

// Drivers
led_driver_t *g_led = NULL; // LED handler
gate_driver_t *gate = NULL; // Gate handler
beam_sensor_driver_t *beam = NULL; // Beam sensor handler
temp_sensor_driver_t *temp_sensor = NULL; // Temperature sensor handler

static void zb_update_temperature(void *param)
{
    int32_t scaled = (int32_t)(intptr_t)param;
    int16_t measured = (int16_t)scaled;

    esp_zb_zcl_set_attribute_val(HA_ESP_GATE_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                 &measured, false);

    ESP_LOGI(TAG, "Temperature updated to: %.2f C", measured / 100.0f);
}

static void temp_cb(float temp_c, void *user_ctx)
{
    (void)user_ctx;

    int32_t scaled = (int32_t)lroundf(temp_c * 100.0f);
    if (scaled > INT16_MAX) scaled = INT16_MAX;
    if (scaled < INT16_MIN) scaled = INT16_MIN;

    esp_zb_scheduler_user_alarm(zb_update_temperature,
                               (void*)(intptr_t)scaled,
                               0);
}

static void zb_update_occupancy(void *param)
{
    uint8_t v = (uint8_t)((uintptr_t)param ? 1 : 0);

    esp_zb_zcl_set_attribute_val(HA_ESP_GATE_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
                                 &v, false);
    ESP_LOGI(TAG, "Occupancy updated to: %s", v ? "occupied" : "unoccupied");
    
    led_driver_set(g_led, v ? LED_MODE_BLINK : LED_MODE_OFF, LED_COLOR_RED);
        
}

static void beam_cb(bool occupied, void *user_ctx)
{
    (void)user_ctx;
    esp_zb_scheduler_user_alarm(zb_update_occupancy, (void*)(uintptr_t)occupied, 0);
    
}

static inline void zb_report_attr_u16(uint16_t cluster_id, uint16_t attr_id)
{
    esp_zb_zcl_report_attr_cmd_t r = {0};
    r.zcl_basic_cmd.dst_addr_u.addr_short = COORDINATOR_ADDR;
    r.zcl_basic_cmd.src_endpoint = HA_ESP_GATE_ENDPOINT;
    r.zcl_basic_cmd.dst_endpoint = COORDINATOR_EP;
    r.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    r.direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    r.clusterID    = cluster_id;
    r.attributeID  = attr_id;
    esp_zb_zcl_report_attr_cmd_req(&r);
}

static void zb_report_beam_debounce(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "Reporting beam_debounce_ms=%u", s_beam_debounce_ms);
    zb_report_attr_u16(GATE_CFG_CLUSTER_ID, GATE_CFG_ATTR_BEAM_DEBOUNCE_MS);
}

static void wc_set_lift_pct(uint8_t pct_closed)
{
    s_lift_pct = pct_closed;
    esp_zb_zcl_set_attribute_val(HA_ESP_GATE_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
        &s_lift_pct, false);

    zb_report_attr_u16(ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID);
}


static esp_err_t zb_window_covering_movement_handler(const esp_zb_zcl_window_covering_movement_message_t *m)
{
    ESP_RETURN_ON_FALSE(m, ESP_FAIL, TAG, "Empty window covering message");
    ESP_RETURN_ON_FALSE(m->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
                        ESP_ERR_INVALID_ARG, TAG, "WC cmd error status(%d)", m->info.status);

    if (!gate) {
        ESP_LOGW(TAG, "WC cmd received but gate driver not started yet");
        return ESP_OK;
    }

    switch (m->command) {

    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN:
        ESP_LOGI(TAG, "GATE_OPEN");
        gate_driver_command(gate, GATE_OPEN);
        wc_set_lift_pct(0);     // 0 = OPEN
        led_driver_blink_once(g_led, LED_COLOR_GREEN);
        break;

    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
        ESP_LOGI(TAG, "GATE_CLOSE");
        gate_driver_command(gate, GATE_CLOSE);
        wc_set_lift_pct(100);
        led_driver_blink_once(g_led, LED_COLOR_RED);
        break;

    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP:
        gate_driver_command(gate, GATE_STOP);
        ESP_LOGI(TAG, "GATE_STOP");
        break;

    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE:
        ESP_LOGI(TAG, "GO_TO_LIFT_PERCENTAGE");
        break;

    default:
        ESP_LOGW(TAG, "unknown cmd=0x%04x", (unsigned)m->command);
        break;
    }

    return ESP_OK;
}

static void start_gate_and_beam_if_needed(void)
{
    static bool started = false;
    if (started) return;
    started = true;

    gate = gate_driver_create();
    beam = beam_sensor_driver_create(beam_cb, NULL);
    beam_sensor_driver_set_filter_ms(beam, s_beam_debounce_ms);

    temp_sensor_config_t temp_sensor_config = TEMP_SENSOR_DRIVER_DEFAULT_CONFIG();
    temp_sensor = temp_sensor_driver_create(&temp_sensor_config, temp_cb, NULL);
}

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
                    start_gate_and_beam_if_needed();
                    esp_zb_scheduler_user_alarm(zb_report_beam_debounce, NULL, 3000);
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

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_ESP_GATE_ENDPOINT) {

    }
    if (message->info.dst_endpoint == HA_ESP_GATE_ENDPOINT &&
        message->info.cluster == GATE_CFG_CLUSTER_ID &&
        message->attribute.data.value)
    {
        if (message->attribute.id == GATE_CFG_ATTR_BEAM_DEBOUNCE_MS &&
            message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
        {
            uint16_t db = *(uint16_t*)message->attribute.data.value;
            s_beam_debounce_ms = db;
            cfg_nvs_set_u16("beam_debounce", db);
            if(beam) {
                beam_sensor_driver_set_filter_ms(beam, db);
            }

            ESP_LOGI(TAG, "Beam debounce set to %dms", db);
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
    case ESP_ZB_CORE_WINDOW_COVERING_MOVEMENT_CB_ID:
        ret = zb_window_covering_movement_handler((esp_zb_zcl_window_covering_movement_message_t *)message);
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

    // Cluster list
    esp_zb_cluster_list_t *clist = esp_zb_zcl_cluster_list_create();

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

    // Window Covering (server)
    esp_zb_attribute_list_t *wc_srv = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING);

    esp_zb_window_covering_cluster_add_attr(
        wc_srv,
        ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
        &s_lift_pct
);

    // Occupancy sensor (server)
    esp_zb_occupancy_sensing_cluster_cfg_t occupancy_cfg = {
        .occupancy=0x00,
        .sensor_type=0x03,
        .sensor_type_bitmap=0x04
    };
    esp_zb_attribute_list_t *occupancy_sensor_srv = esp_zb_occupancy_sensing_cluster_create(&occupancy_cfg);

    // Device temperature
    // esp_zb_device_temp_config_cluster_cfg_t temp_cfg = {
    //     .current_temperature = ESP_ZB_ZCL_DEVICE_TEMP_CONFIG_CURRENT_TEMP_DEFAULT_VALUE
    // };
    // esp_zb_attribute_list_t *device_temp_srv = esp_zb_device_temp_config_cluster_create(&temp_cfg);

    // Temperature Measurement (server) - cluster 0x0402

    static int16_t s_temp_measured = ESP_ZB_ZCL_TEMP_MEASUREMENT_MEASURED_VALUE_DEFAULT;
    static int16_t s_temp_min      = ESP_ZB_ZCL_TEMP_MEASUREMENT_MIN_MEASURED_VALUE_DEFAULT;
    static int16_t s_temp_max      = ESP_ZB_ZCL_TEMP_MEASUREMENT_MAX_MEASURED_VALUE_DEFAULT;

    bool is_temp_sensor_present = temp_sensor_driver_read_once(NULL, &s_temp_measured);

    if(is_temp_sensor_present) {
        esp_zb_attribute_list_t *temp_meas_srv =
            esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);

        esp_zb_temperature_meas_cluster_add_attr(temp_meas_srv,
            ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &s_temp_measured);
        esp_zb_temperature_meas_cluster_add_attr(temp_meas_srv,
            ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &s_temp_min);
        esp_zb_temperature_meas_cluster_add_attr(temp_meas_srv,
            ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &s_temp_max);

        esp_zb_cluster_list_add_temperature_meas_cluster(clist, temp_meas_srv, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    }

    // Experimental features

    esp_zb_attribute_list_t *gate_cfg_srv =
        esp_zb_zcl_attr_list_create(GATE_CFG_CLUSTER_ID);

    // esp_zb_custom_cluster_add_custom_attr(
    //     gate_cfg_srv,
    //     GATE_CFG_ATTR_TRANSITION_DS,
    //     ESP_ZB_ZCL_ATTR_TYPE_U16,
    //     ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
    //     &s_gate_transition_ds
    // );

    esp_zb_custom_cluster_add_custom_attr(
        gate_cfg_srv,
        GATE_CFG_ATTR_BEAM_DEBOUNCE_MS,
        ESP_ZB_ZCL_ATTR_TYPE_U16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &s_beam_debounce_ms
    );
    
    /* Joining cluster lists */
    esp_zb_cluster_list_add_basic_cluster( clist, basic,        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(clist, identify_srv,ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_groups_cluster( clist, groups_srv,  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_scenes_cluster( clist, scenes_srv,  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_window_covering_cluster(clist, wc_srv, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_occupancy_sensing_cluster( clist, occupancy_sensor_srv,   ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // esp_zb_cluster_list_add_device_temp_config_cluster( clist, device_temp_srv,   ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_custom_cluster(clist, gate_cfg_srv, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Device and endpoint registering */
    esp_zb_ep_list_t *eplist = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t epcfg = {
        .endpoint          = HA_ESP_GATE_ENDPOINT,
        .app_profile_id    = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id     = ESP_ZB_HA_WINDOW_COVERING_DEVICE_ID,
        .app_device_version= 1,
    };

    ESP_ERROR_CHECK( esp_zb_ep_list_add_ep(eplist, clist, epcfg) );

    esp_zb_device_register(eplist);

    ESP_ERROR_CHECK(esp_zb_start(true));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    gate_set_standby_state();

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    g_led = led_driver_create();

    s_beam_debounce_ms = cfg_nvs_get_u16_def("beam_debounce", 200);

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
