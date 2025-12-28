#include "esp_zigbee_core.h"
#include "led_driver.h"
#include "gate_driver.h"
#include "beam_sensor_driver.h"
#include "temp_sensor_driver.h"

#define XIAO_RF_EN_GPIO     GPIO_NUM_3
#define XIAO_RF_ANT_GPIO    GPIO_NUM_14

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false   /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */
#define ED_NETWORK_STEERING_RETRY_TIME  10000
#define HA_ESP_GATE_ENDPOINT            10      /* esp gate controller endpoint, used to process gate controlling commands */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Zigbee primary channel mask use in the example */

#define ESP_MANUFACTURER_NAME "\x06""prax19"      /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x0B""Retrofit GC" /* Customized model identifier */

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                        \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,      \
    }
