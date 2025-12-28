idf_component_register(
    SRCS
        "esp_zb_light.c"
        "led_driver.c"
        "gate_driver.c"
        "beam_sensor_driver.c"
        "temp_sensor_driver.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES nvs_flash esp_driver_uart ieee802154
)