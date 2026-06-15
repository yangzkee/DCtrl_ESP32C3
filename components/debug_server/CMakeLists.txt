idf_component_register(
    SRCS "debug_server_wifi.c" "debug_server_ble.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event esp_netif esp_http_server debug_protocol param_store vehicle_state telemetry chassis bt nvs_flash
)
