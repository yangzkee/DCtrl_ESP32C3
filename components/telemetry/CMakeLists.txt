idf_component_register(
    SRCS "telemetry.c"
    INCLUDE_DIRS "include"
    REQUIRES chassis line_sensor param_store vehicle_state esp_timer freertos
)
