idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES board chassis line_sensor control param_store debug_server nvs_flash vehicle_state
)
