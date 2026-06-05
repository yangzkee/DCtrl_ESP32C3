idf_component_register(
    SRCS "vehicle_state.c"
    INCLUDE_DIRS "include"
    REQUIRES chassis esp_timer freertos
)
