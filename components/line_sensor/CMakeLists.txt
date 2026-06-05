idf_component_register(
    SRCS "line_sensor_uart.c"
    INCLUDE_DIRS "include"
    REQUIRES board driver esp_timer
)
