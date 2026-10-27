idf_component_register(
    SRCS "line_sensor_uart.c" "line_sensor_protocol.c"
    INCLUDE_DIRS "include"
    REQUIRES board driver esp_timer
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE DCTRL_TRACK_WHITE=${DCTRL_TRACK_WHITE})
