idf_component_register(
    SRCS "chassis_uart.c" "dflink_motion_codec.c"
    INCLUDE_DIRS "include"
    REQUIRES board driver
)
