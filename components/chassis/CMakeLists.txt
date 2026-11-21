idf_component_register(
    SRCS "chassis_uart.c" "dflink_motion_codec.c" "miniapp_motion_mapping.c"
    INCLUDE_DIRS "include"
    REQUIRES board driver
)
