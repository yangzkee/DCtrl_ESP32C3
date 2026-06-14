idf_component_register(
    SRCS "debug_protocol.c"
    INCLUDE_DIRS "include"
    REQUIRES json param_store telemetry vehicle_state chassis control
)
