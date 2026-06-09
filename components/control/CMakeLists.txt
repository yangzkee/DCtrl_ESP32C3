idf_component_register(
    SRCS "line_trace_controller.c"
         "line_trace_policy.c"
         "motion_contracts.c"
         "motion_inputs.c"
         "line_interpreter.c"
         "motion_policy.c"
         "motion_executor.c"
    INCLUDE_DIRS "include"
    REQUIRES board chassis line_sensor param_store telemetry vehicle_state freertos
)
