#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "line_interpreter.h"
#include "line_trace_policy.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    line_trace_phase_t phase;
    chassis_motion_cmd_t cmd;
    bool should_send_motion;
    bool should_stop;
    bool enter_fault;
    vehicle_fault_reason_t fault_reason;
    bool lost_line;
    uint8_t line_quality;
    uint8_t active_sensor_count;
    float pid_output_mdeg_s;
} motion_intent_t;

void motion_policy_plan(line_trace_policy_runtime_t *runtime,
                        const line_trace_policy_config_t *config,
                        const line_trace_policy_input_t *input,
                        const line_geometry_t *geometry,
                        motion_intent_t *intent);

#ifdef __cplusplus
}
#endif
