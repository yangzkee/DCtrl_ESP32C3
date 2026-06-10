#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control_plan.h"
#include "line_interpreter.h"
#include "line_trace_policy.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    line_trace_phase_t phase;
    control_plan_t plan;
    bool lost_line;
    uint8_t line_quality;
    uint8_t active_sensor_count;
    float pid_output_mdeg_s;
    line_trace_recovery_relation_t recovery_relation;
    line_trace_recovery_stage_t recovery_stage;
    int32_t recovery_angle_mdeg;
    int32_t recovery_target_mdeg;
    int32_t recovery_direction_mdeg;
} motion_intent_t;

void motion_policy_plan(line_trace_policy_runtime_t *runtime,
                        const line_trace_policy_config_t *config,
                        const line_trace_policy_input_t *input,
                        const line_geometry_t *geometry,
                        motion_intent_t *intent);

#ifdef __cplusplus
}
#endif
