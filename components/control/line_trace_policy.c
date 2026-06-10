#include "line_trace_policy.h"

#include <string.h>

#include "line_interpreter.h"
#include "motion_executor.h"
#include "motion_policy.h"

void line_trace_policy_init(line_trace_policy_runtime_t *runtime)
{
    if (runtime == NULL) {
        return;
    }
    memset(runtime, 0, sizeof(*runtime));
    runtime->phase = LINE_TRACE_PHASE_STOPPED;
}

void line_trace_policy_reset_pid(line_trace_policy_runtime_t *runtime)
{
    if (runtime == NULL) {
        return;
    }
    runtime->integral = 0.0f;
    runtime->previous_error = 0.0f;
    runtime->has_previous_error = false;
    runtime->last_tracking_angular_mdeg_s = 0;
    runtime->has_last_tracking_angular = false;
    runtime->recovery_active = false;
    runtime->recovery_angle_mdeg = 0;
    runtime->recovery_target_mdeg = 0;
    runtime->recovery_amplitude_mdeg = 0;
    runtime->recovery_last_direction_mdeg = 0;
    runtime->recovery_reference_turn_mdeg = 0;
    runtime->recovery_segment_active = false;
    runtime->recovery_segment_index = 0;
    runtime->recovery_segment_started_ms = 0;
    runtime->recovery_segment_duration_ms = 0;
    runtime->recovery_relation = LINE_TRACE_RECOVERY_NONE;
    runtime->recovery_stage = LINE_TRACE_RECOVERY_STAGE_NONE;
}

void line_trace_policy_step(line_trace_policy_runtime_t *runtime,
                            const line_trace_policy_config_t *config,
                            const line_trace_policy_input_t *input,
                            line_trace_policy_output_t *output)
{
    if (runtime == NULL || config == NULL || input == NULL || output == NULL) {
        return;
    }

    memset(output, 0, sizeof(*output));

    line_geometry_t geometry = {0};
    line_interpreter_evaluate(input, &geometry);

    motion_intent_t intent = {0};
    motion_policy_plan(runtime, config, input, &geometry, &intent);
    motion_executor_build_output(&intent, output);
}

const char *line_trace_policy_phase_name(line_trace_phase_t phase)
{
    switch (phase) {
    case LINE_TRACE_PHASE_STOPPED:
        return "STOPPED";
    case LINE_TRACE_PHASE_MANUAL_TEST:
        return "MANUAL_TEST";
    case LINE_TRACE_PHASE_ACQUIRE_LINE:
        return "ACQUIRE_LINE";
    case LINE_TRACE_PHASE_TRACK_LINE:
        return "TRACK_LINE";
    case LINE_TRACE_PHASE_LINE_LOST:
        return "LINE_LOST";
    case LINE_TRACE_PHASE_SENSOR_FAULT:
        return "SENSOR_FAULT";
    default:
        return "UNKNOWN";
    }
}

const char *line_trace_recovery_relation_name(line_trace_recovery_relation_t relation)
{
    switch (relation) {
    case LINE_TRACE_RECOVERY_NONE:
        return "NONE";
    case LINE_TRACE_RECOVERY_UNDERSTEER:
        return "UNDERSTEER";
    case LINE_TRACE_RECOVERY_OSCILLATION_SKEW:
        return "OSCILLATION_SKEW";
    case LINE_TRACE_RECOVERY_NO_REFERENCE:
        return "NO_REFERENCE";
    default:
        return "UNKNOWN";
    }
}

const char *line_trace_recovery_stage_name(line_trace_recovery_stage_t stage)
{
    switch (stage) {
    case LINE_TRACE_RECOVERY_STAGE_NONE:
        return "NONE";
    case LINE_TRACE_RECOVERY_STAGE_SWEEP:
        return "SWEEP";
    case LINE_TRACE_RECOVERY_STAGE_ONE_WAY_SEARCH:
        return "ONE_WAY_SEARCH";
    case LINE_TRACE_RECOVERY_STAGE_FAILED:
        return "FAILED";
    default:
        return "UNKNOWN";
    }
}
