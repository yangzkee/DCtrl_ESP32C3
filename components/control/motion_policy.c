#include "motion_policy.h"

#include <string.h>

#include "motion_contracts.h"

static void reset_fault_grace(line_trace_policy_runtime_t *runtime)
{
    runtime->sensor_fault_count = 0;
    runtime->lost_line_count = 0;
}

static void set_stop_intent(motion_intent_t *intent, line_trace_phase_t phase)
{
    intent->phase = phase;
    intent->cmd = (chassis_motion_cmd_t){0};
    intent->should_send_motion = false;
    intent->should_stop = true;
    intent->pid_output_mdeg_s = 0.0f;
}

static int32_t search_turn_mdeg(const line_trace_policy_runtime_t *runtime)
{
    if (runtime != 0 &&
        runtime->has_last_tracking_angular &&
        runtime->last_tracking_angular_mdeg_s != 0) {
        return runtime->last_tracking_angular_mdeg_s > 0 ? -MOTION_LINE_SEARCH_TURN_MDEG : MOTION_LINE_SEARCH_TURN_MDEG;
    }
    if (runtime != 0 && runtime->has_previous_error && runtime->previous_error != 0.0f) {
        return runtime->previous_error < 0.0f ? -MOTION_LINE_SEARCH_TURN_MDEG : MOTION_LINE_SEARCH_TURN_MDEG;
    }
    return MOTION_LINE_SEARCH_TURN_MDEG;
}

static void set_search_intent(line_trace_policy_runtime_t *runtime,
                              motion_intent_t *intent,
                              line_trace_phase_t phase)
{
    intent->phase = phase;
    intent->cmd.linear_mm_s = 0;
    intent->cmd.angular_mdeg_s = search_turn_mdeg(runtime);
    intent->should_send_motion = true;
    intent->lost_line = true;
    intent->pid_output_mdeg_s = 0.0f;
    runtime->last_cmd = intent->cmd;
    runtime->has_last_cmd = true;
}

void motion_policy_plan(line_trace_policy_runtime_t *runtime,
                        const line_trace_policy_config_t *config,
                        const line_trace_policy_input_t *input,
                        const line_geometry_t *geometry,
                        motion_intent_t *intent)
{
    if (runtime == 0 || config == 0 || input == 0 || geometry == 0 || intent == 0) {
        return;
    }

    memset(intent, 0, sizeof(*intent));
    intent->fault_reason = VEHICLE_FAULT_NONE;

    if (input->run_mode == LINE_TRACE_RUN_MANUAL_TEST) {
        line_trace_policy_reset_pid(runtime);
        reset_fault_grace(runtime);
        intent->phase = LINE_TRACE_PHASE_MANUAL_TEST;
        intent->cmd = input->manual_cmd;
        intent->should_send_motion = true;
        runtime->last_cmd = intent->cmd;
        runtime->has_last_cmd = true;
        runtime->phase = intent->phase;
        return;
    }

    if (input->sensor_status != LINE_TRACE_SENSOR_OK) {
        runtime->sensor_fault_count++;
        runtime->lost_line_count = 0;
        if (input->run_mode == LINE_TRACE_RUN_AUTO_RUNNING) {
            set_search_intent(runtime, intent, LINE_TRACE_PHASE_SENSOR_FAULT);
        } else {
            set_stop_intent(intent, LINE_TRACE_PHASE_SENSOR_FAULT);
        }
        runtime->phase = intent->phase;
        return;
    }

    intent->active_sensor_count = geometry->active_sensor_count;
    intent->line_quality = geometry->line_quality;
    intent->lost_line = geometry->lost_line;

    if (input->run_mode == LINE_TRACE_RUN_DISABLED) {
        line_trace_policy_reset_pid(runtime);
        reset_fault_grace(runtime);
        runtime->has_last_cmd = false;
        set_stop_intent(intent, LINE_TRACE_PHASE_STOPPED);
        runtime->phase = intent->phase;
        return;
    }

    if (input->run_mode == LINE_TRACE_RUN_AUTO_ARMED) {
        line_trace_policy_reset_pid(runtime);
        reset_fault_grace(runtime);
        runtime->has_last_cmd = false;
        set_stop_intent(intent, LINE_TRACE_PHASE_ACQUIRE_LINE);
        runtime->phase = intent->phase;
        return;
    }

    if (input->run_mode != LINE_TRACE_RUN_AUTO_RUNNING) {
        line_trace_policy_reset_pid(runtime);
        reset_fault_grace(runtime);
        runtime->has_last_cmd = false;
        set_stop_intent(intent, LINE_TRACE_PHASE_STOPPED);
        runtime->phase = intent->phase;
        return;
    }

    if (geometry->lost_line) {
        runtime->lost_line_count++;
        runtime->sensor_fault_count = 0;
        set_search_intent(runtime, intent, LINE_TRACE_PHASE_LINE_LOST);
        runtime->phase = intent->phase;
        return;
    }

    reset_fault_grace(runtime);
    const float error = geometry->curved_error;
    const float derivative = runtime->has_previous_error ? error - runtime->previous_error : 0.0f;
    runtime->integral = motion_clamp_float(runtime->integral + error,
                                           -config->integral_limit,
                                           config->integral_limit);
    runtime->previous_error = error;
    runtime->has_previous_error = true;

    const float pid_output = (config->kp * error) +
                             (config->ki * runtime->integral) +
                             (config->kd * derivative);
    intent->phase = LINE_TRACE_PHASE_TRACK_LINE;
    intent->cmd.linear_mm_s = motion_line_adaptive_speed_mm_s(config->base_speed_mm_s, input->sample.offset);
    intent->cmd.angular_mdeg_s = motion_clamp_i32((int32_t)(-pid_output),
                                                  -config->max_turn_mdeg_s,
                                                  config->max_turn_mdeg_s);
    intent->should_send_motion = true;
    intent->pid_output_mdeg_s = pid_output;
    runtime->last_cmd = intent->cmd;
    runtime->has_last_cmd = true;
    if (intent->cmd.angular_mdeg_s != 0) {
        runtime->last_tracking_angular_mdeg_s = intent->cmd.angular_mdeg_s;
        runtime->has_last_tracking_angular = true;
    }
    runtime->phase = intent->phase;
}
