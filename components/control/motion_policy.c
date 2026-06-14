#include "motion_policy.h"

#include <string.h>

#include "motion_contracts.h"

static void reset_fault_grace(line_trace_policy_runtime_t *runtime)
{
    runtime->sensor_fault_count = 0;
    runtime->lost_line_count = 0;
}

static int32_t sign_i32(int32_t value)
{
    if (value > 0) {
        return 1;
    }
    if (value < 0) {
        return -1;
    }
    return 0;
}

static void publish_recovery_state(const line_trace_policy_runtime_t *runtime, motion_intent_t *intent)
{
    intent->recovery_relation = runtime->recovery_relation;
    intent->recovery_stage = runtime->recovery_stage;
    intent->recovery_angle_mdeg = runtime->recovery_angle_mdeg;
    intent->recovery_target_mdeg = runtime->recovery_target_mdeg;
    intent->recovery_direction_mdeg = runtime->recovery_last_direction_mdeg;
    intent->recovery_segment_index = runtime->recovery_segment_index;
    if (runtime->recovery_segment_active) {
        intent->recovery_elapsed_ms = (uint32_t)(intent->now_ms - runtime->recovery_segment_started_ms);
        intent->recovery_target_ms = runtime->recovery_segment_duration_ms;
    }
}

static void set_stop_intent(motion_intent_t *intent, line_trace_phase_t phase)
{
    intent->phase = phase;
    intent->plan.chassis_action = CONTROL_CHASSIS_ACTION_STOP;
    intent->plan.chassis_cmd = (chassis_motion_cmd_t){0};
    intent->pid_output_mdeg_s = 0.0f;
}

static void set_noop_intent(motion_intent_t *intent, line_trace_phase_t phase)
{
    intent->phase = phase;
    intent->plan.chassis_action = CONTROL_CHASSIS_ACTION_NONE;
    intent->plan.chassis_cmd = (chassis_motion_cmd_t){0};
    intent->pid_output_mdeg_s = 0.0f;
}

static void set_stop_to_idle_intent(motion_intent_t *intent, line_trace_phase_t phase)
{
    set_stop_intent(intent, phase);
    intent->plan.vehicle_action = CONTROL_VEHICLE_ACTION_STOP_TO_IDLE;
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

static void set_fixed_search_intent(line_trace_policy_runtime_t *runtime,
                                    motion_intent_t *intent,
                                    line_trace_phase_t phase)
{
    intent->phase = phase;
    intent->plan.chassis_action = CONTROL_CHASSIS_ACTION_SEND_MOTION;
    intent->plan.chassis_cmd.vx_mm_s = 0;
    intent->plan.chassis_cmd.yaw_mdeg = search_turn_mdeg(runtime);
    intent->lost_line = true;
    intent->pid_output_mdeg_s = 0.0f;
    runtime->last_cmd = intent->plan.chassis_cmd;
    runtime->has_last_cmd = true;
}

typedef struct {
    int32_t direction;
    uint32_t duration_ms;
    line_trace_recovery_stage_t stage;
} recovery_segment_t;

static const recovery_segment_t k_recovery_segments[] = {
    {-1, 800, LINE_TRACE_RECOVERY_STAGE_SWEEP},
    {1, 1600, LINE_TRACE_RECOVERY_STAGE_SWEEP},
    {-1, 3200, LINE_TRACE_RECOVERY_STAGE_SWEEP},
    {1, 6400, LINE_TRACE_RECOVERY_STAGE_SWEEP},
    {-1, 12800, LINE_TRACE_RECOVERY_STAGE_SWEEP},
    {1, 25600, LINE_TRACE_RECOVERY_STAGE_SWEEP},
    {1, 30000, LINE_TRACE_RECOVERY_STAGE_ONE_WAY_SEARCH},
};

static const size_t k_recovery_segment_count =
    sizeof(k_recovery_segments) / sizeof(k_recovery_segments[0]);

static void start_sweep_recovery_if_needed(line_trace_policy_runtime_t *runtime, uint32_t now_ms)
{
    if (runtime->recovery_active) {
        return;
    }

    runtime->recovery_active = true;
    runtime->recovery_stage = k_recovery_segments[0].stage;
    runtime->recovery_angle_mdeg = 0;
    runtime->recovery_amplitude_mdeg = 0;
    runtime->recovery_target_mdeg = 0;
    runtime->recovery_last_direction_mdeg = 0;
    runtime->recovery_reference_turn_mdeg =
        runtime->has_last_tracking_angular ? runtime->last_tracking_angular_mdeg_s : 0;
    runtime->recovery_segment_active = true;
    runtime->recovery_segment_index = 0;
    runtime->recovery_segment_started_ms = now_ms;
    runtime->recovery_segment_duration_ms = k_recovery_segments[0].duration_ms;
    runtime->recovery_relation = LINE_TRACE_RECOVERY_NONE;
}

static uint32_t recovery_segment_elapsed_ms(const line_trace_policy_runtime_t *runtime, uint32_t now_ms)
{
    return (uint32_t)(now_ms - runtime->recovery_segment_started_ms);
}

static void clear_recovery_segment(line_trace_policy_runtime_t *runtime)
{
    runtime->recovery_segment_active = false;
    runtime->recovery_segment_index = 0;
    runtime->recovery_segment_started_ms = 0;
    runtime->recovery_segment_duration_ms = 0;
}

static void cancel_recovery_segment(line_trace_policy_runtime_t *runtime)
{
    clear_recovery_segment(runtime);
    runtime->recovery_active = false;
    runtime->recovery_stage = LINE_TRACE_RECOVERY_STAGE_NONE;
}

static bool advance_recovery_segment_if_needed(line_trace_policy_runtime_t *runtime, uint32_t now_ms)
{
    while (runtime->recovery_segment_active &&
           runtime->recovery_segment_index < k_recovery_segment_count &&
           recovery_segment_elapsed_ms(runtime, now_ms) >= runtime->recovery_segment_duration_ms) {
        runtime->recovery_segment_started_ms += runtime->recovery_segment_duration_ms;
        runtime->recovery_segment_index++;
        if (runtime->recovery_segment_index >= k_recovery_segment_count) {
            runtime->recovery_stage = LINE_TRACE_RECOVERY_STAGE_FAILED;
            clear_recovery_segment(runtime);
            return false;
        }
        runtime->recovery_stage = k_recovery_segments[runtime->recovery_segment_index].stage;
        runtime->recovery_segment_duration_ms = k_recovery_segments[runtime->recovery_segment_index].duration_ms;
    }
    return runtime->recovery_stage != LINE_TRACE_RECOVERY_STAGE_FAILED;
}

static bool build_sweep_recovery_motion(line_trace_policy_runtime_t *runtime,
                                        const line_trace_policy_config_t *config,
                                        const line_trace_policy_input_t *input,
                                        chassis_motion_cmd_t *motion_cmd)
{
    start_sweep_recovery_if_needed(runtime, input->now_ms);
    if (!advance_recovery_segment_if_needed(runtime, input->now_ms)) {
        return false;
    }

    if (!runtime->recovery_segment_active ||
        runtime->recovery_segment_index >= k_recovery_segment_count) {
        runtime->recovery_stage = LINE_TRACE_RECOVERY_STAGE_FAILED;
        clear_recovery_segment(runtime);
        return false;
    }

    const int32_t angular_mdeg_s =
        k_recovery_segments[runtime->recovery_segment_index].direction * config->max_turn_mdeg_s;
    *motion_cmd = (chassis_motion_cmd_t){
        .vx_mm_s = 0,
        .yaw_mdeg = angular_mdeg_s,
    };
    runtime->recovery_last_direction_mdeg = angular_mdeg_s;
    runtime->last_cmd = *motion_cmd;
    runtime->has_last_cmd = true;
    return true;
}

static void set_sweep_search_intent(line_trace_policy_runtime_t *runtime,
                                    const line_trace_policy_config_t *config,
                                    const line_trace_policy_input_t *input,
                                    motion_intent_t *intent,
                                    line_trace_phase_t phase)
{
    intent->phase = phase;
    chassis_motion_cmd_t motion_cmd = {0};
    const bool send_motion = build_sweep_recovery_motion(runtime, config, input, &motion_cmd);
    if (runtime->recovery_stage == LINE_TRACE_RECOVERY_STAGE_FAILED) {
        set_stop_to_idle_intent(intent, phase);
        intent->lost_line = true;
        return;
    }
    intent->plan.chassis_action = send_motion ? CONTROL_CHASSIS_ACTION_SEND_MOTION : CONTROL_CHASSIS_ACTION_NONE;
    intent->plan.chassis_cmd = motion_cmd;
    intent->lost_line = true;
    intent->pid_output_mdeg_s = 0.0f;
}

static void note_recovery_result_if_needed(line_trace_policy_runtime_t *runtime)
{
    if (!runtime->recovery_active) {
        return;
    }

    const int32_t reference_sign = sign_i32(runtime->recovery_reference_turn_mdeg);
    const int32_t found_sign = sign_i32(runtime->recovery_last_direction_mdeg);

    if (reference_sign == 0 || found_sign == 0) {
        runtime->recovery_relation = LINE_TRACE_RECOVERY_NO_REFERENCE;
    } else if (reference_sign == found_sign) {
        runtime->recovery_relation = LINE_TRACE_RECOVERY_UNDERSTEER;
    } else {
        runtime->recovery_relation = LINE_TRACE_RECOVERY_OSCILLATION_SKEW;
    }

    runtime->recovery_active = false;
    runtime->recovery_stage = LINE_TRACE_RECOVERY_STAGE_NONE;
    clear_recovery_segment(runtime);
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
    intent->now_ms = input->now_ms;
    intent->plan.fault_reason = VEHICLE_FAULT_NONE;

    if (input->run_mode == LINE_TRACE_RUN_MANUAL_TEST) {
        line_trace_policy_reset_pid(runtime);
        reset_fault_grace(runtime);
        intent->phase = LINE_TRACE_PHASE_MANUAL_TEST;
        intent->plan.chassis_action = CONTROL_CHASSIS_ACTION_SEND_MOTION;
        intent->plan.chassis_cmd = input->manual_cmd;
        runtime->last_cmd = intent->plan.chassis_cmd;
        runtime->has_last_cmd = true;
        runtime->phase = intent->phase;
        publish_recovery_state(runtime, intent);
        return;
    }

    if (input->sensor_status != LINE_TRACE_SENSOR_OK) {
        runtime->sensor_fault_count++;
        runtime->lost_line_count = 0;
        if (input->run_mode == LINE_TRACE_RUN_AUTO_RUNNING) {
            if (runtime->recovery_active) {
                cancel_recovery_segment(runtime);
            }
            set_fixed_search_intent(runtime, intent, LINE_TRACE_PHASE_SENSOR_FAULT);
        } else {
            set_noop_intent(intent, LINE_TRACE_PHASE_SENSOR_FAULT);
        }
        runtime->phase = intent->phase;
        publish_recovery_state(runtime, intent);
        return;
    }

    intent->active_sensor_count = geometry->active_sensor_count;
    intent->line_quality = geometry->line_quality;
    intent->lost_line = geometry->lost_line;

    if (input->run_mode == LINE_TRACE_RUN_DISABLED) {
        line_trace_policy_reset_pid(runtime);
        reset_fault_grace(runtime);
        runtime->has_last_cmd = false;
        set_noop_intent(intent, LINE_TRACE_PHASE_STOPPED);
        runtime->phase = intent->phase;
        publish_recovery_state(runtime, intent);
        return;
    }

    if (input->run_mode == LINE_TRACE_RUN_AUTO_ARMED) {
        line_trace_policy_reset_pid(runtime);
        reset_fault_grace(runtime);
        runtime->has_last_cmd = false;
        set_noop_intent(intent, LINE_TRACE_PHASE_ACQUIRE_LINE);
        runtime->phase = intent->phase;
        publish_recovery_state(runtime, intent);
        return;
    }

    if (input->run_mode != LINE_TRACE_RUN_AUTO_RUNNING) {
        line_trace_policy_reset_pid(runtime);
        reset_fault_grace(runtime);
        runtime->has_last_cmd = false;
        set_noop_intent(intent, LINE_TRACE_PHASE_STOPPED);
        runtime->phase = intent->phase;
        publish_recovery_state(runtime, intent);
        return;
    }

    if (geometry->lost_line) {
        runtime->lost_line_count++;
        runtime->sensor_fault_count = 0;
        set_sweep_search_intent(runtime, config, input, intent, LINE_TRACE_PHASE_LINE_LOST);
        runtime->phase = intent->phase;
        publish_recovery_state(runtime, intent);
        return;
    }

    note_recovery_result_if_needed(runtime);
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
    intent->plan.chassis_action = CONTROL_CHASSIS_ACTION_SEND_MOTION;
    intent->plan.chassis_cmd.vx_mm_s = motion_line_adaptive_speed_mm_s(config->base_speed_mm_s, input->sample.offset);
    intent->plan.chassis_cmd.yaw_mdeg = motion_clamp_i32((int32_t)(-pid_output),
                                                               -config->max_turn_mdeg_s,
                                                               config->max_turn_mdeg_s);
    intent->pid_output_mdeg_s = pid_output;
    runtime->last_cmd = intent->plan.chassis_cmd;
    runtime->has_last_cmd = true;
    if (intent->plan.chassis_cmd.yaw_mdeg != 0) {
        runtime->last_tracking_angular_mdeg_s = intent->plan.chassis_cmd.yaw_mdeg;
        runtime->has_last_tracking_angular = true;
    }
    runtime->phase = intent->phase;
    publish_recovery_state(runtime, intent);
}
