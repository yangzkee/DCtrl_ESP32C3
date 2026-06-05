#include "line_trace_policy.h"

#include <stddef.h>
#include <string.h>

#define LINE_TRACE_SEARCH_TURN_MDEG 6000
#define LINE_TRACE_CENTER_OFFSET 1
#define LINE_TRACE_MAX_OFFSET 7
#define LINE_TRACE_EDGE_SPEED_PERCENT 45

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int32_t abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint8_t active_sensor_count(uint8_t bits)
{
    uint8_t count = 0;
    for (int i = 0; i < 8; ++i) {
        if (bits & (1U << i)) {
            count++;
        }
    }
    return count;
}

static uint8_t line_quality_from_count(uint8_t count)
{
    if (count == 0) {
        return 0;
    }
    if (count <= 3) {
        return 100;
    }
    if (count <= 5) {
        return 60;
    }
    return 30;
}

static void set_stop_output(line_trace_policy_output_t *output, line_trace_phase_t phase)
{
    output->phase = phase;
    output->cmd = (chassis_motion_cmd_t){0};
    output->should_send_motion = false;
    output->should_stop = true;
    output->pid_output_mdeg_s = 0.0f;
}

static int32_t search_turn_mdeg(const line_trace_policy_runtime_t *runtime)
{
    if (runtime != NULL &&
        runtime->has_last_tracking_angular &&
        runtime->last_tracking_angular_mdeg_s != 0) {
        return runtime->last_tracking_angular_mdeg_s > 0 ? -LINE_TRACE_SEARCH_TURN_MDEG : LINE_TRACE_SEARCH_TURN_MDEG;
    }
    if (runtime != NULL && runtime->has_previous_error && runtime->previous_error != 0.0f) {
        return runtime->previous_error < 0.0f ? -LINE_TRACE_SEARCH_TURN_MDEG : LINE_TRACE_SEARCH_TURN_MDEG;
    }
    return LINE_TRACE_SEARCH_TURN_MDEG;
}

static int32_t adaptive_linear_speed_mm_s(const line_trace_policy_config_t *config, int8_t offset)
{
    const int32_t base_speed = config->base_speed_mm_s;
    const int32_t abs_offset = clamp_i32(abs_i32((int32_t)offset), 0, LINE_TRACE_MAX_OFFSET);

    if (abs_offset <= LINE_TRACE_CENTER_OFFSET || base_speed <= 0) {
        return base_speed;
    }

    const int32_t max_slowdown = (base_speed * (100 - LINE_TRACE_EDGE_SPEED_PERCENT)) / 100;
    const int32_t range = LINE_TRACE_MAX_OFFSET - LINE_TRACE_CENTER_OFFSET;
    const int32_t severity = abs_offset - LINE_TRACE_CENTER_OFFSET;
    return base_speed - ((max_slowdown * severity * severity) / (range * range));
}

static float curved_error_from_offset(int8_t offset)
{
    const int32_t abs_offset = clamp_i32(abs_i32((int32_t)offset), 0, LINE_TRACE_MAX_OFFSET);
    if (abs_offset == 0) {
        return 0.0f;
    }

    const float normalized = (float)abs_offset / (float)LINE_TRACE_MAX_OFFSET;
    const float curved = normalized * normalized;
    return offset < 0 ? -curved : curved;
}

static void set_search_output(line_trace_policy_runtime_t *runtime,
                              line_trace_policy_output_t *output,
                              line_trace_phase_t phase)
{
    output->phase = phase;
    output->cmd.linear_mm_s = 0;
    output->cmd.angular_mdeg_s = search_turn_mdeg(runtime);
    output->should_send_motion = true;
    output->lost_line = true;
    output->pid_output_mdeg_s = 0.0f;
    runtime->last_cmd = output->cmd;
    runtime->has_last_cmd = true;
}

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
}

static void reset_fault_grace(line_trace_policy_runtime_t *runtime)
{
    runtime->sensor_fault_count = 0;
    runtime->lost_line_count = 0;
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
    output->fault_reason = VEHICLE_FAULT_NONE;

    if (input->run_mode == LINE_TRACE_RUN_MANUAL_TEST) {
        line_trace_policy_reset_pid(runtime);
        reset_fault_grace(runtime);
        output->phase = LINE_TRACE_PHASE_MANUAL_TEST;
        output->cmd = input->manual_cmd;
        output->should_send_motion = true;
        runtime->last_cmd = output->cmd;
        runtime->has_last_cmd = true;
        runtime->phase = output->phase;
        return;
    }

    if (input->sensor_status != LINE_TRACE_SENSOR_OK) {
        runtime->sensor_fault_count++;
        runtime->lost_line_count = 0;
        if (input->run_mode == LINE_TRACE_RUN_AUTO_RUNNING) {
            set_search_output(runtime, output, LINE_TRACE_PHASE_SENSOR_FAULT);
        } else {
            set_stop_output(output, LINE_TRACE_PHASE_SENSOR_FAULT);
        }
        runtime->phase = output->phase;
        return;
    }

    output->active_sensor_count = active_sensor_count(input->sample.bits);
    output->line_quality = line_quality_from_count(output->active_sensor_count);
    output->lost_line = input->sample.bits == 0;

    if (input->run_mode == LINE_TRACE_RUN_DISABLED) {
        line_trace_policy_reset_pid(runtime);
        reset_fault_grace(runtime);
        runtime->has_last_cmd = false;
        set_stop_output(output, LINE_TRACE_PHASE_STOPPED);
        runtime->phase = output->phase;
        return;
    }

    if (input->run_mode == LINE_TRACE_RUN_AUTO_ARMED) {
        line_trace_policy_reset_pid(runtime);
        reset_fault_grace(runtime);
        runtime->has_last_cmd = false;
        set_stop_output(output, LINE_TRACE_PHASE_ACQUIRE_LINE);
        runtime->phase = output->phase;
        return;
    }

    if (input->run_mode != LINE_TRACE_RUN_AUTO_RUNNING) {
        line_trace_policy_reset_pid(runtime);
        reset_fault_grace(runtime);
        runtime->has_last_cmd = false;
        set_stop_output(output, LINE_TRACE_PHASE_STOPPED);
        runtime->phase = output->phase;
        return;
    }

    if (input->sample.bits == 0) {
        runtime->lost_line_count++;
        runtime->sensor_fault_count = 0;
        set_search_output(runtime, output, LINE_TRACE_PHASE_LINE_LOST);
        runtime->phase = output->phase;
        return;
    }

    reset_fault_grace(runtime);
    const float error = curved_error_from_offset(input->sample.offset);
    const float derivative = runtime->has_previous_error ? error - runtime->previous_error : 0.0f;
    runtime->integral = clamp_float(runtime->integral + error,
                                    -config->integral_limit,
                                    config->integral_limit);
    runtime->previous_error = error;
    runtime->has_previous_error = true;

    const float pid_output = (config->kp * error) +
                             (config->ki * runtime->integral) +
                             (config->kd * derivative);
    output->phase = LINE_TRACE_PHASE_TRACK_LINE;
    output->cmd.linear_mm_s = adaptive_linear_speed_mm_s(config, input->sample.offset);
    output->cmd.angular_mdeg_s = clamp_i32((int32_t)(-pid_output),
                                           -config->max_turn_mdeg_s,
                                           config->max_turn_mdeg_s);
    output->should_send_motion = true;
    output->pid_output_mdeg_s = pid_output;
    runtime->last_cmd = output->cmd;
    runtime->has_last_cmd = true;
    if (output->cmd.angular_mdeg_s != 0) {
        runtime->last_tracking_angular_mdeg_s = output->cmd.angular_mdeg_s;
        runtime->has_last_tracking_angular = true;
    }
    runtime->phase = output->phase;
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
