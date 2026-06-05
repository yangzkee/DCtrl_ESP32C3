#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "chassis_uart.h"
#include "line_sensor_uart.h"
#include "vehicle_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LINE_TRACE_RUN_DISABLED = 0,
    LINE_TRACE_RUN_MANUAL_TEST,
    LINE_TRACE_RUN_AUTO_ARMED,
    LINE_TRACE_RUN_AUTO_RUNNING,
} line_trace_run_mode_t;

typedef enum {
    LINE_TRACE_SENSOR_OK = 0,
    LINE_TRACE_SENSOR_TIMEOUT,
    LINE_TRACE_SENSOR_ERROR,
} line_trace_sensor_status_t;

typedef enum {
    LINE_TRACE_PHASE_STOPPED = 0,
    LINE_TRACE_PHASE_MANUAL_TEST,
    LINE_TRACE_PHASE_ACQUIRE_LINE,
    LINE_TRACE_PHASE_TRACK_LINE,
    LINE_TRACE_PHASE_LINE_LOST,
    LINE_TRACE_PHASE_SENSOR_FAULT,
} line_trace_phase_t;

typedef struct {
    int32_t base_speed_mm_s;
    int32_t max_turn_mdeg_s;
    float kp;
    float ki;
    float kd;
    float integral_limit;
} line_trace_policy_config_t;

typedef struct {
    line_trace_phase_t phase;
    float integral;
    float previous_error;
    bool has_previous_error;
    uint8_t sensor_fault_count;
    uint8_t lost_line_count;
    chassis_motion_cmd_t last_cmd;
    bool has_last_cmd;
    int32_t last_tracking_angular_mdeg_s;
    bool has_last_tracking_angular;
} line_trace_policy_runtime_t;

typedef struct {
    line_trace_run_mode_t run_mode;
    line_trace_sensor_status_t sensor_status;
    line_sensor_sample_t sample;
    chassis_motion_cmd_t manual_cmd;
} line_trace_policy_input_t;

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
} line_trace_policy_output_t;

void line_trace_policy_init(line_trace_policy_runtime_t *runtime);
void line_trace_policy_reset_pid(line_trace_policy_runtime_t *runtime);
void line_trace_policy_step(line_trace_policy_runtime_t *runtime,
                            const line_trace_policy_config_t *config,
                            const line_trace_policy_input_t *input,
                            line_trace_policy_output_t *output);
const char *line_trace_policy_phase_name(line_trace_phase_t phase);

#ifdef __cplusplus
}
#endif
