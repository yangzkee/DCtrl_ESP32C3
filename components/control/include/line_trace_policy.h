#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "chassis_uart.h"
#include "control_plan.h"
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

typedef enum {
    LINE_TRACE_RECOVERY_NONE = 0,
    LINE_TRACE_RECOVERY_UNDERSTEER,
    LINE_TRACE_RECOVERY_OSCILLATION_SKEW,
    LINE_TRACE_RECOVERY_NO_REFERENCE,
} line_trace_recovery_relation_t;

typedef enum {
    LINE_TRACE_RECOVERY_STAGE_NONE = 0,
    LINE_TRACE_RECOVERY_STAGE_SWEEP,
    LINE_TRACE_RECOVERY_STAGE_FULL_ROTATION,
    LINE_TRACE_RECOVERY_STAGE_FAILED,
} line_trace_recovery_stage_t;

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
    bool recovery_active;
    int32_t recovery_angle_mdeg;
    int32_t recovery_target_mdeg;
    int32_t recovery_amplitude_mdeg;
    int32_t recovery_last_direction_mdeg;
    int32_t recovery_reference_turn_mdeg;
    line_trace_recovery_relation_t recovery_relation;
    line_trace_recovery_stage_t recovery_stage;
} line_trace_policy_runtime_t;

typedef struct {
    line_trace_run_mode_t run_mode;
    line_trace_sensor_status_t sensor_status;
    line_sensor_sample_t sample;
    chassis_motion_cmd_t manual_cmd;
} line_trace_policy_input_t;

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
} line_trace_policy_output_t;

void line_trace_policy_init(line_trace_policy_runtime_t *runtime);
void line_trace_policy_reset_pid(line_trace_policy_runtime_t *runtime);
void line_trace_policy_step(line_trace_policy_runtime_t *runtime,
                            const line_trace_policy_config_t *config,
                            const line_trace_policy_input_t *input,
                            line_trace_policy_output_t *output);
const char *line_trace_policy_phase_name(line_trace_phase_t phase);
const char *line_trace_recovery_relation_name(line_trace_recovery_relation_t relation);
const char *line_trace_recovery_stage_name(line_trace_recovery_stage_t stage);

#ifdef __cplusplus
}
#endif
