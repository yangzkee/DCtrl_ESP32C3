#pragma once

#include <stdbool.h>

#include "line_trace_policy.h"
#include "vehicle_state.h"

#ifdef __cplusplus
extern "C" {
#endif

line_trace_run_mode_t motion_inputs_run_mode_from_vehicle(const vehicle_state_snapshot_t *vehicle);
void motion_inputs_from_vehicle(line_trace_policy_input_t *input, const vehicle_state_snapshot_t *vehicle);
void motion_inputs_apply_line_inversion(line_sensor_sample_t *sample, bool invert_bits);

#ifdef __cplusplus
}
#endif
