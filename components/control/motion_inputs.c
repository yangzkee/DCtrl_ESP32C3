#include "motion_inputs.h"

#include <stdint.h>

#include "line_sensor_uart.h"

line_trace_run_mode_t motion_inputs_run_mode_from_vehicle(const vehicle_state_snapshot_t *vehicle)
{
    if (vehicle == 0) {
        return LINE_TRACE_RUN_DISABLED;
    }

    switch (vehicle->motion_state) {
    case VEHICLE_MOTION_MANUAL_TEST:
        return LINE_TRACE_RUN_MANUAL_TEST;
    case VEHICLE_MOTION_AUTO_ARMED:
        return LINE_TRACE_RUN_AUTO_ARMED;
    case VEHICLE_MOTION_AUTO_RUNNING:
        return LINE_TRACE_RUN_AUTO_RUNNING;
    default:
        return LINE_TRACE_RUN_DISABLED;
    }
}

void motion_inputs_from_vehicle(line_trace_policy_input_t *input, const vehicle_state_snapshot_t *vehicle)
{
    if (input == 0) {
        return;
    }

    *input = (line_trace_policy_input_t){
        .run_mode = motion_inputs_run_mode_from_vehicle(vehicle),
        .sensor_status = LINE_TRACE_SENSOR_OK,
    };

    if (vehicle != 0) {
        input->manual_cmd = vehicle->manual_cmd;
    }
}

void motion_inputs_apply_line_inversion(line_sensor_sample_t *sample, bool invert_bits)
{
    if (sample == 0 || !invert_bits) {
        return;
    }

    sample->bits = (uint8_t)~sample->bits;
    sample->offset = line_sensor_calculate_offset(sample->bits);
}
