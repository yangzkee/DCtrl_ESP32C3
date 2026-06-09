#include "line_interpreter.h"

#include <string.h>

#include "motion_contracts.h"

void line_interpreter_evaluate(const line_trace_policy_input_t *input, line_geometry_t *geometry)
{
    if (input == 0 || geometry == 0) {
        return;
    }

    memset(geometry, 0, sizeof(*geometry));

    if (input->sensor_status != LINE_TRACE_SENSOR_OK) {
        geometry->pattern = LINE_PATTERN_SENSOR_FAULT;
        return;
    }

    geometry->active_sensor_count = motion_line_active_sensor_count(input->sample.bits);
    geometry->line_quality = motion_line_quality_from_active_count(geometry->active_sensor_count);
    geometry->lost_line = input->sample.bits == 0;
    geometry->curved_error = motion_line_curved_error_from_offset(input->sample.offset);

    if (geometry->lost_line) {
        geometry->pattern = LINE_PATTERN_LOST;
    } else if (input->sample.offset < -MOTION_LINE_CENTER_OFFSET) {
        geometry->pattern = LINE_PATTERN_LEFT_OFFSET;
    } else if (input->sample.offset > MOTION_LINE_CENTER_OFFSET) {
        geometry->pattern = LINE_PATTERN_RIGHT_OFFSET;
    } else {
        geometry->pattern = LINE_PATTERN_CENTERED;
    }
}

const char *line_pattern_state_name(line_pattern_state_t state)
{
    switch (state) {
    case LINE_PATTERN_CENTERED:
        return "CENTERED";
    case LINE_PATTERN_LEFT_OFFSET:
        return "LEFT_OFFSET";
    case LINE_PATTERN_RIGHT_OFFSET:
        return "RIGHT_OFFSET";
    case LINE_PATTERN_LOST:
        return "LOST";
    case LINE_PATTERN_SENSOR_FAULT:
        return "SENSOR_FAULT";
    case LINE_PATTERN_UNKNOWN:
    default:
        return "UNKNOWN";
    }
}
