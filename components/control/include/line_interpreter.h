#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "line_trace_policy.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LINE_PATTERN_UNKNOWN = 0,
    LINE_PATTERN_CENTERED,
    LINE_PATTERN_LEFT_OFFSET,
    LINE_PATTERN_RIGHT_OFFSET,
    LINE_PATTERN_LOST,
    LINE_PATTERN_SENSOR_FAULT,
} line_pattern_state_t;

typedef struct {
    uint8_t active_sensor_count;
    uint8_t line_quality;
    bool lost_line;
    float curved_error;
    line_pattern_state_t pattern;
} line_geometry_t;

void line_interpreter_evaluate(const line_trace_policy_input_t *input, line_geometry_t *geometry);
const char *line_pattern_state_name(line_pattern_state_t state);

#ifdef __cplusplus
}
#endif
