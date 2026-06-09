#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTION_LINE_SENSOR_COUNT 8
#define MOTION_LINE_CENTER_OFFSET 1
#define MOTION_LINE_MAX_OFFSET 7
#define MOTION_LINE_EDGE_SPEED_PERCENT 45
#define MOTION_LINE_SEARCH_TURN_MDEG 6000

int32_t motion_clamp_i32(int32_t value, int32_t min_value, int32_t max_value);
float motion_clamp_float(float value, float min_value, float max_value);
int32_t motion_abs_i32(int32_t value);

uint8_t motion_line_active_sensor_count(uint8_t bits);
uint8_t motion_line_quality_from_active_count(uint8_t count);
float motion_line_curved_error_from_offset(int8_t offset);
int32_t motion_line_adaptive_speed_mm_s(int32_t base_speed_mm_s, int8_t offset);
void motion_speed_profile_from_gear(int32_t gear, int32_t *base_speed_mm_s, int32_t *max_turn_mdeg_s);

#ifdef __cplusplus
}
#endif
