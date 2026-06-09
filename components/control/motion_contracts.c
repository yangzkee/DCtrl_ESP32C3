#include "motion_contracts.h"

int32_t motion_clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

float motion_clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

int32_t motion_abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

uint8_t motion_line_active_sensor_count(uint8_t bits)
{
    uint8_t count = 0;
    for (int i = 0; i < MOTION_LINE_SENSOR_COUNT; ++i) {
        if (bits & (1U << i)) {
            count++;
        }
    }
    return count;
}

uint8_t motion_line_quality_from_active_count(uint8_t count)
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

float motion_line_curved_error_from_offset(int8_t offset)
{
    const int32_t abs_offset = motion_clamp_i32(motion_abs_i32((int32_t)offset), 0, MOTION_LINE_MAX_OFFSET);
    if (abs_offset == 0) {
        return 0.0f;
    }

    const float normalized = (float)abs_offset / (float)MOTION_LINE_MAX_OFFSET;
    const float curved = normalized * normalized;
    return offset < 0 ? -curved : curved;
}

int32_t motion_line_adaptive_speed_mm_s(int32_t base_speed_mm_s, int8_t offset)
{
    const int32_t abs_offset = motion_clamp_i32(motion_abs_i32((int32_t)offset), 0, MOTION_LINE_MAX_OFFSET);

    if (abs_offset <= MOTION_LINE_CENTER_OFFSET || base_speed_mm_s <= 0) {
        return base_speed_mm_s;
    }

    const int32_t max_slowdown = (base_speed_mm_s * (100 - MOTION_LINE_EDGE_SPEED_PERCENT)) / 100;
    const int32_t range = MOTION_LINE_MAX_OFFSET - MOTION_LINE_CENTER_OFFSET;
    const int32_t severity = abs_offset - MOTION_LINE_CENTER_OFFSET;
    return base_speed_mm_s - ((max_slowdown * severity * severity) / (range * range));
}

void motion_speed_profile_from_gear(int32_t gear, int32_t *base_speed_mm_s, int32_t *max_turn_mdeg_s)
{
    if (base_speed_mm_s == 0 || max_turn_mdeg_s == 0) {
        return;
    }

    switch (gear) {
    case 1:
        *base_speed_mm_s = 250;
        *max_turn_mdeg_s = 8000;
        break;
    case 3:
        *base_speed_mm_s = 1000;
        *max_turn_mdeg_s = 10000;
        break;
    case 2:
    default:
        *base_speed_mm_s = 600;
        *max_turn_mdeg_s = 10000;
        break;
    }
}
