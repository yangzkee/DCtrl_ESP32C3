#include "miniapp_motion_mapping.h"

#include <math.h>
#include <string.h>

#include "chassis_motion_contracts.h"

static int32_t read_s32(const uint8_t *p)
{
    const uint32_t raw = (uint32_t)p[0] | (uint32_t)p[1] << 8 |
                         (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
    return (int32_t)(raw <= INT32_MAX ? (int64_t)raw : (int64_t)raw - INT64_C(4294967296));
}

static void write_s32(uint8_t *p, int32_t value)
{
    const uint32_t raw = (uint32_t)value;
    for (int byte = 0; byte < 4; ++byte) p[byte] = (uint8_t)(raw >> (byte * 8));
}

bool miniapp_motion_map_frame(const uint8_t *input, size_t length,
                             uint8_t *output, size_t capacity)
{
    if (output == NULL || capacity < length || !chassis_dflink_frame_valid(input, length)) {
        return false;
    }
    const bool velocity = input[3] == DFLINK_A_MOTION && input[4] == DFLINK_B_MOTION_VELOCITY;
    if (velocity && length != DFLINK_MOTION_FRAME_BYTES) return false;
    memmove(output, input, length);
    if (!velocity) return true;

    // Work directly in the protocol's S32 x10000 domain; Wz stays untouched.
    const float vx = (float)read_s32(output + 6);
    const float vy = (float)read_s32(output + 10);
    const float speed = hypotf(vx, vy);
    if (speed == 0.0f) return true;

    float mapped_speed;
    if (speed <= 2500.0f) {
        mapped_speed = speed * (200.0f / 2500.0f);
    } else if (speed <= 10000.0f) {
        mapped_speed = 200.0f + (speed - 2500.0f) * (300.0f / 7500.0f);
    } else {
        const float bounded = fminf(speed, 30000.0f);
        mapped_speed = 500.0f + (bounded - 10000.0f) * (500.0f / 20000.0f);
    }
    const float factor = mapped_speed / speed;
    write_s32(output + 6, (int32_t)lroundf(vx * factor));
    write_s32(output + 10, (int32_t)lroundf(vy * factor));
    uint16_t sum = 0;
    for (size_t i = 0; i < length - 2; ++i) sum = (uint16_t)(sum + output[i]);
    output[length - 2] = (uint8_t)sum;
    output[length - 1] = (uint8_t)(sum >> 8);
    return true;
}
