#include "chassis_motion_contracts.h"

#include <string.h>

static void write_fixed(uint8_t *out, float value)
{
    const float scaled = value * 10000.0f;
    const int32_t fixed = (int32_t)(scaled >= 0 ? scaled + 0.5f : scaled - 0.5f);
    const uint32_t bits = (uint32_t)fixed;
    for (int i = 0; i < 4; ++i) {
        out[i] = (uint8_t)(bits >> (8 * i));
    }
}

size_t chassis_motion_encode(const chassis_motion_cmd_t *cmd, uint8_t *frame, size_t capacity)
{
    if (cmd == NULL || frame == NULL || capacity < DFLINK_MOTION_FRAME_BYTES) {
        return 0;
    }
    const uint8_t header[] = {DFLINK_FRAME_HEAD, DFLINK_TARGET_ID, DFLINK_SOURCE_ID,
                              DFLINK_A_MOTION, DFLINK_B_MOTION_VELOCITY,
                              DFLINK_MOTION_VELOCITY_PAYLOAD_BYTES};
    memcpy(frame, header, sizeof(header));
    write_fixed(frame + 6, (float)cmd->vx_mm_s / 1000.0f);
    write_fixed(frame + 10, (float)cmd->vy_mm_s / 1000.0f);
    // mdeg/s -> rad/s. Sending frequency never scales this velocity.
    write_fixed(frame + 14, ((float)cmd->angular_mdeg_s / 1000.0f) * (3.14159265358979323846f / 180.0f));
    frame[18] = DFLINK_FRAME_TAIL;
    uint16_t sum = 0;
    for (int i = 0; i < 19; ++i) {
        sum = (uint16_t)(sum + frame[i]);
    }
    frame[19] = (uint8_t)sum;
    frame[20] = (uint8_t)(sum >> 8);
    return DFLINK_MOTION_FRAME_BYTES;
}
